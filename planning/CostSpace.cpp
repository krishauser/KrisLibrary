#include "CostSpace.h"
#include "KinodynamicPath.h"
#include "InterpolatorHelpers.h"
#include "EdgePlannerHelpers.h"
#include "TimeCSpace.h"
using namespace std;

StateCostControlSpace::StateCostControlSpace(const SmartPointer<ControlSpace>& _base,const SmartPointer<ObjectiveFunctionalBase>& _objective)
:base(_base),objective(_objective)
{}



std::string StateCostControlSpace::VariableName(int i)
{
  return base->VariableName(i);
}

SmartPointer<CSet> StateCostControlSpace::GetControlSet(const State& x)
{
  Vector q; Real c; SplitRef(x,q,c);
  return base->GetControlSet(q);
}

class StateCostSteeringFunction : public SteeringFunction
{
public:
  SmartPointer<SteeringFunction> base;
  StateCostSteeringFunction(const SmartPointer<SteeringFunction>& _base)
  :base(_base)
  {}
  virtual bool IsExact() const { return false; }
  virtual bool Connect(const State& x0,const State& x1,KinodynamicMilestonePath& path) {
    Real c0,c1;
    State q0,q1;
    StateCostControlSpace::SplitRef(x0,q0,c0);
    StateCostControlSpace::SplitRef(x1,q1,c1);
    KinodynamicMilestonePath qpath;
    if(!base->Connect(q0,q1,qpath)) return false;
    path.milestones.resize(qpath.milestones.size());
    path.controls.resize(qpath.controls.size());
    path.paths.resize(qpath.paths.size());
    Real dt = 1.0 / Real(qpath.paths.size());
    for(size_t i=0;i<qpath.milestones.size();i++) {
      Real u = Real(i)*dt;
      Real c = c0 + u*(c1-c0);
      StateCostControlSpace::Join(qpath.milestones[i],c,path.milestones[i]);
      if(i+1 < qpath.milestones.size()) {
        path.controls[i] = qpath.controls[i];
        Real u2 = Real(i+1)*dt;
        Real c2 = c0 + u2*(c1-c0);
        path.paths[i] = new MultiInterpolator(qpath.paths[i],new LinearInterpolator(c,c2));
      }
    }
    path.edges.resize(0);
    return true; 
  }
};

SmartPointer<SteeringFunction> StateCostControlSpace::GetSteeringFunction()
{
  SmartPointer<SteeringFunction> bsf = base->GetSteeringFunction();
  if(bsf) return new StateCostSteeringFunction(bsf);
  return NULL;
}

Interpolator* StateCostControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  Vector q; Real c; SplitRef(x0,q,c);
  Interpolator* qpath = base->Simulate(q,u);
  Real dc = objective->IncrementalCost(u,qpath);
  return new MultiInterpolator(qpath,new LinearInterpolator(c,c+dc));
}

void StateCostControlSpace::Successor(const State& x0, const ControlInput& u,State& x1)
{
  Vector q; Real c; SplitRef(x0,q,c);
  Interpolator* qpath = base->Simulate(q,u);
  c += objective->IncrementalCost(u,qpath);
  Join(qpath->End(),c,x1);
  delete qpath;
}

class CostCSpace : public TimeCSpace
{
public:
  CostCSpace(Real cmax) : TimeCSpace(cmax) {}
  virtual std::string VariableName(int i) { return "cost"; }
  virtual Real Distance(const Config& a,const Config& b) {
    if(b(0) > a(0)) return 0;
    return a(0) - b(0);
  }
};

StateCostKinodynamicSpace::StateCostKinodynamicSpace(const SmartPointer<KinodynamicSpace>& _base,const SmartPointer<ObjectiveFunctionalBase>& _objective,Real costMax)
:KinodynamicSpace(NULL,new StateCostControlSpace(_base->GetControlSpace(),_objective)),base(_base),scspace(new MultiCSpace),objective(_objective)
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*scspace);
  mspace->Add("",_base->GetStateSpace());
  mspace->Add("cost",new CostCSpace(costMax));
  this->stateSpace = scspace;
}

EdgePlanner* StateCostKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path)
{
  const MultiInterpolator* mi = dynamic_cast<const MultiInterpolator*>(&*path);
  if(!mi) return KinodynamicSpace::TrajectoryChecker(u,path);
  EdgePlanner* ebase = base->TrajectoryChecker(u,mi->components[0]);
  return new PiggybackEdgePlanner(GetStateSpace(),path,ebase);
}

void StateCostKinodynamicSpace::SetCostMax(Real costmax) 
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*scspace);
  TimeCSpace* costspace = dynamic_cast<TimeCSpace*>(&*mspace->components[1]);
  costspace->SetDomain(Vector(1,0.0),Vector(1,costmax));
  //this sets the sampling range
  if(!IsInf(costmax)) costspace->timeStepMax = costmax;
  else costspace->timeStepMax = 0;
}

void StateCostKinodynamicSpace::SetCostDistanceWeight(Real weight)
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*scspace);
  mspace->distanceWeights.resize(2,1.0);
  mspace->distanceWeights[1] = weight;
}

Real StateCostKinodynamicSpace::GetCostMax()
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*scspace);
  TimeCSpace* costspace = dynamic_cast<TimeCSpace*>(&*mspace->components[1]);
  Vector bmin,bmax;
  costspace->GetDomain(bmin,bmax);
  return bmax[0];
}

Real StateCostKinodynamicSpace::GetCostDistanceWeight()
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*scspace);
  if(mspace->distanceWeights.empty()) return 1;
  return mspace->distanceWeights[1];
}


void StateCostControlSpace::Split(const State& xc,State& x,Real& cost)
{
  Assert(!xc.empty());
  x.resize(xc.n-1);
  xc.getSubVectorCopy(0,x);
  cost = xc[xc.n-1];
}

void StateCostControlSpace::SplitRef(const State& xc,State& x,Real& cost)
{
  Assert(!xc.empty());
  x.setRef(xc,0,1,xc.n-1);
  cost = xc[xc.n-1];
}

void StateCostControlSpace::Join(const State& x,Real cost,State& xc)
{
  xc.resize(x.n+1);
  xc.copySubVector(0,x);
  xc[xc.n-1] = cost;
}

Real StateCostControlSpace::Cost(const State& xc)
{
  Assert(!xc.empty());
  return xc[xc.n-1];
}
