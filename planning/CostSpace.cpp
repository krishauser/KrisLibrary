#include "CostSpace.h"
#include "KinodynamicPath.h"
#include "InterpolatorHelpers.h"
#include "EdgePlannerHelpers.h"
#include "TimeCSpace.h"
using namespace std;

StateCostControlSpace::StateCostControlSpace(const std::shared_ptr<ControlSpace>& _base,const std::shared_ptr<ObjectiveFunctionalBase>& _objective)
:base(_base),objective(_objective)
{}



std::string StateCostControlSpace::VariableName(int i)
{
  return base->VariableName(i);
}

std::shared_ptr<CSet> StateCostControlSpace::GetControlSet(const State& x)
{
  Vector q; Real c; SplitRef(x,q,c);
  return base->GetControlSet(q);
}

class StateCostSteeringFunction : public SteeringFunction
{
public:
  std::shared_ptr<SteeringFunction> base;
  std::shared_ptr<ObjectiveFunctionalBase> objective;
  StateCostSteeringFunction(const std::shared_ptr<SteeringFunction>& _base,const std::shared_ptr<ObjectiveFunctionalBase>& _objective)
  :base(_base),objective(_objective)
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
    Real c = c0;
    for(size_t i=0;i<qpath.milestones.size();i++) {
      Real u = Real(i)*dt;
      StateCostControlSpace::Join(qpath.milestones[i],c,path.milestones[i]);
      if(i+1 < qpath.milestones.size()) {
        path.controls[i] = qpath.controls[i];
        Real u2 = Real(i+1)*dt;
        Real c2 = c + objective->IncrementalCost(qpath.controls[i],qpath.paths[i].get());
        path.paths[i] = make_shared<MultiInterpolator>(qpath.paths[i],make_shared<LinearInterpolator>(c,c2,qpath.paths[i]->ParamStart(),qpath.paths[i]->ParamEnd()));

        c = c2;
      }
    }
    path.edges.resize(0);
    return true; 
  }
};

std::shared_ptr<SteeringFunction> StateCostControlSpace::GetSteeringFunction()
{
  std::shared_ptr<SteeringFunction> bsf = base->GetSteeringFunction();
  if(bsf) return make_shared<StateCostSteeringFunction>(bsf,objective);
  return NULL;
}

InterpolatorPtr StateCostControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  Vector q; Real c; SplitRef(x0,q,c);
  InterpolatorPtr qpath = base->Simulate(q,u);
  Real dc = objective->IncrementalCost(u,qpath.get());
  return make_shared<MultiInterpolator>(qpath,make_shared<LinearInterpolator>(c,c+dc,qpath->ParamStart(),qpath->ParamEnd()));
}

void StateCostControlSpace::Successor(const State& x0, const ControlInput& u,State& x1)
{
  Vector q; Real c; SplitRef(x0,q,c);
  InterpolatorPtr qpath = base->Simulate(q,u);
  c += objective->IncrementalCost(u,qpath.get());
  Join(qpath->End(),c,x1);
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

StateCostKinodynamicSpace::StateCostKinodynamicSpace(const std::shared_ptr<KinodynamicSpace>& _base,const std::shared_ptr<ObjectiveFunctionalBase>& _objective,Real costMax)
:KinodynamicSpace(NULL,make_shared<StateCostControlSpace>(_base->GetControlSpace(),_objective)),base(_base),scspace(make_shared<MultiCSpace>()),objective(_objective)
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*scspace);
  mspace->Add("",_base->GetStateSpace());
  mspace->Add("cost",make_shared<CostCSpace>(costMax));
  this->stateSpace = scspace;
}

EdgePlannerPtr StateCostKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const std::shared_ptr<Interpolator>& path)
{
  const MultiInterpolator* mi = dynamic_cast<const MultiInterpolator*>(path.get());
  if(!mi) return KinodynamicSpace::TrajectoryChecker(u,path);
  Assert(path->Start().n == GetStateSpace()->NumDimensions());
  Assert(path->End().n == GetStateSpace()->NumDimensions());
  Assert(mi->components[0]->Start().n == base->GetStateSpace()->NumDimensions());
  Assert(mi->components[0]->End().n == base->GetStateSpace()->NumDimensions());
  EdgePlannerPtr ebase = base->TrajectoryChecker(u,mi->components[0]);
  return make_shared<PiggybackEdgePlanner>(GetStateSpace().get(),path,ebase);
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
