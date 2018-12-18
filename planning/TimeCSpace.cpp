#include "TimeCSpace.h"
#include "KinodynamicPath.h"
#include "InterpolatorHelpers.h"
#include "EdgePlannerHelpers.h"
#include "CSetHelpers.h"
#include <math/random.h>
using namespace std;

void Join(Real t,const Config& q,Config& tq)
{
  tq.resize(q.n);
  tq[0] = t;
  tq.copySubVector(1,q);
}

void Split(const Config& tq,Real& t,Config& q)
{
  t = tq[0];
  q.setRef(tq,1,1,tq.n-1);
}

class TimeControlSet : public BoxSet
{
public:
  TimeControlSet(Real dtmax=1.0)
  :BoxSet(0,dtmax)
  {}
  virtual bool IsFeasible(const Vector& x) { return x[0] >= 0; }
};

class TimeSteeringFunction : public SteeringFunction
{
public:
  virtual bool Connect(const Config& x,const Config& y,KinodynamicMilestonePath& path) {
    if(x(0) > y(0)) return false;
    path = KinodynamicMilestonePath(y-x,make_shared<LinearInterpolator>(x,y));
    return true;
  }
};


TimeCSpace::TimeCSpace(Real _timeStepMax)
:BoxCSpace(Vector(1,0.0),Vector(1,Inf)),timeStepMax(_timeStepMax)
{}

void TimeCSpace::Sample(Config& x)
{
  x.resize(1);
  x[0] = Rand()*timeStepMax;
}


TimeControlSpace::TimeControlSpace(Real dtmax)
{
  myControlSet = make_shared<TimeControlSet>(dtmax);
  mySteeringFunction = make_shared<TimeSteeringFunction>();
}
void TimeControlSpace::SetMaxTimeStep(Real dtmax) 
{
  myControlSet = make_shared<TimeControlSet>(dtmax);
}

InterpolatorPtr TimeControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  State x1;
  SimulateEndpoint(x0,u,x1);
  return make_shared<LinearInterpolator>(x0,x1);
}



SpaceTimeCSpace::SpaceTimeCSpace(const shared_ptr<CSpace>& stateSpace,Real tmax)
{
  Add("time",make_shared<TimeCSpace>(tmax));
  Add("",stateSpace);
}

void SpaceTimeCSpace::Sample(Config& x) 
{ 
  MultiCSpace::Sample(x);
  if(IsNaN(x(0)))
    x(0)=0; //we can't sample the time variable, infinite bound
}
void SpaceTimeCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  MultiCSpace::SampleNeighborhood(c,r,x);
  x(0)=c(0)+Rand(0,r); 
}

EdgePlannerPtr SpaceTimeCSpace::LocalPlanner(const Config& a,const Config& b)
{
  if(a(0) > b(0)) return make_shared<FalseEdgeChecker>(this,a,b);
  else return MultiCSpace::LocalPlanner(a,b);
}

EdgePlannerPtr SpaceTimeCSpace::PathChecker(const Config& a, const Config& b)
{
  if(a(0) > b(0)) return make_shared<FalseEdgeChecker>(this,a,b);
  else return MultiCSpace::PathChecker(a,b);
}

/** @brief given a steering function on a base space q' = f(q,ubase), adapts it to the proper
 * steering function on a SpaceTimeIntegratedControlSpace
 * where the state component is x=(t,q) and the control is u=(dt,ubase).
 */
class SpaceTimeSteeringFunction : public SteeringFunction
{
public:
  shared_ptr<SteeringFunction> base;
  SpaceTimeSteeringFunction(const shared_ptr<SteeringFunction> &_base)
  :base(_base)
  {}

  virtual bool IsExact() const { return base->IsExact(); };
  virtual bool IsOptimal() const { return base->IsOptimal(); };

  virtual bool Connect(const State& x,const State& y,KinodynamicMilestonePath& path)
  {
    Real tx = x(0),ty = y(0);
    State qx,qy;
    qx.setRef(x,1,1,x.n-1);
    qy.setRef(y,1,1,y.n-1);
    KinodynamicMilestonePath qpath;
    if(!base->Connect(qx,qy,qpath)) {
      return false;
    }
    path.milestones.resize(qpath.milestones.size());
    path.controls.resize(qpath.controls.size());
    path.paths.resize(qpath.paths.size());
    Real dt = 1.0 / Real(qpath.paths.size());
    for(size_t i=0;i<qpath.milestones.size();i++) {
      Real u = Real(i)*dt;
      Real t = tx + u*(ty-tx);
      Join(t,qpath.milestones[i],path.milestones[i]);
      if(i+1 < qpath.milestones.size()) {
        Join(dt,qpath.controls[i],path.controls[i]);
        Real u2 = Real(i+1)*dt;
        path.paths[i] = make_shared<MultiInterpolator>(make_shared<LinearInterpolator>(u,u2),qpath.paths[i]);
      }
    }
    path.edges.resize(0);
    return true; 
  }
};

SpaceTimeIntegratedControlSpace::SpaceTimeIntegratedControlSpace(const shared_ptr<IntegratedControlSpace>& _base)
:IntegratedControlSpace(_base->myDynamics,_base->controlSet,_base->dt,_base->dtmax),base(_base)
{
  shared_ptr<SteeringFunction> sf = base->GetSteeringFunction();
  if(sf)
    mySteeringFunction = make_shared<SpaceTimeSteeringFunction>(sf);
}

std::string SpaceTimeIntegratedControlSpace::VariableName(int i)
{
  return base->VariableName(i);
}

void SpaceTimeIntegratedControlSpace::Derivative(const State& x, const ControlInput& u,State& dx)
{
  State xbase,dxbase;
  xbase.setRef(x,1,1,x.n-1);
  base->Derivative(x,u,dxbase);
  Join(1.0,dxbase,dx);
}

void SpaceTimeIntegratedControlSpace::UpdateIntegrationParameters(const State& x)
{
  Real tbase;
  State xbase;
  Split(x,tbase,xbase);
  base->UpdateIntegrationParameters(xbase);
  dt = base->dt;
  dtmax = base->dtmax;
  controlSet = base->controlSet;
  type = base->type;
}

SpaceTimeIntegratedKinodynamicSpace::SpaceTimeIntegratedKinodynamicSpace(const shared_ptr<CSpace>& space,const shared_ptr<IntegratedControlSpace>& controlSpace)
:IntegratedKinodynamicSpace(make_shared<SpaceTimeCSpace>(space),make_shared<SpaceTimeIntegratedControlSpace>(controlSpace))
{}

