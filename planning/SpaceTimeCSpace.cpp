#include "SpaceTimeCSpace.h"
#include <math/random.h>

SpaceTimeCSpace::SpaceTimeCSpace(KinodynamicCSpace* _base)
  :base(_base),timeDistanceWeight(1.0),timeNeighborhoodWeight(1.0),passControlTime(false)
{}

void SpaceTimeCSpace::Sample(Config& x) {
  Config q;
  base->Sample(q);
  SetState(0,q,x);
}

void SpaceTimeCSpace::SampleNeighborhood(const Config& c,Real r,Config& x) { 
  x.resize(c.n);
  Config qc,qx;
  GetConfig(c,qc);
  GetConfig(x,qx);
  SetTime(GetTime(c)+Rand(0,r*timeNeighborhoodWeight),qx);
  base->SampleNeighborhood(qc,r,qx);
}

EdgePlanner* SpaceTimeCSpace::LocalPlanner(const Config& a,const Config& b)
{
  Config qa,qb;
  GetConfig(a,qa);
  GetConfig(b,qb);
  return new PiggybackEdgePlanner(this,a,b,base->LocalPlanner(qa,qb));
}

bool SpaceTimeCSpace::IsFeasible(const Config& x)
{
  Config q;
  GetConfig(x,q);
  return base->IsFeasible(q);
}

Real SpaceTimeCSpace::Distance(const Config& x, const Config& y) {
  Config qx,qy;
  GetConfig(x,qx);
  GetConfig(y,qy);
  return base->Distance(qx,qy)+timeDistanceWeight*Abs(GetTime(x)-GetTime(y));
}

void SpaceTimeCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{ 
  Config qx,qy,qout;
  GetConfig(x,qx);
  GetConfig(y,qy);
  base->Interpolate(qx,qy,u,qout);
  SetState((1.0-u)*GetTime(x)+u*GetTime(y),qout,out);
}

void SpaceTimeCSpace::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
{
  Config qx0,qu;
  GetConfig(x0,qx0);
  GetBaseControl(u,qu);
  std::vector<Config> qp;
  base->Simulate(qx0,qu,qp);
  p.resize(qp.size());
  Assert(p.size() >= 1);
  SetState(GetTime(x0),qp[0],p[0]);
  for(size_t i=1;i<p.size();i++) {
    Real v=Real(i)/Real(p.size()-1);
    SetState(GetTime(x0)+v*GetTime(u),qp[i],p[i]);
  }
}
void SpaceTimeCSpace::SimulateEndpoint(const State& x0, const ControlInput& u,State& x1)
{
  Config qx0,qu,qx1;
  GetConfig(x0,qx0);
  GetBaseControl(u,qu);
  base->SimulateEndpoint(qx0,qu,qx1);

  SetState(GetTime(x0)+GetTime(u),qx1,x1);
}

EdgePlanner* SpaceTimeCSpace::TrajectoryChecker(const std::vector<State>& p)
{
  std::vector<Config> qp(p.size());
  for(size_t i=0;i<p.size();i++)
    GetConfig(p[i],qp[i]);
  return new PiggybackEdgePlanner(this,p.front(),p.back(),base->TrajectoryChecker(qp));
}

bool SpaceTimeCSpace::IsValidControl(const State& x,const ControlInput& u)
{
  if(GetTime(u) < 0) return false;
  Config qx,qu;
  GetConfig(x,qx);
  GetBaseControl(u,qu);
  return base->IsValidControl(qx,qu);
}

void SpaceTimeCSpace::SampleControl(const State& x,ControlInput& u)
{
  Config qx,qu;
  GetConfig(x,qx);
  base->SampleControl(qx,qu);
  if(passControlTime)
    u = qu;
  else
    SetState(Rand(0.0,maxTimeStep),qu,u);
}

bool SpaceTimeCSpace::ConnectionControl(const State& x,const State& xGoal,ControlInput& u)
{
  if(GetTime(x) > GetTime(xGoal)) return false;
  Config qx,qgoal,qu;
  GetConfig(x,qx);
  GetConfig(xGoal,qgoal);
  if(base->ConnectionControl(qx,qgoal,qu)) {
    if(passControlTime) {
      //the connection must have exact time stamps... otherwise the timing will get messed up
      Assert(GetTime(xGoal)-GetTime(x) == qu(0));
      u = qu;
    }
    else
      SetState(GetTime(xGoal)-GetTime(x),qu,u);
    return true;
  }
  return false;
}

void SpaceTimeCSpace::Properties(PropertyMap& map) const
{
  base->Properties(map);
  std::vector<Real> minimum,maximum;
  if(map.getArray("minimum",minimum)) {
    minimum.insert(minimum.begin(),-Inf);
    map.setArray("minimum",minimum);
  }
  if(map.getArray("maximum",maximum)) {
    maximum.insert(maximum.begin(),-Inf);
    map.setArray("maximum",maximum);
  }
  std::vector<Real> weights;
  if(!map.getArray("metricWeights",weights)) {
    Vector x;
    base->Sample(x);
    weights.resize(x.n+1,1.0);
  }
  weights[0] = timeDistanceWeight;
  map.setArray("metricWeights",weights);
}

bool SpaceTimeFeasible(Real t,const Config& q,SpaceTimeCSpace* space)
{
  Config x;
  space->SetState(t,q,x);
  return space->IsFeasible(x);
}

bool SpaceTimeVisible(Real ta,const Config& a,Real tb,const Config& b,SpaceTimeCSpace* space)
{
  std::vector<State> path(2);
  space->SetState(tb,b,path[1]);
  if(!space->IsFeasible(path[1]))  return false;  //infeasible goal
  space->SetState(ta,a,path[0]);
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(!e->IsVisible()) {
    delete e;
    return false;
  }
  delete e;
  return true;
}
