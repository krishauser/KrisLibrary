#ifndef ROBOTICS_TIME_CSPACE_H
#define ROBOTICS_TIME_CSPACE_H

#include "KinodynamicCSpace.h"

class TimeCSpace : public KinodynamicCSpace
{
 public:
  TimeCSpace() : maxTimeStep(1.0) {}
  virtual void Sample(Config& x) { x.resize(1);  x(0)=0; }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) { 
    x.resize(1);  x(0)=c(0)+Rand(0,r);  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) {
    if(a(0) > b(0)) return new FalseEdgePlanner(this,a,b);
    else return new TrueEdgePlanner(this,a,b); }
  virtual bool IsFeasible(const Config& x) { return true; }
  virtual Real Distance(const Config& x, const Config& y) { return Abs(x(0)-y(0)); }

  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
  {
    p.resize(2);
    p[0] = x0;
    SimulateEndpoint(x0,u,p[1]);
  }
  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1) { x1 = x0+u; }
  virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p)
  {
    for(size_t i=0;i+1<p.size();i++)
      if(p[i](0) > p[i+1](0)) return new FalseEdgePlanner(this,p.front(),p.back());
    return new TrueEdgePlanner(this,p.front(),p.back());
  }

  virtual bool IsValidControl(const State& x,const ControlInput& u)
  {
    Assert(u.n==1);
    return u(0) >= 0;
  }

  virtual void SampleControl(const State& x,ControlInput& u)
  {
    u.resize(1);
    u(0) = Rand(0,maxTimeStep);
  }

  virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u)
  {
    if(xGoal(0) >= x(0)) {
      u.resize(1);
      u(0) = xGoal(0)-x(0);
      return true;
    }
    return false;
  }

  virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u) { return true; }
  virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p) {
    p.resize(2);
    p[0] = x1-u;
    p[1] = x1;
  }
  virtual bool IsValidReverseControl(const State& x1,const ControlInput& u)
  { return IsValidControl(x1,u);  }
  virtual void SampleReverseControl(const State& x,ControlInput& u) {  SampleControl(x,u);  }

  Real maxTimeStep;
};

#endif // ROBOTICS_TIME_CSPACE_H
