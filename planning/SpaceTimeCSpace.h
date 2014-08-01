#ifndef ROBOTICS_SPACE_TIME_CSPACE_H
#define ROBOTICS_SPACE_TIME_CSPACE_H

#include "KinodynamicCSpace.h"

/** @brief A kinodynamic space that sets x=(t,q), u=(dt,u0) with dt > 0.
 * Based on an existing kinodynamic cspace, provided upon input.
 *
 * If feasibility is time-dependent, then IsFeasible() and TrajectoryChecker()
 * must be overloaded.
 *
 * If simulation is time-dependent, then Simulate() and SimulateEndpoint()
 * (and optionally, ReverseSimulate()) must be overloaded.
 */
class SpaceTimeCSpace : public KinodynamicCSpace
{
 public:
  inline void SetState(Real t,const Config& q,State& x) const { x.resize(q.n+1); SetTime(t,x); SetConfig(q,x); }
  inline void GetState(const State& x,Real& t,Config& q) const { t=GetTime(x); GetConfig(x,q); }
  inline Real GetTime(const State& x) const { return x(0); }
  inline void GetConfig(const State& x,Config& q) const { q.setRef(x,1); }
  inline void SetTime(Real t,State& x) const { x(0)=t; }
  inline void SetConfig(const Config& q,State& x) const { x.copySubVector(1,q); }
  inline void GetBaseControl(const ControlInput& u,ControlInput& bu) const { if(passControlTime) bu.setRef(u); else bu.setRef(u,1); }

  SpaceTimeCSpace(KinodynamicCSpace* base);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual bool IsFeasible(const Config& x);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);
  virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p);
  virtual bool IsValidControl(const State& x,const ControlInput& u);
  virtual void SampleControl(const State& x,ControlInput& u);
  virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u);
  virtual void Properties(PropertyMap& map) const;

  KinodynamicCSpace* base;
  Real maxTimeStep;
  bool passControlTime;
  Real timeDistanceWeight,timeNeighborhoodWeight;
};

///Helper: return whether the space-time point (t,q) is feasible
bool SpaceTimeFeasible(Real t,const Config& q,SpaceTimeCSpace* space);

///Helper: returns whether the space-time space contains a path between
///configs a and b at times ta and tb
///assumed (ta,a) is a feasible state
bool SpaceTimeVisible(Real ta,const Config& a,Real tb,const Config& b,SpaceTimeCSpace* space);

#endif // ROBOTICS_SPACE_TIME_CSPACE_H
