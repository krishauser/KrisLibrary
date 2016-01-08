#ifndef ROBOTICS_KINODYNAMIC_CSPACE_H
#define ROBOTICS_KINODYNAMIC_CSPACE_H

#include "CSpace.h"
#include "EdgePlanner.h"
#include <KrisLibrary/errors.h>
#include <queue>
typedef Vector State;
typedef Vector ControlInput;

/** @brief An edge planner that connects two feasible milestones with a
 * specified path
 *
 * path.front() and back() are the start and goal configs, respectively.
 * The straight-line segments are subdivided with resolution epsilon.
 */
class GivenPathEdgePlanner : public EdgePlanner
{
 public:
  GivenPathEdgePlanner(CSpace* space,const std::vector<State>& path,Real epsilon);
  virtual ~GivenPathEdgePlanner();
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual const Config& Start() const { return path.front(); }
  virtual const Config& Goal() const { return path.back(); }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;

  virtual Real Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;

 public:
  CSpace* space;
  std::vector<Config> path;
  std::vector<Real> dists;  //stores the arc length from the start milestone
  Real epsilon;

  //returns the segment of the path that parameter [0,1] lies upon,
  //and the segment parameter t in [0,1]
  int Segment(Real u,Real& t) const;
  //returns the arc length from the start configuration of parameter u
  //assuming a euclidean (or weighted euclidean) metric
  Real ArcLength(Real u) const;

  struct PathBisector
  {
    bool operator < (const PathBisector& p) const { return (bdist-adist) < (p.bdist-p.adist); }
    Real a,b;
    Real adist,bdist;
  };

  std::priority_queue<PathBisector,std::vector<PathBisector> > edgeQueue;
};

/** @brief A CSpace used for kinodynamic planning.
 *
 * Kinodynamic planning can be used for systems with nonholonomic constraints
 * as well as other types of state-dependent constraints.
 *
 * A nonholonomic constraint is expressed as x' = g(x,u), where x is a state,
 * x' is its time derivative, and u is the control input.  x is still 
 * subject to the standard feasibility constraints (checked using IsFeasible()
 * declared in CSpace), but also u is subject to validity constraints
 * (checked using IsValidControl(), declared here).
 * 
 * This constraint is described by a "simulation function".  Given an
 * initial state x0 and control input u, the function f(x0,u) returns
 * a new state x1.  In the case of the nonholonomic constraint x'=g(x,u),
 * we could define f as the Euler step
 *
 *    f(x0,u) = x0 + dt*g(x0,u)
 *
 * or any other type of PDE integrator.  The functionality should be
 * implemented in the function Simulate().
 *
 * Note: the choice of time step dt can potentially create a resolution issue.
 * It may be wise to include dt in the control input variable.  I.e. let
 * u = [dt, v], and g = g(x,v).  This allows some flexibility in choosing the
 * time step during sampling.
 */
class KinodynamicCSpace : public CSpace
{
public:
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { FatalError("Visibility not checked using LocalPlanner for kinodynamic planning"); return NULL; }

  ///Executes the simulation function f(x0,u) and records its trace in p.
  ///The trace is a vector containing all intermediate states.
  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)=0;

  ///Executes the simulation function x1 = f(x0,u)
  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1) {
    std::vector<State> p;
    Simulate(x0,u,p);
    x1 = p.back();
  }

  ///Return an edge planner that checks the simulation trace p for feasibility
  ///Typically, just return new GivenPathEdgePlanner(this,p,tolerance)
  virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p)=0;

  ///Returns true if u is a valid control input starting from state x
  virtual bool IsValidControl(const State& x,const ControlInput& u)=0;

  ///Randomly pick a control input
  virtual void SampleControl(const State& x,ControlInput& u)=0;

  ///Pick control input u that, starting from x, is expected to get closer
  ///to xGoal than a random u
  virtual void BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u) { ChoiceBiasedSampleControl(x,xGoal,u,10); }

  ///Calculate the control input that reaches xGoal from x, return true if
  ///successful.  Required for bidirectional planning.
  virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u) { return false; }

  ///If the system is reversible and x1 = f(x0,u), changes u so that
  ///x0 = f(x1,u)
  virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u) { return false; }

  ///Integrates the simulation function backwards, and records its trace in p.
  ///That is, finds x0 such that x1 = f(x0,u), and records the trace in p.
  ///The trace is a vector containing all intermediate states, starting from x0 and ending at x1.
  virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p) { return false; }

  ///Returns true if u is a valid control input ending at state x.
  ///Default implementation assumes state-independent feasibility
  virtual bool IsValidReverseControl(const State& x1,const ControlInput& u) { return IsValidControl(x1,u); }

  ///Randomly pick a reversed control input.
  ///Default implementation assumes state-independent distribution
  virtual void SampleReverseControl(const State& x,ControlInput& u) { SampleControl(x,u); }

  ///Pick control input u that, reverse-simulating from x1 will get
  ///closer to xStart
  virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u) { ChoiceBiasedSampleReverseControl(x1,xDest,u,10); }

  ///Simple implementation of BiasedSampleControl(). numSamples controls
  ///are chosen uniformly at random, and the best one is picked
  void ChoiceBiasedSampleControl(const State& x,const State& xDest,ControlInput& u,int numSamples);

  ///Simple implementation of BiasedSampleReverseControl(). numSamples controls
  ///are chosen uniformly at random, and the best one is picked
  void ChoiceBiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u,int numSamples);

  ///If the trajectory from x0 controlled by u is feasible, sets x1 to
  ///the end state and returns true.
  ///(implementation calls Simulate() and  TrajectoryChecker()->IsVisible())
  bool NextState(const State& x0,const ControlInput& u,State& x1);

  ///If the trajectory from x1 reverse-controlled by u is feasible, sets x0
  ///to the initial state and returns true.
  ///(implementation calls ReverseSimulate() and  TrajectoryChecker()->IsVisible())
  bool PreviousState(const State& x1,const ControlInput& u,State& x0);
};

/** @brief Implements a simulation function by integrating forward dynamics.
 *
 * Using this as a base class is convenient if you know nothing beyond
 * the system dynamics.
 *
 * The XDerivative() method implements the x'=g(x,u) function.
 *
 * The integration parameters are retrieved using Parameters().
 */
class IntegratedKinodynamicCSpace : public KinodynamicCSpace
{
 public:
  enum Integrator { Euler, RK4 };

  IntegratedKinodynamicCSpace(Integrator type=Euler);
  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);

  //subclasses must override the following:

  ///Compute dx=x'=g(x,u)
  virtual void XDerivative(const State& x, const ControlInput& u,State& dx)=0;

  ///Return (deterministically) the time step in dt and the number of
  ///integration steps in numSteps.
  virtual void Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps)=0;

  Integrator type;
};

/** @brief Adapts a kinematic cspace (given in the constructor) to a
 * kinodynamic one.
 *
 * The state is the same as in the "base" cspace, and the action is a
 * straight-line motion to another state.  If the destination state or motion
 * are infeasible, the path is infeasible.
 */
class KinematicCSpaceAdaptor : public KinodynamicCSpace
{
 public:
  KinematicCSpaceAdaptor(CSpace* base);
  virtual ~KinematicCSpaceAdaptor() {}

  //pass-throughs to base space
  virtual void Sample(Config& x) { base->Sample(x); }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) { base->SampleNeighborhood(c,r,x); }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return base->LocalPlanner(a,b); }
  virtual bool IsFeasible(const Config& x) { return base->IsFeasible(x); }
  virtual Real Distance(const Config& x, const Config& y) { return base->Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { base->Interpolate(x,y,u,out); }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) { base->Midpoint(x,y,out); }

  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);
  virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p);
  virtual bool IsValidControl(const State& x,const ControlInput& u);
  virtual void SampleControl(const State& x,ControlInput& u);
  virtual void BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u);
  virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u);

  virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u);
  virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p);
  virtual bool IsValidReverseControl(const State& x1,const ControlInput& u);
  virtual void SampleReverseControl(const State& x,ControlInput& u);
  virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u);
  virtual void Properties(PropertyMap& map) const;

  CSpace* base;
  Real maxNeighborhoodRadius;
};


#endif
