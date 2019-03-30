#ifndef ROBOTICS_KINODYNAMIC_SPACE_H
#define ROBOTICS_KINODYNAMIC_SPACE_H

#include <KrisLibrary/errors.h>
#include "ControlSpace.h"
#include "CSpaceHelpers.h"


/** @ingroup MotionPlanning
 * @brief A class used for kinodynamic planning.  Combines a 
 * CSpace defining the state space, as well as a ControlSpace.
 *
 * Kinodynamic planning is used for systems with nonholonomic constraints
 * as well as other types of state-dependent constraints.
 *
 * A ControlSpace gives a simulation function x' = f(x,u) where x is the input
 * state, u is the control input, and x' is the next state.
 * The control space
 *
 * For differential constraints, please use IntegratedKinodynamicSpace.
 */
class KinodynamicSpace
{
public:
  KinodynamicSpace(const std::shared_ptr<CSpace>& xspace,const std::shared_ptr<ControlSpace>& uspace);
  virtual ~KinodynamicSpace();

  std::shared_ptr<ControlSpace> GetControlSpace() const { return controlSpace; }
  std::shared_ptr<CSpace> GetStateSpace() const { return stateSpace; }
  std::shared_ptr<CSet> GetControlSet(const Config& x) { return controlSpace->GetControlSet(x); }

  virtual EdgePlannerPtr PathChecker(const InterpolatorPtr& path) { FatalError("Visibility not checked using LocalPlanner for kinodynamic planning, use TrajectoryChecker instead"); return NULL; }

  ///Return an edge planner that checks the simulation trace p for feasibility
  ///Typically, just return the state space's PathChecker
  virtual EdgePlannerPtr TrajectoryChecker(const ControlInput& u,const InterpolatorPtr& path);

  ///Return an edge planner that checks the path for feasibility.
  virtual EdgePlannerPtr TrajectoryChecker(const KinodynamicMilestonePath& path);

  bool IsValidControl(const State& x,const ControlInput& u) { return controlSpace->IsValidControl(x,u); }

  ///Executes the simulation function f(x0,u) and records its trace in the result.
  ///The trace is an interpolator between x0 and the successor state
  InterpolatorPtr Simulate(const State& x0, const ControlInput& u) { return controlSpace->Simulate(x0,u); }

  ///Executes the simulation function x1 = f(x0,u)
  void Successor(const State& x0, const ControlInput& u,State& x1) { controlSpace->Successor(x0,u,x1); }

  ///If the trajectory from x0 controlled by u is feasible, sets x1 to
  ///the end state and returns true.  If infeasible, returns false.
  ///(implementation calls Simulate() and  TrajectoryChecker()->IsVisible())
  bool NextState(const State& x0,const ControlInput& u,State& x1);

  ///If the trajectory from x1 reverse-controlled by u is feasible, sets x0
  ///to the initial state and returns true.  If infeasible, returns false.
  ///(implementation calls ReverseSimulate() and  TrajectoryChecker()->IsVisible())
  bool PreviousState(const State& x1,const ControlInput& u,State& x0);

  ///Marks this as being a dynamic problem
  virtual void Properties(PropertyMap& props) const;

  std::shared_ptr<CSpace> stateSpace;
  std::shared_ptr<ControlSpace> controlSpace;
};

/** @ingroup MotionPlanning
 * @brief A class that produces a KinodynamicCSpace from a dynamics function
 * subclassed from IntegratedControlSpace.  Collision checking is performed at
 * the integration resolution given in the dynamics function.
 *
 * A differential constraint is expressed as x' = g(x,u), where x is a state,
 * x' is its time derivative, and u is the control input.  x is still 
 * subject to the standard feasibility constraints (checked using IsFeasible()
 * declared in CSpace), but also u is subject to validity constraints.
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
class IntegratedKinodynamicSpace : public KinodynamicSpace
{
public:
  IntegratedKinodynamicSpace(const std::shared_ptr<CSpace>& xspace,const std::shared_ptr<IntegratedControlSpace>& controlSpace);
  virtual EdgePlannerPtr TrajectoryChecker(const ControlInput& u,const InterpolatorPtr& path);
};

/** @ingroup MotionPlanning
 * @brief A simple approximate steering function that simply draws several
 * samples and picks the closest one in terms of CSpace distance.
 */
class RandomBiasSteeringFunction : public SteeringFunction
{
public:
  RandomBiasSteeringFunction(KinodynamicSpace* space,int sampleCount);
  virtual bool IsExact() const { return false; };
  virtual bool IsOptimal() const { return false; };
  virtual bool Connect(const State& x,const State& y,KinodynamicMilestonePath& path);

  KinodynamicSpace* space;
  int sampleCount;
};

/** @ingroup MotionPlanning
 * @brief A simple approximate steering function that simply draws several
 * samples and picks the closest one in terms of CSpace distance.
 */
class RandomBiasReverseSteeringFunction : public SteeringFunction
{
public:
  RandomBiasReverseSteeringFunction(KinodynamicSpace* space,int sampleCount);
  virtual bool IsExact() const { return false; };
  virtual bool IsOptimal() const { return false; };
  virtual bool Connect(const State& x,const State& y,KinodynamicMilestonePath& path);

  KinodynamicSpace* space;
  int sampleCount;
};


/** @brief Adapts a kinematic cspace (given to the constructor) to a
 * kinodynamic one.
 *
 * The state is the same as in the "base" cspace, and the control is a
 * straight-line motion to another state.  Controls are drawn in a neighborhood
 * around a given state with distance maxNeighborhoodRadius. 
 * 
 * If the destination state or motion are infeasible, the path is infeasible.
 */
class KinematicCSpaceAdaptor : public KinodynamicSpace
{
 public:
  KinematicCSpaceAdaptor(const std::shared_ptr<CSpace>& base,Real maxNeighborhoodRadius=0.1);
  virtual ~KinematicCSpaceAdaptor() {}
};


/** @brief Adapts a kinodynamic space with steering function to a kinematic
 * cspace where the steering function is used as the visibility function.
 *
 * NOTE: the steering function *must* be exact.
 */
class KinodynamicSteeringCSpaceAdaptor : public PiggybackCSpace
{
public:
  KinodynamicSteeringCSpaceAdaptor(const std::shared_ptr<KinodynamicSpace>& kinodynamicSpace);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int constraint);

  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Properties(PropertyMap& map);

  std::shared_ptr<KinodynamicSpace> kinodynamicSpace;
  std::shared_ptr<SteeringFunction> steeringFunction;
};

#endif
