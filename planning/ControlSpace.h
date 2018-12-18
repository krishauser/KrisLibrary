#ifndef PLANNING_CONTROL_SPACE_H
#define PLANNING_CONTROL_SPACE_H

#include "CSpace.h"
#include "Interpolator.h"
#include "CSpaceHelpers.h"
namespace Math {
  class VectorFieldFunction;
} //namespace Math
typedef Vector State;
typedef Vector ControlInput;
class KinodynamicMilestonePath;

//forward declarations
class ControlSpace;
class ReversibleControlSpace;
class SteeringFunction;
class IntegratedControlSet;
class IntegratedControlSpace;
class KinematicControlSpace;


/** @ingroup MotionPlanning
 * @brief Encodes the dynamics of a system, including the dynamics function
 * f(x,u), control bounds, and available steering functions.
 */
class ControlSpace
{
public:
  ControlSpace() {}
  virtual ~ControlSpace() {}

  ///Returns an identifier for the i'th control variable
  virtual std::string VariableName(int i);

  ///Returns this space's control set at the given
  ///state.  By default returns myControlSet.
  virtual std::shared_ptr<CSet> GetControlSet(const State& x) { return myControlSet; }

  ///Returns this space's steering function, if available
  virtual std::shared_ptr<SteeringFunction> GetSteeringFunction() { return mySteeringFunction; }

  ///Executes the simulation function f(x0,u) and records its trace in p.
  ///The trace is an interpolator between x0 and the successor state
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u)=0;

  ///Executes the simulation function x1 = f(x0,u).  By default, uses
  ///the result from Simulate().
  virtual void Successor(const State& x0, const ControlInput& u,State& x1) {
    InterpolatorPtr p = Simulate(x0,u);
    x1 = p->End();
  }

  ///If possible, express Successor as a VectorFieldFunction on the vector (x,u)
  ///(stacked)
  virtual Math::VectorFieldFunction* SuccessorNumeric() { return NULL; }

  ///Samples a control u at state x.  By default, uses
  ///the Sample method from the set returned from GetControlSet(x).
  ///Note: some planners may use this method, and some may get the
  ///control set and call its Sample method.  So overloads of this
  ///method should not change the sampling distribution (unless you
  ///really know what you're doing.)
  virtual void SampleControl(const State& x,ControlInput& u) { GetControlSet(x)->Sample(u); }

  ///Checks validity of control u at x  By default uses
  ///the Contains method from the set returned from GetControlSet(x).
  ///Note: overloads should not change the *result* of this default
  ///behavior but they are allowed to improve running time.
  virtual bool IsValidControl(const State& x,const ControlInput& u) { return GetControlSet(x)->Contains(u); }

  ///Dynamically overridable default control set (Note: state independent)
  std::shared_ptr<CSet> myControlSet;
  ///Dynamically overridable default steering function
  std::shared_ptr<SteeringFunction> mySteeringFunction;
};

/** @ingroup MotionPlanning
 * A control space that also can be reversed.
 * 
 * In addition to the forward successor function x1 = f(x0,u), this also defines a precessor
 * function x0 = g(x1,u') and a control reversal u' such that g(f(x0,u),u') = x1.
 */
class ReversibleControlSpace : public ControlSpace
{
public:
  ///If the system is reversible and x1 = f(x0,u), changes u so that
  ///x0 = f(x1,u) and returns true.  If no such u exists, return false.
  virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u) { return false; }

  std::shared_ptr<ControlSpace> reverseControlSpace;
};

/** @ingroup MotionPlanning
 * @brief A function in a ControlSpace that attempts to connect two states with a sequence of
 * one or more controls.
 */
class SteeringFunction
{
public:
  SteeringFunction() {}
  virtual ~SteeringFunction() {}
  virtual bool IsExact() const { return true; };
  virtual bool IsOptimal() const { return true; };
  virtual bool Connect(const State& x,const State& y,KinodynamicMilestonePath& path)=0;
};


/** @ingroup MotionPlanning
 * @brief A cartesian-product control space in which groups of states are controlled by
 * individual control spaces.
 */
 /*
class MultiControlSpace : public ControlSpace
{
public:
  MultiControlSpace();
  MultiControlSpace(const std::vector<int>& istateStarts,const std::vector<std::shared_ptr<ControlSpace> >& spaces);
  MultiControlSpace(const MultiCSpace* space,const std::vector<std::shared_ptr<ControlSpace> >& spaces);
  void Add(int istatemin,int istatemax,const std::shared_ptr<ControlSpace>& item);
  void Add(CSpace* space,const std::shared_ptr<ControlSpace>& item);
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);
  virtual Math::VectorFieldFunction* SuccessorNumeric();

  std::vector<pair<int,int>  istateRanges;
  std::vector<std::shared_ptr<ControlSpace> > components;
};
*/

/** @brief The default ControlSet for an IntegratedControlSpace
 */
class IntegratedControlSet : public CSet
{
public:
  enum TimeSelection { Uniform, Maximum, Biased };

  IntegratedControlSet(const std::shared_ptr<CSet>& base,Real dtmax);
  virtual int NumDimensions() const;
  virtual bool Project(Config& x);
  virtual bool IsSampleable() const;
  virtual void Sample(ControlInput& u);
  virtual bool Contains(const ControlInput& u);
  
  TimeSelection timeSelection;
  std::shared_ptr<CSet> base;
  Real dtmax;
};

/** @brief Base class for adapting a simulation function by integrating forward
 * dynamics into a ControlSpace.  The "base" control set does not contain
 * timing information, while the control set for this space also includes a timestep.
 *
 * Using this as a base class is convenient if you know nothing beyond
 * the system dynamics.
 *
 * The Derivative() method implements the x'=g(x,u) function where x is a 
 * state and u is a "base" control.
 * 
 * The UpdateIntegrationParameters() method can be overloaded to implement
 * state-dependent timestep / control selection.
 *
* The resulting control input u contains a timestep parameter and the base control
* such that u = (timestep,ubase).
 */
class IntegratedControlSpace : public ControlSpace
{
 public:
  typedef void (*DynamicsFn)(const State& x, const ControlInput& u,State& dx);

  enum Integrator { Euler, RK4 };

  IntegratedControlSpace(const std::shared_ptr<CSet>& fControlSet,Real dt=0.01,Real dtmax=0.1);
  IntegratedControlSpace(DynamicsFn f,const std::shared_ptr<CSet>& fControlSet,Real dt=0.01,Real dtmax=0.1);
  void SetGeodesicSpace(GeodesicSpace* space);
  void SetBaseControlSet(std::shared_ptr<CSet> baseControlSet);
  std::shared_ptr<CSet> GetBaseControlSet();
  virtual std::string VariableName(int i);
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual std::shared_ptr<CSet> GetControlSet(const Config& x);

  //subclasses may override the following:

  ///Compute dx=x'=g(x,u)
  virtual void Derivative(const State& x, const ControlInput& u,State& dx);

  ///Update controlSpace, dt, or dtmax if state-dependent
  virtual void UpdateIntegrationParameters(const State& x) {}

  DynamicsFn myDynamics;
  Integrator type;
  GeodesicSpace* space;
  std::shared_ptr<CSet> controlSet;
  Real dt;          ///< integration time step
  Real dtmax;       ///< maximum dt chosen in controls
};



/** @brief Adapts a kinematic cspace (given to the constructor) to a
 * control space.
 *
 * The state is the same as in the "base" cspace, and the control is a
 * straight-line motion to another state.  Controls are drawn in a neighborhood
 * around a given state with distance maxNeighborhoodRadius. 
 */
class KinematicControlSpace : public ReversibleControlSpace
{
 public:
  KinematicControlSpace(const std::shared_ptr<CSpace>& base,Real maxNeighborhoodRadius=0.1);
  virtual ~KinematicControlSpace() {}

  virtual std::string VariableName(int i);
  virtual std::shared_ptr<CSet> GetControlSet(const Config& x);

  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);
  virtual Math::VectorFieldFunction* SuccessorNumeric();

  virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u);

  std::shared_ptr<CSpace> base;
  Real maxNeighborhoodRadius;
};



#endif