#ifndef PLANNING_OBJECTIVES_H
#define PLANNING_OBJECTIVES_H

#include <KrisLibrary/Logger.h>
#include <KrisLibrary/planning/CSpace.h>
#include <KrisLibrary/planning/ControlSpace.h>
#include <KrisLibrary/math/matrix.h>
class MilestonePath;
class AnyCollection;

/** @ingroup MotionPlanning
 * @brief A base class for objective functionals of the form
 * J[x,u] = sum_0^N-1 L(xi,ui) dt + Phi(xN)
 *
 * where x0,...,xN is the milestone trajectory and u0,...,uN-1 is the
 * control trajectory.
 */
class ObjectiveFunctionalBase
{
 public:
  ObjectiveFunctionalBase() {}
  virtual ~ObjectiveFunctionalBase() {}

  ///Subclasses: return an identifier for this goal type
  virtual const char* TypeString() { return NULL; }

  ///Subclasses: return a string for printing (optional)
  virtual std::string Description() { if(TypeString()) return TypeString(); return ""; }

  ///Subclasses: return the incremental cost of undertaking the given path. 
  ///This should be equal to the integral of the differential
  ///cost, or at least a good approximation
  virtual Real IncrementalCost(const Interpolator* path) { return 0.0; }
  virtual Real IncrementalCost(const ControlInput& u,const Interpolator* path) { return IncrementalCost(path); }
  virtual Real IncrementalCost(const KinodynamicMilestonePath& path);

  ///Subclasses: return the cost of a terminal state
  virtual Real TerminalCost(const Config& qend) { return 0.0; }

  ///Subclasses: planners may exploit path-invariant costs for faster performance
  virtual bool PathInvariant() const { return false; }

  ///Optional: return the cost of a candidate path starting at time tstart.
  ///This should be equal to the sum of the increment costs + the terminal cost
  virtual Real PathCost(const MilestonePath& path);
  virtual Real PathCost(const KinodynamicMilestonePath& path);

  ///Subclasses: read and write parameters to collection
  virtual bool SaveParams(AnyCollection& collection) { return false; }
  virtual bool LoadParams(AnyCollection& collection) { return false; }
};

/** @ingroup Planning
 * @brief An objective that merges contributions from multiple other
 * objective functions.
 */
class CompositeObjective : public ObjectiveFunctionalBase
{
 public:
  CompositeObjective();
  ~CompositeObjective();

  ///Adds a new component.  Note: this takes ownership of the pointer.
  void Add(const std::shared_ptr<ObjectiveFunctionalBase>& obj,Real weight=1.0);

  virtual const char* TypeString() { return "composite"; }
  virtual std::string Description();

  virtual Real TerminalCost(const Vector& qend);
  virtual Real IncrementalCost(const Interpolator* path);
  virtual bool PathInvariant() const;

  Real norm;
  std::vector<std::shared_ptr<ObjectiveFunctionalBase> > components;
  std::vector<Real> weights;
};

/** @ingroup MotionPlanning
 * @brief A cost functional of the form
 * J[x,u] = int_0^T L(x(t),u(t)) dt + Phi(x(T))
 *
 * where x is the state trajectory and u is the control trajectory
 * with domain [0,T].
 *
 * If timeIndex is provided and nonnegative, then that element of x
 * is assumed to be time. Otherwise, the subclass needs to overload
 * Domain() to tell this class what time domain should be integrated
 * over, i.e., how long each segment takes to execute.  This
 * information may be, for example, in the
 * first element of u.
 */
class IntegratorObjectiveFunctional : public ObjectiveFunctionalBase
{
 public:
  IntegratorObjectiveFunctional(Real dt = 0.1,int timeIndex=-1);
  ///Subclasses must override this
  virtual Real DifferentialCost(const State& x,const ControlInput& u) = 0;

  ///Subclasses should override this if timeIndex < 0 
  virtual Real Domain(const ControlInput& u,const Interpolator* path);

  ///This is implemented for you
  virtual Real IncrementalCost(const ControlInput& u,const Interpolator* path);

  Real dt;
  int timeIndex;
};

/** @ingroup MotionPlanning
 * @brief An objective that measures path length.
 */
class LengthObjective : public ObjectiveFunctionalBase
{
 public:
  LengthObjective() {}
  virtual ~LengthObjective() {}
  virtual const char* TypeString() { return "length"; }
  virtual Real IncrementalCost(const Interpolator* path) { return path->Length(); }
};


/** @ingroup MotionPlanning
 * @brief An objective that measures path execution time.
 * Accumulated time is assumed to be part of the state, specifically the element indexed
 * by timeIndex.
 */
class TimeObjective : public ObjectiveFunctionalBase
{
 public:
  TimeObjective(int timeIndex=0);
  virtual ~TimeObjective() {}
  virtual const char* TypeString() { return "time"; }
  virtual Real IncrementalCost(const Interpolator* path);

  int timeIndex;
};

/** @ingroup Planning
 * @brief A cost that measures distance to a goal configuration qgoal
 */
class ConfigObjective : public ObjectiveFunctionalBase
{
 public:
  ConfigObjective(const Config& qgoal,CSpace* cspace=NULL);
  ConfigObjective(const Config& qgoal,const Vector& weights);
  virtual ~ConfigObjective() {}
  virtual const char* TypeString() { return "config"; }
  virtual Real TerminalCost(const Vector& qend);
  virtual bool PathInvariant() const { return true; }

  Vector qgoal,weights;
  CSpace* cspace;
};

/** @ingroup Planning
 * @brief A cost that measures quadratic tracking error and control cost.
 * Note that the time must exist in the state.
 *
 * The error functional has differential cost
 * L(x,u,t) = (x-xdes(t))^T P (x-xdes(t)) + u^T Q u
 * with P = stateCostMatrix and Q = controlCost matrix. 
 *
 * Terminal cost is Phi(x) = (x-xdes(T))^T R (x-xdes(T))
 * with R = terminalCostMatrix.
 */
class QuadraticObjective : public IntegratorObjectiveFunctional
{
 public:
  QuadraticObjective(int timeIndex=0);
  virtual ~QuadraticObjective() {}
  virtual const char* TypeString() { return "quadratic"; }
  virtual Real TerminalCost(const Vector& qend);
  virtual Real DifferentialCost(const State& x,const ControlInput& u);
  
  InterpolatorPtr desiredPath;
  Math::Matrix stateCostMatrix,controlCostMatrix;
  Math::Matrix terminalCostMatrix;
};


#endif

