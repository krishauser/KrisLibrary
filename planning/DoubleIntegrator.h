#ifndef PLANNING_DOUBLE_INTEGRATOR_SPACE_H
#define PLANNING_DOUBLE_INTEGRATOR_SPACE_H

#include "KinodynamicSpace.h"
#include "CVSpace.h"

/** @ingroup Planning
 * @brief a space that combines configuration and velocity in the state x=(q,v),
 * and is controlled by acceleration v' = a.  The control u = (dt,a) is composed
 * of a time step and acceleration.
 *
 * The timestep dt is drawn from the range [0,dtmax].
 *
 * Integration and interpolation are exact.  Path checking is inexact and just
 * discretizes a path up to a given tolerance
 */
class DoubleIntegratorKinodynamicSpace : public KinodynamicSpace
{
public:
  DoubleIntegratorKinodynamicSpace(std::shared_ptr<CSpace> qspace,std::shared_ptr<CSpace> dqspace,std::shared_ptr<CSet> ddqset,Real dtmax);
  virtual EdgePlannerPtr TrajectoryChecker(const ControlInput& u,const std::shared_ptr<Interpolator>& path);
  void SetVisibilityEpsilon(Real tol);

  Real visibilityEpsilon;
};

class DoubleIntegratorControlSpace : public IntegratedControlSpace
{
public:
  DoubleIntegratorControlSpace(const std::shared_ptr<CSet>& uset,Real dtmax);
  virtual std::string VariableName(int i);
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);

  virtual void Derivative(const State& x, const ControlInput& u,State& dx);
  void EvalDynamics(const State& x0, const ControlInput& ddx,Real t,State& x1);
};

/** @ingroup Planning
 * A steering function for box-bounded velocity and acceleration constraints.
 * Generates time-optimal paths using trapezoidal velocity profiles.
 */
class DoubleIntegratorBoxBoundedSteeringFunction : public SteeringFunction
{
public:
  DoubleIntegratorBoxBoundedSteeringFunction(const Vector& amax,const Vector& vmax);
  virtual ~DoubleIntegratorBoxBoundedSteeringFunction() {}
  void SetConfigurationBounds(const Config& qmin,const Config& qmax);
  virtual bool IsExact() const { return true; };
  virtual bool IsOptimal() const { return true; };
  virtual bool Connect(const State& x,const State& y,KinodynamicMilestonePath& path);

  std::vector<double> amax,vmax;
  std::vector<double> qmin,qmax;  ///< optional
};


#endif 