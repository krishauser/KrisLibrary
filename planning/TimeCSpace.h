#ifndef ROBOTICS_TIME_CSPACE_H
#define ROBOTICS_TIME_CSPACE_H

#include "CSpaceHelpers.h"
#include "KinodynamicSpace.h"

/** @brief a 1D cspace in the range [0,inf) but where Sample samples
 * from the range [0,timeStepMax]
 */
class TimeCSpace : public BoxCSpace
{
public:
  TimeCSpace(Real timeStepMax);
  virtual void Sample(Config& x);

  Real timeStepMax;
};

/** @brief A 1D ControlSpace that simulates time. */
class TimeControlSpace : public ControlSpace
{
 public:
  TimeControlSpace(Real dtmax=1.0);
  void SetMaxTimeStep(Real dtmax);
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1) { x1 = x0+u; }
};

/** @ingroup MotionPlanning
 * @brief A cspace that prepends a time variable to the given CSpace.
 * This is best used with perturbation-sampling planners, and/or
 * SpaceTimeIntegratedKinodynamicCSpace.
 */
class SpaceTimeCSpace : public MultiCSpace
{
public:
  SpaceTimeCSpace(const std::shared_ptr<CSpace>& stateSpace,Real tmax=Inf);
  void SetTimeMetricWeight(Real weight);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
};

/** @brief Given an IntegratedControlSpace on a certain state space, this will produce a
 * control space for the t-c state space that has time prepended to state.
 */
class SpaceTimeIntegratedControlSpace : public IntegratedControlSpace
{
public:
  SpaceTimeIntegratedControlSpace(const std::shared_ptr<IntegratedControlSpace>& base);
  virtual std::string VariableName(int i);
  virtual void Derivative(const State& x, const ControlInput& u,State& dx);
  virtual void UpdateIntegrationParameters(const State& x);
  std::shared_ptr<IntegratedControlSpace> base;
};

/** @ingroup MotionPlanning
 * @brief This KinodynamicCSpace prefixes time onto the state of the given CSpace instance
 * and automatically increments the time state variable as the dynamics are integrated forward.
 */
class SpaceTimeIntegratedKinodynamicSpace : public IntegratedKinodynamicSpace
{
public:
  SpaceTimeIntegratedKinodynamicSpace(const std::shared_ptr<CSpace>& space,const std::shared_ptr<IntegratedControlSpace>& controlSpace);
  void SetTimeMetricWeight(Real weight);
};


#endif // ROBOTICS_TIME_CSPACE_H
