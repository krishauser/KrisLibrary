#ifndef PLANNING_COST_SPACE_H
#define PLANNING_COST_SPACE_H

#include "ControlSpace.h"
#include "KinodynamicSpace.h"
#include "Objective.h"

/** @brief A control space that adapts a state-space control space
 * to a state-cost space.  The new state space
 * is (x,c) where c is accumulated cost.  
 */
class StateCostControlSpace: public ControlSpace
{
public:
  StateCostControlSpace(const std::shared_ptr<ControlSpace>& base,const std::shared_ptr<ObjectiveFunctionalBase>& objective);
  virtual std::string VariableName(int i);
  virtual std::shared_ptr<CSet> GetControlSet(const State& x);
  virtual std::shared_ptr<SteeringFunction> GetSteeringFunction();
  virtual InterpolatorPtr Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);

  static void Split(const State& xc,State& x,Real& cost);
  static void SplitRef(const State& xc,State& x,Real& cost);
  static void Join(const State& x,Real cost,State& xc);
  static Real Cost(const State& xc);

  std::shared_ptr<ControlSpace> base;
  std::shared_ptr<ObjectiveFunctionalBase> objective;
};

class StateCostKinodynamicSpace: public KinodynamicSpace
{
public:
  StateCostKinodynamicSpace(const std::shared_ptr<KinodynamicSpace>& base,const std::shared_ptr<ObjectiveFunctionalBase>& objective,Real costMax=Inf);
  virtual EdgePlannerPtr TrajectoryChecker(const ControlInput& u,const std::shared_ptr<Interpolator>& path);

  void SetCostMax(Real costmax);
  void SetCostDistanceWeight(Real weight);
  Real GetCostMax();
  Real GetCostDistanceWeight();

  std::shared_ptr<KinodynamicSpace> base;
  std::shared_ptr<CSpace> scspace;
  std::shared_ptr<ObjectiveFunctionalBase> objective;
};

#endif