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
  StateCostControlSpace(const SmartPointer<ControlSpace>& base,const SmartPointer<ObjectiveFunctionalBase>& objective);
  virtual std::string VariableName(int i);
  virtual SmartPointer<CSet> GetControlSet(const State& x);
  virtual SmartPointer<SteeringFunction> GetSteeringFunction();
  virtual Interpolator* Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);

  static void Split(const State& xc,State& x,Real& cost);
  static void SplitRef(const State& xc,State& x,Real& cost);
  static void Join(const State& x,Real cost,State& xc);
  static Real Cost(const State& xc);

  SmartPointer<ControlSpace> base;
  SmartPointer<ObjectiveFunctionalBase> objective;
};

class StateCostKinodynamicSpace: public KinodynamicSpace
{
public:
  StateCostKinodynamicSpace(const SmartPointer<KinodynamicSpace>& base,const SmartPointer<ObjectiveFunctionalBase>& objective,Real costMax=Inf);
  virtual EdgePlanner* TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path);

  void SetCostMax(Real costmax);
  void SetCostDistanceWeight(Real weight);
  Real GetCostMax();
  Real GetCostDistanceWeight();

  SmartPointer<KinodynamicSpace> base;
  SmartPointer<CSpace> scspace;
  SmartPointer<ObjectiveFunctionalBase> objective;
};

#endif