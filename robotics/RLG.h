#ifndef ROBOTICS_RLG_H
#define ROBOTICS_RLG_H

#include "JointStructure.h"

/** @ingroup Kinematics
 * @brief A sampler for closed-chain kinematics that takes joint
 * workspace bounds into account.
 *
 * Analytical IK solutions can be incorporated by overloading the
 * SolveAnalyticalIK[X]() functions 
 */
class RLG
{
public:
  RLG(RobotKinematics3D& robot, const JointStructure& js);
  void SampleFrom(int k);
  void Sample(const vector<IKGoal>& ik);
  void Sample(const IKGoal& goal);

  void IterateParent(int k);
  void IterateChild(int k);
  void SampleParent(int k);
  void SampleChild(int k);
  void GetAnglesParent(int k,vector<AngleSet>& a) const;
  void GetAnglesChild(int k,vector<AngleSet>& a) const;
  virtual bool SolveAnalyticIKParent(int k) const { return false; }
  virtual bool SolveAnalyticIKChild(int k) const { return false; }

  RobotKinematics3D& robot;
  const JointStructure& js;
};

/** @ingroup Kinematics
 * @brief RLG that sets a virtual linkage of free dof's using
 * analytic IK.
 *
 * Assumes joints 0-2 are rigid body translation, 3-5 rotation.
 */
class FreeRLG : public RLG
{
public:
  FreeRLG(RobotKinematics3D& robot, const JointStructure& js);
  void SampleFromRoot();
  virtual bool SolveAnalyticIKParent(int k) const;
};

#endif
