#ifndef ANALYTICAL_IK_SOLVERS_H
#define ANALYTICAL_IK_SOLVERS_H

#include "RobotKinematics3D.h"
#include "IK.h"
#include <memory>
#include <vector>
#include <map>


/** @ingroup Kinematics
 * @brief A structure containing computed analytic IK solutions.
 */
struct AnalyticIKSolution
{
  AnalyticIKSolution();
  void DeleteMinimalDistance();

  std::vector<Vector> solutions;   ///< an array of the IK solutions (3D-6D)
  std::vector<bool> minimalDistance;  ///< one per solution: true if the desired point is not reached, but rather the distance is minimized
  bool infinite;            ///< true if infinite # of solutions
};

/** @ingroup Kinematics
 * @brief Hook for an analytic inverse kinematics solver.  The Solve method
 * must be defined by the subclass.
 *
 * For all dof's from the "base" link to the "end effector" link (ee),
 * returns one or more inverse kinematics solutions.
 * T is the desired end effector transform, relative to base.  Usually
 * there are 6 dof's from base to ee (potentially less?).
 * Returns true if the IK for the link is known (if the IK is known,
 * but no IK solutions exist for T, return true with an empty solution).
 */
class AnalyticalIKSolver
{
 public:
  AnalyticalIKSolver() {}
  ~AnalyticalIKSolver() {}
  virtual bool Solve(int base,int ee,
		     const RigidTransform& T,
		     AnalyticIKSolution& solution) const
  { return false; }
};

class AnalyticIKMap
{
 public:
  void GetIndices(const RobotKinematics3D& robot,
		  const IKGoal& goal,
		  std::vector<int>& indices) const;
  bool Solve(RobotKinematics3D& robot,
	     const IKGoal& goal,
	     AnalyticIKSolution& solution) const;
  bool Solve(int base,int ee,
	     const RigidTransform& T,
	     AnalyticIKSolution& solution) const;

  void UpdateRobot(int base,int ee,
		   const AnalyticIKSolution& solution,
		   int index,RobotKinematics3D& robot) const;

  /// Map of (base,ee) pairs to solvers
  std::map<std::pair<int,int>,std::shared_ptr<AnalyticalIKSolver> > solvers;
};

#endif
