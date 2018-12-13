#ifndef ROBOTICS_JOINT_STRUCTURE
#define ROBOTICS_JOINT_STRUCTURE

#include "RobotKinematics3D.h"
#include "IK.h"
#include "WorkspaceBound.h"
using namespace std;


/** @ingroup Kinematics
 * @brief Calculates workspace bounds for a robot with constrained links.  
 * 
 * Define *joint* i to be the pivot point of i, that is, the point
 * between i and parent of i.  Link i is attached to joints
 * i and cj, j=1..., where cj is a child of link i.
 *
 * JointStructure computes a bound on each joint i.
 * Also, it optionally computes bounds on user-provided points 
 * linkPoints[i] on link i.
 *
 * The bounds are calculated for any number of IK constraints as follows.
 * First, call Init().  Then, either fill out an IKProblem structure
 * and call SolveWorkspaceBounds(), or call SolveWorkspaceBounds()
 * for the first constraint, then use IntersectWorkspaceBounds()
 * for the subsequent ones.
 *
 * Alternatively, a link's transformation T_World can be fixed, and
 * then Solve/IntersectWorkspaceBounds() can be called on the link index.
 */
struct JointStructure
{
  JointStructure(const RobotKinematics3D& robot);

  void Init();
  bool IsFeasible() const;
  void SolveWorkspaceBounds(const IKGoal& constraint);
  void IntersectWorkspaceBounds(const IKGoal& constraint);
  void SolveWorkspaceBounds(const vector<IKGoal>& constraints);
  ///assumes link0 is fixed at robot.links[link0].T_World
  void SolveWorkspaceBounds(int link0);
  void IntersectWorkspaceBounds(int link0);
  ///solves bounds starting from the root 0
  void SolveRootBounds();
  void IntersectRootBounds();

  ///solves bounds for joints attached to link
  ///parent is the incoming joint and bounds are assumed to be calulated
  void SolveBoundsIter(int link,int parent);
  void GetLinkWorkspaceBounds(vector<Real>& bounds) const;

  ///expands the bounds on n from the bounds on p
  void Expand(int n,int p);
  Real GetJointDist(int link1, int link2) const;

  static void SetInitialBounds(WorkspaceBound& b,const IKGoal& c,const Vector3& plocal);

  const RobotKinematics3D& robot;
  vector<vector<int> > children;
  vector<vector<Vector3> > linkPoints; ///<any other points on links

  vector<WorkspaceBound> bounds;    ///< bound[i] bounds joint i
  vector<vector<WorkspaceBound> > pointBounds; ///< pointBounds[i][j] bounds linkPoints[i][j]
  //temp
  vector<bool> reached;
};

#endif
