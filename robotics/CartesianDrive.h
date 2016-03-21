#ifndef ROBOTICS_CARTESIAN_DRIVE_H
#define ROBOTICS_CARTESIAN_DRIVE_H

#include "RobotDynamics3D.h"
#include "IK.h"

/** @ingroup Kinematics
 * @brief Simultaneously moves joints on the robot to achieve one or more
 * Cartesian velocity goals.  A highly configurable and reliable solver
 * suitable for long-duration, precise cartesian motions.
 *
 * Most of the solver is stateless except for driveTransforms and
 * driveSpeedAdjustment.
 */
class CartesianDriveSolver
{
 public:
  CartesianDriveSolver(RobotDynamics3D* robot);
  ///To start: call Init with the desired driven links.  Then set up any
  ///additional settings for the IK solver (ikGoals, activeDofs, qmin,
  ///qmax, vmin, vmax).  Then call Drive() each time you to update the drive
  ///commands. 
  void Init(const Config& q,
	    const std::vector<int>& links);
  ///Alternate init: special end effector offsets (aka Tool Center Points)
  void Init(const Config& q,
	    const std::vector<int>& links,
	    const std::vector<Vector3>& endEffectorPositions);
  ///Alternate init: relative positioning mode.  Each link has a baseLink
  ///by which their cartesian movement is measured.
  void Init(const Config& q,
	    const std::vector<int>& links,const std::vector<int>& baseLinks,
	    const std::vector<Vector3>& endEffectorPositions);
  ///Convenience helper for one link
  void Init(const Config& q,int link);
  ///Convenience helper for one link
  void Init(const Config& q,int link,const Vector3& endEffectorPosition);

  ///Given start configuration qcur, and desired angular/linear velocities
  ///of each link, drives the robot by time step dt to reach those cartesian
  ///goals.  qnext is the resulting configuration, and the result is a value
  ///from 0 to 1 indicating how far along the nominal drive amount the
  ///solver was able to achieve.  If the result is < 0, this indicates that
  ///the solver failed to make further progress.
  ///
  ///For longer moves, you should pass qnext back to this function as qcur.
  ///
  ///Hint: set angVel[i] infinite/NaN to turn off orientation control of
  ///constraint i. 
  ///
  ///Hint: set vel[i] infinite/Nan to turn off position control of
  ///constraint i.
  Real Drive(const Config& qcur,
	     const std::vector<Vector3>& angVel,
	     const std::vector<Vector3>& vel,
	     Real dt,
	     Config& qnext);

  ///Convenience helper for one link
  Real Drive(const Config& qcur,
	     const Vector3& angVel,
	     const Vector3& vel,
	     Real dt,
	     Config& qout);

  ///Retrieves an entire trajectory produces by stepping numSteps steps at
  ///resolution dt. 
  ///
  ///If reset is true (on by default), resets the driveTransforms and
  ///driveSpeedAdjustment to the initial state after the trajectory is
  ///computed
  void GetTrajectory(const Config& qcur,
		     const std::vector<Vector3>& angVel,
		     const std::vector<Vector3>& vel,
		     Real dt,
		     int numSteps,
		     std::vector<Config>& trace,
		     bool reset = true);


  RobotDynamics3D* robot;
  ///IK will maintained up to these tolerances.  Default 1e-3.
  Real positionTolerance,rotationTolerance;
  ///IK solver will terminate when the solver meets this tolerance.  If 0,
  ///the tolerance will be determined from positionTolerance and
  ///rotationTolerance.
  Real ikSolveTolerance;
  ///IK solver will terminate when the solver meets this number of iterations
  int ikSolveIters;
  ///The set of constrained links
  std::vector<int> links;
  ///In relative cartesian positioning mode, these are the base links
  std::vector<int> baseLinks;
  ///The set of end effector offsets.
  std::vector<Vector3> endEffectorOffsets;
  ///The current cartesian drive goals.  Must be of size links.size().
  std::vector<RigidTransform> driveTransforms;
  ///If the IK solver had problems, the drive speed will be reduced
  Real driveSpeedAdjustment;
  ///If set, must be of size links.size().  The driver updates the end
  ///effector DOFs selected for by these goals.  If not set, uses 
  ///fixed position/orientation goals.  
  std::vector<IKGoal> ikGoals;
  ///If not set, uses default IK dofs.  If set, these are the indices to be
  ///updated in Klampt
  std::vector<int> activeDofs;
  ///If set, overrides the robot's default joint limits
  Config qmin,qmax;
  ///If set, overrides the robot's default velocity limits
  Vector vmin,vmax;
};

#endif 



