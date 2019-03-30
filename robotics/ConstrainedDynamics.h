#ifndef ROBOTICS_CONSTRAINED_DYNAMICS_H
#define ROBOTICS_CONSTRAINED_DYNAMICS_H

#include "RobotDynamics3D.h"
#include "IK.h"

/** @file ConstrainedDynamics.h  @ingroup Kinematics
 * @brief Functions for computing dynamics under constraints
 */

/** @ingroup Kinematics
 * @brief If the fixed links are fixed in place, computes how torques t map to 
 * accelerations ddq.  Assumes q,dq are given in the robot and that the robot's
 * frames are updated.  Provide f if you want the constraint forces.
 */
bool ConstrainedCalcAccel(RobotDynamics3D& robot,
			  const std::vector<int>& fixedLinks,
			  const std::vector<int>& fixedDofs,
			  const Vector& t,Vector& ddq,Vector* f=NULL);
bool ConstrainedCalcTorque(RobotDynamics3D& robot,
			   const std::vector<int>& fixedLinks,
			   const std::vector<int>& fixedDofs,
			   const Vector& ddq,Vector& t,Vector* f=NULL);


/** @ingroup Kinematics
 * @brief If the IK constraints are fixed in place, computes how torques t map to 
 * accelerations ddq.  Assumes q,dq are given in the robot and that the robot's
 * frames are updated.  Provide f if you want the constraint forces.
 */
bool ConstrainedCalcAccel(RobotDynamics3D& robot,
			  const std::vector<IKGoal>& constraints,
			  const Vector& t,Vector& ddq,Vector* f=NULL);
bool ConstrainedCalcTorque(RobotDynamics3D& robot,
			   const std::vector<IKGoal>& constraints,
			   const Vector& ddq,Vector& t,Vector* f=NULL);

/** @ingroup Kinematics
 * @brief If the constraint ddx=dC/dq(q)ddq is given, computes how the torques t
 * map to accelerations ddq.  The jacobian dC/dq(q) is given in the dC_dq param. 
 * Assumes q,dq are given in the robot and that the robot's frames are updated.
 * Provide f if you want the constraint forces.
 */
bool ConstrainedCalcAccel(RobotDynamics3D& robot,
			  const Vector& ddx,const Matrix& dC_dq,
			  const Vector& t,Vector& ddq,Vector* f=NULL);
bool ConstrainedCalcTorque(RobotDynamics3D& robot,
			   const Vector& ddx,const Matrix& dC_dq,
			   const Vector& ddq,Vector& t,Vector* f=NULL);

/** @ingroup Kinematics
 * @brief Returns a linearized dynamic model onto the constraints, such that
 * if t is the torque applied to the system, the acceleration of the constrained
 * system is ddq=A*t+b
 */
bool ConstrainedForwardDynamics(RobotDynamics3D& robot,
				const std::vector<int>& fixedLinks,
				const std::vector<int>& fixedDofs,
				Matrix& A,Vector& b);

/** @ingroup Kinematics
 * @brief Returns a linearized dynamic model onto the constraints, such that
 * if t is the torque applied to the system, the acceleration of the constrained
 * system is ddq=A*t+b
 */
bool ConstrainedForwardDynamics(RobotDynamics3D& robot,
				const Vector& ddx,const Matrix& dC_dq,
				Matrix& A,Vector& b);

/** @ingroup Kinematics
 * @brief Returns a projection of the dynamics onto the constraints, such that
 * if ddq0 is the acceleration of the unconstrained system, then the
 * acceleration of the constrained system is ddq=A*ddq0+b
 */
bool ConstrainedProjector(RobotDynamics3D& robot,
			  const std::vector<int>& fixedLinks,
			  const std::vector<int>& fixedDofs,
			  Matrix& A,Vector& b);

/** @ingroup Kinematics
 * @brief Returns a projection of the dynamics onto the constraints
 * ddx=dC/dq(q)*ddq, such that if ddq0 is the acceleration of the unconstrained
 * system, then the acceleration of the constrained system is ddq=A*ddq0+b
 */
bool ConstrainedProjector(RobotDynamics3D& robot,
			  const Vector& ddx,const Matrix& dC_dq,
			  Matrix& A,Vector& b);


#endif
