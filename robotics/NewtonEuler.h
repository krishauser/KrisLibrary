#ifndef NEWTON_EULER_H
#define NEWTON_EULER_H

#include "RobotDynamics3D.h"
#include "Wrench.h"


/** @ingroup Kinematics
 * @brief The Featherstone algorithm for O(n) computation of either
 * joint torques from accelerations, or accelerations from torques.
 *
 * After setting all external wrenches, CalcTorques computes the 
 * necessary joint torques to achieve the specified ddq.
 *
 * After setting all external wrenches, CalcAccel computes the ddq
 * given the specified joint torques.
 *
 * All methods operate on the assumption that the robot dynamic state
 * (q,dq) has been set, and the robot's frames are updated.
 *
 * Rigid body velocity/accelerations are the velocity/angular velocity
 * of the origin of the given link.
 *
 * externalWrenches are given about the link's center of mass.  jointWrenches
 * are given about the joint.
 */
class NewtonEulerSolver
{
public:
  NewtonEulerSolver(RobotDynamics3D& robot);
  //sets the external wrenches equal to gravity
  void SetGravityWrenches(const Vector3& gravity);
  //forward and inverse dynamics, assuming the current state of the robot is updated
  void CalcTorques(const Vector& ddq,Vector& t);
  void CalcAccel(const Vector& t,Vector& ddq);
  void CalcKineticEnergyMatrix(Matrix& B);
  void CalcKineticEnergyMatrixInverse(Matrix& Binv);
  void CalcResidualTorques(Vector& CG);
  void CalcResidualAccel(Vector& ddq0);

  //helpers (also assume current state of robot has been updated)
  void MulKineticEnergyMatrix(const Vector& x,Vector& Bx);
  void MulKineticEnergyMatrix(const Matrix& A,Matrix& BA);
  void MulKineticEnergyMatrixInverse(const Vector& x,Vector& Binvx);
  void MulKineticEnergyMatrixInverse(const Matrix& A,Matrix& BinvA);
  void CalcVelocities();
  void CalcLinkAccel(const Vector& ddq);
  void SelfTest();

  RobotDynamics3D& robot;
  std::vector<Wrench> externalWrenches;  ///<set these to the external wrenches on the links (moments about the CM)

  //temporary/output
  std::vector<std::vector<int> > children;
  std::vector<RigidBodyVelocity> velocities;
  std::vector<RigidBodyVelocity> accelerations;
  std::vector<Wrench> jointWrenches;  ///<element i is the force on link i from the joint to its parent
  std::vector<SpatialMatrix> inertiaMatrices;  ///<element i is the i'th inertia matrix computed in the featherstone algorithm
  std::vector<SpatialVector> biasingForces;     ///<element i is the i'th biasing force computed in the featherstone algorithm
};

#endif

