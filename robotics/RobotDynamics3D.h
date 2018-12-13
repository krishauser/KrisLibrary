#ifndef ROBOT_DYNAMICS_3D_H
#define ROBOT_DYNAMICS_3D_H

#include "RobotKinematics3D.h"
#include <KrisLibrary/structs/array2d.h>

/** @ingroup Kinematics
 * @brief Class defining kinematic and dynamic state of a robot, with
 *  commonly used functions.  Inherits from RobotKinematics3D.
 *
 * The current state is given by configuration q, velocity dq.
 * The dynamic equations are
 * @verbatim
 *     q' = dq,
 *     B(q)*q'' + C(q,dq) + G(q) = f,
 * @endverbatim
 * where B is the kinetic energy matrix, C is the coriolis forces, G is the 
 * generalized gravity vector, and f is the generalized torque vector.
 *
 * Forward and inverse dynamics methods use the slow O(n^3) full matrix
 * method. For faster solutions, use the NewtonEulerSolver class.
 *
 * All methods defined here use the current setting of #dq as the state
 * of the robot.
 *
 * The GetKineticEnergyMatrix*(),GetKineticEnergyDeriv(), GetCoriolis*()
 * CalcAcceleration(), and CalcTorques() methods additionally require
 * UpdateDynamics() to have been called beforehand.
 *
 * @see NewtonEulerSolver
 */
class RobotDynamics3D : public RobotKinematics3D
{
public:
  void Initialize(int numBodies);
  void InitializeRigidObject();
  void Merge(const std::vector<RobotDynamics3D*>& robots);
  void Subset(const RobotDynamics3D& robot,const std::vector<int>& subset);
  void UpdateDynamics();   ///< This calls UpdateFrames() and Update_J()
  void Update_J();         ///< Updates JO and JP.
  void Update_dB_dq();     ///< Updates dB_dq. Called internally by GetCoriolis*()

  /// Check physical limits
  bool InVelocityLimits(const Vector& dq) const;
  bool InTorqueLimits(const Vector& torques) const;
  bool InPowerLimits(const Vector& dq,const Vector& torques) const;

  /// Query helpers
  bool IsUnactuatedLink(int i) const { return torqueMax(i)==0 || powerMax(i)==0; }
  bool IsActuatedLink(int i) const { return !IsUnactuatedLink(i); }

  /// Computes the time derivative of dpi/dqj.
  /// That is, the jacobian of pi on link i with respect to qj.
  bool GetJacobianDt(const Vector3& pi, int i, int j, Vector3&dtheta_dt,Vector3& dp_dt) const;
  /// Given some ddq, gets the acceleration of point pi on link i.
  void GetWorldAcceleration(const Vector3& pi, int i, const Vector& ddq, Vector3& dw,Vector3& dv) const;
  /// Given no joint acceleration, gets the acceleration of point pi on link i.
  void GetResidualAcceleration(const Vector3& pi, int i, Vector3& dw,Vector3& dv) const;
  
  /// Linear momentum of link i
  Vector3 GetLinearMomentum(int i) const;
  /// Linear momentum of robot
  Vector3 GetLinearMomentum() const;
  /// Angular momentum of link i
  Vector3 GetAngularMomentum(int i) const;
  /// Angular momentum of robot
  Vector3 GetAngularMomentum() const;
  /// Kinetic energy of link i
  Real GetKineticEnergy(int i) const;
  /// Kinetic energy of robot
  Real GetKineticEnergy() const;

  /// Computes the kinetic energy matrix B
  void GetKineticEnergyMatrix(Matrix& B) const;
  /// Computes dBij/dqz
  Real GetKineticEnergyDeriv(int i,int j,int z) const;
  /// Computes the derivative of B with respect to q(z)
  void GetKineticEnergyMatrixDeriv(int z,Matrix& dB) const;
  /// Computes dB/dt
  void GetKineticEnergyMatrixTimeDeriv(Matrix& dB) const;
  /// Computes the coriolis force matrix C (such that C*dq = coriolis forces)
  void GetCoriolisForceMatrix(Matrix& C);
  /// Computes the coriolis forces
  void GetCoriolisForces(Vector& Cdq);
  
  //B*ddq + C*dq = fext
  void CalcAcceleration(Vector& ddq, const Vector& fext);
  void CalcTorques(const Vector& ddq, Vector& fext);
  
  Vector dq;   ///< current velocity
  Vector velMin,velMax; ///< velocity limits
  Vector torqueMax;     ///< torque limits
  Vector powerMax;      ///< Power=|torque||velocity| limits
    
  //temp storage
  Array2D<Vector3> JP;       ///<derivative of cm[i] w.r.t. q(j)
  Array2D<Vector3> JO;       ///<derivative of orientation[i] w.r.t. q(j)
  std::vector<Matrix> dB_dq; ///<derivative of the kinetic energy matrix w.r.t q(i)
};


#endif
