#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include "RobotLink3D.h"
#include "Chain.h"
#include <KrisLibrary/math/matrix.h>

/** @file RobotKinematics3D.h @ingroup Kinematics 
 * Defines a Config and a RobotKinematics3D.
 */

/// @ingroup Kinematics
/// @brief an alias for Vector 
typedef Vector Config;

/** @ingroup Kinematics
 * @brief Class defining kinematics of a robot, and commonly used functions.
 *
 * Defines a robot's kinematic model as an articulated set of links.
 * The Chain superclass defines each link's parents.  Note: links must be
 * defined in topologically sorted order (parents before children) for
 * many algorithms to work.
 *
 * The current configuration is given by a Vector #q.  The current link 
 * frames are updated when UpdateFrames() is called, and are stored
 * in links[i].T_World.
 *
 * Note that the kinematic model is often used for temporary storage
 * (i.e. in planners, simulators, etc.) so you should count on the state
 * being changed.  If you need to store state, copy out the current
 * configuration.
 */
class RobotKinematics3D : public Chain
{
public:
  RobotKinematics3D() {}
  virtual ~RobotKinematics3D() {}
  virtual std::string LinkName(int i) const;  

  /// Initializer: blank robot
  void Initialize(int numLinks);
  /// Initializer: rigid object
  void InitializeRigidObject();
  /// Initializer: merge some robots into one big robot
  void Merge(const std::vector<RobotKinematics3D*>& robots);
  /// Initializer: select a subset of the robot links as DOFs
  void Subset(const RobotKinematics3D& robot,const std::vector<int>& subset);

  /// based on the values in q, update the frames T
  void UpdateFrames();
  /// based on the values in q, update the frames of link T up to the root
  void UpdateSelectedFrames(int link,int root=-1);
  /// sets the current config q and updates frames
  void UpdateConfig(const Config& q);

  /// returns true if q is within joint limits
  bool InJointLimits(const Config& q) const;
  /// normalizes angles in q to be within the range [qmin,qmax]
  void NormalizeAngles(Config& q) const;

  Real GetTotalMass() const;
  Vector3 GetCOM() const;
  Vector3 GetCOM(const Config& q) { UpdateConfig(q); return GetCOM(); }
  Matrix3 GetTotalInertia() const;
  void GetCOMJacobian(Matrix& Jc) const;
  void GetCOMHessian(Matrix& Hx,Matrix& Hy,Matrix& Hz) const;
  void GetGravityTorques(const Vector3& g0, Vector& G) const;
  Real GetGravityPotentialEnergy(const Vector3& g0,Real refHeight=Zero) const;

  ///in the following, pi is a point in the local frame of body i
  void GetWorldPosition(const Vector3& pi, int i, Vector3& p) const;
  const Matrix3& GetWorldRotation(int i) const;
  void GetWorldRotation_Moment(int i, Vector3& m) const;

  ///gets the world velocity/angular velocity of pi, given dq/dt
  void GetWorldVelocity(const Vector3& pi, int i, const Vector& dq, Vector3& dp) const;
  void GetWorldAngularVelocity(int i, const Vector& dq, Vector3& omega) const;
  ///derivative of Ri w.r.t. qj
  bool GetWorldRotationDeriv(int i, int j, Matrix3& dR) const;
  bool GetWorldRotationDeriv_Moment(int i, int j, Vector3& dm) const;
  ///same as above, but m s.t. Ri=e^[m] is specified
  bool GetWorldRotationDeriv_Moment(int i, int j, const Vector3& m,Vector3& dm) const;

  ///gets the jacobian of pi w.r.t qj
  bool GetJacobian(const Vector3& pi, int i, int j, Vector3& dw, Vector3& dv) const;
  bool GetOrientationJacobian(int i, int j, Vector3& dw) const;
  bool GetPositionJacobian(const Vector3& pi, int i, int j, Vector3& dv) const;
  ///gets the jacobian matrix of pi w.r.t q
  ///row 0-2 are angular, 3-5 are translational
  void GetFullJacobian(const Vector3& pi, int i, Matrix& J) const;
  ///rows 3-5 of the above
  void GetPositionJacobian(const Vector3& pi, int i, Matrix& J) const;

  ///for a wrench w=(torque,force) on link i, returns joint torques F = J^t w
  void GetWrenchTorques(const Vector3& torque, const Vector3& force, int i, Vector& F) const;
  void AddWrenchTorques(const Vector3& torque, const Vector3& force, int i, Vector& F) const;
  ///for a force f at pi on link i, returns joint torques F = J^t f
  void GetForceTorques(const Vector3& f, const Vector3& pi, int i, Vector& F) const;
  void AddForceTorques(const Vector3& f, const Vector3& pi, int i, Vector& F) const;

  ///In the following, the pseudo-hessian H[i,j] is d/dqj(dpm/dqi) for
  ///the points pm on link m (in local coordinates).
  ///It is a pseudo-hessian because the orientation components are not
  ///symmetric.  The position components are a true hessian.
  bool GetJacobianDeriv(const Vector3& pm, int m, int i, int j, Vector3& ddtheta,Vector3& ddp) const;
  ///assumes i,j<=m
  void GetJacobianDeriv_Fast(const Vector3& pm, int m, int i, int j, Vector3& ddtheta,Vector3& ddp) const;
  void GetJacobianDeriv(const Vector3& pm, int m, Matrix* Htheta[3], Matrix* Hp[3]) const;
  void GetPositionHessian(const Vector3& pm, int m, Matrix* Hp[3]) const;
  void GetDirectionalHessian(const Vector3& pm, int m, const Vector3& v, Matrix& Hpv) const;

  ///Upper bounds the distance traveled by the point (attached to link i's
  ///frame) when moving from q1 to q2 (in a linear interpolation).
  Real PointDistanceBound(const Vector3& pi,int i,const Config& q1,const Config& q2) const;
  ///A closer upper bound than the previous.  Overwrites robot's state.
  Real PointDistanceBound2(const Vector3& pi,int i,const Config& q1,const Config& q2);

  ///Upper bounds the distance traveled by a sphere attached to the local
  ///frame of i, centered at ci with radius r, when moving
  ///from q1 to q2 (in a linear interpolation).  Uses algorithm 2.
  Real SphereDistanceBound(const Vector3& ci,Real r,int i,const Config& q1,const Config& q2);

  std::vector<RobotLink3D> links;
  Config q;           ///< current configuration
  Vector qMin,qMax;   ///< joint limits
};


#endif
