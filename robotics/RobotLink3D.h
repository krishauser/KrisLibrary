#ifndef ROBOT_LINK_3D
#define ROBOT_LINK_3D

#include "Frame.h"

/** @ingroup Kinematics
 * @brief Kinematic and physical parameters of a link.
 */
class RobotLink3D
{
public:
  enum Type { Revolute, Prismatic };

  void SetRotationJoint(const Vector3& w);
  void SetTranslationJoint(const Vector3& v);

  ///links's transformation is a function of control qi
  ///say we're link i
  ///T(i->i)(qi) = R(w*qi)oT(v*qi) where R rotates about an axis, T translates
  ///T(i->pi)(qi) = T0(i->pi)*T(i->i)(qi)
  void GetLocalTransform(Real qi,Frame3D& T) const;

  ///velocity of a point (in frame 0) with respect to qi,dqi
  void GetVelocity(Real qi,Real dqi,const Vector3& p,Vector3& vel) const;
  void GetAngularVelocity(Real dqi,Vector3& omega) const;

  ///Jacobian (orientation,position) of a point (in frame 0) with respect to qi
  void GetJacobian(Real qi,const Vector3& p,Vector3& Jo,Vector3& Jp) const;
  void GetOrientationJacobian(Vector3& Jo) const;
  void GetPositionJacobian(Real qi,const Vector3& p,Vector3& Jp) const;

  //Position jacobian of points in frame 0 w.r.t. i 
  void GetJacobian(Real qi,Frame3D& J) const; 
  //Position jacobian of points in frame j w.r.t. i
  void GetJacobian(Real qi,const Frame3D& Tj_World,Frame3D& J) const; 

  void GetWorldInertia(Matrix3& inertiaWorld) const;
  void GetWorldCOM(Vector3& comWorld) const { T_World.mulPoint(com,comWorld); }

  /// Indicates the type of joint- revolute, prismatic
  Type type;
  /// The axis of rotation/translation (in local frame)
  Vector3 w;
  /// The link mass
  Real mass;
  /// The center of mass (in local frame)
  Vector3 com;
  /// The inertia matrix (in local frame)
  Matrix3 inertia;
  /// The initial transformation from the local to the parent frame
  Frame3D T0_Parent;

  /// Temporary - holds the current state of local to world transformation
  Frame3D T_World;
};

#endif
