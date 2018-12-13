#ifndef ROBOTICS_ROTATION_H
#define ROBOTICS_ROTATION_H

#include <KrisLibrary/math/matrix.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>

using namespace Math3D;

/** @file Rotation.h  @ingroup Kinematics
 * @brief Utility functions for converting between different rotation
 * representations and computing derivatives.
 */

/** @addtogroup Kinematics */
/*@{*/

/** @brief 9-D rotation parameterization that lays out the columns of
 * the rotation matrix in a single vector.
 */
struct DirectionCosines : public Vector
{
  DirectionCosines();
  DirectionCosines(const DirectionCosines& c);
  DirectionCosines(const Vector& x);
  DirectionCosines(const Matrix3& m);

  void setMatrix(const Matrix3&);
  void getMatrix(Matrix3&) const;
  void getMoment(MomentRotation& mr) const;
  void getMomentJacobian(Matrix& J) const;
  void getQuaternion(QuaternionRotation& q) const;
  void getQuaternionJacobian(Matrix& J) const;

  ///derivative of parameters w.r.t. moment parameterization
  static void momentJacobian(const MomentRotation& mr,Matrix& J);
  ///derivative of parameters w.r.t. quaternion parameterization
  static void quaternionJacobian(const Quaternion& q,Matrix& J);

  inline Real& a(int i) { return operator()(i); }
  inline Real& b(int i) { return operator()(i+3); }
  inline Real& c(int i) { return operator()(i+6); }

  inline const Real& a(int i) const { return operator()(i); }
  inline const Real& b(int i) const { return operator()(i+3); }
  inline const Real& c(int i) const { return operator()(i+6); }
};

/// Calculates the fixed point of the RigidTransform T.  Returns false
/// if T contains no rotation.
bool GetRotationCenter(const RigidTransform& T,Vector3& p);

/// Calculates the rotation matrix R that rotates x onto y, minimal
/// in the sense of minimal angle.  x,y are unit vectors.
void GetMinimalRotation(const Vector3& x,const Vector3& y,Matrix3& R);

/// Calculates the rotation matrix R that rotates x to be orthogonal
/// to y, minimal in the sense of minimal angle. x,y are unit vectors.
void GetMinimalRotationToPlane(const Vector3& x,const Vector3& y,Matrix3& R);

/** @brief Derivative dR of the rotation matrix R of a frame as it rotates
 * about the axis z.
 *
 * The current rotation is given as R. The derivative is calculated with
 * the formula dR = [z]*R.
 */
void MatrixDerivative(const Matrix3& R,const Vector3& z,Matrix3& dR);

/// Same as above, but R and dR are parameterized by direction
/// cosines.
void DirectionCosinesDerivative(const Vector& R,const Vector3& z,Vector& dR);

/// Same as above, but calculates the derivative of the moment 
/// representation of R (if R = e^[m], calculates dm/dz)
void MomentDerivative(const Matrix3& R,const Vector3& z,Vector3& dm);
/// Same as above, but specifies R with the moment m.
void MomentDerivative(const MomentRotation& m,const Vector3& z,Vector3& dm);
/// Same as above, but both m and R are provided
void MomentDerivative(const Vector3& m,const Matrix3& R,const Vector3& z,Vector3& dm);

/// Calculates the derivative of the quaternion representation of R
/// as it rotates around the axis z.
void QuaternionDerivative(const Matrix3& R,const Vector3& z,Quaternion& dq);

/** @brief Calculates the derivative of the euler angle representation of R
 * as it rotates around the axis z.
 *
 * R is specified with theta= (t1,t2,t3), which rotate about the axes
 * indexed by u,v,w in order. i.e. R=Ru(t1)Rv(t2)Rw(t3) where
 * indices 0,1,2 indicate axes (x,y,z).
 * False may be returned if the euler angles are singular.
 */
bool EulerAngleDerivative(const Vector3& theta,const Vector3& z,int u,int v,int w,Vector3& dtheta);

/** @brief Calculates the angular velocity z of the rotation described by theta
 * given its time derivatives dtheta.
 *
 * R is specified with the euler angles theta=(t1,t2,t3), which rotate about
 * the axes indexed by u,v,w in order. i.e. R=Ru(t1)Rv(t2)Rw(t3) where
 * indices 0,1,2 indicate axes (x,y,z).
 */
void AngularVelocityEulerAngle(const Vector3& theta,const Vector3& dtheta,int u,int v,int w,Vector3& z);

/// Calculates the rotation moment for each euler angle, returns them in
/// the 3 columns of A.  Rotation matrix R specified as above.
void EulerAngleMoments(const Vector3& theta,int u,int v,int w,Matrix3& A);

/** @brief Returns the absolute angle of the rotation R.
 *
 * Calculated as Acos((tr(R)-1)/2)
 */
Real MatrixAbsoluteAngle(const Matrix3& R);

/** @brief Returns the angle that R makes about the axis a.
 *
 * If R is not a rotation about the axis, returns the angle theta such
 * that the angle between R and R(a,theta) is minimized.
 */
Real MatrixAngleAboutAxis(const Matrix3& R,const Vector3& a);

/** @brief Derivative dtheta/dz, the change in the absolute angle of 
 * rotation R as it rotates about the axis z.  
 *
 * The angle of a rotation R is Acos((tr(R)-1)/2), and the derivative is 
 * calculated as -tr([z]*R)/2sin(theta).
 */
Real MatrixAngleDerivative(const Matrix3& R,const Vector3& z);

/** @brief Second derivative ddR of the rotation matrix R of a
 * frame as it rotates with velocity w, acceleration a
 * about the axis z.
 *
 * The current rotation is given as R. The 2nd derivative is calculated with
 * the formula ddR = ([a]+[w]^2)*R.
 */
void MatrixDerivative2(const Matrix3& R,const Vector3& w,const Vector3& a,Matrix3& ddR);

/** @brief Computes the angular velocity z at time 0, given current rotation
 * and rotation at next time step.
 *
 * R0=R(0), R1=R(h), h is the time step.
 */
void ForwardDifferenceAngularVelocity(const Matrix3& R0,const Matrix3& R1,Real h,Vector3& z);

/** @brief Computes the angular velocity z at time 0, given current rotation,
 * and rotations at previous/next time steps.
 *
 * R0=R(0), R_1=R(-h), R1=R(h), h is the time step.
 */
void CenteredDifferenceAngularVelocity(const Matrix3& R_1,const Matrix3& R0,const Matrix3& R1,Real h,Vector3& z);

/** @brief Computes the angular acceleration a at time 0,
 * given current rotation, and rotations at previous/next time steps.
 *
 * R0=R(0), R_1=R(-h), R1=R(h), h is the time step.
 */
void CenteredDifferenceAngularAccel(const Matrix3& R_1,const Matrix3& R0,const Matrix3& R1,Real h,Vector3& a);

/** @brief Computes the "amount" of rotation about the axis a that thegiven
 * rotation matrix R performs.  Specifically, finds the angle theta such that
 * e^[theta*a] minimizes the angular distance to R.
 *
 * E.g., if R=e^[u*a], it will return u.  However, if the axis of R is not aligned
 * along a, this function will return something sensible.
 */
Real AxisRotationMagnitude(const Matrix3& R,const Vector3& a);

/** @brief For a not-quite rotation matrix R, replaces it with the closest
 * rotation matrix. I.e., find an orthogonal matrix that minimizes the distance to
 * R.
 */
void NormalizeRotation(Matrix3& R);


/*@}*/

#endif
