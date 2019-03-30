#ifndef MATH3D_ROTATION_H
#define MATH3D_ROTATION_H

#include <KrisLibrary/math/complex.h>
#include "primitives.h"

/** @file math3d/rotation.h
 * @ingroup Math3D
 * @brief 3D rotation representations.
 */

namespace Math3D {
  /** @addtogroup Math3D */
  /*@{*/

  using namespace Math;

class EulerAngleRotation;
class AngleAxisRotation;
class MomentRotation;
class QuaternionRotation;

/** @brief Euler angle 3D rotation parameterization.
 *
 * The euler angles a,b,c are stored respectively in the x,y,z fields.
 *
 * The resulting rotation matrix is Ru(a)Rv(b)Rw(c) where
 * u,v,w are rotation axes, specified in set/getMatrixUVW().
 */
class EulerAngleRotation : public Vector3
{
public:
  EulerAngleRotation();
  EulerAngleRotation(const EulerAngleRotation&);
  explicit EulerAngleRotation(const Vector3&);
  explicit EulerAngleRotation(Real a, Real b, Real c);

  //inline operator const Vector3&() const { return *this; }
  //inline operator Vector3&()  { return *this; }

  //inline void set(const EulerAngleRotation& r) { Vector3::set(r.x,r.y,r.z); }
  inline void setIdentity() { setZero(); }

  ///set/get MatrixUVW defines the resulting matrix as RU(a)RV(b)RW(c)
  ///where u,v,w are the axis names
  bool setMatrix(int u,int v,int w,const Matrix3&);
  bool setMatrixXYZ(const Matrix3&);
  bool setMatrixXZY(const Matrix3&);
  bool setMatrixYZX(const Matrix3&);
  bool setMatrixYXZ(const Matrix3&);
  bool setMatrixZXY(const Matrix3&);
  bool setMatrixZYX(const Matrix3&);

  void getMatrix(int u,int v,int w,Matrix3&) const;
  void getMatrixXYZ(Matrix3&) const;
  void getMatrixXZY(Matrix3&) const;
  void getMatrixYZY(Matrix3&) const;
  void getMatrixYXZ(Matrix3&) const;
  void getMatrixZXY(Matrix3&) const;
  void getMatrixZYX(Matrix3&) const;
};

/** @brief Angle-axis 3D rotation parameterization
 *
 * Represents a rotation by the unit axis of rotation and a ccw angle
 * of rotation (in radians).
 */
class AngleAxisRotation
{
public:
  AngleAxisRotation();
  AngleAxisRotation(const AngleAxisRotation&);
  explicit AngleAxisRotation(Real angle, const Vector3& axis);
  explicit AngleAxisRotation(const MomentRotation&);

  void set(const AngleAxisRotation&);
  void set(Real angle, const Vector3& axis);
  void setAxis(const Vector3& axis);
  void setIdentity();
  void transformPoint(const Vector3& in,Vector3& out) const;

  bool setMatrix(const Matrix3&);
  void getMatrix(Matrix3&) const;

  void setMoment(const MomentRotation&);
  void getMoment(MomentRotation&) const;

  Real angle;
  Vector3 axis;
};

/** @brief "Moment", a.k.a. exponential map, 3D rotation parameterization
 *
 * Represents a rotation R = e^[w] using the 3D vector w.  (Here [w]
 * denotes the cross product matrix of w).  
 *
 * |w| is the angle of rotation, and w/|w| is the axis.
 */
class MomentRotation : public Vector3
{
public:
  MomentRotation();
  MomentRotation(const MomentRotation&);
  explicit MomentRotation(Real x,Real y,Real z);
  explicit MomentRotation(const Vector3& v);
  explicit MomentRotation(const AngleAxisRotation&);

  //inline operator const Vector3&() const { return *this; }
  //inline operator Vector3&()  { return *this; }

  inline void set(const MomentRotation& r) { Vector3::set(r); }
  inline void set(const Vector3& v)  { Vector3::set(v); }
  inline void set(Real x, Real y, Real z)  { Vector3::set(x,y,z); }
  inline void setIdentity() { Vector3::setZero(); }
  void transformPoint(const Vector3& in,Vector3& out) const;

  bool setMatrix(const Matrix3&);
  void getMatrix(Matrix3&) const;

  void setAngleAxis(const AngleAxisRotation&);
  void getAngleAxis(AngleAxisRotation&) const;
};

/** @brief Quaternion, a.k.a. Euler parameter, 3D rotation parameterization
 *
 * Represents a rotation with a unit quaternion (w,x,y,z).
 * Convenient for rotation interpolation using the slerp() method.
 * Also allows smooth cubic interpolation using the SCerp() or 
 * bezier-curve style interpolation SBezier() functions below.
 */
class QuaternionRotation : public Quaternion
{
public:
  QuaternionRotation();
  QuaternionRotation(const QuaternionRotation&);
  QuaternionRotation(const Quaternion&);
  QuaternionRotation(Real w, Real x, Real y, Real z);

  //inline operator Quaternion&() { return *this; }
  //inline operator const Quaternion&() const { return *this; }
  void slerp(const Quaternion& a, const Quaternion& b, Real t);
  void mag(const Quaternion& a, Real t);
  void transform(const Vector3& a, Vector3& out) const;

  inline void set(const QuaternionRotation& q) { Quaternion::set(q); }
  inline void setIdentity() { Quaternion::set(One); }
  void setAngleAxis(const AngleAxisRotation&);
  void setMoment(const MomentRotation&);
  bool setMatrix(const Matrix3&);

  void getAngleAxis(AngleAxisRotation&) const;
  void getMoment(MomentRotation&) const;
  void getMatrix(Matrix3&) const;
};

void SetMatrixRotationZYX(Matrix3&, const Vector3&);		//euler
void SetMatrixRotationZYX(Matrix4&, const Vector3&);
void SetMatrixRotationVector(Matrix3&, const Vector3&);		//moment
void SetMatrixRotationVector(Matrix4&, const Vector3&);
void SetMatrixRotationVector(Matrix3&, const Vector3&, Real angle);  //angle axis
void SetMatrixRotationVector(Matrix4&, const Vector3&, Real angle);
void SetMatrixRotationQuaternion(Matrix3&, const Quaternion&);  //quaternion
void SetMatrixRotationQuaternion(Matrix4&, const Quaternion&);


/** @brief Linear quaternion rotation interpolation */
void SLerp(const Quaternion& q0,
	   const Quaternion& q1,
	   Quaternion& out,
	   Real t);

/** @brief Cubic quaternion rotation interpolation */
void SCerp(const Quaternion& q_1,
	   const Quaternion& q0,
	   const Quaternion& q1,
	   const Quaternion& q2,
	   Quaternion& out,
	   Real t);

/** @brief Bezier-style quaternion rotation interpolation
 *
 * The curve interpolates q0 and q1, using c0 and c1 as "control points"
 */
void SBezier(const Quaternion& q0,
	     const Quaternion& c0,
	     const Quaternion& c1,
	     const Quaternion& q1,
	     Quaternion& out,
	     Real t);

  /*@}*/
}

#endif
