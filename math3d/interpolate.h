#ifndef MATH3D_INTERPOLATE_H
#define MATH3D_INTERPOLATE_H

#include "primitives.h"
#include <KrisLibrary/math/interpolate.h>

namespace Math3D {

inline void interpolate(const Vector2& a, const Vector2& b, Real u, Vector2& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

inline void interpolate(const Vector3& a, const Vector3& b, Real u, Vector3& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

inline void interpolate(const Vector4& a, const Vector4& b, Real u, Vector4& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

/*

inline void interpolate(const Matrix2& a, const Matrix2& b, Real u, Matrix2& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

inline void interpolate(const Matrix3& a, const Matrix3& b, Real u, Matrix3& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

inline void interpolate(const Matrix4& a, const Matrix4& b, Real u, Matrix4& x)
{
  x.mul(a,One-u);
  x.madd(b,u);
}

*/

void interpolateRotation(const Matrix2& a, const Matrix2& b, Real u, Matrix2& x);

void interpolateRotation(const Matrix3& a, const Matrix3& b, Real u, Matrix3& x);

inline void interpolate(const RigidTransform2D& a, const RigidTransform2D& b, Real u, RigidTransform2D& x)
{
  interpolate(a.t,b.t,u,x.t);
  interpolateRotation(a.R,b.R,u,x.R);
}

inline void interpolate(const RigidTransform& a, const RigidTransform& b, Real u, RigidTransform& x)
{
  interpolate(a.t,b.t,u,x.t);
  interpolateRotation(a.R,b.R,u,x.R);
}

//interpolateDirection assumes that a and b are normalized direction vectors
//and are not on opposite sides of the sphere
template <class VectorT>
void interpolateDirection(const VectorT& a,const VectorT& b,Real u,VectorT& out)
{
  Real d = dot(a,b);
  if(d == One) {	//axes are the same axis
    out.set(b);
    return;
  }
  else if(d == -One) {	//axes are opposing axis
    interpolate(a,b,u,out);
    return;
  }

  Real theta = Acos(d);
  Real sininv = Sin(theta);
  sininv = One/sininv;

  //out = (Sin((One-t)*theta)*sininv) * a +  (Sin(t*theta)*sininv) * b;
  Real a_coeff = Sin((One-u)*theta)*sininv;
  Real b_coeff = Sin(u*theta)*sininv;
  out.mul(a, a_coeff);
  out.madd(b, b_coeff);
}

///if segment is x = a + u*(b-a), returns the value u s.t. x=0
inline Real SegmentZeroCrossing(Real a,Real b)
{
  if(a == b) return Zero;
  return a/(a-b);
}

///if segment is x = a + u*(b-a), returns the value u s.t. x=x0
inline Real SegmentCrossing(Real a,Real b,Real x0)
{
  if(a == b) return Zero;
  return (a-x0)/(a-b);
}

}  //namespace Math3D

#endif
