#ifndef MATH3D_RANDOM_H
#define MATH3D_RANDOM_H

#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/math/complex.h>
#include "primitives.h"

/** @file math3d/random.h
 * @ingroup Math3D
 * @brief Sampling routines for 3D primitives
 */

namespace Math3D {

  /** @addtogroup Math3D */
  /*@{*/

inline void SampleCircle(Real r,Vector2& v)
{
  Math::SampleCircle(r,v.x,v.y);
}

inline void SampleDisk(Real r,Vector2& v)
{
  Math::SampleDisk(r,v.x,v.y);
}

inline void SampleSphere(Real r,Vector3& v)
{
  Math::SampleSphere(r,v.x,v.y,v.z);
}

inline void SampleBall(Real r,Vector3& v)
{
  Math::SampleBall(r,v.x,v.y,v.z);
}

inline void SampleSquare(Real d,Vector2& v)
{
  v.set(Rand(-d,d),Rand(-d,d));
}

inline void SampleCube(Real d,Vector3& v)
{
  v.set(Rand(-d,d),Rand(-d,d),Rand(-d,d));
}

inline void SampleHyperCube(Real d,Vector4& v)
{
  v.set(Rand(-d,d),Rand(-d,d),Rand(-d,d),Rand(-d,d));
}

inline void SampleAABB(const Vector2& bmin,const Vector2& bmax,Vector2& v)
{
  v.set(Rand(bmin.x,bmax.x),Rand(bmin.y,bmax.y));
}

inline void SampleAABB(const Vector3& bmin,const Vector3& bmax,Vector3& v)
{
  v.set(Rand(bmin.x,bmax.x),Rand(bmin.y,bmax.y),Rand(bmin.z,bmax.z));
}

inline void SampleAABB(const Vector4& bmin,const Vector4& bmax,Vector4& v)
{
  v.set(Rand(bmin.x,bmax.x),Rand(bmin.y,bmax.y),Rand(bmin.z,bmax.z),Rand(bmin.w,bmax.w));
}

void RandRotation(Quaternion& q);

//convenience functions, may be slightly slower than the above
inline Vector2 SampleCircle(Real r) { Vector2 v; SampleCircle(r,v); return v; }
inline Vector2 SampleDisk(Real r) { Vector2 v; SampleDisk(r,v); return v; }
inline Vector3 SampleSphere(Real r) { Vector3 v; SampleSphere(r,v); return v; }
inline Vector3 SampleBall(Real r) { Vector3 v; SampleBall(r,v); return v; }
inline Vector2 SampleSquare(Real d) { Vector2 v; SampleSquare(d,v); return v; }
inline Vector3 SampleCube(Real d) { Vector3 v; SampleCube(d,v); return v; }
inline Vector4 SampleHyperCube(Real d) { Vector4 v; SampleHyperCube(d,v); return v; }
inline Vector2 SampleAABB(const Vector2& bmin,const Vector2& bmax) { Vector2 v; SampleAABB(bmin,bmax,v); return v; }
inline Vector3 SampleAABB(const Vector3& bmin,const Vector3& bmax) { Vector3 v; SampleAABB(bmin,bmax,v); return v; }
inline Vector4 SampleAABB(const Vector4& bmin,const Vector4& bmax) { Vector4 v; SampleAABB(bmin,bmax,v); return v; }
inline Quaternion RandRotation() { Quaternion q; RandRotation(q); return q; }

  /*@}*/

} //namespace Math3D

#endif
