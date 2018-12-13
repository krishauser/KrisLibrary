#ifndef MATH3D_MISC_H
#define MATH3D_MISC_H

#include "primitives.h"
#include <assert.h>

/** @file math3d/misc.h
 * @ingroup Math3D
 * @brief Miscellaneous utilities on 3D primitives.
 *
 * Contains predicate tests (IsFinite, IsInf, FuzzyZero), faster comparisons
 * (NormLess, DistanceLess) etc.  The comparisons save a square root.
 */

namespace Math3D {

  /** @addtogroup Math3D */
  /*@{*/

inline bool IsFinite(const Vector2& x) { return Math::IsFinite(x.x)&&Math::IsFinite(x.y); }
inline bool IsFinite(const Vector3& x) { return Math::IsFinite(x.x)&&Math::IsFinite(x.y)&&Math::IsFinite(x.z); }
inline bool IsFinite(const Vector4& x) { return Math::IsFinite(x.x)&&Math::IsFinite(x.y)&&Math::IsFinite(x.z)&&Math::IsFinite(x.w); }
inline bool IsFinite(const Matrix2& R)
{ 
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++) 
      if(!Math::IsFinite(R(i,j))) return false;
  return true;
}
inline bool IsFinite(const Matrix3& R)
{ 
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) 
      if(!Math::IsFinite(R(i,j))) return false;
  return true;
}
inline bool IsFinite(const Matrix4& R)
{ 
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++) 
      if(!Math::IsFinite(R(i,j))) return false;
  return true;
}

inline bool IsInf(const Vector2& x) { return Math::IsInf(x.x)||Math::IsInf(x.y); }
inline bool IsInf(const Vector3& x) { return Math::IsInf(x.x)||Math::IsInf(x.y)||Math::IsInf(x.z); }
inline bool IsInf(const Vector4& x) { return Math::IsInf(x.x)||Math::IsInf(x.y)||Math::IsInf(x.z)||Math::IsInf(x.w); }
inline bool IsInf(const Matrix2& R)
{ 
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++) 
      if(Math::IsInf(R(i,j))) return true;
  return false;
}
inline bool IsInf(const Matrix3& R)
{ 
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) 
      if(Math::IsInf(R(i,j))) return true;
  return false;
}
inline bool IsInf(const Matrix4& R)
{ 
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++) 
      if(Math::IsInf(R(i,j))) return true;
  return false;
}

inline bool FuzzyZero(const Vector2& x,Real tol=Epsilon) { return Math::FuzzyZero(x.x,tol)&&Math::FuzzyZero(x.y,tol); }
inline bool FuzzyZero(const Vector3& x,Real tol=Epsilon) { return Math::FuzzyZero(x.x,tol)&&Math::FuzzyZero(x.y,tol)&&Math::FuzzyZero(x.z,tol); }
inline bool FuzzyZero(const Vector4& x,Real tol=Epsilon) { return Math::FuzzyZero(x.x,tol)&&Math::FuzzyZero(x.y,tol)&&Math::FuzzyZero(x.z,tol)&&Math::FuzzyZero(x.w,tol); }

template<class V>
inline bool NormLess(const V& x, Real d) { return x.normSquared() < Sqr(d); }
template<class V>
inline bool NormLEQ(const V& x, Real d) { return x.normSquared() <= Sqr(d); }
template<class V>
inline bool NormGreater(const V& x, Real d) { return x.normSquared() > Sqr(d); }
template<class V>
inline bool NormGEQ(const V& x, Real d) { return x.normSquared() <= Sqr(d); }

template<class V>
inline bool DistanceLess(const V& x, const V& y, Real d) { return x.distanceSquared(y) < Sqr(d); }
template<class V>
inline bool DistanceLEQ(const V& x, const V& y, Real d) { return x.distanceSquared(y) <= Sqr(d); }
template<class V>
inline bool DistanceGreater(const V& x, const V& y, Real d) { return x.distanceSquared(y) > Sqr(d); }
template<class V>
inline bool DistanceGEQ(const V& x, const V& y, Real d) { return x.distanceSquared(y) <= Sqr(d); }

/** @brief Orientation test for 2d points
 * @return
 * >0 for p2 left of the line through p0 and p1 <br>
 * =0 for p2 on the line <br>
 * <0 for p2 right of the line
 */
inline Real Orient2D(const Vector2& p0, const Vector2& p1, const Vector2& p2)
{
  return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
}

inline Real DistanceSquared2D(Real x1,Real y1,Real x2,Real y2)
{
  return Sqr(x1-x2)+Sqr(y1-y2);
}

inline Real Distance2D(Real x1,Real y1,Real x2,Real y2)
{
  return Sqrt(DistanceSquared2D(x1,y1,x2,y2));
}

inline bool IsSymmetric(const Matrix2& R,Real tol=Epsilon)
{ 
  return FuzzyEquals(R(0,1),R(1,0),tol);
}

inline bool IsSymmetric(const Matrix3& R,Real tol=Epsilon)
{ 
  return FuzzyEquals(R(0,1),R(1,0),tol) && FuzzyEquals(R(0,2),R(2,0),tol) && FuzzyEquals(R(1,2),R(2,1),tol);
}

inline bool IsSymmetric(const Matrix4& R,Real tol=Epsilon)
{ 
  for(int i=0;i<4;i++)
    for(int j=0;j<i;j++) 
      if(!FuzzyEquals(R(i,j),R(j,i),tol)) return false;
  return true;
}

inline bool IsUpperTriangular(const Matrix2& R,Real tol=Epsilon)
{ 
  return Math::FuzzyZero(R(1,0),tol);
}

inline bool IsUpperTriangular(const Matrix3& R,Real tol=Epsilon)
{ 
  return Math::FuzzyZero(R(1,0),tol) && Math::FuzzyZero(R(2,0),tol) && Math::FuzzyZero(R(2,1),tol);
}

inline bool IsUpperTriangular(const Matrix4& R,Real tol=Epsilon)
{ 
  for(int i=0;i<4;i++)
    for(int j=0;j<i;j++) 
      if(!Math::FuzzyZero(R(i,j),tol)) return false;
  return true;
}

  /*@}*/

} //namespace Math3D

#endif
