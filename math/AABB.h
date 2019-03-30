#ifndef MATH_AABB_H
#define MATH_AABB_H

#include "vector.h"
#include <KrisLibrary/errors.h>

/** @file math/AABB.h
 * @ingroup Math
 * @brief Functions defining axis-aligned bounding boxes (AABBs).
 * 
 * An AABB consists of two Vectors (bmin,bmax), such that a Vector x
 * is inside the AABB if \f$ bmin_i \leq x_i \leq bmax_i \f$ for all i.
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

/// Clamps x to be contained in the AABB (bmin,bmax)
inline void AABBClamp(Vector& x,const Vector& bmin,const Vector& bmax,Real d=Zero)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  for(int i=0;i<x.n;i++)
    x(i) = Clamp(x(i),bmin(i)+d,bmax(i)-d);
}

/// Returns the minimum distance from x to the boundaries of the
/// AABB (bmin,bmax).  Also returns the element index that contains the
/// minimum margin.
inline Real AABBMargin(const Vector& x,const Vector& bmin,const Vector& bmax,int& index)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  Real margin=Inf;
  index=-1;
  for(int i=0;i<x.n;i++) {
    if(x(i)-bmin(i) < margin) {
      margin = x(i)-bmin(i);
      index=i;
    }
    if(bmax(i)-x(i) < margin) {
      margin = bmax(i)-x(i);
      index=i;
    } 
  }
  return margin;
}

/// Returns the minimum distance from x to the boundaries of the
/// AABB (bmin,bmax)
inline Real AABBMargin(const Vector& x,const Vector& bmin,const Vector& bmax)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  Real margin=Inf;
  for(int i=0;i<x.n;i++) {
    margin = Min(margin,x(i)-bmin(i));
    margin = Min(margin,bmax(i)-x(i));
  }
  return margin;
}

/// Returns true if the AABB (bmin,bmax) contains x
inline bool AABBContains(const Vector& x,const Vector& bmin,const Vector& bmax,Real d=Zero)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  for(int i=0;i<x.n;i++)
    if(x(i) < bmin(i)+d || x(i) > bmax(i)+d) return false;
  return true;
}

/// Grows an AABB to contain the vector x
inline void AABBGrow(Vector& bmin,Vector& bmax,const Vector& x)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  for(int i=0;i<x.n;i++) {
    if(x[i] < bmin[i]) bmin[i] = x[i];
    if(x[i] > bmax[i]) bmax[i] = x[i];
  }
}


/// Returns the signed distance between the AABB and x.  This is negative
/// if x is in the AABB, positive if it is outside.
inline Real AABBDistance(const Vector& x,const Vector& bmin,const Vector& bmax)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  bool inside = true;
  Real distance2 = 0;
  Real penetration = Inf;
  for(int i=0;i<x.n;i++) {
    if(x[i] <= bmin[i] && x[i] >= bmax[i]) {
      penetration = Min(penetration,x[i]-bmin[i]);
      penetration = Min(penetration,bmax[i]-x[i]);
    }
    else {
      inside = false;
      if(x[i] < bmin[i])
        distance2 += Sqr(bmin[i] - x[i]);
      if(x[i] > bmax[i])
        distance2 += Sqr(bmax[i] - x[i]);
    }    
  }
  if(inside) return -penetration;
  else return Sqrt(distance2);
}

/// For a point inside the AABB (bmin,bmax), limits the desired step t
/// x = x0+t*dx such that x is inside the AABB.  Returns the index responsible
/// for this limit
int AABBLineSearch(const Vector& x0,const Vector& dx,const Vector& bmin,const Vector& bmax,Real& t);

/// Clips the line segment x=x0+u*dx, with t in [u0,u1]
/// to the AABB (bmin,bmax).  Returns false if there is no intersection.
bool AABBClipLine(const Vector& x0,const Vector& dx,
		  const Vector& bmin,const Vector& bmax,
		  Real& u0,Real& u1);

/*@}*/
} //namespace Math

#endif

