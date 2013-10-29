#ifndef MATH3D_ELLIPSOID_H
#define MATH3D_ELLIPSOID_H

#include "LocalCoordinates3D.h"
#include "Line3D.h"

namespace Math3D {

/** @brief A 3D ellipsoid
 * @ingroup Math3D
 *
 * The ellipsoid is the centered unit sphere [-1,1]^3 set in the scaled local
 * coordinate system.  That is, the center is at the origin, and its
 * major axes are {xbasis,ybasis,zbasis} with radii {dims.x,dims.y,dims.z}. 
 */
struct Ellipsoid3D : public ScaledLocalCoordinates3D
{
  bool contains(const Point3D& pt) const;
  bool intersects(const Line3D& l, Real* t1=NULL, Real* t2=NULL) const;
  bool intersects(const Segment3D& s, Real* t1=NULL, Real* t2=NULL) const;
  void getAABB(AABB3D& bb) const;
};

std::ostream& operator << (std::ostream& out,const Ellipsoid3D& b);
std::istream& operator >> (std::istream& in, Ellipsoid3D& b);

} //namespace Math3D

#endif

