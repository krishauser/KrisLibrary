#ifndef MATH3D_CYLINDER3D_H
#define MATH3D_CYLINDER3D_H

#include "Point.h"

namespace Math3D {

struct Circle3D;
struct AABB3D;
struct Line3D;
struct Segment3D;

/** @ingroup Math3D
 * @brief A 3D cylinder.
 *
 * The base is centered at center, extruding a circle of the
 * given radius along the axis, for the given height.
 */
struct Cylinder3D
{
  bool contains(const Point3D& pt) const;
  Real distance(const Point3D& pt) const;
  Real closestPoint(const Point3D& pt,Point3D& closest) const;  ///< returns distance to closest
  void setTransformed(const Cylinder3D& cyl,const RigidTransform& T);
  void getBase(Circle3D& c) const;
  void getCap(Circle3D& c) const;
  void getAABB(AABB3D&) const;

  /// Returns the entry/exit points if they intersect
  bool intersects(const Line3D& line,Real* tmin=NULL,Real* tmax=NULL) const;
  bool intersects(const Segment3D& seg,Real* tmin=NULL,Real* tmax=NULL) const;

  Point3D center;
  Vector3 axis;
  Real radius;
  Real height;
};

std::ostream& operator << (std::ostream& out,const Cylinder3D& b);
std::istream& operator >> (std::istream& in, Cylinder3D& b);

} //namespace Math3D

#endif
