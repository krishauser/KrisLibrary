#ifndef MATH3D_CIRCLE3D_H
#define MATH3D_CIRCLE3D_H

#include "Point.h"

namespace Math3D {

struct Line3D;
struct Plane3D;
struct Sphere3D;
struct AABB3D;

/** @ingroup Math3D
 * @brief A 2D circle in 3D space class
 *
 * Represented by a center, axis, and radius.
 * 
 * Most methods consider the circle as the solid disk.
 * Methods that use the circle boundary have the prefix "boundary".
 */
struct Circle3D
{
  bool setIntersection(const Sphere3D& s,const Plane3D& p);  ///<Returns false if they don't intersect
  Real distance(const Point3D& v) const;
  Real closestPoint(const Point3D& v,Point3D& closest) const;  ///<Returns distance to closest
  Real boundaryDistance(const Point3D& v) const;
  bool intersects(const Circle3D& c) const;
  bool intersects(const Sphere3D& s) const;
  bool intersects(const Line3D& l,Real* t=NULL) const;
  bool intersects(const Plane3D& p) const;
  bool boundaryIntersects(const Sphere3D& s) const;
  void getPlane(Plane3D& p) const;
  void getAABB(AABB3D&) const;

  bool Read(File& f);
  bool Write(File& f) const;

  Point3D center;
  Vector3 axis;
  Real radius;
};

} //namespace Math3D

#endif
