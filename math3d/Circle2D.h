#ifndef MATH3D_CIRCLE2D_H
#define MATH3D_CIRCLE2D_H

#include "Plane2D.h"

namespace Math3D {

/** @ingroup Math3D
 * @brief A 2D circle class
 *
 * Represented by a center and a radius.
 * 
 * Most methods consider the circle as the solid disk.
 * Methods that use the circle boundary have the prefix "boundary".
 */
struct Circle2D
{
  Real distance(const Point2D& v) const;
  bool contains(const Point2D& v) const;
  bool contains(const Circle2D& s) const;
  bool withinDistance(const Point2D& v, Real dist) const;
  bool boundaryWithinDistance(const Point2D& v, Real dist) const;
  bool intersects(const Line2D& l, Real* t1=NULL, Real* t2=NULL) const;
  bool intersects(const Segment2D& s) const;
  bool intersects(const Plane2D& p, Segment2D& S) const;
  bool intersects(const Circle2D& c) const;
  ///returns true if the boundary of this intersects the interior of c
  bool boundaryIntersects(const Circle2D& c) const;
  ///returns true if the boundary of this intersects the boundary of c
  bool boundaryIntersectsBoundary(const Circle2D& c) const;
  bool Read(File& f);
  bool Write(File& f) const;

  void getAABB(AABB2D&) const;
  bool intersects(const AABB2D&) const;

  static bool disksIntersect(const Point2D& ca,Real ra,const Point2D& cb,Real rb);
  static bool diskCircleIntersect(const Point2D& ca,Real ra,const Point2D& cb,Real rb);
  static bool circlesIntersect(const Point2D& ca,Real ra,const Point2D& cb,Real rb);

  Point2D center;
  Real radius;
};

} //namespace Math3D

#endif
