#ifndef MATH3D_BOX2D_H
#define MATH3D_BOX2D_H

#include "LocalCoordinates2D.h"

namespace Math3D {

struct Segment2D;
struct Plane2D;
struct Triangle2D;
struct Circle2D;

/** @brief A 2D box
 * @ingroup Math3D
 *
 * The box is the unit square [0,1]^2 set in the scaled local coordinate
 * system.  That is, one corner is at the origin, and it has dimensions
 * [dims.x,dims.y] in the coordinates given by {xbasis,ybasis}. 
 */
struct Box2D : public ScaledLocalCoordinates2D
{
  void set(const AABB2D& bb);
  void setTransformed(const AABB2D& box,const RigidTransform2D& T);
  void setTransformed(const Box2D& box,const RigidTransform2D& T);
  void setTransformed(const AABB2D& box,Real angle,const Vector2& offset);
  void setTransformed(const Box2D& box,Real angle,const Vector2& offset);
  bool contains(const Point2D& pt) const;
  Real distance(const Point2D& pt) const;
  Real distance(const Point2D& pt,Point2D& closestPt) const;
  Real distanceSquared(const Point2D& pt,Point2D& closestPt) const;
  Real signedDistance(const Point2D& pt) const;
  Real signedDistance(const Point2D& pt,Point2D& closestPt) const;
  void getAABB(AABB2D& bb) const;
  bool intersects(const AABB2D& b) const;
  bool intersects(const Box2D& b) const;
  bool intersects(const Segment2D& s) const;
  bool intersects(const Line2D& l) const;
  bool intersects(const Triangle2D& t) const;
  bool intersects(const Circle2D& b) const;
};

} //namespace Math3D

#endif
