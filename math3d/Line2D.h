#ifndef MATH3D_LINE2D_H
#define MATH3D_LINE2D_H

#include "Point.h"

namespace Math3D {

using namespace Math; 

struct AABB2D;
struct Segment2D;

/** @ingroup Math3D
 * @brief A 2D line class
 *
 * A redundant representation, using a point s on the line and a direction d.
 * Is parameterized by x = s+t*d for all real t.
 */
struct Line2D
{
  void setPoints(const Point2D& a, const Point2D& b);
  void setSegment(const Segment2D& s);
  void setTransformed(const Line2D&, const Matrix3& xform);
  Real closestPointParameter(const Point2D& in) const;
  Real closestPoint(const Point2D& in, Point2D& out) const;  ///<returns the parameter value of the point
  Real closestPoint(const Point2D& in, Point2D& out, Real tmin, Real tmax) const;  ///<tmin,tmax limit the range of the parameter t
  Real distance(const Point2D& pt) const;
  void eval(Real t, Point2D& out) const;
  Real orientation(const Point2D& p) const;
  bool isLeft(const Point2D& p) const { return orientation(p) > Zero; }
  bool isRight(const Point2D& p) const { return orientation(p) < Zero; }
  bool Read(File& f);
  bool Write(File& f) const;

  void getAABB(AABB2D&, Real tmin=-Inf, Real tmax=Inf) const;
  bool lineIntersects(const AABB2D&) const;
  bool rayIntersects(const AABB2D&) const;
  ///given bounds [tmin,tmax] of the line, returns the clipping min/max
  bool intersects(const AABB2D&, Real& tmin, Real& tmax) const;

  Point2D source;
  Vector2 direction;
};

} //namespace Math3D

#endif
