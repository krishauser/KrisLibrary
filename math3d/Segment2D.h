#ifndef MATH3D_SEGMENT2D_H
#define MATH3D_SEGMENT2D_H

#include "Point.h"

namespace Math3D {

struct Line2D;
struct AABB2D;

/** @ingroup Math3D
 * @brief A 2D segment class
 *
 * Represented by the endpoints a and b.  Is parameterized by t in [0,1]
 * going from a->b as t approaches 1.
 */
struct Segment2D
{
  void setTransformed(const Segment2D&, const Matrix3& xform);
  void getLine(Line2D& l) const;
  Real closestPointParameter(const Point2D& in) const;
  Real closestPoint(const Point2D& in, Point2D& out) const;  //returns the parameter value of the point
  Real distance(const Point2D& pt) const;
  void eval(Real t, Point2D& out) const;
  Real orientation(const Point2D& p) const;  ///< >0 for left, <0 for right
  bool isLeft(const Point2D& p) const;  ///< p left of A->B
  bool isRight(const Point2D& p) const;  ///< p right of A->B
  bool intersects(const Segment2D& S) const;
  bool intersects(const Vector2& a,const Vector2& b) const;
  bool intersects(const Segment2D& S,Vector2& p) const;
  bool intersects(const Vector2& a,const Vector2& b,Vector2& p) const;
  bool Read(File& f);
  bool Write(File& f) const;

  void getAABB(AABB2D&) const;
  bool intersects(const AABB2D&) const;
  ///given bounds [tmin,tmax] of the segment, returns the clipping min/max
  bool intersects(const AABB2D&, Real& tmin, Real& tmax) const;
  Real distance(const AABB2D&) const;
  Real distance(const AABB2D&, Real& tmin) const;

  ///orientation of x vs a,b
  static Real orientation(const Point2D& a,const Point2D& b,const Point2D& x);

  Point2D a,b;
};

} //namespace Math3D

#endif
