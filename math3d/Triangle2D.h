#ifndef MATH3D_TRIANGLE2D_H
#define MATH3D_TRIANGLE2D_H

#include "Point.h"

namespace Math3D {

struct Plane2D;
struct Segment2D;
struct Line2D;
struct AABB2D;

/** @ingroup Math3D
 * @brief A 2D triangle class
 *
 * Represented by its vertices a,b,c.
 *
 * Barycentric coordinates (u,v,w) are such that 0 <= u,v,w <= 1
 * and u+v+w = 1.  They parameterize the triangle as x = u*a+v*b+w*c.
 *
 * "Plane" coordinates (p,q) are such that 0 <= p,q and p+q<= 1.
 * They parameterize the triangle as x = a + p*(b-a) + q*(c-a).
 * Barycentric coordinates (u,v,w) = (1-p-q,p,q).
 */
struct Triangle2D
{
  Triangle2D();
  Triangle2D(const Vector2& a,const Vector2& b,const Vector2& c);
  void set(const Vector2& a,const Vector2& b,const Vector2& c);
  void setTransformed(const Triangle2D& t, const RigidTransform2D& xform);
  void setTransformed(const Triangle2D& t, const Matrix3& xform);
  
  Real orientation() const;
  Real area() const;
  void getAABB(AABB2D&) const;
  
  Vector3 barycentricCoords(const Point2D& x) const;
  Point2D barycentricCoordsToPoint(const Vector3& bc) const;
  Vector2 planeCoords(const Point2D& x) const;
  Point2D planeCoordsToPoint(const Vector2& pc) const;
  
  Vector2 closestPointCoords(const Point2D& in) const;  ///<returns the plane-coords of the point
  Point2D closestPoint(const Point2D& in) const;
  bool contains(const Point2D& x) const;
  
  bool intersects(const Plane2D&) const;
  bool intersects(const Plane2D&, Segment2D& S) const;
  bool intersects(const Segment2D& s) const;
  bool intersects(const Triangle2D& t) const;
  //edges are (a,b) (b,c) (c,a), u gives the interpolation parameter 
  //of the plane intersection along each (or -1 if no intersection exists)
  //void edgeIntersections(const Plane2D&, Real u[3]) const;
  //void edgeIntersections(const Triangle2D&, Real u[3]) const;

  bool Read(File& f);
  bool Write(File& f) const;
 
  static Real orientation(const Point2D& a, const Point2D& b, const Point2D& c);
  static Real area(const Point2D& a, const Point2D& b, const Point2D& c);
  static Vector3 barycentricCoords(const Vector2& x, const Point2D& a, const Point2D& b, const Point2D& c);
  static Point2D barycentricCoordsToPoint(const Vector3& bc, const Point2D& a, const Point2D& b, const Point2D& c);
  static bool containsBarycentricCoords(const Vector3& bc);
  static Point2D planeCoordsToPoint(const Vector2& pc, const Point2D& a, const Point2D& b, const Point2D& c);
  static bool containsPlaneCoords(const Vector2& pc);
  
  Point2D a,b,c;
};

} //namespace Math3D

#endif
