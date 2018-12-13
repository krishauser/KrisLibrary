#ifndef MATH3D_TRIANGLE3D_H
#define MATH3D_TRIANGLE3D_H

#include "Point.h"

namespace Math3D {

struct Segment3D;
struct Plane3D;
struct Line3D;
struct AABB3D;
struct Ray3D;

/** @ingroup Math3D
 * @brief A 3D triangle class
 *
 * Represented by its vertices a,b,c in ccw order.
 *
 * Barycentric coordinates (u,v,w) are such that 0 <= u,v,w <= 1
 * and u+v+w = 1.  They parameterize the triangle as x = u*a+v*b+w*c.
 *
 * "Plane" coordinates (p,q) are such that 0 <= p,q and p+q<= 1.
 * They parameterize the triangle as x = a + p*(b-a) + q*(c-a).
 * Barycentric coordinates (u,v,w) = (1-p-q,p,q).
 */
struct Triangle3D
{
  Triangle3D();
  Triangle3D(const Vector3& a,const Vector3& b,const Vector3& c);
  void set(const Vector3& a,const Vector3& b,const Vector3& c);
  void setTransformed(const Triangle3D& t, const Matrix4& xform);
  const Vector3& vertex(int v) const;
  Segment3D edge(int e) const;
  Vector3 normal() const;
  Real area() const;
  void getPlane(Plane3D& p) const;
  
  Vector3 barycentricCoords(const Point3D& x) const;
  Point3D barycentricCoordsToPoint(const Vector3& bc) const;
  Vector2 planeCoords(const Point3D& x) const;
  Point3D planeCoordsToPoint(const Vector2& pc) const;
  
  Vector2 closestPointCoords(const Point3D& in) const;  ///<returns the plane-coords of the point
  Point3D closestPoint(const Point3D& in) const;
  bool contains(const Point3D& x) const;
  
  bool rayIntersects(const Ray3D& ray, Real *t, Real *u, Real *v) const;
  bool rayIntersectsBackfaceCull(const Ray3D& ray, Real *t, Real *u, Real *v) const;
  bool intersects(const Segment3D& s, Real *t=NULL, Real *u=NULL, Real *v=NULL) const;
  bool intersects(const Plane3D&) const;
  bool intersects(const Plane3D&, Segment3D& S) const;
  bool intersects(const Triangle3D&) const;
  bool intersects(const Triangle3D&, Segment3D& S) const;
  ///edges are (a,b) (b,c) (c,a), u gives the interpolation parameter 
  ///of the plane intersection along each (or -1 if no intersection exists)
  void edgeIntersections(const Plane3D&, Real u[3]) const;
  void edgeIntersections(const Triangle3D&, Real u[3]) const;

  void getAABB(AABB3D&) const;
  bool intersects(const AABB3D&) const;

  Real distance(const Triangle3D& other,Vector3& P,Vector3& Q) const;

  bool Read(File& f);
  bool Write(File& f) const;

  static Real area(const Point3D& a, const Point3D& b, const Point3D& c);
  static Vector3 normal(const Point3D& a, const Point3D& b, const Point3D& c);
  static Vector3 barycentricCoords(const Vector3& x, const Point3D& a, const Point3D& b, const Point3D& c);
  static Point3D barycentricCoordsToPoint(const Vector3& bc, const Point3D& a, const Point3D& b, const Point3D& c);
  static bool containsBarycentricCoords(const Vector3& bc);
  static Point3D planeCoordsToPoint(const Vector2& pc, const Point3D& a, const Point3D& b, const Point3D& c);
  static bool containsPlaneCoords(const Vector2& pc);
  static bool rayIntersects(const Ray3D& ray, const Point3D& a, const Point3D& b, const Point3D& c,
		   Real *t, Real *u, Real *v);
  static bool rayIntersectsBackfaceCull(const Ray3D& ray, const Point3D& a, const Point3D& b, const Point3D& c,
		   Real *t, Real *u, Real *v);

  Point3D a,b,c;
};

std::ostream& operator << (std::ostream& out,const Triangle3D& tri);
std::istream& operator >> (std::istream& in,Triangle3D& tri);

} //namespace Math3D

#endif
