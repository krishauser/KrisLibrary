#ifndef MATH3D_PLANE3D_H
#define MATH3D_PLANE3D_H

#include "Point.h"

namespace Math3D {

struct Line3D;
struct Ray3D;
struct Segment3D;
struct AABB3D;

/** @brief A 3D plane class
 * @ingroup Math3D
 *
 * Represents plane with a normal and offset such that x on the plane 
 * satisfy dot(normal,x) = offset.
 */
struct Plane3D
{
  void setPointNormal(const Point3D& a, const Vector3& n);
  void setPointBases(const Point3D& a, const Vector3& b1, const Vector3& b2);
  void setPoints(const Point3D& a, const Point3D& b, const Point3D& c);
  void setTransformed(const Plane3D& pin, const RigidTransform& xform);
  void setTransformed(const Plane3D& pin, const Matrix4& xform);
  
  Real distance(const Point3D& v) const;
  void project(const Point3D& in, Point3D& out) const;		///<projects onto the plane
  void getBasis(Vector3& xb, Vector3& yb) const;		///<returns a basis of the plane
  
  bool intersectsSegment(const Segment3D&, Real* t);
  bool intersectsLine(const Line3D&, Real* t);
  bool intersectsRay(const Ray3D&, Real* t);
  bool intersects(const AABB3D&) const;
  bool intersectsInterior(const AABB3D&) const;
  void distanceLimits(const AABB3D&,Real& dmin,Real& dmax) const;   ///<calculates min/max bbox distances
  
  ///returns the dimension of the intersection of the 2 planes. <br>
  ///if 0, they don't intersect. <br>
  ///1, it's a line returned in l <br>
  ///2, the planes are identitcal
  int allIntersections(const Plane3D& p,Line3D& l) const;

  bool Read(File& f);
  bool Write(File& f) const;
  void Print(std::ostream& out) const;
  
  Vector3 normal;
  Real offset;
};

std::ostream& operator << (std::ostream& out,const Plane3D& p);
std::istream& operator >> (std::istream& out,Plane3D& p);

} //namespace Math

#endif
