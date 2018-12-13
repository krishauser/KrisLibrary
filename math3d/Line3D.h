#ifndef MATH3D_LINE3D_H
#define MATH3D_LINE3D_H

#include "AABB3D.h"

namespace Math3D {

using namespace Math; 

struct Segment3D;

/** @ingroup Math3D
 * @brief A 3D line class
 *
 * A redundant representation, using a point s on the line and a direction d.
 * Is parameterized by x = s+t*d for all real t.
 */
struct Line3D
{
  void setPoints(const Point3D& a, const Point3D& b);
  void setSegment(const Segment3D& s);
  void setTransformed(const Line3D&, const Matrix4& xform);
  Real closestPointParameter(const Point3D& in) const;
  Real closestPoint(const Point3D& in, Point3D& out) const;  ///<returns the parameter value of the point
  Real closestPoint(const Point3D& in, Point3D& out, Real tmin, Real tmax) const;  //tmin,tmax limit the range of the parameter t
  Real distance(const Point3D& pt) const;
  void eval(Real t, Point3D& out) const;
  bool intersects(const Line3D&, Real* t=NULL, Real* u=NULL, Real epsilon=0) const;	///<t is the parameter of this line, u is the other line 
  void closestPoint(const Line3D&,Real& t,Real& u) const;  ///<same comment as above
  void getAABB(AABB3D&, Real tmin=-Inf, Real tmax=Inf) const;
  bool lineIntersects(const AABB3D&) const;
  bool rayIntersects(const AABB3D&) const;
  ///given bounds [tmin,tmax] of the line, returns the clipping min/max
  bool intersects(const AABB3D&, Real& tmin, Real& tmax) const;
  Real distance(const AABB3D& bb) const;
  Real distance(const AABB3D& bb, Real& tclosest, Vector3& bbclosest) const;
  bool Read(File& f);
  bool Write(File& f) const;

  Point3D source;
  Vector3 direction;
};

std::ostream& operator << (std::ostream& out,const Line3D& line);
std::istream& operator >> (std::istream& in,Line3D& line);

} //namespace Math3D

#endif
