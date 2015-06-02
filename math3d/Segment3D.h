#ifndef MATH3D_SEGMENT3D_H
#define MATH3D_SEGMENT3D_H

#include "Point.h"

namespace Math3D {

struct Line3D;
struct Plane3D;
struct AABB3D;

struct Segment3D
{
  void setTransformed(const Segment3D&, const Matrix4& xform);
  void getLine(Line3D& l) const;
  void getAABB(AABB3D& bb) const;
  Real distance(const Point3D& p) const;
  Real closestPointParameter(const Point3D& in) const;
  Real closestPoint(const Point3D& in, Point3D& out) const;  //returns the parameter value of the point
  void closestPoint(const Segment3D&,Real& t,Real& u) const;  //same comment as above
  Real distance(const Segment3D&) const;
  void eval(Real t, Point3D& out) const;
  bool intersects(const AABB3D&) const;
  bool intersects(const AABB3D&, Real& tmin, Real& tmax) const;
  Real distance(const AABB3D& bb) const;
  Real distance(const AABB3D& bb, Real& tclosest, Point3D& bbclosest) const;
  bool Read(File& f);
  bool Write(File& f) const;
  
  Point3D a,b;
};

std::ostream& operator << (std::ostream& out,const Segment3D& s);
std::istream& operator >> (std::istream& in,Segment3D& s);

} //namespace Math3D

#endif
