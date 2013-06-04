#ifndef MATH3D_RAY3D_H
#define MATH3D_RAY3D_H

#include "Line3D.h"

namespace Math3D {

struct Ray3D : public Line3D
{
  Real closestPoint(const Point3D& in, Point3D& out) const;
  Real distance(const Point3D& pt) const;
  bool intersects(const Line3D&, Real* t=NULL, Real* u=NULL, Real epsilon=0) const;	//t is the parameter of this ray, u is the line 
  bool intersects(const Ray3D&, Real* t=NULL, Real* u=NULL, Real epsilon=0) const;	//t is the parameter of this ray, u is the other ray
  void closestPoint(const Line3D&,Real& t,Real& u) const;  //same comment as above
  void closestPoint(const Ray3D&,Real& t,Real& u) const;  //same comment as above
};


} //namespace Math3D

#endif
