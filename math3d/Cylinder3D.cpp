#include "Cylinder3D.h"
#include "geometry3d.h"
#include <math/misc.h>
#include <assert.h>
using namespace std;

namespace Math3D {


bool Cylinder3D::contains(const Point3D& pt) const
{
  Real h = dot(pt-center,axis);
  if(h < 0 || h > height) return false;
  Real r = (pt - center - axis*h).length();
  if(r > radius) return false;
  return true;
}

Real Cylinder3D::distance(const Point3D& pt) const
{
  Point3D closest;
  return closestPoint(pt,closest);
}

Real Cylinder3D::closestPoint(const Point3D& pt,Point3D& closest) const
{
  Real h = dot(pt-center,axis);
  if(h < 0) {
    Circle3D c;
    getBase(c);
    return c.closestPoint(pt,closest);
  }
  else if (h > height) {
    Circle3D c;
    getCap(c);
    return c.closestPoint(pt,closest);
  }
  else {
    Vector3 vperp = (pt-center) - h*axis;
    Real rv = vperp.norm();
    if(rv <= radius) { 
      //inside cylinder
      closest = pt;
      return 0;
    }
    else {
      closest = center + axis*h + vperp*(radius/rv);
      return rv - radius;
    }
  }
}

void Cylinder3D::setTransformed(const Cylinder3D& cyl,const RigidTransform& T)
{
  center = T*cyl.center;
  axis = T.R*cyl.axis;
  radius = cyl.radius;
  height = cyl.height;
}

void Cylinder3D::getBase(Circle3D& c) const
{
  c.center=center;
  c.axis=axis;
  c.radius=radius;
}

void Cylinder3D::getCap(Circle3D& c) const
{
  c.center=center; c.center.madd(axis,height);
  c.axis=axis;
  c.radius=radius;
}

void Cylinder3D::getAABB(AABB3D& aabb) const
{
  Circle3D c;
  getBase(c);
  c.getAABB(aabb);
  if(axis.x > 0) aabb.bmax.x += axis.x*height;
  else aabb.bmin.x -= axis.x*height;
  if(axis.y > 0) aabb.bmax.y += axis.y*height;
  else aabb.bmin.y -= axis.y*height;
  if(axis.z > 0) aabb.bmax.z += axis.z*height;
  else aabb.bmin.z -= axis.z*height;
}

bool Cylinder3D::intersects(const Segment3D& s,Real* tmin,Real* tmax) const
{
  Line3D l;
  l.source = s.a;
  l.direction = s.b-s.a;
  Real u,v;
  if(!intersects(l,&u,&v)) return false;
  if(v < 0.0) return false;
  if(u > 1.0) return false;
  u = Max(u,0.0);
  v = Min(v,1.0);
  if(tmin) *tmin = u;
  if(tmax) *tmax = v;
  return true;
}

bool Cylinder3D::intersects(const Line3D& line,Real* tmin,Real* tmax) const
{
  Real axistmin,axistmax;

  //quick reject - infinite cylinder
  Vector3 src=line.source-center;
  const Vector3& dir=line.direction;
  assert(FuzzyEquals(axis.normSquared(),One));
  //quadratic equation
  Real a,b,c;
  Real dv,sv;
  dv = dir.dot(axis);
  sv = src.dot(axis);
  a=dir.normSquared()-Sqr(dv);
  b=Two*(dir.dot(src)-sv*dv);
  c=src.normSquared()-Sqr(sv)-Sqr(radius);
  int nroots=quadratic(a,b,c,axistmin,axistmax);
  //TODO: if the line is contained within the cylinder, ignore this
  if(nroots == 0) return false;
  else if(nroots == 1) axistmax=axistmin;
  else if(nroots == 2) {
    if(axistmin > axistmax) std::swap(axistmin,axistmax);
  }
  else if(nroots == -1) return false;
  else return false;  //what case is this?

  //projection of intersection pts on the cyl axis
  Real axisumin,axisumax;
  Vector3 temp;
  line.eval(axistmin,temp);  axisumin = axis.dot(temp-center);
  line.eval(axistmax,temp);  axisumax = axis.dot(temp-center);

  //now check the caps
  Real tc;
  Circle3D cir;
  if(axisumin < 0) {   //hits a cap first
    if(dv > 0) //line points along axis
      getBase(cir);
    else  //line points against axis
      getCap(cir);
    if(!cir.intersects(line,&tc)) return false;
    axistmin = tc;
  }
  if(axisumin > height) {  //hits a cap last
    if(dv > 0) //line points along axis
      getCap(cir);
    else  //line points against axis
      getBase(cir);
    if(!cir.intersects(line,&tc)) return false;
    axistmax = tc;
  }
  //if(axistmin > axistmax) return false;
  if(tmin) *tmin=axistmin;
  if(tmax) *tmax=axistmax;
  return true;
}

std::ostream& operator << (std::ostream& out,const Cylinder3D& b)
{
  out<<b.center<<"  "<<b.axis<<"  "<<b.radius<<"  "<<b.height;
  return out;
}

std::istream& operator >> (std::istream& in, Cylinder3D& b)
{
  in>>b.center>>b.axis>>b.radius>>b.height;
  return in;
}


} //namespace Math3D
