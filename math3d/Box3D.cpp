#include "Box3D.h"
#include "Segment3D.h"
#include "Plane3D.h"
#include "Triangle3D.h"
#include "Sphere3D.h"
#include "Line3D.h"
#include <iostream>
#include <geometry/PQP/src/OBB_Disjoint.h>
using namespace std;

namespace Math3D {

Vector3 Box3D::center() const
{
  return origin + 0.5*(xbasis*dims.x+ybasis*dims.y+zbasis*dims.z);
}

void Box3D::setCenter(const Vector3& c)
{
  origin = c - 0.5*(xbasis*dims.x+ybasis*dims.y+zbasis*dims.z);
}

void Box3D::set(const AABB3D& bb)
{
  origin = bb.bmin;
  xbasis.set(1,0,0);
  ybasis.set(0,1,0);
  zbasis.set(0,0,1);
  dims = bb.bmax-bb.bmin;
}

void Box3D::setTransformed(const AABB3D& box,const RigidTransform& T)
{
  T.R.get(xbasis,ybasis,zbasis);
  origin = T.t + T.R*box.bmin;
  dims = box.bmax-box.bmin;
}

void Box3D::setTransformed(const Box3D& box,const RigidTransform& T)
{
  origin = T*box.origin;
  xbasis = T.R*box.xbasis;
  ybasis = T.R*box.ybasis;
  zbasis = T.R*box.zbasis;
  dims = box.dims;
}

bool Box3D::contains(const Point3D& pt) const
{
	Point3D out;
	toLocal(pt,out);
	return 0<=out.x&&out.x<=dims.x &&
		0<=out.y&&out.y<=dims.y &&
		0<=out.z&&out.z<=dims.z;
}

Real Box3D::distance(const Point3D& pt) const
{
  Point3D temp;
  return distance(pt,temp);
}

Real Box3D::distance(const Point3D& pt,Point3D& out) const
{
  return Sqrt(distanceSquared(pt,out));
}

Real Box3D::distanceSquared(const Point3D& pt,Point3D& out) const
{
  Point3D loc;
  toLocal(pt, loc);
  out = loc;
  if(loc.x < 0) out.x = 0;
  if(loc.y < 0) out.y = 0;
  if(loc.z < 0) out.z = 0;
  if(loc.x > dims.x) out.x = dims.x;
  if(loc.y > dims.y) out.y = dims.y;
  if(loc.z > dims.z) out.z = dims.z;
  Real norm2 = loc.distanceSquared(out);
  loc = out;
  fromLocal(loc,out);
  return norm2;
}

Real Box3D::signedDistance(const Point3D& pt) const
{
  Point3D closest;
  return signedDistance(pt,closest);
}

Real Box3D::signedDistance(const Point3D& pt,Point3D& out) const
{
  Point3D loc;
  toLocal(pt, loc);
  out = loc;
  bool inside = true;
  Real dmin = Inf;
  if(loc.x < 0) { out.x = 0; inside=false; }
  else dmin = Min(dmin,loc.x);
  if(loc.y < 0) { out.y = 0; inside=false; }
  else dmin = Min(dmin,loc.y);
  if(loc.z < 0) { out.z = 0; inside=false; }
  else dmin = Min(dmin,loc.z);
  if(loc.x > dims.x) { out.x = dims.x; inside=false; }
  else dmin = Min(dmin,dims.x-loc.x);
  if(loc.y > dims.y) { out.y = dims.y; inside=false; }
  else dmin = Min(dmin,dims.y-loc.y);
  if(loc.z > dims.z) { out.z = dims.z; inside=false; }
  else dmin = Min(dmin,dims.z-loc.z);
  Real norm = loc.distance(out);
  loc = out;
  fromLocal(loc,out);
  return norm;
}

bool Box3D::intersects(const AABB3D& b) const
{
  Box3D bb;
  bb.set(b);
  return intersects(bb);
}

bool Box3D::intersects(const Box3D& b) const
{
  Vector3 c = center();
  Vector3 bc = b.center();
  Vector3 bclocal;
  toLocalReorient(bc-c,bclocal);
  Vector3 bxlocal,bylocal,bzlocal;
  toLocalReorient(b.xbasis,bxlocal);
  toLocalReorient(b.ybasis,bylocal);
  toLocalReorient(b.zbasis,bzlocal);
  Vector3 halfdims = dims*0.5;
  Vector3 bhalfdims = b.dims*0.5;
  PQP_REAL B[3][3],T[3],AD[3],BD[3];
  bxlocal.get(B[0]);
  bylocal.get(B[1]);
  bzlocal.get(B[2]);
  bclocal.get(T);
  halfdims.get(AD);
  bhalfdims.get(BD);
  return obb_disjoint(B,T,AD,BD) == 0;
}

bool Box3D::intersectsApprox(const Box3D& b) const
{
	Box3D temp;
	AABB3D aabb_temp, aabb_temp2;
	//make temp localized
	temp.dims = b.dims;
	toLocal(b.origin, temp.origin);
	toLocalReorient(b.xbasis, temp.xbasis);
	toLocalReorient(b.ybasis, temp.ybasis);
	toLocalReorient(b.zbasis, temp.zbasis);
	temp.getAABB(aabb_temp);
	aabb_temp2.bmin.setZero();
	aabb_temp2.bmax = dims;
	if(!aabb_temp2.intersects(aabb_temp))
		return false;

	temp.dims = dims;
	b.toLocal(origin, temp.origin);
	b.toLocalReorient(xbasis, temp.xbasis);
	b.toLocalReorient(ybasis, temp.ybasis);
	b.toLocalReorient(zbasis, temp.zbasis);
	temp.getAABB(aabb_temp);
	aabb_temp2.bmax = b.dims;
	if(!aabb_temp2.intersects(aabb_temp))
		return false;
	return true;
}

void Box3D::getAABB(AABB3D& bb) const
{
	Vector3 x(dims.x*xbasis),y(dims.y*ybasis),z(dims.z*zbasis);
	Vector3 c=origin + 0.5*(x+y+z);

	Vector3 d;
	d.setZero();
	for(int i=0; i<3; i++)
	{
	  d[i] = Abs(x[i]) + Abs(y[i]) + Abs(z[i]);
	}

	bb.bmin.sub(c,d*0.5);
	bb.bmax.add(c,d*0.5);
}

bool Box3D::intersects(const Segment3D& s) const
{
  Segment3D sloc;
  toLocal(s,sloc);
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return sloc.intersects(bbloc);
}

bool Box3D::intersects(const Line3D& l) const
{
  Line3D lloc;
  toLocal(l,lloc);
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return lloc.lineIntersects(bbloc);
}

bool Box3D::intersects(const Triangle3D& t) const
{
  Triangle3D tloc;
  toLocal(t.a,tloc.a);
  toLocal(t.b,tloc.b);
  toLocal(t.c,tloc.c);
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return tloc.intersects(bbloc);
}

bool Box3D::intersects(const Sphere3D& s) const
{
  Sphere3D sloc;
  toLocal(s.center,sloc.center);
  sloc.radius = s.radius;
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return sloc.intersects(bbloc);
}


std::ostream& operator << (std::ostream& out,const Box3D& b)
{
  out<<b.origin<<"  "<<b.xbasis<<"  "<<b.ybasis<<"  "<<b.zbasis<<"  "<<b.dims;
  return out;
}


std::istream& operator >> (std::istream& in, Box3D& b)
{
  in>>b.origin>>b.xbasis>>b.ybasis>>b.zbasis>>b.dims;
  return in;
}

} //namespace Math3D
