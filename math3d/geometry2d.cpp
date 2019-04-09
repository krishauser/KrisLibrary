#include "geometry2d.h"
#include "clip.h"
#include "misc.h"
#include <KrisLibrary/math/misc.h>
#include <KrisLibrary/math/Interval.h>
#include <KrisLibrary/File.h>
#include "LinearlyDependent.h"
#include "interpolate.h"
using namespace std;

namespace Math3D {


void Box2D::set(const AABB2D& bb)
{
  origin = bb.bmin;
  xbasis.set(1,0);
  ybasis.set(0,1);
  dims = bb.bmax-bb.bmin;
}

void Box2D::setTransformed(const AABB2D& box,Real angle,const Vector2& t)
{
  RigidTransform2D T;
  T.R.setRotate(angle);
  T.t = t;
  setTransformed(box,T);
}

void Box2D::setTransformed(const AABB2D& box,const RigidTransform2D& T)
{
  T.R.get(xbasis,ybasis);
  origin = T*box.bmin;
  dims = box.bmax-box.bmin;
}

void Box2D::setTransformed(const Box2D& box,Real angle,const Vector2& t)
{
  RigidTransform2D T;
  T.R.setRotate(angle);
  T.t = t;
  setTransformed(box,T);
}

void Box2D::setTransformed(const Box2D& box,const RigidTransform2D& T)
{
  origin = T*box.origin;
  xbasis = T.R*box.xbasis;
  ybasis = T.R*box.ybasis;
  dims = box.dims;
}

bool Box2D::contains(const Point2D& pt) const
{
	Point2D out;
	toLocal(pt,out);
	return 0<=out.x&&out.x<=dims.x &&
	  0<=out.y&&out.y<=dims.y;
}

Real Box2D::distance(const Point2D& pt) const
{
  Point2D temp;
  return distance(pt,temp);
}

Real Box2D::distance(const Point2D& pt,Point2D& out) const
{
  return Sqrt(distanceSquared(pt,out));
}

Real Box2D::distanceSquared(const Point2D& pt,Point2D& out) const
{
  Point2D loc;
  toLocal(pt, loc);
  if(loc.x < 0) loc.x = 0;
  if(loc.y < 0) loc.y = 0;
  if(loc.x > dims.x) loc.x = dims.x;
  if(loc.y > dims.y) loc.y = dims.y;
  Real norm2 = loc.normSquared();
  fromLocal(loc,out);
  return norm2;
}

Real Box2D::signedDistance(const Point2D& pt) const
{
  Point2D closest;
  return signedDistance(pt,closest);
}

Real Box2D::signedDistance(const Point2D& pt,Point2D& out) const
{
  Point2D loc;
  toLocal(pt, loc);
  out = loc;
  bool inside = true;
  Real dmin = Inf;
  if(loc.x < 0) { out.x = 0; inside=false; }
  else dmin = Min(dmin,loc.x);
  if(loc.y < 0) { out.y = 0; inside=false; }
  else dmin = Min(dmin,loc.y);
  if(loc.x > dims.x) { out.x = dims.x; inside=false; }
  else dmin = Min(dmin,dims.x-loc.x);
  if(loc.y > dims.y) { out.y = dims.y; inside=false; }
  else dmin = Min(dmin,dims.y-loc.y);
  Real norm = loc.distance(out);
  loc = out;
  fromLocal(loc,out);
  return norm;
}

bool Box2D::intersects(const AABB2D& b) const
{
  Box2D temp;
  temp.set(b);
  return intersects(temp);
}

bool Box2D::intersects(const Box2D& b) const
{
	Box2D temp;
	AABB2D aabb_temp, aabb_temp2;
	//make temp localized
	temp.dims = b.dims;
	toLocal(b.origin, temp.origin);
	toLocalReorient(b.xbasis, temp.xbasis);
	toLocalReorient(b.ybasis, temp.ybasis);
	temp.getAABB(aabb_temp);
	aabb_temp2.bmin.setZero();
	aabb_temp2.bmax = dims;
	if(!aabb_temp2.intersects(aabb_temp))
		return false;

	temp.dims = dims;
	b.toLocal(origin, temp.origin);
	b.toLocalReorient(xbasis, temp.xbasis);
	b.toLocalReorient(ybasis, temp.ybasis);
	temp.getAABB(aabb_temp);
	aabb_temp2.bmax = b.dims;
	if(!aabb_temp2.intersects(aabb_temp))
		return false;
	return true;
}

void Box2D::getAABB(AABB2D& bb) const
{
	Vector2 x(dims.x*xbasis),y(dims.y*ybasis);
	Vector2 c=origin + 0.5*(x+y);

	Vector2 d;
	for(int i=0; i<2; i++)
	{
	  d[i] = Abs(x[i]) + Abs(y[i]);
	}

	bb.bmin.sub(c,d*0.5);
	bb.bmax.add(c,d*0.5);
}

bool Box2D::intersects(const Segment2D& s) const
{
  Segment2D sloc;
  toLocal(s,sloc);
  AABB2D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return sloc.intersects(bbloc);
}

bool Box2D::intersects(const Line2D& l) const
{
  Line2D lloc;
  toLocal(l,lloc);
  AABB2D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return lloc.lineIntersects(bbloc);
}

bool Box2D::intersects(const Triangle2D& t) const
{
  Triangle2D tloc;
  toLocal(t.a,tloc.a);
  toLocal(t.b,tloc.b);
  toLocal(t.c,tloc.c);
  AABB2D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  FatalError("Can't intersect box and triangle yet");
  //return tloc.intersects(bbloc);
  return false;
}

bool Box2D::intersects(const Circle2D& s) const
{
  Circle2D sloc;
  toLocal(s.center,sloc.center);
  sloc.radius = s.radius;
  AABB2D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return sloc.intersects(bbloc);
}



bool Plane2D::Read(File& f)
{
	if(!normal.Read(f)) return false;
	if(!ReadFile(f,offset)) return false;
	return true;
}

bool Plane2D::Write(File& f) const
{
	if(!normal.Write(f)) return false;
	if(!WriteFile(f,offset)) return false;
	return true;
}

void Plane2D::setPointNormal(const Point2D& a, const Vector2& n)
{
	normal.setNormalized(n);
	offset = dot(a,normal);
}

void Plane2D::setLine(const Line2D& l)
{
	normal.setPerpendicular(l.direction);
	setPointNormal(l.source,normal);
}

void Plane2D::setPoints(const Point2D& a, const Point2D& b)
{
  Vector2 v;
  v.setPerpendicular(b-a);
  setPointNormal(a,v);
}

void Plane2D::setTransformed(const Plane2D& pin, const Matrix3& xform)
{
	xform.mulVector(pin.normal, normal);

	Vector2 v, v_out;
	v = pin.normal*pin.offset;
	xform.mulPoint(v, v_out);
	setPointNormal(v_out, normal);
}

//returns the orthogonal distance from the plane to the point
Real Plane2D::distance(const Point2D& v) const
{
	return dot(v,normal) - offset;
}

//projects a Vector2 onto this plane
void Plane2D::project(const Point2D& vin, Point2D& vout) const
{
	vout = vin - normal*distance(vin);
}

void Plane2D::getBasis(Vector2& xb) const
{
  xb.setPerpendicular(normal);
}


bool Plane2D::intersectsSegment(const Segment2D& s, Real* t)
{
  Real da = distance(s.a);
  Real db = distance(s.b);
  if(da < Zero) {
    if(db < Zero) return false;
  }
  else {
    if(db > Zero) return false;
  }
  if(t) {
    //d = da + t*(db-da)
    if(da == db) //segment on plane
      *t = Zero;
    else
      *t = da/(da-db);
  }
  return true;
}

bool Plane2D::intersectsLine(const Line2D& l, Real* t)
{
  Real ds = distance(l.source);
  if(dot(normal,l.direction) == Zero) {
    if(t) *t = Inf;
		return (ds == Zero);
  }
	if(t)
		//dot(s + t*d, n) = o   =>   dot(s,n) + t*dot(d,n) = o
		*t = -ds/dot(l.direction,normal);
	return true;
}

bool Plane2D::intersectsRay(const Ray2D& r, Real* t)
{
	//if source on - side, intersects if dir is +
	//if source on + side, intersects if dir is -
	Real src = distance(r.source);
	Real dir = dot(normal,r.direction);
	if(src < Zero) {
		if(dir > Zero) {
			if(*t) *t = -src/dir;
			return true;
		}
	}
	else if(src > Zero) {
		if(dir < Zero) {
			if(*t) *t = -src/dir;
			return true;
		}
	}
	else {
		if(*t) *t = Zero;
		return true;
	}
	return false;
}

bool Plane2D::intersects(const AABB2D& bb) const
{
	Vector2 vmin,vmax;
	//get the extreme points of the box relative to the plane's normal
	if(normal.x > Zero)
	{
		vmin.x = bb.bmin.x;
		vmax.x = bb.bmax.x;
	}
	else
	{
		vmin.x = bb.bmax.x;
		vmax.x = bb.bmin.x;
	}
	if(normal.y > Zero)
	{
		vmin.y = bb.bmin.y;
		vmax.y = bb.bmax.y;
	}
	else
	{
		vmin.y = bb.bmax.y;
		vmax.y = bb.bmin.y;
	}
	//intersects if the extreme points are on opposite sides of the plane
	return (distance(vmin) < Zero) != (distance(vmax) < Zero);

}

int Plane2D::allIntersections(const Plane2D& p,Vector2& pt) const
{
  Real x = p.normal.y*offset - normal.y*p.offset;
  Real y = normal.x*p.offset - p.normal.x*offset;
  Real w = normal.x*p.normal.y - normal.y*p.normal.x;
  if(Abs(w) < Epsilon) {
    //linearly dependent
    if(Abs(x) < Epsilon && Abs(y) < Epsilon)  //overlap
      return 2;
    //pt at infinity
    pt.x = x;
    pt.y = y;
    return 0;
  }
  else {
    pt.x = x/w;
    pt.y = y/w;
    return 1;
  }
}

/*
void LocalCoordinates2D::getBasis(Matrix3& basis) const
{
	basis.set(xbasis,ybasis,zbasis,origin);
}

void LocalCoordinates2D::getBasisInv(Matrix3& basis) const
{
	//transpose basis
	basis.setIdentity();
	basis.setRow1(xbasis);
	basis.setRow2(ybasis);
	basis.setRow2(zbasis);
	Vector2 v;
	basis.mulVector(origin, v);
	basis.setCol4(-v);
}

void LocalCoordinates2D::toLocalReorient(const Vector2& vin, Vector2& vout) const
{
	vout.x=dot(vin,xbasis);
	vout.y=dot(vin,ybasis);
	vout.z=dot(vin,zbasis);
}

void LocalCoordinates2D::toLocal(const Vector2& vin, Vector2& vout) const
{
	toLocalReorient(vin - origin, vout);	
}

void LocalCoordinates2D::fromLocalReorient(const Vector2& vin, Vector2& vout) const
{
	vout = vin.x*xbasis + vin.y*ybasis + vin.z*zbasis;
}

void LocalCoordinates2D::fromLocal(const Vector2& vin, Vector2& vout) const
{
	fromLocalReorient(vin, vout);	
	vout = vout+origin;
}

void LocalCoordinates2D::toLocal(const Line2D& l, Line2D& out) const
{
	toLocalReorient(l.direction, out.direction);
	toLocal(l.source, out.source);
}

void LocalCoordinates2D::fromLocal(const Line2D& l, Line2D& out) const
{
	fromLocalReorient(l.direction, out.direction);
	fromLocal(l.source, out.source);
}

void LocalCoordinates2D::toLocal(const Segment2D& l, Segment2D& out) const
{
	toLocal(l.a, out.a);
	toLocal(l.b, out.b);
}

void LocalCoordinates2D::fromLocal(const Segment2D& l, Segment2D& out) const
{
	fromLocal(l.a, out.a);
	fromLocal(l.b, out.b);
}

void LocalCoordinates2D::toLocal(const Plane2D& p, Plane2D& out) const
{
	toLocalReorient(p.normal, out.normal);
	Vector2 v = p.normal*p.offset;
	Vector2 v_out;
	toLocal(v, v_out);
	out.offset = dot(v_out, out.normal);
}

void LocalCoordinates2D::fromLocal(const Plane2D& p, Plane2D& out) const
{
	fromLocalReorient(p.normal, out.normal);

	Vector2 v = p.normal*p.offset;
	Vector2 v_out;
	fromLocal(v, v_out);
	out.offset = dot(v_out, out.normal);
}


void ScaleXBasis(Matrix3& xform, Real scale)
{
	xform(0,0) *= scale;
	xform(1,0) *= scale;
	xform(2,0) *= scale;
}

void ScaleYBasis(Matrix3& xform, Real scale)
{
	xform(0,1) *= scale;
	xform(1,1) *= scale;
	xform(2,1) *= scale;
}

void ScaleZBasis(Matrix3& xform, Real scale)
{
	xform(0,2) *= scale;
	xform(1,2) *= scale;
	xform(2,2) *= scale;
}

void ScaledLocalCoordinates2D::getBasisScaled(Matrix3& basis) const
{
	basis.set(xbasis*dims.x, ybasis*dims.y, zbasis*dims.z, origin);
}

void ScaledLocalCoordinates2D::getBasisScaledInv(Matrix3& basis) const
{
	//transpose basis
	basis.setIdentity();
	basis.setRow1(xbasis);
	basis.setRow2(ybasis);
	basis.setRow2(zbasis);

	Vector2 v;
	basis.mulVector(origin, v);
	basis.setCol4(-v);

	ScaleXBasis(basis,Inv(dims.x));
	ScaleYBasis(basis,Inv(dims.y));
	ScaleZBasis(basis,Inv(dims.z));
}

void ScaledLocalCoordinates2D::normalize(const Vector2& vin, Vector2& vout) const
{
	vout.x=vin.x/dims.x;
	vout.y=vin.y/dims.y;
	vout.z=vin.z/dims.z;
}

void ScaledLocalCoordinates2D::denormalize(const Vector2& vin, Vector2& vout) const
{
	vout.x=vin.x*dims.x;
	vout.y=vin.y*dims.y;
	vout.z=vin.z*dims.z;
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Point2D& p, Point2D& out) const
{
	toLocal(p, out);
	normalize(out, out);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Point2D& p, Point2D& out) const
{
	denormalize(out, out);
	fromLocal(p, out);
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Line2D& l, Line2D& out) const
{
	toLocal(l, out);
	normalize(out.direction, out.direction);
	normalize(out.source, out.source);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Line2D& l, Line2D& out) const
{
	Line2D temp;
	denormalize(l.direction, temp.direction);
	denormalize(l.source, temp.source);
	fromLocal(temp, out);
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Segment2D& l, Segment2D& out) const
{
	toLocal(l, out);
	normalize(out.a, out.a);
	normalize(out.b, out.b);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Segment2D& l, Segment2D& out) const
{
	Segment2D temp;
	denormalize(l.a, temp.a);
	denormalize(l.b, temp.b);
	fromLocal(temp, out);
}

void ScaledLocalCoordinates2D::toLocalNormalized(const Plane2D& p, Plane2D& out) const
{
	toLocalReorient(p.normal, out.normal);
	denormalize(out.normal, out.normal);
	out.normal.inplaceNormalize();

	Vector2 v = p.normal*p.offset;
	Vector2 v_out;
	toLocal(v, v_out);
	normalize(v_out, v_out);
	out.offset = dot(v_out, out.normal);
}

void ScaledLocalCoordinates2D::fromLocalNormalized(const Plane2D& p, Plane2D& out) const
{
	Plane2D p_denorm;
	normalize(p.normal, p_denorm.normal);
	p_denorm.normal.inplaceNormalize();

	Vector2 v = p.normal * p.offset;
	Vector2 v_out;
	denormalize(v, v_out);

	p_denorm.offset = dot(v_out, p_denorm.normal);

	fromLocal(p_denorm, out);
}

bool Box2D::contains(const Point2D& pt) const
{
	Point2D out;
	toLocal(pt,out);
	return 0<=out.x&&out.x<=dims.x &&
		0<=out.y&&out.y<=dims.y &&
		0<=out.z&&out.z<=dims.z;
}

bool Box2D::withinDistance(const Point2D& pt, Real dist) const
{
	Circle2D c;
	toLocal(pt, c.center);
	c.radius = dist;

	AABB2D bb_local;
	bb_local.bmin.setZero();
	bb_local.bmax=dims;
	return bb.intersects(c);
}

bool Box2D::intersects(const Box2D& b) const
{
	Box2D temp;
	AABB2D AABB2D_temp, AABB2D_temp2;
	//make temp localized
	temp.dims = b.dims;
	toLocal(b.origin, temp.origin);
	toLocalReorient(b.xbasis, temp.xbasis);
	toLocalReorient(b.ybasis, temp.ybasis);
	toLocalReorient(b.zbasis, temp.zbasis);
	AABB2D_temp.calculate(temp);
	AABB2D_temp2.bmin.setZero();
	AABB2D_temp2.bmax = dims;
	if(!AABB2D_temp2.intersects(AABB2D_temp))
		return false;

	temp.dims = dims;
	b.toLocal(origin, temp.origin);
	b.toLocalReorient(xbasis, temp.xbasis);
	b.toLocalReorient(ybasis, temp.ybasis);
	b.toLocalReorient(zbasis, temp.zbasis);
	AABB2D_temp.calculate(temp);
	AABB2D_temp2.bmax = b.dims;
	if(!AABB2D_temp2.intersects(AABB2D_temp))
		return false;
	return true;
}

*/

bool Circle2D::Read(File& f)
{
	if(!center.Read(f)) return false;
	if(!ReadFile(f,radius)) return false;
	return true;
}

bool Circle2D::Write(File& f) const
{
	if(!center.Write(f)) return false;
	if(!WriteFile(f,radius)) return false;
	return true;
}

Real Circle2D::distance(const Point2D& v) const
{
	return (center-v).norm() - radius;
}

Real Circle2D::signedDistance(const Point2D& v) const
{
    return (center-v).norm() - radius;
}

bool Circle2D::contains(const Point2D& v) const
{
	return DistanceLEQ(center,v,radius);
}

bool Circle2D::contains(const Circle2D& s) const
{
	return DistanceLEQ(center,s.center,radius-s.radius);
}

bool Circle2D::withinDistance(const Point2D& v, Real dist) const
{
	return DistanceLEQ(center,v, radius + dist);
}

bool Circle2D::boundaryWithinDistance(const Point2D& v, Real dist) const
{
    return Abs((center-v).norm()-radius) <= dist;
}

bool Circle2D::intersects(const Segment2D& s) const
{
  Line2D l;
  l.source = s.a;
  l.direction = s.b-s.a;
  Real t1,t2;
  if(!intersects(l,&t1,&t2)) return false;
  if (t1 > t2) Swap(t1,t2);
  if(t2 < 0 || t1 > 1.0) return false;
  return true;
}

bool Circle2D::intersects(const Line2D& l, Real* t1, Real* t2) const
{
	Vector2 offset=center-l.source;
	Real o_o=dot(offset,offset), o_b=dot(offset,l.direction), b_b=dot(l.direction,l.direction);
	//so we know there's a root to |offset-t*b|==r
	//o.o-2t*b.o+t^2*b.b=r^2
	Real a,b,c;
	a=b_b;
	b=-Two*o_b;
	c=o_o-radius*radius;
	Real x1,x2;
	int res=quadratic(a,b,c,x1,x2);
	if(res<=0) return false;
	if(t1 && t2) {
		*t1=x1;
		*t2=x2;
	}
	return true;
}

bool Circle2D::intersects(const Plane2D& p, Segment2D& s) const
{
  Real d=p.distance(center);
  if(Abs(d) <= radius) {
    //project c on plane
    Vector2 c=center,basis;
    c.madd(p.normal,-d);
    p.getBasis(basis);
    Real h=pythag_leg(d,radius);
    s.a = c + h*basis;
    s.b = c - h*basis;
    return true;
  }
  return false;
}

bool Circle2D::intersects(const Circle2D& s) const
{
  return disksIntersect(center,radius,s.center,s.radius);
}

bool Circle2D::boundaryIntersects(const Circle2D& s) const
{
  return diskCircleIntersect(s.center,s.radius,center,radius);
}

bool Circle2D::boundaryIntersectsBoundary(const Circle2D& s) const
{
  return circlesIntersect(center,radius,s.center,s.radius);
}

bool Circle2D::disksIntersect(const Point2D& ca,Real ra,const Point2D& cb,Real rb)
{
	return DistanceLEQ(ca,cb,ra+rb);
}

bool Circle2D::diskCircleIntersect(const Point2D& ca,Real ra,const Point2D& cb,Real rb)
{
  Real r2 = (ca-cb).normSquared();
  if(r2 <= Sqr(ra+rb)) {
    Real r = Sqrt(r2);
    if(r + ra < rb) return false;
    return true;
  }
  return false;
}

bool Circle2D::circlesIntersect(const Point2D& ca,Real ra,const Point2D& cb,Real rb)
{
  Real r2 = (ca-cb).normSquared();
  if(r2 <= Sqr(ra+rb)) {
    Real r = Sqrt(r2);
    if(r + ra < rb) return false;
    if(r + rb < ra) return false;
    return true;
  }
  return false;
}

void Circle2D::getAABB(AABB2D& bb) const
{
	bb.setPoint(center);
	bb.bmin.x-=radius; bb.bmin.y-=radius;
	bb.bmax.x+=radius; bb.bmax.y+=radius;
}

bool Circle2D::intersects(const AABB2D& bb) const
{
  Vector2 temp;
  return bb.distanceSquared(center,temp) < Sqr(radius);
}



/*

bool Ellipse2D::contains(const Point2D& pt) const
{
	Point2D out;
	toLocalNormalized(pt,out);
	return NormLEQ(out,One);
}

bool Ellipse2D::intersects(const Line2D& l, Real* t1, Real* t2) const
{
	Line2D llocal;
	toLocalNormalized(l,llocal);
	Circle2D s;
	s.center.setZero();
	s.radius = One;
	return s.intersects(llocal,t1,t2);
}
*/




GeometricPrimitive2D::GeometricPrimitive2D()
  :type(Empty)
{}

GeometricPrimitive2D::GeometricPrimitive2D(const GeometricPrimitive2D& rhs)
  :type(rhs.type),data(rhs.data)
{}

GeometricPrimitive2D::GeometricPrimitive2D(const Vector2& point)
  :type(Point),data(point)
{}

GeometricPrimitive2D::GeometricPrimitive2D(const Segment2D& segment)
  :type(Segment),data(segment)
{}
GeometricPrimitive2D::GeometricPrimitive2D(const AABB2D& aabb)
  :type(AABB),data(aabb)
{}

GeometricPrimitive2D::GeometricPrimitive2D(const Box2D& box)
  :type(Box),data(box)
{}

GeometricPrimitive2D::GeometricPrimitive2D(const Circle2D& circle)
  :type(Circle),data(circle)
{}

GeometricPrimitive2D::GeometricPrimitive2D(const Triangle2D& triangle)
  :type(Triangle),data(triangle)
{}

const char* GeometricPrimitive2D::TypeName(Type type) 
{
  switch(type) {
  case Empty:
  	return "empty";
  case Point:
    return "point";
  case Segment:
    return "segment";
  case Circle:
    return "circle";
  case AABB:
    return "aabb";
  case Box:
    return "box";
  case Triangle:
    return "triangle";
  default:
    return "error";
  }
}

void GeometricPrimitive2D::Set(const Vector2& point)
{
  type = Point;
  data = point;
}

void GeometricPrimitive2D::Set(const Segment2D& segment)
{
  type = Segment;
  data = segment;
}

void GeometricPrimitive2D::Set(const AABB2D& aabb)
{
  type = AABB;
  data = aabb;
}

void GeometricPrimitive2D::Set(const Box2D& box)
{
  type = Box;
  data = box;
}

void GeometricPrimitive2D::Set(const Circle2D& circle)
{
  type = Circle;
  data = circle;
}

void GeometricPrimitive2D::Set(const Triangle2D& triangle)
{
  type = Triangle;
  data = triangle;
}

bool GeometricPrimitive2D::SupportsCollides(Type a,Type b)
{
  return true;
}

bool GeometricPrimitive2D::Collides(const GeometricPrimitive2D& geom) const
{
  switch(type) {
  case Point:
    return geom.Collides(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    return geom.Collides(*AnyCast_Raw<Segment2D>(&data));
  case Circle:
    return geom.Collides(*AnyCast_Raw<Circle2D>(&data));
  case AABB:
    return geom.Collides(*AnyCast_Raw<AABB2D>(&data));
  case Box:
    return geom.Collides(*AnyCast_Raw<Box2D>(&data));
  case Triangle:
    return geom.Collides(*AnyCast_Raw<Triangle2D>(&data));
  default:
    return false;
  }
}

bool GeometricPrimitive2D::Collides(const Vector2& point) const
{
  switch(type) {
  case Point:
    return *AnyCast_Raw<Vector2>(&data) == point;
  case Segment:
    return AnyCast_Raw<Segment2D>(&data)->distance(point) == 0;
  case Circle:
    return AnyCast_Raw<Circle2D>(&data)->contains(point);
  case AABB:
    return AnyCast_Raw<AABB2D>(&data)->contains(point);
  case Box:
    return AnyCast_Raw<Box2D>(&data)->contains(point);
  case Triangle:
    return AnyCast_Raw<Triangle2D>(&data)->contains(point);
  default:
    return false;
  }
}

bool GeometricPrimitive2D::Collides(const Segment2D& seg) const
{
  switch(type) {
  case Point:
    return seg.distance(*AnyCast_Raw<Vector2>(&data)) == 0;
  case Segment:
    return AnyCast_Raw<Segment2D>(&data)->intersects(seg);
  case Circle:
    return AnyCast_Raw<Circle2D>(&data)->intersects(seg);
  case AABB:
    return seg.intersects(*AnyCast_Raw<AABB2D>(&data));
  case Box:
    return AnyCast_Raw<Box2D>(&data)->intersects(seg);
  case Triangle:
    return AnyCast_Raw<Triangle2D>(&data)->intersects(seg);
  default:
    return false;
  }
}

bool GeometricPrimitive2D::Collides(const AABB2D& aabb) const
{
  Box2D box;
  box.set(aabb);
  return Collides(box);
}

bool GeometricPrimitive2D::Collides(const Box2D& box) const
{
  switch(type) {
  case Point:
    return box.contains(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    return box.intersects(*AnyCast_Raw<Segment2D>(&data));
  case Circle:
    return box.intersects(*AnyCast_Raw<Circle2D>(&data));
  case AABB:
    return box.intersects(*AnyCast_Raw<AABB2D>(&data));
  case Box:
    return box.intersects(*AnyCast_Raw<Box2D>(&data));
  case Triangle:
    return box.intersects(*AnyCast_Raw<Triangle2D>(&data));
  default:
    return false;
  }
}

bool GeometricPrimitive2D::Collides(const Circle2D& circle) const
{
  return Distance(circle.center) <= circle.radius;
}

bool GeometricPrimitive2D::Collides(const Triangle2D& triangle) const
{
  switch(type) {
  case Point:
    return triangle.contains(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    return triangle.intersects(*AnyCast_Raw<Segment2D>(&data));
  case Circle:
    {
      const Circle2D* c=AnyCast_Raw<Circle2D>(&data);
      return c->contains(triangle.closestPoint(c->center));
    }
  case AABB:
    {
      Box2D box;
      box.set(*AnyCast_Raw<AABB2D>(&data));
      return box.intersects(triangle);
    }
  case Box:
    return AnyCast_Raw<Box2D>(&data)->intersects(triangle);
  case Triangle:
    return triangle.intersects(*AnyCast_Raw<Triangle2D>(&data));
  default:
    return false;
  }
}

bool GeometricPrimitive2D::SupportsDistance(Type a,Type b)
{
  if((a==AABB||a==Box||a==Triangle) && (b==Segment || b==AABB || b==Box || b==Triangle))
    return false;

  if(a==Segment && (b==AABB || b==Box || b==Triangle))
    return false;
  
  return true;
}

Real GeometricPrimitive2D::Distance(const GeometricPrimitive2D& geom) const
{
  switch(type) {
  case Point:
    return geom.Distance(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    return geom.Distance(*AnyCast_Raw<Segment2D>(&data));
  case Circle:
    return geom.Distance(*AnyCast_Raw<Circle2D>(&data));
  case AABB:
    return geom.Distance(*AnyCast_Raw<AABB2D>(&data));
  case Box:
    return geom.Distance(*AnyCast_Raw<Box2D>(&data));
  case Triangle:
    return geom.Distance(*AnyCast_Raw<Triangle2D>(&data));
  default:
    return Inf;
  }
}

Real GeometricPrimitive2D::Distance(const Vector2& x) const
{
  switch(type) {
  case Point:
    return x.distance(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    return AnyCast_Raw<Segment2D>(&data)->distance(x);
  case Circle:
    return AnyCast_Raw<Circle2D>(&data)->distance(x);
  case AABB:
    return AnyCast_Raw<AABB2D>(&data)->distance(x);
  case Box:
    return AnyCast_Raw<Box2D>(&data)->distance(x);
  case Triangle:
    return AnyCast_Raw<Triangle2D>(&data)->closestPoint(x).distance(x);
  default:
    return Inf;
  }
}

Real GeometricPrimitive2D::Distance(const Circle2D& s) const
{
  Real d=Distance(s.center);
  return Max(0.0,d-s.radius);
}

Real GeometricPrimitive2D::Distance(const Segment2D& s) const
{
  switch(type) {
  case Point:
    return s.distance(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    {
      const Segment2D* seg=AnyCast_Raw<Segment2D>(&data);
      if(seg->intersects(s)) return 0.0;
      return Min(Min(seg->distance(s.a),seg->distance(s.b)),Min(s.distance(seg->a),s.distance(seg->b)));
    }
  case Circle:
    return Max(0.0,s.distance(AnyCast_Raw<Circle2D>(&data)->center)-AnyCast_Raw<Circle2D>(&data)->radius);
  case AABB:
    FatalError("AABB-Segment distance not implemented");
    return Inf;
  case Box:
    FatalError("Box-Segment distance not implemented");
    return Inf;
  case Triangle:
    FatalError("Triangle-Segment distance not implemented");
    return Inf;
  default:
    return Inf;
  }
}

Real GeometricPrimitive2D::Distance(const AABB2D& b) const
{
  switch(type) {
  case Point:
    return b.distance(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    FatalError("Segment-AABB distance not implemented");
    return Inf;
  case Circle:
    return Max(0.0,b.distance(AnyCast_Raw<Circle2D>(&data)->center)-AnyCast_Raw<Circle2D>(&data)->radius);
  case AABB:
    FatalError("AABB-AABB distance not implemented");
    return Inf;
  case Box:
    FatalError("Box-AABB distance not implemented");
    return Inf;
  case Triangle:
    FatalError("Triangle-AABB distance not implemented");
    return Inf;
  default:
    return Inf;
  }
}

Real GeometricPrimitive2D::Distance(const Box2D& b) const
{
  switch(type) {
  case Point:
    return b.distance(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    FatalError("Segment-Box distance not implemented");
    return Inf;
  case Circle:
    return Max(0.0,b.distance(AnyCast_Raw<Circle2D>(&data)->center)-AnyCast_Raw<Circle2D>(&data)->radius);
  case AABB:
    FatalError("AABB-Box distance not implemented");
    return Inf;
  case Box:
    FatalError("Box-Box distance not implemented");
    return Inf;
  case Triangle:
    FatalError("Triangle-Box distance not implemented");
    return Inf;
  default:
    return Inf;
  }
}

Real GeometricPrimitive2D::Distance(const Triangle2D& t) const
{
  switch(type) {
  case Point:
    return t.closestPoint(*AnyCast_Raw<Vector2>(&data)).distance(*AnyCast_Raw<Vector2>(&data));
  case Segment:
    FatalError("Segment-Triangle distance not implemented");
    return Inf;
  case Circle:
    return Max(0.0,t.closestPoint(AnyCast_Raw<Circle2D>(&data)->center).distance(AnyCast_Raw<Circle2D>(&data)->center)-AnyCast_Raw<Circle2D>(&data)->radius);
  case AABB:
    FatalError("AABB-Triangle distance not implemented");
    return Inf;
  case Box:
    FatalError("Box-Triangle distance not implemented");
    return Inf;
  case Triangle:
    FatalError("Triangle-Triangle distance not implemented");
    return Inf;
  default:
    return Inf;
  }
}




void GeometricPrimitive2D::Transform(const RigidTransform2D& T)
{
  switch(type) {
  case Point:
    data = T*(*AnyCast_Raw<Vector2>(&data));
    break;
  case Segment:
    {
      Segment2D* seg = AnyCast_Raw<Segment2D>(&data);
      seg->a = T*seg->a;
      seg->b = T*seg->b;
    }
    break;
  case Circle:
    AnyCast_Raw<Circle2D>(&data)->center = T*AnyCast_Raw<Circle2D>(&data)->center;
    break;
  case AABB:
    {
      Box2D box;
      box.setTransformed(*AnyCast_Raw<AABB2D>(&data),T);
      Set(box);
    }
    break;
  case Box:
    AnyCast_Raw<Box2D>(&data)->setTransformed(*AnyCast_Raw<Box2D>(&data),T);
    break;
  case Triangle:
    AnyCast_Raw<Triangle2D>(&data)->setTransformed(*AnyCast_Raw<Triangle2D>(&data),T);
    break;
  default:
    return;
  }
}

void GeometricPrimitive2D::ToPolygon(vector<Vector2> & poly,int divs) const
{
  switch(type) {
  case Point:
    poly.resize(1);
    poly[0] = *AnyCast_Raw<Vector2>(&data);
    return;
  case Segment:
    poly.resize(2);
    poly[0] = AnyCast_Raw<Segment2D>(&data)->a;
    poly[1] = AnyCast_Raw<Segment2D>(&data)->b;
    return;
  case Circle:
    {
      const Circle2D* c=AnyCast_Raw<Circle2D>(&data);
      poly.resize(divs);
      for(int k=0;k<divs;k++) {
	Real u = Real(k)/Real(divs);
	Vector2 ofs(Cos(TwoPi*u),Sin(TwoPi*u));
	poly[k].set(c->center + c->radius*ofs);
      }
      return;
    }
  case AABB:
    {
      const AABB2D* aabb=AnyCast_Raw<AABB2D>(&data);
      poly.resize(4);
      poly[0].set(aabb->bmin.x,aabb->bmin.y);
      poly[1].set(aabb->bmax.x,aabb->bmin.y);
      poly[2].set(aabb->bmax.x,aabb->bmax.y);
      poly[3].set(aabb->bmin.x,aabb->bmax.y);
    }
    return;
  case Box:
    {
      const Box2D* box = AnyCast_Raw<Box2D>(&data);
      poly.resize(4);
      poly[0].set(box->origin);
      poly[1].set(box->origin+box->dims.x*box->xbasis);
      poly[2].set(box->origin+box->dims.x*box->xbasis+box->dims.y*box->ybasis);
      poly[3].set(box->origin+box->dims.y*box->ybasis);
    }
    return;
  case Triangle:
    {
      const Triangle2D* tri=AnyCast_Raw<Triangle2D>(&data);
      poly.resize(3);
      poly[0].set(tri->a);
      poly[1].set(tri->b);
      poly[2].set(tri->c);
    }
    return;
  default:
    poly.resize(0);
    return;
  }
}

AABB2D GeometricPrimitive2D::GetAABB() const
{
  AABB2D bb;
  switch(type) {
  case Point:
    bb.bmin = bb.bmin = *AnyCast_Raw<Vector2>(&data);
    break;
  case Segment:
    AnyCast_Raw<Segment2D>(&data)->getAABB(bb);
    break;
  case Circle:
    AnyCast_Raw<Circle2D>(&data)->getAABB(bb);
    break;
  case AABB:
    return *AnyCast_Raw<AABB2D>(&data);
  case Box:
    AnyCast_Raw<Box2D>(&data)->getAABB(bb);
    break;
  case Triangle:
    AnyCast_Raw<Triangle2D>(&data)->getAABB(bb);
    break;
  default:
    break;
  }
  return bb;
}


} // namespace Math2D
