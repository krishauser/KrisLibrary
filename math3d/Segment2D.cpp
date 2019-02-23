#include <KrisLibrary/Logger.h>
#include "Segment2D.h"
#include "geometry2d.h"
#include "clip.h"
#include "misc.h"
#include <math/misc.h>
#include <math/Interval.h>
#include "interpolate.h"
#include <iostream>
using namespace Math3D;
using namespace std;


bool Segment2D::Read(File& f)
{
	if(!a.Read(f)) return false;
	if(!b.Read(f)) return false;
	return true;
}

bool Segment2D::Write(File& f) const
{
	if(!a.Write(f)) return false;
	if(!b.Write(f)) return false;
	return true;
}

void Segment2D::setTransformed(const Segment2D& s, const Matrix3& xform)
{
	xform.mulPoint(s.a,a);
	xform.mulPoint(s.b,b);
}

void Segment2D::getLine(Line2D& l) const
{
	l.source=a;
	l.direction=b-a;
}

Real Segment2D::closestPointParameter(const Point2D& in) const
{
	Vector2 dir=b-a;
	Real numer = dot(in-a,dir);
	Real denom = dot(dir,dir);
	//t = numer/denom, denom always >= 0
	if(numer <= Zero) return Zero;
	if(numer >= denom) return One;
	return numer/denom;
}

Real Segment2D::closestPoint(const Point2D& in,Point2D& out) const
{
	Real t = closestPointParameter(in);
	eval(t,out);
	return t;
}

Real Segment2D::distance(const Point2D& pt) const
{
  Point2D closest;
  closestPoint(pt,closest);
  return (pt-closest).norm();
}

void Segment2D::eval(Real t, Point2D& out) const
{
	out = a;
	out.madd(b-a,t);
}

Real Segment2D::orientation(const Point2D& a,const Point2D& b,const Point2D& x)
{
  return Orient2D(a,b,x);
}

Real Segment2D::orientation(const Vector2& x) const
{
  return Orient2D(a,b,x);
}

bool Segment2D::intersects(const Segment2D& S) const
{
  return intersects(S.a,S.b);
}

bool Segment2D::intersects(const Vector2& A,const Vector2& B) const
{
  Real u,v;
  u = orientation(A,B,a);
  v = orientation(A,B,b);
  if((u < Zero && v < Zero) || (u > Zero && v > Zero)) return false;
  u = orientation(a,b,A);
  v = orientation(a,b,B);
  if((u < Zero && v < Zero) || (u > Zero && v > Zero)) return false;
  return true;
}

bool Segment2D::intersects(const Segment2D& S,Vector2& p) const
{
  return intersects(S.a,S.b,p);
}

bool Segment2D::intersects(const Vector2& A,const Vector2& B,Vector2& p) const
{
  //find (u,v) s.t. a+u(b-a) = A+v(B-A)
  //=> (b-a | A-B)*(u,v)^T = (A-a)
  Matrix2 M;
  Vector2 res,uv;
  M.setCol1(b-a);
  M.setCol2(A-B);
  res = A-a;
  if(Math::FuzzyZero(M.determinant())) {
    //they're parallel
    Vector2 t = b-a;
    Vector2 n; n.setPerpendicular(t);
    Real D = dot(n,A);
    Real d = dot(n,a);
    if(Math::FuzzyEquals(d,D)) {  //they overlap
      ClosedInterval U,u;
      u.a = 0;
      u.b = t.normSquared();
      U.a = dot(t,A-a);
      U.b = dot(t,B-a);
      if(U.intersects(u)) {
	ClosedInterval i;
	i.setIntersection(u,U);
	Real param=0.5*(i.a+i.b);
	p = a + t*(param/u.b);
	return true;
      }
    }
    return false;
  }
  M.inplaceInverse();
  M.mul(res,uv);
  if(uv.x>=Zero && uv.x<=One &&
     uv.y>=Zero && uv.y<=One) {
    interpolate(a,b,uv.x,p);
    Vector2 temp;
    interpolate(A,B,uv.y,temp);
    if(temp.distance(p) > 1e-3) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error: intersection points are too far away ");
      LOG4CXX_INFO(KrisLibrary::logger(),A<<" -> "<<B);
      LOG4CXX_INFO(KrisLibrary::logger(),a<<" -> "<<b);
      LOG4CXX_INFO(KrisLibrary::logger(),"u,v "<<uv);
      LOG4CXX_INFO(KrisLibrary::logger(),"inverse basis "<<M<<"\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"p1,p2 "<<p<<", "<<temp);
      abort();
    }
    return true;
  }
  /*
  if(intersects(a,b)) {
    if(Math::FuzzyZero(uv.x)) { p=a; return true; }
    if(Math::FuzzyEquals(uv.x,One)) { p=b; return true; }
    if(Math::FuzzyZero(uv.y)) { p=a; return true; }
    if(Math::FuzzyEquals(uv.y,One)) { p=b; return true; }
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error! segment is supposed to intersect, but we don't have that in the basis!");
    LOG4CXX_INFO(KrisLibrary::logger(),A<<" -> "<<B);
    LOG4CXX_INFO(KrisLibrary::logger(),a<<" -> "<<b);
    LOG4CXX_INFO(KrisLibrary::logger(),"u,v "<<uv);
    LOG4CXX_INFO(KrisLibrary::logger(),"inverse basis "<<M<<"\n");
    abort();
  }
  */
  return false;
}

void Segment2D::getAABB(AABB2D& bb) const
{
  bb.setPoint(a);
  bb.expand(b);
}

bool Segment2D::intersects(const AABB2D& bb) const
{
  Real u1,u2;
  return intersects(bb,u1,u2);
}

bool Segment2D::intersects(const AABB2D& bb, Real& u1, Real& u2) const
{
  u1=0;
  u2=1;
  return ClipLine(a, b-a, bb, u1,u2);
}

Real Segment2D::distance(const AABB2D& bb) const
{
  Real tmin;
  return distance(bb,tmin);
}

Real Segment2D::distance(const AABB2D& bb, Real& tmin) const
{
  Real tmax;
  if(intersects(bb,tmin,tmax)) return 0;
  //non-intersecting -- either bb vertex - seg is closest feature pair or
  //bb edge - endpoint is closest feature pair

  //first check segment endpoints
  Real dmin;
  dmin = bb.distance(a);
  tmin = 0.0;
  Real d=bb.distance(b);
  if(d < dmin) {
    dmin = d;
    tmin = 1.0;
  }
  //now check vertices
  Vector2 p;
  Real t = closestPoint(bb.bmax,p);
  d = p.distanceSquared(bb.bmax);
  if(d < Sqr(dmin)) {
    dmin = Sqrt(d);
    tmin = t;
  }
  t = closestPoint(bb.bmin,p);
  d = p.distanceSquared(bb.bmin);
  if(d < Sqr(dmin)) {
    dmin = Sqrt(d);
    tmin = t;
  }
  t = closestPoint(Vector2(bb.bmin.x,bb.bmax.y),p);
  d = p.distanceSquared(Vector2(bb.bmin.x,bb.bmax.y));
  if(d < Sqr(dmin)) {
    dmin = Sqrt(d);
    tmin = t;
  }
  t = closestPoint(Vector2(bb.bmax.x,bb.bmin.y),p);
  d = p.distanceSquared(Vector2(bb.bmax.x,bb.bmin.y));
  if(d < Sqr(dmin)) {
    dmin = Sqrt(d);
    tmin = t;
  }
  return dmin;
}
