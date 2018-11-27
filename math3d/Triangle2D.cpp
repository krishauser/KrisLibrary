#include <KrisLibrary/Logger.h>
#include "Triangle2D.h"
#include "geometry2d.h"
#include "clip.h"
#include "misc.h"
//#include <math/misc.h>
//#include <math/Interval.h>
//#include "LinearlyDependent.h"
//#include "interpolate.h"
using namespace Math3D;

Triangle2D::Triangle2D()
{}

Triangle2D::Triangle2D(const Vector2& _a,const Vector2& _b,const Vector2& _c)
:a(_a),b(_b),c(_c)
{}

void Triangle2D::set(const Vector2& _a,const Vector2& _b,const Vector2& _c)
{
  a=_a;
  b=_b;
  c=_c;
}

void Triangle2D::setTransformed(const Triangle2D& t, const RigidTransform2D& xform)
{
  if(&t == this) {
    Triangle2D tmp=t;
    setTransformed(tmp,xform);
  }
  else {
    xform.mulPoint(t.a,a);
    xform.mulPoint(t.b,b);
    xform.mulPoint(t.c,c);
  }
}

void Triangle2D::setTransformed(const Triangle2D& t, const Matrix3& xform)
{
  if(&t == this) {
    Triangle2D tmp=t;
    setTransformed(tmp,xform);
  }
  else {
    xform.mulPoint(t.a,a);
    xform.mulPoint(t.b,b);
    xform.mulPoint(t.c,c);
  }
}

bool Triangle2D::Read(File& f)
{
	if(!a.Read(f)) return false;
	if(!b.Read(f)) return false;
	if(!c.Read(f)) return false;
	return true;
}

bool Triangle2D::Write(File& f) const
{
	if(!a.Write(f)) return false;
	if(!b.Write(f)) return false;
	if(!c.Write(f)) return false;
	return true;
}

Real Triangle2D::orientation() const
{
  return Orient2D(a,b,c);
}

Real Triangle2D::orientation(const Point2D& a, const Point2D& b, const Point2D& c)
{
  return Orient2D(a,b,c);
}

Real Triangle2D::area() const
{
  return Abs(orientation())*Half;
}

Real Triangle2D::area(const Point2D& a, const Point2D& b, const Point2D& c)
{
  return Abs(orientation(a,b,c))*Half;
}

Vector3 Triangle2D::barycentricCoords(const Point2D& x) const
{
	return barycentricCoords(x,a,b,c);
}

Vector3 Triangle2D::barycentricCoords(const Vector2& x, const Point2D& a, const Point2D& b, const Point2D& c)
{
	Vector3 out;
	out.x = area(x,b,c);
	out.y = area(x,c,a);
	out.z = area(x,a,b);
	Real scale = Inv(area(a,b,c));
	return out*scale;
}

Point2D Triangle2D::barycentricCoordsToPoint(const Vector3& bc) const
{
  return barycentricCoordsToPoint(bc,a,b,c);
}

Point2D Triangle2D::barycentricCoordsToPoint(const Vector3& bc, const Point2D& a, const Point2D& b, const Point2D& c)
{
  Point2D x;
  x.mul(a,bc.x);
  x.madd(b,bc.y);
  x.madd(c,bc.z);
  return x;
}

bool Triangle2D::containsBarycentricCoords(const Vector3& bc)
{
  return bc.x >= Zero && bc.y >= Zero && bc.z >= Zero && bc.x+bc.y+bc.z <= One;
}


Vector2 Triangle2D::planeCoords(const Point2D& x) const
{
	Real A,B,C,D,E;
	Vector2 e0=b-a,e1=c-a,x0=a-x;
	A = dot(e0,e0);
	B = dot(e0,e1);
	C = dot(e1,e1);
	D = dot(e0,x0);
	E = dot(e1,x0);
	Real det = A*C-B*B;
	if(det == Zero) { //collapsed to a line
		return Vector2(Zero);
	}
	return Vector2((B*E-C*D)/det,(B*D-A*E)/det);
}

Point2D Triangle2D::planeCoordsToPoint(const Vector2& pc) const
{
  return planeCoordsToPoint(pc,a,b,c);
}

Point2D Triangle2D::planeCoordsToPoint(const Vector2& pc,
		const Point2D& a, const Point2D& b, const Point2D& c)
{
  Point2D pos=a;
  pos.madd(b-a, pc.x);
  pos.madd(c-a, pc.y); 
  return pos;
}

bool Triangle2D::containsPlaneCoords(const Vector2& pc)
{
  return pc.x >= Zero && pc.y >= Zero && pc.x+pc.y <= One;
}

Vector2 Triangle2D::closestPointCoords(const Point2D& in) const
{
	Real A,B,C,D,E;
	Vector2 e1=b-a,e2=c-a,x0=a-in;
	A = dot(e1,e1);
	B = dot(e1,e2);
	C = dot(e2,e2);
	D = dot(e1,x0);
	E = dot(e2,x0);
	Real det = A*C-B*B;
	if(det == Zero) { //collapsed to a line
		return Vector2(Zero);
	}

	Vector2 pc (B*E-C*D,B*D-A*E);
	if(pc.x < Zero) {  //check edge 2
		//t = dot(in-a,e2)/dot(e2,e2) = -E/C
		Real t;
		if(-E <= Zero) t=Zero;
		else if(-E >= C) t=One;
		else t = -E/C;
		return Vector2(Zero,t);
	}
	else if(pc.y < Zero) { //check edge 1
		//t = dot(in-a,e1)/dot(e1,e1) = -D/A
		Real t;
		if(-D <= Zero) t=Zero;
		else if(-D >= A) t=One;
		else t = -D/A;
		return Vector2(t,Zero);
	}
	else if(pc.x+pc.y > det) { //check edge 2
		//t = dot(in-a-e1,e2-e1)/dot(e2-e1,e2-e1)
		//= [dot(in-a,e2) - dot(in-a,e1) - dot(e1,e2) + dot(e1,e1)]/(dot(e2,e2) - 2dot(e2,e1) + dot(e1,e1)]
		//= -(B-A+E-D)/(C-2B+A)
		Real numer = -(B-A+E-D);
		Real denom = (A-2*B+C);
		Real t;
		//t = numer/denom with denom >= 0
		if(numer <= Zero) t=Zero;
		else if(numer >= denom) t=One;
		else t = numer/denom;
		return Vector2(One-t,t);
	}
	else {
		pc /= det;
		return pc;
	}
}

Point2D Triangle2D::closestPoint(const Point2D& in) const
{
  return planeCoordsToPoint(closestPointCoords(in));
}

bool Triangle2D::contains(const Point2D& x) const
{
  return containsPlaneCoords(planeCoords(x));
}



bool Triangle2D::intersects(const Plane2D& p) const
{
  Real minDist,maxDist,d;
  minDist = maxDist = p.distance(a);
  d = p.distance(b);
  if(d < minDist) minDist = d;
  else if(d > maxDist) maxDist = d;
  d = p.distance(c);
  if(d < minDist) minDist = d;
  else if(d > maxDist) maxDist = d;
  return (minDist <= Zero) && (maxDist >= Zero);
}

bool Triangle2D::intersects(const Plane2D& P, Segment2D& S) const
{
  Real d[3]; const Point2D* p[3] = {&a,&b,&c};
  for(int i=0;i<3;i++) d[i]=P.distance(*p[i]);
  //insertion sort
  for(int i=1;i<3;i++) {
    Real di=d[i];
    const Point2D* pi=p[i];
    int j=i;
    for(;j>0;j--) {
      if(d[j-1] <= di) break;
      d[j] = d[j-1];
      p[j] = p[j-1];
    }
    d[j] = di;
    p[j] = pi;
  }
  if(!(d[0] <= d[1] && d[1] <= d[2])) {
    LOG4CXX_INFO(KrisLibrary::logger(),"AAAACK: "<<d[0]<<" "<<d[1]<<" "<<d[2]);
  }
  assert(d[0] <= d[1] && d[1] <= d[2]);

  if(d[0] > Zero) return false;
  if(d[2] < Zero) return false;
  Real u;
  if(d[1] <= Zero) { //both 0 and 1 are inside p
    if(d[0] == d[2]) u = 0;
    else u = d[0]/(d[0]-d[2]);
    S.a = (One-u)*(*p[0]) + u*(*p[2]);
    if(d[1] == d[2]) u = 0;
    else u = d[1]/(d[1]-d[2]);
    S.b = (One-u)*(*p[1]) + u*(*p[2]);
  }
  else { //only 0 is inside p
    u = d[0]/(d[0]-d[1]);
    S.a = (One-u)*(*p[0]) + u*(*p[1]);
    u = d[0]/(d[0]-d[2]);
    S.b = (One-u)*(*p[0]) + u*(*p[2]);
  }
  return true;
}

bool Triangle2D::intersects(const Segment2D& s) const
{
  if(contains(s.a) || contains(s.b)) return true;
  Segment2D temp;
  temp.a = a; temp.b = b;
  if(temp.intersects(s)) return true;
  temp.a = b; temp.b = c;
  if(temp.intersects(s)) return true;
  temp.a = c; temp.b = a;
  if(temp.intersects(s)) return true;
  return false;
}

bool Triangle2D::intersects(const Triangle2D& t) const
{
  //either one triangle is contained in the other, or the edges intersect
  if(t.contains(a)) return true;
  if(contains(t.a)) return true;
  Segment2D s;
  s.a=t.a; s.b=t.b;
  if(intersects(s)) return true;
  s.a=t.b; s.b=t.c;
  if(intersects(s)) return true;
  s.a=t.c; s.b=t.a;
  if(intersects(s)) return true;
  return false;
}

/*
void Triangle2D::edgeIntersections(const Plane2D& P, Real u[3]) const
{
  Real da,db,dc;
  da=P.distance(a);
  db=P.distance(b);
  dc=P.distance(c);

  u[0] = SegmentZeroCrossing(da,db);
  u[1] = SegmentZeroCrossing(db,dc);
  u[2] = SegmentZeroCrossing(dc,da);
}

void Triangle2D::edgeIntersections(const Triangle2D& T, Real u[3]) const
{
  Plane2D PT;
  T.getPlane(PT);
  Real da,db,dc;
  da=PT.distance(a);
  db=PT.distance(b);
  dc=PT.distance(c);

  u[0]=u[1]=u[2]=-One;

  //check to see if these points are within T's boundaries
  Vector2 x; Vector2 U;
  Real ui;
  //edge a,b
  ui = SegmentZeroCrossing(da,db);
  if(ui >= Zero && ui <= One) {
    x.mul(a,One-ui); x.madd(b,ui);
    U = T.planeCoords(x);  
    if(containsPlaneCoords(U)) u[0] = ui;
  }

  //edge b,c
  ui = SegmentZeroCrossing(db,dc);
  if(ui >= Zero && ui <= One) {
    x.mul(b,One-ui); x.madd(c,ui);
    U = T.planeCoords(x);  
    if(containsPlaneCoords(U)) u[1] = ui;
  }

  //edge c,a
  ui = SegmentZeroCrossing(dc,da);
  if(ui >= Zero && ui <= One) {
    x.mul(c,One-ui); x.madd(a,ui);
    U = T.planeCoords(x);  
    if(containsPlaneCoords(U)) u[2] = ui;
  }
}
*/

void Triangle2D::getAABB(AABB2D& bb) const
{
	bb.setPoint(a);
	bb.expand(b);
	bb.expand(c);
}
