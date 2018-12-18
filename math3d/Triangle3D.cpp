#include <KrisLibrary/Logger.h>
#include "Triangle3D.h"
#include "geometry3d.h"
#include "clip.h"
#include "interpolate.h"
#include <iostream>
#include <math/Interval.h>
using namespace Math3D;

Triangle3D::Triangle3D()
{}

Triangle3D::Triangle3D(const Vector3& _a,const Vector3& _b,const Vector3& _c)
:a(_a),b(_b),c(_c)
{}

void Triangle3D::set(const Vector3& _a,const Vector3& _b,const Vector3& _c)
{
  a=_a;
  b=_b;
  c=_c;
}

void Triangle3D::setTransformed(const Triangle3D& t, const Matrix4& xform)
{
  if(&t == this) {
    Triangle3D tmp=t;
    setTransformed(tmp,xform);
  }
  else {
    xform.mulPoint(t.a,a);
    xform.mulPoint(t.b,b);
    xform.mulPoint(t.c,c);
  }
}

const Vector3& Triangle3D::vertex(int v) const
{
  switch(v) {
  case 0: return a;
  case 1: return b;
  case 2: return c;
  }
  abort();
  return a;
}

Segment3D Triangle3D::edge(int e) const
{
  Segment3D s;
  switch(e) {
  case 0: s.a=b; s.b=c; break;
  case 1: s.a=c; s.b=a; break;
  case 2: s.a=a; s.b=b; break;
  default:
    abort();
  }
  return s;
}

bool Triangle3D::Read(File& f)
{
	if(!a.Read(f)) return false;
	if(!b.Read(f)) return false;
	if(!c.Read(f)) return false;
	return true;
}

bool Triangle3D::Write(File& f) const
{
	if(!a.Write(f)) return false;
	if(!b.Write(f)) return false;
	if(!c.Write(f)) return false;
	return true;
}

Vector3 Triangle3D::normal() const
{
	Vector3 z;
	z.setCross(b-a,c-a);
	z.inplaceNormalize();
	return z;
}

Vector3 Triangle3D::normal(const Point3D& a, const Point3D& b, const Point3D& c)
{
	Vector3 z;
	z.setCross(b-a,c-a);
	z.inplaceNormalize();
	return z;
}

Real Triangle3D::area() const
{
	Vector3 z;
	z.setCross(b-a,c-a);
	return z.length()*Half;
}

Real Triangle3D::area(const Point3D& a, const Point3D& b, const Point3D& c)
{
	Vector3 z;
	z.setCross(b-a,c-a);
	return z.length()*Half;
}

void Triangle3D::getPlane(Plane3D& p) const
{
	p.normal = normal();
	p.offset = dot(a,p.normal);
}

Vector3 Triangle3D::barycentricCoords(const Point3D& x) const
{
	return barycentricCoords(x,a,b,c);
}

Vector3 Triangle3D::barycentricCoords(const Vector3& x, const Point3D& a, const Point3D& b, const Point3D& c)
{
	Vector3 out;
	out.x = area(x,b,c);
	out.y = area(x,c,a);
	out.z = area(x,a,b);
	return out*PseudoInv(area(a,b,c));
}


Point3D Triangle3D::barycentricCoordsToPoint(const Vector3& bc) const
{
  return barycentricCoordsToPoint(bc,a,b,c);
}

Point3D Triangle3D::barycentricCoordsToPoint(const Vector3& bc, const Point3D& a, const Point3D& b, const Point3D& c)
{
  Point3D x;
  x.mul(a,bc.x);
  x.madd(b,bc.y);
  x.madd(c,bc.z);
  return x;
}

bool Triangle3D::containsBarycentricCoords(const Vector3& bc)
{
  return bc.x >= Zero && bc.y >= Zero && bc.z >= Zero && bc.x+bc.y+bc.z <= One;
}

Vector2 Triangle3D::planeCoords(const Point3D& x) const
{
	Real A,B,C,D,E;
	Vector3 e0=b-a,e1=c-a,x0=a-x;
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

Point3D Triangle3D::planeCoordsToPoint(const Vector2& pc) const
{
  return planeCoordsToPoint(pc,a,b,c);
}

Point3D Triangle3D::planeCoordsToPoint(const Vector2& pc,
		const Point3D& a, const Point3D& b, const Point3D& c)
{
  Point3D pos=a;
  pos.madd(b-a, pc.x);
  pos.madd(c-a, pc.y); 
  return pos;
}

bool Triangle3D::containsPlaneCoords(const Vector2& pc)
{
  return pc.x >= Zero && pc.y >= Zero && pc.x+pc.y <= One;
}

Vector2 Triangle3D::closestPointCoords(const Point3D& in) const
{
	Real A,B,C,D,E;
	Vector3 e1=b-a,e2=c-a,x0=a-in;
	A = dot(e1,e1);
	B = dot(e1,e2);
	C = dot(e2,e2);
	D = dot(e1,x0);
	E = dot(e2,x0);
	Real det = A*C-B*B;
	if(det == Zero) { //collapsed to a line
	  //LOG4CXX_WARN(KrisLibrary::logger(),"Triangle3D::closestPointCoords(): Warning, triangle is actually a line!!!");
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
	else if(pc.x+pc.y > det) { //check edge 3
	  //edge from b->c
	  //xloc=in-b=(in-a)+(b-a)=-x0-e1
	  //dir=c-b=e2-e1
		//t = -dot(e1+x0,e2-e1)/dot(e2-e1,e2-e1)
		//= -[dot(e1,e2) - dot(e1,e1) + dot(x0,e2) - dot(x0,e1)]/(dot(e2,e2) - 2dot(e2,e1) + dot(e1,e1)]
		//= -(B-A+E-D)/(C-2B+A)
		Real numer = -(B-A+E-D);
		Real denom = (A-Two*B+C);
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

Point3D Triangle3D::closestPoint(const Point3D& x) const
{
  Vector3 pt=planeCoordsToPoint(closestPointCoords(x));

  /*
  Vector2 pc=planeCoords(x);
  pc.x = Max(Zero,pc.x);
  pc.y = Max(Zero,pc.y);
  if(pc.x + pc.y > One)
    pc /= (pc.x+pc.y);
  Vector3 pt2 = planeCoordsToPoint(pc);
  if(x.distance(pt) > x.distance(pt2)+1e-6) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error with closestpointcoords routine!!!");
    LOG4CXX_INFO(KrisLibrary::logger(),"x = "<<x);
    LOG4CXX_INFO(KrisLibrary::logger(),"pt = "<<pt);
    LOG4CXX_INFO(KrisLibrary::logger(),"pt2 = "<<pt2);
    LOG4CXX_INFO(KrisLibrary::logger(),"planecoords = "<<planeCoords(x));
    abort();
  }
  */
  return pt;
}

bool Triangle3D::contains(const Point3D& x) const
{
  return containsPlaneCoords(planeCoords(x));
}

bool Triangle3D::rayIntersects(const Ray3D& ray, Real *t, Real *u, Real *v) const
{
	return rayIntersects(ray,a,b,c,t,u,v);
}

bool Triangle3D::rayIntersectsBackfaceCull(const Ray3D& ray, Real *t, Real *u, Real *v) const
{
	return rayIntersectsBackfaceCull(ray,a,b,c,t,u,v);
}


bool Triangle3D::intersects(const Segment3D& s, Real *t, Real *u, Real *v) const
{
  Real ttemp,utemp,vtemp;
  Ray3D r;
  r.source = s.a;
  r.direction = s.b-s.a;
  if(!rayIntersects(r,&ttemp,&utemp,&vtemp)) return false;
  if(ttemp > 1.0) return false;
  if(t) *t=ttemp;
  if(u) *u=utemp;
  if(v) *v=vtemp;
  return true;
}


//u,v are the coordinates in the triangle's frame, t is the dist
//non-culling
bool Triangle3D::rayIntersects(const Ray3D& ray,
                   const Point3D& a, const Point3D& b, const Point3D& c,
		   Real *t, Real *u, Real *v)
{
   Vector3 edge1, edge2, tvec, pvec, qvec;
   Real det,inv_det;

   /* find vectors for two edges sharing vert0 */
   edge1.sub(b, a);
   edge2.sub(c, a);

   /* begin calculating determinant - also used to calculate U parameter */
   pvec.setCross(ray.direction, edge2);

   /* if determinant is near zero, ray lies in plane of triangle */
   det = dot(edge1, pvec);

   if (FuzzyZero(det))
     return 0;
   inv_det = Inv(det);

   /* calculate distance from vert0 to ray origin */
   tvec.sub(ray.source, a);

   /* calculate U parameter and test bounds */
   *u = dot(tvec, pvec) * inv_det;
   if (*u < Zero || *u > One)
     return 0;

   /* prepare to test V parameter */
   qvec.setCross(tvec, edge1);

   /* calculate V parameter and test bounds */
   *v = dot(ray.direction, qvec) * inv_det;
   if (*v < Zero || *u + *v > One)
     return 0;

   /* calculate t, ray intersects triangle */
   *t = dot(edge2, qvec) * inv_det;

   return 1;
}

//u,v are the coordinates in the triangle's frame, t is the dist
//does backface culling
bool Triangle3D::rayIntersectsBackfaceCull(const Ray3D& ray,
                   const Point3D& a, const Point3D& b, const Point3D& c,
		   Real *t, Real *u, Real *v)
{
   Vector3 edge1, edge2, tvec, pvec, qvec;
   Real det,inv_det;

   /* find vectors for two edges sharing vert0 */
   edge1.sub(b, a);
   edge2.sub(c, a);

   /* begin calculating determinant - also used to calculate U parameter */
   pvec.setCross(ray.direction, edge2);

   /* if determinant is near zero, ray lies in plane of triangle */
   det = dot(edge1, pvec);

   if (det < Epsilon)
      return 0;

   /* calculate distance from vert0 to ray origin */
   tvec.sub(ray.source, a);

   /* calculate U parameter and test bounds */
   *u = dot(tvec, pvec);
   if (*u < Zero || *u > det)
      return 0;

   /* prepare to test V parameter */
   qvec.setCross(tvec, edge1);

    /* calculate V parameter and test bounds */
   *v = dot(ray.direction, qvec);
   if (*v < Zero || *u + *v > det)
      return 0;

   /* calculate t, scale parameters, ray intersects triangle */
   *t = dot(edge2, qvec);
   inv_det = Inv(det);
   *t *= inv_det;
   *u *= inv_det;
   *v *= inv_det;

   return 1;
}

bool Triangle3D::intersects(const Plane3D& p) const
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

bool Triangle3D::intersects(const Plane3D& P, Segment3D& S) const
{
  Real d[3]; const Point3D* p[3] = {&a,&b,&c};
  for(int i=0;i<3;i++) d[i]=P.distance(*p[i]);
  //insertion sort
  for(int i=1;i<3;i++) {
    Real di=d[i];
    const Point3D* pi=p[i];
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
    LOG4CXX_INFO(KrisLibrary::logger() ,"AAAACK: "<<d[0]<<" "<<d[1]<<" "<<d[2]);
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

bool Triangle3D::intersects(const Triangle3D& t) const
{
  Segment3D S;
  return intersects(t,S);
}

bool Triangle3D::intersects(const Triangle3D& T, Segment3D& S) const
{
  //intersect this plane with t
  Plane3D p;
  getPlane(p);
  if(p.normal.isZero()) return false;  //degenerate -- TODO: handle triangles that are just segments
  if(!IsFinite(p.normal.x)) return false; //degenerate
  if(!T.intersects(p,S)) return false;

  //now limit S to the boundaries of this triangle
  //find segment in plane coordinates (uA,vA)->(uB,vB)
  //clip against constraints u,v >= 0, u+v<=1
  Vector2 A,D;
  A = planeCoords(S.a);
  D = planeCoords(S.b)-A;
  if(A.isZero() && D.isZero()) return false; //degenerate
  Real tmin=Zero,tmax=One;
  //(u,v) = A+t*D
  //1) u >= 0 => -uA+t*(-uD) <= 0
  if(!ClipLine1D(-A.x,-D.x,tmin,tmax)) return false;
  //2) v >= 0 => -vA+t*(-vD) <= 0
  if(!ClipLine1D(-A.y,-D.y,tmin,tmax)) return false;
  //3) u+v <= 1 => (uA+vA-1)+t*(uD+vD) <= 0
  if(!ClipLine1D(A.x+A.y-One, D.x+D.y,tmin,tmax)) return false;
  Vector2 U=A; U.madd(D,tmin);
  S.a = planeCoordsToPoint(U);
  U=A; U.madd(D,tmax);
  S.b = planeCoordsToPoint(U);
  return true;
}

void Triangle3D::edgeIntersections(const Plane3D& P, Real u[3]) const
{
  Real da,db,dc;
  da=P.distance(a);
  db=P.distance(b);
  dc=P.distance(c);

  u[0] = SegmentZeroCrossing(da,db);
  u[1] = SegmentZeroCrossing(db,dc);
  u[2] = SegmentZeroCrossing(dc,da);
}

void Triangle3D::edgeIntersections(const Triangle3D& T, Real u[3]) const
{
  Plane3D PT;
  T.getPlane(PT);
  Real da,db,dc;
  da=PT.distance(a);
  db=PT.distance(b);
  dc=PT.distance(c);

  u[0]=u[1]=u[2]=-One;

  //check to see if these points are within T's boundaries
  Vector3 x; Vector2 U;
  Real ui;
  //edge a,b
  ui = SegmentZeroCrossing(da,db);
  if(ui >= Zero && ui <= One) {
    interpolate(a,b,ui,x);
    U = T.planeCoords(x);  
    if(containsPlaneCoords(U)) u[0] = ui;
  }

  //edge b,c
  ui = SegmentZeroCrossing(db,dc);
  if(ui >= Zero && ui <= One) {
    interpolate(b,c,ui,x);
    U = T.planeCoords(x);  
    if(containsPlaneCoords(U)) u[1] = ui;
  }

  //edge c,a
  ui = SegmentZeroCrossing(dc,da);
  if(ui >= Zero && ui <= One) {
    interpolate(c,a,ui,x);
    U = T.planeCoords(x);  
    if(containsPlaneCoords(U)) u[2] = ui;
  }
}

void Triangle3D::getAABB(AABB3D& bb) const
{
	bb.setPoint(a);
	bb.expand(b);
	bb.expand(c);
}

void PlaneExtents(const Triangle3D& tri,Plane3D& p,Real& dmin,Real& dmax)
{
  dmin=dmax=p.distance(tri.a);
  Real d=p.distance(tri.b);
  if(d<dmin) dmin=d;
  else if(d>dmax) dmax=d;
  d=p.distance(tri.c);
  if(d<dmin) dmin=d;
  else if(d>dmax) dmax=d;
}

using namespace std;

bool Triangle3D::intersects(const AABB3D& bb) const
{
  //trival accept: contains any point
  if(bb.contains(a)||bb.contains(b)||bb.contains(c)) return true;
  //trivial reject: bboxes don't intersect
  AABB3D tribb;
  getAABB(tribb);
  if(!bb.intersects(tribb)) {
    return false;
  }

  //check for other splitting planes
  Plane3D p;
  getPlane(p);
  if(!p.intersects(bb)) {
    return false;
  }

  //check planes orthogonal to edge of tri and edge of bb
  ClosedInterval bbInt,triInt;
  Vector3 edge;
  p.offset = Zero;
  //x dir
  edge.set(1,0,0);
  p.normal.setCross(b-a,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  p.normal.setCross(c-b,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  p.normal.setCross(a-c,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  //y dir
  edge.set(0,1,0);
  p.normal.setCross(b-a,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  p.normal.setCross(c-b,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  p.normal.setCross(a-c,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  //z dir
  edge.set(0,0,1);
  p.normal.setCross(b-a,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  p.normal.setCross(c-b,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  p.normal.setCross(a-c,edge);
  p.distanceLimits(bb,bbInt.a,bbInt.b);
  PlaneExtents(*this,p,triInt.a,triInt.b);
  if(!bbInt.intersects(triInt)) return false;
  return true;
}

Real Triangle3D::distance(const Triangle3D& other,Vector3& P,Vector3& Q) const
{
  ///source: PQP

  Vector3 S[3] = {a,b,c};
  Vector3 T[3] = {other.a,other.b,other.c};
  Vector3 Sv[3], Tv[3];
  Vector3 VEC;

  Sv[0].sub(b,a);
  Sv[1].sub(c,b);
  Sv[2].sub(a,c);
  Tv[0].sub(other.b,other.a);
  Tv[1].sub(other.c,other.b);
  Tv[2].sub(other.a,other.c);

  // For each edge pair, the vector connecting the closest points 
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of 
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  Vector3 V,Z,minP,minQ;
  Real mindd;
  int shown_disjoint = 0;

  mindd = S[0].distanceSquared(T[0]) + 1;  // Set first minimum safely high

  Segment3D s1,s2;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      // Find closest points on edges i & j, plus the 
      // vector (and distance squared) between these points
      s1.a = S[i];
      s1.b = Sv[i];
      s2.a = T[j];
      s2.b = Tv[j];
      Real t1,t2;
      s1.closestPoint(s2,t1,t2);
      s1.eval(t1,P);
      s2.eval(t2,Q);
      VEC.sub(Q,P);
      
      Real dd = VEC.dot(VEC);

      // Verify this closest point pair only if the distance 
      // squared is less than the minimum found thus far.

      if (dd <= mindd)
      {
        minP = P;
        minQ = Q;
        mindd = dd;

        Z.sub(S[(i+2)%3],P);
        Real a = Z.dot(VEC);
        Z.sub(T[(j+2)%3],Q);
        Real b = Z.dot(VEC);

        if ((a <= 0) && (b >= 0)) return sqrt(dd);

        Real p = V.dot(VEC);

        if (a < 0) a = 0;
        if (b > 0) b = 0;
        if ((p - a + b) > 0) shown_disjoint = 1;  
      }
    }
  }

  // No edge pairs contained the closest points.  
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the 
  //    triangle points are nearly colinear or coincident, one 
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  Vector3 Sn;
  Real Snl;       
  Sn.setCross(Sv[0],Sv[1]); // Compute normal to S triangle
  Snl = Sn.normSquared();      // Compute square of length of normal
  
  // If cross product is long enough,

  if (Snl > 1e-15)  
  {
    // Get projection lengths of T points

    Vector3 Tp; 

    Tp[0] = (a-other.a).dot(Sn);
    Tp[1] = (a-other.b).dot(Sn);
    Tp[2] = (a-other.c).dot(Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
    {
      if (Tp[0] < Tp[1]) point = 0; else point = 1;
      if (Tp[2] < Tp[point]) point = 2;
    }
    else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
    {
      if (Tp[0] > Tp[1]) point = 0; else point = 1;
      if (Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction, 

    if (point >= 0) 
    {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the 
      // other triangle, lies within the face.
    
      V.sub(T[point],a);
      Z.setCross(Sn,Sv[0]);
      if (V.dot(Z) > 0)
      {
        V.sub(T[point],S[1]);
        Z.setCross(Sn,Sv[1]);
        if (V.dot(Z) > 0)
        {
          V.sub(T[point],S[2]);
          Z.setCross(Sn,Sv[2]);
          if (V.dot(Z) > 0)
          {
            // T[point] passed the test - it's a closest point for 
            // the T triangle; the other point is on the face of S

            P=T[point];
            P.madd(Sn,Tp[point]/Snl);
            Q = T[point];
            return P.distance(Q);
          }
        }
      }
    }
  }

  Vector3 Tn;
  Real Tnl;       
  Tn.setCross(Tv[0],Tv[1]); 
  Tnl = Tn.normSquared();
  
  if (Tnl > 1e-15)  
  {
    Real Sp[3]; 

    Sp[0] = (T[0]-S[0]).dot(Tn);

    Sp[1] = (T[0]-S[1]).dot(Tn);

    Sp[2] = (T[0]-S[2]).dot(Tn);

    int point = -1;
    if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
    {
      if (Sp[0] < Sp[1]) point = 0; else point = 1;
      if (Sp[2] < Sp[point]) point = 2;
    }
    else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
    {
      if (Sp[0] > Sp[1]) point = 0; else point = 1;
      if (Sp[2] > Sp[point]) point = 2;
    }

    if (point >= 0) 
    { 
      shown_disjoint = 1;

      V.sub(S[point],T[0]);
      Z.setCross(Tn,Tv[0]);
      if (V.dot(Z) > 0)
      {
        V.sub(S[point],T[1]);
        Z.setCross(Tn,Tv[1]);
        if (V.dot(Z) > 0)
        {
          V.sub(S[point],T[2]);
          Z.setCross(Tn,Tv[2]);
          if (V.dot(Z) > 0)
          {
            P = S[point];
            Q=S[point];
            Q.madd(Tn,Sp[point]/Tnl);
            return P.distance(Q);
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2, 
  // that the triangles overlap.
  
  if (shown_disjoint)
  {
    P = minP;
    Q = minQ;
    return Sqrt(mindd);
  }
  else return 0;
}



namespace Math3D
{
  ostream& operator << (ostream& out,const Triangle3D& tri)
  {
    out<<tri.a<<"   "<<tri.b<<"   "<<tri.c;
    return out;
  }
  istream& operator >> (istream& in,Triangle3D& tri)
  {
    in>>tri.a>>tri.b>>tri.c;
    return in;
  }
}
