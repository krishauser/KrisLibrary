#include "Triangle3D.h"
#include "geometry3d.h"
#include "clip.h"
#include "interpolate.h"
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
	  //cout<<"Triangle3D::closestPointCoords(): Warning, triangle is actually a line!!!"<<endl;
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
    cout<<"Error with closestpointcoords routine!!!"<<endl;
    cout<<"x = "<<x<<endl;
    cout<<"pt = "<<pt<<endl;
    cout<<"pt2 = "<<pt2<<endl;
    cout<<"planecoords = "<<planeCoords(x)<<endl;
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
    printf ("AAAACK: %f %f %f\n",d[0],d[1],d[2]);
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
  if(!T.intersects(p,S)) return false;

  //now limit S to the boundaries of this triangle
  //find segment in plane coordinates (uA,vA)->(uB,vB)
  //clip against constraints u,v >= 0, u+v<=1
  Vector2 A,D;
  A = planeCoords(S.a);
  D = planeCoords(S.b)-A;
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
