#include <KrisLibrary/Logger.h>
#include "Plane3D.h"
#include "geometry3d.h"
#include "LinearlyDependent.h"
#include <KrisLibrary/File.h>
#include <iostream>
#include <errors.h>
using namespace Math3D;
using namespace std;

bool Plane3D::Read(File& f)
{
	if(!normal.Read(f)) return false;
	if(!ReadFile(f,offset)) return false;
	return true;
}

bool Plane3D::Write(File& f) const
{
	if(!normal.Write(f)) return false;
	if(!WriteFile(f,offset)) return false;
	return true;
}

void Plane3D::Print(ostream& out) const
{
  out<<"["<<normal<<"].x = "<<offset;
}

void Plane3D::setPointNormal(const Point3D& a, const Vector3& n)
{
	normal.setNormalized(n);
	offset = dot(a,normal);
}

void Plane3D::setPointBases(const Point3D& a, const Vector3& b1, const Vector3& b2)
{
	normal.setCross(b1,b2);
	setPointNormal(a,normal);
}

void Plane3D::setPoints(const Point3D& a, const Point3D& b, const Point3D& c)
{
	setPointBases(a, b-a, c-a);
}

void Plane3D::setTransformed(const Plane3D& pin, const RigidTransform& xform)
{
	if(this == &pin) {
		Plane3D temp = pin;
		setTransformed(temp,xform);
		return;
	}
    xform.mulVector(pin.normal, normal);

    Vector3 v, v_out;
    v = pin.normal*pin.offset;
    xform.mulPoint(v, v_out);
    setPointNormal(v_out, normal);
}

void Plane3D::setTransformed(const Plane3D& pin, const Matrix4& xform)
{
	if(this == &pin) {
		Plane3D temp = pin;
		setTransformed(temp,xform);
		return;
	}
  LOG4CXX_ERROR(KrisLibrary::logger(),"Using slow version of Plane3D::setTransformed, may want to use RigidTransform version");
  KrisLibrary::loggerWait();
  Matrix4 xformInv;
  if(xformInv.setInverse(xform)) {
    xformInv.inplaceTranspose();
    Vector4 nhomog(pin.normal),nhomogNew;
    nhomog.w = -pin.offset;
    xformInv.mul(nhomog,nhomogNew);
    normal.set(nhomogNew.x,nhomogNew.y,nhomogNew.z);
    offset = -nhomogNew.w;

    //normalize
    Real nnorm = normal.norm();
    normal /= nnorm;
    offset /= nnorm;

    /* old-fashioned way
    xformInv.mulVector(pin.normal, normal);

    Vector3 v, v_out;
    v = pin.normal*pin.offset;
    xform.mulPoint(v, v_out);
    setPointNormal(v_out, normal);
    */
  }
  else {
    FatalError("TODO: Plane3D::setTransformed for degenerate matrix");
  }
}

//returns the orthogonal distance from the plane to the point
Real Plane3D::distance(const Point3D& v) const
{
	return dot(v,normal) - offset;
}

//projects a Vector3 onto this plane
void Plane3D::project(const Point3D& vin, Point3D& vout) const
{
	vout = vin - normal*distance(vin);
}

void Plane3D::getBasis(Vector3& xb, Vector3& yb) const
{
	if(FuzzyZero(normal.x) && FuzzyZero(normal.y)) {
		xb.set(1,0,0);
		yb.set(0,1,0);
		return;
	}
	yb.set(0,0,1);
	xb.setCross(normal,yb);
	xb.inplaceNormalize();
	yb.setCross(normal,xb);
	yb.inplaceNormalize();
}


bool Plane3D::intersectsSegment(const Segment3D& s, Real* t)
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

bool Plane3D::intersectsLine(const Line3D& l, Real* t)
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

bool Plane3D::intersectsRay(const Ray3D& r, Real* t)
{
	//if source on - side, intersects if dir is +
	//if source on + side, intersects if dir is -
	Real src = distance(r.source);
	Real dir = dot(normal,r.direction);
	if(src < Zero) {
		if(dir > Zero) {
			if(t) *t = -src/dir;
			return true;
		}
	}
	else if(src > Zero) {
		if(dir < Zero) {
			if(t) *t = -src/dir;
			return true;
		}
	}
	else {
		if(t) *t = Zero;
		return true;
	}
	return false;
}

void Plane3D::distanceLimits(const AABB3D& bb,Real& dmin,Real& dmax) const
{
	Vector3 vmin,vmax;
	//get the extreme points of this box relative to the plane's normal
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
	if(normal.z > Zero)
	{
		vmin.z = bb.bmin.z;
		vmax.z = bb.bmax.z;
	}
	else
	{
		vmin.z = bb.bmax.z;
		vmax.z = bb.bmin.z;
	}
	dmin = distance(vmin);
	dmax = distance(vmax);
}

bool Plane3D::intersects(const AABB3D& bb) const
{
  Real dmin,dmax;
  distanceLimits(bb,dmin,dmax);
  //intersects if dmin,dmax straddle zero (inclusive)
  return (dmin <= Zero) && (dmax >= Zero);
}

bool Plane3D::intersectsInterior(const AABB3D& bb) const
{
  Real dmin,dmax;
  distanceLimits(bb,dmin,dmax);
  //intersects if the extreme points are strictly on opposite sides of the plane
  return (dmin < Zero) && (dmax > Zero);
}

int Plane3D::allIntersections(const Plane3D& p,Line3D& l) const
{
  //n1.x = o1, n2.x = o2 is a 2x3 system
  //if n1 and n2 aren't linearly dependent, find the l that satisfies this
  //otherwise either return 0 or 2

  //linearly dependent if there exists a c s.t. c*n1 = n2 (+eps) or vice versa
  Real eps = 1e-3;
  Real c; bool cb;
  if(LinearlyDependent_Robust(normal,p.normal,c,cb,eps)) {
    if(cb) {  //normal = c*p.normal const
      if(FuzzyEquals(offset,c*p.offset,eps)) return 2;
      else return 0;
    }
    else {  //c*normal = p.normal
      if(FuzzyEquals(c*offset,p.offset,eps)) return 2;
      else return 0;
    }
  }
  else {
    //find x = x0 + t*dx
    //we have dx.n1 = dx.n2 = 0
    l.direction.setCross(normal,p.normal);
    l.direction.inplaceNormalize();

    //and x0 = u*n1 + v*n2
    //with n1.x0 = o1 and n2.x0 = o2
    Matrix2 N;  Vector2 o(offset,p.offset), u;
    N(0,0) = normal.normSquared();
    N(0,1) = N(1,0) = normal.dot(p.normal);
    N(1,1) = p.normal.normSquared();
    if(FuzzyZero(N.determinant())) {
      //LOG4CXX_ERROR(KrisLibrary::logger(),"That's strange, I thought these weren't linearly dependent");
      return 0;
    }
    N.inplaceInverse();
    N.mul(o,u);
    l.source.mul(normal,u.x);
    l.source.madd(p.normal,u.y);
    /*
    if(!Fuzzyzero(distance(l.source),Epsilon)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Hmm... plane-plane intersection has a weird result: ");
      LOG4CXX_ERROR(KrisLibrary::logger(),"distance to line point "<<l.source<<" is "<<distance(l.source));
      LOG4CXX_ERROR(KrisLibrary::logger(),"determinant of normal equations: "<<N.determinant());
      LOG4CXX_INFO(KrisLibrary::logger(),"p1 : normal "<<normal<<" offset "<<offset);
      LOG4CXX_INFO(KrisLibrary::logger(),"p2 : normal "<<p.normal<<" offset "<<p.offset);
      KrisLibrary::loggerWait();
    }
    if(!FuzzyZero(p.distance(l.source),Epsilon)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Hmm... plane-plane intersection has a weird result: ");
      LOG4CXX_ERROR(KrisLibrary::logger(),"distance to line point "<<l.source<<" is "<<distance(l.source));
      LOG4CXX_ERROR(KrisLibrary::logger(),"determinant of normal equations: "<<N.determinant());
      LOG4CXX_INFO(KrisLibrary::logger(),"p1 : normal "<<normal<<" offset "<<offset);
      LOG4CXX_INFO(KrisLibrary::logger(),"p2 : normal "<<p.normal<<" offset "<<p.offset);
      KrisLibrary::loggerWait();
    }
    assert(FuzzyZero(distance(l.source),Epsilon));
    assert(FuzzyZero(p.distance(l.source),Epsilon));
    */
  }
  return 1;
}


namespace Math3D 
{
  ostream& operator << (ostream& out,const Plane3D& p)
  {
    out<<p.normal<<"  "<<p.offset;
    return out;
  }
  istream& operator >> (istream& in,Plane3D& p)
  {
    in>>p.normal>>p.offset;
    return in;
  }
}
