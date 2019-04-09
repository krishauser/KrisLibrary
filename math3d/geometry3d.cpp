#include <KrisLibrary/Logger.h>
#include "geometry3d.h"
#include "AABB2D.h"
#include "Polygon2D.h"
#include "interpolate.h"
#include "misc.h"
#include "basis.h"
#include <iostream>
#include <KrisLibrary/meshing/TriMesh.h>
#include <KrisLibrary/math/misc.h>
#include <KrisLibrary/File.h>
#include <string>
using namespace std;

namespace Math3D {


bool Sphere3D::Read(File& f)
{
	if(!center.Read(f)) return false;
	if(!ReadFile(f,radius)) return false;
	return true;
}

bool Sphere3D::Write(File& f) const
{
	if(!center.Write(f)) return false;
	if(!WriteFile(f,radius)) return false;
	return true;
}

Real Sphere3D::distance(const Point3D& v) const
{
	return (center-v).norm() - radius;
}

Real Sphere3D::signedDistance(const Point3D& v) const
{
  return (center-v).norm() - radius;
}

bool Sphere3D::contains(const Point3D& v) const
{
	return DistanceLEQ(center,v,radius);
}

bool Sphere3D::contains(const Sphere3D& s) const
{
	return DistanceLEQ(center,s.center,radius-s.radius);
}

bool Sphere3D::withinDistance(const Point3D& v, Real dist) const
{
	return DistanceLEQ(center,v, radius + dist);
}

Real Sphere3D::closestPoint(const Point3D& in, Point3D& out) const
{
  Vector3 v=(in-center);
  Real d = v.norm();
  if(d < radius) { out = in; return 0; }
  out = center + v*(radius/d);
  return d - radius;
}

bool Sphere3D::boundaryWithinDistance(const Point3D& v, Real dist) const
{
    return Abs((center-v).norm()-radius) <= dist;
}

bool Sphere3D::intersects(const Segment3D& s, Real* t1, Real* t2) const
{
  Real u1,u2;
  Line3D l;
  l.source = s.a;
  l.direction = s.b-s.a;
  if(!intersects(l,&u1,&u2)) return false;
  if(u1 > 1.0 || u2 < 0.0) return false;
  u2 = Min(u2,1.0);
  u1 = Max(u1,0.0);
  if(t1) *t1=u1;
  if(t2) *t2=u2;
  return true;
}


bool Sphere3D::intersects(const Line3D& l, Real* t1, Real* t2) const
{
	Vector3 offset=center-l.source;
	Real o_o=dot(offset,offset), o_b=dot(offset,l.direction), b_b=dot(l.direction,l.direction);
	//so we know there's a root to |offset-t*b|==r
	//o.o-2t*b.o+t^2*b.b=r^2
	Real a,b,c;
	a=b_b;
	b=-Two*o_b;
	c=o_o-radius*radius;
	if(b_b == Zero) {
	  if(c < Zero) {
	    if(t1 && t2) { *t1=-Inf; *t2=Inf; }
	    return true;
	  }
	}
	Real x1,x2;
	int res=quadratic(a,b,c,x1,x2);
	if(res<=0) return false;
	if(res==1) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Whoa, line just intersects at one point on the sphere");
	  //LOG4CXX_INFO(KrisLibrary::logger(),"l= "<<l.source<<"->"<<l.direction);
	  //LOG4CXX_INFO(KrisLibrary::logger(),"c= "<<center<<", r="<<radius);
	  //LOG4CXX_INFO(KrisLibrary::logger(),"t="<<x1);
	  //KrisLibrary::loggerWait();
	  x2=x1;
	}
	if(x1 > x2) Swap(x1,x2);
	if(t1 && t2) {
		*t1=x1;
		*t2=x2;
	}
	return true;
}

bool Sphere3D::intersects(const Plane3D& p) const
{
  return Abs(p.distance(center)) <= radius;
}

bool Sphere3D::intersects(const Sphere3D& s) const
{
  return ballsIntersect(center,radius,s.center,s.radius);
}

bool Sphere3D::boundaryIntersects(const Sphere3D& s) const
{
  return ballSphereIntersect(s.center,s.radius,center,radius);
}

bool Sphere3D::boundaryIntersectsBoundary(const Sphere3D& s) const
{
  return spheresIntersect(center,radius,s.center,s.radius);
}

bool Sphere3D::ballsIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb)
{
	return DistanceLEQ(ca,cb,ra+rb);
}

bool Sphere3D::ballSphereIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb)
{
  Real r2 = (ca-cb).normSquared();
  if(r2 <= Sqr(ra+rb)) {
    Real r = Sqrt(r2);
    if(r + ra < rb) return false;
    return true;
  }
  return false;
}

bool Sphere3D::spheresIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb)
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

void Sphere3D::getAABB(AABB3D& bb) const
{
  bb.setPoint(center);
  bb.bmin.x-=radius; bb.bmin.y-=radius; bb.bmin.z-=radius;
  bb.bmax.x+=radius; bb.bmax.y+=radius; bb.bmax.z+=radius;
}

bool Sphere3D::intersects(const AABB3D& bb) const
{
  Vector3 temp;
  return bb.distanceSquared(center,temp) < Sqr(radius);
}



bool Ellipsoid3D::contains(const Point3D& pt) const
{
	Point3D out;
	toLocalNormalized(pt,out);
	return NormLEQ(out,One);
}

bool Ellipsoid3D::intersects(const Line3D& l, Real* t1, Real* t2) const
{
	Line3D llocal;
	toLocalNormalized(l,llocal);
	Sphere3D s;
	s.center.setZero();
	s.radius = One;
	return s.intersects(llocal,t1,t2);
}


bool Ellipsoid3D::intersects(const Segment3D& seg, Real* t1, Real* t2) const
{
	Segment3D slocal;
	toLocalNormalized(seg,slocal);
	Sphere3D s;
	s.center.setZero();
	s.radius = One;
	return s.intersects(slocal,t1,t2);
}

void Ellipsoid3D::getAABB(AABB3D& bb) const
{
	//get the bases of world space in ellipsoid space
	Vector3 xb,yb,zb;
	xb.x = xbasis.x;
	xb.y = ybasis.x;
	xb.z = zbasis.x;
	yb.x = xbasis.y;
	yb.y = ybasis.y;
	yb.z = zbasis.y;
	zb.x = xbasis.z;
	zb.y = ybasis.z;
	zb.z = zbasis.z;

	normalize(xb,xb);
	normalize(yb,yb);
	normalize(zb,zb);

	//now find the points on the sphere with the correct tangent planes
	Vector3 xt,yt,zt;
	xt = cross(yb,zb);
	yt = cross(zb,xb);
	zt = cross(xb,yb);

	//these are the normals, just normalize them
	xt.inplaceNormalize();
	yt.inplaceNormalize();
	zt.inplaceNormalize();

	xb = xbasis * dims.x;
	yb = ybasis * dims.y;
	zb = zbasis * dims.z;

	//aliases
	Vector3& bmin=bb.bmin, &bmax = bb.bmax;

	//take these points back to world coordinates- these will be the min and max points
	bmax.x = bmin.x = xt.x * xb.x + xt.y * yb.x + xt.z * zb.x;
	bmax.y = bmin.y = yt.x * xb.y + yt.y * yb.y + yt.z * zb.y;
	bmax.z = bmin.z = zt.x * xb.z + zt.y * yb.z + zt.z * zb.z;

	if(bmax.x < 0)
		bmax.x = -bmax.x;
	else
		bmin.x = -bmin.x;

	if(bmax.y < 0)
		bmax.y = -bmax.y;
	else
		bmin.y = -bmin.y;

	if(bmax.z < 0)
		bmax.z = -bmax.z;
	else
		bmin.z = -bmin.z;

	bmax += origin;
	bmin += origin;
}

std::ostream& operator << (std::ostream& out,const Ellipsoid3D& b)
{
  out<<b.origin<<"  "<<b.xbasis<<"  "<<b.ybasis<<"  "<<b.zbasis<<"  "<<b.dims;
  return out;
}

std::istream& operator >> (std::istream& in, Ellipsoid3D& b)
{
  in>>b.origin>>b.xbasis>>b.ybasis>>b.zbasis>>b.dims;
  return in;
}







const char* GeometricPrimitive3D::TypeName(Type type)
{
  switch(type) {
  case Point:
    return "point";
  case Segment:
    return "segment";
  case Triangle:
    return "triangle";
  case Polygon:
    return "polygon";
  case Sphere:
    return "sphere";
  case AABB:
    return "aabb";
  case Box:
    return "box";
  case Cylinder:
    return "cylinder";
  case Empty:
    return "empty";
  default:
    return "error";
  }
}

GeometricPrimitive3D::GeometricPrimitive3D()
  :type(Empty)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const GeometricPrimitive3D& prim)
  :type(prim.type),data(prim.data)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Vector3& pt)
  :type(Point),data(pt)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Segment3D& line)
  :type(Segment),data(line)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Triangle3D& t)
  :type(Triangle),data(t)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Polygon3D& p)
  :type(Polygon),data(p)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Sphere3D& s)
  :type(Sphere),data(s)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Ellipsoid3D& s)
  :type(Ellipsoid),data(s)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const Cylinder3D& s)
  :type(Cylinder),data(s)
{}

GeometricPrimitive3D::GeometricPrimitive3D(const AABB3D& s)
  :type(AABB),data(s)
{}


GeometricPrimitive3D::GeometricPrimitive3D(const Box3D& s)
  :type(Box),data(s)
{}

AABB3D GeometricPrimitive3D::GetAABB() const
{
  AABB3D bb;
  switch(type) {
  case Empty:
    bb.minimize();
    return bb;
  case Point:
    {
      const Vector3* p = AnyCast_Raw<Vector3>(&data);
      return AABB3D(*p,*p);
    }
  case Segment:
    AnyCast_Raw<Segment3D>(&data)->getAABB(bb);
    return bb;
  case Triangle:
    AnyCast_Raw<Triangle3D>(&data)->getAABB(bb);
    return bb;
  case Polygon:
    AnyCast_Raw<Polygon3D>(&data)->getAABB(bb);
    return bb;
  case Sphere:
    AnyCast_Raw<Sphere3D>(&data)->getAABB(bb);
    return bb;
  case AABB:
    bb = *AnyCast_Raw<AABB3D>(&data);
    return bb;
  case Box:
    AnyCast_Raw<Box3D>(&data)->getAABB(bb);
    return bb;
  case Cylinder:
    AnyCast_Raw<Cylinder3D>(&data)->getAABB(bb);
    return bb;
  default:
    FatalError("Invalid primitive type");
    bb.minimize();
    return bb;
  }
}

Box3D GeometricPrimitive3D::GetBB() const
{
  Box3D bb;
  bb.origin.setZero();
  bb.xbasis.set(1,0,0);
  bb.ybasis.set(0,1,0);
  bb.zbasis.set(0,0,1);
  bb.dims.set(-1.0);
  switch(type) {
  case Point:
    {
      const Vector3* p = AnyCast_Raw<Vector3>(&data);
      bb.origin = *p;
      bb.dims.setZero();
      return bb;
    }
  case Segment:
    {
      const Segment3D* s=AnyCast_Raw<Segment3D>(&data);
      bb.origin = s->a;
      bb.xbasis = s->b-s->a;
      bb.dims.set(bb.xbasis.norm(),0,0);
      bb.xbasis /= bb.dims.x;
      GetCanonicalBasis(bb.xbasis,bb.ybasis,bb.zbasis);
    }
    return bb;
  case Triangle:
    {
      const Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      bb.zbasis = t->normal();
      bb.dims.z = 0;
      //find longest edge
      int e=0;
      Real maxL = t->b.distance(t->a);
      Real l=t->c.distance(t->b);
      if(l > maxL) { maxL = l; e=1; }
      l=t->a.distance(t->c);
      if(l > maxL) { maxL = l; e=2; }
      //compute frame
      bb.origin = t->vertex(e);
      bb.xbasis = t->vertex((e+1)%3)-t->vertex(e);
      bb.dims.x = bb.xbasis.length();
      bb.xbasis /= bb.dims.x;
      bb.ybasis.setCross(bb.zbasis,bb.xbasis);
      bb.dims.y = bb.ybasis.dot(t->vertex((e+2)%3)-t->vertex(e));
      if(bb.dims.y < 0) {
	bb.dims.y *= -1;
	bb.ybasis *= -1;
      }
    }
    return bb;
  case Polygon:
    {
      Polygon2D p;
      Matrix4 T;
      AnyCast_Raw<Polygon3D>(&data)->getPlanarPolygon(p,T);
      T.get(bb.xbasis,bb.ybasis,bb.zbasis,bb.origin);
      AABB2D bb2;
      p.getAABB(bb2);
      bb.origin = bb.origin - bb.xbasis*bb2.bmin.x - bb.ybasis*bb2.bmin.y;
      bb.dims.x = bb2.bmax.x-bb2.bmin.x;
      bb.dims.y = bb2.bmax.y-bb2.bmin.y;
      bb.dims.z = 0;
    }
    return bb;
  case Sphere:
    {
      const Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      bb.origin = s->center - Vector3(s->radius,s->radius,s->radius);
      bb.dims = 2.0*Vector3(s->radius,s->radius,s->radius);
    }
    return bb;
  case AABB:
    {
      const AABB3D* b = AnyCast_Raw<AABB3D>(&data);
      bb.xbasis.set(1,0,0);
      bb.ybasis.set(0,1,0);
      bb.zbasis.set(0,0,1);
      bb.origin = b->bmin;
      bb.dims = b->bmax-b->bmin;
    }
    return bb;
  case Box:
    bb = *AnyCast_Raw<Box3D>(&data);
    return bb;
  case Cylinder:
    {
      const Cylinder3D* c=AnyCast_Raw<Cylinder3D>(&data);
      bb.xbasis = c->axis;
      bb.dims.x = c->height;
      bb.dims.y = bb.dims.z = c->radius*2.0;
      GetCanonicalBasis(bb.xbasis,bb.ybasis,bb.zbasis);
      bb.origin = c->center - bb.ybasis*c->radius - bb.zbasis*c->radius;
    }
    return bb;
  default:
    FatalError("Invalid primitive type");
    return bb;
  }
}

RigidTransform GeometricPrimitive3D::GetFrame() const
{
  Box3D bb = GetBB();
  return RigidTransform(bb.xbasis,bb.ybasis,bb.zbasis,bb.center());
}

void GeometricPrimitive3D::Transform(const RigidTransform& T)
{
  switch(type) {
  case Empty:
    break;
  case Point:
    {
      Vector3* p = AnyCast_Raw<Vector3>(&data);
      *p = T*(*p);
    }
    break;
  case Segment:
    {
      Segment3D* s=AnyCast_Raw<Segment3D>(&data);
      s->a = T*s->a;
      s->b = T*s->b;
    }
    break;
  case Triangle:
    {
      Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      t->setTransformed(*t,T);
    }
    break;
  case Polygon:
    {
      Polygon3D* p=AnyCast_Raw<Polygon3D>(&data);
      p->setTransformed(*p,T);
    }
    break;
  case Sphere:
    {
      Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      s->center = T*s->center;
    }
    break;
  case AABB:
    {
      Box3D b = GetBB();
      b.setTransformed(b,T);
      type = Box;
      data = b;
    }
    break;
  case Box:
    {
      Box3D* b = AnyCast_Raw<Box3D>(&data);
      b->setTransformed(*b,T);
    }
    break;
  case Cylinder:
    {
      Cylinder3D* c=AnyCast_Raw<Cylinder3D>(&data);
      c->setTransformed(*c,T);
    }
    break;
  default:
    FatalError("Invalid primitive type");
    break;
  }
}

void GeometricPrimitive3D::Transform(const Matrix4& T)
{
  /*
  bool rotation = (T(0,1)!=0||T(0,2)!=0||T(1,2)!=0||T(1,0)!=0||T(2,0)!=0||T(2,1)!=0);
  Matrix3 R,temp;
  T.get(R);
  temp.mulTransposeB(R,R);
  bool scale = (!FuzzyEquals(temp(0,0),1.0) || !FuzzyEquals(temp(1,1),1.0) || !FuzzyEquals(temp(2,2),1.0))
  bool nonuniform = (!FuzzyEquals(temp(0,0),temp(1,1)) || !FuzzyEquals(temp(1,1),temp(2,2)))
  */

  switch(type) {
  case Empty:
    break;
  case Point:
    {
      Vector3* p = AnyCast_Raw<Vector3>(&data);
      Vector3 temp=*p;
      T.mulPoint(temp,*p);
    }
    break;
  case Segment:
    {
      Segment3D* s=AnyCast_Raw<Segment3D>(&data);
      Vector3 temp=s->a;
      T.mulPoint(temp,s->a);
      temp=s->b;
      T.mulPoint(temp,s->b);
    }
    break;
  case Triangle:
    {
      Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      t->setTransformed(*t,T);
    }
    break;
  case Polygon:
    {
      Polygon3D* p=AnyCast_Raw<Polygon3D>(&data);
      p->setTransformed(*p,T);
    }
    break;
  case Sphere:
    {
      Matrix3 R,temp;
      T.get(R);
      temp.mulTransposeB(R,R);
      bool nonuniform = (!FuzzyEquals(temp(0,0),temp(1,1)) || !FuzzyEquals(temp(1,1),temp(2,2)));
      if(nonuniform) {
	//convert to ellipsoid type
	FatalError("Can't yet convert spheres to ellipsoids\n");
      }

      Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      Vector3 ctemp=s->center;
      T.mulPoint(ctemp,s->center);
      s->radius *= Sqrt(temp(0,0));
    }
    break;
  case AABB:
    {
      Matrix3 R,temp;
      T.get(R);
      temp.mulTransposeB(R,R);
      bool scale = (!FuzzyEquals(temp(0,0),1.0) || !FuzzyEquals(temp(1,1),1.0) || !FuzzyEquals(temp(2,2),1.0));
      if(scale) {
	FatalError("Can't yet scale / transform AABBs\n");
      }
      Box3D b = GetBB();
      b.setTransformed(b,T);
      type = Box;
      data = b;
    }
    break;
  case Box:
    {
      Matrix3 R,temp;
      T.get(R);
      temp.mulTransposeB(R,R);
      bool scale = (!FuzzyEquals(temp(0,0),1.0) || !FuzzyEquals(temp(1,1),1.0) || !FuzzyEquals(temp(2,2),1.0));
      if(scale) {
	FatalError("Can't yet scale / transform Box's\n");
      }
      Box3D* b = AnyCast_Raw<Box3D>(&data);
      b->setTransformed(*b,T);
    }
    break;
  case Cylinder:
    {
      Matrix3 R,temp;
      T.get(R);
      temp.mulTransposeB(R,R);
      bool scale = (!FuzzyEquals(temp(0,0),1.0) || !FuzzyEquals(temp(1,1),1.0) || !FuzzyEquals(temp(2,2),1.0));
      if(scale) {
	FatalError("Can't yet scale / transform Cylinders\n");
      }
      Cylinder3D* c=AnyCast_Raw<Cylinder3D>(&data);
      c->setTransformed(*c,T);
    }
    break;
  default:
    FatalError("Invalid primitive type");
    break;
  }
}

vector<double> GeometricPrimitive3D::ClosestPointParameters(const Vector3& pt) const
{
  vector<double> res;
  switch(type) {
  case Point:
    return res;
  case Segment:
    {
      res.resize(1);
      const Segment3D* s=AnyCast_Raw<Segment3D>(&data);
      res[0] = s->closestPointParameter(pt);
    }
    return res;
  case Triangle:
    {
      const Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      Vector2 pc = t->closestPointCoords(pt);
      res.resize(2);
      res[0] = pc.x;
      res[1] = pc.y;
    }
    return res;
  case Polygon:
    return res;
  case Sphere:
    {
      const Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      Vector3 cp;
      s->closestPoint(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);
    }
    return res;
  case AABB:
    {
      const AABB3D* b = AnyCast_Raw<AABB3D>(&data);
      Vector3 cp;
      b->distanceSquared(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);      
    }
    return res;
  case Box:
    {
      const Box3D* b = AnyCast_Raw<Box3D>(&data);
      Vector3 cp;
      b->distanceSquared(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);      
    }
    return res;
  case Cylinder:
    {
      const Cylinder3D* c=AnyCast_Raw<Cylinder3D>(&data);
      Vector3 cp;
      c->closestPoint(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);
    }
    return res;
  default:
    FatalError("Invalid primitive type");
    return res;
  }
}

Vector3 GeometricPrimitive3D::ParametersToPoint(const vector<double>& params) const
{
  switch(type) {
  case Point:
    return *AnyCast_Raw<Vector3>(&data);
  case Segment:
    {
      Assert(params.size()==1);
      const Segment3D* s=AnyCast_Raw<Segment3D>(&data);
      return s->a + params[0]*(s->b-s->a);
    }
  case Triangle:
    {
      const Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      return t->planeCoordsToPoint(Vector2(params[0],params[1]));
    }
  case Sphere:
  case AABB:
  case Box:
  case Cylinder:
    Assert(params.size()==3);
    return Vector3(params[0],params[1],params[2]);
  default:
    FatalError("Invalid primitive type");
    return Vector3(0.0);
  }
}

Vector3 GeometricPrimitive3D::ParametersToNormal(const vector<double>& params) const
{
  FatalError("Not done yet");
  return Vector3(0.0);
}

bool GeometricPrimitive3D::RayCast(const Ray3D& ray,Vector3& pt) const
{
  switch(type) {
  case Point:
    if(ray.distance(*AnyCast_Raw<Vector3>(&data)) < Epsilon) {
      pt = *AnyCast_Raw<Vector3>(&data);
      return true;
    }
    return false;
  case Segment:
        LOG4CXX_ERROR(KrisLibrary::logger(),"Segment ray cast not done yet\n");
    return false;
  case Sphere:
    {
      const Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      Real t,u;
      if(s->intersects(ray,&t,&u)) {
	if(u >= 0) {
	  if(t < 0) t = 0;
	  ray.eval(t,pt);
	  return true;
	}
      }
      return false;
    }
  case Ellipsoid:
    {
      const Ellipsoid3D* s=AnyCast_Raw<Ellipsoid3D>(&data);
      Real t,u;
      if(s->intersects(ray,&t,&u)) {
	if(u >= 0) {
	  if(t < 0) t = 0;
	  ray.eval(t,pt);
	  return true;
	}
      }
      return false;
    }
  case Cylinder:
    {
      const Cylinder3D* s=AnyCast_Raw<Cylinder3D>(&data);
      Real t,u;
      if(s->intersects(ray,&t,&u)) {
	if(u >= 0) {
	  if(t < 0) t = 0;
	  ray.eval(t,pt);
	  return true;
	}
      }
      return false;
    }
  case AABB:
    {
      Real tmin=0,tmax=Inf;
      Line3D l=ray;
      if(l.intersects(*AnyCast_Raw<AABB3D>(&data),tmin,tmax)) {
	ray.eval(tmin,pt);
	return true;
      }
      return false;
    }
  case Box:
    {
      const Box3D* b=AnyCast_Raw<Box3D>(&data);
      RigidTransform T_world_box;
      b->getTransformInv(T_world_box);
      Ray3D rlocal; rlocal.setTransformed(ray,T_world_box);
      AABB3D localbox; localbox.bmin.setZero(); localbox.bmax = b->dims;
      Real tmin=0,tmax=Inf;
      Line3D l=rlocal;
      if(l.intersects(localbox,tmin,tmax)) {
	ray.eval(tmin,pt);
	return true;
      }
      return false;
    }
  case Triangle:
    {
      Real t,u,v;
      if(AnyCast_Raw<Triangle3D>(&data)->rayIntersects(ray,&t,&u,&v)) {
	ray.eval(t,pt);
	return true;
      }
      return false;
    }
  default:
    return false;
  }
}




bool GeometricPrimitive3D::SupportsCollides(Type a,Type b)
{
  if(a==Point || b==Segment)
    return (b==Point || b==Segment || b==Sphere || b==Ellipsoid || b==Cylinder || b==AABB || b==Box || b==Triangle);
  if(a==AABB || a==Box || a==Triangle)
    return (b==Point || b==Segment || b==Sphere || b==AABB || b==Box || b==Triangle);
  if(a==Sphere) return SupportsDistance(a,b);
  return false;
}

bool GeometricPrimitive3D::Collides(const GeometricPrimitive3D& geom) const
{
  switch(type) {
  case Point:
    return geom.Collides(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    return geom.Collides(*AnyCast_Raw<Segment3D>(&data));
  case Sphere:
    return geom.Collides(*AnyCast_Raw<Sphere3D>(&data));
  case Ellipsoid:
    return geom.Collides(*AnyCast_Raw<Ellipsoid3D>(&data));
  case Cylinder:
    return geom.Collides(*AnyCast_Raw<Cylinder3D>(&data));
  case AABB:
    return geom.Collides(*AnyCast_Raw<AABB3D>(&data));
  case Box:
    return geom.Collides(*AnyCast_Raw<Box3D>(&data));
  case Triangle:
    return geom.Collides(*AnyCast_Raw<Triangle3D>(&data));
  case Polygon:
    return geom.Collides(*AnyCast_Raw<Polygon3D>(&data));
  default:
    return false;
  }
}

bool GeometricPrimitive3D::Collides(const Vector3& point) const
{
  switch(type) {
  case Point:
    return *AnyCast_Raw<Vector3>(&data) == point;
  case Segment:
    return AnyCast_Raw<Segment3D>(&data)->distance(point) == 0;
  case Sphere:
    return AnyCast_Raw<Sphere3D>(&data)->contains(point);
  case Ellipsoid:
    return AnyCast_Raw<Ellipsoid3D>(&data)->contains(point);
  case Cylinder:
    return AnyCast_Raw<Cylinder3D>(&data)->contains(point);
  case AABB:
    return AnyCast_Raw<AABB3D>(&data)->contains(point);
  case Box:
    return AnyCast_Raw<Box3D>(&data)->contains(point);
  case Triangle:
    return AnyCast_Raw<Triangle3D>(&data)->contains(point);
  default:
    return false;
  }
}

bool GeometricPrimitive3D::Collides(const Segment3D& seg) const
{
  switch(type) {
  case Point:
    return seg.distance(*AnyCast_Raw<Vector3>(&data)) == 0;
  case Segment:
    return AnyCast_Raw<Segment3D>(&data)->distance(seg) == 0;
  case Sphere:
    return AnyCast_Raw<Sphere3D>(&data)->intersects(seg);
  case Ellipsoid:
    return AnyCast_Raw<Ellipsoid3D>(&data)->intersects(seg);
  case Cylinder:
    return AnyCast_Raw<Cylinder3D>(&data)->intersects(seg);
  case AABB:
    return seg.intersects(*AnyCast_Raw<AABB3D>(&data));
  case Box:
    return AnyCast_Raw<Box3D>(&data)->intersects(seg);
  case Triangle:
    return AnyCast_Raw<Triangle3D>(&data)->intersects(seg);
  default:
    return false;
  }
}

bool GeometricPrimitive3D::Collides(const AABB3D& aabb) const
{
  switch(type) {
  case Point:
    return aabb.contains(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    return AnyCast_Raw<Segment3D>(&data)->intersects(aabb);
  case Sphere:
    return AnyCast_Raw<Sphere3D>(&data)->intersects(aabb);
  case AABB:
    return aabb.intersects(*AnyCast_Raw<AABB3D>(&data));
  case Box: {
    Box3D bb;
    bb.set(aabb);
    return AnyCast_Raw<Box3D>(&data)->intersects(bb);
  }
  case Triangle:
    return AnyCast_Raw<Triangle3D>(&data)->intersects(aabb);
  default:
    return false;
  }
}

bool GeometricPrimitive3D::Collides(const Box3D& box) const
{
  GeometricPrimitive3D loc = *this;
  RigidTransform Tbinv;
  box.getTransformInv(Tbinv);
  loc.Transform(Tbinv);
  AABB3D bb; bb.bmin.setZero(); bb.bmax=box.dims;
  return loc.Collides(bb);
}

bool GeometricPrimitive3D::Collides(const Sphere3D& circle) const
{
  return Distance(circle.center) <= circle.radius;
}

bool GeometricPrimitive3D::Collides(const Triangle3D& triangle) const
{
  switch(type) {
  case Point:
    return triangle.contains(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    return triangle.intersects(*AnyCast_Raw<Segment3D>(&data));
  case Sphere:
    {
      const Sphere3D* c=AnyCast_Raw<Sphere3D>(&data);
      return c->contains(triangle.closestPoint(c->center));
    }
  case AABB:
    return triangle.intersects(*AnyCast_Raw<AABB3D>(&data));
  case Box:
    {
      RigidTransform Tb;
      AnyCast_Raw<Box3D>(&data)->getTransformInv(Tb);
      Triangle3D loc;
      loc.a = Tb*triangle.a;
      loc.b = Tb*triangle.b;
      loc.c = Tb*triangle.c;
      AABB3D bb; bb.bmin.setZero(); bb.bmax=AnyCast_Raw<Box3D>(&data)->dims;
      return loc.intersects(bb);
    }
  case Triangle:
    return triangle.intersects(*AnyCast_Raw<Triangle3D>(&data));
  default:
    return false;
  }
}

bool GeometricPrimitive3D::Collides(const Polygon3D& p) const
{
  return false;
}

bool GeometricPrimitive3D::Collides(const Ellipsoid3D& e) const
{
  switch(type) {
  case Point:
    return e.contains(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    return e.intersects(*AnyCast_Raw<Segment3D>(&data));
  case Triangle:
    {
      const Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      Triangle3D tloc;
      e.toLocalNormalized(t->a,tloc.a);
      e.toLocalNormalized(t->b,tloc.b);
      e.toLocalNormalized(t->c,tloc.c);
      Sphere3D s;
      s.center.setZero();
      s.radius = 1.0;
      return s.contains(tloc.closestPoint(s.center));
    }
  default:
    return false;
  }
}

bool GeometricPrimitive3D::Collides(const Cylinder3D& c) const
{
  switch(type) {
  case Point:
    return c.contains(*AnyCast_Raw<Vector3>(&data));
  case Sphere:
    {
      const Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      Vector3 temp;
      c.closestPoint(s->center,temp);
      return s->center.distance(temp) <= s->radius;
    }
  case Segment:
    return c.intersects(*AnyCast_Raw<Segment3D>(&data));
  default:
    return false;
  }
}


bool GeometricPrimitive3D::SupportsDistance(Type a,Type b)
{
  if((a == Point || a == Sphere))
    return (b==Point || b==Sphere || b==Segment || b==AABB || b==Box || b==Triangle);
  if((b == Point || b == Sphere))
    return (a==Point || a==Sphere || a==Segment || a==AABB || a==Box || a==Triangle);
  if(a==Segment && b==Segment) return true;
  if(a==AABB && b==AABB) return true;
  return false;
}

Real GeometricPrimitive3D::Distance(const GeometricPrimitive3D& geom) const
{
  switch(type) {
  case Point:
    return geom.Distance(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    return geom.Distance(*AnyCast_Raw<Segment3D>(&data));
  case Sphere:
    return geom.Distance(*AnyCast_Raw<Sphere3D>(&data));
  case Ellipsoid:
    return geom.Distance(*AnyCast_Raw<Ellipsoid3D>(&data));
  case Cylinder:
    return geom.Distance(*AnyCast_Raw<Cylinder3D>(&data));
  case AABB:
    return geom.Distance(*AnyCast_Raw<AABB3D>(&data));
  case Box:
    return geom.Distance(*AnyCast_Raw<Box3D>(&data));
  case Triangle:
    return geom.Distance(*AnyCast_Raw<Triangle3D>(&data));
  case Polygon:
    return geom.Distance(*AnyCast_Raw<Polygon3D>(&data));
  default:
    return Inf;
  }
}

Real GeometricPrimitive3D::Distance(const Vector3& x) const
{
  switch(type) {
  case Point:
    return x.distance(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    return AnyCast_Raw<Segment3D>(&data)->distance(x);
  case Sphere:
    return AnyCast_Raw<Sphere3D>(&data)->signedDistance(x);
  case Cylinder:
    return AnyCast_Raw<Cylinder3D>(&data)->distance(x);
  case AABB:
    return AnyCast_Raw<AABB3D>(&data)->signedDistance(x);
  case Box:
    return AnyCast_Raw<Box3D>(&data)->signedDistance(x);
  case Triangle:
    return AnyCast_Raw<Triangle3D>(&data)->closestPoint(x).distance(x);
  default:
    return Inf;
  }
}

Real GeometricPrimitive3D::Distance(const Sphere3D& s) const
{
  Real d=Distance(s.center);
  return Max(0.0,d-s.radius);
}

Real GeometricPrimitive3D::Distance(const Segment3D& s) const
{
  switch(type) {
  case Point:
    return s.distance(*AnyCast_Raw<Vector3>(&data));
  case Segment:
    {
      const Segment3D* seg=AnyCast_Raw<Segment3D>(&data);
      return seg->distance(s);
    }
  case Sphere:
    return Max(0.0,s.distance(AnyCast_Raw<Sphere3D>(&data)->center)-AnyCast_Raw<Sphere3D>(&data)->radius);
  default:
    return Inf;
  }
}

Real GeometricPrimitive3D::Distance(const AABB3D& b) const
{
  switch(type) {
  case Point:
    return b.signedDistance(*AnyCast_Raw<Vector3>(&data));
  case Sphere:
    return Max(0.0,b.signedDistance(AnyCast_Raw<Sphere3D>(&data)->center)-AnyCast_Raw<Sphere3D>(&data)->radius);
  case AABB:
    return b.distance(*AnyCast_Raw<AABB3D>(&data));
  default:
    return false;
  }
}

Real GeometricPrimitive3D::Distance(const Box3D& b) const
{
  switch(type) {
  case Point:
    return b.signedDistance(*AnyCast_Raw<Vector3>(&data));
  case Sphere:
    return Max(0.0,b.signedDistance(AnyCast_Raw<Sphere3D>(&data)->center)-AnyCast_Raw<Sphere3D>(&data)->radius);
  default:
    return false;
  }
}

Real GeometricPrimitive3D::Distance(const Triangle3D& t) const
{
  switch(type) {
  case Point:
    return t.closestPoint(*AnyCast_Raw<Vector3>(&data)).distance(*AnyCast_Raw<Vector3>(&data));
  case Sphere:
    return Max(0.0,t.closestPoint(AnyCast_Raw<Sphere3D>(&data)->center).distance(AnyCast_Raw<Sphere3D>(&data)->center)-AnyCast_Raw<Sphere3D>(&data)->radius);
  default:
    return false;
  }
}

Real GeometricPrimitive3D::Distance(const Polygon3D& p) const
{
  return Inf;
}

Real GeometricPrimitive3D::Distance(const Ellipsoid3D& e) const
{
  return Inf;
}

Real GeometricPrimitive3D::Distance(const Cylinder3D& c) const
{
  switch(type) {
  case Point:
    return c.distance(*AnyCast_Raw<Vector3>(&data));
  default:
    return Inf;
  }
}

bool GeometricPrimitive3D::SupportsClosestPoints(Type a,Type b)
{
  if(a == Point || a == Sphere || b == Point || b == Sphere) return true;
  if(a == AABB && b == AABB) return true;
  if(a == Segment && b == Segment) return true;
  if(a == Triangle && b == Triangle) return true;
  return false;
}

Vector3 Unit(const Vector3& v)
{
  Real n = v.norm();
  if(Math::FuzzyZero(n)) return Vector3(0.0);
  else return v*(1.0/n);
}

Real GeometricPrimitive3D::ClosestPoints(const Vector3& pt,Vector3& cp,Vector3& direction) const
{
  Real d;
  switch(type) {
  case Point:
    cp = *AnyCast_Raw<Vector3>(&data);
    direction = Unit(pt-cp);
    return cp.distance(pt);
  case Segment:
    {
      const Segment3D* s=AnyCast_Raw<Segment3D>(&data);
      d = s->closestPoint(pt,cp);
    }
    break;
  case Triangle:
    {
      const Triangle3D* t=AnyCast_Raw<Triangle3D>(&data);
      cp = t->closestPoint(pt);
      direction = pt-cp;
      Real n = direction.norm();
      if(Math::FuzzyZero(n)) direction = t->normal();
      else direction = direction/n;
      return n;
    }
    break;
  case Polygon:
    return Inf;
  case Sphere:
    {
      const Sphere3D* s=AnyCast_Raw<Sphere3D>(&data);
      Real d = s->closestPoint(pt,cp);
      direction = Unit(pt - s->center);
      return d;
    }
  case AABB:
    {
      const AABB3D* b = AnyCast_Raw<AABB3D>(&data);
      ///TODO better normal estimation
      d = b->distance(pt,cp);
    }
    break;
  case Box:
    {
      const Box3D* b = AnyCast_Raw<Box3D>(&data);
      ///TODO better normal estimation 
      d = b->distance(pt,cp);
    }
    break;
  case Cylinder:
    {
      const Cylinder3D* c=AnyCast_Raw<Cylinder3D>(&data);
      d = c->closestPoint(pt,cp);
    }
    break;
  default:
    FatalError("Invalid primitive type");
    return Inf;
  }
  //default: assume d and cp are calculated
  direction = Unit(pt-cp);
  return d;
}

Real GeometricPrimitive3D::ClosestPoints(const Segment3D& s,Vector3& cp,Vector3& direction) const
{
  Vector3 cp_other;
  switch(type) {
  case Point:
    cp = *AnyCast_Raw<Vector3>(&data);
    s.closestPoint(cp,cp_other);
    break;
  case Sphere:
    {
      const Sphere3D* sme=AnyCast_Raw<Sphere3D>(&data);
      Real d = s.closestPoint(sme->center,cp_other);
      direction = cp_other-sme->center;
      direction.inplaceNormalize();
      cp = sme->center + sme->radius*direction;
      return d - sme->radius;
    }
  case Segment:
    {
      Real t,u;
      const Segment3D* sme=AnyCast_Raw<Segment3D>(&data);
      sme->closestPoint(s,t,u);
      sme->eval(t,cp);
      s.eval(u,cp_other);
    }
    break;
  default:
    return Inf;
  }
  //assume cp and cp_other are evaluated
  direction = cp_other-cp;
  Real n = direction.norm();
  if(Math::FuzzyZero(n)) direction = Vector3(0.0);
  else direction *= 1.0/n;
  return n;
}
Real GeometricPrimitive3D::ClosestPoints(const Triangle3D& t,Vector3& cp,Vector3& direction) const
{
  Vector3 cp_other;
  switch(type) {
  case Point:
    {
      cp = *AnyCast_Raw<Vector3>(&data);
      cp_other = t.closestPoint(cp);
      direction = cp_other-cp;
      Real n = direction.norm();
      if(Math::FuzzyZero(n)) direction = -t.normal();
      else direction *= 1.0/n;
      return n;
    }
  case Sphere:
    {
      const Sphere3D* sme=AnyCast_Raw<Sphere3D>(&data);
      cp_other = t.closestPoint(sme->center);
      Real d = cp_other.distance(sme->center);
      direction = cp_other-sme->center;
      direction.inplaceNormalize();
      cp = sme->center + sme->radius*direction;
      return d - sme->radius;
    }
  case Triangle:
    {
      const Triangle3D* tme=AnyCast_Raw<Triangle3D>(&data);
      Real d = tme->distance(t,cp,cp_other);
    }
    break;
  default:
    return Inf;
  }
  //assume cp and cp_other are evaluated
  direction = cp_other-cp;
  Real n = direction.norm();
  if(Math::FuzzyZero(n)) direction = Vector3(0.0);
  else direction *= 1.0/n;
  return n;
}
Real GeometricPrimitive3D::ClosestPoints(const Polygon3D& p,Vector3& cp,Vector3& direction) const
{
  return Inf;
}
Real GeometricPrimitive3D::ClosestPoints(const Sphere3D& s,Vector3& cp,Vector3& direction) const
{
  Real d = ClosestPoints(s.center,cp,direction);
  return d - s.radius;
}

Real GeometricPrimitive3D::ClosestPoints(const Ellipsoid3D& s,Vector3& cp,Vector3& direction) const
{
  return Inf;
}
Real GeometricPrimitive3D::ClosestPoints(const Cylinder3D& s,Vector3& cp,Vector3& direction) const
{
  return Inf;
}
Real GeometricPrimitive3D::ClosestPoints(const AABB3D& s,Vector3& cp,Vector3& direction) const
{
  Vector3 cp_other;
  switch(type) {
  case Point:
    {
      cp = *AnyCast_Raw<Vector3>(&data);
      Real d = s.distance(cp,cp_other);
      direction = Unit(cp_other - cp);
      return d;
    }
  case Sphere:
    {
      const Sphere3D* sme=AnyCast_Raw<Sphere3D>(&data);
      Real d = s.distance(sme->center,cp_other);
      direction = cp_other-sme->center;
      direction.inplaceNormalize();
      cp = sme->center + sme->radius*direction;
      return d - sme->radius;
    }
  case AABB:
    {
      const AABB3D* sme=AnyCast_Raw<AABB3D>(&data);
      Real d = sme->distance(s,cp,cp_other);
      direction = Unit(cp_other-cp);
      return d;
    }
    break;
  default:
    return Inf;
  }
}
Real GeometricPrimitive3D::ClosestPoints(const Box3D& s,Vector3& cp,Vector3& direction) const
{
  Vector3 cp_other;
  switch(type) {
  case Point:
    {
      cp = *AnyCast_Raw<Vector3>(&data);
      Real d = s.distance(cp,cp_other);
      direction = Unit(cp_other-cp);
      return d;
    }
  case Sphere:
    {
      const Sphere3D* sme=AnyCast_Raw<Sphere3D>(&data);
      Real d = s.distance(sme->center,cp_other);
      direction = cp_other-sme->center;
      direction.inplaceNormalize();
      cp = sme->center + sme->radius*direction;
      return d - sme->radius;
    }
  default:
    return Inf;
  }
}
Real GeometricPrimitive3D::ClosestPoints(const GeometricPrimitive3D& g,Vector3& cp,Vector3& direction) const
{
  switch(g.type) {
  case Point:
    return ClosestPoints(*AnyCast_Raw<Vector3>(&g.data),cp,direction);
  case Segment:
    return ClosestPoints(*AnyCast_Raw<Segment3D>(&g.data),cp,direction);
  case Sphere:
    return ClosestPoints(*AnyCast_Raw<Sphere3D>(&g.data),cp,direction);
  case Ellipsoid:
    return ClosestPoints(*AnyCast_Raw<Ellipsoid3D>(&g.data),cp,direction);
  case Cylinder:
    return ClosestPoints(*AnyCast_Raw<Cylinder3D>(&g.data),cp,direction);
  case AABB:
    return ClosestPoints(*AnyCast_Raw<AABB3D>(&g.data),cp,direction);
  case Box:
    return ClosestPoints(*AnyCast_Raw<Box3D>(&g.data),cp,direction);
  case Triangle:
    return ClosestPoints(*AnyCast_Raw<Triangle3D>(&g.data),cp,direction);
  case Polygon:
    return ClosestPoints(*AnyCast_Raw<Polygon3D>(&g.data),cp,direction);
  default:
    return Inf;
  }
}



std::ostream& operator <<(std::ostream& out,const GeometricPrimitive3D& g)
{
  switch(g.type) {
  case GeometricPrimitive3D::Point:
    {
      const Vector3* p = AnyCast_Raw<Vector3>(&g.data);
      out<<"Point "<<*p;
    }
    return out;
  case GeometricPrimitive3D::Segment:
    {
      const Segment3D* s=AnyCast_Raw<Segment3D>(&g.data);
      out<<"Segment "<<*s;
    }
    return out;
  case GeometricPrimitive3D::Triangle:
    {
      const Triangle3D* t=AnyCast_Raw<Triangle3D>(&g.data);
      out<<"Triangle "<<*t;
    }
    return out;
  case GeometricPrimitive3D::Sphere:
    {
      const Sphere3D* s=AnyCast_Raw<Sphere3D>(&g.data);
      out<<"Sphere "<<s->center<<"  "<<s->radius;
    }
    return out;
  case GeometricPrimitive3D::AABB:
    {
      const AABB3D* b=AnyCast_Raw<AABB3D>(&g.data);
      out<<"AABB "<<*b;
    }
    return out;
  case GeometricPrimitive3D::Box:
    {
      const Box3D* b=AnyCast_Raw<Box3D>(&g.data);
      out<<"Box "<<*b;
    }
    return out;
  case GeometricPrimitive3D::Cylinder:
    {
      const Cylinder3D* b=AnyCast_Raw<Cylinder3D>(&g.data);
      out<<"Cylinder "<<b->center<<"  "<<b->axis<<"  "<<b->radius<<"  "<<b->height;
    }
    return out;
  case GeometricPrimitive3D::Polygon:
    {
      const Polygon3D* b=AnyCast_Raw<Polygon3D>(&g.data);
      out<<"Polygon "<<*b;
    }
    return out;
  default:
    FatalError("Invalid primitive type");
    return out;
  }
}

std::istream& operator >>(std::istream& in,GeometricPrimitive3D& g)
{
  string type;
  in >> type;
  if(type == "Point") {
    Point3D p;
    in>>p;
    g = GeometricPrimitive3D(p);
    return in;
  }
  else if(type == "Segment") {
    Segment3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Box") {
    Box3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "AABB") {
    AABB3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Triangle") {
    Triangle3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Polygon") {
    Polygon3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Sphere") {
    Sphere3D s;
    in>>s.center>>s.radius;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Ellipsoid") {
    Ellipsoid3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Polygon") {
    Polygon3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else if(type == "Cylinder") {
    Cylinder3D s;
    in>>s;
    g = GeometricPrimitive3D(s);
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"istream >> GeometricPrimitive3D: Invalid type "<<type);
    in.setstate(ios::badbit);
  }
  return in;
}

} // namespace Math3D
