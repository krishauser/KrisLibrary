#include "geometry3d.h"
#include "AABB2D.h"
#include "Polygon2D.h"
#include "interpolate.h"
#include "misc.h"
#include "basis.h"
#include <meshing/TriMesh.h>
#include <math/misc.h>
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
	  //cout<<"Whoa, line just intersects at one point on the sphere"<<endl;
	  //cout<<"l= "<<l.source<<"->"<<l.direction<<endl;
	  //cout<<"c= "<<center<<", r="<<radius<<endl;
	  //cout<<"t="<<x1<<endl;
	  //getchar();
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
  default:
    return "error";
  }
}

GeometricPrimitive3D::GeometricPrimitive3D()
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
  case Point:
    {
      const Vector3* p = AnyCast<Vector3>(&data);
      return AABB3D(*p,*p);
    }
  case Segment:
    AnyCast<Segment3D>(&data)->getAABB(bb);
    return bb;
  case Triangle:
    AnyCast<Triangle3D>(&data)->getAABB(bb);
    return bb;
  case Polygon:
    AnyCast<Polygon3D>(&data)->getAABB(bb);
    return bb;
  case Sphere:
    AnyCast<Sphere3D>(&data)->getAABB(bb);
    return bb;
  case AABB:
    bb = *AnyCast<AABB3D>(&data);
    return bb;
  case Box:
    AnyCast<Box3D>(&data)->getAABB(bb);
    return bb;
  case Cylinder:
    AnyCast<Cylinder3D>(&data)->getAABB(bb);
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
      const Vector3* p = AnyCast<Vector3>(&data);
      bb.origin = *p;
      bb.dims.setZero();
      return bb;
    }
  case Segment:
    {
      const Segment3D* s=AnyCast<Segment3D>(&data);
      bb.origin = s->a;
      bb.xbasis = s->b-s->a;
      bb.dims.set(bb.xbasis.norm(),0,0);
      bb.xbasis /= bb.dims.x;
      GetCanonicalBasis(bb.xbasis,bb.ybasis,bb.zbasis);
    }
    return bb;
  case Triangle:
    {
      const Triangle3D* t=AnyCast<Triangle3D>(&data);
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
      AnyCast<Polygon3D>(&data)->getPlanarPolygon(p,T);
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
      const Sphere3D* s=AnyCast<Sphere3D>(&data);
      bb.origin = s->center - Vector3(s->radius,s->radius,s->radius);
      bb.dims = 2.0*Vector3(s->radius,s->radius,s->radius);
    }
    return bb;
  case AABB:
    {
      const AABB3D* b = AnyCast<AABB3D>(&data);
      bb.xbasis.set(1,0,0);
      bb.ybasis.set(0,1,0);
      bb.zbasis.set(0,0,1);
      bb.origin = b->bmin;
      bb.dims = b->bmax-b->bmin;
    }
    return bb;
  case Box:
    bb = *AnyCast<Box3D>(&data);
    return bb;
  case Cylinder:
    {
      const Cylinder3D* c=AnyCast<Cylinder3D>(&data);
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
  case Point:
    {
      Vector3* p = AnyCast<Vector3>(&data);
      *p = T*(*p);
    }
    break;
  case Segment:
    {
      Segment3D* s=AnyCast<Segment3D>(&data);
      s->a = T*s->a;
      s->b = T*s->b;
    }
    break;
  case Triangle:
    {
      Triangle3D* t=AnyCast<Triangle3D>(&data);
      t->setTransformed(*t,T);
    }
    break;
  case Polygon:
    {
      Polygon3D* p=AnyCast<Polygon3D>(&data);
      p->setTransformed(*p,T);
    }
    break;
  case Sphere:
    {
      Sphere3D* s=AnyCast<Sphere3D>(&data);
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
      Box3D* b = AnyCast<Box3D>(&data);
      b->setTransformed(*b,T);
    }
    break;
  case Cylinder:
    {
      Cylinder3D* c=AnyCast<Cylinder3D>(&data);
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
      const Segment3D* s=AnyCast<Segment3D>(&data);
      res[0] = s->closestPointParameter(pt);
    }
    return res;
  case Triangle:
    {
      const Triangle3D* t=AnyCast<Triangle3D>(&data);
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
      const Sphere3D* s=AnyCast<Sphere3D>(&data);
      Vector3 cp;
      s->closestPoint(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);
    }
    return res;
  case AABB:
    {
      const AABB3D* b = AnyCast<AABB3D>(&data);
      Vector3 cp;
      b->distanceSquared(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);      
    }
    return res;
  case Box:
    {
      const Box3D* b = AnyCast<Box3D>(&data);
      Vector3 cp;
      b->distanceSquared(pt,cp);
      res.resize(3);
      cp.get(res[0],res[1],res[2]);      
    }
    return res;
  case Cylinder:
    {
      const Cylinder3D* c=AnyCast<Cylinder3D>(&data);
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
    return *AnyCast<Vector3>(&data);
  case Segment:
    {
      Assert(params.size()==1);
      const Segment3D* s=AnyCast<Segment3D>(&data);
      return s->a + params[0]*(s->b-s->a);
    }
  case Triangle:
    {
      const Triangle3D* t=AnyCast<Triangle3D>(&data);
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

std::ostream& operator <<(std::ostream& out,const GeometricPrimitive3D& g)
{
  switch(g.type) {
  case GeometricPrimitive3D::Point:
    {
      const Vector3* p = AnyCast<Vector3>(&g.data);
      out<<"Point "<<*p;
    }
    return out;
  case GeometricPrimitive3D::Segment:
    {
      const Segment3D* s=AnyCast<Segment3D>(&g.data);
      out<<"Segment "<<*s;
    }
    return out;
  case GeometricPrimitive3D::Triangle:
    {
      const Triangle3D* t=AnyCast<Triangle3D>(&g.data);
      out<<"Triangle "<<*t;
    }
    return out;
  case GeometricPrimitive3D::Sphere:
    {
      const Sphere3D* s=AnyCast<Sphere3D>(&g.data);
      out<<"Sphere "<<s->center<<"  "<<s->radius;
    }
    return out;
  case GeometricPrimitive3D::AABB:
    {
      const AABB3D* b=AnyCast<AABB3D>(&g.data);
      out<<"AABB "<<*b;
    }
    return out;
  case GeometricPrimitive3D::Box:
    {
      const Box3D* b=AnyCast<Box3D>(&g.data);
      out<<"Box "<<*b;
    }
    return out;
  case GeometricPrimitive3D::Cylinder:
    {
      const Cylinder3D* b=AnyCast<Cylinder3D>(&g.data);
      out<<"Cylinder "<<b->center<<"  "<<b->axis<<"  "<<b->radius<<"  "<<b->height;
    }
    return out;
  case GeometricPrimitive3D::Polygon:
    {
      const Polygon3D* b=AnyCast<Polygon3D>(&g.data);
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
    cout<<"istream >> GeometricPrimitive3D: Invalid type "<<type<<endl;
    in.setstate(ios::badbit);
  }
  return in;
}

} // namespace Math3D
