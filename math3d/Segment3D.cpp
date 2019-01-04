#include "Segment3D.h"
#include "geometry3d.h"
#include "clip.h"
#include "interpolate.h"
#include <KrisLibrary/File.h>
using namespace Math3D;
using namespace std;

bool Segment3D::Read(File& f)
{
	if(!a.Read(f)) return false;
	if(!b.Read(f)) return false;
	return true;
}

bool Segment3D::Write(File& f) const
{
	if(!a.Write(f)) return false;
	if(!b.Write(f)) return false;
	return true;
}

void Segment3D::setTransformed(const Segment3D& s, const Matrix4& xform)
{
	xform.mulPoint(s.a,a);
	xform.mulPoint(s.b,b);
}

void Segment3D::getLine(Line3D& l) const
{
	l.source=a;
	l.direction=b-a;
}

void Segment3D::getAABB(AABB3D& bb) const
{
  bb.setPoint(a);
  bb.expand(b);
}

Real Segment3D::distance(const Point3D& in) const
{
  Point3D temp;
  closestPoint(in,temp);
  return in.distance(temp);
}

Real Segment3D::closestPointParameter(const Point3D& in) const
{
	Vector3 dir=b-a;
	Real numer = dot(in-a,dir);
	Real denom = dot(dir,dir);
	//t = numer/denom, denom always >= 0
	if(numer <= Zero) return Zero;
	if(numer >= denom) return One;
	return numer/denom;
}


Real Segment3D::distance(const Segment3D& s) const
{
  Real t,u;
  closestPoint(s,t,u);
  Vector3 p1,p2;
  eval(t,p1);
  s.eval(u,p2);
  return p1.distance(p2);
}

Real Segment3D::closestPoint(const Point3D& in,Point3D& out) const
{
	Real t = closestPointParameter(in);
	eval(t,out);
	return t;
}

void Segment3D::closestPoint(const Segment3D& s, Real& t, Real& u) const
{
  Line3D l1,l2;
  getLine(l1);
  s.getLine(l2);
  l1.closestPoint(l2,t,u);
  t=Clamp(t,Zero,One);
  u=Clamp(u,Zero,One);
}

void Segment3D::eval(Real t, Point3D& out) const
{
  interpolate(a,b,t,out);
}

bool Segment3D::intersects(const AABB3D& bb) const
{
	Real u1,u2;
	return intersects(bb,u1,u2);
}

bool Segment3D::intersects(const AABB3D& bb, Real& u1, Real& u2) const
{
  u1 = 0;
  u2 = 1;
  return ClipLine(a, b-a, bb, u1,u2);
}

Real Segment3D::distance(const AABB3D& bb) const
{
  Real tclosest;
  Vector3 bbclosest;
  return distance(bb,tclosest,bbclosest);
}

Real Segment3D::distance(const AABB3D& bb, Real& tclosest, Point3D& bbclosest) const
{
  Real u1,u2;
  if(intersects(bb,u1,u2)) {
    tclosest = u1;
    eval(u1,bbclosest);
    return 0;
  }
  //cases: start-face, end-face, middle-edge, middle-point
  Vector3 ca,cb;
  Real da2 = bb.distanceSquared(a,ca);
  Real db2 = bb.distanceSquared(b,cb);
  //if these points are on the same face then the middle is not closest
  bool matchx,matchy,matchz;
  matchx = (ca.x==cb.x);
  matchy = (ca.y==cb.y);
  matchz = (ca.z==cb.z);
  int nmatch = 0;
  if(matchx) nmatch++;
  if(matchy) nmatch++;
  if(matchz) nmatch++;
  if(nmatch <= 1) {
    //the closest must be middle-edge or middle point
    Line3D l;
    l.source = a;
    l.direction = b;
    Real tl;
    Vector3 cl;
    Real dl = l.distance(bb,tl,cl);
    if(tl >= 0 && tl <= 1 && dl*dl < Min(da2,db2)) {
      tclosest = tl;
      bbclosest = cb;
      return dl;
    }
  }
  if(da2 < db2) {
    tclosest = 0;
    bbclosest = ca;
    return Sqrt(da2);
  }
  else {
    tclosest = 1;
    bbclosest = cb;
    return Sqrt(db2);
  }
}


namespace Math3D 
{
  ostream& operator << (ostream& out,const Segment3D& s)
  {
    out<<s.a<<"  "<<s.b;
    return out;
  }
  istream& operator >> (istream& in,Segment3D& s)
  {
    in>>s.a>>s.b;
    return in;
  }
}
