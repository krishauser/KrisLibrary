#include "AABB3D.h"
#include <KrisLibrary/File.h>
#include <iostream>
using namespace Math3D;
using namespace std;

AABB3D::AABB3D()
{
}

AABB3D::AABB3D(const Vector3& _bmin,const Vector3& _bmax)
  :bmin(_bmin),bmax(_bmax)
{
}

AABB3D::AABB3D(const AABB3D& rhs)
  :bmin(rhs.bmin),bmax(rhs.bmax)
{
}

bool AABB3D::Read(File& f)
{
  if(!ReadFile(f,bmin)) return false;
  if(!ReadFile(f,bmax)) return false;
  return false;
}

bool AABB3D::Write(File& f) const
{
  if(!WriteFile(f,bmin)) return false;
  if(!WriteFile(f,bmax)) return false;
  return false;
}

void AABB3D::Print(ostream& out) const 
{
  out<<"["<<bmin.x<<","<<bmax.x<<"]x["<<bmin.y<<","<<bmax.y<<"]x["<<bmin.z<<","<<bmax.z<<"]";
}

void AABB3D::maximize()
{
  bmin.set(-Inf);
  bmax.set(Inf);
}

void AABB3D::minimize()
{
  bmin.set(Inf);
  bmax.set(-Inf);
}

void AABB3D::expand(const Point3D& v)
{
  bmin.setMinimum(v);
  bmax.setMaximum(v);
}

void AABB3D::setIntersection(const AABB3D& b)
{
  bmin.setMaximum(b.bmin);
  bmax.setMinimum(b.bmax);
}

void AABB3D::setUnion(const AABB3D& b)
{
  bmin.setMinimum(b.bmin);
  bmax.setMaximum(b.bmax);
}

void AABB3D::setPoint(const Point3D& p)
{
  bmin.set(p);
  bmax.set(p);
}

void AABB3D::getSize(Vector3& v) const
{
  v.sub(bmax,bmin);
}

Vector3 AABB3D::size() const { return bmax-bmin; }

void AABB3D::getMidpoint(Point3D& p) const
{
  p.add(bmax,bmin);
  p.inplaceMul(Half);
}

Vector3 AABB3D::midpoint() const { return (bmax+bmin)*0.5; }


bool AABB3D::contains(const Vector3& v) const
{
	return (v.x>=bmin.x && v.x<=bmax.x &&
		v.y>=bmin.y && v.y<=bmax.y &&
		v.z>=bmin.z && v.z<=bmax.z);
}

bool AABB3D::contains(const AABB3D& bb) const
{
  return contains(bb.bmin) && contains(bb.bmax);
}

Real AABB3D::distance(const Vector3& v) const
{
  Vector3 temp;
  return distance(v,temp);
}

Real AABB3D::distance(const Vector3& v,Vector3& out) const
{
  return Sqrt(distanceSquared(v,out));
}

Real AABB3D::distanceSquared(const Vector3& v,Vector3& out) const
{
  out=v;
  if(out.x < bmin.x) out.x=bmin.x;
  if(out.y < bmin.y) out.y=bmin.y;
  if(out.z < bmin.z) out.z=bmin.z;
  if(out.x > bmax.x) out.x=bmax.x;
  if(out.y > bmax.y) out.y=bmax.y;
  if(out.z > bmax.z) out.z=bmax.z;
  return out.distanceSquared(v);
}

Real AABB3D::signedDistance(const Point3D& pt) const
{
  Point3D temp;
  return signedDistance(pt,temp);
}

Real AABB3D::signedDistance(const Point3D& pt,Point3D& out) const
{
  out=pt;
  bool inside = true;
  Real dmin = Inf;
  if(out.x < bmin.x) { out.x=bmin.x; inside=false; }
  else dmin = Min(dmin,out.x-bmin.x); 
  if(out.y < bmin.y) { out.y=bmin.y; inside=false; }
  else dmin = Min(dmin,out.y-bmin.y); 
  if(out.z < bmin.z) { out.z=bmin.z; inside=false; }
  else dmin = Min(dmin,out.z-bmin.z); 
  if(out.x > bmax.x) { out.x=bmax.x; inside=false; }
  else dmin = Min(dmin,bmax.x-out.x); 
  if(out.y > bmax.y) { out.y=bmax.y; inside=false; }
  else dmin = Min(dmin,bmax.y-out.y); 
  if(out.z > bmax.z) { out.z=bmax.z; inside=false; }
  else dmin = Min(dmin,bmax.z-out.z); 
  if(!inside) return out.distance(pt);
  else return -dmin;
}

Real AABB3D::distance(const AABB3D& bb) const
{
  Point3D x,y;
  return distance(bb,x,y);
}

Real AABB3D::distance(const AABB3D& bb,Point3D& myclosest,Point3D& bbclosest) const
{
  if(bmax.x <= bb.bmin.x) {
    myclosest.x = bmax.x;
    bbclosest.x = bb.bmin.x;
  }
  else if(bmin.x >= bb.bmax.x) {
    myclosest.x = bmin.x;
    bbclosest.x = bb.bmax.x;
  }
  else { //overlap
    myclosest.x = bmax.x;
    bbclosest.x = bmax.x;
  }
  if(bmax.y <= bb.bmin.y) {
    myclosest.y = bmax.y;
    bbclosest.y = bb.bmin.y;
  }
  else if(bmin.y >= bb.bmax.y) {
    myclosest.y = bmin.y;
    bbclosest.y = bb.bmax.y;
  }
  else { //overlap
    myclosest.y = bmax.y;
    bbclosest.y = bmax.y;
  }
  if(bmax.z <= bb.bmin.z) {
    myclosest.z = bmax.z;
    bbclosest.z = bb.bmin.z;
  }
  else if(bmin.z >= bb.bmax.z) {
    myclosest.z = bmin.z;
    bbclosest.z = bb.bmax.z;
  }
  else { //overlap
    myclosest.z = bmax.z;
    bbclosest.z = bmax.z;
  }
  return myclosest.distance(bbclosest);
}

bool AABB3D::intersects(const AABB3D& a) const
{
	return (bmin.x <= a.bmax.x && bmax.x >= a.bmin.x &&
		bmin.y <= a.bmax.y && bmax.y >= a.bmin.y &&
		bmin.z <= a.bmax.z && bmax.z >= a.bmin.z);
}

void AABB3D::justify()
{
	if(bmin.x > bmax.x) std::swap(bmin.x,bmax.x);
	if(bmin.y > bmax.y) std::swap(bmin.y,bmax.y);
	if(bmin.z > bmax.z) std::swap(bmin.z,bmax.z);
}

void AABB3D::setTransform(const AABB3D& b,const Matrix4& mat)
{
	*this = b;
	inplaceTransform(mat);
}

void AABB3D::inplaceTransform(const Matrix4& mat)
{
	Vector3 dims[8];
	Vector3 dimsTransformed[8];
	dims[0].set(bmin.x,bmin.y,bmin.z);
	dims[1].set(bmin.x,bmin.y,bmax.z);
	dims[2].set(bmin.x,bmax.y,bmin.z);
	dims[3].set(bmin.x,bmax.y,bmax.z);
	dims[4].set(bmax.x,bmin.y,bmin.z);
	dims[5].set(bmax.x,bmin.y,bmax.z);
	dims[6].set(bmax.x,bmax.y,bmin.z);
	dims[7].set(bmax.x,bmax.y,bmax.z);
	for(int i=0;i<8;i++)
		mat.mulPoint(dims[i],dimsTransformed[i]);
	setPoint(dimsTransformed[0]);
	for(int i=1;i<8;i++)
		expand(dimsTransformed[i]);
}

namespace Math3D {
  ostream& operator << (ostream& out,const AABB3D& bb)
  {
    out<<bb.bmin<<"   "<<bb.bmax;
    return out;
  }
  istream& operator >> (istream& in,AABB3D& bb)
  {
    in>>bb.bmin>>bb.bmax;
    return in;
  }
};


