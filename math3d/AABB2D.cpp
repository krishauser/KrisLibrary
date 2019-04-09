#include "AABB2D.h"
#include <KrisLibrary/File.h>
#include <algorithm>
using namespace Math3D;

AABB2D::AABB2D()
{
}

AABB2D::AABB2D(const Vector2& _bmin,const Vector2& _bmax)
  :bmin(_bmin),bmax(_bmax)
{
}

AABB2D::AABB2D(const AABB2D& rhs)
  :bmin(rhs.bmin),bmax(rhs.bmax)
{
}

bool AABB2D::Read(File& f)
{
  if(!ReadFile(f,bmin)) return false;
  if(!ReadFile(f,bmax)) return false;
  return false;
}

bool AABB2D::Write(File& f) const
{
  if(!WriteFile(f,bmin)) return false;
  if(!WriteFile(f,bmax)) return false;
  return false;
}

void AABB2D::maximize()
{
  bmin.set(-Inf);
  bmax.set(Inf);
}

void AABB2D::minimize()
{
  bmin.set(Inf);
  bmax.set(-Inf);
}

void AABB2D::expand(const Point2D& v)
{
  bmin.setMinimum(v);
  bmax.setMaximum(v);
}

void AABB2D::setIntersection(const AABB2D& b)
{
  bmin.setMaximum(b.bmin);
  bmax.setMinimum(b.bmax);
}

void AABB2D::setUnion(const AABB2D& b)
{
  bmin.setMinimum(b.bmin);
  bmax.setMaximum(b.bmax);
}

void AABB2D::setPoint(const Point2D& p)
{
  bmin.set(p);
  bmax.set(p);
}

void AABB2D::getSize(Vector2& v) const
{
  v.sub(bmax,bmin);
}

void AABB2D::getMidpoint(Point2D& p) const
{
  p.add(bmax,bmin);
  p.inplaceMul(Half);
}

bool AABB2D::contains(const Vector2& v) const
{
	return (v.x>=bmin.x && v.x<=bmax.x &&
		v.y>=bmin.y && v.y<=bmax.y);
}

bool AABB2D::contains(const AABB2D& a) const
{
  return contains(a.bmin) && contains(a.bmax);
}

bool AABB2D::intersects(const AABB2D& a) const
{
	return (bmin.x <= a.bmax.x && bmax.x >= a.bmin.x &&
		bmin.y <= a.bmax.y && bmax.y >= a.bmin.y);
}


void AABB2D::justify()
{
	if(bmin.x > bmax.x) std::swap(bmin.x,bmax.x);
	if(bmin.y > bmax.y) std::swap(bmin.y,bmax.y);
}

void AABB2D::setTransform(const AABB2D& b,const Matrix3& mat)
{
	*this = b;
	inplaceTransform(mat);
}

void AABB2D::inplaceTransform(const Matrix3& mat)
{
	Vector2 dims[4];
	Vector2 dimsTransformed[4];
	dims[0].set(bmin.x,bmin.y);
	dims[1].set(bmin.x,bmax.y);
	dims[2].set(bmax.x,bmin.y);
	dims[3].set(bmax.x,bmax.y);
	for(int i=0;i<4;i++)
		mat.mulPoint(dims[i],dimsTransformed[i]);
	setPoint(dimsTransformed[0]);
	for(int i=1;i<4;i++)
		expand(dimsTransformed[i]);
}


/*
void AABB2D::calculate(const Box2D& b)
{
	Vector2 x(b.dims.x*b.xbasis),y(b.dims.y*b.ybasis),z(b.dims.z*b.zbasis);

	Vector2 tmin, tmax;
	tmin.setZero();
	tmax.setZero();
	for(int i=0; i<2; i++)
	{
		if(x[i] > Zero) tmax[i] = x[i];
		else tmin[i] = x[i];
		if(y[i] > Zero) tmax[i] += y[i];
		else tmin[i] += y[i];
		if(z[i] > Zero) tmax[i] += z[i];
		else tmin[i] += z[i];
	}

	bmin.add(tmin, b.origin);
	bmax.add(tmax, b.origin);
}
*/



/*
void AABB2D::calculate(const Ellipsoid2D& e)
{
	//get the bases of world space in ellipsoid space
	Vector2 xb,yb,zb;
	xb.x = e.xbasis.x;
	xb.y = e.ybasis.x;
	xb.z = e.zbasis.x;
	yb.x = e.xbasis.y;
	yb.y = e.ybasis.y;
	yb.z = e.zbasis.y;
	zb.x = e.xbasis.z;
	zb.y = e.ybasis.z;
	zb.z = e.zbasis.z;

	e.normalize(xb,xb);
	e.normalize(yb,yb);
	e.normalize(zb,zb);

	//now find the points on the sphere with the correct tangent planes
	Vector2 xt,yt,zt;
	xt = cross(yb,zb);
	yt = cross(zb,xb);
	zt = cross(xb,yb);

	//these are the normals, just normalize them
	normalize(xt);
	normalize(yt);
	normalize(zt);

	xb = e.xbasis * e.dims.x;
	yb = e.ybasis * e.dims.y;
	zb = e.zbasis * e.dims.z;

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

	bmax += e.origin;
	bmin += e.origin;
}*/


Real AABB2D::distance(const Vector2& v) const
{
  Vector2 temp;
  return distance(v,temp);
}

Real AABB2D::distance(const Vector2& v,Vector2& out) const
{
  return Sqrt(distanceSquared(v,out));
}

Real AABB2D::distanceSquared(const Vector2& v,Vector2& out) const
{
  out=v;
  if(out.x < bmin.x) out.x=bmin.x;
  if(out.y < bmin.y) out.y=bmin.y;
  if(out.x > bmax.x) out.x=bmax.x;
  if(out.y > bmax.y) out.y=bmax.y;
  return out.distanceSquared(v);
}

Real AABB2D::signedDistance(const Vector2& pt) const
{
  Vector2 temp;
  return signedDistance(pt,temp);
}

Real AABB2D::signedDistance(const Vector2& pt,Vector2& out) const
{
  out=pt;
  bool inside = true;
  Real dmin = Inf;
  if(out.x < bmin.x) { out.x=bmin.x; inside=false; }
  else dmin = Min(dmin,out.x-bmin.x); 
  if(out.y < bmin.y) { out.y=bmin.y; inside=false; }
  else dmin = Min(dmin,out.y-bmin.y); 
  if(out.x > bmax.x) { out.x=bmax.x; inside=false; }
  else dmin = Min(dmin,bmax.x-out.x); 
  if(out.y > bmax.y) { out.y=bmax.y; inside=false; }
  else dmin = Min(dmin,bmax.y-out.y); 
  if(!inside) return out.distance(pt);
  else return -dmin;
}

