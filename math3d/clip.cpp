#include "clip.h"

namespace Math3D {

bool ClipLine1D(Real q, Real p, Real& umin, Real& umax)
{
   Real r;
   if(p<0) {			//entering
     r=-q/p;
     if(r > umax) return false;
     if(r > umin) umin = r;
   }
   else if(p>0) {
     r=-q/p;
     if(r < umin) return false;
     if(r < umax) umax = r;
   }
   else {
     if(q>0) return false;
   }
   return true;
}

bool ClipLine(const Vector2& x, const Vector2& v, const Plane2D& b, Real& u1, Real& u2)
{
	//p is dot(v, normal), q is signed dist to plane
	return ClipLine1D(b.distance(x), dot(v,b.normal), u1,u2);
}

bool ClipLine(const Vector2& x, const Vector2& v, const AABB2D& b, Real& u1, Real& u2)
{
	//for each face, p is dot(v, normal), q is signed dist to plane (dot(v,normal)-offset)
	//normal order: (-1,0), (1,0), (0,-1), (0,1)
	//offset order: -bmin.x, bmax.x, -bmin.y, bmax.y,
	if(ClipLine1D(b.bmin.x - x.x, -v.x, u1,u2) && ClipLine1D(x.x - b.bmax.x, v.x, u1,u2)) {
		if(ClipLine1D(b.bmin.y - x.y, -v.y, u1,u2) && ClipLine1D(x.y - b.bmax.y, v.y, u1,u2))	{
  	  return true;
		}
	}
	return false;
}

bool ClipLine(const Vector2& x, const Vector2& v, const ConvexPolygon2D& p, Real& u1, Real& u2)
{
  Plane2D plane;
  for(size_t i=0; i<p.vertices.size(); i++) {
    p.getPlane((int)i,plane);
    if(!ClipLine(x,v,plane,u1,u2)) return false;
  }
  return true;
}


bool ClipLine(const Vector3& x, const Vector3& v, const Plane3D& b, Real& u1, Real& u2)
{
	//p is dot(v, normal), q is signed dist to plane
	return ClipLine1D(b.distance(x), dot(v,b.normal), u1,u2);
}

bool ClipLine(const Vector3& x, const Vector3& v, const Box3D& b, Real& u1, Real& u2)
{
	Vector3 x2, v2;
	b.toLocal(x, x2);
	//for each face, p is dot(v, normal), q is signed dist to plane (dot(v,normal)-offset)
	//normal order: (-1,0,0), (1,0,0), (0,-1,0), (0,1,0), (0,0,-1), (0,0,1)
	//offset order: 0, dims.x, 0, dims.y, 0, dims.z
	v2.x=dot(v,b.xbasis);
	if(ClipLine1D(-x2.x, -v2.x, u1,u2) && ClipLine1D(x2.x-b.dims.x, v2.x, u1,u2))	{
		v2.y=dot(v,b.ybasis);
		if(ClipLine1D(-x2.y, -v2.y, u1,u2) && ClipLine1D(x2.y-b.dims.y, v2.y, u1,u2))	{
			v2.z=dot(v,b.zbasis);
			if(ClipLine1D(-x2.z,-v2.z, u1,u2) && ClipLine1D(x2.z-b.dims.z, v2.z, u1,u2)) {
				return true;
			}
		}
	}
	return false;
}

bool ClipLine(const Vector3& x, const Vector3& v, const AABB3D& b, Real& u1, Real& u2)
{
	//for each face, p is dot(v, normal), q is signed dist to plane (dot(v,normal)-offset)
	//normal order: (-1,0,0), (1,0,0), (0,-1,0), (0,1,0), (0,0,-1), (0,0,1)
	//offset order: -bmin.x, bmax.x, -bmin.y, bmax.y, -bmin.z, bmax.z
	if(ClipLine1D(b.bmin.x - x.x, -v.x, u1,u2) && ClipLine1D(x.x - b.bmax.x, v.x, u1,u2))
	{
		if(ClipLine1D(b.bmin.y - x.y, -v.y, u1,u2) && ClipLine1D(x.y - b.bmax.y, v.y, u1,u2))
		{
			if(ClipLine1D(b.bmin.z - x.z, -v.z, u1,u2) && ClipLine1D(x.z - b.bmax.z, v.z, u1, u2))
			{
				return true;
			}
		}
	}
	return false;
}

bool ClipLine(const Vector3& x, const Vector3& v, const ConvexPolyhedron3D& p, Real& u1, Real& u2)
{
	for(int i=0; i<p.numPlanes; i++)
		if(!ClipLine(x,v,p.planes[i],u1,u2)) return false;
	return true;
}

} //namespace Math3D
