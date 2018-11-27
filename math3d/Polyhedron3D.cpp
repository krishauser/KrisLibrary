#include <KrisLibrary/Logger.h>
#include "Polyhedron3D.h"
#include "geometry3d.h"
#include <math/Interval.h>
#include <iostream>
#include "clip.h"
using namespace Math3D;
using namespace std;

ConvexPolyhedron3D::ConvexPolyhedron3D()
  :planes(NULL),vertices(NULL),numPlanes(0),numVertices(0)
{}

ConvexPolyhedron3D::~ConvexPolyhedron3D()
{
	cleanup();
}

void ConvexPolyhedron3D::initialize(int np, int nv)
{
	cleanup();
	numPlanes = np;
	numVertices = nv;
	planes = new Plane3D[np];
	vertices = new Point3D[nv];
}

void ConvexPolyhedron3D::cleanup()
{
	numPlanes=0;
	numVertices=0;
	SafeArrayDelete(planes);
	SafeArrayDelete(vertices);
}

void ConvexPolyhedron3D::resize(int np, int nv)
{
	if(numPlanes != np || numVertices != nv)
		initialize(np,nv);
}

void ConvexPolyhedron3D::makeFromTriangles(const Point3D* p, int nv, int* tris, int nf)
{
	resize(nf,nv);
	int i;
	for(i=0; i<nv; i++)
		vertices[i] = p[i];
	for(i=0; i<nf; i++) {
		int a,b,c;
		a = tris[i*3];
		b = tris[i*3+1];
		c = tris[i*3+2];
		planes[i].setPoints(p[a], p[b], p[c]);
	}
}

void ConvexPolyhedron3D::operator = (const ConvexPolyhedron3D& p)
{
	resize(p.numPlanes, p.numVertices);
	int i;
	for(i=0; i<numVertices; i++)
		vertices[i] = p.vertices[i];
	for(i=0; i<numPlanes; i++)
		planes[i] = p.planes[i];
}

void ConvexPolyhedron3D::getAABB(AABB3D& b) const
{
  b.setPoint(vertices[0]);
  for(int i=1; i<numVertices; i++)
    b.expand(vertices[i]);
}

bool ConvexPolyhedron3D::intersects(const ConvexPolyhedron3D& other) const
{
	int i;
	for(i=0; i<other.numPlanes; i++)
		if(planePos(other.planes[i])) return false;

	for(i=0; i<numPlanes; i++)
		if(other.planePos(planes[i])) return false;

	//check edge/edge 
	LOG4CXX_INFO(KrisLibrary::logger(),"Not done with edge/edge splitting planes");
	abort();

	return true;
}

void ConvexPolyhedron3D::planeExtents(const Plane3D& p,Real& dmin,Real& dmax) const
{
  if(numVertices == 0) {
    dmin=Inf;
    dmax=-Inf;
    return;
  }
  dmin=dmax = p.distance(vertices[0]);
  Real d;
  for(int i=1; i<numVertices; i++) {
      d = p.distance(vertices[i]);
      if(d < dmin) dmin=d;
      else if(d > dmax) dmax=d;
  }	
}

bool ConvexPolyhedron3D::planeSplits(const Plane3D& p) const
{
	ClosedInterval x;
	x.setEmpty();
	Real vpos;
	for(int i=0; i<numVertices; i++)
	{
		vpos = p.distance(vertices[i]);
		x.expand(vpos);

		if(x.contains(Zero))
			return true;
	}

	return false;
}

bool ConvexPolyhedron3D::planePos(const Plane3D& p) const
{
	Real vpos;
	for(int i=0; i<numVertices; i++)
	{
		vpos = p.distance(vertices[i]);
		if(vpos < Zero)
			return false;
	}

	return true;
}

bool ConvexPolyhedron3D::planeNeg(const Plane3D& p) const
{
	Real vpos;
	for(int i=0; i<numVertices; i++)
	{
		vpos = p.distance(vertices[i]);
		if(vpos > Zero)
			return false;
	}

	return true;
}

bool ConvexPolyhedron3D::contains(const Point3D& v) const
{
	return withinDistance(v,Zero);
}

bool ConvexPolyhedron3D::withinDistance(const Point3D& v, Real dist) const
{
	for(int i=0; i<numPlanes; i++)
		if(planes[i].distance(v) > dist)
			return false;
	return true;
}

Real ConvexPolyhedron3D::planeDistance(const Point3D& v) const
{
	if(numPlanes<=0) return 0;
	Real dmax=planes[0].distance(v);
	for(int i=1;i<numPlanes;i++) {
		Real d=planes[i].distance(v);
		if(d > dmax) dmax=d;
	}
	return dmax;
}

void ConvexPolyhedron3D::setTransformed(const ConvexPolyhedron3D& in, const Matrix4& T)
{
	resize(in.numPlanes, in.numVertices);

	int i;
	for(i=0; i<numVertices; i++)
		T.mulPoint(in.vertices[i], vertices[i]);

	for(i=0; i<numPlanes; i++)
		planes[i].setTransformed(in.planes[i], T);
}

bool ConvexPolyhedron3D::intersects(const Line3D& l, Real& tmin, Real& tmax) const
{
	return ClipLine(l.source,l.direction,*this,tmin,tmax);
}

bool ConvexPolyhedron3D::intersects(const Line3D& l) const
{
	Real tmin=-Inf, tmax=Inf;
	return intersects(l,tmin,tmax);
}

bool ConvexPolyhedron3D::intersects(const Segment3D& s) const
{
  Real tmin=0,tmax=1;
  return ClipLine(s.a,s.b-s.a,*this,tmin,tmax);
}
