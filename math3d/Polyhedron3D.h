#ifndef MATH3D_POLYHEDRON3D_H
#define MATH3D_POLYHEDRON3D_H

#include "Point.h"
#include "Plane3D.h"

namespace Math3D {

struct Line3D;
struct Segment3D;
struct AABB3D;

/** @ingroup Math3D
 * @brief A convex polyhedron given in a dual representation, planes
 * and vertices of convex hull.
 *
 * Plane normals point to the outside.
 */
struct ConvexPolyhedron3D
{
	ConvexPolyhedron3D();
	~ConvexPolyhedron3D();
	void initialize(int numPlanes, int numVertices);
	void resize(int numPlanes, int numVertices);
	void cleanup();
	void makeFromTriangles(const Point3D* p, int numVertices, int* triangles, int numFaces);	//faces is of size numFaces*3
	void setTransformed(const ConvexPolyhedron3D& in, const Matrix4& T);
	void operator = (const ConvexPolyhedron3D&);
  void getAABB(AABB3D&) const;

	bool intersects(const ConvexPolyhedron3D& other) const;		//returns true if the polyhedra intersect
	void planeExtents(const Plane3D& p,Real& dmin,Real& dmax) const;
	bool planeSplits(const Plane3D& p) const;				//returns true if the plane intersects this polyhedron
	bool planePos(const Plane3D& p) const;				//returns true if this poly is entirely on the positive side of the plane
	bool planeNeg(const Plane3D& p) const;				//returns true if this poly is entirely on the negative side of the plane
	bool contains(const Point3D& v) const;				//returns true if the point is contained
	bool withinDistance(const Point3D& v, Real dist) const;	//returns true if the point is within the given distance
	Real planeDistance(const Point3D& v) const;	//return the closest planar distance to this polyhedron
	bool intersects(const Line3D& l, Real& tmin, Real& tmax) const;
	bool intersects(const Line3D& l) const;
	bool intersects(const Segment3D& l) const;

	bool Read(File& f);
	bool Write(File& f) const;

  Plane3D* planes;
  Point3D* vertices;
  int numPlanes;
  int numVertices;
};

} //namespace Math3d

#endif

