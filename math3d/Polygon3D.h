#ifndef MATH3D_POLYGON3D_H
#define MATH3D_POLYGON3D_H

#include "Point.h"
#include <vector>

namespace Math3D {

struct Plane3D;
struct Segment3D;
struct Triangle3D;
struct AABB3D;
struct Polygon2D;

/** @ingroup Math3D
 * @brief An arbitrary connected polygon in 3D space given by a vertex list
 *
 * Vertices are numbered 0...n-1. 
 * Edge i connects verts i and i+1 (looping around to 0).
 */
struct Polygon3D
{
  inline size_t next(size_t i) const { return (i+1>=vertices.size()? 0: i+1); }
  inline size_t prev(size_t i) const { return (i==0? vertices.size()-1: i-1); }
  void triangulateConvex(std::vector<Triangle3D>& tris) const;
  Real areaConvex() const;
  Vector3 centroidConvex() const;
  void setTransformed(const Polygon2D& in, const Matrix4& T);  ///< assigns coordinate z=0 for in
  void setTransformed(const Polygon3D& in, const Matrix4& T);
  
  void getEdge(int i,Segment3D& ei) const;
  ///sets p to contain (pi,pi+1,pi+2)
  void getPlane(int i,Plane3D& p) const;
  ///uses a least-squares fit
  void getPlaneFit(Plane3D& p) const;
  ///maximum abs distance to the plane
  Real maxDistance(const Plane3D& p) const;
  ///gets the best-fit plane, and projects the polygon onto that 2d plane.
  ///Returns the matrix that transforms p to this.
  void getPlanarPolygon(Polygon2D& p,Matrix4& T) const;

  //bool planeSplits(const Plane3D& p) const;				///<returns true if the plane intersects this polyhedron
  //bool planePos(const Plane3D& p) const;				///<returns true if this is entirely on the positive side of this plane
  //bool planeNeg(const Plane3D& p) const;				///<returns true if this is entirely on the negative side of this plane

  void getAABB(AABB3D&) const;
  
  bool Read(File& f);
  bool Write(File& f) const;

  ///a list of points around the boundary
  std::vector<Point3D> vertices;
};

std::ostream& operator << (std::ostream& out,const Polygon3D& b);
std::istream& operator >> (std::istream& in, Polygon3D& b);


} //namespace Math

#endif
