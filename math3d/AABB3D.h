#ifndef MATH3D_AABB3D_H
#define MATH3D_AABB3D_H

#include <KrisLibrary/math/math.h>
#include "Point.h"
class File;

namespace Math3D {

/** @brief A 3D axis-aligned bounding box
 * @ingroup Math3D
 */
struct AABB3D
{
  AABB3D();
  AABB3D(const Vector3& bmin,const Vector3& bmax);
  AABB3D(const AABB3D&);
  bool Read(File& f);
  bool Write(File& f) const;
  void Print(std::ostream& out) const;

  void justify();  ///<swaps negative sized entries (where min>max)
  void setTransform(const AABB3D&,const Matrix4& mat);
  void inplaceTransform(const Matrix4& mat);
  void minimize();
  void maximize();  
  void expand(const Point3D&);
  void setPoint(const Point3D&);
  void setIntersection(const AABB3D&);
  void setUnion(const AABB3D&);
  void getSize(Vector3&) const;
  Vector3 size() const;
  void getMidpoint(Point3D&) const;
  Vector3 midpoint() const;
  bool contains(const Point3D&) const;
  bool contains(const AABB3D&) const;
  bool intersects(const AABB3D&) const;
  Real distance(const Point3D&) const;
  Real distance(const Point3D& pt,Point3D& closest) const;
  Real distanceSquared(const Point3D& pt,Point3D& closest) const;
  Real signedDistance(const Point3D&) const;
  Real signedDistance(const Point3D& pt,Point3D& surface) const;
  Real distance(const AABB3D& bb) const;
  Real distance(const AABB3D& bb,Point3D& myclosest,Point3D& bbclosest) const;

  Vector3 bmin, bmax;
};

std::ostream& operator << (std::ostream& out,const AABB3D& bb);
std::istream& operator >> (std::istream& in,AABB3D& bb);

} //namespace Math3D

#endif

