#ifndef MATH3D_AABB2D_H
#define MATH3D_AABB2D_H

#include "Point.h"
#include <KrisLibrary/math/math.h>
class File;

namespace Math3D {

/** @ingroup Math3D
 * @brief A 2D axis-aligned bounding box
 */
struct AABB2D
{
  AABB2D();
  AABB2D(const Vector2& bmin,const Vector2& bmax);
  AABB2D(const AABB2D&);
  bool Read(File& f);
  bool Write(File& f) const;

  void justify();  ///<swaps negative sized entries (where min<max)
  void setTransform(const AABB2D&,const Matrix3& mat);
  void inplaceTransform(const Matrix3& mat);
  void minimize();
  void maximize();  
  void expand(const Point2D&);
  void setPoint(const Point2D&);
  void setIntersection(const AABB2D&);
  void setUnion(const AABB2D&);
  void getSize(Vector2&) const;
  void getMidpoint(Point2D&) const;

  bool contains(const Point2D&) const;
  bool contains(const AABB2D&) const;
  bool intersects(const AABB2D&) const;
  Real distance(const Point2D&) const;
  Real distance(const Point2D& pt,Point2D& closest) const;
  Real distanceSquared(const Point2D& pt,Point2D& closest) const;
  Real signedDistance(const Point2D&) const;
  Real signedDistance(const Point2D& pt,Point2D& closest) const;

  Vector2 bmin, bmax;
};

} //namespace Math3D

#endif
