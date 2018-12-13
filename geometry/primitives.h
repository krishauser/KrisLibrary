#ifndef GEOMETRY_PRIMITIVES_H
#define GEOMETRY_PRIMITIVES_H 

#include <KrisLibrary/math3d/geometry2d.h>
#include <KrisLibrary/math3d/geometry3d.h>

/** @file geometry/primitives.h
 * @ingroup Geometry
 * @brief Contains ordering primitives used in geometry computations.
 */

/** @ingroup Geometry
 * @brief Contains all definitions in the Geometry package.
 */
namespace Geometry {

  using namespace Math3D;

  /** @addtogroup Geometry */
  /*@{*/

/// Lexical < order on 2D points
inline bool Lexical2DOrder (const Point2D& p1,const Point2D& p2)
{
  if(p1.x < p2.x) return true;
  else if(p1.x > p2.x) return false;
  return (p1.y < p2.y);
}

/// Lexical < order on 3D points
inline bool Lexical3DOrder (const Point3D& p1,const Point3D& p2)
{
  if(p1.x < p2.x) return true;
  else if(p1.x > p2.x) return false;
  if(p1.y < p2.y) return true;
  else if(p1.y > p2.y) return false;
  return (p1.z < p2.z);
}

/** @brief Orientation of p2 relative to p1, relative to p0.
 *
 * @return
 * >0 for p2 left of the line through p0 and p1
 * =0 for p2 on the line
 * <0 for p2 right of the line
 */
inline Real Orient2D(const Point2D& p0, const Point2D& p1, const Point2D& p2)
{
  return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
}


/// A < comparison operator that gives an angular ordering around the
/// point p0.
struct Angular2DOrdering
{
  inline Angular2DOrdering(const Point2D& _p0) :p0(_p0) {}
  inline bool operator () (const Point2D& p1,const Point2D& p2) const
  {
    return Orient2D(p0,p1,p2) > 0;
  }
  Point2D p0;
};

  /*@}*/

} // namespace Geometry

#endif
