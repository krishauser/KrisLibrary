
#ifndef MATH3D_CONVEXHULL3D_H
#define MATH3D_CONVECHULL3D_H

#include <KrisLibrary/math/math.h>
#include <KrisLibrary/math/vector.h>
#include "primitives.h"
#include "AABB3D.h"
#include "Box3D.h"
#include <tuple>

namespace Math3D {

using namespace Math; 

/** @ingroup Math3D
 * @brief A 3D convex hull class
 *
 * This class usess SOLID3 library as backend and not so many function are exposed to the user.
 * 
 */
struct ConvexHull3D
{
  void setPoints(const Vector& a);
  void setPoints(const std::vector<Vector3> & a);
  void setPoints(const std::vector<Vector> & a);
  // void setTransformed(const ConvexHull3D&, const Matrix4& xform);
  double distance(const ConvexHull3D &);
  double Distance(const Vector3 &);
  std::tuple<double, Vector3, Vector3> closest_points(const ConvexHull3D &) const;
  Real ClosestPoints(const Vector3& pt,Vector3& cp,Vector3& direction) const;
  Real ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const;
  void Transform(const Matrix4 &T);
  AABB3D GetAABB() const;
  Box3D GetBB() const;
  Vector points;  // points are stored as one dimension array
};

std::ostream& operator << (std::ostream& out,const ConvexHull3D& h);
std::istream& operator >> (std::istream& in,ConvexHull3D& h);

} //namespace Math3D

#endif
