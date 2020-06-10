
#ifndef MATH3D_CONVEXHULL3D_H
#define MATH3D_CONVECHULL3D_H

#include <KrisLibrary/utils/AnyValue.h>
#include <KrisLibrary/math/math.h>
#include <KrisLibrary/math/vector.h>
#include "primitives.h"
#include "AABB3D.h"
#include "Box3D.h"
#include <tuple>
#include "SOLID/SOLID.h"

namespace Math3D {

using namespace Math; 

/** @ingroup Math3D
 * @brief A 3D convex hull class
 *
 * This class usess SOLID3 library as backend and not so many function are exposed to the user.
 * To use the full capability of SOLID3, I allow this class to support many PrimitiveShape
 * and complex stuff like polytope (from points), Minkowski (sum of shape), Hull (hull of two shapes), Composite (vector of Hull3D)
 * This makes things much harder and I'm excited to see them.
 */
struct ConvexHull3D;
// define stored data for various stuff
// struct Solid_Polytope {
//   std::vector<double> points;
// };
// struct Solid_Box {
//   Vector3 sizes;
// };
// struct Solid_Cone {
//   Vector2 r_h;
// };
// struct Solid_Cylinder {
//   Vector2 r_h;
// };
// struct Sphere {
//   double radius;
// };
// struct Solid_Point {
//   Vector3 point;
// };
// struct Solid_Line {
//   Vector3 source, target;
// };
// struct Solid_Minkowski {
//   ConvexHull3D shape1, shape2;
// };
// struct Solid_Hull {
//   ConvexHull3D shape1, shape2;
// };
// struct Solid_Composite {
//   std::vector<ConvexHull3D> shapes;
// };

struct ConvexHull3D
{
  typedef std::pair<ConvexHull3D, ConvexHull3D> prch3d;
  //end of definition
  // Tran means transform one shape to form the hull
  // Hull means the hull of two objects with fixed relative transform..
  // HullFree means hull of two objects with free relative transform, they both store a pair of Hulls but creates different collision data
  enum Type { Polytope, Box, Cone, Cylinder, Sphere, Point, Line, Minkowski, Trans, Hull, HullFree, Composite };
  void setPoints(const Vector& a);
  void setPoints(const std::vector<double>& a);
  void setPoints(const std::vector<Vector3> & a);
  void setPoints(const std::vector<Vector> & a);
  void from_hulls(const ConvexHull3D &hull1, const ConvexHull3D &hull2, bool is_free);
  // void setTransformed(const ConvexHull3D&, const Matrix4& xform);
  double distance(const ConvexHull3D &);
  double Distance(const Vector3 &);
  std::tuple<double, Vector3, Vector3> closest_points(const ConvexHull3D &) const;
  Real ClosestPoints(const Vector3& pt,Vector3& cp,Vector3& direction) const;
  Real ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const;
  void Transform(const Matrix4 &T);
  AABB3D GetAABB() const;
  Box3D GetBB() const;
  std::vector<double> &points();
  const std::vector<double> &points() const;
  DT_ShapeHandle shape_handle() const;  // create the shape handle...

  AnyValue data;  // points are stored as one dimension array
  Type type;
};

/** @ingroup Math3D
 * @brief The collision data class for ConvexHull3D, it contains the solid3 data structure
*/
struct CollisionConvexHull3D
{
  CollisionConvexHull3D(const ConvexHull3D& hull);
  CollisionConvexHull3D(const ConvexHull3D& hull, const ConvexHull3D &hull2, bool is_free);
  double Distance(const Vector3 &, const RigidTransform *tran=nullptr);
  // double Distance(CollisionConvexHull3D &, const RigidTransform *tran=nullptr, const RigidTransform *tran2=nullptr);
  // double Distance(const ConvexHull3D &, const RigidTransform *tran=nullptr, const RigidTransform *tran2=nullptr);
  Real ClosestPoints(const Vector3& pt,Vector3& cp,Vector3& direction, const RigidTransform *tran=nullptr);
  // Real ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction, const RigidTransform *tran=nullptr);
  Real ClosestPoints(CollisionConvexHull3D& g, Vector3& cp, Vector3& direction, const RigidTransform *tran=nullptr, const RigidTransform *tran2=nullptr);

  void _update_transform(const RigidTransform *tran=nullptr);
  void _update_relative_transform(const RigidTransform *tran);
  void _update_free_relative_transform(const RigidTransform *tran);

  void _find_support(const double *, double *);

  DT_ObjectHandle& object();
  std::vector<DT_ObjectHandle> & objects();
  // DT_ObjectHandle object;
  AnyValue data;
  ConvexHull3D::Type type;
  double transform[16];
};

std::ostream& operator << (std::ostream& out,const ConvexHull3D& h);
std::istream& operator >> (std::istream& in,ConvexHull3D& h);

} //namespace Math3D

#endif
