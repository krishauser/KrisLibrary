
#ifndef GEOMETRY_CONVEXHULL3D_H
#define GEOMETRY_CONVEXHULL3D_H

#include <KrisLibrary/utils/AnyValue.h>
#include <KrisLibrary/math/math.h>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/Box3D.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <KrisLibrary/utils/IntTriple.h>
#include <tuple>
#include <memory>

/** @file ConvexHull3D.h
 * @ingroup Geometry
 * @brief 3D-D convex hull routines
 */

typedef struct DT_ShapeHandle__* DT_ShapeHandle;
typedef struct DT_ObjectHandle__* DT_ObjectHandle;

namespace Geometry {

using namespace Math; 
using namespace Math3D;

/** @ingroup Geometry
 * @brief Computes the convex hull of the point set using the Qhull library.
 *
 * The result is a set of facets of the convex hull, indexed into the points list.
 *
 * Returns true if successful.
 */
bool ConvexHull3D_Qhull(const std::vector<Vector3>& points,std::vector<std::vector<int> >& facets);
bool ConvexHull3D_Qhull(const std::vector<double>& points,std::vector<std::vector<int> >& facets);

/** @ingroup Geometry
 * @brief Computes the convex hull of the point set using the Qhull library.
 *
 * The result is triangulated arbitrarily.
 * 
 * Returns true if successful.
 */

bool ConvexHull3D_Qhull(const std::vector<Vector3>& points,std::vector<IntTriple>& tris);


/** @ingroup Geometry
 * @brief A 3D convex hull class
 * 
 * Author: Gao Tang
 *
 * This class uses SOLID3 library as backend and not so many function are exposed to the user.
 * To use the full capability of SOLID3, I allow this class to support many PrimitiveShape
 * and complex stuff like polytope (from points), Minkowski (sum of shape), and Hull (hull of
 * two shapes).
 *
 * Distance() returns the signed distance.
 */
class ConvexHull3D
{
public:
  typedef std::vector<double> PolytopeData;
  typedef Vector3 PointData;
  typedef Segment3D LineSegmentData;
  typedef std::pair<ConvexHull3D, ConvexHull3D> MinkowskiData;
  typedef std::pair<ConvexHull3D, RigidTransform> TransData;
  typedef std::pair<ConvexHull3D, ConvexHull3D> HullData;
  //typedef std::vector<ConvexHull3D> CompositeData;
  
  /// Polytope, Box, Cone, Cylinder, Sphere, Point, LineSegment, correspond to SOLID basic data types
  /// Minkowski means the minkowski sum of two ConvexHull3D's (not implemented yet)
  /// Tran means a transformed version of another ConvexHull3D.
  /// Hull means the hull of two objects.
  enum Type { Empty, Polytope, Box, Cone, Cylinder, Sphere, Point, LineSegment, Minkowski, Trans, Hull};
  ConvexHull3D();
  void SetPoint(const Vector3& a);
  void SetPoints(const Vector& a);
  void SetPoints(const std::vector<double>& a);
  void SetPoints(const std::vector<Vector3> & a);
  void SetPoints(const std::vector<Vector> & a);
  void SetLineSegment(const Segment3D& s);
  ///Sets this as a transformed version of hull
  void SetTrans(const ConvexHull3D &hull, const RigidTransform& xform);
  void SetHull(const ConvexHull3D &hull1, const ConvexHull3D &hull2);
  bool Contains(const Vector3& pt);
  Real Distance(const Vector3& pt);
  bool Collides(const ConvexHull3D& g,Vector3* common_point=NULL);
  Real Distance(const ConvexHull3D & g);
  ///Returns distance, point on this geometry, and point on the other geometry
  std::tuple<Real, Vector3, Vector3> ClosestPoints(const ConvexHull3D & other) const;
  Real ClosestPoint(const Vector3& pt,Vector3& cp,Vector3& direction) const;
  Real ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const;
  void Transform(const Matrix4 &T);
  AABB3D GetAABB() const;
  Box3D GetBB() const;
  PolytopeData& AsPolytope();
  const PolytopeData& AsPolytope() const;
  const PointData& AsPoint() const;
  const LineSegmentData& AsLineSegment() const;
  const TransData& AsTrans() const;
  const HullData& AsHull() const;
  size_t NumPrimitives() const;
  GeometricPrimitive3D GetPrimitive(int index) const;

  AnyValue data;  // Stored in the same format as the [Type]Data typedefs listed above
  Type type;

  struct ShapeHandleContainer {
    ShapeHandleContainer(DT_ShapeHandle data);
    ~ShapeHandleContainer();
    DT_ShapeHandle data;
  };
  std::shared_ptr<ShapeHandleContainer> shapeHandle; //internal SOLID data
};

/** @ingroup Geometry
 * @brief The collision data class for ConvexHull3D that contains the solid3 data
 * structure moved by some rigid transform.
 *
 * This also lets you update the relative transform for a Hull ConvexHull3D datatype.
 * 
 * Author: Gao Tang
 */
class CollisionConvexHull3D
{
public:
  CollisionConvexHull3D(const ConvexHull3D& hull);
  CollisionConvexHull3D();  // no value, waiting to be initialized
  bool Contains(const Vector3& pt) const;
  Real Distance(const Vector3 &pt) const;
  bool Collides(const CollisionConvexHull3D& geometry, Vector3* common_point=nullptr) const;
  Real ClosestPoint(const Vector3& pt,Vector3& cp,Vector3& direction) const;
  Real ClosestPoints(const CollisionConvexHull3D& g, Vector3& cp, Vector3& direction) const;

  void UpdateTransform(const RigidTransform& tran);
  ///For Hull objects, updates the relative transform of the second object
  void UpdateHullSecondRelativeTransform(const RigidTransform& tran);

  Vector3 FindSupport(const Vector3& dir) const;

  ConvexHull3D::Type type;
  struct ObjectHandleContainer {
    ObjectHandleContainer(DT_ObjectHandle data);
    ~ObjectHandleContainer();
    DT_ObjectHandle data;
  };
  std::shared_ptr<ObjectHandleContainer> objectHandle;
  DT_ShapeHandle shapeHandle;
  double transform[16];
};

std::ostream& operator << (std::ostream& out,const ConvexHull3D& h);
std::istream& operator >> (std::istream& in,ConvexHull3D& h);

} //namespace Geometry

#endif
