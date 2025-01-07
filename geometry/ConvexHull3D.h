
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
  typedef Sphere3D SphereData;
  typedef Segment3D LineSegmentData;
  typedef Box3D BoxData;
  typedef std::pair<ConvexHull3D, ConvexHull3D> MinkowskiData;
  typedef std::pair<ConvexHull3D, RigidTransform> TransData;
  typedef std::pair<ConvexHull3D, ConvexHull3D> HullData;
  //typedef std::vector<ConvexHull3D> CompositeData;
  
  /// Polytope, Box, Cone, Cylinder, Sphere, Point, LineSegment, correspond to SOLID basic data types
  /// Minkowski means the minkowski sum of two ConvexHull3D's (not implemented yet)
  /// Trans means a transformed version of another ConvexHull3D.
  /// Hull means the hull of two objects.
  enum Type { Empty, Polytope, Box, Cone, Cylinder, Sphere, Point, LineSegment, Minkowski, Trans, Hull};
  
  ConvexHull3D();
  ConvexHull3D(const ConvexHull3D& rhs);
  const ConvexHull3D& operator = (const ConvexHull3D& rhs); 
  bool Set(const GeometricPrimitive3D& g);
  void Set(const Vector3& a);
  void Set(const Sphere3D& s);
  void Set(const Segment3D& s);
  void Set(const Triangle3D& tri);  ///Note: triangle is stored as a polytope
  void Set(const AABB3D& bb);
  void Set(const Box3D& b);
  void SetPoints(const std::vector<Vector3> & a);
  void SetPointBuffer(const Vector& a);
  void SetPointBuffer(const std::vector<double>& a);
  /// Sets this as a transformed version of hull.  Note: this stores a reference to hull, and it must be kept alive.
  void SetTrans(const ConvexHull3D &hull, const RigidTransform& xform);
  /// Sets this as the convex hull of two other objects.  Note: this stores a reference to hull1 and hull2, and they must be kept alive.
  void SetHull(const ConvexHull3D &hull1, const ConvexHull3D &hull2);
  bool Contains(const Vector3& pt) const;
  Real Distance(const Vector3& pt) const;
  bool Collides(const ConvexHull3D& g,Vector3* common_point=NULL) const;
  Real Distance(const ConvexHull3D & g) const;
  ///Returns distance, point on this geometry, and point on the other geometry
  std::tuple<Real, Vector3, Vector3> ClosestPoints(const ConvexHull3D & other) const;
  Real ClosestPoint(const Vector3& pt,Vector3& cp,Vector3& direction) const;
  Real ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const;
  bool RayCast(const Ray3D& r,Real* param=NULL,Real maxParam=Inf) const;
  void Transform(const Matrix4 &T);
  AABB3D GetAABB() const;
  Box3D GetBB() const;
  PolytopeData& AsPolytope();
  const PolytopeData& AsPolytope() const;
  const PointData& AsPoint() const;
  const SphereData& AsSphere() const;
  const LineSegmentData& AsLineSegment() const;
  const BoxData& AsBox() const;
  const TransData& AsTrans() const;
  const HullData& AsHull() const;
  size_t NumPrimitives() const;
  GeometricPrimitive3D GetPrimitive(int index) const;
  std::vector<Vector3> GetPoints() const;

  AnyValue data;  // Stored in the same format as the [Type]Data typedefs listed above
  Type type;

  struct ShapeHandleContainer {
    ShapeHandleContainer(DT_ShapeHandle data, DT_ShapeHandle data2=NULL);
    ~ShapeHandleContainer();
    DT_ShapeHandle data;
    DT_ShapeHandle data2;
    DT_ObjectHandle object_ref;  //object corresponding to data at rest
  };
  std::shared_ptr<ShapeHandleContainer> shapeHandle; //internal SOLID data
};

std::ostream& operator << (std::ostream& out,const ConvexHull3D& h);
std::istream& operator >> (std::istream& in,ConvexHull3D& h);

} //namespace Geometry

#endif
