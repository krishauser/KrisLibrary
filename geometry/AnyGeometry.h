#ifndef ANY_GEOMETRY_H
#define ANY_GEOMETRY_H

#include <utils/AnyValue.h>
#include "CollisionMesh.h"

class TiXmlElement;

//forward declarations
namespace Meshing { class VolumeGrid; class PointCloud3D; }
namespace Geometry { class CollisionPointCloud; }
namespace Math3D { class GeometricPrimitive3D; }

namespace Geometry {


  using namespace Math3D;
  using namespace std;

/** @brief A class that stores any kind of geometry we've defined.
 *
 * To get the data, first check the "type" member.  Then call the appropriate
 * AsX method to retrieve the data in the underlying format.
 */
class AnyGeometry3D
{
 public:
  /** 
   * Map of types to classes in the value member
   * - Primitive: GeometricPrimitive3D
   * - TriangleMesh: TriMesh
   * - PointCloud: PointCloud3D
   * - ImplicitSurface: VolumeGrid
   * - Group: vector<AnyGeometry3D>
   */
  enum Type { Primitive, TriangleMesh, PointCloud, ImplicitSurface, Group };

  AnyGeometry3D();
  AnyGeometry3D(const GeometricPrimitive3D& primitive);
  AnyGeometry3D(const Meshing::TriMesh& mesh);
  AnyGeometry3D(const Meshing::PointCloud3D& pc);
  AnyGeometry3D(const Meshing::VolumeGrid& grid);
  AnyGeometry3D(const vector<AnyGeometry3D>& items);
  AnyGeometry3D(const AnyGeometry3D& geom);
  static const char* TypeName(Type type);
  const char* TypeName() const { return AnyGeometry3D::TypeName(type); }
  const GeometricPrimitive3D& AsPrimitive() const;
  const Meshing::TriMesh& AsTriangleMesh() const;
  const Meshing::PointCloud3D& AsPointCloud() const;
  const Meshing::VolumeGrid& AsImplicitSurface() const;
  const vector<AnyGeometry3D>& AsGroup() const;
  GeometricPrimitive3D& AsPrimitive();
  Meshing::TriMesh& AsTriangleMesh();
  Meshing::PointCloud3D& AsPointCloud();
  Meshing::VolumeGrid& AsImplicitSurface();
  vector<AnyGeometry3D>& AsGroup();
  static bool CanLoadExt(const char* ext);
  static bool CanSaveExt(const char* ext);
  bool Load(const char* fn);
  bool Save(const char* fn) const;
  bool Load(istream& in);
  bool Save(ostream& out) const;
  bool Load(TiXmlElement* in);
  bool Save(TiXmlElement* out) const;
  bool Empty() const;
  size_t NumElements() const;
  AABB3D GetAABB() const;
  void Transform(const RigidTransform& T);
  void Transform(const Matrix4& mat);
  void Merge(const vector<AnyGeometry3D>& geoms);

  Type type;
  ///The data, according to the type
  AnyValue data;
};


/** @brief An AnyGeometry with collision detection information.  
 */
class AnyCollisionGeometry3D : public AnyGeometry3D
{
 public:
  AnyCollisionGeometry3D();
  AnyCollisionGeometry3D(const GeometricPrimitive3D& primitive);
  AnyCollisionGeometry3D(const Meshing::TriMesh& mesh);
  AnyCollisionGeometry3D(const Meshing::PointCloud3D& pc);
  AnyCollisionGeometry3D(const Meshing::VolumeGrid& grid);
  AnyCollisionGeometry3D(const AnyGeometry3D& geom);
  AnyCollisionGeometry3D(const vector<AnyGeometry3D>& group);
  AnyCollisionGeometry3D(const AnyCollisionGeometry3D& geom);
  ///May be called after Load()
  void InitCollisions();
  const RigidTransform& PrimitiveCollisionData() const;
  const CollisionMesh& TriangleMeshCollisionData() const;
  const CollisionPointCloud& PointCloudCollisionData() const;
  const RigidTransform& ImplicitSurfaceCollisionData() const;
  const vector<AnyCollisionGeometry3D>& GroupCollisionData() const;
  RigidTransform& PrimitiveCollisionData();
  CollisionMesh& TriangleMeshCollisionData();
  CollisionPointCloud& PointCloudCollisionData();
  RigidTransform& ImplicitSurfaceCollisionData();
  vector<AnyCollisionGeometry3D>& GroupCollisionData();
  AABB3D GetAABB() const;
  Box3D GetBB() const;
  ///Gets the active transform
  RigidTransform GetTransform() const;
  ///Sets the *active* transform without modifying the underlying geometry.  To modify
  ///the geometry, call Transform().  InitCollisions() should be called after modifying
  ///geometry as usual
  void SetTransform(const RigidTransform& T);
  Real Distance(const Vector3& pt) const;
  bool Collides(const AnyCollisionGeometry3D& geom) const;
  bool Collides(const AnyCollisionGeometry3D& geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) const;
  Real Distance(const AnyCollisionGeometry3D& geom) const;
  Real Distance(const AnyCollisionGeometry3D& geom,int& elem1,int& elem2) const;
  bool WithinDistance(const AnyCollisionGeometry3D& geom,Real d) const;
  bool WithinDistance(const AnyCollisionGeometry3D& geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) const;
  bool RayCast(const Ray3D& r,Real* distance=NULL,int* element=NULL) const;
  GeometricPrimitive3D GetElement(int id) const;

  /** The collision data structure, according to the type.
   * - Primitive: RigidTransform
   * - TriangleMesh: CollisionMesh
   * - PointCloud: CollisionPointCloud
   * - VolumeGrid: RigidTransform
   * - Group: vector<AnyCollisionGeometry3D>
   */
  AnyValue collisionData;
  ///Amount by which the underlying geometry is "fattened"
  Real margin;
};

/** @brief A class that stores information regarding a collision query.
 * May be slightly faster than running individual queries.
 */
class AnyCollisionQuery
{
 public:
  AnyCollisionQuery();
  AnyCollisionQuery(const AnyCollisionGeometry3D& a,const AnyCollisionGeometry3D& b);
  AnyCollisionQuery(const AnyCollisionQuery& q);

  ///Returns true if colliding
  bool Collide();
  ///Returns true if colliding, computes all colliding pairs
  bool CollideAll();
  ///Returns true if objects are within distance d
  bool WithinDistance(Real d);
  ///Returns true if objects are within distance d, computes all nearby pairs
  bool WithinDistanceAll(Real d);
  ///Returns an estimate of penetration depth
  Real PenetrationDepth();
  ///Computes the distance with max absolute error absErr, relative error relErr,
  ///and if bound is given, will terminate early if distance > bound
  Real Distance(Real absErr,Real relErr,Real bound=Inf);

  //extracts the pairs of interacting features on a previous call to Collide[All], WithinDistance[All], PenetrationDepth, or Distance
  void InteractingPairs(std::vector<int>& t1,std::vector<int>& t2) const;
  //extracts the pairs of interacting points on a previous call to WithinDistance[All], PenetrationDepth, or Distance.  Points are given
  //in local frames.
  void InteractingPoints(std::vector<Vector3>& p1,std::vector<Vector3>& p2) const;

  const AnyCollisionGeometry3D *a, *b;

  CollisionMeshQueryEnhanced qmesh;
  std::vector<int> elements1,elements2; 
  std::vector<Vector3> points1,points2;
};

} //namespace Geometry

#endif
