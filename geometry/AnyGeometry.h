#ifndef ANY_GEOMETRY_H
#define ANY_GEOMETRY_H

#include "GeometryType.h"
#include "CollisionMesh.h"

//forward declarations
namespace Meshing { template <class T> class VolumeGridTemplate; typedef VolumeGridTemplate<Math::Real> VolumeGrid; class PointCloud3D; }
namespace Math3D { class GeometricPrimitive3D; }
namespace GLDraw { class GeometryAppearance; }
namespace Geometry { class ConvexHull3D; }

namespace Geometry {

  using namespace Math3D;
  using namespace std;

typedef DistanceQuerySettings AnyDistanceQuerySettings;
typedef DistanceQueryResult AnyDistanceQueryResult;
typedef ContactsQuerySettings AnyContactsQuerySettings;
typedef ContactsQueryResult AnyContactsQueryResult;
class Collider3DPrimitive;
class Collider3DConvexHull;
class CollisionMesh;
class CollisionPointCloud;
class Collider3DImplicitSurface;
class Collider3DOccupancyGrid;
class Collider3DHeightmap;

/** @brief Optional flags for collision data hints.  The collision data preprocessor
 * may use these to improve preprocessing / collision checking performance.
 * 
 * - CollisionDataHintFast: The preprocessor should prioritize speed rather than tightness, e.g., the data is dynamic.
 * - CollisionDataHintTemporallyCoherent: The collision / proximity checker will be called with small changes in transform
 * - CollisionDataHintGridlike: The elements are arranged approximately along a regular grid.
 * 
 */
enum {
  CollisionDataHintFast=0x01,     
  CollisionDataHintTemporallyCoherent=0x02,
  CollisionDataHintGridlike=0x04
};


/** @brief Types to flag how VolumeGrid values should be interpreted.
 * 
 * - Unknown: not marked.
 * - ImplicitSurface: surface of the geometry correspond to the zero level set, <0 inside, >0 outside.
 * - SDF: signed distance function. Functionally equivalent to ImplicitSurface but actually measures distance.
 * - TSDF: truncated signed distance function.  Functionally equivalent to ImplicitSurface but estimates distance up to some threshold.
 * - OccupancyGrid: stores the occupancy of cells, in the range [0,1].
 * - Density: stores a density in cells. An implicit surface is defined with 0.5 - density.
 */
enum { 
  VolumeGridUnknown=0,
  VolumeGridImplicitSurface=1,
  VolumeGridSDF=2,
  VolumeGridTSDF=3,
  VolumeGridOccupancyGrid=4,
  VolumeGridDensity=5
};


/** @brief A container class that stores any kind of geometry we've defined.
 *
 * To get the data, first check the "type" member.  Then call the appropriate
 * AsX method to retrieve the data in the underlying format.
 *
 * Some types may also store auxiliary appearance data.  Right now the only
 * thing that's supported is the GLDraw::GeometryAppearance data for
 * TriangleMesh data.  This may also be NULL, for geometries that don't store
 * appearance information.  You can get these using the XAppearanceData
 * functions.
 */
class AnyGeometry3D
{
 public:
  typedef Geometry3D::Type Type;

  AnyGeometry3D();
  AnyGeometry3D(const GeometricPrimitive3D& primitive);
  AnyGeometry3D(const ConvexHull3D& cvxhull);
  AnyGeometry3D(const Meshing::TriMesh& mesh);
  AnyGeometry3D(const Meshing::PointCloud3D& pc);
  AnyGeometry3D(const Meshing::VolumeGrid& grid,int value_type=VolumeGridImplicitSurface);
  AnyGeometry3D(const Meshing::Heightmap& hm);
  AnyGeometry3D(const vector<AnyGeometry3D>& items);
  AnyGeometry3D(const AnyGeometry3D& geom);
  AnyGeometry3D(AnyGeometry3D&& geom) = default;
  AnyGeometry3D& operator = (const AnyGeometry3D& rhs);
  AnyGeometry3D& operator = (AnyGeometry3D&& rhs) = default;
  static const char* TypeName(Type type) { return Geometry3D::TypeName(type); }
  const char* TypeName() const { return Geometry3D::TypeName(type); }
  const GeometricPrimitive3D& AsPrimitive() const;
  const ConvexHull3D& AsConvexHull() const;
  const Meshing::TriMesh& AsTriangleMesh() const;
  const Meshing::PointCloud3D& AsPointCloud() const;
  const Meshing::VolumeGrid& AsImplicitSurface() const;
  const Meshing::VolumeGrid& AsOccupancyGrid() const;
  const Meshing::Heightmap& AsHeightmap() const;
  const vector<AnyGeometry3D>& AsGroup() const;
  GeometricPrimitive3D& AsPrimitive();
  ConvexHull3D& AsConvexHull();
  Meshing::TriMesh& AsTriangleMesh();
  Meshing::PointCloud3D& AsPointCloud();
  Meshing::VolumeGrid& AsImplicitSurface();
  Meshing::VolumeGrid& AsOccupancyGrid();
  Meshing::Heightmap& AsHeightmap();
  vector<AnyGeometry3D>& AsGroup();
  GLDraw::GeometryAppearance* TriangleMeshAppearanceData();
  const GLDraw::GeometryAppearance* TriangleMeshAppearanceData() const;
  static bool CanLoadExt(const char* ext);
  static bool CanSaveExt(const char* ext);
  bool Load(const char* fn);
  bool Save(const char* fn) const;
  bool Load(istream& in);
  bool Save(ostream& out) const;
  bool Empty() const;
  size_t NumElements() const;
  GeometricPrimitive3D GetElement(int elem) const;
  AABB3D GetAABB() const;
  bool Transform(const RigidTransform& T);
  bool Transform(const Matrix4& mat);
  void Union(const vector<AnyGeometry3D>& geoms);
  /// Merges one geometry into this one.
  ///
  /// Some types (such as implicit surfaces) may support merging geometries
  /// with mismatched types.  
  ///
  /// if Tgeom is provided, geom should be interpreted as being in the local
  /// coordinates of Tgeom.
  bool Merge(const AnyGeometry3D& other, const RigidTransform* Tgeom=NULL);
  ///Converts to another geometry type, storing the result in res and returning true
  ///if the conversion is available. 
  ///
  ///If restype==type, returns a copy of this.
  ///
  ///param is interpreted as follows, with 0 being a "reasonable" default value:
  ///- Primitive -> TriangleMesh: desired resolution of mesh
  ///- Primitive -> PointCloud: desired resolution of point cloud
  ///- PointCloud -> OccupancyGrid: desired width of volume grid cells
  ///- TriangleMesh -> PointCloud: desired resolution of point cloud
  ///- TriangleMesh -> ImplicitSurface: desired width of volume grid cells
  ///- TriangleMesh -> OccupancyGrid: desired width of volume grid cells
  ///- ImplicitSurface -> TriangleMesh: level set to be extracted
  ///- ImplicitSurface -> PointCloud: level set to be extracted
  ///- OccupancyGrid -> TriangleMesh: threshold of occupancy
  bool Convert(Type restype,AnyGeometry3D& res,Real param=0) const;
  ///Re-meshes the geometry at the desired resolution, storing the result into res.
  ///
  ///Resolution is interpreted as
  ///- Primitive, ConvexHull: ignored
  ///- TriangleMesh: desired length of triangle edges.  Refinement only.
  ///- PointCloud: desire 1 point per grid cell of width resolution.  Coarsen only.
  ///- ImplicitSurface: new cell width.
  ///- OccupancyGrid: new cell width.
  ///- Group: sent to each sub-geometry
  bool Remesh(Real resolution,AnyGeometry3D& res,bool refine=true,bool coarsen=true) const;
  ///Extracts a slice from the geometry at a given plane.  The plane is specified
  ///as the local X-Y plane of the given world coordinates T.  The resulting values are
  ///given in T's local coordinates.
  ///
  ///For point clouds, tol must be > 0.
  bool Slice(const RigidTransform& T,AnyGeometry3D& res,Real tol=0) const;
  ///Extracts a region of interest (bounding box) from the geometry.  The region of interest
  ///may be specified in the flag as all geometry intersecting the box, within the box, 
  ///or touching the box.  It is also possible to invert the selection.  See the
  ///ExtractROIFlagX enum for more details.
  bool ExtractROI(const AABB3D& bb,AnyGeometry3D& res,int flag=1) const;
  bool ExtractROI(const Box3D& bb,AnyGeometry3D& res,int flag=1) const;

  Type type;
  ///The data, which is one of the types in AnyGeometryTypeImpl.h
  shared_ptr<Geometry3D> data;
};


/** @brief An AnyGeometry with collision detection information.  
 * 
 * Because collision detection information is slow to initialize, this will
 * only initialize it when needed to answer a collision query.
 *
 * Note: Distance() returns the distance to the other geometry. If
 * the result is <= 0, the two objects are in collision. A geometry type
 * may support returning a signed distance if the objects are penetrating, i.e.
 * Distance() < 0.
 */
class AnyCollisionGeometry3D : public AnyGeometry3D
{
 public:
  AnyCollisionGeometry3D();
  AnyCollisionGeometry3D(const GeometricPrimitive3D& primitive);
  AnyCollisionGeometry3D(const ConvexHull3D& primitive);
  AnyCollisionGeometry3D(const Meshing::TriMesh& mesh);
  AnyCollisionGeometry3D(const Meshing::PointCloud3D& pc);
  AnyCollisionGeometry3D(const Meshing::VolumeGrid& grid,int value_type=VolumeGridImplicitSurface);
  AnyCollisionGeometry3D(const Meshing::Heightmap& hm);
  AnyCollisionGeometry3D(const AnyGeometry3D& geom);
  AnyCollisionGeometry3D(const vector<AnyGeometry3D>& group);
  AnyCollisionGeometry3D(const AnyCollisionGeometry3D& geom);
  AnyCollisionGeometry3D(AnyCollisionGeometry3D&& geom) = default;
  AnyCollisionGeometry3D& operator = (const AnyCollisionGeometry3D& rhs);
  AnyCollisionGeometry3D& operator = (AnyCollisionGeometry3D&& rhs) = default;
  ///If the collision detection data structure isn't initialized yet,
  ///this initializes it.  Constructors DO NOT call this, meaning that
  ///upon construction the XCollisionData functions must not be called.
  void InitCollisionData();
  ///Call this any time the underlying geometry changes to reinitialize the
  ///collision detection data structure.
  void ReinitCollisionData();
  ///Returns true if the collision data is initialized
  bool CollisionDataInitialized() const { return collider != NULL; }
  ///Clears the current collision data
  void ClearCollisionData() { collider.reset(); }
  const Collider3DPrimitive& PrimitiveCollisionData() const;
  const Collider3DConvexHull& ConvexHullCollisionData() const;
  const CollisionMesh& TriangleMeshCollisionData() const;
  const CollisionPointCloud& PointCloudCollisionData() const;
  const Collider3DImplicitSurface& ImplicitSurfaceCollisionData() const;
  const Collider3DOccupancyGrid& OccupancyGridCollisionData() const;
  const Collider3DHeightmap& HeightmapCollisionData() const;
  const vector<AnyCollisionGeometry3D>& GroupCollisionData() const;
  Collider3DPrimitive& PrimitiveCollisionData();
  Collider3DConvexHull& ConvexHullCollisionData();
  CollisionMesh& TriangleMeshCollisionData();
  CollisionPointCloud& PointCloudCollisionData();
  Collider3DImplicitSurface& ImplicitSurfaceCollisionData();
  Collider3DOccupancyGrid& OccupancyGridCollisionData();
  Collider3DHeightmap& HeightmapCollisionData();
  vector<AnyCollisionGeometry3D>& GroupCollisionData();
  ///Performs a type conversion, also copying the active transform and collision margin.
  ///May be a bit faster than AnyGeometry3D.Convert() for some conversions
  ///(TriangleMesh->ImplicitSurface, specifically) and will respect collision margins
  bool Convert(Type restype,AnyCollisionGeometry3D& res,Real param=0);
  ///Returns an axis-aligned bounding box in the world coordinate frame
  ///containing the transformed geometry.  Note: if collision data is
  ///initialized, this returns a bound around the transformed bounding
  ///volume of the collision data, which is not necessarily tight. 
  ///Note: if collision data is not yet initialized, this performs
  ///the potentially slower AnyGeometry3D::GetAABB call.
  AABB3D GetAABB() const; 
  ///Returns an oriented bounding box in the world coordinate frame
  ///containing the transformed geometry.  Note: if collision data is not yet
  ///initialized, this performs the potentially slower AnyGeometry3D::GetAABB
  ///call.
  Box3D GetBB() const;
  ///Returns a tight bounding box around the data.  Slower than GetAABB 
  ///but tighter.
  AABB3D GetAABBTight() const; 
  ///Gets the active transform
  RigidTransform GetTransform() const;
  ///Sets the *active* transform without modifying the underlying geometry. 
  ///To modify the geometry, call Transform().  Note: if you do actually 
  ///modify the geometry using Transform(), ReinitCollisions() should be
  ///called.
  void SetTransform(const RigidTransform& T);
  /// Modifies the underlying geometry in the *local* frame by T.  This is
  /// equivalent to AnyGeometry3D::Transform except that it will warn if
  /// the collision data needs to be re-initialized.
  bool Transform(const RigidTransform& T);
  bool Transform(const Matrix4& mat);

  ///Groups together these geometries, as usual
  void Union(const vector<AnyGeometry3D>& geoms);
  ///Groups together these geometries.  For compatible types, they are merged
  ///into a single geometry.  For non-compatible types, a Group is created.
  ///In this case, this geometry is placed such that
  ///the active transform is the same as the active transform for the
  ///first geometry, and all subsequent geometries are transformed relative
  ///to the first.
  void Union(const vector<AnyCollisionGeometry3D>& geoms);
  /// Merges one geometry into this one.
  ///
  /// The current contents are preserved and the other geometry is blended
  /// with this one (interpreted as both being
  /// in the world frame, but the other's data is transformed to the local frame
  /// when blending with this object's geometry). 
  ///
  /// Some types (such as implicit surfaces) may support merging geometries
  /// with mismatched types.
  bool Merge(const AnyCollisionGeometry3D& other);
  ///Computes the furthest point on the geometry in the direction dir
  bool Support(const Vector3& dir,Vector3& pt);
  bool Collides(AnyCollisionGeometry3D& geom);
  bool Collides(AnyCollisionGeometry3D& geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
  Real Distance(AnyCollisionGeometry3D& geom);
  Real Distance(const Vector3& pt);
  AnyDistanceQueryResult Distance(const Vector3& pt,const AnyDistanceQuerySettings& settings);
  AnyDistanceQueryResult Distance(AnyCollisionGeometry3D& geom,const AnyDistanceQuerySettings& settings);
  bool WithinDistance(AnyCollisionGeometry3D& geom,Real d);
  bool WithinDistance(AnyCollisionGeometry3D& geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
  AnyContactsQueryResult Contacts(AnyCollisionGeometry3D& other,const AnyContactsQuerySettings& settings);
  bool RayCast(const Ray3D& r,Real* distance=NULL,int* element=NULL);
  ///Extracts a slice from the geometry at a given plane.  The plane is specified
  ///as the local X-Y plane of the given world coordinates T.  The resulting values are
  ///given with data in T's local coordinates, with active transform T.
  ///
  ///For point clouds, tol must be > 0.
  bool Slice(const RigidTransform& T,AnyCollisionGeometry3D& res,Real tol=0) const;
  ///Extracts a region of interest (bounding box) from the geometry.  The region of interest
  ///may be specified in the flag as all geometry intersecting the box, within the box, 
  ///or touching the box.  It is also possible to invert the selection.  See the
  ///ExtractROIFlagX enum for more details.
  bool ExtractROI(const AABB3D& bb,AnyCollisionGeometry3D& res,int flag=1) const;
  bool ExtractROI(const Box3D& bb,AnyCollisionGeometry3D& res,int flag=1) const;

  shared_ptr<Collider3D> collider;
  ///Amount by which the underlying geometry is "fattened"
  Real margin;
  ///The current transform, used if the collision data is not initialized yet.
  RigidTransform currentTransform;
  ///A hint for initializing the collision data appropriately.  See flags starting
  ///with CollisionDataHint.  Default 0.
  int collisionHint;
};

/** @brief A class that stores information regarding a collision query.
 * May be slightly faster than running individual queries.
 */
class AnyCollisionQuery
{
 public:
  AnyCollisionQuery();
  AnyCollisionQuery(AnyCollisionGeometry3D& a,AnyCollisionGeometry3D& b);
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

  AnyCollisionGeometry3D *a, *b;

  CollisionMeshQueryEnhanced qmesh;
  std::vector<int> elements1,elements2; 
  std::vector<Vector3> points1,points2;
};


} //namespace Geometry

#endif
