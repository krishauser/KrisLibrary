#ifndef ANY_GEOMETRY_H
#define ANY_GEOMETRY_H

#include <KrisLibrary/utils/AnyValue.h>
#include "CollisionMesh.h"
#include "ConvexHull3D.h"

class TiXmlElement;

//forward declarations
namespace Meshing { template <class T> class VolumeGridTemplate; typedef VolumeGridTemplate<Real> VolumeGrid; class PointCloud3D; }
namespace Geometry { class CollisionPointCloud; class CollisionImplicitSurface; class ConvexHull3D; class CollisionConvexHull3D; }
namespace Math3D { class GeometricPrimitive3D; }
namespace GLDraw { class GeometryAppearance; }

namespace Geometry {

  using namespace Math3D;
  using namespace std;

class AnyDistanceQuerySettings;
class AnyDistanceQueryResult;
class AnyContactsQuerySettings;
class AnyContactsQueryResult;

//foward declaration
#ifndef GEOMETRY_ROI_H
enum {
    ExtractROIFlagIntersection=0x01,
    ExtractROIFlagTouching=0x02,
    ExtractROIFlagWithin=0x04,
    ExtractROIFlagInvert=0x08
};
#endif //GEOMETRY_ROI_H

/** @brief A class that stores any kind of geometry we've defined.
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
  /** 
   * Map of types to classes in the value member
   * - Primitive: GeometricPrimitive3D
   * - TriangleMesh: TriMesh
   * - PointCloud: PointCloud3D
   * - ImplicitSurface: VolumeGrid
   * - Group: vector<AnyGeometry3D>
   */
  enum Type { Primitive, TriangleMesh, PointCloud, ImplicitSurface, ConvexHull, Group };

  AnyGeometry3D();
  AnyGeometry3D(const GeometricPrimitive3D& primitive);
  AnyGeometry3D(const Meshing::VolumeGrid& grid);
  AnyGeometry3D(const Meshing::TriMesh& mesh);
  AnyGeometry3D(const Meshing::PointCloud3D& pc);
  AnyGeometry3D(const vector<AnyGeometry3D>& items);
  AnyGeometry3D(const ConvexHull3D& cvxhull);
  AnyGeometry3D(const AnyGeometry3D& geom) = default;
  AnyGeometry3D(AnyGeometry3D&& geom) = default;
  AnyGeometry3D& operator = (const AnyGeometry3D& rhs) = default;
  AnyGeometry3D& operator = (AnyGeometry3D&& rhs) = default;
  static const char* TypeName(Type type);
  const char* TypeName() const { return AnyGeometry3D::TypeName(type); }
  const GeometricPrimitive3D& AsPrimitive() const;
  const Meshing::TriMesh& AsTriangleMesh() const;
  const Meshing::PointCloud3D& AsPointCloud() const;
  const Meshing::VolumeGrid& AsImplicitSurface() const;
  const ConvexHull3D& AsConvexHull() const;
  const vector<AnyGeometry3D>& AsGroup() const;
  GeometricPrimitive3D& AsPrimitive();
  Meshing::TriMesh& AsTriangleMesh();
  Meshing::PointCloud3D& AsPointCloud();
  Meshing::VolumeGrid& AsImplicitSurface();
  ConvexHull3D& AsConvexHull();
  vector<AnyGeometry3D>& AsGroup();
  GLDraw::GeometryAppearance* TriangleMeshAppearanceData();
  const GLDraw::GeometryAppearance* TriangleMeshAppearanceData() const;
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
  GeometricPrimitive3D GetElement(int elem) const;
  AABB3D GetAABB() const;
  void Transform(const RigidTransform& T);
  void Transform(const Matrix4& mat);
  void Merge(const vector<AnyGeometry3D>& geoms);
  ///Converts to another geometry type, storing the result in res and returning true
  ///if the conversion is available. 
  ///
  ///If restype==type, returns a copy of this.
  ///
  ///param is interpreted as follows, with 0 being a "reasonable" default value:
  ///- Primitive -> TriangleMesh: desired resolution of mesh
  ///- Primitive -> PointCloud: desired resolution of point cloud
  ///- TriangleMesh -> PointCloud: desired resolution of point cloud
  ///- TriangleMesh -> ImplicitSurface: desired width of volume grid cells
  ///- ImplicitSurface -> TriangleMesh: level set to be extracted
  ///- ImplicitSurface -> PointCloud: level set to be extracted
  bool Convert(Type restype,AnyGeometry3D& res,Real param=0) const;
  ///Re-meshes the geometry at the desired resolution, storing the result into res.
  ///
  ///Resolution is interpreted as
  ///- Primitive, ConvexHull: ignored
  ///- TriangleMesh: desired length of triangle edges.  Refinement only.
  ///- PointCloud: desire 1 point per grid cell of width resolution.  Coarsen only.
  ///- ImplicitSurface: new surface cell width.
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
  ///The data, according to the type
  AnyValue data;
  ///Optional appearance data, according to the type
  AnyValue appearanceData;
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
  AnyCollisionGeometry3D(const Meshing::TriMesh& mesh);
  AnyCollisionGeometry3D(const Meshing::PointCloud3D& pc);
  AnyCollisionGeometry3D(const Meshing::VolumeGrid& grid);
  AnyCollisionGeometry3D(const AnyGeometry3D& geom);
  AnyCollisionGeometry3D(const ConvexHull3D& primitive);
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
  bool CollisionDataInitialized() const { return !collisionData.empty(); }
  ///Clears the current collision data
  void ClearCollisionData() { collisionData = AnyValue(); }
  const RigidTransform& PrimitiveCollisionData() const;
  const CollisionMesh& TriangleMeshCollisionData() const;
  const CollisionPointCloud& PointCloudCollisionData() const;
  const CollisionImplicitSurface& ImplicitSurfaceCollisionData() const;
  const CollisionConvexHull3D& ConvexHullCollisionData() const;
  const vector<AnyCollisionGeometry3D>& GroupCollisionData() const;
  RigidTransform& PrimitiveCollisionData();
  CollisionMesh& TriangleMeshCollisionData();
  CollisionPointCloud& PointCloudCollisionData();
  CollisionImplicitSurface& ImplicitSurfaceCollisionData();
  CollisionConvexHull3D& ConvexHullCollisionData();
  vector<AnyCollisionGeometry3D>& GroupCollisionData();
  ///Performs a type conversion, also copying the active transform and collision margin.
  ///May be a bit faster than AnyGeometry3D.Convert for some conversions
  ///(TriangleMesh->VolumeGrid, specifically) and will respect collision margins
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
  ///Groups together these geometries, as usual
  void Merge(const vector<AnyGeometry3D>& geoms);
  ///Groups together these geometries.  This geometry is placed such that
  ///the active transform is the same as the active transform for the
  ///first geometry, and all subsequent geometries are transformed relative
  ///to the first
  void Merge(const vector<AnyCollisionGeometry3D>& geoms);
  //Computes the furthest point on the geometry in the direction dir
  //TODO: is this useful to implement outside of ConvexHull types?
  // Vector3 FindSupport(const Vector3& dir);
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

  /** The collision data structure, according to the type.
   * - Primitive: null
   * - TriangleMesh: CollisionMesh
   * - PointCloud: CollisionPointCloud
   * - VolumeGrid: CollisionImplicitSurface
   * - Group: vector<AnyCollisionGeometry3D>
   */
  AnyValue collisionData;
  ///Amount by which the underlying geometry is "fattened"
  Real margin;
  ///The current transform, used if the collision data is not initialized yet
  ///or the data type is Primitive / VolumeGrid.
  RigidTransform currentTransform;
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

class AnyDistanceQuerySettings
{
public:
  AnyDistanceQuerySettings();
  ///Allowable relative and absolute errors
  Real relErr,absErr;
  ///An upper bound on the distance, and if the two objects are farther than this distance the computation may break
  Real upperBound;
};

class AnyDistanceQueryResult
{
public:
  AnyDistanceQueryResult();
  ///flags indicating which elements are filled out
  bool hasPenetration,hasElements,hasClosestPoints,hasDirections;
  ///The distance, with negative values indicating penetration if hasPenetration=true. Otherwise, 0 indicates penetration.
  Real d;
  ///The elements defining the closest points on the geometries
  int elem1,elem2;
  ///The closest points on the two geometries, in world coordinates
  Vector3 cp1,cp2;
  ///The direction from geometry 1 to geometry 2, and the distance from geometry 2 to geometry 1, in world coordinates
  ///These are typically proportional to cp2-cp1 and cp1-cp2, respectively, EXCEPT when the points are exactly
  ///coincident.
  Vector3 dir1,dir2;
  ///If the item is a group, this vector will recursively define the sub-elements
  vector<int> group_elem1,group_elem2;
};

class AnyContactsQuerySettings
{
public:
  AnyContactsQuerySettings();
  ///Extra padding on the geometries, padding1 for this object and padding2 for the other other
  Real padding1,padding2;
  ///Maximum number of contacts queried
  size_t maxcontacts;
  ///True if you'd like to cluster the contacts into at most maxcontacts results
  bool cluster;
};

class AnyContactsQueryResult
{
public:
  struct ContactPair
  {
    ///the depth of the contact, padding included
    Real depth;
    ///the contact points on the padded geometries of object1 and object2, in world coordinates
    Vector3 p1,p2;
    ///the outward contact normal from object 1 pointing into object 2, in world coordinates
    Vector3 n;
    ///the item defining the element to which this point belongs
    int elem1,elem2;
    ///if true, the contact normal can't be estimated accurately
    bool unreliable;
  };

  AnyContactsQueryResult();
  AnyContactsQueryResult(AnyContactsQueryResult&& other) = default;
  ///The list of computed contact points
  vector<ContactPair> contacts;
  ///True if clustering was performed
  bool clustered;
};

} //namespace Geometry

#endif
