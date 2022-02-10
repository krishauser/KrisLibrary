#ifndef COLLISION_IMPLICIT_SURFACE_H
#define COLLISION_IMPLICIT_SURFACE_H

#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include "AnyGeometryTypeImpl.h"

namespace Geometry {

  class CollisionPointCloud;

  using namespace Math3D;

/** @brief An implicit surface (usually signed-distance function) with a fast collision
 * detection data structure.
 */
class CollisionImplicitSurface
{
 public:
  CollisionImplicitSurface();
  explicit CollisionImplicitSurface(const Meshing::VolumeGrid& vg);
  CollisionImplicitSurface(const CollisionImplicitSurface& vg);
  ///Sets up the collision detection data structures.  This is automatically
  ///called during initialization, and needs to be called any time the implicit
  ///surface changes
  void InitCollisions();

  ///O(1) call to get a range of minimum and maximum implicit surface values within a bounding box,
  ///expressed in local frame
  void DistanceRangeLocal(const AABB3D& bb,Real& vmin,Real& vmax) const;

  ///The original implicit surface
  Meshing::VolumeGrid baseGrid;
  ///The transformation of the implicit surface in space 
  RigidTransform currentTransform;
  ///A hierarchy of volume grids of decreasing resolution
  std::vector<Meshing::VolumeGrid> minHierarchy,maxHierarchy;
  std::vector<Real> resolutionMap;
};


class Collider3DImplicitSurface : public Collider3D
{
public:
    Collider3DImplicitSurface(shared_ptr<Geometry3DImplicitSurface> data);
    virtual ~Collider3DImplicitSurface() {}
    virtual shared_ptr<Geometry3D> GetData() const { return dynamic_pointer_cast<Geometry3D>(data); }
    virtual void Reset();
    virtual RigidTransform GetTransform() const { return collisionData.currentTransform; }
    virtual bool ConvertFrom(Collider3D* geom,Real param=0,Real domainExpansion=0);
    virtual void SetTransform(const RigidTransform& T) { collisionData.currentTransform = T; }
    virtual bool Contains(const Vector3& pt,bool& result);
    virtual bool Distance(const Vector3& pt,Real& result);
    virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res);
    virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res);
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
    virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res);
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element);
    virtual Collider3D* Slice(const RigidTransform& T,Real tol=0) const;
    virtual Collider3D* ExtractROI(const AABB3D& bb,int flag=1) const;
    virtual Collider3D* ExtractROI(const Box3D& bb,int flag=1) const;

    shared_ptr<Geometry3DImplicitSurface> data;
    CollisionImplicitSurface collisionData;
};


///Returns the distance between s and pt, assuming it's a signed distance field.  Negative values
///indicate interior points.  Input is in world coordinates.
Real Distance(const CollisionImplicitSurface& s,const Vector3& pt);

///Returns true if the primitive is within margin distance of the geometry represented by s.
///If so, a colliding point (in world coordinates) is returned in pt.
bool Collides(const CollisionImplicitSurface& s,const GeometricPrimitive3D& geom,Real margin,Vector3& pt);

///Returns the distance between s and pt, assuming s is a signed distance field.  Negative values
///indicate interior points.  
///
///Outputs:
///- surfacePt is the closest point on the surface
///- direction is the unit normal in the direction of decreasing signed distance.  If pt is outside, 
///  this points toward surfacePt, but if pt is inside, this points further into s.
///
///Inputs and outputs are all in world coordinates.
Real Distance(const CollisionImplicitSurface& s,const Vector3& pt,Vector3& surfacePt,Vector3& direction);

///Same as above, except that:
///- geomPt is the closest/deepest point on geom.
///- direction is the unit normal of decreasing distance, in that if geom is moved in this direction, the distance decreases.
///
///Only points and spheres are currently supported
Real Distance(const CollisionImplicitSurface& s,const GeometricPrimitive3D& geom,Vector3& surfacePt,Vector3& geomPt,Vector3& direction);

///Returns true if the point cloud is within margin distance of the geometry represented by s. One or more colliding 
///points can also be returned in collidingPoints.
bool Collides(const CollisionImplicitSurface& s,const CollisionPointCloud& pc,Real margin,std::vector<int>& collidingPoints,size_t maxContacts=1);

///Returns the distance and closest point to a CollisionPointCloud
Real Distance(const CollisionImplicitSurface& s,const CollisionPointCloud& pc,int& closestPoint,Real upperBound=Inf);

///Returns the distance to the closest point on the implicit surface defined at the given level set.
///tmax will be returned if no collision is found.
///
///The algorithm marches along cells intersected by the ray until a zero-crossing is met (O(n))
///where n is the resolution of the grid.
Real RayCast(const Meshing::VolumeGrid& grid,const Ray3D& ray,Real levelSet=0,Real tmax=Inf);

///Returns the distance to the closest point on the implicit surface defined at the given level set.
///tmax will be returned if no collision is found.
///
///The algorithm uses the collision hierarchy (O(log n) where n is the resolution of the grid.
Real RayCast(const CollisionImplicitSurface& s,const Ray3D& ray,Real levelSet=0,Real tmax=Inf);

} //namespace Geometry

#endif
