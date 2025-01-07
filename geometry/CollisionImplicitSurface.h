#ifndef COLLISION_IMPLICIT_SURFACE_H
#define COLLISION_IMPLICIT_SURFACE_H

#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include "GeometryTypeImpl.h"

namespace Geometry {

    class CollisionPointCloud;
    class CollisionMesh;

    using namespace Math3D;

/// @brief A collider for a signed distance field.
///
/// The collision data includes a hierarchy of distance fields at
/// coarser resolutions.  This allows for faster collision checking.
class Collider3DImplicitSurface : public Collider3D
{
public:
    Collider3DImplicitSurface(shared_ptr<Geometry3DImplicitSurface> data);
    Collider3DImplicitSurface(const Collider3DImplicitSurface& data);
    virtual ~Collider3DImplicitSurface() {}
    virtual shared_ptr<Geometry3D> GetData() const override { return dynamic_pointer_cast<Geometry3D>(data); }
    virtual void Reset();
    virtual RigidTransform GetTransform() const override { return currentTransform; }
    virtual Collider3D* Copy(shared_ptr<Geometry3D>) const override;
    virtual bool ConvertFrom(Collider3D* geom,Real param=0,Real domainExpansion=0) override;
    virtual bool Merge(Collider3D* geom) override;

    virtual void SetTransform(const RigidTransform& T) override { currentTransform = T; }
    virtual bool Contains(const Vector3& pt,bool& result) override;
    virtual bool Distance(const Vector3& pt,Real& result) override;
    virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
    virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
    virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) override;
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) override;

    /// Returns the closest element to pt (given in world coordinates) in the domain 
    int PointToElement(const Vector3& pt) const;
    ///Returns true if the primitive is within margin distance of the geometry represented by s.
    ///If so, a colliding point (in world coordinates) is returned in pt.
    ///
    ///Only works for points, spheres, segments, and triangles.  Sets pt to
    ///Inf if the type is not supported.
    bool Collides(const GeometricPrimitive3D& geom,Real margin,Vector3& pt) const;
    bool CollidesLocal(const Triangle3D& tri_local,Real margin,IntTriple& collides_cell) const;

    /// Standard point distance. O(1)
    Real Distance(const Vector3& pt) const;
    Real DistanceLocal(const Vector3& pt_local) const;
    ///Returns the distance between s and pt, assuming s is a signed distance field.  Negative values
    ///indicate interior points.  
    ///
    ///Outputs:
    ///- surfacePt is the closest point on the surface.  This may be incorrect near the
    ///  medial axis.
    ///- direction is the unit normal in the direction of decreasing signed distance.  If pt is outside, 
    ///  this points toward surfacePt, but if pt is inside, this points further into s.
    ///
    ///Inputs and outputs are all in world coordinates.
    Real Distance(const Vector3& pt,Vector3& surfacePt,Vector3& direction) const;

    ///Same as above, except that:
    ///- triPt is the closest/deepest point on tri.
    ///- direction is the unit normal of decreasing distance, in that if tri is moved in this direction, the distance decreases.
    ///- upperBound: if the distance to tri is > upperBound, the function will return upperBound quickly.
    Real Distance(const Triangle3D& tri,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real upperBound=Inf) const;
    ///Same as Distance, but all inputs are in local coordinates.
    Real DistanceLocal(const Triangle3D& tri_local,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance) const;
    
    ///Same as Distance, generalized to arbitrary primitives. 
    ///
    ///Here:
    ///- geomPt is the closest/deepest point on geom.
    ///
    ///Only points, segments, spheres, and triangles are currently supported
    Real Distance(const GeometricPrimitive3D& geom,Vector3& surfacePt,Vector3& geomPt,Vector3& direction,Real upperBound=Inf) const;

    ///Returns true if the point cloud is within margin distance of the implicit surface.
    ///One or more colliding points can also be returned in collidingPoints.
    bool Collides(const CollisionPointCloud& pc,Real margin,std::vector<int>& collidingPoints,size_t maxContacts=1) const;

    ///Returns the signed distance and closest point to a CollisionPointCloud
    Real Distance(const CollisionPointCloud& pc,int& closestPoint,Real upperBound=Inf) const;

    ///Returns true if the mesh is within margin distance of the implicit surface.
    ///One or more colliding points can also be returned in collidingPoints.
    bool Collides(const CollisionMesh &tm, Real margin,std::vector<int> &elements1, std::vector<int> &elements2, size_t maxContacts) const;

    ///Returns the signed distance and closest point to a CollisionPointCloud
    Real Distance(const CollisionMesh& tm,Vector3 closestPtGrid, int& closestTri, Vector3& closestPtMesh,Real upperBound=Inf) const;

    ///Returns the distance to the closest point on the implicit surface defined at the given level set.
    ///tmax will be returned if no collision is found.
    ///
    ///The algorithm uses the collision hierarchy (O(log n) where n is the resolution of the grid.
    Real RayCast(const Ray3D& ray,Real levelSet=0,Real tmax=Inf) const;

    ///O(1) call to get a range of minimum and maximum implicit surface values within a bounding box,
    ///expressed in local frame
    void DistanceRangeLocal(const AABB3D& bb,Real& vmin,Real& vmax) const;

    ///The implicit surface data
    shared_ptr<Geometry3DImplicitSurface> data;

    ///The transform of the implicit surface in space 
    RigidTransform currentTransform;

    ///A hierarchy of volume grids of decreasing resolution
    std::vector<Meshing::VolumeGrid> minHierarchy,maxHierarchy;
    std::vector<Real> resolutionMap;
};

///Returns the distance between the implicit surface and the given point.
Real DistanceSDF(const Meshing::VolumeGrid& grid,const Vector3& pt,Vector3* surfacePt=NULL,Vector3* direction=NULL);

///Returns the distance between the implicit surface and the given triangle.
Real DistanceSDF(const Meshing::VolumeGrid& sdf,const Triangle3D& tri,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance);

///Returns the distance to the closest point on the implicit surface defined at the given level set.
///tmax will be returned if no collision is found.
///
///The algorithm marches along cells intersected by the ray until a zero-crossing is met (O(n))
///where n is the resolution of the grid.
Real RayCast(const Meshing::VolumeGrid& grid,const Ray3D& ray,Real levelSet=0,Real tmax=Inf);


} //namespace Geometry

#endif
