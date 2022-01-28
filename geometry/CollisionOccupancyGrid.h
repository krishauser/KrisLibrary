#ifndef GEOMETRY_OCCUPANCY_GRID_H
#define GEOMETRY_OCCUPANCY_GRID_H

#include "AnyGeometryType.h"
#include "AnyGeometryTypeImpl.h"

namespace Geometry {

using namespace std;

class Collider3DOccupancyGrid : public Collider3D
{
public:
    Collider3DOccupancyGrid(shared_ptr<Geometry3DOccupancyGrid> data);
    virtual ~Collider3DOccupancyGrid() {}
    virtual shared_ptr<Geometry3D> GetData() const { return dynamic_pointer_cast<Geometry3D>(data); }
    virtual void Reset();
    virtual AABB3D GetAABB() const;
    virtual Box3D GetBB() const;
    virtual RigidTransform GetTransform() const { return currentTransform; }
    virtual Collider3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0);
    virtual void SetTransform(const RigidTransform& T) { currentTransform = T; }
    virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
    virtual bool Contains(const Vector3& pt,bool& result);
    virtual bool Distance(const Vector3& pt,Real& result);
    virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res);
    virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res);
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
    virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) { return false; }
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element);
    virtual Collider3D* Slice(const RigidTransform& T,Real tol=0) const;
    virtual Collider3D* ExtractROI(const AABB3D& bb,int flag=1) const;
    virtual Collider3D* ExtractROI(const Box3D& bb,int flag=1) const;

    shared_ptr<Geometry3DOccupancyGrid> data;
    RigidTransform currentTransform;
};

}//namespace Geometry

#endif // GEOMETRY_OCCUPANCY_GRID_H