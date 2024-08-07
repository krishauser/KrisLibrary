#ifndef GEOMETRY_OCCUPANCY_GRID_H
#define GEOMETRY_OCCUPANCY_GRID_H

#include "GeometryType.h"
#include "GeometryTypeImpl.h"

namespace Geometry {

using namespace std;

class Collider3DOccupancyGrid : public Collider3D
{
public:
    Collider3DOccupancyGrid(shared_ptr<Geometry3DOccupancyGrid> data);
    virtual ~Collider3DOccupancyGrid() {}
    virtual shared_ptr<Geometry3D> GetData() const { return dynamic_pointer_cast<Geometry3D>(data); }
    virtual void Reset();
    virtual RigidTransform GetTransform() const { return currentTransform; }
    virtual void SetTransform(const RigidTransform& T) { currentTransform = T; }
    virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
    virtual bool Contains(const Vector3& pt,bool& result);
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element);

    shared_ptr<Geometry3DOccupancyGrid> data;
    RigidTransform currentTransform;
};

}//namespace Geometry

#endif // GEOMETRY_OCCUPANCY_GRID_H