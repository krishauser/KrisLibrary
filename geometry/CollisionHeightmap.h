#ifndef GEOMETRY_HEIGHTMAP_H
#define GEOMETRY_HEIGHTMAP_H

#include "GeometryType.h"
#include "GeometryTypeImpl.h"

namespace Geometry {

using namespace std;

class Collider3DHeightmap : public Collider3D
{
public:
    Collider3DHeightmap(shared_ptr<Geometry3DHeightmap> data);
    virtual ~Collider3DHeightmap() {}
    virtual shared_ptr<Geometry3D> GetData() const override { return dynamic_pointer_cast<Geometry3D>(data); }
    virtual void Reset() override;
    virtual RigidTransform GetTransform() const override { return currentTransform; }
    virtual Box3D GetBB() const override;
    virtual void SetTransform(const RigidTransform& T) override { currentTransform = T; }
    virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
    virtual bool Contains(const Vector3& pt,bool& result) override;
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) override;

    /// Returns index of grid cell closest to the projection of ptworld 
    int PointToElement(const Vector3& ptworld) const;

    shared_ptr<Geometry3DHeightmap> data;
    RigidTransform currentTransform;
    Real hmin,hmax;
};

}//namespace Geometry

#endif // GEOMETRY_HEIGHTMAP_H