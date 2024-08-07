#ifndef GEOMETRY_COLLISION_PRIMITIVE_H
#define GEOMETRY_COLLISION_PRIMITIVE_H

#include "GeometryType.h"
#include "GeometryTypeImpl.h"

namespace Geometry {

class Collider3DPrimitive : public Collider3D
{
public:
    Collider3DPrimitive(shared_ptr<Geometry3DPrimitive> data);
    virtual ~Collider3DPrimitive() {}
    virtual shared_ptr<Geometry3D> GetData() const override { return dynamic_pointer_cast<Geometry3D>(data); }
    virtual AABB3D GetAABB() const override;
    virtual Box3D GetBB() const override;
    virtual RigidTransform GetTransform() const override { return T; }
    virtual void SetTransform(const RigidTransform& T) override { this->T = T; }
    virtual bool Contains(const Vector3& pt,bool& result) override;
    virtual bool Distance(const Vector3& pt,Real& result) override;
    virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
    virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
    virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) override;
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) override;
    
    shared_ptr<Geometry3DPrimitive> data;
    RigidTransform T;
};

} //namespace Geometry

#endif //GEOMETRY_COLLISION_PRIMITIVE_H