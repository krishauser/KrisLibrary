#ifndef GEOMETRY_COLLISION_CONVEXHULL3D_H
#define GEOMETRY_COLLISION_CONVEXHULL3D_H

#include "ConvexHull3D.h"
#include "GeometryType.h"
#include "GeometryTypeImpl.h"


namespace Geometry {

/** @ingroup Geometry
 * @brief The collision data class for ConvexHull3D that contains the solid3 data
 * structure moved by some rigid transform.
 *
 * This also lets you update the relative transform for a Hull ConvexHull3D datatype.
 * 
 * Author: Gao Tang
 */
class Collider3DConvexHull : public Collider3D
{
public:
  Collider3DConvexHull(const ConvexHull3D& hull);
  Collider3DConvexHull(shared_ptr<Geometry3DConvexHull> data);  
  virtual ~Collider3DConvexHull() {}
  virtual shared_ptr<Geometry3D> GetData() const { return dynamic_pointer_cast<Geometry3D>(data); }
  virtual void Reset() override;
  virtual AABB3D GetAABB() const override;
  virtual RigidTransform GetTransform() const override { return T; }
  virtual void SetTransform(const RigidTransform& T) override;
  virtual bool Merge(Collider3D* cgeom) override;
  virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
  virtual bool Contains(const Vector3& pt,bool& result) override;
  virtual bool Distance(const Vector3& pt,Real& result) override;
  virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
  virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
  virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
  virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res)  override { return false; }
  virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) override;
  virtual bool Support(const Vector3& dir,Vector3& pt) override;
  
  bool Collides(const GeometricPrimitive3D& primitive,bool& result);
  bool Collides(const Collider3DConvexHull& geometry, Vector3* common_point=nullptr) const;
  Real ClosestPoint(const Vector3& pt,Vector3& cp,Vector3& direction) const;
  bool ClosestPoint(const GeometricPrimitive3D& primitive,Real& dist,Vector3& cp,Vector3& direction) const;
  Real ClosestPoint(const Collider3DConvexHull& g, Vector3& cp, Vector3& direction) const;

  ///For Hull objects, updates the relative transform of the second object
  void UpdateHullSecondRelativeTransform(const RigidTransform& tran);

  shared_ptr<Geometry3DConvexHull> data;
  ConvexHull3D::Type type;
  struct ObjectHandleContainer {
    ObjectHandleContainer(DT_ObjectHandle data);
    ~ObjectHandleContainer();
    DT_ObjectHandle data;
  };
  std::shared_ptr<ObjectHandleContainer> objectHandle;
  RigidTransform T;
};

} //namespace Geometry

#endif // GEOMETRY_COLLISION_CONVEXHULL3D_H