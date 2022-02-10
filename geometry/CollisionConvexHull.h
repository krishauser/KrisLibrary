#ifndef GEOMETRY_COLLISION_CONVEXHULL3D_H
#define GEOMETRY_COLLISION_CONVEXHULL3D_H

#include "ConvexHull3D.h"
#include "AnyGeometryType.h"
#include "AnyGeometryTypeImpl.h"


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
  virtual void Reset();
  virtual AABB3D GetAABB() const;
  virtual RigidTransform GetTransform() const { return T; }
  virtual void SetTransform(const RigidTransform& T);
  virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
  virtual bool Contains(const Vector3& pt,bool& result);
  virtual bool Distance(const Vector3& pt,Real& result);
  virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res);
  virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res);
  virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX);
  virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) { return false; }
  virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element);
  virtual bool Support(const Vector3& dir,Vector3& pt) const;
  
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
  DT_ShapeHandle shapeHandle;
  RigidTransform T;
};

} //namespace Geometry

#endif // GEOMETRY_COLLISION_CONVEXHULL3D_H