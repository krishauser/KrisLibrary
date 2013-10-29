#ifndef COLLISION_POINT_CLOUD_H
#define COLLISION_POINT_CLOUD_H

#include <meshing/PointCloud.h>
#include <math3d/geometry3d.h>
#include <limits.h>

namespace Geometry {

  using namespace Math3D;

class CollisionPointCloud : public Meshing::PointCloud3D
{
 public:
  CollisionPointCloud();
  CollisionPointCloud(const Meshing::PointCloud3D& pc);
  ///Needs to be called if this point cloud was loaded or set up any other
  ///way than the constructor
  void InitCollisions();

  //TODO: collision accelerators
  AABB3D bblocal;
  RigidTransform currentTransform;
};

void GetBB(const CollisionPointCloud& pc,Box3D& b);
bool WithinDistance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol);
Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g);
void NearbyPoints(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol,std::vector<int>& points,size_t maxContacts=INT_MAX);

} //namespace Geometry

#endif
