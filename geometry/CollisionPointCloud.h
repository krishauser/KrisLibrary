#ifndef COLLISION_POINT_CLOUD_H
#define COLLISION_POINT_CLOUD_H

#include <meshing/PointCloud.h>
#include <math3d/geometry3d.h>
#include <utils/SmartPointer.h>
#include <limits.h>
#include "GridSubdivision.h"
#include "Octree.h"

namespace Geometry {

  using namespace Math3D;

/** @brief A point cloud with a fast collision detection data structure
 */
class CollisionPointCloud : public Meshing::PointCloud3D
{
 public:
  CollisionPointCloud();
  CollisionPointCloud(const Meshing::PointCloud3D& pc);
  ///Needs to be called if this point cloud was loaded or set up any other
  ///way than the constructor
  void InitCollisions();

  ///The local bounding box of the point cloud
  AABB3D bblocal;
  ///The transformation of the point cloud in space 
  RigidTransform currentTransform;
  Real gridResolution; ///< default value is 0, which auto-determines from point cloud
  GridSubdivision grid;
  SmartPointer<OctreePointSet> octree;
};

///Returns the orientd bounding box of the point cloud
void GetBB(const CollisionPointCloud& pc,Box3D& b);

///Computes whether any point in the pc is within tol distance of the
///primitive g. O(min(n,c)) running time, where c is the number of grid
///cells within distance tol of the bounding box of g.
bool WithinDistance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol);
///Returns the nearest distance from any point in pc to g.  O(n) running time.
Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g);
///Computes the set of points in the pc that are within tol distance of the
///primitive g.  O(min(n,c)) running time, where c is the number of grid
///cells within distance tol of the bounding box of g.
void NearbyPoints(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol,std::vector<int>& points,size_t maxContacts=INT_MAX);

///Casts a ray at the point cloud (where each point is fattened by radius rad).
///Returns index of the first point hit (-1 if none) and the colliding point
///in pt. 
int RayCast(const CollisionPointCloud& pc,Real rad,const Ray3D& r,Vector3& pt);

///Same as RayCast, but the ray (and colliding point) are in the local
///frame of the point cloud
int RayCastLocal(const CollisionPointCloud& pc,Real rad,const Ray3D& r,Vector3& pt);

} //namespace Geometry

#endif
