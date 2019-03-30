#ifndef COLLISION_POINT_CLOUD_H
#define COLLISION_POINT_CLOUD_H

#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <memory>
#include <limits>
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
  explicit CollisionPointCloud(const Meshing::PointCloud3D& pc);
  CollisionPointCloud(const CollisionPointCloud& pc);
  ///Sets up the collision detection data structures.  This is automatically
  ///called during initialization, and needs to be called any time the point
  ///cloud changes
  void InitCollisions();

  ///The local bounding box of the point cloud
  AABB3D bblocal;
  ///The transformation of the point cloud in space 
  RigidTransform currentTransform;
  Real gridResolution; ///< default value is 0, which auto-determines from point cloud
  GridSubdivision3D grid;
  shared_ptr<OctreePointSet> octree;
};

///Returns the orientd bounding box of the point cloud
void GetBB(const CollisionPointCloud& pc,Box3D& b);

///Computes whether any point in the pc is within tol distance of the
///primitive g. O(min(n,c)) running time, where c is the number of grid
///cells within distance tol of the bounding box of g.
bool WithinDistance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol);
///Returns the nearest distance from any point in pc to g.  O(n) running time.
Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g);
///Returns the nearest distance from any point in pc to g.  O(n) running time.  Saves the closest
///point index into closestPoint, and if upperBound is given, then if no point is closer than upperBound,
///this may return upperBound as the return value and closestPoint=-1.
Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,int& closestPoint,Real upperBound=Inf);
///Computes the set of points in the pc that are within tol distance of the
///primitive g.  O(min(n,c)) running time, where c is the number of grid
///cells within distance tol of the bounding box of g.
void NearbyPoints(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol,std::vector<int>& points,size_t maxContacts=std::numeric_limits<size_t>::max());

///Returns true if the point clouds are within margin distance of one another.
///points1 and points2 return one or more pairs of close-by points.
///maxContacts controls how many pairs are computed.
bool Collides(const CollisionPointCloud& pc1,const CollisionPointCloud& pc2,Real margin,std::vector<int>& points1,std::vector<int>& points2,size_t maxContacts=1);

///Casts a ray at the point cloud (where each point is fattened by radius rad).
///Returns index of the first point hit (-1 if none) and the colliding point
///in pt. 
int RayCast(const CollisionPointCloud& pc,Real rad,const Ray3D& r,Vector3& pt);

///Same as RayCast, but the ray (and colliding point) are in the local
///frame of the point cloud
int RayCastLocal(const CollisionPointCloud& pc,Real rad,const Ray3D& r,Vector3& pt);

} //namespace Geometry

#endif
