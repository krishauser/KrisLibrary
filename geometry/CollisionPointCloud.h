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
 *
 * If the point cloud has radii, the octree nodes are expanded to contain
 * the points grown by their radii.
 */
class CollisionPointCloud : public Meshing::PointCloud3D
{
 public:
  CollisionPointCloud();
  explicit CollisionPointCloud(const Meshing::PointCloud3D& pc,int hints=0);
  CollisionPointCloud(const CollisionPointCloud& pc);
  ///Sets up the collision detection data structures.  This is automatically
  ///called during initialization, and needs to be called any time the point
  ///cloud changes
  void InitCollisions(int hints=0);

  ///The local bounding box of the point cloud
  AABB3D bblocal;
  ///The transformation of the point cloud in space 
  RigidTransform currentTransform;
  Real gridResolution; ///< default value is 0, which auto-determines from point cloud
  GridSubdivision3D grid;
  shared_ptr<OctreePointSet> octree;

  int radiusIndex;   ///< the index of point radii, or -1 if not defined
  Real maxRadius;    ///< the maximum over all point radii
};

///Returns the oriented bounding box of the point cloud
void GetBB(const CollisionPointCloud& pc,Box3D& b);

///Computes whether any point in the pc is within tol distance of the
///primitive g. O(min(n,c)) running time, where c is the number of grid
///cells within distance tol of the bounding box of g.
bool WithinDistance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol);
///Returns the nearest distance from any point in pc to g.  O(log n) running time.
Real Distance(const CollisionPointCloud& pc,const Vector3& pt);
///Returns the nearest distance from any point in pc to g.  O(log n) running time.
///Saves the closest point index into ClosestPoint.
///If upperBound is given, and if no point is closer than upperBound,
///this may return upperBound as the return value and closestPoint=-1.
Real Distance(const CollisionPointCloud& pc,const Vector3& pt,int& closestPoint,Real upperBound);
///Returns the nearest distance from any point in pc to g.  O(log n) running time
///for points and spheres, O(n) running time for everythign else.
Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g);
///Returns the nearest distance from any point in pc to g.  O(log n) running time
///for points and spheres, O(n) running time for everything else.  Saves the closest
///point index into closestPoint.
///If upperBound is given, and if no point is closer than upperBound,
///this may return upperBound as the return value and closestPoint=-1.
Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,int& closestPoint,Real upperBound=Inf);
///Returns the nearest distance from any point in pc1 to any point in pc2.  O(n log n) running time. 
///Saves the closest point pair into closestPoint1 and closestPoint2.
///If upperBound is given, then if no pair of points is closer than upperBound, this may return
///upperBound as the return distance and closestPoint1=closestPoint2=-1.
Real Distance(const CollisionPointCloud& pc1,const CollisionPointCloud& pc2,int& closestPoint1,int& closestPoint2,Real upperBound=Inf);
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
