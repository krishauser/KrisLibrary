#include "CollisionPointCloud.h"

namespace Geometry {

CollisionPointCloud::CollisionPointCloud()
{
  currentTransform.setIdentity();
}

CollisionPointCloud::CollisionPointCloud(const Meshing::PointCloud3D& _pc)
  :Meshing::PointCloud3D(_pc)
{
  currentTransform.setIdentity();
  InitCollisions();
}

void CollisionPointCloud::InitCollisions()
{
  bblocal.minimize();
  for(size_t i=0;i<points.size();i++)
    bblocal.expand(points[i]);
}

void GetBB(const CollisionPointCloud& pc,Box3D& b)
{
  b.setTransformed(pc.bblocal,pc.currentTransform);
}

bool WithinDistance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol)
{
  Box3D bb;
  GetBB(pc,bb);
  //quick reject test
  if(g.Distance(bb) > tol) return false;

  //test all points, linearly
  for(size_t i=0;i<pc.points.size();i++)
    if(g.Distance(pc.points[i]) <= tol) return true;
  return false;
}

Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g)
{
  Real dmax = Inf;
  //test all points, linearly
  for(size_t i=0;i<pc.points.size();i++)
    dmax = Min(dmax,g.Distance(pc.points[i]));
  return dmax;
}

void NearbyPoints(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol,std::vector<int>& points,size_t maxContacts)
{
  Box3D bb;
  GetBB(pc,bb);
  //quick reject test
  if(g.Distance(bb) > tol) return;

  //test all points, linearly
  for(size_t i=0;i<pc.points.size();i++)
    if(g.Distance(pc.points[i]) <= tol) {
      points.push_back(int(i));
      if(points.size()>=maxContacts) return;
    }
}

} //namespace Geometry
