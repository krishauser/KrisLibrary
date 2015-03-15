#include "CollisionPointCloud.h"
#include <Timer.h>

namespace Geometry {

CollisionPointCloud::CollisionPointCloud()
  :gridResolution(0),grid(3)
{
  currentTransform.setIdentity();
}

CollisionPointCloud::CollisionPointCloud(const Meshing::PointCloud3D& _pc)
  :Meshing::PointCloud3D(_pc),gridResolution(0),grid(3)
{
  currentTransform.setIdentity();
  InitCollisions();
}

void CollisionPointCloud::InitCollisions()
{
  Timer timer;
  bblocal.minimize();
  for(size_t i=0;i<points.size();i++)
    bblocal.expand(points[i]);
  //set up the grid
  Real res = gridResolution;
  if(gridResolution <= 0) {
    Vector3 dims = bblocal.bmax-bblocal.bmin;
    Real maxdim = Max(dims.x,dims.y,dims.z);
    Real mindim = Min(dims.x,dims.y,dims.z);
    //default grid size: assume points are evenly distributed on a 2D manifold
    //in space, try to get 50 points per grid cell
    Real vol = dims.x*dims.y*dims.z;
    //h^2 * n = vol
    int ptsPerCell = 50;
    Real h = Pow(vol / points.size() * ptsPerCell, 1.0/2.0);
    if(h > mindim) { 
      //TODO: handle relatively flat point clouds
    }
    res = h;
  }
  grid.h.set(res);
  for(size_t i=0;i<points.size();i++) {
    Vector p(3,points[i]);
    GridSubdivision::Index ind;
    grid.PointToIndex(p,ind);
    grid.Insert(ind,&points[i]);
  }
  printf("CollisionPointCloud::InitCollisions: %d points, res %g, time %gs\n",points.size(),res,timer.ElapsedTime());
  //print stats
  int nmax = 0;
  for(GridSubdivision::HashTable::const_iterator i=grid.buckets.begin();i!=grid.buckets.end();i++)
    nmax = Max(nmax,(int)i->second.size());
  printf("  %d nonempty buckets, max size %d, avg %g\n",grid.buckets.size(),nmax,Real(points.size())/grid.buckets.size());
}

void GetBB(const CollisionPointCloud& pc,Box3D& b)
{
  b.setTransformed(pc.bblocal,pc.currentTransform);
}

static Real gWithinDistanceTestThreshold = 0;
static GeometricPrimitive3D* gWithinDistanceTestObject = NULL;
bool withinDistanceTest(void* obj)
{
  Point3D* p = reinterpret_cast<Point3D*>(obj);
  if(gWithinDistanceTestObject->Distance(*p) <= gWithinDistanceTestThreshold)
    return false;
  return true;
}

bool WithinDistance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol)
{
  Box3D bb;
  GetBB(pc,bb);
  //quick reject test
  if(g.Distance(bb) > tol) return false;

  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);

  AABB3D gbb = glocal.GetAABB();  
  gbb.setIntersection(pc.bblocal);
  GridSubdivision::Index imin,imax;
  pc.grid.PointToIndex(Vector(3,gbb.bmin),imin);
  pc.grid.PointToIndex(Vector(3,gbb.bmax),imax);
  int numCells = (imax[0]-imin[0]+1)*(imax[1]-imin[1]+1)*(imax[2]-imin[2]+1);
  if(numCells > (int)pc.points.size()) {
    //test all points, linearly
    for(size_t i=0;i<pc.points.size();i++)
      if(glocal.Distance(pc.points[i]) <= tol) return true;
    return false;
  }
  else {
    gWithinDistanceTestThreshold = tol;
    gWithinDistanceTestObject = &glocal;
    bool collisionFree = pc.grid.IndexQuery(imin,imax,withinDistanceTest);
    return !collisionFree;
  }
}

static Real gDistanceTestValue = 0;
static GeometricPrimitive3D* gDistanceTestObject = NULL;
bool distanceTest(void* obj)
{
  Point3D* p = reinterpret_cast<Point3D*>(obj);
  gDistanceTestValue = Min(gDistanceTestValue,gDistanceTestObject->Distance(*p));
  return true;
}

Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g)
{
  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);

  /*
  AABB3D gbb = glocal.GetAABB();
  gDistanceTestValue = Inf;
  gDistanceTestObject = &glocal;
  pc.grid.BoxQuery(Vector(3,gbb.bmin),Vector(3,gbb.bmax),distanceTest);
  return gDistanceTestValue;
  */
  //test all points, linearly
  Real dmax = Inf;
  for(size_t i=0;i<pc.points.size();i++)
    dmax = Min(dmax,glocal.Distance(pc.points[i]));
  return dmax;
}

static Real gNearbyTestThreshold = 0;
static GeometricPrimitive3D* gNearbyTestObject = NULL;
static std::vector<Point3D*> gNearbyTestResults;
static size_t gNearbyTestBranch = 0;
bool nearbyTest(void* obj)
{
  Point3D* p = reinterpret_cast<Point3D*>(obj);
  if(gNearbyTestObject->Distance(*p) <= gNearbyTestThreshold)
    gNearbyTestResults.push_back(p);
  if(gNearbyTestResults.size() >= gNearbyTestBranch) return false;
  return true;
}

void NearbyPoints(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol,std::vector<int>& points,size_t maxContacts)
{
  Box3D bb;
  GetBB(pc,bb);
  //quick reject test
  if(g.Distance(bb) > tol) return;

  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);

  AABB3D gbb = glocal.GetAABB();
  gbb.setIntersection(pc.bblocal);
  GridSubdivision::Index imin,imax;
  pc.grid.PointToIndex(Vector(3,gbb.bmin),imin);
  pc.grid.PointToIndex(Vector(3,gbb.bmax),imax);
  int numCells = (imax[0]-imin[0]+1)*(imax[1]-imin[1]+1)*(imax[2]-imin[2]+1);
  if(numCells > (int)pc.points.size()) {
    printf("Testing all points\n");
    //test all points, linearly
    for(size_t i=0;i<pc.points.size();i++)
      if(glocal.Distance(pc.points[i]) <= tol) {
	points.push_back(int(i));
	if(points.size()>=maxContacts) return;
      }
  }
  else {
    printf("Testing points in BoxQuery\n");
    gNearbyTestThreshold = tol;
    gNearbyTestResults.resize(0);
    gNearbyTestObject = &glocal;
    gNearbyTestBranch = maxContacts;
    pc.grid.BoxQuery(Vector(3,gbb.bmin),Vector(3,gbb.bmax),nearbyTest);
    points.resize(gNearbyTestResults.size());
    for(size_t i=0;i<points.size();i++)
      points[i] = gNearbyTestResults[i] - &pc.points[0];
  }
}

} //namespace Geometry
