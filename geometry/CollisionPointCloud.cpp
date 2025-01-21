#include <KrisLibrary/Logger.h>
#include "CollisionPointCloud.h"
#include "CollisionPrimitive.h"
#include "CollisionMesh.h"
#include "CollisionConvexHull.h"
#include "Conversions.h"
#include "ROI.h"
#include "Slice.h"
#include <Timer.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/structs/Heap.h>

#include "PQP/include/PQP.h"
#include "PQP/src/MatVec.h"
#include "PQP/src/OBB_Disjoint.h"
#include "PQP/src/TriDist.h"

DECLARE_LOGGER(Geometry)


namespace Geometry {

typedef ContactsQueryResult::ContactPair ContactPair;

//declared in CollisionMesh
void RigidTransformToPQP(const RigidTransform &f, PQP_REAL R[3][3], PQP_REAL T[3]);


CollisionPointCloud::CollisionPointCloud()
  :gridResolution(0),grid(3),radiusIndex(-1),maxRadius(0)
{
  currentTransform.setIdentity();
  bblocal.minimize();
}

CollisionPointCloud::CollisionPointCloud(const Meshing::PointCloud3D& _pc,int hints)
  :Meshing::PointCloud3D(_pc),gridResolution(0),grid(3),radiusIndex(-1),maxRadius(0)
{
  currentTransform.setIdentity();
  InitCollisions(hints);
}

CollisionPointCloud::CollisionPointCloud(const CollisionPointCloud& _pc)
  :Meshing::PointCloud3D(_pc),bblocal(_pc.bblocal),currentTransform(_pc.currentTransform),
   gridResolution(_pc.gridResolution),grid(_pc.grid),
   octree(_pc.octree),radiusIndex(_pc.radiusIndex),maxRadius(_pc.maxRadius)
{

}

void CollisionPointCloud::InitCollisions(int hints)
{
  radiusIndex = PropertyIndex("radius");
  if(radiusIndex >= 0) {
    Real minRadius = 0.0;
    maxRadius = 0.0;
    for(int i=0;i<properties.m;i++) {
      maxRadius = Max(maxRadius,properties(i,radiusIndex));
      minRadius = Min(minRadius,properties(i,radiusIndex));
    }
    if(minRadius < 0)
      FatalError("Can't create a collision point cloud with negative radius?");
  }

  bblocal.minimize();
  grid.buckets.clear();
  octree = NULL;
  if(points.empty()) 
    return;
  Assert(points.size() > 0);
  Timer timer;
  if(radiusIndex >= 0) {
    for(size_t i=0;i<points.size();i++) {
      bblocal.expand(points[i]-Vector3(properties(i,radiusIndex)));
      bblocal.expand(points[i]+Vector3(properties(i,radiusIndex)));
    }
  }
  else {
    for(size_t i=0;i<points.size();i++)
      bblocal.expand(points[i]);
  }

  //set up the grid
  Real res = gridResolution;
  if(gridResolution <= 0) {
    Vector3 dims = bblocal.bmax-bblocal.bmin;
    //Real maxdim = Max(dims.x,dims.y,dims.z);
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
  grid.hinv.set(1.0/res);
  int validptcount = 0;
  for(size_t i=0;i<points.size();i++) {
    if(IsFinite(points[i].x)) {
      GridSubdivision3D::Index ind;
      grid.PointToIndex(points[i],ind);
      grid.Insert(ind,&points[i]);
      validptcount++;
    }
  }
  LOG4CXX_INFO(GET_LOGGER(Geometry),"CollisionPointCloud::InitCollisions: "<<validptcount<<" valid points, res "<<res<<", time "<<timer.ElapsedTime());
  //print stats
  int nmax = 0;
  for(GridSubdivision3D::HashTable::const_iterator i=grid.buckets.begin();i!=grid.buckets.end();i++)
    nmax = Max(nmax,(int)i->second.size());
  LOG4CXX_INFO(GET_LOGGER(Geometry),"  "<<grid.buckets.size()<<" nonempty grid buckets, max size "<<nmax<<", avg "<<Real(points.size())/grid.buckets.size());
  timer.Reset();

  //initialize the octree, 10 points per cell, res is minimum cell size
  octree = make_shared<OctreePointSet>(bblocal,10,res);
  if(radiusIndex >= 0)  {
    for(size_t i=0;i<points.size();i++) {
      if(IsFinite(points[i].x))
        octree->AddSphere(points[i],properties(i,radiusIndex),(int)i);
    }
  }
  else {
    for(size_t i=0;i<points.size();i++) {
      if(IsFinite(points[i].x))
        octree->Add(points[i],(int)i);
    }
  }
  LOG4CXX_INFO(GET_LOGGER(Geometry),"  octree initialized in time "<<timer.ElapsedTime()<<"s, "<<octree->Size()<<" nodes, depth "<<octree->MaxDepth());
  //TEST: should we fit to points?
  if(!(hints & CollisionDataHintFast)) {
    octree->FitToPoints();
    LOG4CXX_INFO(GET_LOGGER(Geometry),"  octree fit to points in time "<<timer.ElapsedTime());
  }
  /*
  //TEST: method 2.  Turns out to be much slower
  timer.Reset();
  octree = new OctreePointSet(bblocal,points.size());
  octree->SplitToResolution(res);
  for(size_t i=0;i<points.size();i++)
    octree->Add(points[i],(int)i);
  octree->Collapse(10);
  LOG4CXX_INFO(GET_LOGGER(Geometry),"  octree 2 initialized in time "<<timer.ElapsedTime()<<"s, "<<octree->Size());
  */
}

void GetBB(const CollisionPointCloud& pc,Box3D& b)
{
  b.setTransformed(pc.bblocal,pc.currentTransform);
  if(!IsFinite(pc.bblocal.bmin.x))
    printf("CollisionPointCloud doesn't contain didn't return a finite AABB\n");
}

static Real gWithinDistanceTestThreshold = 0;
static GeometricPrimitive3D* gWithinDistanceTestObject = NULL;
bool withinDistanceTest(void* obj)
{
  Assert(gWithinDistanceTestObject != NULL);
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

  vector<Vector3> points;
  vector<int> ids;
  if(pc.maxRadius > 0) {
    //points have radii defined
    Assert(pc.radiusIndex >= 0);
    if(glocal.type == GeometricPrimitive3D::Point) {
      const Vector3& x = *AnyCast<Point3D>(&glocal.data);
      pc.octree->BallQuery(x,tol,points,ids);
      if(!points.empty()) return true;
    }
    else if(glocal.type == GeometricPrimitive3D::Sphere) {
      const Sphere3D& s = *AnyCast<Sphere3D>(&glocal.data);
      pc.octree->BallQuery(s.center,s.radius+tol,points,ids);
      if(!points.empty()) return true;
    }
    else {
      AABB3D gbb = glocal.GetAABB();  
      gbb.setIntersection(pc.bblocal);
      pc.octree->BoxQuery(gbb.bmin-Vector3(tol),gbb.bmax+Vector3(tol),points,ids);
      for(size_t i=0;i<points.size();i++) 
        if(glocal.Distance(points[i]) <= tol + pc.properties(ids[i],pc.radiusIndex)) return true;
    }
    return false;
  }

  //octree overlap method
  if(glocal.type == GeometricPrimitive3D::Point) {
    const Vector3& x = *AnyCast<Point3D>(&glocal.data);
    pc.octree->BallQuery(x,tol,points,ids);
    if(!points.empty()) return true;
  }
  else if(glocal.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D& s = *AnyCast<Sphere3D>(&glocal.data);
    pc.octree->BallQuery(s.center,s.radius+tol,points,ids);
    if(!points.empty()) return true;
  }
  else {
    AABB3D gbb = glocal.GetAABB();  
    gbb.setIntersection(pc.bblocal);
    pc.octree->BoxQuery(gbb.bmin-Vector3(tol),gbb.bmax+Vector3(tol),points,ids);
    for(size_t i=0;i<points.size();i++) 
      if(glocal.Distance(points[i]) <= tol) return true;
  }
  return false;

  /*
  //grid enumeration method
  GridSubdivision::Index imin,imax;
  pc.grid.PointToIndex(gbb.bmin,imin);
  pc.grid.PointToIndex(gbb.bmax,imax);
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
  */
}

static Real gDistanceTestValue = 0;
static GeometricPrimitive3D* gDistanceTestObject = NULL;
bool distanceTest(void* obj)
{
  Point3D* p = reinterpret_cast<Point3D*>(obj);
  gDistanceTestValue = Min(gDistanceTestValue,gDistanceTestObject->Distance(*p));
  return true;
}

Real Distance(const CollisionPointCloud& pc,const Vector3& pt)
{
  int closestPoint;
  return Distance(pc,pt,closestPoint);
}

Real Distance(const CollisionPointCloud& pc,const Vector3& pt,int& closestPoint,Real upperBound)
{
  Vector3 ptlocal;
  pc.currentTransform.mulInverse(pt,ptlocal);
  Vector3 closest;
  bool res = pc.octree->NearestNeighbor(ptlocal,closest,closestPoint,upperBound);
  if(!res) {
    closestPoint = -1;
    return upperBound;
  }
  return closest.distance(ptlocal);
}

Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g)
{
  int cpoint;
  return Distance(pc,g,cpoint);
}

Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,int& closestPoint,Real upperBound)
{
  if(g.type == GeometricPrimitive3D::Point) {
    const Vector3& x = *AnyCast<Point3D>(&g.data);
    return Distance(pc,x,closestPoint,upperBound);
  }
  else if(g.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D& s = *AnyCast<Sphere3D>(&g.data);
    return Distance(pc,s.center,closestPoint,upperBound+s.radius)-s.radius; 
  }
  //do point-pointcloud distance
  Vector3 c = g.GetCentroid();
  Real dbound = Distance(pc,c,closestPoint,upperBound);
  if(closestPoint < 0) return upperBound;
  //use this bound to get a sphere that must contain the closest point on the point cloud 
  Vector3 dir;
  Real d = g.ClosestPoints(pc.currentTransform*pc.points[closestPoint],c,dir);
  if(d == 0) return d;
  //now query all points within the radius s
  vector<int> candidatePoints;
  NearbyPoints(pc,GeometricPrimitive3D(c),d,candidatePoints);

  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);

  Real dmax = d;
  bool doBoundsTest= (g.type != GeometricPrimitive3D::AABB);
  if(doBoundsTest) {
    Sphere3D sbb = glocal.GetBoundingSphere();
    Real radius0 = sbb.radius;
    sbb.radius += upperBound;
    //AABB3D bb0 = gbb;
    //gbb.bmin -= Vector3(upperBound);
    //gbb.bmax += Vector3(upperBound);
    //test all points, linearly
    for(auto i:candidatePoints) {
      if(sbb.contains(pc.points[i])) {
        Real d = glocal.Distance(pc.points[i]);
        if(pc.radiusIndex >= 0)
          d -= pc.properties(i,pc.radiusIndex);
        if(d < dmax) {
          closestPoint = (int)i;
          dmax = d;
          sbb.radius = radius0 + dmax;
        }
      }
    }
  }
  else {
    //bounds testing not worth it
    for(auto i:candidatePoints) {
      Real d = glocal.Distance(pc.points[i]);
      if(pc.radiusIndex >= 0)
        d -= pc.properties(i,pc.radiusIndex);
      if(d < dmax) {
        closestPoint = (int)i;
        dmax = d;
      }
    }
  }
  return dmax;
}

Real Distance(const CollisionPointCloud& pc1,const CollisionPointCloud& pc2,int& closestPoint1,int& closestPoint2,Real upperBound)
{
  if(pc1.points.size() > pc2.points.size())
    return Distance(pc2,pc1,closestPoint2,closestPoint1,upperBound);
  closestPoint1 = closestPoint2 = -1;
  RigidTransform T12;
  T12.mulInverseA(pc2.currentTransform,pc1.currentTransform);
  //go linearly through the smaller point cloud
  for(size_t i=0;i<pc1.points.size();i++) {
    Vector3 x = T12*pc1.points[i];
    int cp2x;
    Vector3 xclosest;
    bool res = pc2.octree->NearestNeighbor(x,xclosest,cp2x,upperBound);
    if(res) {
      Real d = x.distance(xclosest);
      if(pc1.radiusIndex >= 0) d -= pc1.properties(i,pc1.radiusIndex);
      if(pc2.radiusIndex >= 0) {
        d -= pc2.properties(cp2x,pc2.radiusIndex);
      }
      if(d < upperBound) {
        upperBound = d;
        closestPoint1 = (int)i;
        closestPoint2 = cp2x;
      }
    }
  }
  return upperBound;
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

void NearbyPoints(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,Real tol,std::vector<int>& pointIds,size_t maxContacts)
{
  Box3D bb;
  GetBB(pc,bb);
  //quick reject test
  if(tol == 0) {
    if(!g.Collides(bb)) {
      return;
    }
  }
  else if(g.SupportsDistance(GeometricPrimitive3D::Box)) {
    if(g.Distance(bb) > tol) {
      return;
    }
  }
  else {
    bb.expand(tol);
    if(!g.Collides(bb)) {
      return;
    }
  }

  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);

  //octree overlap method
  vector<Vector3> points;
  vector<int> ids;

  if(pc.maxRadius > 0) {
    //spheres, expand the search radius
    if(glocal.type == GeometricPrimitive3D::Point) {
      const Vector3& x = *AnyCast<Point3D>(&glocal.data);
      pc.octree->BallQuery(x,tol,points,ids);
      for(size_t i=0;i<points.size();i++) {
        pointIds.push_back(ids[i]);
        if(pointIds.size()>=maxContacts) return;
      }
    }
    else if(glocal.type == GeometricPrimitive3D::Sphere) {
      const Sphere3D& s = *AnyCast<Sphere3D>(&glocal.data);
      pc.octree->BallQuery(s.center,s.radius+tol,points,ids);
      for(size_t i=0;i<points.size();i++) {
        pointIds.push_back(ids[i]);
        if(pointIds.size()>=maxContacts) return;
      }
    }
    else {
      AABB3D gbb = glocal.GetAABB();
      gbb.setIntersection(pc.bblocal);
      pc.octree->BoxQuery(gbb.bmin-Vector3(tol),gbb.bmax+Vector3(tol),points,ids);
      for(size_t i=0;i<points.size();i++) {
        if(glocal.Distance(points[i]) <= tol + pc.properties(ids[i],pc.radiusIndex)) {
          pointIds.push_back(ids[i]);
          if(pointIds.size()>=maxContacts) return;
        }
      }
    }
    return;
  }

  if(glocal.type == GeometricPrimitive3D::Point) {
    const Vector3& x = *AnyCast<Point3D>(&glocal.data);
    pc.octree->BallQuery(x,tol,points,ids);
    for(size_t i=0;i<points.size();i++) {
      pointIds.push_back(ids[i]);
      if(pointIds.size()>=maxContacts) return;
    }
  }
  else if(glocal.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D& s = *AnyCast<Sphere3D>(&glocal.data);
    pc.octree->BallQuery(s.center,s.radius+tol,points,ids);
    for(size_t i=0;i<points.size();i++) {
      pointIds.push_back(ids[i]);
      if(pointIds.size()>=maxContacts) return;
    }
  }
  else {
    AABB3D gbb = glocal.GetAABB();
    gbb.setIntersection(pc.bblocal);
    pc.octree->BoxQuery(gbb.bmin-Vector3(tol),gbb.bmax+Vector3(tol),points,ids);
    for(size_t i=0;i<points.size();i++) {
      if(glocal.Distance(points[i]) <= tol) {
        pointIds.push_back(ids[i]);
        if(pointIds.size()>=maxContacts) return;
      }
    }
  }
  return;

  /*
  //grid enumeration method
  GridSubdivision3D::Index imin,imax;
  pc.grid.PointToIndex(gbb.bmin,imin);
  pc.grid.PointToIndex(gbb.bmax,imax);
  int numCells = (imax[0]-imin[0]+1)*(imax[1]-imin[1]+1)*(imax[2]-imin[2]+1);
  if(numCells > (int)pc.points.size()) {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"Testing all points\n");
    //test all points, linearly
    for(size_t i=0;i<pc.points.size();i++)
      if(glocal.Distance(pc.points[i]) <= tol) {
	pointIds.push_back(int(i));
	if(pointIds.size()>=maxContacts) return;
      }
  }
  else {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"Testing points in BoxQuery\n");
    gNearbyTestThreshold = tol;
    gNearbyTestResults.resize(0);
    gNearbyTestObject = &glocal;
    gNearbyTestBranch = maxContacts;
    pc.grid.BoxQuery(gbb.bmin,gbb.bmax,nearbyTest);
    pointIds.resize(gNearbyTestResults.size());
    for(size_t i=0;i<points.size();i++)
      pointIds[i] = gNearbyTestResults[i] - &pc.points[0];
  }
  */
}

int RayCast(const CollisionPointCloud& pc,Real rad,const Ray3D& r,Vector3& pt)
{
  Ray3D rlocal;
  pc.currentTransform.mulInverse(r.source,rlocal.source);
  pc.currentTransform.R.mulTranspose(r.direction,rlocal.direction);
  int res = RayCastLocal(pc,rad,rlocal,pt);
  if(res >= 0) {
    pt = pc.currentTransform*pt;
  }
  return res;
}

int RayCastLocal(const CollisionPointCloud& pc,Real rad,const Ray3D& r,Vector3& pt)
{
  Real tmin=0,tmax=Inf;
  if(!((const Line3D&)r).intersects(pc.bblocal,tmin,tmax)) return -1;

  int value = pc.octree->RayCast(r,rad);
  if(value >= 0)
    pt = pc.points[value];
  return value;

  /*
  Real closest = Inf;
  int closestpt = -1;
  Sphere3D s;
  s.radius = margin;
  if(margin < 1e-3) {
    //grid rasterization method
    Segment3D slocal;
    r.eval(tmin,slocal.a);
    r.eval(tmax,slocal.b);
    //normalize to grid resolution
    slocal.a.x /= pc.grid.h[0];
    slocal.a.y /= pc.grid.h[1];
    slocal.a.z /= pc.grid.h[2];
    slocal.b.x /= pc.grid.h[0];
    slocal.b.y /= pc.grid.h[1];
    slocal.b.z /= pc.grid.h[2];
    vector<IntTriple> cells;
    Meshing::GetSegmentCells(slocal,cells);
    GridSubdivision::Index ind;
    ind.resize(3);
    for(size_t i=0;i<cells.size();i++) {
      ind[0] = cells[i].a;
      ind[1] = cells[i].b;
      ind[2] = cells[i].c;
      //const list<void*>* objs = pc.grid.GetObjectSet(ind);
      //if(!objs) continue;
      GridSubdivision::HashTable::const_iterator objs = pc.grid.buckets.find(ind);
      if(objs != pc.grid.buckets.end()) {
	for(list<void*>::const_iterator k=objs->second.begin();k!=objs->second.end();k++) {
	  Vector3* p = reinterpret_cast<Vector3*>(*k);
	  s.center = *p;
	  Real tmin,tmax;
	  if(s.intersects(r,&tmin,&tmax)) {
	    if(tmax >= 0 && tmin < closest) {
	      closest = Max(tmin,0.0);
	      closestpt = (p - &pc.points[0]);
	    }
	  }
	}
      }
    }
  }
  else {
    //brute force method
    for(size_t i=0;i<pc.points.size();i++) {
      s.center = pc.points[i];
      Real tmin,tmax;
      if(s.intersects(r,&tmin,&tmax)) {
	if(tmax >= 0 && tmin < closest) {
	  closest = Max(tmin,0.0);
	  closestpt = i;
	}
      }
    }
  }
  pt = r.eval(closest);
  return closestpt;
  */
}

inline Real Volume(const AABB3D& bb)
{
  Vector3 d = bb.bmax-bb.bmin;
  return d.x*d.y*d.z;
}

inline Real Volume(const OctreeNode& n)
{
  return Volume(n.bb);
}

class PointPointCollider
{
public:
  const CollisionPointCloud& a;
  const CollisionPointCloud& b;
  RigidTransform Tba,Twa,Tab;
  Real margin;
  size_t maxContacts;
  vector<int> acollisions,bcollisions;
  PointPointCollider(const CollisionPointCloud& _a,const CollisionPointCloud& _b,Real _margin)
    :a(_a),b(_b),margin(_margin),maxContacts(1)
  {
    Twa.setInverse(a.currentTransform);
    Tba.mul(Twa,b.currentTransform);
    Tab.setInverse(Tba);
    if(_a.maxRadius > 0 || _b.maxRadius > 0)
      FatalError("Unable to do point-cloud collisions when the point clouds have point-specific radii");
  }
  bool Recurse(size_t _maxContacts=1)
  {
    maxContacts=_maxContacts;
    _Recurse(0,0);
    return !acollisions.empty();
  }
  bool Prune(const OctreeNode& anode,const OctreeNode& bnode) {
    Box3D meshbox_pc;
    meshbox_pc.setTransformed(bnode.bb,Tba);
    if(margin==0)
      return !meshbox_pc.intersects(anode.bb);
    else {
      AABB3D expanded_bb = anode.bb;
      expanded_bb.bmin -= Vector3(margin);
      expanded_bb.bmax += Vector3(margin);
      return !meshbox_pc.intersects(expanded_bb);
    }
  }
  bool _Recurse(int aindex,int bindex) {
    const OctreeNode& anode = a.octree->Node(aindex);
    const OctreeNode& bnode = b.octree->Node(bindex);
    //returns true to keep recursing
    if(Prune(anode,bnode))
      return true;
    if(a.octree->IsLeaf(anode)) {
      if(a.octree->NumPoints(anode)==0) return true;
      if(b.octree->IsLeaf(bnode)) {
        if(b.octree->NumPoints(bnode)==0) return true;
        //collide the two points contained within
        vector<Vector3> apts,bpts;
        vector<int> aids,bids;
        a.octree->GetPointIDs(aindex,aids);
        b.octree->GetPointIDs(bindex,bids);
        a.octree->GetPoints(aindex,apts);
        b.octree->GetPoints(bindex,bpts);
        for(auto& bpt: bpts)
          bpt = Tba*bpt;
        //pick only the closest point amongst these guys, not all points within the margin
        bool doswap = false;
        if(apts.size() > bpts.size()) {
          swap(apts,bpts);
          swap(aids,bids);
          doswap= true;
        }
        for(size_t i=0;i<apts.size();i++) {
          Real dmin2 = Sqr(margin);
          int bclosest = -1;
          for(size_t j=0;j<bpts.size();j++) {
            Real d2 = apts[i].distanceSquared(bpts[j]);
            if(d2 <= dmin2) {
              dmin2 = d2;
              bclosest = bids[j];
            }
          }
          if(bclosest >= 0) {
            if(doswap) {
              bcollisions.push_back(aids[i]);
              acollisions.push_back(bclosest);
            }
            else {
              acollisions.push_back(aids[i]);
              bcollisions.push_back(bclosest);
            }
            if(acollisions.size() >= maxContacts) return false;
          }
        }
        //continue
        return true;
      }
      else {
        //split b
        return _RecurseSplitB(aindex,bindex);
      }
    }
    else {
      if(b.octree->IsLeaf(bnode)) {
        if(b.octree->NumPoints(bnode)==0) return true;
        //split a
        return _RecurseSplitA(aindex,bindex);
      }
      else {
        //determine which BVH to split
        Real va = Volume(anode);
        Real vb = Volume(bnode);
        if(va < vb)
          return _RecurseSplitB(aindex,bindex);
        else
          return _RecurseSplitA(aindex,bindex);
      }
    }
  }
  bool _RecurseSplitA(int aindex,int bindex) {
    const OctreeNode& anode = a.octree->Node(aindex);
    for(int i=0;i<8;i++)
      if(!_Recurse(anode.childIndices[i],bindex)) return false;
    return true;
  }
  bool _RecurseSplitB(int aindex,int bindex) {
    const OctreeNode& bnode = b.octree->Node(bindex);
    for(int i=0;i<8;i++)
      if(!_Recurse(aindex,bnode.childIndices[i])) return false;
    return true;
  }
};


bool Collides(const CollisionPointCloud& a,const CollisionPointCloud& b,Real margin,std::vector<int>& apoints,std::vector<int>& bpoints,size_t maxContacts)
{
  PointPointCollider collider(a,b,margin);
  bool res=collider.Recurse(maxContacts);
  if(res) {
    apoints=collider.acollisions;
    bpoints=collider.bcollisions;
    return true;
  }
  return false;
}




bool Collides(const GeometricPrimitive3D &a, const CollisionPointCloud &b, Real margin, vector<int> &pcelements, size_t maxContacts)
{
  NearbyPoints(b, a, margin, pcelements, maxContacts);
  return !pcelements.empty();
}

/*
static Real gWithinDistanceMargin = 0;
static const CollisionPointCloud* gWithinDistancePC = NULL;
static AnyCollisionGeometry3D* gWithinDistanceGeom = NULL;
static vector<int>* gWithinDistanceElements1 = NULL;
static vector<int>* gWithinDistanceElements2 = NULL;
static size_t gWithinDistanceMaxContacts = 0;
static GeometricPrimitive3D point_primitive(Vector3(0.0));
bool withinDistance_PC_AnyGeom(void* obj)
{
  Point3D* p = reinterpret_cast<Point3D*>(obj);
  Vector3 pw = gWithinDistancePC->currentTransform*(*p);
  RigidTransform Tident; Tident.R.setIdentity(); Tident.t = pw;
  vector<int> temp;
  if(Collides(point_primitive,Tident,gWithinDistanceMargin,*gWithinDistanceGeom,temp,*gWithinDistanceElements1,gWithinDistanceMaxContacts)) {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"Colliding point "<<pw.x<<" "<<pw.y<<" "<<pw.z<<" distance "<<gWithinDistanceGeom->Distance(pw));
    gWithinDistanceElements2->push_back(p-&gWithinDistancePC->points[0]);
    if(gWithinDistanceElements1->size() >= gWithinDistanceMaxContacts) 
      return false;
  }
  return true;
}
*/

inline void Copy(const PQP_REAL p[3], Vector3 &x)
{
  x.set(p[0], p[1], p[2]);
}

inline void Copy(const Vector3 &x, PQP_REAL p[3])
{
  p[0] = x.x;
  p[1] = x.y;
  p[2] = x.z;
}

inline void BVToBox(const BV &b, Box3D &box)
{
  Copy(b.d, box.dims);
  Copy(b.To, box.origin);
  //box.xbasis.set(b.R[0][0],b.R[0][1],b.R[0][2]);
  //box.ybasis.set(b.R[1][0],b.R[1][1],b.R[1][2]);
  //box.zbasis.set(b.R[2][0],b.R[2][1],b.R[2][2]);
  box.xbasis.set(b.R[0][0], b.R[1][0], b.R[2][0]);
  box.ybasis.set(b.R[0][1], b.R[1][1], b.R[2][1]);
  box.zbasis.set(b.R[0][2], b.R[1][2], b.R[2][2]);

  //move the box to have origin at the corner
  box.origin -= box.dims.x * box.xbasis;
  box.origin -= box.dims.y * box.ybasis;
  box.origin -= box.dims.z * box.zbasis;
  box.dims *= 2;
}

inline void BoxToBV(const Box3D &box, BV &b)
{
  Copy(box.dims * 0.5, b.d);
  Copy(box.center(), b.To);
  box.xbasis.get(b.R[0][0], b.R[1][0], b.R[2][0]);
  box.ybasis.get(b.R[0][1], b.R[1][1], b.R[2][1]);
  box.zbasis.get(b.R[0][2], b.R[1][2], b.R[2][2]);
}

inline bool Collide(const Triangle3D &tri, const Sphere3D &s)
{
  Vector3 pt = tri.closestPoint(s.center);
  return s.contains(pt);
}

inline Real Volume(const BV &b)
{
  return 8.0 * b.d[0] * b.d[1] * b.d[2];
}

class PointMeshCollider
{
public:
  const CollisionPointCloud &pc;
  const CollisionMesh &mesh;
  RigidTransform Tba, Twa, Tab;
  Real margin;
  size_t maxContacts;
  vector<int> pcpoints, meshtris;
  vector<Real> pairdistances2;
  PointMeshCollider(const CollisionPointCloud &a, const CollisionMesh &b, Real _margin)
      : pc(a), mesh(b), margin(_margin), maxContacts(1)
  {
    Twa.setInverse(a.currentTransform);
    Tba.mul(Twa, b.currentTransform);
    Tab.setInverse(Tba);
  }
  bool Recurse(size_t _maxContacts = 1)
  {
    maxContacts = _maxContacts;
    _Recurse(0, 0);
    return !pcpoints.empty();
  }
  bool Prune(const OctreeNode &pcnode, const BV &meshnode)
  {
    Box3D meshbox, meshbox_pc;
    BVToBox(meshnode, meshbox);
    meshbox_pc.setTransformed(meshbox, Tba);
    if (margin == 0)
      return !meshbox_pc.intersects(pcnode.bb);
    else
    {
      AABB3D expanded_bb = pcnode.bb;
      expanded_bb.bmin -= Vector3(margin);
      expanded_bb.bmax += Vector3(margin);
      return !meshbox_pc.intersects(expanded_bb);
    }
  }
  bool _Recurse(int pcOctreeNode, int meshBVHNode)
  {
    const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
    const BV &meshnode = mesh.pqpModel->b[meshBVHNode];
    //returns true to keep recursing
    if (Prune(pcnode, meshnode)) {
      return true;
    }
    if (pc.octree->IsLeaf(pcnode))
    {
      if (pc.octree->NumPoints(pcnode) == 0)
        return true;
      if (meshnode.Leaf())
      {
        int t = -meshnode.first_child - 1;
        Triangle3D tri;
        Copy(mesh.pqpModel->tris[t].p1, tri.a);
        Copy(mesh.pqpModel->tris[t].p2, tri.b);
        Copy(mesh.pqpModel->tris[t].p3, tri.c);
        tri.a = Tba * tri.a;
        tri.b = Tba * tri.b;
        tri.c = Tba * tri.c;
        //collide the triangle and points
        vector<Vector3> pts;
        vector<int> pcids;
        pc.octree->GetPoints(pcOctreeNode, pts);
        pc.octree->GetPointIDs(pcOctreeNode, pcids);
        Real dmin2 = Sqr(margin);
        int closestpt = -1;
        for (size_t i = 0; i < pts.size(); i++)
        {
          Vector3 pt = tri.closestPoint(pts[i]);
          Real d2 = pts[i].distanceSquared(pt);
          if (d2 < dmin2)
          {
            //only use the closest point in a leaf?
            #if !POINT_CLOUD_MESH_COLLIDE_ALL_POINTS
              dmin2 = d2;
              closestpt = pcids[i];
            #else 
              //Use all points in a batch
              pcpoints.push_back(pcids[i]);
              meshtris.push_back(mesh.pqpModel->tris[t].id);
              pairdistances2.push_back(d2);
              if(pcpoints.size() >= maxContacts)
                return false;
            #endif // !POINT_CLOUD_MESH_COLLIDE_ALL_POINTS
          }
        }
        //only use the closest point in a leaf?
        #if !POINT_CLOUD_MESH_COLLIDE_ALL_POINTS
          if (closestpt >= 0)
          {
            pcpoints.push_back(closestpt);
            meshtris.push_back(mesh.pqpModel->tris[t].id);
            pairdistances2.push_back(dmin2);
            if(pcpoints.size() >= maxContacts)
                return false; //stop
          }
        #endif // !POINT_CLOUD_MESH_COLLIDE_ALL_POINTS
        //continue
        return true;
      }
      else
      {
        //split mesh BVH
        return _RecurseSplitMesh(pcOctreeNode, meshBVHNode);
      }
    }
    else
    {
      if (meshnode.Leaf())
      {
        //split octree node
        return _RecurseSplitOctree(pcOctreeNode, meshBVHNode);
      }
      else
      {
        //determine which BVH to split
        Real vpc = Volume(pcnode);
        Real vmesh = Volume(meshnode);
        if (vpc < vmesh)
          return _RecurseSplitMesh(pcOctreeNode, meshBVHNode);
        else
          return _RecurseSplitOctree(pcOctreeNode, meshBVHNode);
      }
    }
  }
  bool _RecurseSplitMesh(int pcOctreeNode, int meshBVHNode)
  {
    int c1 = mesh.pqpModel->b[meshBVHNode].first_child;
    int c2 = c1 + 1;
    if (!_Recurse(pcOctreeNode, c1))
      return false;
    if (!_Recurse(pcOctreeNode, c2))
      return false;
    return true;
  }
  bool _RecurseSplitOctree(int pcOctreeNode, int meshBVHNode)
  {
    const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
    for (int i = 0; i < 8; i++)
      if (!_Recurse(pcnode.childIndices[i], meshBVHNode))
        return false;
    return true;
  }
};

bool Collides(const CollisionPointCloud &a, Real margin, const CollisionMesh &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  PointMeshCollider collider(a, b, margin);
  bool res = collider.Recurse(maxContacts);
  if (res)
  {
    //do we want to filter by closest pairs?
    map<int, int> closest1, closest2;
    map<int, Real> dclosest1, dclosest2;
    for (size_t i = 0; i < collider.pcpoints.size(); i++)
    {
      if (closest1.count(collider.pcpoints[i]) == 0 || collider.pairdistances2[i] < dclosest1[collider.pcpoints[i]])
      {
        closest1[collider.pcpoints[i]] = collider.meshtris[i];
        dclosest1[collider.pcpoints[i]] = collider.pairdistances2[i];
      }
    }
    for (size_t i = 0; i < collider.meshtris.size(); i++)
    {
      Real dclosest = Inf;
      if(closest2.count(collider.meshtris[i])) 
        dclosest = dclosest2[collider.meshtris[i]];
      if(!b.triNeighbors.empty()) {
        //filter out closest points on edges 
        IntTriple neighbors = b.triNeighbors[collider.meshtris[i]];
        if(neighbors.getIndex(closest1[collider.pcpoints[i]]) >= 0) {
          continue;
        }
        if(neighbors.a >= 0 && closest2.count(neighbors.a))
          dclosest = Min(dclosest,dclosest2[neighbors.a]);
        if(neighbors.b >= 0 && closest2.count(neighbors.b))
          dclosest = Min(dclosest,dclosest2[neighbors.b]);
        if(neighbors.c >= 0 && closest2.count(neighbors.c))
          dclosest = Min(dclosest,dclosest2[neighbors.c]);
      }
      if (collider.pairdistances2[i] < dclosest)
      {
        closest2[collider.meshtris[i]] = collider.pcpoints[i];
        dclosest2[collider.meshtris[i]] = collider.pairdistances2[i];
      }
    }
    elements1.resize(0);
    elements2.resize(0);
    for (const auto &c1 : closest1)
    {
      elements1.push_back(c1.first);
      elements2.push_back(c1.second);
    }
    for (const auto &c2 : closest2)
    {
      if (closest1[c2.second] != c2.first)
      {
        elements1.push_back(c2.second);
        elements2.push_back(c2.first);
      }
    }
    //Basic version
    //elements1=collider.pcpoints;
    //elements2=collider.meshtris;
    return true;
  }
  return false;
}

bool Collides(const CollisionPointCloud &a, Real margin, const CollisionPointCloud &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  return Geometry::Collides(a, b, margin, elements1, elements2, maxContacts);
}



Geometry3DPointCloud::Geometry3DPointCloud()
{}

Geometry3DPointCloud::Geometry3DPointCloud(const Meshing::PointCloud3D& _data)
: data(_data)
{}

Geometry3DPointCloud::Geometry3DPointCloud(Meshing::PointCloud3D&& _data)
: data(_data)
{}

Geometry3DPointCloud::~Geometry3DPointCloud ()
{}

bool Geometry3DPointCloud::Merge(const Geometry3D* geom, const RigidTransform* Tgeom)
{
  if(geom->GetType() != Type::PointCloud) return false;
  if(Tgeom != NULL) {
    Geometry3D* temp = geom->Copy();
    temp->Transform(*Tgeom);
    bool res = Merge(temp);
    delete temp;
    return res;
  }
  const auto& pc = dynamic_cast<const Geometry3DPointCloud*>(geom)->data;
  if(pc.propertyNames.size() != data.propertyNames.size()) return false;
  data.Concat(pc);
  return true;
}

bool Geometry3DPointCloud::Union(const vector<Geometry3D*>& geoms)
{
  size_t numProperties = 0;
  int numPropertyEntries = 0;
  for(size_t i=0;i<geoms.size();i++) {
    if(geoms[i]->GetType() != Type::PointCloud) return false;
    const auto& pc = dynamic_cast<const Geometry3DPointCloud*>(geoms[i])->data;
    if(i == 0) numProperties = pc.propertyNames.size();
    if(pc.propertyNames.size() != numProperties) return false;
    numPropertyEntries += pc.properties.m;
  }
  data.points.resize(0);
  data.properties.resize(numPropertyEntries,numProperties);
  int offset = 0;
  for (size_t i = 0; i < geoms.size(); i++) {
    const auto& pc = dynamic_cast<const Geometry3DPointCloud*>(geoms[i])->data;
    if(i == 0)
      data.propertyNames = pc.propertyNames;
    data.points.insert(data.points.end(),pc.points.begin(),pc.points.end());
    data.properties.copySubMatrix(offset,0,pc.properties);
    offset += pc.properties.m;
  }
  return true;
}

Geometry3D* Geometry3DPointCloud::ConvertTo(Type restype, Real param, Real expansionParameter) const
{
    switch (restype) {
    case Type::TriangleMesh:
        {
        if (!data.IsStructured()) return NULL;
        if (param == 0)
            param = Inf;
        auto* res = new Geometry3DTriangleMesh();
        res->appearance.reset(new GLDraw::GeometryAppearance());
        PointCloudToMesh(data, res->data, *res->appearance, param);
        return res;
        }
    case Type::ConvexHull:
        {
        auto* res = new Geometry3DConvexHull();
        PointCloudToConvexHull(data,res->data);
        return res;
        }
    case Type::OccupancyGrid:
        FatalError("TODO: Convert PointCloud to OccupancyGrid");
        return NULL;
    default:
        return NULL;
    }
}

bool Geometry3DPointCloud::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion) 
{
    switch(geom->GetType()) {
    case Type::Primitive:
    {
        const auto& prim = dynamic_cast<const Geometry3DPrimitive*>(geom)->data;
        if(prim.type == GeometricPrimitive3D::Segment) {
            //special-purpose construction for segments
            const Math3D::Segment3D& s = *AnyCast<Math3D::Segment3D>(&prim.data);

            auto& pc = data;
            if(param == 0) {
                pc.points.push_back(s.a);
                pc.points.push_back(s.b);
            }
            else {
                Real len = s.a.distance(s.b);
                int numSegs = (int)Ceil(len / param);
                pc.points.push_back(s.a);
                for(int i=0;i<numSegs-1;i++) {
                    Real u = Real(i+1)/Real(numSegs+1);
                    pc.points.push_back(s.a+u*(s.b-s.a));
                }
                pc.points.push_back(s.b);
            }
            return true;
        }
        else {
            auto* tmesh = geom->ConvertTo(Type::TriangleMesh, param);
            if(tmesh == NULL) {
              Geometry3DTriangleMesh tmesh;
              if(!tmesh.ConvertFrom(geom,param)) {
                LOG4CXX_WARN(GET_LOGGER(Geometry),"Geometry "<<Geometry3D::TypeName(geom->GetType())<<" could not be converted to TriangleMesh");
                return false;
              }
              if (param == 0)
                  param = Inf;
              return ConvertFrom(&tmesh,param);
            }
            if (param == 0)
                param = Inf;
            ConvertFrom(tmesh,param);
            delete tmesh;
            return true;
        }
    }
    case Type::ConvexHull:
    {
        auto* tmesh = Geometry3D::Make(Type::TriangleMesh);
        bool res = tmesh->ConvertFrom(geom,param);
        if(!res) { delete tmesh; return false; }
        if (param == 0) param = Inf;
        res = ConvertFrom(tmesh,param);
        delete tmesh;
        return res;
    }
    case Type::TriangleMesh:
    {
        if (param == 0)
            param = Inf;
        const auto& mesh = dynamic_cast<const Geometry3DTriangleMesh*>(geom)->data;
        MeshToPointCloud(mesh, data, param, true);
        return true;
    }
    default:
        return false;
    }
}

Geometry3D* Geometry3DPointCloud::Remesh(Real resolution,bool refine,bool coarsen) const
{
    if(resolution <= 0) return NULL;
    const Meshing::PointCloud3D& pc = data;
    auto* res = new Geometry3DPointCloud;
    Meshing::PointCloud3D& output = res->data;
    if(coarsen) {
        GridSubdivision3D grid(resolution);
        GridSubdivision3D::Index index;
        for(size_t i=0;i<pc.points.size();i++) {
            grid.PointToIndex(pc.points[i],index);
            grid.Insert(index,(void*)&pc.points[i]);
        }
        output.points.resize(grid.buckets.size());
        output.properties.resize(grid.buckets.size(),pc.propertyNames.size());
        output.propertyNames = pc.propertyNames;
        output.settings = pc.settings;
        if(pc.IsStructured()) {
            output.settings.erase(output.settings.find("width")); 
            output.settings.erase(output.settings.find("height")); 
        }
        int count = 0;
        for(auto i=grid.buckets.begin();i!=grid.buckets.end();i++,count++) {
            Vector3 ptavg(Zero);
            int n=(int)i->second.size();
            Real scale = 1.0/n;
            for(auto pt:i->second) {
                int ptindex = (const Vector3*)pt - &pc.points[0];
                ptavg += pc.points[ptindex];
            }
            output.points[count] = ptavg*scale;
            for(size_t j=0;j<pc.propertyNames.size();j++) {
                if(pc.propertyNames[j]=="rgb" || pc.propertyNames[j]=="rgba" || pc.propertyNames[j]=="c") {
                    //int numchannels = (int)pc.propertyNames[j].length();
                    int propavg[4] = {0,0,0,0};
                    for(auto pt:i->second) {
                      int ptindex = (const Vector3*)pt - &pc.points[0];
                      int prop = (int)pc.properties(ptindex,j);
                      propavg[0] += (prop&0xff);
                      propavg[1] += ((prop>>8)&0xff);
                      propavg[2] += ((prop>>16)&0xff);
                      propavg[3] += ((prop>>24)&0xff);
                    }
                    for(int k=0;k<4;k++)
                    propavg[k] = (propavg[k]/n)&0xff;
                    output.properties(count,j) = (Real)(propavg[0] | (propavg[1] << 8) | (propavg[2] << 16) | (propavg[3] << 24));
                }
                else {
                    Real propavg = 0;
                    for(auto pt:i->second) {
                        int ptindex = (const Vector3*)pt - &pc.points[0];
                        propavg += pc.properties(ptindex,j);
                    }
                    output.properties(count,j) = propavg*scale;
                }
            }
        }
    }
    else
        res->data = data;
    return res;
}

Geometry3D* Geometry3DPointCloud::Slice(const RigidTransform& T,Real tol) const
{
  const Meshing::PointCloud3D& pc=data;
  vector<Vector2> pts;
  vector<int> inds;
  Geometry::SliceXY(pc,T,tol,pts,inds);
  auto* res = new Geometry3DPointCloud;
  Meshing::PointCloud3D& pc_out = res->data;
  pc_out.propertyNames = pc.propertyNames;
  pc_out.settings = pc.settings;
  pc_out.settings.remove("width");
  pc_out.settings.remove("height");
  pc_out.points.resize(pts.size());
  pc_out.properties.resize(pts.size(),pc.properties.n);
  for(size_t i=0;i<pts.size();i++) {
    pc_out.points[i].set(pts[i].x,pts[i].y,0);
    Vector prop;
    pc.properties.getRowRef(inds[i],prop);
    pc_out.properties.copyRow(i,prop);
  }
  return res;
}
  
Geometry3D* Geometry3DPointCloud::ExtractROI(const AABB3D& bb,int flags) const
{
  auto* res = new Geometry3DPointCloud;
  Geometry::ExtractROI(data,bb,res->data,flags);
  return res;
}

Geometry3D* Geometry3DPointCloud::ExtractROI(const Box3D& bb,int flags) const
{
  auto* res = new Geometry3DPointCloud;
  Geometry::ExtractROI(data,bb,res->data,flags);
  return res;
}

shared_ptr<Geometry3D> Geometry3DPointCloud::GetElement(int elem) const
{
  return make_shared<Geometry3DPrimitive>(GeometricPrimitive3D(data.points[elem]));
}

AABB3D Geometry3DPointCloud::GetAABB() const
{
  AABB3D bb;
  data.GetAABB(bb.bmin,bb.bmax);
  return bb;
}

bool Geometry3DPointCloud::Support(const Vector3& dir,Vector3& pt) const
{
  if(data.points.empty()) return false;
  Real farthest = -Inf;
  for(size_t i=0;i<data.points.size();i++) {
    Real d=dir.dot(data.points[i]);
    if(d > farthest) {
      farthest = d;
      pt = data.points[i];
    }
  }
  return true;
}

bool Geometry3DPointCloud::Load(const char *fn)
{
  const char *ext = ::FileExtension(fn);
  if (0 == strcmp(ext, "pcd"))
  {
    data = Meshing::PointCloud3D();
    return data.LoadPCL(fn);
  }
  return false;
}

    
bool Geometry3DPointCloud::Save(const char* fn) const
{
  const char *ext = ::FileExtension(fn);
  if (0 == strcmp(ext, "pcd"))
  {
    return data.SavePCL(fn);
  }
  else
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Save: Unknown point cloud file extension " << fn);
    return false;
  }
}
  
bool Geometry3DPointCloud::Load(istream& in)
{
  return data.LoadPCL(in);
}

bool Geometry3DPointCloud::Save(ostream& out) const
{
  return data.SavePCL(out);
}

bool Geometry3DPointCloud::Transform(const Matrix4 &T)
{
  data.Transform(T);
  return true;
}

Collider3DPointCloud::Collider3DPointCloud(shared_ptr<Geometry3DPointCloud> _data)
:data(_data),collisionData(data->data)
{
  Reset();
}

Collider3DPointCloud::Collider3DPointCloud(const Collider3DPointCloud& rhs)
:data(rhs.data),collisionData(rhs.collisionData)
{
}

void Collider3DPointCloud::Reset()
{
  collisionData.points = data->data.points;
  collisionData.settings = data->data.settings;
  collisionData.properties = data->data.properties;
  collisionData.propertyNames = data->data.propertyNames;
  collisionData.InitCollisions();
}

Collider3D* Collider3DPointCloud::Copy(shared_ptr<Geometry3D> geom) const
{
  auto* res = new Collider3DPointCloud(*this);
  res->data = dynamic_pointer_cast<Geometry3DPointCloud>(geom);
  return res;
}
    

AABB3D Collider3DPointCloud::GetAABB() const
{
  Box3D b = GetBB();
  AABB3D bb;
  b.getAABB(bb);
  return bb;
}

AABB3D Collider3DPointCloud::GetAABBTight() const
{
  const CollisionPointCloud &pc = collisionData;
  AABB3D bb;
  bb.minimize();
  for (size_t i = 0; i < pc.points.size(); i++)
    bb.expand(pc.currentTransform * pc.points[i]);
  return bb;
}

Box3D Collider3DPointCloud::GetBB() const
{
  Box3D b;
  b.setTransformed(collisionData.bblocal,collisionData.currentTransform);
  return b;
}

bool Collider3DPointCloud::Distance(const Vector3& pt,Real& result)
{
  result = Geometry::Distance(collisionData, pt);
  return true;
}

bool Collider3DPointCloud::Distance(const Vector3 &pt, const AnyDistanceQuerySettings &settings,AnyDistanceQueryResult& res)
{
  res.hasClosestPoints = true;
  res.hasElements = true;
  res.elem2 = 0;
  res.cp2 = pt;
  Vector3 ptlocal;

  GetTransform().mulInverse(pt, ptlocal);
  const CollisionPointCloud &pc = collisionData;
  if (!pc.octree->NearestNeighbor(ptlocal, res.cp1, res.elem1, settings.upperBound)) {
    res.d = settings.upperBound;
    return true;
  }
  res.d = res.cp1.distance(ptlocal);
  Transform1(res, GetTransform());
  return true;
}

bool Collider3DPointCloud::Contains(const Vector3& pt,bool& result) 
{
  GeometricPrimitive3D gpt(pt);
  vector<int> elements;
  if (Geometry::Collides(gpt, collisionData, 0.0, elements, 1)) {
    result = true;
  }
  else {
    result = false;
  }
  return true;  
}

bool Collider3DPointCloud::WithinDistance(Collider3D* geom,Real d,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  switch (geom->GetType())
  {
  case Type::Primitive:
  {
    GeometricPrimitive3D bw = dynamic_cast<Collider3DPrimitive*>(geom)->data->data;
    bw.Transform(geom->GetTransform());
    if (Geometry::Collides(bw, collisionData, d, elements1, maxContacts))
    {
      elements2.resize(elements1.size());
      std::fill(elements2.begin(),elements2.end(),0);
    }
    return true;
  }
  case Type::TriangleMesh:
  {
    auto& b = dynamic_cast<Collider3DTriangleMesh*>(geom)->collisionData;
    bool res = Geometry::Collides(collisionData, d, b, elements1, elements2, maxContacts);
    if(res) Assert(!elements1.empty());
    else Assert(elements1.empty());
    return true;
  }
  case Type::PointCloud:
  {
    auto& b = dynamic_cast<Collider3DPointCloud*>(geom)->collisionData;
    bool res = Geometry::Collides(collisionData, d, b, elements1, elements2, maxContacts);
    if(res) Assert(!elements1.empty());
    else Assert(elements1.empty());
    return true;
  }
  case Type::ConvexHull:
  {
    auto* b = dynamic_cast<Collider3DConvexHull*>(geom);
    AABB3D bb = b->GetAABBTight();
    GeometricPrimitive3D gbb(bb);
    vector<int> cand_elements1;
    size_t extra_maxContacts = maxContacts;
    if(maxContacts < INT_MAX/2) extra_maxContacts *= 2;
    while(true) {   //check hull bbox for candidate points.  Increase extra points until we find some that intersect the hull
        cand_elements1.resize(0);
        elements1.resize(0);
        elements2.resize(0);
        if(Geometry::Collides(gbb, collisionData, d, cand_elements1, extra_maxContacts)) {
            //printf("Checking %d candidate points for containment\n",(int)cand_elements1.size());
            for(size_t i=0;i<cand_elements1.size();i++) {
                Vector3 pt = collisionData.currentTransform * collisionData.points[cand_elements1[i]];
                bool inside;
                if(!b->Contains(pt,inside)) return false;
                if(inside) {
                    elements1.push_back(cand_elements1[i]);
                    elements2.push_back(0);
                    if(elements1.size() >= maxContacts) break;
                }
            }
        }
        if(cand_elements1.size() < extra_maxContacts) break;
        if(extra_maxContacts >= INT_MAX/2) break;
        if(elements1.size() >= maxContacts) break;
        extra_maxContacts *= 2;   //try again with a larger limit 
    }
    return true;
  }
  default:
    return false;
  } 
}

bool Collider3DPointCloud::RayCast(const Ray3D& r,Real margin,Real& distance,int& element)
{
  const CollisionPointCloud &pc = collisionData;
  Vector3 pt;
  element = Geometry::RayCast(pc, margin, r, pt);
  if (element < 0)
    return true;
  Vector3 temp;
  distance = r.closestPoint(pt, temp);
  return true;
}

const static Real gNormalFromGeometryTolerance = 1e-5;

//defined in CollisionMesh.cpp
Vector3 ContactNormal(CollisionMesh &m, const Vector3 &p, int t, const Vector3 &closestPt);

void MeshPointCloudContacts(CollisionMesh &m1, Real outerMargin1, CollisionPointCloud &pc2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  vector<int> tris;
  if (!Collides(pc2, tol, m1, points, tris, maxcontacts))
    return;
  Assert(points.size() == tris.size());
  Triangle3D tri, triw;
  contacts.reserve(points.size());
  for (size_t i = 0; i < points.size(); i++)
  {
    Vector3 pw = pc2.currentTransform * pc2.points[points[i]];
    m1.GetTriangle(tris[i], tri);
    triw.a = m1.currentTransform * tri.a;
    triw.b = m1.currentTransform * tri.b;
    triw.c = m1.currentTransform * tri.c;
    Vector3 cp = triw.closestPoint(pw);
    Vector3 n = pw - cp;
    Real d = n.length();
    if (d < gNormalFromGeometryTolerance)
    { //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp, plocal);
      n = ContactNormal(m1, plocal, tris[i], pw);
      n.inplaceNegative();
    }
    else if (d > tol)
    { //some penetration -- we can't trust the result of PQP
      continue;
    }
    else
      n /= d;
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = cp + outerMargin1 * n;
    contacts[k].p2 = pw - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = tris[i];
    contacts[k].elem2 = points[i];
    contacts[k].unreliable = false;
  }
  /*
  Real tol = outerMargin1 + outerMargin2;
  Box3D mbb,mbb_pclocal;
  GetBB(m1,mbb);
  RigidTransform Tw_pc;
  Tw_pc.setInverse(pc2.currentTransform);
  mbb_pclocal.setTransformed(mbb,Tw_pc);
  AABB3D maabb_pclocal;
  mbb_pclocal.getAABB(maabb_pclocal);
  maabb_pclocal.bmin -= Vector3(tol);
  maabb_pclocal.bmax += Vector3(tol);
  maabb_pclocal.setIntersection(pc2.bblocal);
  list<void*> nearpoints;
  pc2.grid.BoxItems(Vector(3,maabb_pclocal.bmin),Vector(3,maabb_pclocal.bmax),nearpoints);
  int k=0;
  vector<int> tris;
  Triangle3D tri,triw;
  for(list<void*>::iterator i=nearpoints.begin();i!=nearpoints.end();i++) {
    Vector3 pcpt = *reinterpret_cast<Vector3*>(*i);
    Vector3 pw = pc2.currentTransform*pcpt;
    NearbyTriangles(m1,pw,tol,tris,maxcontacts-k);
    for(size_t j=0;j<tris.size();j++) {   
      m1.GetTriangle(tris[j],tri);
      triw.a = m1.currentTransform*tri.a;
      triw.b = m1.currentTransform*tri.b;
      triw.c = m1.currentTransform*tri.c;
      Vector3 cp = triw.closestPoint(pw);
      Vector3 n = cp - pw;
      Real d = n.length();
      if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
        Vector3 plocal;
        m1.currentTransform.mulInverse(cp,plocal);
        n = ContactNormal(m1,plocal,tris[j],pw);
      }
      else if(d > tol) {  //some penetration -- we can't trust the result of PQP
        continue;
      }
      else n /= d;
      //migrate the contact point to the center of the overlap region
      CopyVector(contact[k].pos,0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n);
      CopyVector(contact[k].n,n);
      contact[k].depth = tol - d;
      k++;
      if(k == maxcontacts) break;
    }
  }
  */
}

void PointCloudPrimitiveContacts(CollisionPointCloud &pc1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  if (g2.type == GeometricPrimitive3D::Empty)
    return;
  if (!g2.SupportsClosestPoints(GeometricPrimitive3D::Point))
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Cannot do contact checking on point cloud vs primitive " << g2.TypeName() << " yet");
    return;
  }

  GeometricPrimitive3D gworld = g2;
  gworld.Transform(T2);

  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  NearbyPoints(pc1, gworld, tol, points, maxcontacts);
  contacts.reserve(points.size());
  for (size_t j = 0; j < points.size(); j++)
  {
    Vector3 pw = pc1.currentTransform * pc1.points[points[j]];
    Vector3 cp, n;
    Real d = gworld.ClosestPoints(pw,cp,n);
    if (d > tol)
      continue;
    if (d < gNormalFromGeometryTolerance)
    { //too close?
      continue;
    }
    else if (d > tol)
    { //why did this point get farther away?
      continue;
    }
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = pw + outerMargin1 * n;
    contacts[k].p2 = cp - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points[j];
    contacts[k].elem2 = -1;
    contacts[k].unreliable = false;
  }
}

void PointCloudConvexHullContacts(CollisionPointCloud &pc1, Real outerMargin1, ConvexHull3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  if(g2.type == ConvexHull3D::Type::Empty)
    return;
  RigidTransform T_pc_to_ch;
  T_pc_to_ch.mulInverseA(T2, pc1.currentTransform);

  AABB3D g2bb = g2.GetAABB();
  Box3D g2bb_world;
  g2bb_world.setTransformed(g2bb, T2);

  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  NearbyPoints(pc1, GeometricPrimitive3D(g2bb_world), tol, points, maxcontacts);
  contacts.reserve(points.size());
  for (size_t j = 0; j < points.size(); j++)
  {
    Vector3 pg = T_pc_to_ch * pc1.points[points[j]];
    Vector3 cpg, ng;
    Real d = g2.ClosestPoint(pg,cpg,ng);
    if (d > tol)
      continue;
    Vector3 n = T2.R*ng;
    Vector3 cp = T2 * cpg;
    if (d < gNormalFromGeometryTolerance)
    { //too close?
      continue;
    }
    else if (d > tol)
    { //why did this point get farther away?
      continue;
    }
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = pc1.currentTransform*pc1.points[points[j]] + outerMargin1 * n;
    contacts[k].p2 = cp - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points[j];
    contacts[k].elem2 = -1;
    contacts[k].unreliable = false;
  }
}


void PointCloudPointCloudContacts(CollisionPointCloud &pc1, Real outerMargin1, CollisionPointCloud &pc2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points1, points2;
  Collides(pc1, pc2, tol, points1, points2, maxcontacts);

  //this is already done in Collides
  /*  
  map<int,int> closest1,closest2;
  map<int,Real> dclosest1,dclosest2;
  //filter by closest point 
  for(size_t i=0;i<points1.size();i++) {
    Vector3 p1w = pc1.currentTransform*pc1.points[points1[i]];
    Vector3 p2w = pc2.currentTransform*pc2.points[points2[i]];
    Real d=p1w.distance(p2w);
    if(closest1.count(points1[i])==0 || d < dclosest1[points1[i]]) {
      closest1[points1[i]] = points2[i];
      dclosest1[points1[i]] = d;
    }
    if(closest2.count(points2[i])==0 || d < dclosest2[points2[i]]) {
      closest2[points2[i]] = points1[i];
      dclosest2[points2[i]] = d;
    }
  }
  points1.resize(0);
  points2.resize(0);
  for(const auto& c1: closest1) {
    points1.push_back(c1.first);
    points2.push_back(c1.second);
  }
  for(const auto& c2: closest2) {
    if(closest1[c2.second] != c2.first) {
      points1.push_back(c2.second);
      points2.push_back(c2.first);
    }
  }
  */
  contacts.reserve(points1.size());
  for (size_t j = 0; j < points1.size(); j++)
  {
    Vector3 p1w = pc1.currentTransform * pc1.points[points1[j]];
    Vector3 p2w = pc2.currentTransform * pc2.points[points2[j]];
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].n = p2w - p1w;
    Real d = contacts[k].n.norm();
    if (!FuzzyZero(d))
    {
      contacts[k].n /= d;
      contacts[k].unreliable = false;
    }
    else
    {
      contacts[k].n.setZero();
      contacts[k].unreliable = true;
    }
    contacts[k].p1 = p1w + contacts[k].n * outerMargin1;
    contacts[k].p2 = p2w - contacts[k].n * outerMargin2;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points1[j];
    contacts[k].elem2 = points2[j];
  }
}

bool Collider3DPointCloud::Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) 
{
  switch (other->GetType())
  {
  case Type::Primitive:
    {
      auto* prim = dynamic_cast<Collider3DPrimitive*>(other);
      PointCloudPrimitiveContacts(collisionData, settings.padding1, prim->data->data, prim->T, settings.padding2, res.contacts, settings.maxcontacts);
      return true;
    }
  case Type::TriangleMesh:
    {
      auto* mesh = dynamic_cast<Collider3DTriangleMesh*>(other);
      MeshPointCloudContacts(mesh->collisionData, settings.padding2, collisionData, settings.padding1, res.contacts, settings.maxcontacts);
      for(size_t i=0;i<res.contacts.size();i++) 
        ReverseContact(res.contacts[i]);
      return true;
    }
  case Type::PointCloud:
    {
      auto* pc = dynamic_cast<Collider3DPointCloud*>(other);
      PointCloudPointCloudContacts(collisionData, settings.padding1, pc->collisionData, settings.padding2, res.contacts, settings.maxcontacts);
      return true;
    }
  case Type::ConvexHull:
    {
      auto* ch = dynamic_cast<Collider3DConvexHull*>(other);
      PointCloudConvexHullContacts(collisionData, settings.padding1, ch->data->data, ch->T, settings.padding2, res.contacts, settings.maxcontacts);
      return true;
    }
  default:
      return false;
  }
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.elem1 = 0;
  res.d = Distance(b, a, res.elem2, settings.upperBound);
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.cp2 = b.currentTransform * b.points[res.elem2];
  a.ClosestPoints(res.cp2, res.cp1, res.dir1);
  res.dir2.setNegative(res.dir1);
  return res;
}

class PointMeshDistance
{
public:
  const CollisionPointCloud &pc;
  const CollisionMesh &mesh;
  RigidTransform Tba, Twa, Tab;
  PQP_REAL pqpRab[3][3], pqpTab[3];
  BV pcbv;
  Real absErr, relErr;
  Real upperBound;
  int pcpoint;
  int meshtri;
  Vector3 meshpoint; //note: local coordinates
  Heap<pair<int, int>, Real> queue;

  PointMeshDistance(const CollisionPointCloud &a, const CollisionMesh &b, Real _absErr, Real _relErr, Real _upperBound = Inf)
      : pc(a), mesh(b), absErr(_absErr), relErr(_relErr), upperBound(_upperBound), pcpoint(-1), meshtri(-1)
  {
    Twa.setInverse(a.currentTransform);
    Tba.mul(Twa, b.currentTransform);
    Tab.setInverse(Tba);
    //RigidTransformToPQP(Tab,pqpRab,pqpTab);
    RigidTransformToPQP(Tba, pqpRab, pqpTab);

    RigidTransform ident;
    ident.setIdentity();
    RigidTransformToPQP(ident, pcbv.R, pcbv.To);

    //start with initial upper bound by taking some arbitrary point
    Triangle3D tri;
    b.GetTriangle(b.tris.size() / 3, tri);
    Vector3 pt = Tab * pc.points[pc.points.size() / 3];
    Vector3 cp = tri.closestPoint(pt);
    Real d = cp.distance(pt);
    if (d < upperBound)
    {
      upperBound = d;
      pcpoint = int(pc.points.size() / 3);
      meshtri = int(b.tris.size() / 3);
      meshpoint = cp;
    }
  }
  Real Distance(const OctreeNode &pcnode, const BV &meshnode)
  {
    //Vector3 halfdims = (pcnode.bb.bmax-pcnode.bb.bmin)*0.5;
    //Vector3 center = (pcnode.bb.bmin+pcnode.bb.bmax)*0.5;
    //halfdims.get(pcbv.d);
    //center.get(pcbv.Tr);
    //PQP uses rectangle-swept-sphere to do distance calculation
    Vector3 d = pcnode.bb.bmax - pcnode.bb.bmin;
    pcbv.Tr[0] = pcnode.bb.bmin.x;
    pcbv.Tr[1] = pcnode.bb.bmin.y;
    pcbv.Tr[2] = (pcnode.bb.bmin.z + pcnode.bb.bmax.z) * 0.5;
    pcbv.l[0] = d.x;
    pcbv.l[1] = d.y;
    pcbv.r = d.z * 0.5;
    return BV_Distance2(pqpRab, pqpTab, &pcbv, &meshnode);
  }
  void UpdateLeaves(const OctreeNode &pcOctreeNode, const BV &meshnode)
  {
    int t = -meshnode.first_child - 1;
    Triangle3D tri;
    Copy(mesh.pqpModel->tris[t].p1, tri.a);
    Copy(mesh.pqpModel->tris[t].p2, tri.b);
    Copy(mesh.pqpModel->tris[t].p3, tri.c);
    tri.a = Tba * tri.a;
    tri.b = Tba * tri.b;
    tri.c = Tba * tri.c;
    //collide the triangle and points
    vector<Vector3> pts;
    vector<int> pcids;
    pc.octree->GetPoints(pcOctreeNode, pts);
    pc.octree->GetPointIDs(pcOctreeNode, pcids);
    Vector3 cp;
    for (size_t i = 0; i < pts.size(); i++)
    {
      cp = tri.closestPoint(pts[i]);
      Real d = pts[i].distance(cp);
      if (d < upperBound)
      {
        upperBound = d;
        pcpoint = pcids[i];
        meshtri = t;
        meshpoint = Tab * cp;
      }
    }
  }
  void Recurse()
  {
    int maxqueuesize = 100;
    queue.push(make_pair(0, 0), -Distance(pc.octree->Node(0), mesh.pqpModel->b[0]));
    while (!queue.empty())
    {
      if (-queue.topPriority() * (1 + relErr) + absErr >= upperBound)
        break;
      pair<int, int> next = queue.top();
      queue.pop();
      int pcOctreeNode = next.first;
      int meshBVHNode = next.second;

      if (queue.size() >= maxqueuesize)
      {
        Recurse(pcOctreeNode, meshBVHNode);
        continue;
      }

      const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
      const BV &meshnode = mesh.pqpModel->b[meshBVHNode];
      if (pc.octree->IsLeaf(pcnode))
      {
        if (pc.octree->NumPoints(pcnode) == 0)
          continue;
        if (meshnode.Leaf())
        {
          UpdateLeaves(pcnode, meshnode);
          continue;
        }
      }
      bool splitPC = true;
      if (pc.octree->IsLeaf(pcnode))
        splitPC = false;
      else if (!meshnode.Leaf())
      {
        //determine which BVH to split
        Real vpc = Volume(pcnode);
        Real vmesh = Volume(meshnode);
        if (vpc * 10 < vmesh)
          splitPC = false;
      }
      if (splitPC)
      {
        for (int i = 0; i < 8; i++)
        {
          const auto &child = pc.octree->Node(pcnode.childIndices[i]);
          if (child.bb.bmin.x > child.bb.bmax.x)
            continue;
          Real d = Distance(child, meshnode);
          if (d * (1.0 + relErr) + absErr < upperBound)
          {
            queue.push(pair<int, int>(pcnode.childIndices[i], meshBVHNode), -d);
          }
        }
      }
      else
      {
        int c1 = mesh.pqpModel->b[meshBVHNode].first_child;
        int c2 = c1 + 1;
        assert(c1 >= 0);
        Real d1 = Distance(pcnode, mesh.pqpModel->b[c1]);
        Real d2 = Distance(pcnode, mesh.pqpModel->b[c2]);
        if (d1 * (1.0 + relErr) + absErr < upperBound)
        {
          queue.push(pair<int, int>(pcOctreeNode, c1), -d1);
        }
        if (d2 * (1.0 + relErr) + absErr < upperBound)
        {
          queue.push(pair<int, int>(pcOctreeNode, c2), -d2);
        }
      }
    }
  }
  void Recurse(int pcOctreeNode, int meshBVHNode)
  {
    const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
    const BV &meshnode = mesh.pqpModel->b[meshBVHNode];
    if (pc.octree->IsLeaf(pcnode))
    {
      if (pc.octree->NumPoints(pcnode) == 0)
        return;
      if (meshnode.Leaf())
      {
        UpdateLeaves(pcnode, meshnode);
        return;
      }
    }
    bool splitPC = true;
    if (pc.octree->IsLeaf(pcnode))
      splitPC = false;
    else if (!meshnode.Leaf())
    {
      //determine which BVH to split
      Real vpc = Volume(pcnode);
      Real vmesh = Volume(meshnode);
      if (vpc * 10 < vmesh)
        splitPC = false;
    }
    if (splitPC)
    {
      vector<pair<Real, int>> sorter;
      for (int i = 0; i < 8; i++)
      {
        const auto &child = pc.octree->Node(pcnode.childIndices[i]);
        if (child.bb.bmin.x > child.bb.bmax.x)
          continue;
        Real d = Distance(child, meshnode);
        if (d * (1.0 + relErr) + absErr < upperBound)
        {
          sorter.push_back(make_pair(d, pcnode.childIndices[i]));
        }
      }
      sort(sorter.begin(), sorter.end());
      for (const auto &order : sorter)
      {
        if (order.first * (1.0 + relErr) + absErr < upperBound)
          Recurse(order.second, meshBVHNode);
        else
          break;
      }
    }
    else
    {
      int c1 = mesh.pqpModel->b[meshBVHNode].first_child;
      int c2 = c1 + 1;
      assert(c1 >= 0);
      Real d1 = Distance(pcnode, mesh.pqpModel->b[c1]);
      Real d2 = Distance(pcnode, mesh.pqpModel->b[c2]);
      if (d1 < d2)
      {
        if (d1 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c1);
        }
        if (d2 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c2);
        }
      }
      else
      {
        if (d2 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c2);
        }
        if (d1 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c1);
        }
      }
    }
  }
};

AnyDistanceQueryResult Distance(const CollisionPointCloud &a, const CollisionMesh &b, const AnyDistanceQuerySettings &settings)
{
  PointMeshDistance solver(a, b, settings.absErr, settings.relErr, settings.upperBound);
  solver.Recurse();
  AnyDistanceQueryResult res;
  res.d = solver.upperBound;
  res.elem1 = solver.pcpoint;
  res.elem2 = solver.meshtri;
  res.cp1 = a.currentTransform * a.points[solver.pcpoint];
  res.cp2 = b.currentTransform * solver.meshpoint;
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}

AnyDistanceQueryResult Distance(const CollisionPointCloud &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.d = Geometry::Distance(a, b, res.elem1, res.elem2, settings.upperBound);
  res.cp1 = a.currentTransform * a.points[res.elem1];
  res.cp2 = b.currentTransform * b.points[res.elem2];
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}


bool Collider3DPointCloud::Distance(Collider3D* other,const DistanceQuerySettings& settings,DistanceQueryResult& res) 
{
  switch (other->GetType())
  {
  case Type::Primitive:
  {
    GeometricPrimitive3D bw = dynamic_cast<Collider3DPrimitive*>(other)->data->data;
    bw.Transform(other->GetTransform());
    res = Geometry::Distance(bw, collisionData, settings);
    Flip(res);
    return true;
  }
  case Type::TriangleMesh:
  {
    CollisionMesh& b = dynamic_cast<Collider3DTriangleMesh*>(other)->collisionData;
    res = Geometry::Distance(collisionData, b, settings);
    return true;
  }
  case Type::PointCloud:
  {
    CollisionPointCloud& b = dynamic_cast<Collider3DPointCloud*>(other)->collisionData;
    res = Geometry::Distance(collisionData, b, settings);
    return true;
  }
  case Type::ConvexHull:
  {
    res.d = Inf;
    DistanceQuerySettings modsettings = settings;
    DistanceQueryResult tempres;
    Collider3DConvexHull* b = dynamic_cast<Collider3DConvexHull*>(other);
    //bound check
    AABB3D chbb = other->GetAABBTight();
    Box3D bb = GetBB();
    Collider3DPrimitive prim(make_shared<Geometry3DPrimitive>(bb));
    Vector3 tmp;
    if(b->Distance(&prim,settings,tempres) && tempres.d > 0) {
      // printf("Distance between convex hull and point cloud bounding box: %f\n",tempres.d);
      if(tempres.d >= settings.upperBound) return true; //no need to check further
      //find points within the given distance of the closest point on the hull
      Assert(tempres.hasClosestPoints);
      Real d_cp_points;
      bool supported = Distance(tempres.cp1,d_cp_points);
      // printf("Distance between point on convex hull and point cloud: %f\n",d_cp_points);
      Assert(supported);
      //find all points within d_cp_points of the bound of the convex hull
      vector<int> pc_elements,bb_elements;
      Collider3DPrimitive bb_geom(make_shared<Geometry3DPrimitive>(chbb));
      if(WithinDistance(&bb_geom,d_cp_points+1e-5,pc_elements,bb_elements)) {
        for(auto i:pc_elements) {
          Vector3 pw = collisionData.currentTransform*collisionData.points[i];
          if(chbb.distanceSquared(pw,tmp) > modsettings.upperBound*modsettings.upperBound) continue;
          if(b->Distance(pw,modsettings,tempres)) {
            if(tempres.d < modsettings.upperBound) {
              res = tempres;
              res.hasElements = true;
              res.elem1 = i;
              res.elem2 = 0;
              modsettings.upperBound = tempres.d;
              if(tempres.d <= 0) break;
            }
          }
        }
      }
      else {
        //no points within the distance bound, must be a numerical error?
        res.d = d_cp_points;
        return true;
      }
    }
    else {
      //couldn't find a convex hull - box distance?  must have overlap or be a weird convex hull type
      static bool warned = false;
      if(!warned) {
        LOG4CXX_DEBUG(GET_LOGGER(Geometry),"Warning, point cloud-convex hull distance using slow linear search");
        warned = true;
      }

      //brute force
      for(size_t i=0;i<data->data.points.size();i++) {
        Vector3 pw = collisionData.currentTransform*collisionData.points[i];
        if(chbb.distanceSquared(pw,tmp) > modsettings.upperBound*modsettings.upperBound) continue;
        if(b->Distance(pw,modsettings,tempres)) {
          if(tempres.d < modsettings.upperBound) {
            res = tempres;
            res.hasElements = true;
            res.elem1 = i;
            res.elem2 = 0;
            modsettings.upperBound = tempres.d;
            if(tempres.d <= 0) break;
          }
        }
      }
    }
    Flip(res);
    return true;
  }
  default:
    return false;
  }
}

Collider3D* Collider3DPointCloud::Slice(const RigidTransform& T,Real tol) const
{
  vector<int> inds;
  vector<Vector2> pts;
  Geometry::SliceXY(collisionData,T,tol,pts,inds);
  auto* pc = new Geometry3DPointCloud();
  Meshing::PointCloud3D& pc_out = pc->data;
  pc_out.propertyNames = data->data.propertyNames;
  pc_out.settings = data->data.settings;
  pc_out.settings.remove("width");
  pc_out.settings.remove("height");
  pc_out.points.resize(pts.size());
  pc_out.properties.resize(pts.size(),data->data.properties.n);
  for(size_t i=0;i<pts.size();i++) {
    pc_out.points[i].set(pts[i].x,pts[i].y,0);
    Vector prop;
    data->data.properties.getRowRef(inds[i],prop);
    pc_out.properties.copyRow(i,prop);
  }
  auto* res = new Collider3DPointCloud(shared_ptr<Geometry3DPointCloud>(pc));
  res->SetTransform(T);
  return res;
}

Collider3D* Collider3DPointCloud::ExtractROI(const AABB3D& bb,int flags) const
{
  auto *res = new Collider3DPointCloud(make_shared<Geometry3DPointCloud>());
  Geometry::ExtractROI(collisionData,bb,res->collisionData,flags);
  res->data->data = res->collisionData;
  return res;
}

Collider3D* Collider3DPointCloud::ExtractROI(const Box3D& bb,int flags) const
{
  auto *res = new Collider3DPointCloud(make_shared<Geometry3DPointCloud>());
  Geometry::ExtractROI(collisionData,bb,res->collisionData,flags);
  res->data->data = res->collisionData;
  return res;
}


} //namespace Geometry
