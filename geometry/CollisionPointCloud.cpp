#include <KrisLibrary/Logger.h>
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

CollisionPointCloud::CollisionPointCloud(const CollisionPointCloud& _pc)
  :Meshing::PointCloud3D(_pc),bblocal(_pc.bblocal),currentTransform(_pc.currentTransform),
   gridResolution(_pc.gridResolution),grid(_pc.grid),
   octree(_pc.octree)
{}

void CollisionPointCloud::InitCollisions()
{
  bblocal.minimize();
  grid.buckets.clear();
  octree = NULL;
  if(points.empty()) 
    return;
  Assert(points.size() > 0);
  Timer timer;
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
  LOG4CXX_INFO(KrisLibrary::logger(),"CollisionPointCloud::InitCollisions: "<<validptcount<<" valid points, res "<<res<<", time "<<timer.ElapsedTime());
  //print stats
  int nmax = 0;
  for(GridSubdivision3D::HashTable::const_iterator i=grid.buckets.begin();i!=grid.buckets.end();i++)
    nmax = Max(nmax,(int)i->second.size());
  LOG4CXX_INFO(KrisLibrary::logger(),"  "<<grid.buckets.size()<<" nonempty grid buckets, max size "<<nmax<<", avg "<<Real(points.size())/grid.buckets.size());
  timer.Reset();

  //initialize the octree, 10 points per cell, res is minimum cell size
  octree = make_shared<OctreePointSet>(bblocal,10,res);
  for(size_t i=0;i<points.size();i++) {
    if(IsFinite(points[i].x))
      octree->Add(points[i],(int)i);
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"  octree initialized in time "<<timer.ElapsedTime()<<"s, "<<octree->Size()<<" nodes, depth "<<octree->MaxDepth());
  //TEST: should we fit to points
  octree->FitToPoints();
  LOG4CXX_INFO(KrisLibrary::logger(),"  octree fit to points in time "<<timer.ElapsedTime());
  /*
  //TEST: method 2.  Turns out to be much slower
  timer.Reset();
  octree = new OctreePointSet(bblocal,points.size());
  octree->SplitToResolution(res);
  for(size_t i=0;i<points.size();i++)
    octree->Add(points[i],(int)i);
  octree->Collapse(10);
  LOG4CXX_INFO(KrisLibrary::logger(),"  octree 2 initialized in time "<<timer.ElapsedTime()<<"s, "<<octree->Size());
  */
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

  //octree overlap method
  vector<Vector3> points;
  vector<int> ids;
  pc.octree->BoxQuery(gbb.bmin-Vector3(tol),gbb.bmax+Vector3(tol),points,ids);
  for(size_t i=0;i<points.size();i++) 
    if(glocal.Distance(points[i]) <= tol) return true;
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

Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g)
{
  int cpoint;
  return Distance(pc,g,cpoint);
}

Real Distance(const CollisionPointCloud& pc,const GeometricPrimitive3D& g,int& closestPoint,Real upperBound)
{
  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);
  closestPoint=-1;
  if(!IsInf(upperBound) && glocal.Distance(pc.bblocal) > upperBound) {
    return upperBound;
  }
  bool doBoundsTest= (g.type != GeometricPrimitive3D::Point && g.type != GeometricPrimitive3D::Sphere && g.type != GeometricPrimitive3D::AABB);
  if(doBoundsTest) {
    AABB3D gbb = glocal.GetAABB();
    Sphere3D sbb;
    sbb.center = 0.5*(gbb.bmin+gbb.bmax);
    sbb.radius = sbb.center.distance(gbb.bmin);
    /*
    gDistanceTestValue = Inf;
    gDistanceTestObject = &glocal;
    pc.grid.BoxQuery(gbb.bmin,gbb.bmax,distanceTest);
    return gDistanceTestValue;
    */
    Real radius0 = sbb.radius;
    sbb.radius += upperBound;
    //AABB3D bb0 = gbb;
    //gbb.bmin -= Vector3(upperBound);
    //gbb.bmax += Vector3(upperBound);
    //test all points, linearly
    Real dmax = upperBound;
    for(size_t i=0;i<pc.points.size();i++) {
      if(sbb.contains(pc.points[i])) {
        Real d = glocal.Distance(pc.points[i]);
        if(d < dmax) {
          closestPoint = (int)i;
          dmax = d;
          sbb.radius = radius0 + dmax;
        }
      }
    }
    return dmax;
  }
  else {
    Real dmax = upperBound;
    //bounds testing not worth it
    for(size_t i=0;i<pc.points.size();i++) {
      Real d = glocal.Distance(pc.points[i]);
      if(d < dmax) {
        closestPoint = (int)i;
        dmax = d;
      }
    }
    return dmax;
  }
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
  if(g.Distance(bb) > tol) return;

  GeometricPrimitive3D glocal = g;
  RigidTransform Tinv;
  Tinv.setInverse(pc.currentTransform);
  glocal.Transform(Tinv);

  AABB3D gbb = glocal.GetAABB();
  gbb.setIntersection(pc.bblocal);

  //octree overlap method
  vector<Vector3> points;
  vector<int> ids;
  pc.octree->BoxQuery(gbb.bmin-Vector3(tol),gbb.bmax+Vector3(tol),points,ids);
  for(size_t i=0;i<points.size();i++) 
    if(glocal.Distance(points[i]) <= tol) {
      pointIds.push_back(ids[i]);
      if(pointIds.size()>=maxContacts) return;
    }
  return;

  /*
  //grid enumeration method
  GridSubdivision3D::Index imin,imax;
  pc.grid.PointToIndex(gbb.bmin,imin);
  pc.grid.PointToIndex(gbb.bmax,imax);
  int numCells = (imax[0]-imin[0]+1)*(imax[1]-imin[1]+1)*(imax[2]-imin[2]+1);
  if(numCells > (int)pc.points.size()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Testing all points\n");
    //test all points, linearly
    for(size_t i=0;i<pc.points.size();i++)
      if(glocal.Distance(pc.points[i]) <= tol) {
	pointIds.push_back(int(i));
	if(pointIds.size()>=maxContacts) return;
      }
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Testing points in BoxQuery\n");
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
      if(b.octree->IsLeaf(bnode)) {
        //collide the two points contained within
        vector<Vector3> apts,bpts;
        vector<int> aids,bids;
        a.octree->GetPoints(aindex,apts);
        b.octree->GetPoints(bindex,bpts);
        a.octree->GetPointIDs(aindex,aids);
        b.octree->GetPointIDs(bindex,bids);
        for(size_t i=0;i<apts.size();i++) {
          for(size_t j=0;j<bpts.size();j++) {
            if(apts[i].distanceSquared(bpts[j]) <= Sqr(margin)) {
              acollisions.push_back(aids[i]);
              acollisions.push_back(bids[i]);
              if(acollisions.size() >= maxContacts) return false;
            }
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

} //namespace Geometry
