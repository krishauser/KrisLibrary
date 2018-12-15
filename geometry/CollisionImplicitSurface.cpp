#include "CollisionImplicitSurface.h"
#include "CollisionPointCloud.h"
#include <spline/TimeSegmentation.h>
#include <structs/Heap.h>
#include <KrisLibrary/Logger.h>
#include <Timer.h>

//switch to brute force when # points drops below 1000
#define DEBUG_DISTANCE_CHECKING 0
//#define DEBUG_DISTANCE_CHECKING 1
#define BRUTE_FORCE_DISTANCE_CHECKING_NPOINTS 1000
#define BOUNDING_VOLUME_FUZZ 1e-2

//only do brute force checking
//#define DEBUG_DISTANCE_CHECKING 0
//#define BRUTE_FORCE_DISTANCE_CHECKING_NPOINTS 100000000

using namespace std;

namespace Geometry {

void GetMinMax(const Meshing::VolumeGrid* mingrid,const Meshing::VolumeGrid* maxgrid,const AABB3D& bb,Real& vmin,Real& vmax)
{
  vmin = Inf;
  vmax = -Inf;
  IntTriple imin,imax;
  mingrid->GetIndexRange(bb,imin,imax);
  if(imax.a < 0) imax.a = 0;
  if(imin.a >= mingrid->value.m) imin.a = mingrid->value.m-1;
  if(imax.b < 0) imax.b = 0;
  if(imin.b >= mingrid->value.n) imin.b = mingrid->value.n-1;
  if(imax.c < 0) imax.c = 0;
  if(imin.c >= mingrid->value.p) imin.c = mingrid->value.p-1;
  for(int i=max(imin.a,0);i<=min(imax.a,mingrid->value.m-1);i++) {
    for(int j=max(imin.b,0);j<=min(imax.b,mingrid->value.n-1);j++) {
      for(int k=max(imin.c,0);k<=min(imax.c,mingrid->value.p-1);k++) {
        vmin = Min(vmin,mingrid->value(i,j,k));
        vmax = Max(vmax,maxgrid->value(i,j,k));
      }
    }
  }
}


CollisionImplicitSurface::CollisionImplicitSurface()
{
  currentTransform.setIdentity();
}

CollisionImplicitSurface::CollisionImplicitSurface(const Meshing::VolumeGrid& vg)
:baseGrid(vg)
{
  currentTransform.setIdentity();
  InitCollisions();
}

CollisionImplicitSurface::CollisionImplicitSurface(const CollisionImplicitSurface& vg)
:baseGrid(vg.baseGrid),currentTransform(vg.currentTransform),minHierarchy(vg.minHierarchy),maxHierarchy(vg.maxHierarchy),resolutionMap(vg.resolutionMap)
{}

void CollisionImplicitSurface::InitCollisions()
{
  Vector3 dims = baseGrid.bb.bmax-baseGrid.bb.bmin;
  Real maxdim = Max(dims.x,dims.y,dims.z);
  Vector3 res = dims;
  res.x /= (baseGrid.value.m);
  res.y /= (baseGrid.value.n);
  res.z /= (baseGrid.value.p);
  Real h = Min(res.x,res.y,res.z);
  Assert(h > 0);
  h = h*2;
  Real h0 = h;
  //first resize
  resolutionMap.resize(0);
  while(int(maxdim / h) >= 2) {
    resolutionMap.push_back(h);
    h = h*2;
  }
  /*
  //debugging
  printf("CollisionImplicitSurface: %d resolutions ",(int)resolutionMap.size());
  for(size_t i=0;i<resolutionMap.size();i++)
    printf("%g ",resolutionMap[i]);
  printf("\n");
  getchar();
  */
  resolutionMap.push_back(Inf);
  minHierarchy.resize(resolutionMap.size()-1);
  maxHierarchy.resize(resolutionMap.size()-1);
  for(size_t i=0;i+1<resolutionMap.size();i++) {
    Real res = resolutionMap[i];
    minHierarchy[i].bb = baseGrid.bb;
    minHierarchy[i].ResizeByResolution(Vector3(res));
    maxHierarchy[i].bb = baseGrid.bb;
    maxHierarchy[i].ResizeByResolution(Vector3(res));
  }
  Meshing::VolumeGrid *minprev = &baseGrid, *maxprev = &baseGrid;
  for(size_t i=0;i<minHierarchy.size();i++) {
    Meshing::VolumeGrid::iterator itmin = minHierarchy[i].getIterator();
    Meshing::VolumeGrid::iterator itmax = maxHierarchy[i].getIterator();
    AABB3D bb;
    Real vmin,vmax;
    while(!itmin.isDone()) {
      itmin.getCell(bb);
      GetMinMax(minprev,maxprev,bb,vmin,vmax);
      *itmin = vmin;
      *itmax = vmax;
      ++itmin;
      ++itmax;
    }
    minprev = &minHierarchy[i];
    maxprev = &maxHierarchy[i];
  }
}


void CollisionImplicitSurface::DistanceRangeLocal(const AABB3D& bb,Real& vmin,Real& vmax) const
{
  Vector3 size = bb.bmax-bb.bmin;
  Real res = Max(size.x,size.y,size.z);
  const Meshing::VolumeGrid *chosenMin = &baseGrid, *chosenMax = &baseGrid;
  IntTriple imin,imax;
  if(resolutionMap.empty() || res < resolutionMap[0]) {
    //just look it up in the base grid
  }
  else {
    int index=Spline::TimeSegmentation::Map(resolutionMap,res);
    if(index < 0 || index >= (int)minHierarchy.size()) {
      printf("Uh... can't look up resolution? %g, result %d\n",res,index);
      for(size_t i=0;i<resolutionMap.size();i++)
        printf("%g ",resolutionMap[i]);
      printf("\n");
    }
    Assert(index >= 0 && index < (int)minHierarchy.size());
    chosenMin = &minHierarchy[index];
    chosenMax = &maxHierarchy[index];
  }
  GetMinMax(chosenMin,chosenMax,bb,vmin,vmax);
}

Real Distance(const CollisionImplicitSurface& s,const Vector3& pt)
{
  Vector3 ptlocal;
  s.currentTransform.mulInverse(pt,ptlocal);
  Real sdf_value = s.baseGrid.TrilinearInterpolate(ptlocal);
  Vector3 pt_clamped;
  Real d_bb = s.baseGrid.bb.distance(ptlocal,pt_clamped);
  return sdf_value + d_bb;
}


Real Distance(const CollisionImplicitSurface& s,const Vector3& pt,Vector3& surfacePt,Vector3& direction)
{
  Vector3 ptlocal;
  s.currentTransform.mulInverse(pt,ptlocal);
  Real sdf_value = s.baseGrid.TrilinearInterpolate(ptlocal);
  Vector3 pt_clamped;
  Real d_bb = s.baseGrid.bb.distance(ptlocal,pt_clamped);
 
  s.baseGrid.Gradient(pt_clamped,direction);
  //cout<<"Gradient is "<<direction<<endl;
  direction.inplaceNormalize();
  surfacePt = pt_clamped - direction*sdf_value;
  if(d_bb > 0) {
    direction = surfacePt - ptlocal;
    direction.inplaceNormalize();
    //cout<<"External, changing direction to "<<direction<<endl;
  }
  else
    direction.inplaceNegative();
  surfacePt = s.currentTransform*surfacePt;
  direction = s.currentTransform.R*direction;
  return sdf_value + d_bb;
}

Real Distance(const CollisionImplicitSurface& grid,const GeometricPrimitive3D& a,Vector3& gridclosest,Vector3& geomclosest,Vector3& direction)
{
  if(a.type == GeometricPrimitive3D::Point) {
    const Vector3& pt = *AnyCast_Raw<Vector3>(&a.data);
    geomclosest = pt;
    Real d= Distance(grid,pt,gridclosest,direction);
    return d;
  }
  else if(a.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D* s=AnyCast_Raw<Sphere3D>(&a.data);
    Real d = Distance(grid,s->center,gridclosest,direction);
    geomclosest = s->center - Min(s->radius,Max(d,-s->radius))*direction;
    return d - s->radius;
  }
  else {
    FatalError("Can't collide an implicit surface and a non-sphere primitive yet\n");
    return 0;
  }
}

bool Collides(const CollisionImplicitSurface& s,const CollisionPointCloud& pc,Real margin,vector<int>& collidingPoints,size_t maxContacts)
{
  //first do a quick reject test
  Timer timer;
  Box3D pcbb;
  GetBB(pc,pcbb);
  Box3D sbb;
  sbb.setTransformed(s.baseGrid.bb,s.currentTransform);
  Box3D sbbexpanded = sbb;
  sbbexpanded.dims += Vector3(margin*2.0);
  sbbexpanded.origin -= margin*(sbb.xbasis+sbb.ybasis+sbb.zbasis);
  //quick reject test
  if(!pcbb.intersectsApprox(sbbexpanded)) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"0 contacts (quick reject) time "<<timer.ElapsedTime());
    return false;
  }
  RigidTransform Tw_pc;
  Tw_pc.setInverse(pc.currentTransform);
  Box3D sbb_pc;
  sbb_pc.setTransformed(sbbexpanded,Tw_pc);
  
  //octree-vs b's bounding box method (not terribly smart, could be
  //improved by descending bounding box hierarchies)
  vector<Vector3> apoints;
  vector<int> aids;
  pc.octree->BoxQuery(sbb_pc,apoints,aids);
  RigidTransform Tident; Tident.setIdentity();
  //test all points, linearly
  for(size_t i=0;i<apoints.size();i++) {
    Vector3 p_w = pc.currentTransform*apoints[i];
    if(Distance(s,p_w) <= margin) {
      collidingPoints.push_back(aids[i]);
      if(collidingPoints.size() >= maxContacts) {
        LOG4CXX_INFO(KrisLibrary::logger(),"PointCloud-ImplicitSurface "<<maxContacts<<" contacts time "<<timer.ElapsedTime());

        //LOG4CXX_INFO(KrisLibrary::logger(),"Collision in time "<<timer.ElapsedTime());
        return true;
      }
    }
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"PointCloud-ImplicitSurface "<<maxContacts<<" contacts time "<<timer.ElapsedTime());     
  // LOG4CXX_INFO(KrisLibrary::logger(),"No collision in time "<<timer.ElapsedTime());
  return !collidingPoints.empty();

  /*
  AABB3D baabb_a;
  sbb_pc.getAABB(baabb_a);
  baabb_a.setIntersection(a.bblocal);

  //grid method
  GridSubdivision::Index imin,imax;
  a.grid.PointToIndex(Vector(3,baabb_a.bmin),imin);
  a.grid.PointToIndex(Vector(3,baabb_a.bmax),imax);
  int numCells = (imax[0]-imin[0]+1)*(imax[1]-imin[1]+1)*(imax[2]-imin[2]+1);
  Timer timer;
  if(numCells >(int) a.points.size()) {
    RigidTransform Tident; Tident.setIdentity();
    //test all points, linearly
    for(size_t i=0;i<a.points.size();i++) {
      Vector3 p_w = a.currentTransform*a.points[i];
      Tident.t = p_w;
      vector<int> temp;
      if(Collides(point_primitive,Tident,margin,b,temp,elements1,maxContacts)) {
        elements2.push_back(i);
        if(elements2.size() >= maxContacts) {
          LOG4CXX_INFO(KrisLibrary::logger(),"Collision in time "<<timer.ElapsedTime());
          return true;
        }
      }
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"No collision in time "<<timer.ElapsedTime());
    return false;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Box testing\n");
    gWithinDistanceMargin = margin;
    gWithinDistancePC = &a;
    gWithinDistanceGeom = &b;
    gWithinDistanceElements1 = &elements1;
    gWithinDistanceElements2 = &elements2;
    gWithinDistanceMaxContacts = maxContacts;
    bool collisionFree = a.grid.IndexQuery(imin,imax,withinDistance_PC_AnyGeom);
    if(collisionFree) LOG4CXX_INFO(KrisLibrary::logger(),"No collision in time "<<timer.ElapsedTime());
    else LOG4CXX_INFO(KrisLibrary::logger(),"Collision in time "<<timer.ElapsedTime());
    return !collisionFree;
  }
  */
}

Real DistanceLowerBound(const CollisionImplicitSurface& s,const CollisionPointCloud& pc,int nindex,const Matrix4& Mpc_s)
{
  const OctreeNode& n = pc.octree->Node(nindex);
  if(n.bb.bmin.x > n.bb.bmax.x) {
    //empty
    return Inf;
  }
  //method 1: use the multiresolution hierarchy
  /*
  AABB3D orientedBB;
  orientedBB.setTransform(n.bb,Mpc_s);
  s.DistanceRangeLocal(orientedBB,dmin,dmax);
  return dmin+orientedBB.distance(s.baseGrid.bb) - BOUNDING_VOLUME_FUZZ;
  */
  //method2: put everything in a sphere, assume the value of the baseGrid is an SDF
  Vector3 c=(n.bb.bmin+n.bb.bmax)*0.5;
  Vector3 dims = n.bb.bmax-n.bb.bmin;
  Real rad = dims.norm()*0.5;
  /*
  const Vector3& c = pc.octree->Ball(nindex).center;
  Real rad = pc.octree->Ball(nindex).radius;
  */

  Vector3 clocal;
  Mpc_s.mulPoint(c,clocal);
  Real dc = s.baseGrid.TrilinearInterpolate(clocal);
  Real d_bb = s.baseGrid.bb.distance(clocal);
  return d_bb + dc - rad - BOUNDING_VOLUME_FUZZ;
}

Real Distance(const CollisionImplicitSurface& s,const CollisionPointCloud& pc,int& closestPoint,Real upperBound)
{
  //Timer timer;

  assert(pc.octree != NULL);
  RigidTransform Tpc_s;
  Tpc_s.mulInverseA(s.currentTransform,pc.currentTransform);
  Matrix4 Mpc_s(Tpc_s);
  AABB3D bb;
  //Real dmin,dmax;
  Heap<int,Real> heap;

  int bruteForceClosestPoint = -1;
  Real bruteForceDmin = upperBound;
  if(DEBUG_DISTANCE_CHECKING || pc.points.size() < BRUTE_FORCE_DISTANCE_CHECKING_NPOINTS) {
    bruteForceClosestPoint = -1;
    for(size_t i=0;i<pc.points.size();i++) {
      Vector3 ptlocal;
      Tpc_s.mul(pc.points[i],ptlocal);
      Vector3 pt_clamped;
      Real sdf_value = s.baseGrid.TrilinearInterpolate(ptlocal);
      if(sdf_value < bruteForceDmin) {
        Real d_bb = s.baseGrid.bb.distance(ptlocal,pt_clamped);
        if(sdf_value + d_bb < bruteForceDmin) {
          bruteForceClosestPoint = (int)i;
          bruteForceDmin = sdf_value + d_bb;
        }
      }
    }
    //printf("Naive distance checking result %g, %d, time %g\n",bruteForceDmin,bruteForceClosestPoint,timer.ElapsedTime());
    //timer.Reset();
    if(!DEBUG_DISTANCE_CHECKING) {
      closestPoint = bruteForceClosestPoint;
      return bruteForceDmin;
    }

    /*
    //MORE DEBUGGING
    const OctreePointSet* cpset = pc.octree;
    OctreePointSet* pset = (OctreePointSet*)cpset;
    OctreeNode* nclosest = pset->Lookup(pc.points[bruteForceClosestPoint]);
    Assert(nclosest != NULL);
    cout<<"Point is "<<pc.points[bruteForceClosestPoint]<<endl;
    cout<<"Node bounding box is "<<nclosest->bb<<endl;
    Real d = DistanceLowerBound(s,*nclosest,Mpc_s);
    printf("Distance lower bound to octree box containing naive closest point: %g vs %g\n",bruteForceDmin,d);
    nclosest = pset->Lookup(pset->Node(0),pc.points[bruteForceClosestPoint],2);
    Assert(nclosest != NULL);
    d = DistanceLowerBound(s,*nclosest,Mpc_s);
    printf("Distance lower bound to octree box at depth 2 containing naive closest point: %g\n",d);
    nclosest = pset->Lookup(pset->Node(0),pc.points[bruteForceClosestPoint],3);
    Assert(nclosest != NULL);
    d = DistanceLowerBound(s,*nclosest,Mpc_s);
    printf("Distance lower bound to octree box at depth 3 containing naive closest point: %g\n",d);
    nclosest = pset->Lookup(pset->Node(0),pc.points[bruteForceClosestPoint],4);
    Assert(nclosest != NULL);
    d = DistanceLowerBound(s,*nclosest,Mpc_s);
    printf("Distance lower bound to octree box at depth 4 containing naive closest point: %g\n",d);
    getchar();
    */
  }

  Real d = DistanceLowerBound(s,pc,0,Mpc_s);
  if(d < upperBound) 
    heap.push(0,d);
  
  int numPointsChecked = 0;
  int numBBsChecked = 1;
  int numBBsPruned = 0;
  Real mindist = upperBound;
  closestPoint = -1;
  d = Distance(s,pc.currentTransform*pc.points[0]);
  if(d < mindist) {
    mindist = d;
    closestPoint = 0;
  }
  vector<int> pointids;
  while(!heap.empty()) {
    const OctreeNode& n = pc.octree->Node(heap.top());
    if(heap.topPriority() >= mindist) {
      heap.pop();
      numBBsPruned += 1;
      continue;
    }
    heap.pop();
    if(pc.octree->IsLeaf(n)) {
      //look through points
      pc.octree->GetPointIDs(n,pointids);
      for(size_t i=0;i<pointids.size();i++) {
        int id=pointids[i];
        //Real d = Distance(s,pc.currentTransform*pc.points[id]);
        Vector3 ptlocal;
        Tpc_s.mul(pc.points[id],ptlocal);
        Vector3 pt_clamped;
        Real sdf_value = s.baseGrid.TrilinearInterpolate(ptlocal);
        if(sdf_value < mindist) {
          Real d_bb = s.baseGrid.bb.distance(ptlocal,pt_clamped);
          Real d = d_bb + sdf_value;
          if(d < mindist) {
            //debugging
            Assert(FuzzyEquals(d,Distance(s,pc.currentTransform*pc.points[id])));
            mindist = d;
            closestPoint = id;
          }
        }
        numPointsChecked ++;
      }
    }
    else {
      numBBsChecked += 8;
      for(int c=0;c<8;c++) {
        Real d = DistanceLowerBound(s,pc,n.childIndices[c],Mpc_s);
        if(d < mindist) {
          heap.push(n.childIndices[c],d);
        }
        else {
          numBBsPruned += 1;
        }
      }
    }
  }
  if(DEBUG_DISTANCE_CHECKING) {
    if(bruteForceDmin != mindist && bruteForceDmin < upperBound) {
      printf("CollisionImplicitSurface vs CollisionPointCloud Distance error");
      printf("   Discrepancy between brute force and expedited checking: %g vs %g\n",bruteForceDmin,mindist);
      printf("   Points %d vs %d\n",bruteForceClosestPoint,closestPoint);
      //Abort();
    }
  }
  //printf("Expedited distance checking result %g, %d, time %g\n",mindist,closestPoint,timer.ElapsedTime());
  //printf("Checked %d / %d points, %d bounding boxes (%d pruned)\n",numPointsChecked,(int)pc.points.size(),numBBsChecked,numBBsPruned);
  return mindist;
}

} //namespace Geometry