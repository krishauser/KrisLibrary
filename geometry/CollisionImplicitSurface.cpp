#include "CollisionImplicitSurface.h"
#include "CollisionPointCloud.h"
#include <meshing/Voxelize.h>
#include <spline/TimeSegmentation.h>
#include <structs/Heap.h>
#include <KrisLibrary/Logger.h>
#include <Timer.h>
#include <tuple>

DECLARE_LOGGER(Geometry)

//switch to brute force when # points drops below 1000
#define DEBUG_DISTANCE_CHECKING 0
//#define DEBUG_DISTANCE_CHECKING 1
#define BRUTE_FORCE_DISTANCE_CHECKING_NPOINTS 1000
//The bounding volume method is a bit inaccurate due to the discretization of the implicit surface. 
//This value allows a little bit of slop in the pruning step so that more points are considered.
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
  if(!mingrid->GetIndexRangeClamped(bb,imin,imax)) return;
  for(int i=imin.a;i<=imax.a;i++) {
    for(int j=imin.b;j<=imax.b;j++) {
      for(int k=imin.c;k<=imax.c;k++) {
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

Real DistanceLocal(const Meshing::VolumeGrid& grid,const Vector3& pt,Vector3* surfacePt=NULL,Vector3* direction=NULL)
{
  Vector3 pt_clamped;
  Real d_bb = grid.bb.distance(pt,pt_clamped);
  Real sdf_value = grid.TrilinearInterpolate(pt);
  if(surfacePt || direction) {
    grid.Gradient(pt_clamped,*direction);
    direction->inplaceNormalize();
    *surfacePt = pt_clamped - (*direction)*sdf_value;
    if(d_bb > 0) {
      *direction = *surfacePt - pt;
      d_bb = direction->norm();
      *direction /= d_bb;
      return d_bb;
    }
    else
      direction->inplaceNegative();
  } 
  return d_bb + sdf_value;
}

Real Distance(const CollisionImplicitSurface& s,const Vector3& pt,Vector3& surfacePt,Vector3& direction)
{
  Vector3 ptlocal;
  s.currentTransform.mulInverse(pt,ptlocal);

  Real d=DistanceLocal(s.baseGrid,ptlocal,&surfacePt,&direction);
  surfacePt = s.currentTransform*surfacePt;
  direction = s.currentTransform.R*direction;
  return d;
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
    geomclosest = s->center + s->radius*direction;
    return d - s->radius;
  }
  else if(a.type == GeometricPrimitive3D::Segment) {
    const Segment3D* s=AnyCast_Raw<Segment3D>(&a.data);
    Segment3D slocal;
    grid.currentTransform.mulInverse(s->a,slocal.a);
    grid.currentTransform.mulInverse(s->b,slocal.b);
    Vector3 cellsize = grid.baseGrid.GetCellSize();
    Real cellbnd = Max(cellsize.x,cellsize.y,cellsize.z)*0.5;
    Segment3D sgrid;
    sgrid.a.x = (slocal.a.x - grid.baseGrid.bb.bmin.x) / cellsize.x;
    sgrid.a.y = (slocal.a.y - grid.baseGrid.bb.bmin.y) / cellsize.y;
    sgrid.a.z = (slocal.a.z - grid.baseGrid.bb.bmin.z) / cellsize.z;
    sgrid.b.x = (slocal.b.x - grid.baseGrid.bb.bmin.x) / cellsize.x;
    sgrid.b.y = (slocal.b.y - grid.baseGrid.bb.bmin.y) / cellsize.y;
    sgrid.b.z = (slocal.b.z - grid.baseGrid.bb.bmin.z) / cellsize.z;
    vector<Real> params;
    vector<IntTriple> cells;
    Meshing::GetSegmentCells(sgrid,cells,&params);
    cells.push_back(cells.back());
    vector<tuple<Real,IntTriple,Real> > candidates;
    for(size_t i=0;i<cells.size();i++) {
      IntTriple c = cells[i];
      if(c.a < 0) c.a=0; else if(c.a >= grid.baseGrid.value.m) c.a = grid.baseGrid.value.m-1;
      if(c.b < 0) c.b=0; else if(c.b >= grid.baseGrid.value.n) c.b = grid.baseGrid.value.n-1;
      if(c.c < 0) c.c=0; else if(c.c >= grid.baseGrid.value.p) c.c = grid.baseGrid.value.p-1;
      Real d=grid.baseGrid.value(c);
      if(candidates.empty() || get<1>(candidates.back()) != c) {
        candidates.push_back(make_tuple(d,c,params[i]));
        candidates.push_back(make_tuple(d,c,(params[i]+params[i+1])*0.5));
      }
      else if(!candidates.empty()) {
        ///same grid cell -- start in middle
        get<2>(candidates.back()) = 0.5*(get<2>(candidates.back())+params[i]);
      }
    }
    Assert(candidates.size() > 0);
    Real dmin=Inf;
    Real tmin=0;
    sort(candidates.begin(),candidates.end());
    Vector3 disp=slocal.b-slocal.a;
    for(size_t i=0;i<candidates.size();i++) {
      Real d = get<0>(candidates[i]);
      if(d - cellbnd >= dmin) break;
      Real t = get<2>(candidates[i]);
      Vector3 x = slocal.a + t*disp;
      Vector3 xclamped;
      Real d_bb = grid.baseGrid.bb.distance(x,xclamped);
      if(d_bb >= dmin) continue;
      if(d_bb + d - cellbnd >= dmin) continue;
      Real sdf_value = grid.baseGrid.TrilinearInterpolate(xclamped);
      if(d_bb + sdf_value < dmin) {
        tmin = t;
        dmin = d_bb + sdf_value;
      }
    }
    geomclosest = slocal.a + tmin*disp;
    Vector3 pt_clamped;
    Real d_bb = grid.baseGrid.bb.distance(geomclosest,pt_clamped);
    Real sdf_value = grid.baseGrid.TrilinearInterpolate(pt_clamped);
    grid.baseGrid.Gradient(pt_clamped,direction);
    direction.inplaceNormalize();
    gridclosest = pt_clamped - direction*sdf_value;
    if(d_bb > 0) {
      direction = gridclosest - geomclosest;
      direction.inplaceNormalize();
    }
    else
      direction.inplaceNegative();

    geomclosest = grid.currentTransform*geomclosest;
    gridclosest = grid.currentTransform*gridclosest;
    direction = grid.currentTransform.R*direction;
    return geomclosest.distance(gridclosest);
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Can't collide an implicit surface and a non-sphere primitive yet");
    return 0;
  }
}

bool Collides(const CollisionImplicitSurface& grid,const GeometricPrimitive3D& a,Real margin,Vector3& gridclosest)
{
  if(a.type == GeometricPrimitive3D::Point || a.type == GeometricPrimitive3D::Sphere) {
    Vector3 temp,temp2;
    return Distance(grid,a,gridclosest,temp,temp2) <= margin;
  }
  else if(a.type == GeometricPrimitive3D::Segment) {
    const Segment3D* s=AnyCast_Raw<Segment3D>(&a.data);
    Segment3D slocal;
    grid.currentTransform.mulInverse(s->a,slocal.a);
    grid.currentTransform.mulInverse(s->b,slocal.b);
    if(slocal.distance(grid.baseGrid.bb) > margin) {
      return false;
    }

    //shrink the range considered by examining the endpoint distances
    Real da = DistanceLocal(grid.baseGrid,slocal.a);
    Real db = DistanceLocal(grid.baseGrid,slocal.b);
    Real t1,t2;
    Real slength = slocal.a.distance(slocal.b);
    if(da <= margin) t1=0;
    else {
      if(da - slength > margin) {
        return false;
      }
      t1 = (da - margin)/slength;
    }
    if(db <= margin) t2=1;
    else {
      if(db - slength > margin) {
        return false;
      }
      t2 = 1 - (db - margin)/slength;
    }
    Vector3 a,b;
    a = slocal.a + t1*(slocal.b-slocal.a);
    b = slocal.a + t2*(slocal.b-slocal.a);
    slocal.a = a;
    slocal.b = b;

    Vector3 cellsize = grid.baseGrid.GetCellSize();
    Real cellbnd = Max(cellsize.x,cellsize.y,cellsize.z)*0.5;
    Segment3D sgrid;
    sgrid.a.x = (slocal.a.x - grid.baseGrid.bb.bmin.x) / cellsize.x;
    sgrid.a.y = (slocal.a.y - grid.baseGrid.bb.bmin.y) / cellsize.y;
    sgrid.a.z = (slocal.a.z - grid.baseGrid.bb.bmin.z) / cellsize.z;
    sgrid.b.x = (slocal.b.x - grid.baseGrid.bb.bmin.x) / cellsize.x;
    sgrid.b.y = (slocal.b.y - grid.baseGrid.bb.bmin.y) / cellsize.y;
    sgrid.b.z = (slocal.b.z - grid.baseGrid.bb.bmin.z) / cellsize.z;
    vector<Real> params;
    vector<IntTriple> cells;
    Meshing::GetSegmentCells(sgrid,cells,&params);
    cells.push_back(cells.back());
    vector<tuple<Real,IntTriple,Real> > candidates;
    for(size_t i=0;i<cells.size();i++) {
      IntTriple c = cells[i];
      if(c.a < 0) c.a=0; else if(c.a >= grid.baseGrid.value.m) c.a = grid.baseGrid.value.m-1;
      if(c.b < 0) c.b=0; else if(c.b >= grid.baseGrid.value.n) c.b = grid.baseGrid.value.n-1;
      if(c.c < 0) c.c=0; else if(c.c >= grid.baseGrid.value.p) c.c = grid.baseGrid.value.p-1;
      Real d=grid.baseGrid.value(c);
      if(d - cellbnd >= margin) continue;
      if(candidates.empty() || get<1>(candidates.back()) != c) {
        candidates.push_back(make_tuple(d,c,params[i]));
        candidates.push_back(make_tuple(d,c,(params[i]+params[i+1])*0.5));
      }
      else if(!candidates.empty()) {
        ///same grid cell -- start in middle
        get<2>(candidates.back()) = 0.5*(get<2>(candidates.back())+params[i]);
      }
    }
    if(candidates.empty()) return false;
    Real dmin=Max(margin+1e-7,1e-5);
    Real tmin=0;
    sort(candidates.begin(),candidates.end());
    Vector3 disp=slocal.b-slocal.a;
    for(size_t i=0;i<candidates.size();i++) {
      Real d = get<0>(candidates[i]);
      if(d - cellbnd >= dmin) break;
      Real t = get<2>(candidates[i]);
      Vector3 x = slocal.a + t*disp;
      Vector3 xclamped;
      Real d_bb = grid.baseGrid.bb.distance(x,xclamped);
      if(d_bb >= dmin) continue;
      if(d_bb + d - cellbnd >= dmin) continue;
      Real sdf_value = grid.baseGrid.TrilinearInterpolate(xclamped);
      if(d_bb + sdf_value < dmin) {
        tmin = t;
        dmin = d_bb + sdf_value;
      }
    }
    Vector3 geomclosest = slocal.a + tmin*disp;
    Vector3 direction;
    DistanceLocal(grid.baseGrid,geomclosest,&gridclosest,&direction);

    gridclosest = grid.currentTransform*gridclosest;
    return (dmin < Max(margin+1e-6,1e-5));
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Can't collide an implicit surface and a non-sphere primitive yet");
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
  if(pc.radiusIndex >= 0) {
    sbbexpanded.dims += Vector3(pc.maxRadius*2.0);
    sbbexpanded.origin -= pc.maxRadius*(sbb.xbasis+sbb.ybasis+sbb.zbasis);
  }
  //quick reject test
  if(!pcbb.intersectsApprox(sbbexpanded)) {
    //LOG4CXX_INFO(GET_LOGGER(Geometry),"0 contacts (quick reject) time "<<timer.ElapsedTime());
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
    Real margini = margin;
    if(pc.radiusIndex >= 0) margini += pc.properties[aids[i]][pc.radiusIndex];
    if(Distance(s,p_w) <= margini) {
      collidingPoints.push_back(aids[i]);
      if(collidingPoints.size() >= maxContacts) {
        LOG4CXX_DEBUG(GET_LOGGER(Geometry),"PointCloud-ImplicitSurface "<<maxContacts<<" contacts time "<<timer.ElapsedTime());

        //LOG4CXX_INFO(GET_LOGGER(Geometry),"Collision in time "<<timer.ElapsedTime());
        return true;
      }
    }
  }
  LOG4CXX_DEBUG(GET_LOGGER(Geometry),"PointCloud-ImplicitSurface "<<maxContacts<<" contacts time "<<timer.ElapsedTime());     
  // LOG4CXX_INFO(GET_LOGGER(Geometry),"No collision in time "<<timer.ElapsedTime());
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
          LOG4CXX_INFO(GET_LOGGER(Geometry),"Collision in time "<<timer.ElapsedTime());
          return true;
        }
      }
    }
    LOG4CXX_INFO(GET_LOGGER(Geometry),"No collision in time "<<timer.ElapsedTime());
    return false;
  }
  else {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"Box testing\n");
    gWithinDistanceMargin = margin;
    gWithinDistancePC = &a;
    gWithinDistanceGeom = &b;
    gWithinDistanceElements1 = &elements1;
    gWithinDistanceElements2 = &elements2;
    gWithinDistanceMaxContacts = maxContacts;
    bool collisionFree = a.grid.IndexQuery(imin,imax,withinDistance_PC_AnyGeom);
    if(collisionFree) LOG4CXX_INFO(GET_LOGGER(Geometry),"No collision in time "<<timer.ElapsedTime());
    else LOG4CXX_INFO(GET_LOGGER(Geometry),"Collision in time "<<timer.ElapsedTime());
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
  /*
  Vector3 c=(n.bb.bmin+n.bb.bmax)*0.5;
  Vector3 dims = n.bb.bmax-n.bb.bmin;
  Real rad = dims.norm()*0.5;
  */

  const Sphere3D& spherebound = pc.octree->Ball(nindex);
  /*
  if(spherebound.radius < rad) {
    c = spherebound.center;
    rad = spherebound.radius;
  }
  */
  const Vector3& c = spherebound.center;
  Real rad = spherebound.radius;

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
      if(pc.radiusIndex >= 0)
        sdf_value -= pc.properties[i][pc.radiusIndex];
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
  if(pc.radiusIndex >= 0)
    d -= pc.properties[0][pc.radiusIndex];
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
        if(pc.radiusIndex >= 0)
          sdf_value -= pc.properties[id][pc.radiusIndex];
        if(sdf_value < mindist) {
          Real d_bb = s.baseGrid.bb.distance(ptlocal,pt_clamped);
          Real d = d_bb + sdf_value;
          if(d < mindist) {
            //debugging
            if(DEBUG_DISTANCE_CHECKING && pc.radiusIndex < 0)
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
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"CollisionImplicitSurface vs CollisionPointCloud Distance error");
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"   Discrepancy between brute force and expedited checking: "<<bruteForceDmin<<" vs "<<mindist);
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"   Points "<<bruteForceClosestPoint<<" vs "<<closestPoint);
      //Abort();
    }
  }
  //printf("Expedited distance checking result %g, %d, time %g\n",mindist,closestPoint,timer.ElapsedTime());
  //printf("Checked %d / %d points, %d bounding boxes (%d pruned)\n",numPointsChecked,(int)pc.points.size(),numBBsChecked,numBBsPruned);
  return mindist;
}

Real RayGridCellIntersect(const Ray3D& ray,const Meshing::VolumeGrid& grid,const IntTriple& c,Real umin,Real umax,Real levelSet,Real vtol=1e-3)
{
  Real v000 = grid.value(c.a,c.b,c.c);
  Real v001 = grid.value(c.a,c.b,c.c+1);
  Real v010 = grid.value(c.a,c.b+1,c.c);
  Real v011 = grid.value(c.a,c.b+1,c.c+1);
  Real v100 = grid.value(c.a+1,c.b,c.c);
  Real v101 = grid.value(c.a+1,c.b,c.c+1);
  Real v110 = grid.value(c.a+1,c.b+1,c.c);
  Real v111 = grid.value(c.a+1,c.b+1,c.c+1);
  int numIn=0;
  if(v000 < levelSet) numIn++;
  if(v001 < levelSet) numIn++;
  if(v010 < levelSet) numIn++;
  if(v011 < levelSet) numIn++;
  if(v100 < levelSet) numIn++;
  if(v101 < levelSet) numIn++;
  if(v110 < levelSet) numIn++;
  if(v111 < levelSet) numIn++;
  if(numIn == 0) return -1;
  if(numIn == 8) //popped into interior?
    return umin;
  //use secant method to find point at which levelset is intersected
  Real vmin = grid.TrilinearInterpolate(ray.source+umin*ray.direction);
  Real vmax = grid.TrilinearInterpolate(ray.source+umax*ray.direction);
  if(FuzzyEquals(vmin,levelSet,vtol)) return umin;
  if(FuzzyEquals(vmax,levelSet,vtol)) return umax;
  if((vmin < levelSet) == (vmax < levelSet)) return -1;
  //printf("Beginning secant method at u in range [%f,%f], v in range [%f,%f]\n",umin,umax,vmin,vmax);
  int numIters = 0;
  while(umax > umin + 1e-4) {
    numIters += 1;
    if(numIters > 1000) {
      printf("RayGridCellIntersect: Uh... secant method taking forever?\n");
      break;
    }
    //v(s) = vmin + s*(vmax-vmin)
    //v(s) = levelSet with s = -(vmin - levelSet)/(vmax-vmin)
    Real s = -(vmin - levelSet) / (vmax-vmin);
    Real umid = umin + s*(umax-umin);
    //printf("Level set at s=%f, umid = %f\n",s,umid);
    Assert(umid > umin && umid < umax);
    Real vmid = grid.TrilinearInterpolate(ray.source+umid*ray.direction);
    if(FuzzyEquals(vmid,levelSet,vtol)) return umid;
    if((vmin < levelSet) == (vmid < levelSet)) {
      vmin = vmid;
      umin = umid;
    }
    else {
      vmax = vmid;
      umax = umid;
    }
  }
  return -1;
}

///Returns the distance to the closest point on the implicit surface defined at the given level set.
///The algorithm marches along cells intersected by the ray until a zero-crossing is met (O(n))
///where n is the resolution of the grid.
Real RayCast(const Meshing::VolumeGrid& grid,const Ray3D& ray,Real levelSet,Real tmax)
{
  Real umin=0,umax=tmax;
  AABB3D center_bb = grid.bb;
  Vector3 celldims = grid.GetCellSize();
  //make correction for grid cell centers
  center_bb.bmin += celldims*0.5;
  center_bb.bmax -= celldims*0.5;
  if(!ray.intersects(center_bb,umin,umax)) {
    return tmax;
  }
  if(umin >= tmax) return tmax;
  if(umax > tmax) umax = tmax;
  Segment3D overlap;
  overlap.a = ray.source + umin*ray.direction;
  overlap.b = ray.source + umax*ray.direction;
  if(grid.TrilinearInterpolate(overlap.a) < levelSet) return 0; //already inside
  vector<IntTriple> cells;
  vector<Real> params;
  //grid values are interpreted at their centers
  Real segscale = (umax-umin);
  Meshing::GetSegmentCells(overlap,grid.value.m-1,grid.value.n-1,grid.value.p-1,center_bb,cells,&params);
  for(size_t i=0;i<cells.size();i++) {
    //determine whether there's a possibility with hitting a levelset value in the cell whose lower corner is value(c)
    Real res = RayGridCellIntersect(ray,grid,cells[i],umin+params[i]*segscale,umin+params[i+1]*segscale,levelSet);
    if(res >= 0) return res;
  }
  return tmax;
}

///Returns the distance to the closest point on the implicit surface defined at the given level set.
///The algorithm uses the collision hierarchy (O(log n) where n is the resolution of the grid.
Real RayCast(const CollisionImplicitSurface& s,const Ray3D& rayWorld,Real levelSet,Real tmax)
{
  Ray3D ray;
  s.currentTransform.mulInverse(rayWorld.source,ray.source);
  s.currentTransform.R.mulTranspose(rayWorld.direction,ray.direction);
  if(s.resolutionMap.empty())
    return RayCast(s.baseGrid,ray,levelSet,tmax);
  return RayCast(s.baseGrid,ray,levelSet,tmax);

  //TODO: make this experimental code work

  Real umin=0,umax=tmax;
  AABB3D center_bb = s.baseGrid.bb;
  Vector3 celldims = s.baseGrid.GetCellSize();
  //make correction for grid cell centers
  center_bb.bmin += celldims*0.5;
  center_bb.bmax -= celldims*0.5;
  if(!ray.intersects(center_bb,umin,umax)) return tmax;

  vector<IntTriple> lastLevelCells;
  vector<IntTriple> temp;
  AABB3D cell;
  vector<pair<Real,IntTriple> > intersections;
  for(size_t res=0;res<s.resolutionMap.size();res++) {
    size_t level = s.resolutionMap.size()-res-1;
    const Meshing::VolumeGrid& smin = s.minHierarchy[level];
    const Meshing::VolumeGrid& smax = s.maxHierarchy[level];
    if(res==0) { //top level, do initialization
      Segment3D overlap;
      overlap.a = ray.source + umin*ray.direction;
      overlap.b = ray.source + umax*ray.direction;
      if(s.baseGrid.TrilinearInterpolate(overlap.a) < levelSet) return 0; //already inside
      Meshing::GetSegmentCells(overlap,smin.value.m-1,smin.value.n-1,smin.value.p-1,smin.bb,temp);
      for(const auto& c:temp) {
        if(smin.value(c) <= levelSet && smax.value(c) >= levelSet)
          lastLevelCells.push_back(c);
      }
      printf("%d cells at top level (res %f)\n",lastLevelCells.size(),s.resolutionMap[level]);
    }
    else {
      const Meshing::VolumeGrid& snmin = s.minHierarchy[level+1];
      temp.resize(0);
      for(const auto& c:lastLevelCells) {
        snmin.GetCell(c,cell);
        IntTriple imin,imax;
        if(!smin.GetIndexRangeClamped(cell,imin,imax)) continue;
        intersections.resize(0);
        for(int i=imin.a;i<=imax.a;i++) {
          for(int j=imin.b;j<=imax.b;j++) {
            for(int k=imin.c;k<=imax.c;k++) {
              IntTriple c2(i,j,k);
              if(smin.value(c2) <= levelSet && smax.value(c2) >= levelSet) {
                Vector3 cc;
                smin.GetCellCenter(c2.a,c2.b,c2.c,cc);
                Real u = ray.closestPointParameter(cc);
                intersections.push_back(make_pair(u,c2));
              }
            }
          }
        }
        //make sure these are in intersection order
        sort(intersections.begin(),intersections.end());
        for(const auto& i:intersections)
          temp.push_back(i.second);
      }
      lastLevelCells = temp;
      printf("%d cells at level res %f\n",lastLevelCells.size(),s.resolutionMap[level]);
    }
  }
  const Meshing::VolumeGrid& snmin = s.minHierarchy[0];
  for(const auto& c: lastLevelCells) {
    snmin.GetCell(c,cell);    
    IntTriple imin,imax;
    s.baseGrid.GetIndexRange(cell,imin,imax);
    Real dmin = Inf;
    for(int i=imin.a;i<=imax.a;i++) {
      for(int j=imin.b;j<=imax.b;j++) {
        for(int k=imin.c;k<=imax.c;k++) {
          IntTriple c2(i,j,k);
          AABB3D cell2;
          s.baseGrid.GetCell(c2,cell2);
          cell2.bmin += celldims*0.5;
          cell2.bmax += celldims*0.5;
          umin = 0;
          umax = tmax;
          if(!ray.intersects(cell2,umin,umax))
            continue;
          Real res = RayGridCellIntersect(ray,s.baseGrid,c2,umin,umax,levelSet,tmax);
          if(res >= 0) dmin = Min(dmin,res);
        }
      }
    }
    if(dmin < tmax) return dmin;
  }
  return tmax;
}

} //namespace Geometry