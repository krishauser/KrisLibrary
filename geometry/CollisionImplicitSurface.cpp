#include "CollisionImplicitSurface.h"
#include "CollisionPrimitive.h"
#include "CollisionPointCloud.h"
#include "CollisionMesh.h"
#include "Conversions.h"
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/meshing/MarchingCubes.h>
#include <KrisLibrary/spline/TimeSegmentation.h>
#include <KrisLibrary/structs/Heap.h>
#include <KrisLibrary/Logger.h>
#include <Timer.h>
#include <tuple>
#include <vector>
DECLARE_LOGGER(Geometry)

#define DEBUG_DISTANCE_CHECKING 0
//#define DEBUG_DISTANCE_CHECKING 1
//switch to brute force when # points drops below 1000
#define BRUTE_FORCE_DISTANCE_CHECKING_NPOINTS 1000
//The bounding volume method is a bit inaccurate due to the discretization of the implicit surface. 
//This value allows a little bit of slop in the pruning step so that more points are considered.
#define BOUNDING_VOLUME_FUZZ 1e-2

//only do brute force checking
//#define DEBUG_DISTANCE_CHECKING 0
//#define BRUTE_FORCE_DISTANCE_CHECKING_NPOINTS 100000000

using namespace std;

namespace Geometry {

typedef ContactsQueryResult::ContactPair ContactPair;

//defined in Conversions.cpp
void FitGridToBB(const AABB3D& bb,Meshing::VolumeGrid& grid,Real resolution,Real expansion=0.5);

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
  //Real h0 = h;
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


Real DistanceLocal(const CollisionImplicitSurface& s,const Triangle3D& tri,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance=Inf)
{
  vector<IntTriple> tcells;
  Vector3 cellSize = s.baseGrid.GetCellSize();
  AABB3D center_bb = s.baseGrid.bb;
  center_bb.bmin += cellSize * 0.5;
  center_bb.bmax -= cellSize * 0.5;
  Real cellRad = cellSize.norm() * 0.5;

  //quickly test whether a sphere containing the triangle is within the margin
  Vector3 tempPt,tempDir;
  Vector3 c = (tri.a+tri.b+tri.c) / 3.0;
  Real d = Geometry::DistanceLocal(s.baseGrid,c,&surfacePt,&direction);
  if(d > minDistance) {
    return minDistance;
  }
  triPt = c;
  minDistance = d;
  d = Geometry::DistanceLocal(s.baseGrid,tri.a,&tempPt,&tempDir);
  if(d < minDistance) {
    surfacePt = tempPt;
    triPt = tri.a;
    direction = tempDir;
    minDistance = d;
  }
  d = Geometry::DistanceLocal(s.baseGrid,tri.b,&tempPt,&tempDir);
  if(d < minDistance) {
    surfacePt = tempPt;
    triPt = tri.b;
    direction = tempDir;
    minDistance = d;
  }
  d = Geometry::DistanceLocal(s.baseGrid,tri.c,&tempPt,&tempDir);
  if(d < minDistance) {
    surfacePt = tempPt;
    triPt = tri.c;
    direction = tempDir;
    minDistance = d;
  }

  //high-resolution checking
  tcells.resize(0);
  Meshing::GetTriangleCells(tri,s.baseGrid.value.m-1,s.baseGrid.value.n-1,s.baseGrid.value.p-1,center_bb,tcells);
  for(const auto& cell: tcells) {
    if(s.baseGrid.value(cell) >= minDistance + cellRad) continue;
    AABB3D cellbb;
    cellbb.bmin = center_bb.bmin;
    cellbb.bmin.x += cell.a*cellSize.x;
    cellbb.bmin.y += cell.b*cellSize.y;
    cellbb.bmin.z += cell.c*cellSize.z;
    cellbb.bmax = cellbb.bmin + cellSize;
    Vector3 triTemp = tri.closestPoint((cellbb.bmin+cellbb.bmax)*0.5);
    d = Geometry::DistanceLocal(s.baseGrid,triTemp,&tempPt,&tempDir);
    if(d < minDistance) {
      surfacePt = tempPt;
      triPt = triTemp;
      direction = tempDir;
      minDistance = d;
    }
  }
  return minDistance;
}

Real Distance(const CollisionImplicitSurface& s,const Triangle3D& triw,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance)
{
  Triangle3D tri;
  s.currentTransform.mulInverse(triw.a,tri.a);
  s.currentTransform.mulInverse(triw.b,tri.b);
  s.currentTransform.mulInverse(triw.c,tri.c);
  Real d = DistanceLocal(s,triw,surfacePt,triPt,direction,minDistance);
  surfacePt = s.currentTransform*surfacePt;
  direction = s.currentTransform.R*direction;
  return d;
}


Real Distance(const CollisionImplicitSurface& grid,const GeometricPrimitive3D& a,Vector3& gridclosest,Vector3& geomclosest,Vector3& direction,Real upperBound)
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
    if(!IsInf(upperBound)) {
      if(slocal.distance(grid.baseGrid.bb) > upperBound) {
        return upperBound;
      }
      
      //shrink the range considered by examining the endpoint distances
      Real da = DistanceLocal(grid.baseGrid,slocal.a);
      Real db = DistanceLocal(grid.baseGrid,slocal.b);
      Real t1,t2;
      Real slength = slocal.a.distance(slocal.b);
      if(da <= upperBound) t1=0;
      else {
        if(da - slength > upperBound) {
          return false;
        }
        t1 = (da - upperBound)/slength;
      }
      if(db <= upperBound) t2=1;
      else {
        if(db - slength > upperBound) {
          return false;
        }
        t2 = 1 - (db - upperBound)/slength;
      }
      Vector3 a,b;
      a = slocal.a + t1*(slocal.b-slocal.a);
      b = slocal.a + t2*(slocal.b-slocal.a);
      slocal.a = a;
      slocal.b = b;
    }
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
    Real dmin=upperBound;
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
  else if(a.type == GeometricPrimitive3D::Triangle) {
    const Triangle3D& t = *AnyCast_Raw<Triangle3D>(&a.data);
    Real d= Distance(grid,t,gridclosest,geomclosest,direction,upperBound);
    return d;
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Can only query distance between implicit surface and primitives of type point, sphere, segment, and triangle");
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
    Vector3 temp,temp2;
    return Distance(grid,a,gridclosest,temp,temp2,margin+Epsilon) <= margin;
  }
  else if(a.type == GeometricPrimitive3D::Triangle) {
    Vector3 temp,temp2;
    return Distance(grid,a,gridclosest,temp,temp2,margin+Epsilon) <= margin;
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Can only check collisions between implicit surface and primitives of type point, sphere, segment, and triangle");
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
      printf("%d cells at top level (res %f)\n",(int)lastLevelCells.size(),s.resolutionMap[level]);
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
      printf("%d cells at level res %f\n",(int)lastLevelCells.size(),s.resolutionMap[level]);
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



int PointIndex(const CollisionImplicitSurface &s, const Vector3 &ptworld)
{
  Vector3 plocal;
  s.currentTransform.mulInverse(ptworld, plocal);
  IntTriple cell;
  s.baseGrid.GetIndex(plocal, cell);
  if (cell.a < 0)
    cell.a = 0;
  if (cell.a >= s.baseGrid.value.m)
    cell.a = s.baseGrid.value.m - 1;
  if (cell.b < 0)
    cell.b = 0;
  if (cell.b >= s.baseGrid.value.n)
    cell.b = s.baseGrid.value.n - 1;
  if (cell.c < 0)
    cell.c = 0;
  if (cell.c >= s.baseGrid.value.p)
    cell.c = s.baseGrid.value.p - 1;
  return cell.a * s.baseGrid.value.n * s.baseGrid.value.p + cell.b * s.baseGrid.value.p + cell.c;
}

bool Collides(const CollisionPointCloud &a, Real margin, const CollisionImplicitSurface &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  bool res = Geometry::Collides(b, a, margin, elements1, maxContacts);
  elements2.resize(elements1.size());
  for (size_t i = 0; i < elements1.size(); i++)
    elements2[i] = PointIndex(b, a.currentTransform * a.points[elements1[i]]);
  return res;
}

bool Collides(const CollisionImplicitSurface &a, const CollisionImplicitSurface &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  LOG4CXX_ERROR(GET_LOGGER(Geometry), "Volume grid to volume grid collisions not done");
  return false;
}

bool Collides(const CollisionImplicitSurface &a, const CollisionMesh &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  RigidTransform Tlocal;
  Tlocal.mulInverseA(a.currentTransform, b.currentTransform);
  Triangle3D tri;
  vector<IntTriple> tcells;
  Real cell_corner_values[8];
  Vector3 cellSize = a.baseGrid.GetCellSize();
  AABB3D center_bb = a.baseGrid.bb;
  center_bb.bmin += cellSize * 0.5;
  center_bb.bmax -= cellSize * 0.5;
  Real cellRad = cellSize.norm() * 0.5;
  Meshing::TriMesh cube_mesh;
  cube_mesh.verts.reserve(8);
  for(size_t i=0;i<b.tris.size();i++) {
    b.GetTriangle(i,tri);
    tri.a = Tlocal*tri.a;
    tri.b = Tlocal*tri.b;
    tri.c = Tlocal*tri.c;

    //quickly test whether a sphere containing the triangle is within the margin
    Real test = a.baseGrid.TrilinearInterpolate(tri.a);
    Vector3 pt_clamped;
    test += a.baseGrid.bb.distance(tri.a,pt_clamped);
    Real triDiam = Max(tri.a.distance(tri.b),tri.a.distance(tri.c));
    bool collides = false;
    IntTriple collide_cell;
    if(test <= margin - cellRad) {
      collides = true;
      a.baseGrid.GetIndexClamped(tri.a,collide_cell);
    }
    else if(test > margin + triDiam + cellRad) {
      continue;
    }
    else {
      //TODO: if margin extends implicit surface outside of bounding box, this may be incorrect 
      //high-resolution checking
      tcells.resize(0);
      Meshing::GetTriangleCells(tri,a.baseGrid.value.m-1,a.baseGrid.value.n-1,a.baseGrid.value.p-1,center_bb,tcells);
      for(const auto& cell: tcells) {
        cell_corner_values[0] = a.baseGrid.value(cell.a,cell.b,cell.c);
        if(cell_corner_values[0] > margin + cellRad) continue;
        if(cell_corner_values[0] <= margin - cellRad) {
          collides = true;
          collide_cell = cell;
          break;
        }
        cell_corner_values[1] = a.baseGrid.value(cell.a,cell.b,cell.c+1);
        cell_corner_values[2] = a.baseGrid.value(cell.a,cell.b+1,cell.c);
        cell_corner_values[3] = a.baseGrid.value(cell.a,cell.b+1,cell.c+1);
        cell_corner_values[4] = a.baseGrid.value(cell.a+1,cell.b,cell.c);
        cell_corner_values[5] = a.baseGrid.value(cell.a+1,cell.b,cell.c+1);
        cell_corner_values[6] = a.baseGrid.value(cell.a+1,cell.b+1,cell.c);
        cell_corner_values[7] = a.baseGrid.value(cell.a+1,cell.b+1,cell.c+1);
        AABB3D cellbb;
        cellbb.bmin = center_bb.bmin;
        cellbb.bmin.x += cell.a*cellSize.x;
        cellbb.bmin.y += cell.b*cellSize.y;
        cellbb.bmin.z += cell.c*cellSize.z;
        cellbb.bmax = cellbb.bmin + cellSize;
        cube_mesh.verts.resize(0);
        cube_mesh.tris.resize(0);
        Meshing::CubeToMesh(cell_corner_values,margin,cellbb,cube_mesh);
        Triangle3D tri2;
        Vector3 P,Q;
        for(size_t j=0;j<cube_mesh.tris.size();j++) {
          cube_mesh.GetTriangle(j,tri2);
          if((margin == 0 && tri.intersects(tri2)) || (tri.distance(tri2,P,Q) <= margin)) {
            collides = true;
            collide_cell = cell;
            break;
          }
        }
        if(collides) break;
      }
    }
    if(collides) { 
      elements1.push_back(collide_cell.a*a.baseGrid.value.n*a.baseGrid.value.p + collide_cell.b*a.baseGrid.value.p + collide_cell.c);
      elements2.push_back(int(i));
      if(elements1.size() >= maxContacts) {
        return true;
      }
    }
  }
  return !elements1.empty();
}



bool Geometry3DVolume::Empty() const
{
  return data.value.empty();
}

size_t Geometry3DVolume::NumElements() const 
{
  IntTriple size = data.value.size();
  return size.a * size.b * size.c;
}

shared_ptr<Geometry3D> Geometry3DVolume::GetElement(int elem) const
{
  const Meshing::VolumeGrid &grid = data;
  IntTriple cell = ElementToIndex(elem);
  AABB3D bb;
  grid.GetCell(cell, bb);
  return make_shared<Geometry3DPrimitive>(GeometricPrimitive3D(bb));
}

int Geometry3DVolume::IndexToElement(const IntTriple& idx) const
{
  IntTriple size = data.value.size();
  return idx.a * size.b * size.c + idx.b * size.c + idx.c;
}

IntTriple Geometry3DVolume::ElementToIndex(int elem) const
{
  IntTriple size = data.value.size();
  IntTriple cell;
  cell.a = elem / (size.b * size.c);
  cell.b = (elem / size.c) % size.b;
  cell.c = elem % size.c;
  return cell;
}

bool Geometry3DVolume::Load(istream& in) 
{
  in >> data;
  if(!in) return false;
  return true;
}

bool Geometry3DVolume::Save(ostream& out) const
{
    out << data << endl;
    return true;
}

bool Geometry3DVolume::Transform(const Matrix4 &T)
{
  if (T(0, 1) != 0 || T(0, 2) != 0 || T(1, 2) != 0 || T(1, 0) != 0 || T(2, 0) != 0 || T(2, 1) != 0)
  {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Cannot transform volume grid except via translation / scale");
    return false;
  }
  data.bb.bmin = T * data.bb.bmin;
  data.bb.bmax = T * data.bb.bmax;
  return true;
}

AABB3D Geometry3DVolume::GetAABB() const
{
  return data.bb;
}

void Geometry3DVolume::ResizeTo(const Geometry3D* geom,Real resolution,Real domainExpansion)
{
  AABB3D bb = geom->GetAABB();
  Assert(resolution > 0);
  FitGridToBB(bb,data,resolution,domainExpansion/resolution);
}


Geometry3DImplicitSurface::Geometry3DImplicitSurface(Real _truncation)
: truncationDistance(_truncation)
{}

Geometry3DImplicitSurface::Geometry3DImplicitSurface(const Meshing::VolumeGrid& _data, Real _truncation)
: Geometry3DVolume(_data), truncationDistance(_truncation)
{}

Geometry3DImplicitSurface::Geometry3DImplicitSurface(Meshing::VolumeGrid&& _data, Real _truncation)
: Geometry3DVolume(_data), truncationDistance(_truncation)
{}

bool Geometry3DImplicitSurface::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion)  
{
    switch(geom->GetType()) {
    case Type::Primitive:
        if (param == 0)
        {
            AABB3D bb = geom->GetAABB();
            Real w = (bb.bmax - bb.bmin).maxAbsElement();
            param = w * 0.05;
        }
        PrimitiveToImplicitSurface(dynamic_cast<const Geometry3DPrimitive*>(geom)->data, data, param, domainExpansion);
        return true;
    case Type::ConvexHull:
        if (param == 0)
        {
            AABB3D bb = geom->GetAABB();
            Real w = (bb.bmax - bb.bmin).maxAbsElement();
            param = w * 0.05;
        }
        ConvexHullToImplicitSurface(dynamic_cast<const Geometry3DConvexHull*>(geom)->data, data, param, domainExpansion);
        return true;
    case Type::TriangleMesh:
        {
        const auto& mesh = dynamic_cast<const Geometry3DTriangleMesh*>(geom)->data;
        if (param == 0) {
            if (mesh.tris.empty()) return NULL;
            Real sumlengths = 0;
            for (size_t i = 0; i < mesh.tris.size(); i++) {
                sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
                sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
                sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
            }
            Real avglength = sumlengths / (3 * mesh.tris.size());
            param = avglength / 2;
            Vector3 bmin, bmax;
            mesh.GetAABB(bmin, bmax);
            param = Min(param, 0.25 * (bmax.x - bmin.x));
            param = Min(param, 0.25 * (bmax.y - bmin.y));
            param = Min(param, 0.25 * (bmax.z - bmin.z));
            LOG4CXX_INFO(GET_LOGGER(Geometry), "AnyGeometry::Convert: Auto-determined grid resolution " << param);
        }
        CollisionMesh cmesh(mesh);
        cmesh.CalcTriNeighbors();
        MeshToImplicitSurface_FMM(cmesh, data, param);
        LOG4CXX_INFO(GET_LOGGER(Geometry), "AnyGeometry::Convert: FMM grid bounding box " << data.bb);
        return true;
        }
    default:
        return false;
    }
}

Geometry3D* Geometry3DImplicitSurface::ConvertTo(Type restype, Real param,Real domainExpansion) const
{
    switch (restype)
    {
    case Type::PointCloud:
        {
        auto* pc = new Geometry3DPointCloud();
        Meshing::VolumeGridIterator<Real> it=data.getIterator();
        Vector3 c;
        while(!it.isDone()) {
            if(*it <= param) {
                it.getCellCenter(c);
                pc->data.points.push_back(c);
            }
            ++it;
        }
        return pc;
        }
    case Type::TriangleMesh:
        {
        auto* res = new Geometry3DTriangleMesh();
        ImplicitSurfaceToMesh(data, res->data, param);
        return res;
        }
    case Type::OccupancyGrid:
        {
        auto* res = new Geometry3DOccupancyGrid();
        if(!res->ConvertFrom(this)) {
          LOG4CXX_ERROR(GET_LOGGER(Geometry),"Geometry3DImplicitSurface::ConvertTo: Error converting to occupancy grid");
          delete res;
          return NULL;
        }
        return res;
        }
    default:
        return NULL;
    }
}

Geometry3D* Geometry3DImplicitSurface::Remesh(Real resolution,bool refine,bool coarsen) const
{
  const Meshing::VolumeGrid& grid = data;
  auto* res = new Geometry3DImplicitSurface;
  Vector3 size=grid.GetCellSize();
  if((resolution < size.x && refine) || (resolution > size.x && coarsen) ||
    (resolution < size.y && refine) || (resolution > size.y && coarsen) ||
    (resolution < size.z && refine) || (resolution > size.z && coarsen)) {
    int m = (int)Ceil((grid.bb.bmax.x-grid.bb.bmin.x) / resolution);
    int n = (int)Ceil((grid.bb.bmax.y-grid.bb.bmin.y) / resolution);
    int p = (int)Ceil((grid.bb.bmax.z-grid.bb.bmin.z) / resolution);
    Meshing::VolumeGrid& output = res->data;
    output.Resize(m,n,p);
    output.bb = grid.bb;
    output.ResampleTrilinear(grid);
  }
  else {
    res->data = grid;
  }
  return res;
}

bool Geometry3DImplicitSurface::Merge(const Geometry3D* geom,const RigidTransform* Tgeom)
{
  if(Tgeom) {
      Geometry3D* temp = geom->Copy();
      if(!temp->Transform(*Tgeom)) return false;
      bool res = Merge(temp);
      delete temp;
      return res;
  }
  if(geom->GetType() == Type::ImplicitSurface) {
    auto* g = dynamic_cast<const Geometry3DImplicitSurface*>(geom);
    if(g->data.value.empty()) return true;
    if(data.value.empty()) {
      data = g->data;
      return true;
    }
    if(data.IsSimilar(g->data)) {
      data.Min(g->data);
    }
    else {
      Meshing::VolumeGrid temp;
      temp.MakeSimilar(data);
      temp.ResampleTrilinear(g->data);
      data.Min(temp);
    }
    return true;
  }
  else if(geom->GetType() == Type::ConvexHull) {
    auto* g = dynamic_cast<const Geometry3DConvexHull*>(geom);
    Meshing::VolumeGrid temp;
    temp.MakeSimilar(data);
    temp.value.set(Inf);
    ConvexHullImplicitSurfaceFill(g->data, temp, truncationDistance);
    data.Min(temp);
    return true;
  }
  else if(geom->GetType() == Type::Primitive) {
    auto* g = dynamic_cast<const Geometry3DPrimitive*>(geom);
    Meshing::VolumeGrid temp;
    temp.MakeSimilar(data);
    temp.value.set(Inf);
    PrimitiveImplicitSurfaceFill(g->data, temp, truncationDistance);
    data.Min(temp);
    return true;
  }
  else if(geom->GetType() == Type::PointCloud) {
    auto* g = dynamic_cast<const Geometry3DPointCloud*>(geom);
    PointCloudImplicitSurfaceFill_FMM(g->data, data, truncationDistance);
    return true;
  }
  else {
    return false;
  }
}

Collider3DImplicitSurface::Collider3DImplicitSurface(shared_ptr<Geometry3DImplicitSurface> _data)
:data(_data)
{
  Reset();
}

void Collider3DImplicitSurface::Reset()
{
  collisionData.baseGrid = data->data;
  collisionData.InitCollisions();
}

bool Collider3DImplicitSurface::Merge(Collider3D* geom)
{
  if(geom->GetType() == Type::TriangleMesh) {
    auto* mesh = dynamic_cast<Collider3DTriangleMesh*>(geom);
    //copy mesh collision data and transform it to this geometry's frame
    CollisionMesh temp = mesh->collisionData;
    RigidTransform Tg_this;
    Tg_this.mulInverseA(collisionData.currentTransform, temp.currentTransform);
    temp.Transform(Tg_this);
    Array3D<Vector3> gradient;
    std::vector<IntTriple> surfaceCells;
    /*
    //TODO: see why this doesn't work well.  It doesn't seem to update the internal cells well
    Meshing::FastMarchingMethod_Fill(temp,data->data.value,gradient,data->data.bb,surfaceCells,true);
    */
    ///more expensive but higher accuracy
    Meshing::VolumeGrid vg;
    vg.MakeSimilar(data->data);
    MeshImplicitSurfaceFill_FMM(temp,vg);
    data->data.Min(vg);
    
    Reset();
    return true;
  }
  else {
    return Collider3D::Merge(geom);
  }
}


bool Collider3DImplicitSurface::Contains(const Vector3& pt,bool& result)
{
  Real d;
  Distance(pt,d);
  result = (d <= 0);
  return true;
}

bool Collider3DImplicitSurface::Distance(const Vector3& pt,Real& result)
{
    result = Geometry::Distance(collisionData, pt);
    return true;
}

bool Collider3DImplicitSurface::Distance(const Vector3 &pt, const DistanceQuerySettings &settings,DistanceQueryResult& res)
{
    res.hasClosestPoints = true;
    res.hasElements = true;
    res.elem2 = 0;
    res.cp2 = pt;

    const CollisionImplicitSurface &vg = collisionData;
    res.d = Geometry::Distance(vg, pt, res.cp1, res.dir2);
    res.dir1.setNegative(res.dir2);
    res.hasPenetration = true;
    res.hasDirections = true;
    res.elem1 = PointIndex(vg, res.cp1);
    //cout<<"Doing ImplicitSurface - point collision detection, with direction "<<res.dir2<<endl;
    return true;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const CollisionImplicitSurface &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.elem1 = 0;
  res.d = Geometry::Distance(b, a, res.cp2, res.cp1, res.dir1);
  res.elem2 = PointIndex(b, res.cp2);
  res.dir2.setNegative(res.dir1);
  return res;
}

AnyDistanceQueryResult Distance(const CollisionImplicitSurface &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.d = Geometry::Distance(a, b, res.elem2, settings.upperBound);
  res.cp2 = b.currentTransform * b.points[res.elem2];
  Geometry::Distance(a, res.cp2, res.cp1, res.dir1);
  res.dir2.setNegative(res.dir1);
  res.elem1 = PointIndex(a, res.cp1);
  return res;
}

bool Collider3DImplicitSurface::Distance(Collider3D* geom, const DistanceQuerySettings &settings,DistanceQueryResult& res)
{
  switch (geom->GetType())
  {
  case Type::Primitive:
  {
    GeometricPrimitive3D bw = dynamic_cast<Collider3DPrimitive*>(geom)->data->data;
    bw.Transform(geom->GetTransform());
    res = Geometry::Distance(bw, collisionData, settings);
    Flip(res);
    return true;
  }
  case Type::PointCloud:
  {
    auto& pc = dynamic_cast<Collider3DPointCloud*>(geom)->collisionData;
    res = Geometry::Distance(collisionData, pc, settings);
    return true;
  }
  default:
    return false;
  }
}


bool Collider3DImplicitSurface::WithinDistance(Collider3D* geom,Real d,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  switch (geom->GetType())
  {
  case Type::Primitive:
  {
    GeometricPrimitive3D bw = dynamic_cast<Collider3DPrimitive*>(geom)->data->data;
    bw.Transform(geom->GetTransform());
    Vector3 cpa;
    if (Geometry::Collides(collisionData, bw, d, cpa))
    {
      elements1.resize(1);
      elements1[0] = PointIndex(collisionData, cpa);
      elements2.push_back(0);
    }
    return true;
  }
  case Type::ImplicitSurface:
  {
        auto& b = dynamic_cast<Collider3DImplicitSurface*>(geom)->collisionData;
        Geometry::Collides(collisionData, b, d, elements1, elements2, maxContacts);
        return true;
  }
  case Type::TriangleMesh:
  {
        auto& b = dynamic_cast<Collider3DTriangleMesh*>(geom)->collisionData;
        Geometry::Collides(collisionData, b, d, elements1, elements2, maxContacts);
        return true;
  }
  case Type::PointCloud:
  {
        auto& b = dynamic_cast<Collider3DPointCloud*>(geom)->collisionData;
        bool res = Geometry::Collides(collisionData, b, d, elements2, maxContacts);
        elements1.resize(elements2.size());
        for (size_t i = 0; i < elements2.size(); i++)
            elements1[i] = PointIndex(collisionData, b.currentTransform * b.points[elements2[i]]);
        if(res) assert(!elements1.empty());
        return true;
  }
  break;
  default:
    return false;
  }
  return false;
}


bool Collider3DImplicitSurface::RayCast(const Ray3D& r,Real margin,Real& distance, int& element)
{
    const auto& surf = collisionData;
    distance = Geometry::RayCast(surf,r,margin);
    if(!IsInf(distance)) {
        element = PointIndex(surf,r.source+distance*r.direction);
        return true;
    }
    return true;
}

void PointCloudImplicitSurfaceContacts(CollisionPointCloud &pc1, Real outerMargin1, CollisionImplicitSurface &s2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  Geometry::Collides(s2, pc1, tol, points, maxcontacts);
  contacts.reserve(points.size());
  for (size_t j = 0; j < points.size(); j++)
  {
    Vector3 p1w = pc1.currentTransform * pc1.points[points[j]];
    Vector3 p2w;
    Vector3 n;
    Real d = Geometry::Distance(s2, p1w, p2w, n);
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = p1w + n * outerMargin1;
    contacts[k].p2 = p2w - n * outerMargin2;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points[j];
    contacts[k].elem2 = PointIndex(s2, p2w);
    contacts[k].unreliable = false;
  }
}

void TriangleMeshImplicitSurfaceContacts(CollisionMesh &m1, Real outerMargin1, CollisionImplicitSurface &s2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  Triangle3D tri;
  Vector3 surfacePt, triPt, direction;
  for(size_t i=0;i<m1.tris.size();i++) {
    m1.GetTriangle(i,tri);
    Real d=Distance(s2,tri,surfacePt,triPt,direction,tol+Epsilon);
    if(d <= tol) {
      size_t k = contacts.size();
      contacts.resize(k + 1);
      contacts[k].p1 = triPt + direction * outerMargin1;
      contacts[k].p2 = surfacePt - direction * outerMargin2;
      contacts[k].n = direction;
      contacts[k].depth = tol - d;
      contacts[k].elem1 = i;
      contacts[k].elem2 = PointIndex(s2, surfacePt);
      contacts[k].unreliable = false;
    }
  }
}

void ImplicitSurfaceSphereContacts(CollisionImplicitSurface &s1, Real outerMargin1, const Sphere3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cp, dir;
  Real d = Distance(s1, s.center, cp, dir) - s.radius;
  if (d > tol)
    return;
  Vector3 pw = s.center + dir * s.radius;
  contacts.resize(1);
  Vector3 n;
  n.setNegative(dir);
  contacts[0].p1 = cp + outerMargin1 * n;
  contacts[0].p2 = pw - outerMargin2 * n;
  contacts[0].n = n;
  contacts[0].depth = tol - d;
  contacts[0].elem1 = PointIndex(s1, cp);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfaceSegmentContacts(CollisionImplicitSurface &s1, Real outerMargin1, const Segment3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cpgrid, cpseg, dir;
  Real d = Geometry::Distance(s1, GeometricPrimitive3D(s), cpgrid, cpseg, dir);
  if (d > tol)
    return;
  contacts.resize(1);
  Vector3 n;
  n.setNegative(dir);
  contacts[0].p1 = cpgrid + outerMargin1 * n;
  contacts[0].p2 = cpseg - outerMargin2 * n;
  contacts[0].n = n;
  contacts[0].depth = tol - d;
  contacts[0].elem1 = PointIndex(s1, cpgrid);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfaceTriangleContacts(CollisionImplicitSurface &s1, Real outerMargin1, const Triangle3D &t, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cpgrid, cpseg, dir;
  Real d = Geometry::Distance(s1, GeometricPrimitive3D(t), cpgrid, cpseg, dir);
  if (d > tol)
    return;
  contacts.resize(1);
  Vector3 n;
  n.setNegative(dir);
  contacts[0].p1 = cpgrid + outerMargin1 * n;
  contacts[0].p2 = cpseg - outerMargin2 * n;
  contacts[0].n = n;
  contacts[0].depth = tol - d;
  contacts[0].elem1 = PointIndex(s1, cpgrid);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfacePrimitiveContacts(CollisionImplicitSurface &s1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  GeometricPrimitive3D gworld = g2;
  gworld.Transform(T2);

  if (gworld.type == GeometricPrimitive3D::Point)
  {
    Sphere3D s;
    s.center = *AnyCast<Point3D>(&gworld.data);
    s.radius = 0;
    ImplicitSurfaceSphereContacts(s1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else if (gworld.type == GeometricPrimitive3D::Sphere)
  {
    const Sphere3D &s = *AnyCast<Sphere3D>(&gworld.data);
    ImplicitSurfaceSphereContacts(s1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else if (gworld.type == GeometricPrimitive3D::Segment)
  {
    const Segment3D &s = *AnyCast<Segment3D>(&gworld.data);
    ImplicitSurfaceSegmentContacts(s1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else if (gworld.type == GeometricPrimitive3D::Triangle)
  {
    const Triangle3D &s = *AnyCast<Triangle3D>(&gworld.data);
    ImplicitSurfaceTriangleContacts(s1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Contact computations between ImplicitSurface and " << gworld.TypeName() << " not supported");
    return;
  }
}

bool Collider3DImplicitSurface::Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) 
{
  switch (other->GetType())
  {
    case Type::Primitive:
      {
        auto* prim = dynamic_cast<Collider3DPrimitive*>(other);
        ImplicitSurfacePrimitiveContacts(collisionData, settings.padding1, prim->data->data, prim->T, settings.padding2, res.contacts, settings.maxcontacts);
        return true;
      }
    case Type::TriangleMesh:
      {
        auto* m = dynamic_cast<Collider3DTriangleMesh*>(other);
        TriangleMeshImplicitSurfaceContacts(m->collisionData, settings.padding2, collisionData, settings.padding1, res.contacts, settings.maxcontacts);
        for (auto &c : res.contacts)
          ReverseContact(c);
      }
      return true;
    case Type::PointCloud:
      {
        auto* pc = dynamic_cast<Collider3DPointCloud*>(other);
        PointCloudImplicitSurfaceContacts(pc->collisionData, settings.padding2, collisionData, settings.padding1, res.contacts, settings.maxcontacts);
        for (auto &c : res.contacts)
          ReverseContact(c);
      }
      return true;
    case Type::ImplicitSurface:
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: implicit surface-implicit surface contacts");
      break;
    case Type::Group:
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: implicit surface-group contacts");
      break;
    case Type::ConvexHull:
      LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: implicit surface-convex hull contacts");
      break;
    default:
      return false;
  }
  return false;
}

bool Collider3DImplicitSurface::ConvertFrom(Collider3D* geom,Real param,Real domainExpansion)
{
  if(geom->GetType() == Type::TriangleMesh)
  {
    auto* gmesh = dynamic_cast<Collider3DTriangleMesh*>(geom);
    if(gmesh->collisionData.triNeighbors.empty())
      gmesh->collisionData.CalcTriNeighbors();
    if (param == 0)
    {
      const Meshing::TriMesh &mesh = gmesh->data->data;
      if (mesh.tris.empty())
        return false;
      Real sumlengths = 0;
      for (size_t i = 0; i < mesh.tris.size(); i++)
      {
        sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
        sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
        sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
      }
      Real avglength = sumlengths / (3 * mesh.tris.size());
      param = avglength / 2;
      Vector3 bmin, bmax;
      mesh.GetAABB(bmin, bmax);
      param = Min(param, 0.25 * (bmax.x - bmin.x));
      param = Min(param, 0.25 * (bmax.y - bmin.y));
      param = Min(param, 0.25 * (bmax.z - bmin.z));
      LOG4CXX_INFO(GET_LOGGER(Geometry), "Auto-determined grid resolution " << param);
    }
    Meshing::VolumeGrid& grid = data->data;
    RigidTransform Torig, Tident;
    Tident.setIdentity();
    CollisionMesh &cmesh = gmesh->collisionData;
    cmesh.GetTransform(Torig);
    cmesh.UpdateTransform(Tident);
    MeshToImplicitSurface_FMM(cmesh, grid, param, domainExpansion);
    //MeshToImplicitSurface_SpaceCarving(TriangleMeshCollisionData(),grid,param,40);
    cmesh.UpdateTransform(Torig);
    //sLOG4CXX_INFO(GET_LOGGER(Geometry), "Grid bb " << grid.bb);
    SetTransform(geom->GetTransform());
    return true;
  }
  return Collider3D::ConvertFrom(geom, param, domainExpansion);
}




} //namespace Geometry