#include "CollisionImplicitSurface.h"
#include "CollisionPrimitive.h"
#include "CollisionPointCloud.h"
#include "CollisionMesh.h"
#include "CollisionOccupancyGrid.h"
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

#include "PQP/include/PQP.h"
#include "PQP/src/MatVec.h"

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
  if(!mingrid->GetIndexRangeClamped(bb,imin,imax)) {
    return;
  }
  for(int i=imin.a;i<=imax.a;i++) {
    for(int j=imin.b;j<=imax.b;j++) {
      for(int k=imin.c;k<=imax.c;k++) {
        vmin = Min(vmin,mingrid->value(i,j,k));
        vmax = Max(vmax,maxgrid->value(i,j,k));
      }
    }
  }
}


AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const Collider3DImplicitSurface &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.elem1 = 0;
  res.d = b.Distance(a, res.cp2, res.cp1, res.dir1);
  res.elem2 = b.PointToElement(res.cp2);
  res.dir2.setNegative(res.dir1);
  return res;
}

AnyDistanceQueryResult Distance(const Collider3DImplicitSurface &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.d = a.Distance(b, res.elem2, settings.upperBound);
  res.cp2 = b.currentTransform * b.points[res.elem2];
  a.Distance(res.cp2, res.cp1, res.dir1);
  res.dir2.setNegative(res.dir1);
  res.elem1 = a.PointToElement(res.cp1);
  return res;
}


Real DistanceSDF(const Meshing::VolumeGrid& grid,const Vector3& pt,Vector3* surfacePt,Vector3* direction)
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

Real DistanceSDF(const Meshing::VolumeGrid& sdf,const Triangle3D& tri,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance)
{
  vector<IntTriple> tcells;
  Vector3 cellSize = sdf.GetCellSize();
  AABB3D center_bb = sdf.bb;
  center_bb.bmin += cellSize * 0.5;
  center_bb.bmax -= cellSize * 0.5;
  Real cellRad = cellSize.norm() * 0.5;

  //quickly test whether a sphere containing the triangle is within the margin
  Vector3 tempPt,tempDir;
  Vector3 c = (tri.a+tri.b+tri.c) / 3.0;
  Real d = DistanceSDF(sdf,c,&surfacePt,&direction);
  if(d > minDistance) {
    return minDistance;
  }
  triPt = c;
  minDistance = d;
  d = DistanceSDF(sdf,tri.a,&tempPt,&tempDir);
  if(d < minDistance) {
    surfacePt = tempPt;
    triPt = tri.a;
    direction = tempDir;
    minDistance = d;
  }
  d = DistanceSDF(sdf,tri.b,&tempPt,&tempDir);
  if(d < minDistance) {
    surfacePt = tempPt;
    triPt = tri.b;
    direction = tempDir;
    minDistance = d;
  }
  d = DistanceSDF(sdf,tri.c,&tempPt,&tempDir);
  if(d < minDistance) {
    surfacePt = tempPt;
    triPt = tri.c;
    direction = tempDir;
    minDistance = d;
  }

  //high-resolution checking
  tcells.resize(0);
  Meshing::GetTriangleCells(tri,sdf.value.m-1,sdf.value.n-1,sdf.value.p-1,center_bb,tcells);
  for(const auto& cell: tcells) {
    if(sdf.value(cell) >= minDistance + cellRad) continue;
    AABB3D cellbb;
    cellbb.bmin = center_bb.bmin;
    cellbb.bmin.x += cell.a*cellSize.x;
    cellbb.bmin.y += cell.b*cellSize.y;
    cellbb.bmin.z += cell.c*cellSize.z;
    cellbb.bmax = cellbb.bmin + cellSize;
    Vector3 triTemp = tri.closestPoint((cellbb.bmin+cellbb.bmax)*0.5);
    d = DistanceSDF(sdf,triTemp,&tempPt,&tempDir);
    if(d < minDistance) {
      surfacePt = tempPt;
      triPt = triTemp;
      direction = tempDir;
      minDistance = d;
    }
  }
  return minDistance;
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


} //namespace Geometry

using namespace Geometry;

int Collider3DImplicitSurface::PointToElement(const Vector3 &ptworld) const
{
  Vector3 plocal;
  currentTransform.mulInverse(ptworld, plocal);
  return data->PointToElement(plocal);
}


void Collider3DImplicitSurface::DistanceRangeLocal(const AABB3D& bb,Real& vmin,Real& vmax) const
{
  Vector3 size = bb.bmax-bb.bmin;
  Real res = Max(size.x,size.y,size.z);
  const Meshing::VolumeGrid *chosenMin = &data->data, *chosenMax = &data->data;
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


Real Collider3DImplicitSurface::Distance(const Triangle3D& triw,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance) const
{
  Triangle3D tri;
  currentTransform.mulInverse(triw.a,tri.a);
  currentTransform.mulInverse(triw.b,tri.b);
  currentTransform.mulInverse(triw.c,tri.c);
  Real d= DistanceLocal(tri,surfacePt,triPt,direction,minDistance);
  surfacePt = currentTransform*surfacePt;
  direction = currentTransform.R*direction;
  return d;
}

Real Collider3DImplicitSurface::DistanceLocal(const Triangle3D& tri_local,Vector3& surfacePt,Vector3& triPt,Vector3& direction,Real minDistance) const
{
  return Geometry::DistanceSDF(data->data,tri_local,surfacePt,triPt,direction,minDistance);
}


Real Collider3DImplicitSurface::Distance(const GeometricPrimitive3D& a,Vector3& gridclosest,Vector3& geomclosest,Vector3& direction,Real upperBound) const
{
  if(a.type == GeometricPrimitive3D::Point) {
    const Vector3& pt = *AnyCast_Raw<Vector3>(&a.data);
    geomclosest = pt;
    Real d= Distance(pt,gridclosest,direction);
    return d;
  }
  else if(a.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D* s=AnyCast_Raw<Sphere3D>(&a.data);
    Real d = Distance(s->center,gridclosest,direction);
    geomclosest = s->center + s->radius*direction;
    return d - s->radius;
  }
  else if(a.type == GeometricPrimitive3D::Segment) {
    const Segment3D* s=AnyCast_Raw<Segment3D>(&a.data);
    Segment3D slocal;
    currentTransform.mulInverse(s->a,slocal.a);
    currentTransform.mulInverse(s->b,slocal.b);
    if(!IsInf(upperBound)) {
      if(slocal.distance(data->data.bb) > upperBound) {
        return upperBound;
      }
      
      //shrink the range considered by examining the endpoint distances
      Real da = Geometry::DistanceSDF(data->data,slocal.a);
      Real db = Geometry::DistanceSDF(data->data,slocal.b);
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
    Vector3 cellsize = data->data.GetCellSize();
    Real cellbnd = Max(cellsize.x,cellsize.y,cellsize.z)*0.5;
    Segment3D sgrid;
    sgrid.a.x = (slocal.a.x - data->data.bb.bmin.x) / cellsize.x;
    sgrid.a.y = (slocal.a.y - data->data.bb.bmin.y) / cellsize.y;
    sgrid.a.z = (slocal.a.z - data->data.bb.bmin.z) / cellsize.z;
    sgrid.b.x = (slocal.b.x - data->data.bb.bmin.x) / cellsize.x;
    sgrid.b.y = (slocal.b.y - data->data.bb.bmin.y) / cellsize.y;
    sgrid.b.z = (slocal.b.z - data->data.bb.bmin.z) / cellsize.z;
    vector<Real> params;
    vector<IntTriple> cells;
    Meshing::GetSegmentCells(sgrid,cells,&params);
    cells.push_back(cells.back());
    vector<tuple<Real,IntTriple,Real> > candidates;
    for(size_t i=0;i<cells.size();i++) {
      IntTriple c = cells[i];
      if(c.a < 0) c.a=0; else if(c.a >= data->data.value.m) c.a = data->data.value.m-1;
      if(c.b < 0) c.b=0; else if(c.b >= data->data.value.n) c.b = data->data.value.n-1;
      if(c.c < 0) c.c=0; else if(c.c >= data->data.value.p) c.c = data->data.value.p-1;
      Real d=data->data.value(c);
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
      Real d_bb = data->data.bb.distance(x,xclamped);
      if(d_bb >= dmin) continue;
      if(d_bb + d - cellbnd >= dmin) continue;
      Real sdf_value = data->data.TrilinearInterpolate(xclamped);
      if(d_bb + sdf_value < dmin) {
        tmin = t;
        dmin = d_bb + sdf_value;
      }
    }
    geomclosest = slocal.a + tmin*disp;
    Vector3 pt_clamped;
    Real d_bb = data->data.bb.distance(geomclosest,pt_clamped);
    Real sdf_value = data->data.TrilinearInterpolate(pt_clamped);
    data->data.Gradient(pt_clamped,direction);
    direction.inplaceNormalize();
    gridclosest = pt_clamped - direction*sdf_value;
    if(d_bb > 0) {
      direction = gridclosest - geomclosest;
      direction.inplaceNormalize();
    }
    else
      direction.inplaceNegative();

    geomclosest = currentTransform*geomclosest;
    gridclosest = currentTransform*gridclosest;
    direction = currentTransform.R*direction;
    return geomclosest.distance(gridclosest);
  }
  else if(a.type == GeometricPrimitive3D::Triangle) {
    const Triangle3D& t = *AnyCast_Raw<Triangle3D>(&a.data);
    Real d= Distance(t,gridclosest,geomclosest,direction,upperBound);
    return d;
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Can only query distance between implicit surface and primitives of type point, sphere, segment, and triangle");
    gridclosest.set(Inf);
    geomclosest.set(Inf);
    return 0;
  }
}

bool Collider3DImplicitSurface::Collides(const GeometricPrimitive3D& a,Real margin,Vector3& gridclosest) const
{
  if(a.type == GeometricPrimitive3D::Point || a.type == GeometricPrimitive3D::Sphere) {
    Vector3 temp,temp2;
    return Distance(a,gridclosest,temp,temp2) <= margin;
  }
  else if(a.type == GeometricPrimitive3D::Segment) {
    Vector3 temp,temp2;
    return Distance(a,gridclosest,temp,temp2,margin+Epsilon) <= margin;
  }
  else if(a.type == GeometricPrimitive3D::Triangle) {
    Vector3 temp,temp2;
    return Distance(a,gridclosest,temp,temp2,margin+Epsilon) <= margin;
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Can only check collisions between implicit surface and primitives of type point, sphere, segment, and triangle");
    gridclosest.set(Inf);
    return 0;
  }
}

bool Collider3DImplicitSurface::Collides(const CollisionPointCloud& pc,Real margin,vector<int>& collidingPoints,size_t maxContacts) const
{
  //first do a quick reject test
  Timer timer;
  Box3D pcbb;
  Geometry::GetBB(pc,pcbb);
  Box3D sbb;
  sbb.setTransformed(data->data.bb,currentTransform);
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
    if(Distance(p_w) <= margini) {
      collidingPoints.push_back(aids[i]);
      if(collidingPoints.size() >= maxContacts) {
        //LOG4CXX_DEBUG(GET_LOGGER(Geometry),"PointCloud-ImplicitSurface "<<collidingPoints.size()<<">="<<maxContacts<<" contacts time "<<timer.ElapsedTime());
        //LOG4CXX_INFO(GET_LOGGER(Geometry),"Collision in time "<<timer.ElapsedTime());
        return true;
      }
    }
  }
  //LOG4CXX_DEBUG(GET_LOGGER(Geometry),"PointCloud-ImplicitSurface "<<collidingPoints.size()<<" contacts time "<<timer.ElapsedTime());     
  //LOG4CXX_INFO(GET_LOGGER(Geometry),"No collision in time "<<timer.ElapsedTime());
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

Real DistanceLowerBound(const Collider3DImplicitSurface& s,const CollisionPointCloud& pc,int nindex,const Matrix4& Mpc_s)
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
  return dmin+orientedBB.distance(s.data->data.bb) - BOUNDING_VOLUME_FUZZ;
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
  Real dc = s.data->data.TrilinearInterpolate(clocal);
  Real d_bb = s.data->data.bb.distance(clocal);
  return d_bb + dc - rad - BOUNDING_VOLUME_FUZZ;
}

Real Collider3DImplicitSurface::Distance(const CollisionPointCloud& pc,int& closestPoint,Real upperBound) const
{
  //Timer timer;
  assert(pc.octree != NULL);
  RigidTransform Tpc_s;
  Tpc_s.mulInverseA(currentTransform,pc.currentTransform);
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
      Real sdf_value = data->data.TrilinearInterpolate(ptlocal);
      if(pc.radiusIndex >= 0)
        sdf_value -= pc.properties[i][pc.radiusIndex];
      if(sdf_value < bruteForceDmin) {
        Real d_bb = data->data.bb.distance(ptlocal,pt_clamped);
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

  Real d = DistanceLowerBound(*this,pc,0,Mpc_s);
  if(d < upperBound) 
    heap.push(0,d);
  
  int numPointsChecked = 0;
  int numBBsChecked = 1;
  int numBBsPruned = 0;
  Real mindist = upperBound;
  closestPoint = -1;
  d = Distance(pc.currentTransform*pc.points[0]);
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
        Real sdf_value = data->data.TrilinearInterpolate(ptlocal);
        if(pc.radiusIndex >= 0)
          sdf_value -= pc.properties[id][pc.radiusIndex];
        if(sdf_value < mindist) {
          Real d_bb = data->data.bb.distance(ptlocal,pt_clamped);
          Real d = d_bb + sdf_value;
          if(d < mindist) {
            //debugging
            if(DEBUG_DISTANCE_CHECKING && pc.radiusIndex < 0) {
              Assert(FuzzyEquals(d,Distance(pc.currentTransform*pc.points[id])));
            }
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
        Real d = DistanceLowerBound(*this,pc,n.childIndices[c],Mpc_s);
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
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"Collider3DImplicitSurface vs CollisionPointCloud Distance error");
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"   Discrepancy between brute force and expedited checking: "<<bruteForceDmin<<" vs "<<mindist);
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"   Points "<<bruteForceClosestPoint<<" vs "<<closestPoint);
      //Abort();
    }
  }
  //printf("Expedited distance checking result %g, %d, time %g\n",mindist,closestPoint,timer.ElapsedTime());
  //printf("Checked %d / %d points, %d bounding boxes (%d pruned)\n",numPointsChecked,(int)pc.points.size(),numBBsChecked,numBBsPruned);
  return mindist;
}

///Returns the distance to the closest point on the implicit surface defined at the given level set.
///The algorithm uses the collision hierarchy (O(log n) where n is the resolution of the grid.
Real Collider3DImplicitSurface::RayCast(const Ray3D& rayWorld,Real levelSet,Real tmax) const
{
  Ray3D ray;
  currentTransform.mulInverse(rayWorld.source,ray.source);
  currentTransform.R.mulTranspose(rayWorld.direction,ray.direction);
  if(resolutionMap.empty())
    return Geometry::RayCast(data->data,ray,levelSet,tmax);
  return Geometry::RayCast(data->data,ray,levelSet,tmax);

  //TODO: make this experimental code work

  Real umin=0,umax=tmax;
  AABB3D center_bb = data->data.bb;
  Vector3 celldims = data->data.GetCellSize();
  //make correction for grid cell centers
  center_bb.bmin += celldims*0.5;
  center_bb.bmax -= celldims*0.5;
  if(!ray.intersects(center_bb,umin,umax)) return tmax;

  vector<IntTriple> lastLevelCells;
  vector<IntTriple> temp;
  AABB3D cell;
  vector<pair<Real,IntTriple> > intersections;
  for(size_t res=0;res<resolutionMap.size();res++) {
    size_t level = resolutionMap.size()-res-1;
    const Meshing::VolumeGrid& smin = minHierarchy[level];
    const Meshing::VolumeGrid& smax = maxHierarchy[level];
    if(res==0) { //top level, do initialization
      Segment3D overlap;
      overlap.a = ray.source + umin*ray.direction;
      overlap.b = ray.source + umax*ray.direction;
      if(data->data.TrilinearInterpolate(overlap.a) < levelSet) return 0; //already inside
      Meshing::GetSegmentCells(overlap,smin.value.m-1,smin.value.n-1,smin.value.p-1,smin.bb,temp);
      for(const auto& c:temp) {
        if(smin.value(c) <= levelSet && smax.value(c) >= levelSet)
          lastLevelCells.push_back(c);
      }
      printf("%d cells at top level (res %f)\n",(int)lastLevelCells.size(),resolutionMap[level]);
    }
    else {
      const Meshing::VolumeGrid& snmin = minHierarchy[level+1];
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
      printf("%d cells at level res %f\n",(int)lastLevelCells.size(),resolutionMap[level]);
    }
  }
  const Meshing::VolumeGrid& snmin = minHierarchy[0];
  for(const auto& c: lastLevelCells) {
    snmin.GetCell(c,cell);    
    IntTriple imin,imax;
    data->data.GetIndexRange(cell,imin,imax);
    Real dmin = Inf;
    for(int i=imin.a;i<=imax.a;i++) {
      for(int j=imin.b;j<=imax.b;j++) {
        for(int k=imin.c;k<=imax.c;k++) {
          IntTriple c2(i,j,k);
          AABB3D cell2;
          data->data.GetCell(c2,cell2);
          cell2.bmin += celldims*0.5;
          cell2.bmax += celldims*0.5;
          umin = 0;
          umax = tmax;
          if(!ray.intersects(cell2,umin,umax))
            continue;
          Real res = RayGridCellIntersect(ray,data->data,c2,umin,umax,levelSet,tmax);
          if(res >= 0) dmin = Min(dmin,res);
        }
      }
    }
    if(dmin < tmax) return dmin;
  }
  return tmax;
}




bool Collides(const CollisionPointCloud &a, Real margin, const Collider3DImplicitSurface &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  bool res = b.Collides(a, margin, elements1, maxContacts);
  elements2.resize(elements1.size());
  for (size_t i = 0; i < elements1.size(); i++)
    elements2[i] = b.PointToElement(a.currentTransform * a.points[elements1[i]]);
  return res;
}

bool Collides(const Collider3DImplicitSurface &a, const Collider3DImplicitSurface &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  Vector3 sa = a.data->data.GetCellSize();
  Vector3 sb = b.data->data.GetCellSize();
  Assert(IsFinite(sa.maxAbsElement()) && IsFinite(sb.maxAbsElement()));
  if(sa.maxAbsElement() > sb.maxAbsElement()) {
    return Collides(b,a,margin,elements2,elements1,maxContacts);
  }
  //ordered so that a is finer than b
  AABB3D cell;
  Vector3 centerLocal,center;
  Array3D<bool> skippedCells,newSkippedCells;
  for(int i=(int)a.maxHierarchy.size()-1;i>=0;i--) {
    const Meshing::VolumeGrid& smin = a.minHierarchy[i];
    //const Meshing::VolumeGrid& smax = a.maxHierarchy[i];

    newSkippedCells.resize(smin.value.m,smin.value.n,smin.value.p,false);
    int numSkipped = 0;
    for(Meshing::VolumeGrid::iterator c = smin.getIterator();!c.isDone();++c) {
      c.getCell(cell);
      centerLocal = (cell.bmin + cell.bmax)*0.5;
      a.currentTransform.mul(centerLocal,center);
      if(i+1<(int)a.maxHierarchy.size()) {
        IntTriple pindex;
        a.maxHierarchy[i+1].GetIndex(centerLocal,pindex);
        // cout<<"Checking cell "<<c.getIndex()<<" with center "<<centerLocal<<endl;
        // cout<<"Parent grid bounds "<<a.maxHierarchy[i+1].bb<<endl;
        // cout<<"Parent cell index "<<pindex<<endl;
        Assert(pindex.a >= 0 && pindex.a < a.maxHierarchy[i+1].value.m);
        Assert(pindex.b >= 0 && pindex.b < a.maxHierarchy[i+1].value.n);
        Assert(pindex.c >= 0 && pindex.c < a.maxHierarchy[i+1].value.p);
        if(skippedCells(pindex)) {
          newSkippedCells(c.getIndex()) = true;
          numSkipped++;
          continue;        
        }
      }
      IntTriple test = c.getIndex();
      Assert(test.a >= 0 && test.a < newSkippedCells.m);
      Assert(test.b >= 0 && test.b < newSkippedCells.n);
      Assert(test.c >= 0 && test.c < newSkippedCells.p);
      Real radius = cell.bmin.distance(cell.bmax)*0.5;
      Real damin = *c;
      Real damax = a.maxHierarchy[i].value(test);
      Vector3 cpb,gradb;
      Real db = b.Distance(center,cpb,gradb);
      if(db + damax <= margin) { //sphere at center certainly collides with b
        newSkippedCells(c.getIndex()) = true;
        numSkipped++;
        // Vector3 grada;
        // a.currentTransform.mulInverse(cpb,centerLocal);
        // a.data->data.Gradient(centerLocal,grada);
        // cout<<"Known collision found at hierarchy level "<<i<<endl;
        // cout<<"Cell "<<c.getIndex()<<" center "<<center<<endl;
        // cout<<"A minimum depth "<<damin<<" maximum depth "<<damax<<endl;
        // cout<<"B depth "<<db<<endl;
        elements1.push_back(a.PointToElement(cpb));
        elements2.push_back(b.PointToElement(cpb));
        if(elements1.size() >= maxContacts) {
          return true;
        }
      }
      else if(db + damin >= margin + radius) {
        //no way for the sphere centered at cell to collide with b
        newSkippedCells(c.getIndex()) = true;
        numSkipped++;
      }
    }
    if(numSkipped == smin.value.m*smin.value.n*smin.value.p) {
      //all cells are skipped
      return false;
    }
    swap(skippedCells,newSkippedCells);
  }
  //check the leaf cells
  for(Meshing::VolumeGrid::iterator c = a.data->data.getIterator();!c.isDone();++c) {
    c.getCell(cell);
    centerLocal = (cell.bmin + cell.bmax)*0.5;
    a.currentTransform.mul(centerLocal,center);
    IntTriple pindex;
    a.maxHierarchy[0].GetIndex(centerLocal,pindex);
    if(skippedCells(pindex)) {
      continue;        
    }
    Real da = *c;
    Vector3 cpb,gradb;
    Real db = b.Distance(center,cpb,gradb);
    if(db + da <= margin) { //sphere at center collides with b
      // cout<<"Collision found"<<endl;
      // cout<<"Cell "<<c.getIndex()<<" center "<<center<<endl;
      // cout<<"A depth "<<da<<"   B depth "<<db<<endl;

      elements1.push_back(a.PointToElement(cpb));
      elements2.push_back(b.PointToElement(cpb));
      if(elements1.size() >= maxContacts) {
        return true;
      }
    }
  }    
  return !elements1.empty();
}

bool Collider3DImplicitSurface::CollidesLocal(const Triangle3D& tri,Real margin,IntTriple& collides_cell) const
{
  Vector3 cellSize = data->data.GetCellSize();
  Real cellRad = cellSize.norm() * 0.5;

  //quickly test whether a sphere containing the triangle is within the margin
  Vector3 c = (tri.a + tri.b + tri.c) / 3.0;
  Real test = data->data.TrilinearInterpolate(c);
  Vector3 pt_clamped;
  test += data->data.bb.distance(c,pt_clamped);
  Real triDiam = Max(c.distance(tri.a),c.distance(tri.b),c.distance(tri.c));
  IntTriple collide_cell;
  if(test <= margin - cellRad) {
    data->data.GetIndexClamped(c,collide_cell);
    return true;
  }
  else if(test > margin + triDiam + cellRad) {
    return false;
  }
  else {
    //high-resolution checking
    //TODO: if margin extends implicit surface outside of bounding box, this may be incorrect 
    AABB3D center_bb = data->data.bb;
    center_bb.bmin += cellSize * 0.5;
    center_bb.bmax -= cellSize * 0.5;
    Meshing::TriMesh cube_mesh;
    cube_mesh.verts.reserve(8);

    Real cell_corner_values[8];
    vector<IntTriple> tcells;
    tcells.resize(0);
    Meshing::GetTriangleCells(tri,data->data.value.m-1,data->data.value.n-1,data->data.value.p-1,center_bb,tcells);
    for(const auto& cell: tcells) {
      cell_corner_values[0] = data->data.value(cell.a,cell.b,cell.c);
      if(cell_corner_values[0] > margin + cellRad) continue;
      if(cell_corner_values[0] <= margin - cellRad) {
        collide_cell = cell;
        return true;
      }
      cell_corner_values[1] = data->data.value(cell.a,cell.b,cell.c+1);
      cell_corner_values[2] = data->data.value(cell.a,cell.b+1,cell.c);
      cell_corner_values[3] = data->data.value(cell.a,cell.b+1,cell.c+1);
      cell_corner_values[4] = data->data.value(cell.a+1,cell.b,cell.c);
      cell_corner_values[5] = data->data.value(cell.a+1,cell.b,cell.c+1);
      cell_corner_values[6] = data->data.value(cell.a+1,cell.b+1,cell.c);
      cell_corner_values[7] = data->data.value(cell.a+1,cell.b+1,cell.c+1);
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
          collide_cell = cell;
          return true;
        }
      }
    }
  }
  return false;
}

bool Collider3DImplicitSurface::Collides(const CollisionMesh &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts) const
{
  Box3D bba,bbb;
  bba = GetBB();
  Geometry::GetBB(b,bbb);
  if(!bba.intersects(bbb)) return false;
  
  RigidTransform Tba;
  Tba.mulInverseA(currentTransform, b.currentTransform);
  Triangle3D tri;
  for(size_t i=0;i<b.tris.size();i++) {
    b.GetTriangle(i,tri);
    tri.a = Tba*tri.a;
    tri.b = Tba*tri.b;
    tri.c = Tba*tri.c;

    IntTriple collide_cell;
    if(CollidesLocal(tri,margin,collide_cell)) {
      elements1.push_back(collide_cell.a*data->data.value.n*data->data.value.p + collide_cell.b*data->data.value.p + collide_cell.c);
      elements2.push_back(int(i));
      if(elements1.size() >= maxContacts) {
        return true;
      }
    }
  }
  return !elements1.empty();
}


inline void Copy(const PQP_REAL p[3],Vector3& x)
{
  x.set(p[0],p[1],p[2]);
}

inline void Copy(const Vector3& x,PQP_REAL p[3])
{
  p[0] = x.x;
  p[1] = x.y;
  p[2] = x.z;
}

struct SDFDistanceCallback
{
  struct ActivePair
  {
    bool operator < (const ActivePair& rhs) const { return minDist < rhs.minDist; }
    int index;          //bound index
    Real minDist;       //minimum distance of bound b
  };

  SDFDistanceCallback()
    :grid(NULL),dmin(Inf),dmax(Inf),closestTri(-1), numTrianglesChecked(0), numBBsChecked(0)
  {}
  void Execute(const CollisionMesh& m, const Collider3DImplicitSurface* _grid) {
    grid = _grid;
    Assert(m.pqpModel->num_bvs != 0);
    //compute distance of random triangle -- perhaps faster pruning
    //int t = RandInt(m.num_tris);
    //compute distance of arbitrary triangle -- perhaps faster pruning
    int t = 0;
    Copy(m.pqpModel->tris[t].p1,tri.a);
    Copy(m.pqpModel->tris[t].p2,tri.b);
    Copy(m.pqpModel->tris[t].p3,tri.c);
    tri.a = m.currentTransform*tri.a;
    tri.b = m.currentTransform*tri.b;
    tri.c = m.currentTransform*tri.c;
    Real d = grid->Distance(tri,cp_grid,cp_mesh,direction_mesh_grid,dmin);
    if(d < dmin) {
      dmax = dmin = d;
      closestTri = m.pqpModel->tris[t].id;
    }
    numTrianglesChecked = 1;
    numBBsChecked = 0;
    ExecuteRecurse(m,0);
    return;
  }

  void ExecuteRecurse(const CollisionMesh& m,int b)
  {
    //which child to pick first?
    if(m.pqpModel->b[b].Leaf()) { //it's a triangle
      numTrianglesChecked++;
      int t = -m.pqpModel->b[b].first_child - 1;
      Assert(t < m.pqpModel->num_tris);
      //compute distance squared
      Copy(m.pqpModel->tris[t].p1,tri.a);
      Copy(m.pqpModel->tris[t].p2,tri.b);
      Copy(m.pqpModel->tris[t].p3,tri.c);
      Vector3 cp_grid_temp,cp_mesh_temp,direction_mesh_grid_temp;
      Real d = grid->Distance(tri,cp_grid_temp,cp_mesh_temp,direction_mesh_grid_temp,dmin);
      if(d < dmin) {
        dmax = dmin = d;
        cp_grid = cp_grid_temp;
        cp_mesh = cp_mesh_temp;
        direction_mesh_grid = direction_mesh_grid_temp;
        closestTri = m.pqpModel->tris[t].id;
      }
    }
    else {
      numBBsChecked++;
      int c1 = m.pqpModel->b[b].first_child;
      int c2 = c1+1;

      //pick the closest child to descend on first
      //quick reject: closest distance is less than dmin
      Real dbmin1,dbmax1;
      Real dbmin2,dbmax2;
      Vector3 center1,center2;
      Real radius1,radius2;
      Copy(m.pqpModel->b[c1].To,center1);
      center1 = m.currentTransform*center1;
      radius1 = Sqrt(Sqr(m.pqpModel->b[c1].d[0]) + Sqr(m.pqpModel->b[c1].d[1]) + Sqr(m.pqpModel->b[c1].d[2]));
      Copy(m.pqpModel->b[c2].To,center2);
      center2 = m.currentTransform*center2;
      radius2 = Sqrt(Sqr(m.pqpModel->b[c2].d[0]) + Sqr(m.pqpModel->b[c2].d[1]) + Sqr(m.pqpModel->b[c2].d[2]));
      dbmin1 = grid->Distance(center1) - radius1;
      dbmax1 = grid->Distance(center1) + radius1;
      dbmin2 = grid->Distance(center2) - radius2;
      dbmax2 = grid->Distance(center2) + radius2;
      //LOG4CXX_INFO(GET_LOGGER(Geometry),"Children "<<c1<<", "<<c2<<", distances "<<dbmin1<<", "<<dbmin2);
      bool reverse=false;
      if(dbmin2 == dbmin1) { //point is in the bboxes
        reverse = (dbmax2 < dbmax1);
      }
      else reverse = (dbmin2 < dbmin1);
      
      if(reverse) {
        if(dbmax2 < dmax) dmax = dbmax2;
        if(dbmax1 < dmax) dmax = dbmax1;
        if(dbmin2 < dmax) ExecuteRecurse(m,c2);
        if(dbmin1 < dmax) ExecuteRecurse(m,c1);
      }
      else {
        if(dbmax1 < dmax) dmax = dbmax1;
        if(dbmax2 < dmax) dmax = dbmax2;
        if(dbmin1 < dmax) ExecuteRecurse(m,c1);
        if(dbmin2 < dmax) ExecuteRecurse(m,c2);
      }
    }
  }

  const Collider3DImplicitSurface* grid;
  Real dmin,dmax;  //min squared distance, max squared distance bound
  int closestTri;
  Triangle3D tri;
  Vector3 cp_mesh,cp_grid;
  Vector3 direction_mesh_grid; //direction from mesh to grid at closest point

  int numTrianglesChecked,numBBsChecked;
};


Real Collider3DImplicitSurface::Distance(const CollisionMesh &b, Vector3 closestPtGrid, int& closestTri, Vector3& closestPtMesh, Real upperBound) const
{
  SDFDistanceCallback callback;
  callback.Execute(b,this);
  closestPtGrid = callback.cp_grid;
  closestPtMesh = callback.cp_mesh;
  closestTri = callback.closestTri;
  return callback.dmin;
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

int Geometry3DVolume::PointToElement(const Vector3 &pt) const
{
  IntTriple cell;
  data.GetIndexClamped(pt, cell);
  return IndexToElement(cell);
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

Geometry3D* Geometry3DImplicitSurface::ExtractROI(const AABB3D& bb,int flag) const
{
  if(flag & ExtractROIFlagInvert) return NULL;
  const Meshing::VolumeGrid& grid = data;
  auto* res = new Geometry3DImplicitSurface;
  IntTriple imin,imax;
  Vector3 umin,umax;
  grid.GetIndexAndParamsClamped(bb.bmin,imin,umin);
  grid.GetIndexAndParamsClamped(bb.bmax,imax,umax);
  if(flag & ExtractROIFlagWithin || flag & ExtractROIFlagTouching) {
    //get only cells entirely within or touching the bbox
    if(umax.x == 1) imax.a += 1;
    if(umax.y == 1) imax.b += 1;
    if(umax.z == 1) imax.c += 1;
    if(umin.x == 0) imin.a -= 1;
    if(umin.y == 0) imin.b -= 1;
    if(umin.z == 0) imin.c -= 1;
    if(flag & ExtractROIFlagWithin) {
      imax.a -= 1;
      imax.b -= 1;
      imax.c -= 1;
      imin.a += 1;
      imin.b += 1;
      imin.c += 1;
    }
    AABB3D cellmin,cellmax;
    grid.GetCell(imin,cellmin);
    grid.GetCell(imax,cellmax);
    res->data.bb.bmin = cellmin.bmin;
    res->data.bb.bmax = cellmax.bmax;
    if(imin.a > imax.a || imin.b > imax.b || imin.c > imax.c) {
      res->data.value.resize(0,0,0);
      return res;
    }
    res->data.Resize(imax.a-imin.a+1,imax.b-imin.b+1,imax.c-imin.c+1);
    for(int i=imin.a;i<=imax.a;i++)
      for(int j=imin.b;j<=imax.b;j++)
        for(int k=imin.c;k<=imax.c;k++)
          res->data.value(i-imin.a,j-imin.b,k-imin.c) = grid.value(i,j,k);
    return res;
  }
  else if(flag & ExtractROIFlagIntersection) {
    //need to resample
    AABB3D cellmin,cellmax;
    grid.GetCell(imin,cellmin);
    grid.GetCell(imax,cellmax);
    Vector3 bmin = cellmin.bmin;
    bmin.x += umin.x*(cellmax.bmin.x-cellmin.bmin.x); 
    bmin.y += umin.y*(cellmax.bmin.y-cellmin.bmin.y);
    bmin.z += umin.z*(cellmax.bmin.z-cellmin.bmin.z);
    Vector3 bmax = cellmax.bmax;
    bmax.x += umax.x*(cellmax.bmax.x-cellmin.bmax.x);
    bmax.y += umax.y*(cellmax.bmax.y-cellmin.bmax.y);
    bmax.z += umax.z*(cellmax.bmax.z-cellmin.bmax.z);
    res->data.bb.bmin = bmin;
    res->data.bb.bmax = bmax;
    Vector3 size=grid.GetCellSize();
    res->data.ResizeByResolution(size);
    res->data.ResampleTrilinear(grid);
    return res;
  }
  delete res;
  return NULL;
}


Collider3DImplicitSurface::Collider3DImplicitSurface(shared_ptr<Geometry3DImplicitSurface> _data)
:data(_data)
{
  currentTransform.setIdentity();
  Reset();
}

Collider3DImplicitSurface::Collider3DImplicitSurface(const Collider3DImplicitSurface& vg)
:data(vg.data),currentTransform(vg.currentTransform),minHierarchy(vg.minHierarchy),maxHierarchy(vg.maxHierarchy),resolutionMap(vg.resolutionMap)
{
}

Collider3D* Collider3DImplicitSurface::Copy(shared_ptr<Geometry3D> data) const
{
  auto* res = new Collider3DImplicitSurface(*this);
  res->data = dynamic_pointer_cast<Geometry3DImplicitSurface>(data);
  return res;
}
    

void Collider3DImplicitSurface::Reset()
{
  auto& baseGrid = data->data;
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
  printf("Collider3DImplicitSurface: %d resolutions ",(int)resolutionMap.size());
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

bool Collider3DImplicitSurface::Merge(Collider3D* geom)
{
  if(geom->GetType() == Type::TriangleMesh) {
    auto* mesh = dynamic_cast<Collider3DTriangleMesh*>(geom);
    //copy mesh collision data and transform it to this geometry's frame
    CollisionMesh temp = mesh->collisionData;
    RigidTransform Tg_this;
    Tg_this.mulInverseA(currentTransform, temp.currentTransform);
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
  result = Distance(pt);
  return true;
}

Real Collider3DImplicitSurface::DistanceLocal(const Vector3& pt_local) const
{
  Real sdf_value = data->data.TrilinearInterpolate(pt_local);
  Vector3 pt_clamped;
  Real d_bb = data->data.bb.distance(pt_local,pt_clamped);
  return sdf_value + d_bb;
}

Real Collider3DImplicitSurface::Distance(const Vector3& pt) const
{
  Vector3 ptlocal;
  currentTransform.mulInverse(pt,ptlocal);
  return DistanceLocal(ptlocal);
}

Real Collider3DImplicitSurface::Distance(const Vector3& pt,Vector3& surfacePt,Vector3& direction) const
{
  Vector3 ptlocal;
  currentTransform.mulInverse(pt,ptlocal);

  Real d=Geometry::DistanceSDF(data->data,ptlocal,&surfacePt,&direction);
  surfacePt = currentTransform*surfacePt;
  direction = currentTransform.R*direction;
  return d;
}

bool Collider3DImplicitSurface::Distance(const Vector3 &pt, const DistanceQuerySettings &settings,DistanceQueryResult& res)
{
    res.hasClosestPoints = true;
    res.hasElements = true;
    res.elem2 = 0;
    res.cp2 = pt;

    res.d = Distance(pt, res.cp1, res.dir2);
    res.dir1.setNegative(res.dir2);
    res.hasPenetration = true;
    res.hasDirections = true;
    res.elem1 = PointToElement(res.cp1);
    //cout<<"Doing ImplicitSurface - point collision detection, with direction "<<res.dir2<<endl;
    return true;
}

bool Collider3DImplicitSurface::Distance(Collider3D* geom, const DistanceQuerySettings &settings,DistanceQueryResult& res)
{
  switch (geom->GetType())
  {
  case Type::Primitive:
  {
    GeometricPrimitive3D bw = dynamic_cast<Collider3DPrimitive*>(geom)->data->data;
    bw.Transform(geom->GetTransform());
    res = Geometry::Distance(bw, *this, settings);
    if(!IsFinite(res.cp1.x)) {
      return false;
    }
    Flip(res);
    return true;
  }
  case Type::PointCloud:
  {
    auto& pc = dynamic_cast<Collider3DPointCloud*>(geom)->collisionData;
    res = Geometry::Distance(*this, pc, settings);
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
    if (Collides(bw, d, cpa))
    {
      if(!IsFinite(cpa.x)) return false;
      elements1.resize(1);
      elements1[0] = PointToElement(cpa);
      elements2.push_back(0);
    }
    if(!IsFinite(cpa.x)) return false;
    return true;
  }
  case Type::ImplicitSurface:
  {
        auto& b = *dynamic_cast<Collider3DImplicitSurface*>(geom);
        bool res=::Collides(*this, b, d, elements1, elements2, maxContacts);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size() == elements2.size());
        return true;
  }
  case Type::TriangleMesh:
  {
        auto& b = dynamic_cast<Collider3DTriangleMesh*>(geom)->collisionData;
        bool res = Collides(b, d, elements1, elements2, maxContacts);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size() == elements2.size());
        return true;
  }
  case Type::PointCloud:
  {
        auto& b = dynamic_cast<Collider3DPointCloud*>(geom)->collisionData;
        bool res = Collides(b, d, elements2, maxContacts);
        elements1.resize(elements2.size());
        for (size_t i = 0; i < elements2.size(); i++)
            elements1[i] = PointToElement(b.currentTransform * b.points[elements2[i]]);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size() == elements2.size());
        return true;
  }
  default:
    return false;
  }
  return false;
}


bool Collider3DImplicitSurface::RayCast(const Ray3D& r,Real margin,Real& distance, int& element)
{
    distance = RayCast(r,margin);
    if(!IsInf(distance)) {
        element = PointToElement(r.source+distance*r.direction);
        return true;
    }
    return true;
}

void PointCloudImplicitSurfaceContacts(CollisionPointCloud &pc1, Real outerMargin1, Collider3DImplicitSurface &s2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  s2.Collides(pc1, tol, points, maxcontacts);
  contacts.reserve(points.size());
  for (size_t j = 0; j < points.size(); j++)
  {
    Vector3 p1w = pc1.currentTransform * pc1.points[points[j]];
    Vector3 p2w;
    Vector3 n;
    Real d = s2.Distance(p1w, p2w, n);
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = p1w + n * outerMargin1;
    contacts[k].p2 = p2w - n * outerMargin2;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points[j];
    contacts[k].elem2 = s2.PointToElement(p2w);
    contacts[k].unreliable = false;
  }
}

void TriangleMeshImplicitSurfaceContacts(CollisionMesh &m1, Real outerMargin1, Collider3DImplicitSurface &s2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  Triangle3D tri;
  Vector3 surfacePt, triPt, direction;
  for(size_t i=0;i<m1.tris.size();i++) {
    m1.GetTriangle(i,tri);
    Real d=s2.Distance(tri,surfacePt,triPt,direction,tol+Epsilon);
    if(d <= tol) {
      size_t k = contacts.size();
      contacts.resize(k + 1);
      contacts[k].p1 = triPt + direction * outerMargin1;
      contacts[k].p2 = surfacePt - direction * outerMargin2;
      contacts[k].n = direction;
      contacts[k].depth = tol - d;
      contacts[k].elem1 = i;
      contacts[k].elem2 = s2.PointToElement(surfacePt);
      contacts[k].unreliable = false;
    }
  }
}

void ImplicitSurfaceSphereContacts(Collider3DImplicitSurface &s1, Real outerMargin1, const Sphere3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cp, dir;
  Real d = s1.Distance(s.center, cp, dir) - s.radius;
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
  contacts[0].elem1 = s1.PointToElement(cp);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfaceSegmentContacts(Collider3DImplicitSurface &s1, Real outerMargin1, const Segment3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cpgrid, cpseg, dir;
  Real d = s1.Distance(GeometricPrimitive3D(s), cpgrid, cpseg, dir);
  if (d > tol)
    return;
  contacts.resize(1);
  Vector3 n;
  n.setNegative(dir);
  contacts[0].p1 = cpgrid + outerMargin1 * n;
  contacts[0].p2 = cpseg - outerMargin2 * n;
  contacts[0].n = n;
  contacts[0].depth = tol - d;
  contacts[0].elem1 = s1.PointToElement(cpgrid);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfaceTriangleContacts(Collider3DImplicitSurface &s1, Real outerMargin1, const Triangle3D &t, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cpgrid, cpseg, dir;
  Real d = s1.Distance(GeometricPrimitive3D(t), cpgrid, cpseg, dir);
  if (d > tol)
    return;
  contacts.resize(1);
  Vector3 n;
  n.setNegative(dir);
  contacts[0].p1 = cpgrid + outerMargin1 * n;
  contacts[0].p2 = cpseg - outerMargin2 * n;
  contacts[0].n = n;
  contacts[0].depth = tol - d;
  contacts[0].elem1 = s1.PointToElement(cpgrid);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfacePrimitiveContacts(Collider3DImplicitSurface &s1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
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
        ImplicitSurfacePrimitiveContacts(*this, settings.padding1, prim->data->data, prim->T, settings.padding2, res.contacts, settings.maxcontacts);
        return true;
      }
    case Type::TriangleMesh:
      {
        auto* m = dynamic_cast<Collider3DTriangleMesh*>(other);
        TriangleMeshImplicitSurfaceContacts(m->collisionData, settings.padding2, *this, settings.padding1, res.contacts, settings.maxcontacts);
        for (auto &c : res.contacts)
          ReverseContact(c);
      }
      return true;
    case Type::PointCloud:
      {
        auto* pc = dynamic_cast<Collider3DPointCloud*>(other);
        PointCloudImplicitSurfaceContacts(pc->collisionData, settings.padding2, *this, settings.padding1, res.contacts, settings.maxcontacts);
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


