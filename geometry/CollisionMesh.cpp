#include "CollisionMesh.h"
#include "PenetrationDepth.h"
#include "GridSubdivision.h"
#include <KrisLibrary/Logger.h>
#include <math3d/clip.h>
#include <iostream>
#include "Conversions.h"
#include "ROI.h"
#include "Slice.h"
#include <KrisLibrary/meshing/IO.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/meshing/Meshing.h>
#include "CollisionPrimitive.h"
#include "CollisionPointCloud.h"
#include "CollisionConvexHull.h"
using namespace Meshing;
using namespace std;

const static Real third = 1.0/3.0;

#include "PQP/include/PQP.h"
#include "PQP/src/MatVec.h"
#include "PQP/src/OBB_Disjoint.h"
#include "PQP/src/TriDist.h"

class PQP_Results
{
public:
  PQP_CollideResult collide;
  PQP_DistanceResult distance;
  PQP_ToleranceResult tolerance;
  PQP_ToleranceAllResult toleranceAll;
};

DECLARE_LOGGER(Geometry)



namespace Geometry {

typedef ContactsQueryResult::ContactPair ContactPair;

void RigidTransformToPQP(const RigidTransform& f,PQP_REAL R[3][3],PQP_REAL T[3])
{
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++)
      R[i][j] = f.R(i,j);
    T[i] = f.t[i];
  }
}

void PQPToRigidTransform(const PQP_REAL R[3][3],const PQP_REAL T[3],RigidTransform& f)
{
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++)
      f.R(i,j) = R[i][j];
    f.t[i] = T[i];
  }
}

void ConvertTriToPQP(const TriMesh& tri, PQP_Model& pqp)
{
  PQP_REAL p1[3],p2[3],p3[3];

  pqp.BeginModel(tri.tris.size());
  for(size_t i=0;i<tri.tris.size();i++) {
    const Vector3& v1 = tri.TriangleVertex(i,0);
    const Vector3& v2 = tri.TriangleVertex(i,1);
    const Vector3& v3 = tri.TriangleVertex(i,2);
    p1[0] = v1.x;     p1[1] = v1.y;     p1[2] = v1.z; 
    p2[0] = v2.x;     p2[1] = v2.y;     p2[2] = v2.z; 
    p3[0] = v3.x;     p3[1] = v3.y;     p3[2] = v3.z; 
    pqp.AddTri(p1,p2,p3,i);
  }
  pqp.EndModel();
}



CollisionMesh::CollisionMesh()
{
  pqpModel=NULL;
  currentTransform.setIdentity();
}

CollisionMesh::CollisionMesh(const Meshing::TriMesh& mesh)
{
  pqpModel=NULL;
  verts = mesh.verts;
  tris = mesh.tris;
  currentTransform.setIdentity();
  InitCollisions();
}

CollisionMesh::CollisionMesh(const Meshing::TriMeshWithTopology& mesh)
{
  pqpModel=NULL;
  TriMeshWithTopology::operator = (mesh);
  currentTransform.setIdentity();
  InitCollisions();
}

CollisionMesh::CollisionMesh(const CollisionMesh& model)
{
  pqpModel=NULL;
  operator = (model);
}

CollisionMesh::~CollisionMesh()
{
  SafeDelete(pqpModel)
}

void CollisionMesh::InitCollisions()
{
  SafeDelete(pqpModel);
  if(!tris.empty()) {
    pqpModel = new PQP_Model;
    ConvertTriToPQP(*this,*pqpModel);
    CalcVertexNeighbors();
    CalcTriNeighbors();
  }
}

void CopyPQPModel(const PQP_Model* source, PQP_Model* dest)
{
  dest->build_state=source->build_state;

  dest->num_tris = source->num_tris;
  dest->num_tris_alloced = source->num_tris;
  dest->tris = new Tri[source->num_tris];  
  for(int i=0;i<source->num_tris;i++)
    dest->tris[i] = source->tris[i];

  dest->num_bvs = source->num_bvs;
  dest->num_bvs_alloced = source->num_bvs;
  dest->b = new BV[source->num_bvs];  
  for(int i=0;i<source->num_bvs;i++)
    dest->b[i] = source->b[i];
}

const CollisionMesh& CollisionMesh::operator = (const CollisionMesh& model)
{
  SafeDelete(pqpModel);
  TriMeshWithTopology::operator = (model);
  currentTransform.setIdentity();
  if(!tris.empty()) {
    pqpModel = new PQP_Model;
    CopyPQPModel(model.pqpModel,pqpModel);
  }
  currentTransform = model.currentTransform;

  return *this;
}


CollisionMeshQuery::CollisionMeshQuery()
  :m1(NULL),m2(NULL),
   penetration1(NULL),penetration2(NULL)
{
  pqpResults = new PQP_Results;
  pqpResults->distance.t1 = 0;
  pqpResults->distance.t2 = 0;
}

CollisionMeshQuery::CollisionMeshQuery(const CollisionMesh& _m1, const CollisionMesh& _m2)
  :m1(&_m1),m2(&_m2),
   penetration1(NULL),penetration2(NULL)
{
  pqpResults = new PQP_Results;
  pqpResults->distance.t1 = 0;
  pqpResults->distance.t2 = 0;
}

CollisionMeshQuery::CollisionMeshQuery(const CollisionMeshQuery& q)
  :m1(q.m1),m2(q.m2),
   penetration1(NULL),penetration2(NULL)
{
  pqpResults = new PQP_Results;
  pqpResults->distance.t1 = 0;
  pqpResults->distance.t2 = 0;
  //*pqpResults = *q.pqpResults;
}

const CollisionMeshQuery& CollisionMeshQuery::operator =(const CollisionMeshQuery& q)
{
  m1 = q.m1;
  m2 = q.m2;
  //*pqpResults = *q.pqpResults;
  pqpResults->distance.t1 = 0;
  pqpResults->distance.t2 = 0;
  SafeDelete(penetration1);
  SafeDelete(penetration2);
  return *this;
}


CollisionMeshQuery::~CollisionMeshQuery()
{
  delete pqpResults;
  SafeDelete(penetration1);
  SafeDelete(penetration2);
}

bool CollisionMeshQuery::Collide()
{
  if(m1->tris.empty() || m2->tris.empty()) return false;
  if(m1->pqpModel == NULL || m2->pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1->currentTransform,R1,T1);
  RigidTransformToPQP(m2->currentTransform,R2,T2);
  int res=PQP_Collide(&pqpResults->collide,
		      R1,T1,m1->pqpModel,
		      R2,T2,m2->pqpModel,
		      PQP_FIRST_CONTACT);
  Assert(res == PQP_OK);
  return (pqpResults->collide.Colliding()!=0);
}

bool CollisionMeshQuery::CollideAll()
{
  if(m1->tris.empty() || m2->tris.empty()) return false;
  if(m1->pqpModel == NULL || m2->pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1->currentTransform,R1,T1);
  RigidTransformToPQP(m2->currentTransform,R2,T2);
  int res=PQP_Collide(&pqpResults->collide,
		      R1,T1,m1->pqpModel,
		      R2,T2,m2->pqpModel,
		      PQP_ALL_CONTACTS);
  Assert(res == PQP_OK);
  return (pqpResults->collide.Colliding()!=0);
}

Real CollisionMeshQuery::Distance(Real absErr,Real relErr,Real bound)
{
  if(m1->tris.empty() || m2->tris.empty()) return Inf;
  if(m1->pqpModel == NULL || m2->pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1->currentTransform,R1,T1);
  RigidTransformToPQP(m2->currentTransform,R2,T2);
  if(IsInf(bound)) bound=-1;
  int res = PQP_Distance(&pqpResults->distance,
			 R1,T1,m1->pqpModel,
			 R2,T2,m2->pqpModel,
			 relErr,absErr,
			 100,bound);
  Assert(res == PQP_OK);
  return pqpResults->distance.Distance();
}

Real CollisionMeshQuery::Distance_Coherent(Real absErr,Real relErr,Real bound)
{
  if(m1->tris.empty() || m2->tris.empty()) return Inf;
  if(m1->pqpModel == NULL || m2->pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1->currentTransform,R1,T1);
  RigidTransformToPQP(m2->currentTransform,R2,T2);
  if(IsInf(bound)) bound=-1;
  int res = PQP_Distance(&pqpResults->distance,
			 R1,T1,m1->pqpModel,
			 R2,T2,m2->pqpModel,
			 relErr,absErr,
			 2,bound);
  Assert(res == PQP_OK);
  return pqpResults->distance.Distance();
}

Real CollisionMeshQuery::PenetrationDepth()
{
  if(!CollideAll()) return -Zero;
  int n = pqpResults->collide.NumPairs();
  if(n == 0) return -Zero;
  tc1.resize(n);
  tc2.resize(n);
  for(int i=0;i<n;i++) {
    tc1[i] = pqpResults->collide.Id1(i);
    tc2[i] = pqpResults->collide.Id2(i);
  }

  if(!penetration1) penetration1 = new ApproximatePenetrationDepth(*m1,*m2);
  if(!penetration2) penetration2 = new ApproximatePenetrationDepth(*m2,*m1);
  penetration1->Reset();
  penetration1->ComputeInitial(m1->currentTransform,m2->currentTransform,&tc1[0],&tc2[0],n);
  penetration1->ComputeDepth();
  penetration2->Reset();
  penetration2->ComputeInitial(m2->currentTransform,m1->currentTransform,&tc2[0],&tc1[0],n);
  penetration2->ComputeDepth();
  if(penetration1->maxDepth <= 0 && penetration2->maxDepth <= 0) {
    Real d=Distance(1e-3,1e-2);
    if(d > 1e-3) {
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"PenetrationDepth(): Error, the two objects aren't penetrating?!?!");
      LOG4CXX_INFO(GET_LOGGER(Geometry),"Distance "<<d);
      Abort();
    }
    LOG4CXX_WARN(GET_LOGGER(Geometry),"PenetrationDepth(): Warning, the approximate computation failed, returning "<<Max(-d,(Real)1e-5));
    //KrisLibrary::loggerWait();
    return Max(-d,(Real)1e-5);
  }
  if(penetration1->maxDepth <= 0) return penetration2->maxDepth;
  else if(penetration2->maxDepth <= 0) return penetration1->maxDepth;
  else return Min(penetration1->maxDepth,penetration2->maxDepth);
}

void CollisionMeshQuery::CollisionPairs(vector<int>& t1,vector<int>& t2) const
{ 
  int n=pqpResults->collide.NumPairs();
  t1.resize(n);
  t2.resize(n);
  for(int i=0;i<n;i++) {
    t1[i] = pqpResults->collide.Id1(i);
    t2[i] = pqpResults->collide.Id2(i);
  }
}

void CollisionMeshQuery::TolerancePairs(vector<int>& t1,vector<int>& t2) const
{
  const PQP_ToleranceAllResult& allRes = pqpResults->toleranceAll;
  t1.resize(0);
  t2.resize(0);
  //fill out pairs from the results
  /*
  for(size_t i=0;i<allRes.triPartner1.size();i++)
    if(allRes.triPartner1[i] >= 0) {
      t1.push_back(i);
      t2.push_back(allRes.triPartner1[i]);
    }
  for(size_t i=0;i<allRes.triPartner2.size();i++) {
    if(allRes.triPartner2[i] >= 0 && allRes.triPartner1[allRes.triPartner2[i]] != (int)i) { //avoid double counting  
      t1.push_back(allRes.triPartner2[i]);
      t2.push_back(i);
    }
  }
  */
  for(map<int,int>::const_iterator i=allRes.triPartner1.begin();i!=allRes.triPartner1.end();++i) {
    t1.push_back(i->first);
    t2.push_back(i->second);
  }
  for(map<int,int>::const_iterator i=allRes.triPartner2.begin();i!=allRes.triPartner2.end();++i) {
    if(allRes.triPartner1.find(i->second)->second != i->first) { //avoid double counting  
      t1.push_back(i->second);
      t2.push_back(i->first);
    }
  }
}


//returns true if the triangles a and b on m1 and m2, respectively, collide.  If so, p1 and p2 are set to the
//respective local coordinates of a point on the most deeply colliding region
bool OverlappingTriangleCollision(const CollisionMesh* m1,const CollisionMesh* m2,int a,int b,Vector3& p1,Vector3& p2)
{
  Triangle3D tri1,tri2,tri2loc;
  m1->GetTriangle(a,tri1);
  m2->GetTriangle(b,tri2);
  const RigidTransform& T1=m1->currentTransform;
  const RigidTransform& T2=m2->currentTransform;
  RigidTransform T21;
  T21.mulInverseA(T1,T2);
  tri2loc.a = T21*tri2.a;
  tri2loc.b = T21*tri2.b;
  tri2loc.c = T21*tri2.c;
  Segment3D s;
  if(tri2loc.intersects(tri1,s)) { 
    p1 = (s.a+s.b)*0.5;
    T2.mulInverse(T1*p1,p2);
    return true;
  }
  return false;
}

void CollisionMeshQuery::TolerancePoints(vector<Vector3>& p1,vector<Vector3>& p2) const
{
  const PQP_ToleranceAllResult& allRes = pqpResults->toleranceAll;
  p1.resize(0);
  p2.resize(0);
  //fill out pairs from the results
  /*
  for(size_t i=0;i<allRes.triPartner1.size();++i)
    if(allRes.triPartner1[i] >= 0) {
      p1.push_back(allRes.triCp1[i].first);
      p2.push_back(allRes.triCp1[i].second);
    }
  for(size_t i=0;i<allRes.triPartner2.size();++i) {
    if(allRes.triPartner2[i] >= 0 && allRes.triPartner1[allRes.triPartner2[i]] != (int)i) { //avoid double counting  
      p1.push_back(allRes.triCp2[i].first);
      p2.push_back(allRes.triCp2[i].second);
    }
  }
  */
  for(map<int,PQP_ClosestPoints >::const_iterator i=allRes.triCp1.begin();i!=allRes.triCp1.end();++i) {
    if(allRes.triDist1.find(i->first)->second == 0) {
      //overlapping triangles
      //NOTE: if any triangles are overlapping, PQP gives junk results for the pairs of points, not the intersecting points!
      Vector3 pl1,pl2;
      if(OverlappingTriangleCollision(m1,m2,i->first,allRes.triPartner1.find(i->first)->second,pl1,pl2)) {
        p1.push_back(pl1);
        p2.push_back(pl2);
      }
      else {
        LOG4CXX_WARN(GET_LOGGER(Geometry),"Warning, PQP detected overlapping triangles but we find they don't intersect?");
        continue;
      }
    }
    else {
      p1.push_back(Vector3(i->second.p1));
      p2.push_back(Vector3(i->second.p2));
    }
  }
  for(map<int,int>::const_iterator i=allRes.triPartner2.begin();i!=allRes.triPartner2.end();++i) {
    if(allRes.triPartner1.find(i->second)->second != i->first) { //avoid double counting  
      if(allRes.triDist2.find(i->first)->second == 0) {
        //overlapping triangles
        //NOTE: if any triangles are overlapping, PQP gives junk results for the pairs of points, not the intersecting points!
        Vector3 pl1,pl2;
        if(OverlappingTriangleCollision(m1,m2,allRes.triPartner2.find(i->first)->second,i->first,pl1,pl2)) {
          p1.push_back(pl1);
          p2.push_back(pl2);
        }
        else {
          LOG4CXX_WARN(GET_LOGGER(Geometry),"Warning, PQP detected overlapping triangles but we find they don't intersect?");
          continue;
        }
      }
      else {
        Assert(allRes.triCp2.find(i->first) != allRes.triCp2.end());
        p1.push_back(Vector3(allRes.triCp2.find(i->first)->second.p1));
        p2.push_back(Vector3(allRes.triCp2.find(i->first)->second.p2));
      }
    }
  }
}

Real CollisionMeshQuery::Distance_Cached() const
{
  return pqpResults->distance.Distance();
}


Real CollisionMeshQuery::PenetrationDepth_Cached() const
{
  if(penetration1->maxDepth <= 0 && penetration2->maxDepth <= 0) {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"PenetrationDepth_Cached(): Error, the two objects have no interior vertices!");
    Abort();
  }
  if(penetration1->maxDepth <= 0) return penetration2->maxDepth;
  else if(penetration2->maxDepth <= 0) return penetration1->maxDepth;
  else return Min(penetration1->maxDepth,penetration2->maxDepth);
}

inline void VEC3_COPY(PQP_REAL* res,const PQP_REAL* in)
{
  res[0] = in[0];
  res[1] = in[1];
  res[2] = in[2];
}

bool CollisionMeshQuery::WithinDistance(Real tol)
{
  if(m1->tris.empty() || m2->tris.empty()) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1->currentTransform,R1,T1);
  RigidTransformToPQP(m2->currentTransform,R2,T2);
  int res = PQP_Tolerance(&pqpResults->tolerance,
			 R1,T1,m1->pqpModel,
			 R2,T2,m2->pqpModel,
			 tol);
  Assert(res == PQP_OK);
  ///in case CollisionMeshQueryEnhanced is used to query TolerancePoints/TolerancePairs
  pqpResults->toleranceAll.triDist1.clear();
  pqpResults->toleranceAll.triDist2.clear();
  pqpResults->toleranceAll.triPartner1.clear();
  pqpResults->toleranceAll.triPartner2.clear();
  pqpResults->toleranceAll.triCp1.clear();
  pqpResults->toleranceAll.triCp2.clear();
  pqpResults->toleranceAll.triDist1[pqpResults->tolerance.tid1] = pqpResults->tolerance.distance;
  pqpResults->toleranceAll.triDist2[pqpResults->tolerance.tid2] = pqpResults->tolerance.distance;
  pqpResults->toleranceAll.triPartner1[pqpResults->tolerance.tid1] = pqpResults->tolerance.tid2;
  pqpResults->toleranceAll.triPartner2[pqpResults->tolerance.tid2] = pqpResults->tolerance.tid1;
  VEC3_COPY(pqpResults->toleranceAll.triCp1[pqpResults->tolerance.tid1].p1,pqpResults->tolerance.p1);
  VEC3_COPY(pqpResults->toleranceAll.triCp1[pqpResults->tolerance.tid1].p2,pqpResults->tolerance.p2);
  VEC3_COPY(pqpResults->toleranceAll.triCp2[pqpResults->tolerance.tid2].p1,pqpResults->tolerance.p1);
  VEC3_COPY(pqpResults->toleranceAll.triCp2[pqpResults->tolerance.tid2].p2,pqpResults->tolerance.p2);
  return pqpResults->tolerance.CloserThanTolerance();
}

bool CollisionMeshQuery::WithinDistanceAll(Real tol)
{
  if(m1->tris.empty() || m2->tris.empty()) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1->currentTransform,R1,T1);
  RigidTransformToPQP(m2->currentTransform,R2,T2);
  int res = PQP_ToleranceAll(&pqpResults->tolerance,
			   R1,T1,m1->pqpModel,
			   R2,T2,m2->pqpModel,
			   tol,pqpResults->toleranceAll);
  Assert(res == PQP_OK);
  return pqpResults->tolerance.CloserThanTolerance();
}

void CollisionMeshQuery::ClosestPoints(Vector3& p1,Vector3& p2) const
{
  const PQP_REAL* P1 = pqpResults->distance.P1();
  const PQP_REAL* P2 = pqpResults->distance.P2();
  p1.set(P1[0],P1[1],P1[2]);
  p2.set(P2[0],P2[1],P2[2]);
  //these are in local coords
}

void CollisionMeshQuery::ClosestPair(int& t1,int& t2) const
{
  t1 = pqpResults->distance.tid1;
  t2 = pqpResults->distance.tid2;
}

void CollisionMeshQuery::TolerancePoints(Vector3& p1,Vector3& p2) const
{
  const PQP_REAL* P1 = pqpResults->tolerance.P1();
  const PQP_REAL* P2 = pqpResults->tolerance.P2();
  p1.set(P1[0],P1[1],P1[2]);
  p2.set(P2[0],P2[1],P2[2]);
}


void CollisionMeshQuery::TolerancePair(int& t1,int& t2) const
{
  t1 = pqpResults->tolerance.tid1;
  t2 = pqpResults->tolerance.tid2;
}


//d1 is the direction that m2 can move to get out of m1
void CollisionMeshQuery::PenetrationPoints(Vector3& p1,Vector3& p2,Vector3& d1) const
{
  const RigidTransform& f1=m1->currentTransform;
  const RigidTransform& f2=m2->currentTransform;
  if(penetration1->maxDepth > 0) p1 = penetration1->deepestPoint;
  if(penetration2->maxDepth > 0) p2 = penetration2->deepestPoint;
  if(penetration1->maxDepth <= 0 && penetration2->maxDepth <= 0) {
    LOG4CXX_WARN(GET_LOGGER(Geometry),"PenetrationPoints(): Warning, the two objects have no interior vertices!  Results are undefined");
    //TODO : estimate penetration points/distance using collision pairs
    //closest points between m1 and m2
    Real closestDist = Inf, d;
    Triangle3D t1,t2;
    Vector3 v;
    for(size_t i=0;i<tc1.size();i++) {
      m1->GetTriangle(tc1[i],t1);
      m2->GetTriangle(tc2[i],t2);
      v = t2.closestPoint(t1.a); d = t1.a.distance(v);
      if(d < closestDist) { p1=t1.a; p2=v; closestDist=d; }
      v = t2.closestPoint(t1.b); d = t1.b.distance(v);
      if(d < closestDist) { p1=t1.b; p2=v; closestDist=d; }
      v = t2.closestPoint(t1.c); d = t1.c.distance(v);
      if(d < closestDist) { p1=t1.c; p2=v; closestDist=d; }
      
      v = t1.closestPoint(t2.a); d = t2.a.distance(v);
      if(d < closestDist) { p1=v; p2=t2.a; closestDist=d; }
      v = t1.closestPoint(t2.b); d = t2.b.distance(v);
      if(d < closestDist) { p1=v; p2=t2.b; closestDist=d; }
      v = t1.closestPoint(t2.c); d = t2.c.distance(v);
      if(d < closestDist) { p1=v; p2=t2.c; closestDist=d; }
    }
    
    d1 = p2-p1;
    d1.inplaceNormalize();
    LOG4CXX_INFO(GET_LOGGER(Geometry),"Returning closest points "<<p1<<", "<<p2<<", dir "<<d1);
    //KrisLibrary::loggerWait();
    return;
  }
  else {
    Real pen1 = (penetration1->maxDepth <= 0 ? Inf: penetration1->maxDepth);
    Real pen2 = (penetration2->maxDepth <= 0 ? Inf: penetration2->maxDepth);
    if(pen1 < pen2) {
      f1.mulVector(penetration1->deepestNormal,d1);
      Vector3 pt = p1; pt.madd(penetration1->deepestNormal,penetration1->maxDepth);
      Vector3 ptworld;
      f1.mulPoint(pt,ptworld);
      f2.mulPointInverse(ptworld,p2);
    }
    else {
      f2.mulVector(penetration2->deepestNormal,d1);
      d1.inplaceNegative();
      Vector3 pt = p2; pt.madd(penetration2->deepestNormal,penetration2->maxDepth);
      Vector3 ptworld;
      f2.mulPoint(pt,ptworld);
      f1.mulPointInverse(ptworld,p1);
    }
  }
}








CollisionMeshQueryEnhanced::CollisionMeshQueryEnhanced()
  :margin1(0),margin2(0)
{}

CollisionMeshQueryEnhanced::CollisionMeshQueryEnhanced(const CollisionMesh& _m1, const CollisionMesh& _m2)
  :CollisionMeshQuery(_m1,_m2),margin1(0),margin2(0)
{
}

CollisionMeshQueryEnhanced::CollisionMeshQueryEnhanced(const CollisionMeshQueryEnhanced& q)
  :CollisionMeshQuery(q),margin1(q.margin1),margin2(q.margin2)
{
}

CollisionMeshQueryEnhanced::~CollisionMeshQueryEnhanced()
{
}

bool CollisionMeshQueryEnhanced::Collide()
{
  Assert(margin1 >= 0 && margin2 >= 0);
  if(margin1 + margin2 > 0) 
    return CollisionMeshQuery::WithinDistance(margin1+margin2);
  else
    return CollisionMeshQuery::Collide();
}

bool CollisionMeshQueryEnhanced::CollideAll()
{
  Assert(margin1 >= 0 && margin2 >= 0);
  if(margin1 + margin2 > 0) 
    return CollisionMeshQuery::WithinDistanceAll(margin1+margin2);
  else
    return CollisionMeshQuery::CollideAll();
}

bool CollisionMeshQueryEnhanced::WithinDistance(Real tol)
{
  Assert(tol > 0);
  Assert(margin1 >= 0 && margin2 >= 0);
  return CollisionMeshQuery::WithinDistance(margin1+margin2+tol);
}

bool CollisionMeshQueryEnhanced::WithinDistanceAll(Real tol)
{
  Assert(tol > 0);
  Assert(margin1 >= 0 && margin2 >= 0);
  return CollisionMeshQuery::WithinDistanceAll(margin1+margin2+tol);
}

Real CollisionMeshQueryEnhanced::Distance(Real absErr,Real relErr,Real bound)
{
  Assert(margin1 >= 0 && margin2 >= 0);
  if(bound >= 0) bound += margin1+margin2;
  Real d0 = CollisionMeshQuery::Distance(absErr,relErr,bound);
  return d0-margin1-margin2;
}

Real CollisionMeshQueryEnhanced::Distance_Coherent(Real absErr,Real relErr,Real bound)
{
  Assert(margin1 >= 0 && margin2 >= 0);
  if(bound >= 0) bound += margin1+margin2;
  Real d0 = CollisionMeshQuery::Distance_Coherent(absErr,relErr,bound);
  return d0-margin1-margin2;
}


Real CollisionMeshQueryEnhanced::PenetrationDepth()
{
  if(margin1 + margin2 == 0) return CollisionMeshQuery::PenetrationDepth();
  else {
    Real d0=CollisionMeshQuery::PenetrationDepth();
    if(d0 < 0) {  //it's outside
      Real d=CollisionMeshQuery::Distance(0,0,margin1+margin2);
      if(d > margin1+margin2) return -Zero;
      return margin1+margin2-d;
    }
    else {
      return margin1+margin2+d0;
    }
  }
}

void CollisionMeshQueryEnhanced::CollisionPairs(vector<int>& t1,vector<int>& t2) const
{ 
  if(margin1 + margin2 > 0) {
    CollisionMeshQuery::TolerancePairs(t1,t2);
  }
  else 
    CollisionMeshQuery::CollisionPairs(t1,t2);
}

void CollisionMeshQueryEnhanced::TolerancePairs(vector<int>& t1,vector<int>& t2) const
{
  CollisionMeshQuery::TolerancePairs(t1,t2);
}

void CollisionMeshQueryEnhanced::TolerancePoints(vector<Vector3>& p1,vector<Vector3>& p2) const
{
  CollisionMeshQuery::TolerancePoints(p1,p2);
  if(margin1 + margin2 > 0) {
    for(size_t i=0;i<p1.size();i++) {
      Vector3 p1w = m1->currentTransform*p1[i];
      Vector3 p2w = m2->currentTransform*p2[i];
      Vector3 d=p2w-p1w;
      Real dn = d.norm();
      if(dn != 0) {
	p1w += d*(margin1/dn);
	p2w -= d*(margin2/dn);
	m1->currentTransform.mulInverse(p1w,p1[i]);
	m2->currentTransform.mulInverse(p2w,p2[i]);
      }
    }
  }
}

Real CollisionMeshQueryEnhanced::PenetrationDepth_Cached() const
{
  //TODO: what about when only the margins overlap?
  if(margin1 + margin2 > 0 ) {
    FatalError("Can't do PenetrationDepth_Cached yet");
    return 0;
  }
  else {
    return CollisionMeshQuery::PenetrationDepth_Cached();
  }
}


void CollisionMeshQueryEnhanced::ClosestPoints(Vector3& p1,Vector3& p2) const
{ 
  CollisionMeshQuery::ClosestPoints(p1,p2);
  if(margin1 + margin2 > 0) {
    Vector3 p1w = m1->currentTransform*p1;
    Vector3 p2w = m2->currentTransform*p2;
    Vector3 d=p2w-p1w;
    Real dn = d.norm();
    if(dn != 0) {
      p1w += d*(margin1/dn);
      p2w -= d*(margin2/dn);
      m1->currentTransform.mulInverse(p1w,p1);
      m2->currentTransform.mulInverse(p2w,p2);
    }
  }
}


void CollisionMeshQueryEnhanced::TolerancePoints(Vector3& p1,Vector3& p2) const
{
  CollisionMeshQuery::TolerancePoints(p1,p2);
  if(margin1 + margin2 > 0) {
    Vector3 p1w = m1->currentTransform*p1;
    Vector3 p2w = m2->currentTransform*p2;
    Vector3 d=p2w-p1w;
    Real dn = d.norm();
    if(dn != 0) {
      p1w += d*(margin1/dn);
      p2w -= d*(margin2/dn);
      m1->currentTransform.mulInverse(p1w,p1);
      m2->currentTransform.mulInverse(p2w,p2);
    }
  }
}


//d1 is the direction that m2 can move to get out of m1
void CollisionMeshQueryEnhanced::PenetrationPoints(Vector3& p1,Vector3& p2,Vector3& d1) const
{
  if(margin1+margin2 == 0)
    CollisionMeshQuery::PenetrationPoints(p1,p2,d1);
  else {
    FatalError("TODO: PenetrationPoints with margin\n");
  }
}











inline Real distance(const Segment3D& s,const Point3D& p)
{
  Vector3 temp;
  s.closestPoint(p,temp);
  return p.distance(temp);
}

bool Collide(const Meshing::TriMesh& m1,const Meshing::TriMesh& m2)
{
  CollisionMesh cm1(m1),cm2(m2);
  return Collide(cm1,cm2);
}

bool WithinDistance(const Meshing::TriMesh& m1,const Meshing::TriMesh& m2,Real tol)
{
  CollisionMesh cm1(m1),cm2(m2);
  CollisionMeshQuery q(cm1,cm2);
  return q.WithinDistance(tol);
}

Real Distance(const Meshing::TriMesh& m1,const Meshing::TriMesh& m2,Real absErr,Real relErr)
{
  CollisionMesh cm1(m1),cm2(m2);
  CollisionMeshQuery q(cm1,cm2);
  return q.Distance(absErr,relErr);
}

Real ClosestPoints(const Meshing::TriMesh& m1,const Meshing::TriMesh& m2,Real absErr,Real relErr,Vector3& p1,Vector3& p2)
{
  CollisionMesh cm1(m1),cm2(m2);
  CollisionMeshQuery q(cm1,cm2);
  Real d = q.Distance(absErr,relErr);
  q.ClosestPoints(p1,p2);
  return d;
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


void BVToBox(const BV& b,Box3D& box)
{
  Copy(b.d,box.dims);
  Copy(b.To,box.origin);
  //box.xbasis.set(b.R[0][0],b.R[0][1],b.R[0][2]);
  //box.ybasis.set(b.R[1][0],b.R[1][1],b.R[1][2]);
  //box.zbasis.set(b.R[2][0],b.R[2][1],b.R[2][2]);
  box.xbasis.set(b.R[0][0],b.R[1][0],b.R[2][0]);
  box.ybasis.set(b.R[0][1],b.R[1][1],b.R[2][1]);
  box.zbasis.set(b.R[0][2],b.R[1][2],b.R[2][2]);

  //move the box to have origin at the corner
  box.origin -= box.dims.x*box.xbasis;
  box.origin -= box.dims.y*box.ybasis;
  box.origin -= box.dims.z*box.zbasis;
  box.dims *= 2;
}

void BoxToBV(const Box3D& box,BV& b)
{
  //convert corner-origin box to center-origin BV
  Vector3 halfDims = box.dims*Half;
  Vector3 center = box.origin + halfDims.x*box.xbasis + halfDims.y*box.ybasis + halfDims.z*box.zbasis;
  Copy(halfDims,b.d);
  Copy(center,b.To);

  /*
  b.R[0][0] = box.xbasis.x;
  b.R[0][1] = box.xbasis.y;
  b.R[0][2] = box.xbasis.z;
  b.R[1][0] = box.ybasis.x;
  b.R[1][1] = box.ybasis.y;
  b.R[1][2] = box.ybasis.z;
  b.R[2][0] = box.zbasis.x;
  b.R[2][1] = box.zbasis.y;
  b.R[2][2] = box.zbasis.z;
  */
  b.R[0][0] = box.xbasis.x;
  b.R[0][1] = box.ybasis.x;
  b.R[0][2] = box.zbasis.x;
  b.R[1][0] = box.xbasis.y;
  b.R[1][1] = box.ybasis.y;
  b.R[1][2] = box.zbasis.y;
  b.R[2][0] = box.xbasis.z;
  b.R[2][1] = box.ybasis.z;
  b.R[2][2] = box.zbasis.z;
}


//like obb_disjoint, returns 0 if collision, 1 if disjoint
//box_dims are the half-dimensions of the box
//c is the center of the sphere
//r is the radius of the sphere
int aabb_sphere_disjoint(const PQP_REAL box_dims[3],const PQP_REAL c[3], PQP_REAL r)
{
  PQP_REAL c_cropped[3];
  for(int i=0;i<3;i++) c_cropped[i] = (c[i] > box_dims[i] ? box_dims[i] : 
				       (c[i] < -box_dims[i] ? -box_dims[i] :
					c[i]));
  PQP_REAL d_squared=0;
  d_squared = Sqr(c_cropped[0]-c[0])+Sqr(c_cropped[1]-c[1])+Sqr(c_cropped[2]-c[2]);
  return (d_squared > Sqr(r) ? 1 : 0);
}

//returns the ray-box collision parameter if colliding, inf if not.
//box_dims are the half-dimensions of the box
//s is the source of the ray
//d is the direction of the ray
Real aabb_ray_collide(const PQP_REAL box_dims[3],const PQP_REAL s[3],const PQP_REAL d[3])
{
  AABB3D b;
  Copy(box_dims,b.bmin); b.bmin.inplaceNegative();
  Copy(box_dims,b.bmax);
  if(box_dims[0] < 1e-8) b.bmax.x = 1e-8;  //to avoid numerical errors
  if(box_dims[1] < 1e-8) b.bmax.y = 1e-8;  //to avoid numerical errors
  if(box_dims[2] < 1e-8) b.bmax.z = 1e-8;  //to avoid numerical errors
  Ray3D r;
  Copy(s,r.source);
  Copy(d,r.direction);
  Real min=0,max=Inf;
  if(ClipLine(Vector3(s),Vector3(d),b,min,max)) return min;
  return Inf;
}


//transforms a parent-coordinate point p to local coordinates of the BV
//OBB transform is x' = b.R*x + b.To
void ToLocal(const BV& b,const PQP_REAL p[3],PQP_REAL ploc[3])
{
  PQP_REAL psub[3];
  VmV(psub,p,b.To);
  MTxV(ploc,b.R,psub);
}


//for each geometric primitive, define three functions: ToLocal, CollideBV, DistanceLimitsBV

/**********  Points **************/

inline void ToLocal(const BV& b,const Vector3& in,Vector3& out)
{
  PQP_REAL temp[3];
  VmV(temp,in,b.To);
  MTxV(out,b.R,temp);
}

inline bool CollideBV(const PQP_REAL d[3],const Vector3& pt)
{
  return Abs(pt.x)<=d[0] && Abs(pt.y)<=d[1] && Abs(pt.z)<=d[2];
}

inline void DistanceLimitsBV(const PQP_REAL d[3],const Vector3& pt,Real& dmin,Real& dmax)
{
  PQP_REAL p[3];
  //normalize to positive quadrant
  p[0] = Abs(pt.x);
  p[1] = Abs(pt.y);
  p[2] = Abs(pt.z);

  //closest feature can be face, edge, vertex
  PQP_REAL excess[3];
  VmV(excess,p,d);
  dmin=0;
  for(int i=0;i<3;i++)
    if(excess[i] > 0) dmin += Sqr(excess[i]);

  //furthest feature must be the opposite vertex
  dmax=Sqr(p[0] + d[0]) + Sqr(p[1] + d[1]) + Sqr(p[2] + d[2]);
}

/**********  Segments ************/

inline void ToLocal(const BV& bv,const Segment3D& in,Segment3D& out)
{
  ToLocal(bv,in.a,out.a);
  ToLocal(bv,in.b,out.b);
}

//returns 1 if the aabb [-d,d] collides with the segment s
inline bool CollideBV(const PQP_REAL d[3],const Segment3D& s)
{
  //quick reject -- endpoint out of box
  if(s.a.x < -d[0] && s.b.x < -d[0]) return false;
  if(s.a.x > d[0] && s.b.x > d[0]) return false;
  if(s.a.y < -d[1] && s.b.y < -d[1]) return false;
  if(s.a.y > d[1] && s.b.y > d[1]) return false;
  if(s.a.z < -d[2] && s.b.z < -d[2]) return false;
  if(s.a.z > d[2] && s.b.z > d[2]) return false;
  //quick accept - endpoint in box
  if(CollideBV(d,s.a)) return true;
  if(CollideBV(d,s.b)) return true;
  Vector3 v;
  v.sub(s.b,s.a);
  Real u1=0,u2=1;
  if(ClipLine1D(-d[0] - s.a.x, -v.x, u1,u2) && ClipLine1D(s.a.x - d[0], v.x, u1,u2)) {
    if(ClipLine1D(-d[1] - s.a.y, -v.y, u1,u2) && ClipLine1D(s.a.y - d[1], v.y, u1,u2)) {
      if(ClipLine1D(-d[2] - s.a.z, -v.z, u1,u2) && ClipLine1D(s.a.z - d[2], v.z, u1, u2)) {
	return true;
      }
    }
  }
  return false;
}

inline bool Collide(const Triangle3D& tri,const Segment3D& s,Vector3& pt)
{
  Ray3D ray;
  ray.source=s.a;
  ray.direction=s.b-s.a;
  Real t,u,v;
  if(!tri.rayIntersects(ray,&t,&u,&v)) return false;
  if(t < 0 || t > 1) return false;
  ray.eval(t,pt);
  return true;
}



/**********  Spheres ************/

inline void ToLocal(const BV& bv,const Sphere3D& in,Sphere3D& out)
{
  ToLocal(bv,in.center,out.center);
  out.radius = in.radius;
}

inline bool CollideBV(const PQP_REAL d[3],const Sphere3D& s)
{
  PQP_REAL dmin,dmax;
  DistanceLimitsBV(d,s.center,dmin,dmax);
  return dmin < Sqr(s.radius);
}

inline bool Collide(const Triangle3D& tri,const Sphere3D& s,Vector3& pt)
{
  pt = tri.closestPoint(s.center);
  return s.contains(pt);
}



/**********  Boxes ************/

inline void ToLocal(const BV& bv,const BV& in,BV& out)
{
  MTxM(out.R,bv.R,in.R);
  PQP_REAL temp[3];
  VmV(temp,in.To,bv.To);
  MTxV(out.To,bv.R,temp);  
  VcV(out.d,in.d);
}

inline bool CollideBV(const PQP_REAL d[3],BV& bv)
{
  PQP_REAL temp[3];
  VcV(temp,d);
  return (obb_disjoint(bv.R,bv.To,temp,bv.d)==0);
}

inline bool Collide(const Triangle3D& tri,const BV& b,Vector3& pt)
{
  Box3D box;
  BVToBox(b,box);
  return box.intersects(tri);
}


/**********  Planes ************/

inline void ToLocal(const BV& bv,const Plane3D& in,Plane3D& out)
{
  Vector3 pt = in.normal*in.offset;
  Vector3 ptout;
  ToLocal(bv,pt,ptout);
  MTxV(out.normal,bv.R,in.normal);
  out.offset = out.normal.dot(ptout);
}

inline bool CollideBV(const PQP_REAL d[3],const Plane3D& p)
{
  AABB3D bb;
  bb.bmin.set(-d[0],-d[1],-d[2]);
  bb.bmax.set(d[0],d[1],d[2]);
  return p.intersects(bb);
}

inline bool Collide(const Triangle3D& tri,const Plane3D& p,Vector3& pt)
{
  Segment3D s;
  if(!tri.intersects(p,s)) return false;
  pt = (s.a+s.b)*0.5;
  return true;
}



//returns the index of a triangle colliding with g.  g is given
//in the local frame of the models.  pt is a colliding point. 
template <class Geom>
int CollideRecurse(const Geom& g,const PQP_Model& m,int b,Vector3& pt)
{
  Geom gloc;
  ToLocal(m.b[b],g,gloc);
  if(CollideBV(m.b[b].d,gloc)) {
    if(m.b[b].Leaf()) {
      int t=-m.b[b].first_child-1;
      Triangle3D tri;
      Copy(m.tris[t].p1,tri.a);
      Copy(m.tris[t].p2,tri.b);
      Copy(m.tris[t].p3,tri.c);
      if(Collide(tri,g,pt))
	return m.tris[t].id;
    }
    else {
      int c1=m.b[b].first_child;
      int c2=c1+1;
      int t=CollideRecurse(g,m,c1,pt);
      if(t != -1) return t;
      t=CollideRecurse(g,m,c2,pt);
      return t;
    }
  }
  return -1;
}

//computes the indices of all triangles colliding with g.  g is given in the
//local frame of the model.
template <class Geom>
void CollideAllRecurse(const Geom& g,const PQP_Model& m,int b,vector<int>& tris,size_t max)
{
  Geom gloc;
  ToLocal(m.b[b],g,gloc);
  if(CollideBV(m.b[b].d,gloc)) {
    if(m.b[b].Leaf()) {
      int t=-m.b[b].first_child-1;
      Triangle3D tri;
      Copy(m.tris[t].p1,tri.a);
      Copy(m.tris[t].p2,tri.b);
      Copy(m.tris[t].p3,tri.c);
      Vector3 pt;
      if(Collide(tri,g,pt)) 
	tris.push_back(m.tris[t].id);
    }
    else {
      int c1=m.b[b].first_child;
      int c2=c1+1;
      CollideAllRecurse(g,m,c1,tris,max);
      if(tris.size()==max) return;
      CollideAllRecurse(g,m,c2,tris,max);
    }
  }
}




struct ClosestPointCallback
{
  struct ActivePair
  {
    bool operator < (const ActivePair& rhs) const { return minDist < rhs.minDist; }
    int index;          //bound index
    Real minDist;       //minimum distance of bound b
  };

  ClosestPointCallback()
    :normalWeight(0),dmin(Inf),dmax(Inf),closestTri(-1), numTrianglesChecked(0), numBBsChecked(0)
  {}
  void Execute(const PQP_Model& m,const Vector3& p) {
    plocal = p;
    Assert(m.num_bvs != 0);
    //compute distance of random triangle -- perhaps faster pruning
    //int t = RandInt(m.num_tris);
    //compute distance of arbitrary triangle -- perhaps faster pruning
    int t = 0;
    Copy(m.tris[t].p1,tri.a);
    Copy(m.tris[t].p2,tri.b);
    Copy(m.tris[t].p3,tri.c);
    cp=tri.closestPoint(p);
    Real d=cp.distanceSquared(p);
    if(normalWeight != 0) 
      d += normalWeight*nlocal.distanceSquared(tri.normal());
    if(d < dmin) {
      dmax = dmin = d;
      closestTri = m.tris[t].id;
    }
    numTrianglesChecked = 1;
    numBBsChecked = 0;
    if(normalWeight != 0) {
      //do this "soft" recursion
      ExecuteRecurse(m,0);
      return;
    }
    ExecuteRecurse(m,0);
    return;

    /*
    ActivePair temp;
    temp.index = 0;
    temp.minDist = 0;
    VcV(temp.plocal,ploc);
    set<ActivePair> q;
    q.insert(temp);

    while(!q.empty()) {
      temp=*q.begin(); q.erase(q.begin());
      numBBsChecked++;
      LOG4CXX_INFO(GET_LOGGER(Geometry),"Popped bound with min distance: "<<temp.minDist);
      int b=temp.index;

      bool prune = false;
      if(m.b[b].Leaf()) { //it's a triangle -- must be the closest
	int t = -m.b[b].first_child - 1;
	Assert(t < m.num_tris);
	Copy(m.tris[t].p1,tri.a);
	Copy(m.tris[t].p2,tri.b);
	Copy(m.tris[t].p3,tri.c);
	cp=tri.closestPoint(p);
	Real d=cp.distanceSquared(p);
	if(d < dmin) {
	Assert(normalWeight == 0);
	dmin = dmax = d;
	closestTri = m.tris[t].id;
	prune = true;
        }
	numTrianglesChecked++;
      }
      else {
	int c1 = m.b[b].first_child;
	int c2 = c1+1;

	//pick the closest one to do first
	//quick reject: dmin is greater than dmax
	Real dbmin1,dbmax1;
	Real dbmin2,dbmax2;
	PQP_REAL p1[3],p2[3];
	ToLocal(m.b[c1],temp.plocal,p1);
	ToLocal(m.b[c2],temp.plocal,p2);
	DistanceLimitsBV(m.b[c1].d,p1,dbmin1,dbmax1);
	DistanceLimitsBV(m.b[c2].d,p2,dbmin2,dbmax2);

	LOG4CXX_INFO(GET_LOGGER(Geometry),"Children "<<c1<<", "<<c2<<", distances "<<dbmin1<<", "<<dbmin2);
	Assert(dbmin1 <= dbmax1);
	Assert(dbmin2 <= dbmax2);
	if(dbmin1 < dmax) {
	  temp.index = c1;
	  temp.minDist = dbmin1;
	  VcV(temp.plocal,p1);
	  q.insert(temp);
	  if(dbmax1 < dmax) {
	    LOG4CXX_INFO(GET_LOGGER(Geometry),"Max distance set to "<<dbmax1<<" on child 1");
	    dmax = dbmax1;
	    prune = true;
	  }
	}
	if(dbmin2 < dmax) {
	  temp.index = c2;
	  temp.minDist = dbmin2;
	  VcV(temp.plocal,p2);
	  q.insert(temp);
	  if(dbmax2 < dmax) {
	    LOG4CXX_INFO(GET_LOGGER(Geometry),"Max distance set to "<<dbmax2<<" on child 2");
	    dmax = dbmax2;
	    prune = true;
	  }
	}
      }
      //prune the queue, if dmax changed
      temp.minDist = dmax;
      set<ActivePair>::iterator i=q.upper_bound(temp);
      q.erase(i,q.end());
    }
    LOG4CXX_INFO(GET_LOGGER(Geometry),numTrianglesChecked<<" triangles, "<<numBBsChecked<<" BBs checked");
    */
  }

  void ExecuteRecurse(const PQP_Model& m,int b)
  {
    //which child to pick first?
    if(m.b[b].Leaf()) { //it's a triangle
      numTrianglesChecked++;
      int t = -m.b[b].first_child - 1;
      Assert(t < m.num_tris);
      //compute distance squared
      Copy(m.tris[t].p1,tri.a);
      Copy(m.tris[t].p2,tri.b);
      Copy(m.tris[t].p3,tri.c);
      Vector3 temp=tri.closestPoint(plocal);
      Real d=temp.distanceSquared(plocal);
      if(normalWeight != 0) 
	d += normalWeight*nlocal.distanceSquared(tri.normal());
      if(d < dmin) {
	dmax = dmin = d;
	cp = temp;
	closestTri = m.tris[t].id;
      }
    }
    else {
      numBBsChecked++;
      int c1 = m.b[b].first_child;
      int c2 = c1+1;

      //pick the closest one to do first
      //quick reject: closest distance is less than dmin
      Real dbmin1,dbmax1;
      Real dbmin2,dbmax2;
      Vector3 p1,p2;
      ToLocal(m.b[c1],plocal,p1);
      ToLocal(m.b[c2],plocal,p2);
      DistanceLimitsBV(m.b[c1].d,p1,dbmin1,dbmax1);
      DistanceLimitsBV(m.b[c2].d,p2,dbmin2,dbmax2);
      //increas the bounds if the normals are needed
      if(normalWeight != 0) {
	dbmax1=dbmax1+Two*normalWeight;
	dbmax2=dbmax2+Two*normalWeight;
      }
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

  Real normalWeight;
  Vector3 plocal,nlocal;
  Real dmin,dmax;  //min squared distance, max squared distance bound
  int closestTri;
  Triangle3D tri;
  Vector3 cp;

  int numTrianglesChecked,numBBsChecked;
};

int ClosestPoint(const CollisionMesh& mesh,const Vector3& p,Vector3& cp,Real bound)
{
  Vector3 plocal;
  mesh.currentTransform.mulInverse(p,plocal);
  ClosestPointCallback cb;
  cb.dmin = bound*bound;
  cb.dmax = bound*bound;
  cb.Execute(*mesh.pqpModel,plocal);
  cp = cb.cp;

  /*
  //TEST
  int regularTri = mesh.ClosestPoint(p,cp);
  if(!FuzzyEquals(cb.cp.distance(p),cp.distance(p),1e-3)) {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"ClosestPoint error: "<<cb.cp.distance(p)<<" != "<<cp.distance(p));
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Triangle "<<cb.closestTri<<" vs "<<regularTri);
  }
  Assert(FuzzyEquals(cb.cp.distance(p),cp.distance(p),1e-3));
  */
  return cb.closestTri;
}




int Collide(const CollisionMesh& m,const Segment3D& s,Vector3& pt)
{
  Segment3D slocal;
  m.currentTransform.mulInverse(s.a,slocal.a);
  m.currentTransform.mulInverse(s.b,slocal.b);
  return CollideRecurse(slocal,*m.pqpModel,0,pt);
}

bool Collide(const CollisionMesh& m,const Sphere3D& s)
{
  Sphere3D slocal;
  m.currentTransform.mulInverse(s.center,slocal.center);
  slocal.radius = s.radius;
  Vector3 pt;
  return CollideRecurse(slocal,*m.pqpModel,0,pt)>=0;
}


bool Collide(const CollisionMesh& m,const AABB3D& bb)
{
  Box3D box;
  box.xbasis.set(1,0,0);
  box.ybasis.set(0,1,0);
  box.zbasis.set(0,0,1);
  box.origin = bb.bmin;
  box.dims = bb.bmax-bb.bmin;
  return Collide(m,box);
}

bool Collide(const CollisionMesh& m,const Box3D& b)
{
  RigidTransform Tinv;
  Tinv.setInverse(m.currentTransform);
  Box3D blocal;
  blocal.setTransformed(b,Tinv);
  Vector3 pt;
  BV bv;
  BoxToBV(blocal,bv);
  return CollideRecurse(bv,*m.pqpModel,0,pt)>=0;
}

bool Collide(const CollisionMesh& m,const GeometricPrimitive3D& g)
{
  switch(g.type) { 
  case GeometricPrimitive3D::Point:
    {
      Sphere3D s;
      s.center = *AnyCast_Raw<Vector3>(&g.data);
      s.radius = 0;
      return Collide(m,s);
    }
  case GeometricPrimitive3D::Segment:
    {
      Vector3 pt;
      return Collide(m,*AnyCast_Raw<Segment3D>(&g.data),pt) >= 0;
    }
  case GeometricPrimitive3D::Triangle:
    return Collide(m,*AnyCast_Raw<Triangle3D>(&g.data));
  case GeometricPrimitive3D::AABB:
    return Collide(m,*AnyCast_Raw<AABB3D>(&g.data));
  case GeometricPrimitive3D::Box:
    return Collide(m,*AnyCast_Raw<Box3D>(&g.data));
  case GeometricPrimitive3D::Sphere:
    return Collide(m,*AnyCast_Raw<Sphere3D>(&g.data));
  case GeometricPrimitive3D::Empty:
    return false;
  default:
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"Collide: Collider for type "<<g.TypeName());
    return false;
  }
}

bool Collide(const CollisionMesh& m,const Plane3D& p)
{
  vector<int> tris;
  CollideAll(m,p,tris,1);
  return !tris.empty();
}

bool WithinDistance(const CollisionMesh& c,const GeometricPrimitive3D& a,Real d)
{
  switch(a.type) {
  case GeometricPrimitive3D::Point:
    {
      Sphere3D s;
      s.center = *AnyCast_Raw<Vector3>(&a.data);
      s.radius = d;
      return Collide(c,s);
    }
  case GeometricPrimitive3D::Segment:
  case GeometricPrimitive3D::Triangle:
  case GeometricPrimitive3D::AABB:
  case GeometricPrimitive3D::Box:
    {
      if(d != 0) {
	LOG4CXX_ERROR(GET_LOGGER(Geometry),"Not yet able to within-distance test of "<<a.TypeName()<<" vs CollisionMesh");
  return false;
      }
      return Collide(c,a);
    }
  case GeometricPrimitive3D::Sphere:
    {
      Sphere3D s = *AnyCast_Raw<Sphere3D>(&a.data);
      s.radius += d;
      return Collide(c,s);
    }
  default:
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Not yet able to collide a primitive of type "<<a.TypeName()<<" vs CollisionMesh");
    return false;
  }
}

bool Collide(const CollisionMesh& m1,const CollisionMesh& m2)
{

  if(m1.tris.empty() || m2.tris.empty()) return false;
  if(m1.pqpModel == NULL || m2.pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  PQP_CollideResult collide;
  int res=PQP_Collide(&collide,
		      R1,T1,m1.pqpModel,
		      R2,T2,m2.pqpModel,
		      PQP_FIRST_CONTACT);
  Assert(res == PQP_OK);
  return (collide.Colliding()!=0);
}




void CollideAll(const CollisionMesh& m,const Sphere3D& s,vector<int>& tris,int max)
{
  Sphere3D slocal;
  m.currentTransform.mulInverse(s.center,slocal.center);
  slocal.radius = s.radius;
  tris.resize(0);
  CollideAllRecurse(slocal,*m.pqpModel,0,tris,max);
}

void CollideAll(const CollisionMesh& m,const Segment3D& s,vector<int>& tris,int max)
{
  Segment3D slocal;
  m.currentTransform.mulInverse(s.a,slocal.a);
  m.currentTransform.mulInverse(s.b,slocal.b);
  tris.resize(0);
  CollideAllRecurse(slocal,*m.pqpModel,0,tris,max);
}

void CollideAll(const CollisionMesh& m,const AABB3D& bb,vector<int>& tris,int max)
{
  Box3D box;
  box.xbasis.set(1,0,0);
  box.ybasis.set(0,1,0);
  box.zbasis.set(0,0,1);
  box.origin = bb.bmin;
  box.dims = bb.bmax-bb.bmin;
  CollideAll(m,box,tris,max);
}

void CollideAll(const CollisionMesh& m,const Box3D& bb,vector<int>& tris,int max)
{
  RigidTransform Tinv;
  Tinv.setInverse(m.currentTransform);
  Box3D blocal;
  blocal.setTransformed(bb,Tinv);
  BV bv;
  BoxToBV(blocal,bv);
  tris.resize(0);
  CollideAllRecurse(bv,*m.pqpModel,0,tris,max);
}

void CollideAll(const CollisionMesh& m,const GeometricPrimitive3D& g,std::vector<int>& tris,int max)
{
  switch(g.type) { 
  case GeometricPrimitive3D::Point:
    {
      Sphere3D s;
      s.center = *AnyCast_Raw<Vector3>(&g.data);
      s.radius = 0;
      CollideAll(m,s,tris,max);
    }
    break;
  case GeometricPrimitive3D::Segment:
    CollideAll(m,*AnyCast_Raw<Segment3D>(&g.data),tris,max);
    break;
  case GeometricPrimitive3D::Triangle:
    CollideAll(m,*AnyCast_Raw<Triangle3D>(&g.data),tris,max);
    break;
  case GeometricPrimitive3D::AABB:
    CollideAll(m,*AnyCast_Raw<AABB3D>(&g.data),tris,max);
    break;
  case GeometricPrimitive3D::Box:
    CollideAll(m,*AnyCast_Raw<Box3D>(&g.data),tris,max);
    break;
  case GeometricPrimitive3D::Sphere:
    CollideAll(m,*AnyCast_Raw<Sphere3D>(&g.data),tris,max);
    break;
  case GeometricPrimitive3D::Empty:
    return;
  default:
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"CollideAll: Collider for type "<<g.TypeName());
  }
}

void CollideAll(const CollisionMesh& m,const Plane3D& p,vector<int>& tris,int max)
{
  RigidTransform Tinv;
  Tinv.setInverse(m.currentTransform);
  Plane3D plocal;
  plocal.setTransformed(p,Tinv);
  tris.resize(0);
  CollideAllRecurse(plocal,*m.pqpModel,0,tris,max);
}

///Returns true if p is within distance d of m using the PQP bounding heirarchy
bool WithinDistance(const CollisionMesh& m,const Vector3& p,Real d)
{
  Sphere3D s;
  s.center=p;
  s.radius=d;
  return Collide(m,s);
}

///Finds the triangles within distance d to p on m using the PQP
///bounding heirarchy
void NearbyTriangles(const CollisionMesh& m,const Vector3& p,Real d,vector<int>& tris,int max)
{
  Sphere3D s;
  s.center=p;
  s.radius=d;
  CollideAll(m,s,tris,max);
}

///fits an oriented bounding box to the geometry.
///TODO: do a better job on triangles, segments
void FitBox(const GeometricPrimitive3D& g,Box3D& box) {
  switch(g.type) {
  case GeometricPrimitive3D::Box:
    box = *AnyCast<Box3D>(&g.data);
    break;
  default:
    {
      box.set(g.GetAABB());
    }
    break;
  }
}

void NearbyTriangles(const CollisionMesh& m,const GeometricPrimitive3D& g,Real d,vector<int>& tris,int max)
{
  switch(g.type) { 
  case GeometricPrimitive3D::Point:
    {
      Sphere3D s;
      s.center = *AnyCast_Raw<Vector3>(&g.data);
      s.radius = d;
      CollideAll(m,s,tris,max);
    }
    break;
  case GeometricPrimitive3D::Segment:
  case GeometricPrimitive3D::Triangle:
  case GeometricPrimitive3D::AABB:
  case GeometricPrimitive3D::Box:
    if(d != 0) {
      if(!g.SupportsDistance(GeometricPrimitive3D::Triangle)) {
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"Not yet able to within-distance test of "<<g.TypeName()<<" vs CollisionMesh");
        return;
      }
      //compute an expanded bounding box and collide them
      Box3D bbox;
      FitBox(g,bbox);
      bbox.origin -= d*(bbox.xbasis+bbox.ybasis+bbox.zbasis);
      bbox.dims += Vector3(d*2);
      vector<int> temptris;
      int tempmax = max;
      while(true) {
        CollideAll(m,bbox,temptris,tempmax);
        //check which ones are actually within the given distance
        tris.resize(0);
        Triangle3D tri;
        for(size_t i=0;i<temptris.size();i++) {
          m.GetTriangle(temptris[i],tri);
          if(g.Distance(tri) <= d) {
            tris.push_back(temptris[i]);
            if((int)tris.size()==max) return; //done!
          }
        }
        if((int)temptris.size()<tempmax) 
          return;
        else  
          //filled out all the temp triangles, but may have missed some due to filtering
          tempmax *= 2;
      }
    }
    CollideAll(m,g,tris,max);
    break;
  case GeometricPrimitive3D::Sphere:
    {
      Sphere3D s;
      s.center = AnyCast_Raw<Sphere3D>(&g.data)->center;
      s.radius = AnyCast_Raw<Sphere3D>(&g.data)->radius+d;
      return CollideAll(m,s,tris,max);
    }
    break;
  case GeometricPrimitive3D::Empty:
    return;
  default:
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"NearbyTriangles: Collider for type "<<g.TypeName()<<" not available");
    return;
  }
}

void NearbyTriangles(const CollisionMesh& m1,const CollisionMesh& m2,Real d,vector<int>& tris1,vector<int>& tris2,int max)
{
  CollisionMeshQuery q(m1,m2);
  if(max==1) {
    bool res = q.WithinDistance(d);
    if(res) {
      tris1.resize(1);
      tris2.resize(1);
      q.TolerancePair(tris1[0],tris2[0]);
    }
  }
  else {
    q.WithinDistanceAll(d);
    q.TolerancePairs(tris1,tris2);
  }
}

Real Distance(const CollisionMesh& c,const Vector3& pt,Real bound)
{
  Vector3 cp;
  int res=ClosestPoint(c,pt,cp,bound);
  if(res < 0) return bound;
  return pt.distance(cp);
}

Real Distance(const CollisionMesh& c,const Vector3& pt,int& closestTri,Vector3& cp,Vector3& dir,Real bound)
{
  closestTri = ClosestPoint(c,pt,cp,bound);
  if(closestTri < 0) return bound;
  cp = c.currentTransform*cp;
  dir = pt - cp;
  Real l=dir.norm();
  if(FuzzyZero(l)) {
    Vector3 nlocal = c.TriangleNormal(closestTri);
    c.currentTransform.R.mul(nlocal,dir);
  }
  dir /= l;
  return l;
}

Real Distance(const CollisionMesh& c,const GeometricPrimitive3D& a,Real bound)
{
  int closestTri;
  Vector3 cp,dir;
  return Distance(c,a,closestTri,cp,dir,bound);
}

Real Distance(const CollisionMesh& c,const GeometricPrimitive3D& a,int& closestTri,Vector3& cp,Vector3& dir,Real bound)
{
  switch(a.type) {
  case GeometricPrimitive3D::Point:
    return Distance(c,*AnyCast_Raw<Vector3>(&a.data),closestTri,cp,dir,bound);
  case GeometricPrimitive3D::Segment:
  case GeometricPrimitive3D::Triangle:
  case GeometricPrimitive3D::AABB:
  case GeometricPrimitive3D::Box:
    {
      if(!a.SupportsDistance(GeometricPrimitive3D::Triangle)) {
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"Not yet able to within-distance test of "<<a.TypeName()<<" vs CollisionMesh");
        return Inf;
      }
      LOG4CXX_DEBUG(GET_LOGGER(Geometry),"CollisionMesh-"<<a.TypeName()<<" distance uses inefficient linear search");
      LOG4CXX_DEBUG(GET_LOGGER(Geometry),"CollisionMesh-"<<a.TypeName()<<" distance does not return correct closest point and direction");
      RigidTransform Tlocal;
      Tlocal.setInverse(c.currentTransform);
      GeometricPrimitive3D alocal = a;
      alocal.Transform(Tlocal);
      Real d=bound;
      closestTri = -1;
      Triangle3D tri;
      for(size_t i=0;i<c.tris.size();i++) {
        c.GetTriangle(i,tri);
        Real di = alocal.Distance(tri);
        if(di < d) {
          d = di;
          closestTri = (int)i;
        }
      }
      return d;
    }
  case GeometricPrimitive3D::Sphere:
    {
      Sphere3D s = *AnyCast_Raw<Sphere3D>(&a.data);
      Real dcenter = Distance(c,s.center,closestTri,cp,dir,bound+s.radius);
      return dcenter-s.radius;
    }
  default:
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"Not yet able to collide a primitive of type "<<a.TypeName()<<" vs CollisionMesh");
    return false;
  }
}




int ClosestPointAndNormal(const TriMesh& m,Real pWeight,Real nWeight,const Vector3& p,const Vector3& n,Vector3& cp)
{
  Real dmin=Inf;
  int t=-1;
  Triangle3D tri;
  for(size_t i=0;i<m.tris.size();i++) {
    m.GetTriangle((int)i,tri);
    Vector3 tn=tri.normal();
    Vector3 tx=tri.closestPoint(p);
    Real d = pWeight*tx.distanceSquared(p) + nWeight*tn.distanceSquared(n);
    if(d < dmin) {
      cp = tx;
      t = (int)i;
      dmin = d;
    }
  }
  return t;
}

int ClosestPointAndNormal(const CollisionMesh& mesh,Real nWeight,const Vector3& p,const Vector3& n,Vector3& cp)
{
  Vector3 plocal,nlocal;
  mesh.currentTransform.mulInverse(p,plocal);
  mesh.currentTransform.R.mulTranspose(n,nlocal);
  ClosestPointCallback cb;
  cb.normalWeight = nWeight;
  cb.nlocal = nlocal;
  cb.Execute(*mesh.pqpModel,plocal);
  cp = cb.cp;
  return cb.closestTri;
}

Real BVRayCollision(const BV& bv,const Ray3D& r)
{
  PQP_REAL S0[3],D0[3],S[3],D[3],temp[3];
  Copy(r.source,S0);
  Copy(r.direction,D0);
  //transform to initial frame
  VmV(temp,S0,bv.To);
  MTxV(S,bv.R,temp);
  MTxV(D,bv.R,D0);
  return aabb_ray_collide(bv.d,S,D); 
}

struct RayCastCallback
{
  RayCastCallback(const PQP_Model& _mesh,const Ray3D& _r)
    :m(_mesh),r(_r),closestParam(Inf),closestTri(-1)
  {}

  void Compute() {
    closestParam = Inf;
    closestTri = -1;
    if(m.num_bvs==0) return;  //empty model

    //test bounding box of whole object
    Real p=BVRayCollision(m.b[0],r);  //inf returned if no collision
    if(IsInf(p)) {
      return;
    }
    //recurse on root of bb heirarchy
    Recurse(0);
  }

  void Recurse(int b)
  {
    if(m.b[b].Leaf()) {
      int t=-m.b[b].first_child-1;
      //actually cast the ray to the triangle
      Triangle3D tri;
      Copy(m.tris[t].p1,tri.a);
      Copy(m.tris[t].p2,tri.b);
      Copy(m.tris[t].p3,tri.c);
      Real param,u,v;
      
      if(tri.rayIntersects(r,&param,&u,&v)) {
	if(param < closestParam) {
	  closestParam = param;
	  closestPoint = tri.planeCoordsToPoint(Vector2(u,v));
	  closestTri = m.tris[t].id;
	}
      }
    }
    else {
      int c1=m.b[b].first_child;
      int c2=c1+1;
      
      //inf returned if no collision
      Real p1=BVRayCollision(m.b[c1],r);
      Real p2=BVRayCollision(m.b[c2],r);
    
      if(p1 < p2) {  //do child 1 first
	if(p1 < closestParam) Recurse(c1);
	if(p2 < closestParam) Recurse(c2);
      }
      else {   //do child 2 first
	if(p2 < closestParam) Recurse(c2);
	if(p1 < closestParam) Recurse(c1);
      }
    }
  }

  const PQP_Model& m;
  const Ray3D& r;

  Real closestParam;
  int closestTri;
  Vector3 closestPoint;
};

int RayCast(const CollisionMesh& mesh,const Ray3D& r,Vector3& pt)
{
  Ray3D rLocal;
  mesh.currentTransform.mulPointInverse(r.source,rLocal.source);
  mesh.currentTransform.mulVectorInverse(r.direction,rLocal.direction);

  RayCastCallback callback(*mesh.pqpModel,rLocal);
  callback.Compute();
  pt = mesh.currentTransform*callback.closestPoint;
  return callback.closestTri;
}

int RayCastLocal(const CollisionMesh& mesh,const Ray3D& r,Vector3& pt)
{
  RayCastCallback callback(*mesh.pqpModel,r);
  callback.Compute();
  pt = callback.closestPoint;
  return callback.closestTri;
}

void GetBB(const CollisionMesh& m,Box3D& bb)
{
  BVToBox(m.pqpModel->b[0],bb);
  bb.origin = m.currentTransform*bb.origin;
  bb.xbasis = m.currentTransform.R*bb.xbasis;
  bb.ybasis = m.currentTransform.R*bb.ybasis;
  bb.zbasis = m.currentTransform.R*bb.zbasis;
}





















inline
PQP_REAL
TriDistance(PQP_REAL R[3][3], PQP_REAL T[3], Tri *t1, Tri *t2,
            PQP_REAL p[3], PQP_REAL q[3])
{
  // transform tri 2 into same space as tri 1

  PQP_REAL tri1[3][3], tri2[3][3];

  VcV(tri1[0], t1->p1);
  VcV(tri1[1], t1->p2);
  VcV(tri1[2], t1->p3);
  MxVpV(tri2[0], R, t2->p1, T);
  MxVpV(tri2[1], R, t2->p2, T);
  MxVpV(tri2[2], R, t2->p3, T);
  
  PQP_REAL d=TriDist(p,q,tri1,tri2);
  if(d == 0) {  //TriDistance doesn't give the right points
    Vector3 aa(tri1[0]), ab(tri1[1]), ac(tri1[2]);
    Triangle3D a,b;
    a.a.set(tri1[0]);
    a.b.set(tri1[1]);
    a.c.set(tri1[2]);
    b.a.set(tri2[0]);
    b.b.set(tri2[1]);
    b.c.set(tri2[2]);
    Segment3D s;
    bool res=a.intersects(b,s);
    if(!res) {
      LOG4CXX_WARN(GET_LOGGER(Geometry),"Warning: PQP says triangles intersect, but I don't find an intersection");
      Copy((a.a+a.b+a.c)/3.0,p);
      Copy((b.a+b.b+b.c)/3.0,q);
    }
    else {
      Copy((s.a+s.b)*0.5,p);
      Copy((s.a+s.b)*0.5,q);
    }
  }
  return d;
}



Geometry3DTriangleMesh::Geometry3DTriangleMesh()
{}

Geometry3DTriangleMesh::Geometry3DTriangleMesh(const Meshing::TriMesh& _data)
: data(_data)
{}

Geometry3DTriangleMesh::Geometry3DTriangleMesh(const Meshing::TriMesh& _data,shared_ptr<GLDraw::GeometryAppearance> _appearance)
: data(_data),appearance(_appearance)
{}

Geometry3DTriangleMesh::Geometry3DTriangleMesh(Meshing::TriMesh&& _data)
: data(_data)
{}

Geometry3DTriangleMesh::~Geometry3DTriangleMesh ()
{}

Geometry3D* Geometry3DTriangleMesh::ConvertTo(Type restype, Real param, Real expansionParameter) const
{
    switch (restype)
    {
    case Type::ConvexHull:
        {
        auto* res = new Geometry3DConvexHull();
        MeshConvexDecomposition(data,res->data,param);
        return res;
        }
    default:
        return NULL;
    }
}

bool Geometry3DTriangleMesh::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion)
{
    switch(geom->GetType()) {
    case Type::Primitive:
    {
        const auto& prim = dynamic_cast<const Geometry3DPrimitive*>(geom)->data;
        if (param == 0)
            param = 16;
        else
        {
            AABB3D bb = geom->GetAABB();
            Real w = (bb.bmax - bb.bmin).maxAbsElement();
            if (prim.type == GeometricPrimitive3D::Cylinder)
            w = AnyCast_Raw<Math3D::Cylinder3D>(&prim.data)->radius * 2;
            param = int(w / param);
        }
        PrimitiveToMesh(prim, data, int(param));
        return true;
    }
    case Type::ConvexHull:
        ConvexHullToMesh(dynamic_cast<const Geometry3DConvexHull*>(geom)->data,data);    
        return true;
    default:
        return false;
    }
}

shared_ptr<Geometry3D> Geometry3DTriangleMesh::GetElement(int elem) const
{
    Math3D::Triangle3D tri;
    data.GetTriangle(elem, tri);
    return make_shared<Geometry3DPrimitive>(GeometricPrimitive3D(tri));
}

bool Geometry3DTriangleMesh::Load(const char *fn)
{
    data = Meshing::TriMesh();
    GLDraw::GeometryAppearance blank, temp;
    if (!Meshing::Import(fn, data, temp))
        return false;
    if (temp.faceColor != blank.faceColor ||
        !temp.vertexColors.empty() ||
        !temp.faceColors.empty()) { //loaded appearance data
        appearance.reset(new GLDraw::GeometryAppearance(temp));
    }
    else {
        appearance.reset();
    }
    return true;
}

vector<string> Geometry3DTriangleMesh::FileExtensions() const
{
  vector<string> res;
  Meshing::LoadTriMeshExtensions(res);
  return res;
}

bool Geometry3DTriangleMesh::Load(istream &in)
{
    auto pos = in.tellg();
    //try OFF and tri formats
    if(!LoadOFF(in,data)) {
        in.seekg(pos);
        in >> data;
        if(!in) return false;
    }
    appearance.reset();
    return true;
}

bool Geometry3DTriangleMesh::Save(const char *fn) const
{
    if(appearance)
        return Meshing::Export(fn, data, *appearance);
    return Meshing::Export(fn, data);
}

bool Geometry3DTriangleMesh::Save(ostream& out) const
{
    out << data << endl;
    return true;
}

bool Geometry3DTriangleMesh::Transform(const Matrix4 &T)
{
    data.Transform(T);
    return true;
}

AABB3D Geometry3DTriangleMesh::GetAABB() const
{
    AABB3D bb;
    data.GetAABB(bb.bmin, bb.bmax);
    return bb;
}

bool Geometry3DTriangleMesh::Support(const Vector3& dir,Vector3& pt) const
{
  if(data.verts.empty()) return false;
  Real farthest = -Inf;
  for(size_t i=0;i<data.verts.size();i++) {
    Real d=dir.dot(data.verts[i]);
    if(d > farthest) {
      farthest = d;
      pt = data.verts[i];
    }
  }
  return true;
}


bool Geometry3DTriangleMesh::Merge(const Geometry3D* geom,const RigidTransform* Tgeom)
{
  if(geom->GetType() != Type::TriangleMesh) return false;
  auto* meshgeom = dynamic_cast<const Geometry3DTriangleMesh*>(geom);
  if(Tgeom) {
    Meshing::TriMesh temp;
    temp = meshgeom->data;
    temp.Transform(*Tgeom);
    data.MergeWith(temp);
  }
  else {
    data.MergeWith(meshgeom->data);
  }
  return true;
}

bool Geometry3DTriangleMesh::Union(const vector<Geometry3D*>& geoms)
{
    for(size_t i=0;i<geoms.size();i++) {
        if(geoms[i]->GetType() != Type::TriangleMesh) return false;
        auto* meshgeom = dynamic_cast<const Geometry3DTriangleMesh*>(geoms[i]);
        if(meshgeom->appearance && !meshgeom->appearance->vertexColors.empty()) return false;
    }
    vector<Meshing::TriMesh> items(geoms.size());
    for(size_t i=0;i<geoms.size();i++)
        items[i] = dynamic_cast<const Geometry3DTriangleMesh*>(geoms[i])->data;
    data.Union(items);
    return true;
}

Geometry3D* Geometry3DTriangleMesh::Remesh(Real resolution,bool refine,bool coarsen) const
{
    if(resolution <= 0) return NULL;
    if(!refine) return NULL;
    Meshing::TriMeshWithTopology mesh;
    mesh.verts = data.verts;
    mesh.tris = data.tris;
    if(refine) {
        Meshing::SubdivideToResolution(mesh,resolution);
        return new Geometry3DTriangleMesh(mesh);
    }
    return NULL;
}



Geometry3D* Geometry3DTriangleMesh::Slice(const RigidTransform& T,Real tol) const
{
    const Meshing::TriMesh& mesh=data;
    vector<Segment2D> segs;
    vector<int> tri_indices;
    Geometry::SliceXY(mesh,T,segs,tri_indices);

    auto* res = new Geometry3DGroup();
    res->data.resize(segs.size());
    for(size_t i=0;i<segs.size();i++) {
        Segment3D seg;
        seg.a.set(segs[i].a.x,segs[i].a.y,0);
        seg.b.set(segs[i].b.x,segs[i].b.y,0);
        res->data[i] = AnyGeometry3D(seg);
    }
    return res;
}


Geometry3D* Geometry3DTriangleMesh::ExtractROI(const AABB3D& bb,int flags) const
{
    auto* res = new Geometry3DTriangleMesh;
    Geometry::ExtractROI(data,bb,res->data,flags);
    return res;
}

Geometry3D* Geometry3DTriangleMesh::ExtractROI(const Box3D& bb,int flags) const
{
    auto* res = new Geometry3DTriangleMesh;
    Geometry::ExtractROI(data,bb,res->data,flags);
    return res;
}


bool Collides(const CollisionMesh &a, const CollisionMesh &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  if (maxContacts == 1)
  {
    CollisionMeshQueryEnhanced query(a, b);
    query.margin1 = 0;
    query.margin2 = margin;
    bool res = query.Collide();
    if (res)
    {
      query.CollisionPairs(elements1, elements2);
      assert(elements1.size() == 1);
      assert(elements2.size() == 1);
    }
    return res;
  }
  NearbyTriangles(a, b, margin, elements1, elements2, (int)maxContacts);
  return !elements1.empty();
}

Collider3DTriangleMesh::Collider3DTriangleMesh(shared_ptr<Geometry3DTriangleMesh> _data)
:data(_data),collisionData(_data->data)
{
  Reset();
}

Collider3DTriangleMesh::Collider3DTriangleMesh(const Collider3DTriangleMesh& rhs)
:data(rhs.data),collisionData(rhs.collisionData)
{}

void Collider3DTriangleMesh::Reset()
{
  collisionData.InitCollisions();
}

Collider3D* Collider3DTriangleMesh::Copy() const
{
  return new Collider3DTriangleMesh(*this);
}

Collider3D* Collider3DTriangleMesh::ConvertTo(Type restype, Real param,Real domainExpansion)
{
    if(restype == Type::ConvexHull) {
      auto convex_hull = make_shared<Geometry3DConvexHull>();
      MeshConvexDecomposition(data->data,convex_hull->data,param);
      return new Collider3DConvexHull(convex_hull);
    }
    return NULL;
}

AABB3D Collider3DTriangleMesh::GetAABBTight() const
{
    const CollisionMesh &m = collisionData;
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < m.verts.size(); i++)
        bb.expand(m.currentTransform * m.verts[i]);
    return bb;
}

Box3D Collider3DTriangleMesh::GetBB() const
{
    Box3D b;
    Geometry::GetBB(collisionData, b);
    return b;
}

AABB3D Collider3DTriangleMesh::GetAABB() const
{
    Box3D b = GetBB();
    AABB3D bb;
    b.getAABB(bb);
    return bb;
}

bool Collider3DTriangleMesh::Distance(const Vector3& pt,Real& result)
{
    result = Geometry::Distance(collisionData, pt);
    return true;
}

bool Collider3DTriangleMesh::Distance(const Vector3 &pt, const AnyDistanceQuerySettings &settings,AnyDistanceQueryResult& res)
{
    res.hasClosestPoints = true;
    res.hasElements = true;
    res.elem2 = 0;
    res.cp2 = pt;

    const CollisionMesh& cm = collisionData;
    int tri = ClosestPoint(cm, pt, res.cp1, settings.upperBound);
    if(tri<0) res.d = settings.upperBound;
    else {
      res.cp1 = cm.currentTransform * res.cp1;
      res.elem2 = tri;
      res.d = pt.distance(res.cp1);
    }
    return true;
}


bool Collides(const GeometricPrimitive3D &a, const CollisionMesh &c, Real margin,
              vector<int> &meshelements, size_t maxContacts)
{
  NearbyTriangles(c, a, margin, meshelements, (int)maxContacts);
  return !meshelements.empty();
}

bool Collider3DTriangleMesh::WithinDistance(Collider3D* geom,Real d,
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
      elements2.push_back(0);
    }
    return true;
  }
  case Type::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do convex hull-anything collision yet");
    return false;
  case Type::TriangleMesh:
  {
        auto& b = dynamic_cast<Collider3DTriangleMesh*>(geom)->collisionData;
        Geometry::Collides(collisionData, b, d, elements1, elements2, maxContacts);
        return true;
  }
  default:
    return false; 
  }
}

bool Collider3DTriangleMesh::RayCast(const Ray3D& r,Real margin,Real& distance,int& element) 
{
  Vector3 worldpt;
  element = -1;
  int tri = Geometry::RayCast(collisionData, r, worldpt);
  if (tri >= 0)
  {
    distance = worldpt.distance(r.source);
    //TODO: this isn't perfect if the margin is > 0 -- will miss silhouettes
    distance -= margin;
    element = tri;
  }
  return true;
}

bool Collider3DTriangleMesh::Contains(const Vector3& pt,bool& result)
{
  Ray3D rtest;
  rtest.source = pt;
  const Real directions [14][3] = {
    {1,0,0},
    {-1,0,0},
    {0,1,0},
    {0,-1,0},
    {0,0,1},
    {0,0,-1},
    {1,1,1},
    {-1,1,1},
    {1,-1,1},
    {-1,-1,1},
    {1,1,-1},
    {-1,1,-1},
    {1,-1,-1},
    {-1,-1,-1},
  };
  Vector3 n;
  Vector3 worldpt;
  int numInside = 0, numOutside = 0;
  for(int i=0;i<14;i++) {
    rtest.direction.setNormalized(Vector3(directions[i]));
    int tri = Geometry::RayCast(collisionData,rtest,worldpt);
    if(tri >= 0) {
      n = data->data.TriangleNormal(tri);
      if(n.dot(rtest.direction) < 0)
        numInside += 1;
      else
        numOutside += 1;
    }
    else numOutside += 1;
  }
  return numInside > numOutside; //majority voting
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const CollisionMesh &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.elem1 = 0;
  res.d = Geometry::Distance(b, a, res.elem2, res.cp2, res.dir2, settings.upperBound);
  res.dir1.setNegative(res.dir2);
  res.cp1 = res.cp2 + res.d * res.dir2;
  return res;
}

AnyDistanceQueryResult Distance(const CollisionMesh &a, const CollisionMesh &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  CollisionMeshQuery q(a, b);
  res.d = q.Distance(settings.absErr, settings.relErr, settings.upperBound);
  q.ClosestPair(res.elem1, res.elem2);
  q.ClosestPoints(res.cp1, res.cp2);
  res.cp1 = a.currentTransform * res.cp1;
  res.cp2 = b.currentTransform * res.cp2;
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}

bool Collider3DTriangleMesh::Distance(Collider3D* geom, const AnyDistanceQuerySettings &settings,AnyDistanceQueryResult& res)
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
  case Type::TriangleMesh:
  {
    CollisionMesh& b = dynamic_cast<Collider3DTriangleMesh*>(geom)->collisionData;
    res = Geometry::Distance(collisionData, b, settings);
    return true;
  }
  break;
  case Type::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do mesh-convex hull distance yet");
    return false;
  default:
    return false;
  }
}

typedef ContactsQueryResult::ContactPair ContactPair;

//if a normal has this length then it is ignored
const static Real gZeroNormalTolerance = 1e-4;

//if two contact points are closer than this threshold, will try to look
//at the local geometry to derive a contact normal
const static Real gNormalFromGeometryTolerance = 1e-5;
//const static Real gNormalFromGeometryTolerance = 1e-2;

//if a barycentric coordinate is within this tolerance of zero, it will be
//considered a zero
const static Real gBarycentricCoordZeroTolerance = 1e-3;

//if true, takes the ODE tolerance points and performs additional contact
//checking -- useful for flat contacts
const static bool gDoTriangleTriangleCollisionDetection = false;

//doesn't consider unique contact points if they are between this tolerance
const static Real cptol = 1e-5;


//1 = pt, 2 = edge, 3 = face
inline int FeatureType(const Vector3 &b)
{
  int type = 0;
  if (FuzzyZero(b.x, gBarycentricCoordZeroTolerance))
    type++;
  if (FuzzyZero(b.y, gBarycentricCoordZeroTolerance))
    type++;
  if (FuzzyZero(b.z, gBarycentricCoordZeroTolerance))
    type++;
  return 3 - type;
}

int EdgeIndex(const Vector3 &b)
{
  if (FuzzyZero(b.x, gBarycentricCoordZeroTolerance))
    return 0;
  if (FuzzyZero(b.y, gBarycentricCoordZeroTolerance))
    return 1;
  if (FuzzyZero(b.z, gBarycentricCoordZeroTolerance))
    return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

int VertexIndex(const Vector3 &b)
{
  if (FuzzyEquals(b.x, One, gBarycentricCoordZeroTolerance))
    return 0;
  if (FuzzyEquals(b.y, One, gBarycentricCoordZeroTolerance))
    return 1;
  if (FuzzyEquals(b.z, One, gBarycentricCoordZeroTolerance))
    return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

Vector3 VertexNormal(CollisionMesh &m, int tri, int vnum)
{
  if (m.incidentTris.empty())
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "VertexNormal: mesh is not properly initialized with incidentTris array?");
    m.CalcIncidentTris();
    //return Vector3(0.0);
    //FatalError("VertexNormal: mesh is not properly initialized with incidentTris array?");
  }
  Assert(vnum >= 0 && vnum < 3);
  int v = m.tris[tri][vnum];
  Assert(v >= 0 && v < (int)m.incidentTris.size());
  if (m.incidentTris[v].empty())
    return Vector3(0.0);
  Vector3 n(Zero);
  for (size_t i = 0; i < m.incidentTris[v].size(); i++)
    n += m.TriangleNormal(m.incidentTris[v][i]);
  n.inplaceNormalize();
  return m.currentTransform.R * n;
}

Vector3 EdgeNormal(CollisionMesh &m, int tri, int e)
{
  if (m.triNeighbors.empty())
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "EdgeNormal: Warning, mesh is not properly initialized with triNeighbors");
    m.CalcTriNeighbors();
    //return Vector3(0.0);
  }
  Assert(!m.triNeighbors.empty());
  Vector3 n = m.TriangleNormal(tri);
  if (m.triNeighbors[tri][e] != -1)
  {
    n += m.TriangleNormal(m.triNeighbors[tri][e]);
    n.inplaceNormalize();
  }
  return m.currentTransform.R * n;
}

///Compute normal from mesh geometry: returns the local normal needed for
///triangle 1 on m1 to get out of triangle 2 on m2.
///p1 and p2 are given in local coordinates
Vector3 ContactNormal(CollisionMesh &m1, CollisionMesh &m2, const Vector3 &p1, const Vector3 &p2, int t1, int t2)
{
  Triangle3D tri1, tri2;
  m1.GetTriangle(t1, tri1);
  m2.GetTriangle(t2, tri2);
  Vector3 b1 = tri1.barycentricCoords(p1);
  Vector3 b2 = tri2.barycentricCoords(p2);
  int type1 = FeatureType(b1), type2 = FeatureType(b2);
  switch (type1)
  {
  case 1: //pt
    switch (type2)
    {
    case 1: //pt
      //get the triangle normals
      {
        //printf("ODECustomMesh: Point-point contact\n");
        Vector3 n1 = VertexNormal(m1, t1, VertexIndex(b1));
        Vector3 n2 = VertexNormal(m2, t2, VertexIndex(b2));
        n2 -= n1;
        n2.inplaceNormalize();
        return n2;
      }
      break;
    case 2: //edge
    {
      //printf("ODECustomMesh: Point-edge contact\n");
      Vector3 n1 = VertexNormal(m1, t1, VertexIndex(b1));
      int e = EdgeIndex(b2);
      Segment3D s = tri2.edge(e);
      Vector3 ev = m2.currentTransform.R * (s.b - s.a);
      Vector3 n2 = EdgeNormal(m2, t2, e);
      n2 -= (n1 - ev * ev.dot(n1) / ev.dot(ev)); //project onto normal
      n2.inplaceNormalize();
      return n2;
    }
    break;
    case 3: //face
      return m2.currentTransform.R * tri2.normal();
    }
    break;
  case 2: //edge
    switch (type2)
    {
    case 1: //pt
    {
      //printf("ODECustomMesh: Edge-point contact\n");
      Vector3 n2 = VertexNormal(m2, t2, VertexIndex(b2));
      int e = EdgeIndex(b1);
      Segment3D s = tri1.edge(e);
      Vector3 ev = m1.currentTransform.R * (s.b - s.a);
      Vector3 n1 = EdgeNormal(m1, t1, e);
      n2 = (n2 - ev * ev.dot(n2) / ev.dot(ev)) - n1; //project onto normal
      n2.inplaceNormalize();
      return n2;
    }
    break;
    case 2: //edge
    {
      //printf("ODECustomMesh: Edge-edge contact\n");
      int e = EdgeIndex(b1);
      Segment3D s1 = tri1.edge(e);
      Vector3 ev1 = m1.currentTransform.R * (s1.b - s1.a);
      ev1.inplaceNormalize();
      e = EdgeIndex(b2);
      Segment3D s2 = tri2.edge(e);
      Vector3 ev2 = m2.currentTransform.R * (s2.b - s2.a);
      ev2.inplaceNormalize();
      Vector3 n;
      n.setCross(ev1, ev2);
      Real len = n.length();
      if (len < gZeroNormalTolerance)
      {
        //hmm... edges are parallel?
      }
      n /= len;
      //make sure the normal direction points into m1 and out of m2
      if (n.dot(m1.currentTransform * s1.a) < n.dot(m2.currentTransform * s2.a))
        n.inplaceNegative();
      /*
        if(n.dot(m1.currentTransform.R*tri1.normal()) > 0.0) {
          if(n.dot(m2.currentTransform.R*tri2.normal()) > 0.0) {
            printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
          }
          n.inplaceNegative();
        }
        else {
          if(n.dot(m2.currentTransform.R*tri2.normal()) < 0.0) {
            printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
          }
        }
        */
      //cout<<"Edge vector 1 "<<ev1<<", vector 2" <<ev2<<", normal: "<<n<<endl;
      return n;
    }
    break;
    case 3: //face
      return m2.currentTransform.R * tri2.normal();
    }
    break;
  case 3: //face
    if (type2 == 3)
    {
      //printf("ODECustomMesh: Warning, face-face contact?\n");
    }
    return m1.currentTransform.R * (-tri1.normal());
  }
  static int warnedCount = 0;
  if (warnedCount % 10000 == 0)
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Warning, degenerate triangle, types " << type1 << " " << type2);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

//Returns a contact normal for the closest point to the triangle t.  p is the point on the triangle.
//The direction is the one in which triangle t can move to get away from closestpt
Vector3 ContactNormal(CollisionMesh &m, const Vector3 &p, int t, const Vector3 &closestPt)
{
  Triangle3D tri;
  m.GetTriangle(t, tri);
  Vector3 b = tri.barycentricCoords(p);
  int type = FeatureType(b);
  switch (type)
  {
  case 1: //pt
    //get the triangle normal
    {
      Vector3 n = VertexNormal(m, t, VertexIndex(b));
      n.inplaceNegative();
      return n;
    }
    break;
  case 2: //edge
  {
    int e = EdgeIndex(b);
    Vector3 n = EdgeNormal(m, t, e);
    n.inplaceNegative();
    return n;
  }
  break;
  case 3: //face
    return m.currentTransform.R * (-tri.normal());
  }
  static int warnedCount = 0;
  if (warnedCount % 10000 == 0)
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Warning, degenerate triangle, types " << type);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

void MeshMeshContacts(CollisionMesh &m1, Real outerMargin1, CollisionMesh &m2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  CollisionMeshQuery q(m1, m2);
  bool res = q.WithinDistanceAll(outerMargin1 + outerMargin2);
  if (!res)
  {
    return;
  }

  vector<int> t1, t2;
  vector<Vector3> cp1, cp2;
  q.TolerancePairs(t1, t2);
  q.TolerancePoints(cp1, cp2);
  vector<bool> unreliable(t1.size(), false);

  const RigidTransform &T1 = m1.currentTransform;
  const RigidTransform &T2 = m2.currentTransform;
  RigidTransform T21;
  T21.mulInverseA(T1, T2);
  RigidTransform T12;
  T12.mulInverseA(T2, T1);
  Real tol = outerMargin1 + outerMargin2;
  Real tol2 = Sqr(tol);

  size_t imax = t1.size();
  Triangle3D tri1, tri2, tri1loc, tri2loc;
  if (gDoTriangleTriangleCollisionDetection)
  {
    //test if more triangle vertices are closer than tolerance
    for (size_t i = 0; i < imax; i++)
    {
      m1.GetTriangle(t1[i], tri1);
      m2.GetTriangle(t2[i], tri2);

      tri1loc.a = T12 * tri1.a;
      tri1loc.b = T12 * tri1.b;
      tri1loc.c = T12 * tri1.c;
      tri2loc.a = T21 * tri2.a;
      tri2loc.b = T21 * tri2.b;
      tri2loc.c = T21 * tri2.c;
      bool usecpa, usecpb, usecpc, usecpa2, usecpb2, usecpc2;
      Vector3 cpa = tri1.closestPoint(tri2loc.a);
      Vector3 cpb = tri1.closestPoint(tri2loc.b);
      Vector3 cpc = tri1.closestPoint(tri2loc.c);
      Vector3 cpa2 = tri2.closestPoint(tri1loc.a);
      Vector3 cpb2 = tri2.closestPoint(tri1loc.b);
      Vector3 cpc2 = tri2.closestPoint(tri1loc.c);
      usecpa = (cpa.distanceSquared(tri2loc.a) < tol2);
      usecpb = (cpb.distanceSquared(tri2loc.b) < tol2);
      usecpc = (cpc.distanceSquared(tri2loc.c) < tol2);
      usecpa2 = (cpa2.distanceSquared(tri1loc.a) < tol2);
      usecpb2 = (cpb2.distanceSquared(tri1loc.b) < tol2);
      usecpc2 = (cpc2.distanceSquared(tri1loc.c) < tol2);
      //if already existing, disable it
      if (usecpa && cpa.isEqual(cp1[i], cptol))
        usecpa = false;
      if (usecpb && cpb.isEqual(cp1[i], cptol))
        usecpb = false;
      if (usecpc && cpc.isEqual(cp1[i], cptol))
        usecpc = false;
      if (usecpa2 && cpa2.isEqual(cp2[i], cptol))
        usecpa2 = false;
      if (usecpb2 && cpb2.isEqual(cp2[i], cptol))
        usecpb2 = false;
      if (usecpc2 && cpc2.isEqual(cp2[i], cptol))
        usecpc2 = false;

      if (usecpa)
      {
        if (usecpb && cpb.isEqual(cpa, cptol))
          usecpb = false;
        if (usecpc && cpc.isEqual(cpa, cptol))
          usecpc = false;
      }
      if (usecpb)
      {
        if (usecpc && cpc.isEqual(cpb, cptol))
          usecpc = false;
      }
      if (usecpa2)
      {
        if (usecpb2 && cpb2.isEqual(cpa2, cptol))
          usecpb2 = false;
        if (usecpc2 && cpc2.isEqual(cpa2, cptol))
          usecpc2 = false;
      }
      if (usecpb)
      {
        if (usecpc2 && cpc.isEqual(cpb2, cptol))
          usecpc2 = false;
      }

      if (usecpa)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpa);
        cp2.push_back(tri2.a);
      }
      if (usecpb)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpb);
        cp2.push_back(tri2.b);
      }
      if (usecpc)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpc);
        cp2.push_back(tri2.c);
      }
      if (usecpa2)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.a);
        cp2.push_back(cpa2);
      }
      if (usecpb2)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.b);
        cp2.push_back(cpb2);
      }
      if (usecpc2)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.c);
        cp2.push_back(cpc2);
      }
    }
    /*
    if(t1.size() != imax)
      printf("ODECustomMesh: Triangle vert checking added %d points\n",t1.size()-imax);
    */
    //getchar();
  }

  imax = t1.size();
  for (size_t i = 0; i < imax; i++)
  {
    m1.GetTriangle(t1[i], tri1);
    m2.GetTriangle(t2[i], tri2);

    //tri1loc.a = T12*tri1.a;
    //tri1loc.b = T12*tri1.b;
    //tri1loc.c = T12*tri1.c;
    tri2loc.a = T21 * tri2.a;
    tri2loc.b = T21 * tri2.b;
    tri2loc.c = T21 * tri2.c;
    Segment3D s;
    //this is here to avoid degenerate triangles
    bool collides;
    Vector3 n1, n2;
    n1.setCross(tri1.b - tri1.a, tri1.c - tri1.a);
    n2.setCross(tri2.b - tri2.a, tri2.c - tri2.a);
    if (n2.normSquared() > n1.normSquared())
      collides = tri2loc.intersects(tri1, s);
    else
      collides = tri1.intersects(tri2loc, s);
    if (collides)
    {
      unreliable[i] = true;
      /*
      cout<<"Triangle 1"<<endl;
      cout<<"  "<<tri1.a<<endl;
      cout<<"  "<<tri1.b<<endl;
      cout<<"  "<<tri1.c<<endl;
      cout<<"intersects triangle 2"<<endl;
      cout<<"  "<<tri2loc.a<<endl;
      cout<<"  "<<tri2loc.b<<endl;
      cout<<"  "<<tri2loc.c<<endl;
      */
      /*
      //the two triangles intersect! can't trust results of PQP
      t1[i] = t1.back();
      t2[i] = t2.back();
      cp1[i] = cp1.back();
      cp2[i] = cp2.back();
      i--;
      imax--;
      */
    }
  }
  if (t1.size() != imax)
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), t1.size() - imax << " candidate points were removed due to mesh collision");
    t1.resize(imax);
    t2.resize(imax);
    cp1.resize(imax);
    cp2.resize(imax);
  }

  contacts.reserve(cp1.size());
  for (size_t i = 0; i < cp1.size(); i++)
  {
    Vector3 p1 = T1 * cp1[i];
    Vector3 p2 = T2 * cp2[i];
    Vector3 n = p2 - p1;
    Real d = n.norm();
    if (d < gNormalFromGeometryTolerance)
    { //compute normal from the geometry
      n = ContactNormal(m1, m2, cp1[i], cp2[i], t1[i], t2[i]);
      n.inplaceNegative();
    }
    else if (d > tol)
    { //some penetration -- we can't trust the result of PQP
      LOG4CXX_WARN(GET_LOGGER(Geometry), "Skipping contact due to irregular distance between points " << d);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  cp 1 " << p1);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  cp 2 " << p2);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  local cp 1 " << cp1[i]);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  local cp 2 " << cp2[i]);
      continue;
    }
    else
      n /= d;
    //check for invalid normals
    Real len = n.length();
    if (len < gZeroNormalTolerance || !IsFinite(len))
    {
      LOG4CXX_WARN(GET_LOGGER(Geometry), "Skipping contact due to irregular normal length " << len);
      continue;
    }
    //cout<<"Local Points "<<cp1[i]<<", "<<cp2[i]<<endl;
    //cout<<"Points "<<p1<<", "<<p2<<endl;
    //Real utol = (tol)*0.5/d + 0.5;
    //CopyVector(contact[k].pos,p1+utol*(p2-p1));
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = p1 + outerMargin1 * n;
    contacts[k].p2 = p2 - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    if (contacts[k].depth < 0)
      contacts[k].depth = 0;
    contacts[k].elem1 = t1[i];
    contacts[k].elem2 = t2[i];
    contacts[k].unreliable = unreliable[i];
    //cout<<"Normal "<<n<<", depth "<<contact[i].depth<<endl;
    //getchar();
  }
}


void MeshSphereContacts(CollisionMesh &m1, Real outerMargin1, const Sphere3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Triangle3D tri;
  vector<int> tris;
  NearbyTriangles(m1, s.center, s.radius + tol, tris, maxcontacts);
  for (size_t j = 0; j < tris.size(); j++)
  {
    m1.GetTriangle(tris[j], tri);
    tri.a = m1.currentTransform * tri.a;
    tri.b = m1.currentTransform * tri.b;
    tri.c = m1.currentTransform * tri.c;

    Vector3 cp = tri.closestPoint(s.center);
    Vector3 n = s.center - cp;
    Real nlen = n.length();
    Real d = nlen - s.radius;
    Vector3 pw = s.center;
    if (s.radius > 0)
      //adjust pw to the sphere surface
      pw -= n * (s.radius / nlen);
    if (d < gNormalFromGeometryTolerance)
    { //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp, plocal);
      n = ContactNormal(m1, plocal, tris[j], pw);
      n.inplaceNegative();
    }
    else if (d > tol)
    { //some penetration -- we can't trust the result of PQP
      continue;
    }
    else
      n /= nlen;
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = cp + outerMargin1 * n;
    contacts[k].p2 = pw - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = tris[j];
    contacts[k].elem2 = -1;
    contacts[k].unreliable = false;
  }
  //filter by neighboring triangles
  if(!m1.triNeighbors.empty()) {
    //TODO: if the mesh is super fine, this may miss some duplicates. Instead should look for local minima?
    map<int,int> triToContact;
    for(size_t i=0;i<contacts.size();i++) {
      int t = contacts[i].elem1;
      triToContact[t] = (int)i;
    }
    for(size_t i=0;i<contacts.size();i++) {
      int t = contacts[i].elem1;
      const IntTriple& neighbors = m1.triNeighbors[t];
      if((triToContact.count(neighbors.a) && contacts[triToContact[neighbors.a]].depth > contacts[i].depth) ||
        (triToContact.count(neighbors.b) && contacts[triToContact[neighbors.b]].depth > contacts[i].depth) ||
        (triToContact.count(neighbors.c) && contacts[triToContact[neighbors.c]].depth > contacts[i].depth))
      {
        triToContact.erase(triToContact.find(t));
        int replaceT = contacts.back().elem1;
        triToContact[replaceT] = i;
        contacts[i] = contacts.back();
        contacts.resize(contacts.size()-1);
        i--;
      }
    }
  }
}

void MeshPrimitiveContacts(CollisionMesh &m1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  GeometricPrimitive3D gworld = g2;
  gworld.Transform(T2);

  if (gworld.type == GeometricPrimitive3D::Point)
  {
    Sphere3D s;
    s.center = *AnyCast<Point3D>(&gworld.data);
    s.radius = 0;
    MeshSphereContacts(m1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else if (gworld.type == GeometricPrimitive3D::Sphere)
  {
    const Sphere3D &s = *AnyCast<Sphere3D>(&gworld.data);
    MeshSphereContacts(m1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Distance computations between Triangles and " << gworld.TypeName() << " not supported");
    return;
  }
}


Collider3D* Collider3DTriangleMesh::Slice(const RigidTransform& T,Real tol) const
{
  vector<Segment2D> segs;
  vector<int> tri_indices;
  Geometry::SliceXY(collisionData,T,segs,tri_indices);
  auto* group = new Geometry3DGroup();
  vector<AnyGeometry3D>& segGeoms = group->data;
  segGeoms.resize(segs.size());
  for(size_t i=0;i<segs.size();i++) {
    Segment3D seg;
    seg.a.set(segs[i].a.x,segs[i].a.y,0);
    seg.b.set(segs[i].b.x,segs[i].b.y,0);
    segGeoms[i] = AnyGeometry3D(GeometricPrimitive3D(seg));
  }
  auto* res = Collider3D::Make(shared_ptr<Geometry3D>(group));
  res->SetTransform(T);
  return res;
}

Collider3D* Collider3DTriangleMesh::ExtractROI(const AABB3D& bb,int flags) const
{
  auto* res = new Collider3DTriangleMesh(make_shared<Geometry3DTriangleMesh>());
  Geometry::ExtractROI(collisionData,bb,res->collisionData,flags);
  res->data->data = res->collisionData;
  res->collisionData.InitCollisions();
  return res;
}

Collider3D* Collider3DTriangleMesh::ExtractROI(const Box3D& bb,int flags) const
{
  auto* res = new Collider3DTriangleMesh(make_shared<Geometry3DTriangleMesh>());
  Geometry::ExtractROI(collisionData,bb,res->collisionData,flags);
  res->data->data = res->collisionData;
  res->collisionData.InitCollisions();
  return res;
}

bool Collider3DTriangleMesh::Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) 
{
  switch (other->GetType())
  {
  case Type::Primitive:
    {
      auto* prim = dynamic_cast<Collider3DPrimitive*>(other);
      MeshPrimitiveContacts(collisionData, settings.padding1,
                            prim->data->data, prim->T, settings.padding2,
                            res.contacts, settings.maxcontacts);
      return true;
    }
  case Type::TriangleMesh:
    {
      auto* mesh = dynamic_cast<Collider3DTriangleMesh*>(other);
      MeshMeshContacts(collisionData, settings.padding1,
                      mesh->collisionData, settings.padding2,
                            res.contacts, settings.maxcontacts);
      return true;
    }
  case Type::ConvexHull:
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: triangle mesh-convex hull contacts");
    break;
  default:
    return false;
  }
  return false;
}

} //namespace Geometry

