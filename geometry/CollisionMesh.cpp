#include <KrisLibrary/Logger.h>
#include "CollisionMesh.h"
#include "PenetrationDepth.h"
#include <math3d/clip.h>
#include <iostream>
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




namespace Geometry {


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
      LOG4CXX_ERROR(KrisLibrary::logger(),"PenetrationDepth(): Error, the two objects aren't penetrating?!?!");
      LOG4CXX_INFO(KrisLibrary::logger(),"Distance "<<d);
      Abort();
    }
    LOG4CXX_WARN(KrisLibrary::logger(),"PenetrationDepth(): Warning, the approximate computation failed, returning "<<Max(-d,(Real)1e-5));
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
        LOG4CXX_WARN(KrisLibrary::logger(),"Warning, PQP detected overlapping triangles but we find they don't intersect?\n");
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
          LOG4CXX_WARN(KrisLibrary::logger(),"Warning, PQP detected overlapping triangles but we find they don't intersect?\n");
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
    LOG4CXX_ERROR(KrisLibrary::logger(),"PenetrationDepth_Cached(): Error, the two objects have no interior vertices!");
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
    LOG4CXX_WARN(KrisLibrary::logger(),"PenetrationPoints(): Warning, the two objects have no interior vertices!");
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
    LOG4CXX_INFO(KrisLibrary::logger(),"Returning closest points "<<p1<<", "<<p2<<", dir "<<d1);
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
    pworld = p;
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
      d += normalWeight*nworld.distanceSquared(tri.normal());
    dmax = dmin = d;
    closestTri = m.tris[t].id;
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
      LOG4CXX_INFO(KrisLibrary::logger(),"Popped bound with min distance: "<<temp.minDist);
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

	LOG4CXX_INFO(KrisLibrary::logger(),"Children "<<c1<<", "<<c2<<", distances "<<dbmin1<<", "<<dbmin2);
	Assert(dbmin1 <= dbmax1);
	Assert(dbmin2 <= dbmax2);
	if(dbmin1 < dmax) {
	  temp.index = c1;
	  temp.minDist = dbmin1;
	  VcV(temp.plocal,p1);
	  q.insert(temp);
	  if(dbmax1 < dmax) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Max distance set to "<<dbmax1<<" on child 1");
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
	    LOG4CXX_INFO(KrisLibrary::logger(),"Max distance set to "<<dbmax2<<" on child 2");
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
    LOG4CXX_INFO(KrisLibrary::logger(),numTrianglesChecked<<" triangles, "<<numBBsChecked<<" BBs checked");
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
      Vector3 temp=tri.closestPoint(pworld);
      Real d=temp.distanceSquared(pworld);
      if(normalWeight != 0) 
	d += normalWeight*nworld.distanceSquared(tri.normal());
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
      ToLocal(m.b[c1],pworld,p1);
      ToLocal(m.b[c2],pworld,p2);
      DistanceLimitsBV(m.b[c1].d,p1,dbmin1,dbmax1);
      DistanceLimitsBV(m.b[c2].d,p2,dbmin2,dbmax2);
      //increas the bounds if the normals are needed
      if(normalWeight != 0) {
	dbmax1=dbmax1+Two*normalWeight;
	dbmax2=dbmax2+Two*normalWeight;
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"Children "<<c1<<", "<<c2<<", distances "<<dbmin1<<", "<<dbmin2);
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
  Vector3 pworld,nworld;
  Real dmin,dmax;  //min squared distance, max squared distance bound
  int closestTri;
  Triangle3D tri;
  Vector3 cp;

  int numTrianglesChecked,numBBsChecked;
};

int ClosestPoint(const CollisionMesh& mesh,const Vector3& p,Vector3& cp)
{
  Vector3 plocal;
  mesh.currentTransform.mulInverse(p,plocal);
  ClosestPointCallback cb;
  cb.Execute(*mesh.pqpModel,plocal);
  cp = cb.cp;

  /*
  //TEST
  int regularTri = mesh.ClosestPoint(p,cp);
  if(!FuzzyEquals(cb.cp.distance(p),cp.distance(p),1e-3)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ClosestPoint error: "<<cb.cp.distance(p)<<" != "<<cp.distance(p));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Triangle "<<cb.closestTri<<" vs "<<regularTri);
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"Collide: Collider for type "<<g.TypeName());
    return false;
  }
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
	FatalError("Not yet able to within-distance test of %s with a CollisionMesh\n",a.TypeName());
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
    FatalError("Not yet able to collide a primitive of type %s with a CollisionMesh\n",a.TypeName());
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
  CollideAllRecurse(s,*m.pqpModel,0,tris,max);
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"CollideAll: Collider for type "<<g.TypeName());
  }
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
        FatalError("Not yet able to within-distance test of %s with a CollisionMesh\n",g.TypeName());
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"NearbyTriangles: Collider for type "<<g.TypeName());
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
  //TODO: take into account current transform of the mesh?
  ClosestPointCallback cb;
  cb.normalWeight = nWeight;
  cb.nworld = n;
  cb.Execute(*mesh.pqpModel,p);
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
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: PQP says triangles intersect, but I don't find an intersection\n");
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






} //namespace Geometry
