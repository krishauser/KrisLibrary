#include "CollisionMesh.h"
#include <math3d/random.h>
#include <math3d/clip.h>
#include <math/random.h>
#include <map>
using namespace Meshing;
using namespace std;

const static Real third = 1.0/3.0;

#ifdef HAVE_PQP
#include <PQP.h>
#include <../src/MatVec.h>
#include <../src/OBB_Disjoint.h>
#include <../src/TriDist.h>

struct ToleranceAllResult
{
  /*
  vector<Real> triDist1,triDist2;       //closest distance per triangle
  vector<int> triPartner1,triPartner2;  //closest triangle on the other
  vector<pair<Vector3,Vector3> > triCp1,triCp2;   //closest points per triangle
  */
  map<int,Real> triDist1,triDist2;       //closest distance per triangle
  map<int,int> triPartner1,triPartner2;  //closest triangle on the other
  map<int,pair<Vector3,Vector3> > triCp1,triCp2;   //closest points per triangle
};

class PQP_Results
{
public:
  PQP_CollideResult collide;
  PQP_DistanceResult distance;
  PQP_ToleranceResult tolerance;
  ToleranceAllResult toleranceAll;
};

#endif



namespace Geometry {

//my new hack to return all pairs
int
PQP_Tolerance2(PQP_ToleranceResult* res,PQP_REAL R1[3][3], PQP_REAL T1[3], PQP_Model *o1,
	       PQP_REAL R2[3][3], PQP_REAL T2[3], PQP_Model *o2,
	       PQP_REAL tolerance,ToleranceAllResult& allRes);


#ifdef HAVE_PQP

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


const CollisionMesh& CollisionMesh::operator = (const CollisionMesh& model)
{
  SafeDelete(pqpModel);
  TriMeshWithTopology::operator = (model);
  currentTransform = model.currentTransform;

  if(!tris.empty())
    InitCollisions();
  return *this;
}



CollisionMeshQuery::CollisionMeshQuery(CollisionMesh& _m1, CollisionMesh& _m2)
  :m1(_m1),m2(_m2),
   penetration1(m1,m2),penetration2(m2,m1)
{
  pqpResults = new PQP_Results;
}

CollisionMeshQuery::~CollisionMeshQuery()
{
  delete pqpResults;
}

bool CollisionMeshQuery::Collide()
{
  if(m1.tris.empty() || m2.tris.empty()) return false;
  if(m1.pqpModel == NULL || m2.pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  int res=PQP_Collide(&pqpResults->collide,
		      R1,T1,m1.pqpModel,
		      R2,T2,m2.pqpModel,
		      PQP_FIRST_CONTACT);
  Assert(res == PQP_OK);
  return (pqpResults->collide.Colliding()!=0);
}

bool CollisionMeshQuery::CollideAll()
{
  if(m1.tris.empty() || m2.tris.empty()) return false;
  if(m1.pqpModel == NULL || m2.pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  int res=PQP_Collide(&pqpResults->collide,
		      R1,T1,m1.pqpModel,
		      R2,T2,m2.pqpModel,
		      PQP_ALL_CONTACTS);
  Assert(res == PQP_OK);
  return (pqpResults->collide.Colliding()!=0);
}

Real CollisionMeshQuery::Distance(Real absErr,Real relErr)
{
  if(m1.tris.empty() || m2.tris.empty()) return Inf;
  if(m1.pqpModel == NULL || m2.pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  int res = PQP_Distance(&pqpResults->distance,
			 R1,T1,m1.pqpModel,
			 R2,T2,m2.pqpModel,
			 relErr,absErr,
			 100);
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

  penetration1.Reset();
  penetration1.ComputeInitial(m1.currentTransform,m2.currentTransform,&tc1[0],&tc2[0],n);
  penetration1.ComputeDepth();
  penetration2.Reset();
  penetration2.ComputeInitial(m2.currentTransform,m1.currentTransform,&tc2[0],&tc1[0],n);
  penetration2.ComputeDepth();
  if(penetration1.maxDepth <= 0 && penetration2.maxDepth <= 0) {
    Real d=Distance(1e-3,1e-2);
    if(d > 1e-3) {
      cout<<"PenetrationDepth(): Error, the two objects aren't penetrating?!?!"<<endl;
      cout<<"Distance "<<d<<endl;
      Abort();
    }
    cout<<"PenetrationDepth(): Warning, the approximate computation failed, returning "<<Max(-d,(Real)1e-5)<<endl;
    //getchar();
    return Max(-d,(Real)1e-5);
  }
  if(penetration1.maxDepth <= 0) return penetration2.maxDepth;
  else if(penetration2.maxDepth <= 0) return penetration1.maxDepth;
  else return Min(penetration1.maxDepth,penetration2.maxDepth);
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
  const ToleranceAllResult& allRes = pqpResults->toleranceAll;
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
  for(map<int,int>::const_iterator i=allRes.triPartner1.begin();i!=allRes.triPartner1.end();i++) {
    t1.push_back(i->first);
    t2.push_back(i->second);
  }
  for(map<int,int>::const_iterator i=allRes.triPartner2.begin();i!=allRes.triPartner2.end();i++) {
    if(allRes.triPartner1.find(i->second)->second != i->first) { //avoid double counting  
      t1.push_back(i->second);
      t2.push_back(i->first);
    }
  }
}

void CollisionMeshQuery::TolerancePoints(vector<Vector3>& p1,vector<Vector3>& p2) const
{
  const ToleranceAllResult& allRes = pqpResults->toleranceAll;
  p1.resize(0);
  p2.resize(0);
  //fill out pairs from the results
  /*
  for(size_t i=0;i<allRes.triPartner1.size();i++)
    if(allRes.triPartner1[i] >= 0) {
      p1.push_back(allRes.triCp1[i].first);
      p2.push_back(allRes.triCp1[i].second);
    }
  for(size_t i=0;i<allRes.triPartner2.size();i++) {
    if(allRes.triPartner2[i] >= 0 && allRes.triPartner1[allRes.triPartner2[i]] != (int)i) { //avoid double counting  
      p1.push_back(allRes.triCp2[i].first);
      p2.push_back(allRes.triCp2[i].second);
    }
  }
  */
  for(map<int,pair<Vector3,Vector3> >::const_iterator i=allRes.triCp1.begin();i!=allRes.triCp1.end();i++) {
    p1.push_back(i->second.first);
    p2.push_back(i->second.second);
  }
  for(map<int,int>::const_iterator i=allRes.triPartner2.begin();i!=allRes.triPartner2.end();i++) {
    if(allRes.triPartner1.find(i->second)->second != i->first) { //avoid double counting  
      Assert(allRes.triCp2.find(i->first) != allRes.triCp2.end());
      p1.push_back(allRes.triCp2.find(i->first)->second.first);
      p2.push_back(allRes.triCp2.find(i->first)->second.second);
    }
  }
}

Real CollisionMeshQuery::PenetrationDepth_Cached()
{
  if(penetration1.maxDepth <= 0 && penetration2.maxDepth <= 0) {
    cout<<"PenetrationDepth_Cached(): Error, the two objects have no interior vertices!"<<endl;
    Abort();
  }
  if(penetration1.maxDepth <= 0) return penetration2.maxDepth;
  else if(penetration2.maxDepth <= 0) return penetration1.maxDepth;
  else return Min(penetration1.maxDepth,penetration2.maxDepth);
}

Real CollisionMeshQuery::Distance_Coherent(Real absErr,Real relErr)
{
  if(m1.tris.empty() || m2.tris.empty()) return Inf;
  if(m1.pqpModel == NULL || m2.pqpModel == NULL) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  int res = PQP_Distance(&pqpResults->distance,
			 R1,T1,m1.pqpModel,
			 R2,T2,m2.pqpModel,
			 relErr,absErr,
			 2);
  Assert(res == PQP_OK);
  return pqpResults->distance.Distance();
}

bool CollisionMeshQuery::WithinDistance(Real tol)
{
  if(m1.tris.empty() || m2.tris.empty()) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  int res = PQP_Tolerance(&pqpResults->tolerance,
			 R1,T1,m1.pqpModel,
			 R2,T2,m2.pqpModel,
			 tol);
  Assert(res == PQP_OK);
  return pqpResults->tolerance.CloserThanTolerance();
}

bool CollisionMeshQuery::WithinDistanceAll(Real tol)
{
  if(m1.tris.empty() || m2.tris.empty()) return false;
  PQP_REAL R1[3][3],T1[3],R2[3][3],T2[3];
  RigidTransformToPQP(m1.currentTransform,R1,T1);
  RigidTransformToPQP(m2.currentTransform,R2,T2);
  int res = PQP_Tolerance2(&pqpResults->tolerance,
			   R1,T1,m1.pqpModel,
			   R2,T2,m2.pqpModel,
			   tol,pqpResults->toleranceAll);
  Assert(res == PQP_OK);
  return pqpResults->tolerance.CloserThanTolerance();
}

void CollisionMeshQuery::ClosestPoints(Vector3& p1,Vector3& p2)
{
  const PQP_REAL* P1 = pqpResults->distance.P1();
  const PQP_REAL* P2 = pqpResults->distance.P2();
  p1.set(P1[0],P1[1],P1[2]);
  p2.set(P2[0],P2[1],P2[2]);
  //these are in local coords
}

void CollisionMeshQuery::TolerancePoints(Vector3& p1,Vector3& p2)
{
  const PQP_REAL* P1 = pqpResults->tolerance.P1();
  const PQP_REAL* P2 = pqpResults->tolerance.P2();
  p1.set(P1[0],P1[1],P1[2]);
  p2.set(P2[0],P2[1],P2[2]);
}


//d1 is the direction that m2 can move to get out of m1
void CollisionMeshQuery::PenetrationPoints(Vector3& p1,Vector3& p2,Vector3& d1)
{
  const RigidTransform& f1=m1.currentTransform;
  const RigidTransform& f2=m2.currentTransform;
  if(penetration1.maxDepth > 0) p1 = penetration1.deepestPoint;
  if(penetration2.maxDepth > 0) p2 = penetration2.deepestPoint;
  if(penetration1.maxDepth <= 0 && penetration2.maxDepth <= 0) {
    cout<<"PenetrationPoints(): Warning, the two objects have no interior vertices!"<<endl;
    //TODO : estimate penetration points/distance using collision pairs
    //closest points between m1 and m2
    Real closestDist = Inf, d;
    Triangle3D t1,t2;
    Vector3 v;
    for(size_t i=0;i<tc1.size();i++) {
      m1.GetTriangle(tc1[i],t1);
      m2.GetTriangle(tc2[i],t2);
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
    
    //SampleSphere(1,d1);
    d1 = p2-p1;
    d1.inplaceNormalize();
    cout<<"Returning closest points "<<p1<<", "<<p2<<", dir "<<d1<<endl;
    //getchar();
    return;
  }
  else {
    Real pen1 = (penetration1.maxDepth <= 0 ? Inf: penetration1.maxDepth);
    Real pen2 = (penetration2.maxDepth <= 0 ? Inf: penetration2.maxDepth);
    if(pen1 < pen2) {
      f1.mulVector(penetration1.deepestNormal,d1);
      Vector3 pt = p1; pt.madd(penetration1.deepestNormal,penetration1.maxDepth);
      Vector3 ptworld;
      f1.mulPoint(pt,ptworld);
      f2.mulPointInverse(ptworld,p2);
    }
    else {
      f2.mulVector(penetration2.deepestNormal,d1);
      d1.inplaceNegative();
      Vector3 pt = p2; pt.madd(penetration2.deepestNormal,penetration2.maxDepth);
      Vector3 ptworld;
      f2.mulPoint(pt,ptworld);
      f1.mulPointInverse(ptworld,p1);
    }
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
  CollisionMeshQuery q(cm1,cm2);
  return q.Collide();
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
  if(box_dims[0] == 0.0) b.bmax.x = 1e-8;  //to avoid numerical errors
  if(box_dims[1] == 0.0) b.bmax.y = 1e-8;  //to avoid numerical errors
  if(box_dims[2] == 0.0) b.bmax.z = 1e-8;  //to avoid numerical errors
  Ray3D r;
  Copy(s,r.source);
  Copy(d,r.direction);
  Real min=0,max=Inf;
  if(ClipLine(s,d,b,min,max)) return min;
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





//returns the index of a triangle colliding with g
//geometry g is specified relative to the frame of m[b]'s parent
template <class Geom>
int CollideRecurse(const Geom& g0,const PQP_Model& m,int b,const Geom& g,Vector3& pt)
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
      if(Collide(tri,g0,pt))
	return m.tris[t].id;
    }
    else {
      int c1=m.b[b].first_child;
      int c2=c1+1;
      int t=CollideRecurse(g0,m,c1,gloc,pt);
      if(t != -1) return t;
      t=CollideRecurse(g0,m,c2,gloc,pt);
      return t;
    }
  }
  return -1;
}


//geometry g is specified relative to the frame of m[b]'s parent
template <class Geom>
void CollideAllRecurse(const Geom& g0,const PQP_Model& m,int b,const Geom& g,vector<int>& tris)
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
      if(Collide(tri,g0,pt))
	tris.push_back(m.tris[t].id);
    }
    else {
      int c1=m.b[b].first_child;
      int c2=c1+1;
      CollideAllRecurse(g0,m,c1,gloc,tris);
      CollideAllRecurse(g0,m,c2,gloc,tris);
    }
  }
}




//NOTE: transformations in bounding volume heirarchy are relative to parent
//triangles in bounding volume are relative to world frame (object frame)
struct ClosestPointCallback
{
  struct ActivePair
  {
    bool operator < (const ActivePair& rhs) const { return minDist < rhs.minDist; }
    int index;          //bound index
    Real minDist;       //minimum distance of bound b
    PQP_REAL plocal[3]; //local position of point
  };

  ClosestPointCallback()
    :normalWeight(0),dmin(Inf),dmax(Inf),closestTri(-1)
  {}
  void Execute(const PQP_Model& m,const Vector3& p) {
    Assert(m.num_bvs != 0);
    pworld = p;
    //compute distance of random triangle -- perhaps faster pruning
    int t = RandInt(m.num_tris);
    Copy(m.tris[t].p1,tri.a);
    Copy(m.tris[t].p2,tri.b);
    Copy(m.tris[t].p3,tri.c);
    cp=tri.closestPoint(pworld);
    Real d=cp.distanceSquared(pworld);
    if(normalWeight != 0) 
      d += normalWeight*nworld.distanceSquared(tri.normal());
    dmax = dmin = d;
    closestTri = m.tris[t].id;
    numTrianglesChecked = 1;
    numBBsChecked = 0;
    Vector3 ploc;
    ToLocal(m.b[0],p,ploc);
    if(normalWeight != 0) {
      //do this "soft" recursion
      ExecuteRecurse(m,0,ploc);
      return;
    }
    ExecuteRecurse(m,0,ploc);
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
      cout<<"Popped bound with min distance: "<<temp.minDist<<endl;
      int b=temp.index;

      bool prune = false;
      if(m.b[b].Leaf()) { //it's a triangle -- must be the closest
	int t = -m.b[b].first_child - 1;
	Assert(t < m.num_tris);
	Copy(m.tris[t].p1,tri.a);
	Copy(m.tris[t].p2,tri.b);
	Copy(m.tris[t].p3,tri.c);
	cp=tri.closestPoint(pworld);
	Real d=cp.distanceSquared(pworld);
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

	cout<<"Children "<<c1<<", "<<c2<<", distances "<<dbmin1<<", "<<dbmin2<<endl;
	Assert(dbmin1 <= dbmax1);
	Assert(dbmin2 <= dbmax2);
	if(dbmin1 < dmax) {
	  temp.index = c1;
	  temp.minDist = dbmin1;
	  VcV(temp.plocal,p1);
	  q.insert(temp);
	  if(dbmax1 < dmax) {
	    cout<<"Max distance set to "<<dbmax1<<" on child 1"<<endl;
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
	    cout<<"Max distance set to "<<dbmax2<<" on child 2"<<endl;
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
    cout<<numTrianglesChecked<<" triangles, "<<numBBsChecked<<" BBs checked"<<endl;
    */
  }

  //p is assumed to be in local coordinates
  void ExecuteRecurse(const PQP_Model& m,int b,const Vector3& p)
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
      ToLocal(m.b[c1],p,p1);
      ToLocal(m.b[c2],p,p2);
      DistanceLimitsBV(m.b[c1].d,p1,dbmin1,dbmax1);
      DistanceLimitsBV(m.b[c2].d,p2,dbmin2,dbmax2);
      //increas the bounds if the normals are needed
      if(normalWeight != 0) {
	dbmax1=dbmax1+Two*normalWeight;
	dbmax2=dbmax2+Two*normalWeight;
      }
      //cout<<"Children "<<c1<<", "<<c2<<", distances "<<dbmin1<<", "<<dbmin2<<endl;
      bool reverse=false;
      if(dbmin2 == dbmin1) { //point is in the bboxes
	reverse = (dbmax2 < dbmax1);
      }
      else reverse = (dbmin2 < dbmin1);
      
      if(reverse) {
	if(dbmax2 < dmax) dmax = dbmax2;
	if(dbmax1 < dmax) dmax = dbmax1;
	if(dbmin2 < dmax) ExecuteRecurse(m,c2,p2);
	if(dbmin1 < dmax) ExecuteRecurse(m,c1,p1);
      }
      else {
	if(dbmax1 < dmax) dmax = dbmax1;
	if(dbmax2 < dmax) dmax = dbmax2;
	if(dbmin1 < dmax) ExecuteRecurse(m,c1,p1);
	if(dbmin2 < dmax) ExecuteRecurse(m,c2,p2);
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
  //TODO: take into account current transform of the mesh?
  ClosestPointCallback cb;
  cb.Execute(*mesh.pqpModel,p);
  cp = cb.cp;

  /*
  //TEST
  int regularTri = mesh.ClosestPoint(p,cp);
  if(!FuzzyEquals(cb.cp.distance(p),cp.distance(p),1e-3)) {
    cerr<<"ClosestPoint error: "<<cb.cp.distance(p)<<" != "<<cp.distance(p)<<endl;
    cerr<<"Triangle "<<cb.closestTri<<" vs "<<regularTri<<endl;
  }
  Assert(FuzzyEquals(cb.cp.distance(p),cp.distance(p),1e-3));
  */
  return cb.closestTri;
}




int Collide(const CollisionMesh& m,const Segment3D& s,Vector3& pt)
{
  return CollideRecurse(s,*m.pqpModel,0,s,pt);
}

bool Collide(const CollisionMesh& m,const Sphere3D& s)
{
  Vector3 pt;
  return CollideRecurse(s,*m.pqpModel,0,s,pt)>=0;
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
  Vector3 pt;
  BV bv;
  BoxToBV(b,bv);
  return CollideRecurse(bv,*m.pqpModel,0,bv,pt)>=0;
}




void CollideAll(const CollisionMesh& m,const Sphere3D& s,vector<int>& tris)
{
  tris.resize(0);
  CollideAllRecurse(s,*m.pqpModel,0,s,tris);
}

void CollideAll(const CollisionMesh& m,const Segment3D& s,vector<int>& tris)
{
  tris.resize(0);
  CollideAllRecurse(s,*m.pqpModel,0,s,tris);
}

void CollideAll(const CollisionMesh& m,const AABB3D& bb,vector<int>& tris)
{
  Box3D box;
  box.xbasis.set(1,0,0);
  box.ybasis.set(0,1,0);
  box.zbasis.set(0,0,1);
  box.origin = bb.bmin;
  box.dims = bb.bmax-bb.bmin;
  CollideAll(m,box,tris);
}

void CollideAll(const CollisionMesh& m,const Box3D& bb,vector<int>& tris)
{
  //TODO: take into account current transform of the mesh?  
  BV bv;
  BoxToBV(bb,bv);
  tris.resize(0);
  CollideAllRecurse(bv,*m.pqpModel,0,bv,tris);
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
void NearbyTriangles(const CollisionMesh& m,const Vector3& p,Real d,vector<int>& tris)
{
  Sphere3D s;
  s.center=p;
  s.radius=d;
  CollideAll(m,s,tris);
}




int ClosestPointAndNormal(const TriMesh& m,Real pWeight,Real nWeight,const Vector3& p,const Vector3& n,Vector3& cp)
{
  Real dmin=Inf;
  int t=-1;
  Triangle3D tri;
  for(size_t i=0;i<m.tris.size();i++) {
    m.GetTriangle(i,tri);
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

struct RayCastCallback
{
  RayCastCallback(const PQP_Model& _mesh,const Ray3D& _r)
    :m(_mesh),r(_r)
  {}

  void Compute() {
    closestParam = Inf;
    closestTri = -1;
    if(m.num_bvs==0) return;  //empty model

    PQP_REAL S0[3],D0[3],S[3],D[3],temp[3];
    Copy(r.source,S0);
    Copy(r.direction,D0);
    //transform to initial frame
    VmV(temp,S0,m.b[0].To);
    MTxV(S,m.b[0].R,temp);
    MTxV(D,m.b[0].R,D0);
    //test bounding box of whole object
    Real p=aabb_ray_collide(m.b[0].d,S,D);  //inf returned if no collision
    if(IsInf(p)) {
      return;
    }
    //recurse on root of bb heirarchy
    Recurse(0,S,D);
  }

  //S is the ray source, D is the direction relative to the frame of m[b].
  void Recurse(int b,const PQP_REAL S[3],const PQP_REAL D[3])
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
      PQP_REAL S1[3],D1[3],S2[3],D2[3],temp[3];
      int c1=m.b[b].first_child;
      int c2=c1+1;
      
      VmV(temp,S,m.b[c1].To);
      MTxV(S1,m.b[c1].R,temp);
      MTxV(D1,m.b[c1].R,D);
      VmV(temp,S,m.b[c2].To);
      MTxV(S2,m.b[c2].R,temp);
      MTxV(D2,m.b[c2].R,D);
      //inf returned if no collision
      Real p1=aabb_ray_collide(m.b[c1].d,S1,D1);
      Real p2=aabb_ray_collide(m.b[c2].d,S2,D2);
    
      if(p1 < p2) {  //do child 1 first
	if(p1 < closestParam) Recurse(c1,S1,D1);
	if(p2 < closestParam) Recurse(c2,S2,D2);
      }
      else {   //do child 2 first
	if(p2 < closestParam) Recurse(c2,S2,D2);
	if(p1 < closestParam) Recurse(c1,S1,D1);
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
  //TODO: take into account current transform of the mesh?
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
    Triangle3D a(tri1[0],tri1[1],tri1[2]);
    Triangle3D b(tri2[0],tri2[1],tri2[2]);
    Segment3D s;
    bool res=a.intersects(b,s);
    if(!res) {
      printf("Warning: PQP says triangles intersect, but I don't find an intersection\n");
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


// Tolerance Stuff
//
//---------------------------------------------------------------------------

//if the distance between b1 and b2 is <= tolerance, returns the
//distance.  Otherwise, returns some number > tolerance
inline PQP_REAL BV_CheckTolerance(PQP_REAL R[3][3], PQP_REAL T[3],BV* b1,BV* b2,PQP_REAL tol)
{
  return BV_Distance(R, T, b1, b2);

  BV temp1;
  temp1 = *b1;
  temp1.r += tol;
  temp1.d[0] += tol;
  temp1.d[1] += tol;
  temp1.d[2] += tol;
  if(BV_Overlap(R,T,&temp1,b2))  //quick reject
    return BV_Distance(R, T, b1, b2);
  else
    return tol + 1e-4;
}

void 
ToleranceRecurse2(PQP_ToleranceResult *res, 
                 PQP_REAL R[3][3], PQP_REAL T[3],
		  PQP_Model *o1, int b1, PQP_Model *o2, int b2, ToleranceAllResult& allRes)
{
  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2) 
  {
    // both leaves - find if tri pair within tolerance
    
    res->num_tri_tests++;

    PQP_REAL p[3], q[3];

    int ti1=-o1->child(b1)->first_child - 1;
    int ti2=-o2->child(b2)->first_child - 1;
    //printf("Testing leaves %d %d\n",ti1,ti2);
    Tri *t1 = &o1->tris[ti1];
    Tri *t2 = &o2->tris[ti2];

    PQP_REAL d = TriDistance(res->R,res->T,t1,t2,p,q);
    
    if (d <= res->tolerance)  
    {  
      // triangle pair distance less than tolerance
      //printf("Triangle pair %d %d within tolerance, distance %g\n",ti1,ti2,d);
      res->closer_than_tolerance = 1;
      if(d < res->distance) {
	res->distance = d;
	VcV(res->p1, p);         // p already in c.s. 1
	VcV(res->p2, q);         // q must be transformed 
                                 // into c.s. 2 later
      }
      //mark it in the list
      if(allRes.triDist1.count(ti1)==0 || d < allRes.triDist1[ti1]) {
	allRes.triDist1[ti1] = d;
	allRes.triPartner1[ti1]=ti2;
	allRes.triCp1[ti1] = pair<Vector3,Vector3>(p,q);
      }
      if(allRes.triDist2.count(ti2)==0 || d < allRes.triDist2[ti2]) {
	allRes.triDist2[ti2] = d;
	allRes.triPartner2[ti2]=ti1;
	allRes.triCp2[ti2] = pair<Vector3,Vector3>(p,q);
      }
    }
    return;
  }

  int a1,a2,c1,c2;  // new bv tests 'a' and 'c'
  PQP_REAL R1[3][3], T1[3], R2[3][3], T2[3], Ttemp[3];
  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();

  PQP_REAL lim1=res->tolerance,lim2=res->tolerance;
  if(l2) {  //clip points of interest by existing per-triangle tolerance
    int ti2=-o2->child(b2)->first_child - 1;
    lim1=allRes.triDist2[ti2];
  }
  if(l1) {  //clip points of interest by existing per-triangle tolerance
    int ti1=-o1->child(b1)->first_child - 1;
    lim2=allRes.triDist1[ti1];
  }
  if (l2 || (!l1 && (sz1 > sz2)))
  {
    // visit the children of b1 
    a1 = o1->child(b1)->first_child;
    a2 = b2;
    c1 = o1->child(b1)->first_child+1;
    c2 = b2;
    
    MTxM(R1,o1->child(a1)->R,R);
#if PQP_BV_TYPE & RSS_TYPE
    VmV(Ttemp,T,o1->child(a1)->Tr);
#else
    VmV(Ttemp,T,o1->child(a1)->To);
#endif
    MTxV(T1,o1->child(a1)->R,Ttemp);

    MTxM(R2,o1->child(c1)->R,R);
#if PQP_BV_TYPE & RSS_TYPE
    VmV(Ttemp,T,o1->child(c1)->Tr);
#else
    VmV(Ttemp,T,o1->child(c1)->To);
#endif
    MTxV(T2,o1->child(c1)->R,Ttemp);
  }
  else 
  {
    // visit the children of b2
    a1 = b1;
    a2 = o2->child(b2)->first_child;
    c1 = b1;
    c2 = o2->child(b2)->first_child+1;

    MxM(R1,R,o2->child(a2)->R);
#if PQP_BV_TYPE & RSS_TYPE
    MxVpV(T1,R,o2->child(a2)->Tr,T);
#else
    MxVpV(T1,R,o2->child(a2)->To,T);
#endif
    MxM(R2,R,o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
    MxVpV(T2,R,o2->child(c2)->Tr,T);
#else
    MxVpV(T2,R,o2->child(c2)->To,T);
#endif
  }

  res->num_bv_tests += 2;

  PQP_REAL d1 = BV_CheckTolerance(R1, T1, o1->child(a1), o2->child(a2),lim1);
  PQP_REAL d2 = BV_CheckTolerance(R2, T2, o1->child(c1), o2->child(c2),lim2);

  if (d2 < d1) 
  {
    if (d2 <= lim2) ToleranceRecurse2(res, R2, T2, o1, c1, o2, c2, allRes);
    if (d1 <= lim1) ToleranceRecurse2(res, R1, T1, o1, a1, o2, a2, allRes);
  }
  else 
  {
    if (d1 <= lim1) ToleranceRecurse2(res, R1, T1, o1, a1, o2, a2, allRes);
    if (d2 <= lim2) ToleranceRecurse2(res, R2, T2, o1, c1, o2, c2, allRes);
  }
}


int
PQP_Tolerance2(PQP_ToleranceResult* res,PQP_REAL R1[3][3], PQP_REAL T1[3], PQP_Model *o1,
	       PQP_REAL R2[3][3], PQP_REAL T2[3], PQP_Model *o2,
	       PQP_REAL tolerance,ToleranceAllResult& allRes)
{
  // make sure that the models are built
  if (o1->build_state != 2) 
    return PQP_ERR_UNPROCESSED_MODEL;
  if (o2->build_state != 2) 
    return PQP_ERR_UNPROCESSED_MODEL;

  allRes.triDist1.clear();
  allRes.triPartner1.clear();
  allRes.triCp1.clear();
  allRes.triDist2.clear();
  allRes.triPartner2.clear();
  allRes.triCp2.clear();
    /*
    allRes.triDist1.resize(o1->num_tris);
    allRes.triPartner1.resize(o1->num_tris);
    allRes.triCp1.resize(o1->num_tris);
    allRes.triDist2.resize(o2->num_tris);
    allRes.triPartner2.resize(o2->num_tris);
    allRes.triCp2.resize(o2->num_tris);
    fill(allRes.triDist1.begin(),allRes.triDist1.end(),tolerance);
    fill(allRes.triDist2.begin(),allRes.triDist2.end(),tolerance);
    fill(allRes.triPartner1.begin(),allRes.triPartner1.end(),-1);
    fill(allRes.triPartner2.begin(),allRes.triPartner2.end(),-1);
    */

  // Compute the transform [R,T] that takes us from cs2 to cs1.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]

  MTxM(res->R,R1,R2);
  PQP_REAL Ttemp[3];
  VmV(Ttemp, T2, T1);
  MTxV(res->T, R1, Ttemp);

  // set tolerance, used to prune the search

  if (tolerance < 0.0) tolerance = 0.0;
  res->tolerance = tolerance;
  res->distance = tolerance;
  
  // clear the stats

  res->num_bv_tests = 0;
  res->num_tri_tests = 0;

  // initially assume not closer than tolerance

  res->closer_than_tolerance = 0;
  
  // compute the transform from o1->child(0) to o2->child(0)

  PQP_REAL Rtemp[3][3], R[3][3], T[3];

  MxM(Rtemp,res->R,o2->child(0)->R);
  MTxM(R,o1->child(0)->R,Rtemp);
#if PQP_BV_TYPE & RSS_TYPE
  MxVpV(Ttemp,res->R,o2->child(0)->Tr,res->T);
  VmV(Ttemp,Ttemp,o1->child(0)->Tr);
#else
  MxVpV(Ttemp,res->R,o2->child(0)->To,res->T);
  VmV(Ttemp,Ttemp,o1->child(0)->To);
#endif
  MTxV(T,o1->child(0)->R,Ttemp);

  // find a distance lower bound for trivial reject

  res->num_bv_tests += 2;
  PQP_REAL d = BV_CheckTolerance(R, T, o1->child(0), o2->child(0),res->tolerance);
  
  if (d <= res->tolerance)
  {
    ToleranceRecurse2(res, R, T, o1, 0, o2, 0, allRes);

    for(map<int,pair<Vector3,Vector3> >::iterator i=allRes.triCp1.begin();i!=allRes.triCp1.end();i++) {
      // res->p2 is in cs 1 ; transform it to cs 2 
      PQP_REAL u[3];
      VmV(u, i->second.second, res->T);
      MTxV(i->second.second, res->R, u);
    }
    for(map<int,pair<Vector3,Vector3> >::iterator i=allRes.triCp2.begin();i!=allRes.triCp2.end();i++) {
      // res->p2 is in cs 1 ; transform it to cs 2 
      PQP_REAL u[3];
      VmV(u, i->second.second, res->T);
      MTxV(i->second.second, res->R, u);
    }
    /*
    for(size_t i=0;i<allRes.triCp1.size();i++) {
      if(allRes.triPartner1[i] >= 0) {
	// res->p2 is in cs 1 ; transform it to cs 2 
	PQP_REAL u[3];
	VmV(u, allRes.triCp1[i].second, res->T);
	MTxV(allRes.triCp1[i].second, res->R, u);
      }
    }
    for(size_t i=0;i<allRes.triCp2.size();i++) {
      if(allRes.triPartner2[i] >= 0) {
	// res->p2 is in cs 1 ; transform it to cs 2 
	PQP_REAL u[3];
	VmV(u, allRes.triCp2[i].second, res->T);
	MTxV(allRes.triCp2[i].second, res->R, u);
      }
    }
    */
  }

  // res->p2 is in cs 1 ; transform it to cs 2

  PQP_REAL u[3];
  VmV(u, res->p2, res->T);
  MTxV(res->p2, res->R, u);

  //double time2 = GetTime();
  //res->query_time_secs = time2 - time1;
  res->query_time_secs = 0;

  //printf("Res num bv tests %d, num tri tests %d\n",res->num_bv_tests,res->num_tri_tests);

  return PQP_OK;
}

#else // HAVE_PQP

#endif // HAVE_PQP


} //namespace Geometry
