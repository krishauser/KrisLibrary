#include <KrisLibrary/Logger.h>
#include "RobotWithGeometry.h"
#include <meshing/IO.h>
#include <errors.h>
#include <Timer.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
//#include <GLdraw/drawMesh.h>
#include <GLdraw/GeometryAppearance.h>
#include <fstream>
#include <algorithm>
using namespace GLDraw;
using namespace std;

RobotWithGeometry::RobotWithGeometry()
{}

RobotWithGeometry::RobotWithGeometry(const RobotDynamics3D& rhs)
{
  operator = (rhs);
}

RobotWithGeometry::RobotWithGeometry(const RobotWithGeometry& rhs)
{
  operator = (rhs);
}

RobotWithGeometry::~RobotWithGeometry()
{
  CleanupCollisions();
  CleanupSelfCollisions();
}

void RobotWithGeometry::Initialize(int n)
{
  CleanupCollisions();
  CleanupSelfCollisions();

  RobotDynamics3D::Initialize(n);
  Assert((int)links.size() == n);
  geometry.resize(n);
  selfCollisions.resize(n,n,NULL);
  envCollisions.resize(n,NULL);
}

void RobotWithGeometry::Merge(const std::vector<RobotWithGeometry*>& robots)
{
  vector<RobotDynamics3D*> drobots(robots.size());
  copy(robots.begin(),robots.end(),drobots.begin());
  RobotDynamics3D::Merge(drobots);

  CleanupCollisions();
  CleanupSelfCollisions();
  int n = (int)links.size();
  geometry.resize(n);
  selfCollisions.resize(n,n,NULL);
  envCollisions.resize(n,NULL);
  
  size_t nl = 0;
  vector<size_t> offset(robots.size());
  for(size_t i=0;i<robots.size();i++) {
    offset[i] = nl;
    nl += robots[i]->links.size();
  }
  InitAllSelfCollisions();
  for(size_t i=0;i<robots.size();i++) {
    for(size_t j=0;j<robots[i]->geometry.size();j++)
      geometry[j+offset[i]] = robots[i]->geometry[j];
    for(size_t j=0;j<robots[i]->envCollisions.size();j++) {
      if(robots[i]->envCollisions[j])
	envCollisions[j+offset[i]] = new CollisionQuery(*geometry[j+offset[i]],*robots[i]->envCollisions[j]->b);
    }
    //delete self collisions that are not allowed
    for(int j=0;j<robots[i]->selfCollisions.m;j++)
      for(int k=0;k<robots[i]->selfCollisions.n;k++)
	if(!robots[i]->selfCollisions(j,k))
	  selfCollisions(j+offset[i],k+offset[i]) = NULL;
  }
}

const RobotWithGeometry& RobotWithGeometry::operator = (const RobotWithGeometry& rhs)
{
  RobotDynamics3D::operator = (rhs);
  CleanupCollisions();
  CleanupSelfCollisions();
  int n = (int)links.size();
  geometry.resize(n);
  selfCollisions.resize(n,n,NULL);
  envCollisions.resize(n,NULL);
  geometry = rhs.geometry;
  for(int j=0;j<n;j++) {
    if(rhs.envCollisions[j])
      envCollisions[j] = new CollisionQuery(*geometry[j],*rhs.envCollisions[j]->b);
  }
  for(int j=0;j<selfCollisions.m;j++)
    for(int k=0;k<selfCollisions.n;k++)
      if(rhs.selfCollisions(j,k))
	InitSelfCollisionPair(j,k);
  return *this;
}


const RobotWithGeometry& RobotWithGeometry::operator = (const RobotDynamics3D& rhs)
{
  RobotDynamics3D::operator = (rhs);
  CleanupCollisions();
  CleanupSelfCollisions();
  int n = (int)links.size();
  geometry.resize(n);
  selfCollisions.resize(n,n,NULL);
  envCollisions.resize(n,NULL);
  return *this;
}


bool RobotWithGeometry::LoadGeometry(int i,const char* file)
{
  geometry[i].reset(new CollisionGeometry);
  if(!geometry[i]->Load(file)) return false;
  return true;
}

bool RobotWithGeometry::SaveGeometry(int i,const char* file)
{
  if(!geometry[i]) return false;
  if(!geometry[i]->Save(file)) return false;
  return true;
}

bool RobotWithGeometry::IsGeometryEmpty(int i)
{
  return !geometry[i] || geometry[i]->Empty();
}

void RobotWithGeometry::InitCollisions()
{
  Timer timer;
  for(size_t i=0;i<geometry.size();i++) {
    if(!IsGeometryEmpty(i)) 
      geometry[i]->InitCollisionData();
  }
  double t = timer.ElapsedTime();
  if(t > 0.2) 
    LOG4CXX_INFO(KrisLibrary::logger(),"Initialized robot collision data structures in time "<<t);
}

void RobotWithGeometry::CleanupCollisions()
{
  for(size_t i=0;i<envCollisions.size();i++) SafeDelete(envCollisions[i]);
}

void RobotWithGeometry::InitAllSelfCollisions()
{
  for(size_t i=0;i<links.size();i++) {
    for(size_t j=i+1;j<links.size();j++) {
      if(parents[i] != (int)j && parents[j] != (int)i)
	InitSelfCollisionPair(i,j);
    }
  }
}

void RobotWithGeometry::InitSelfCollisionPair(int i,int j)
{
  Assert(i<j);
  Assert(!selfCollisions(i,j));
  Assert(j < (int)geometry.size());
  if(!IsGeometryEmpty(i) && !IsGeometryEmpty(j)) 
    selfCollisions(i,j) = new CollisionQuery(*geometry[i],*geometry[j]);
}


void RobotWithGeometry::InitSelfCollisionPairs(const Array2D<bool>& collision)
{
  assert(collision.m == (int)geometry.size());
  assert(collision.n == (int)geometry.size());
  CleanupSelfCollisions();
  for(int i=0;i<collision.m;i++)
    for(int j=i+1;j<collision.n;j++)
      if(collision(i,j))
	InitSelfCollisionPair(i,j);
}

void RobotWithGeometry::GetSelfCollisionPairs(Array2D<bool>& collision) const
{
  collision.resize(geometry.size(),geometry.size(),false);
  for(int i=0;i<collision.m;i++)
    for(int j=0;j<collision.n;j++)
      if(selfCollisions(i,j) != NULL)
	collision(i,j) = true;

}

void RobotWithGeometry::CleanupSelfCollisions()
{
  for(int i=0;i<selfCollisions.m;i++) 
    for(int j=0;j<selfCollisions.n;j++)
      SafeDelete(selfCollisions(i,j));
}

void RobotWithGeometry::UpdateGeometry()
{
  for(size_t i=0;i<links.size();i++) 
    UpdateGeometry(i);
}

void RobotWithGeometry::UpdateGeometry(int i)
{
  if(geometry[i]) geometry[i]->SetTransform(links[i].T_World);
}

void RobotWithGeometry::InitMeshCollision(CollisionGeometry& mesh)
{
  for(size_t i=0;i<links.size();i++) {
    if(IsGeometryEmpty(i)) continue;
    if(envCollisions[i] == NULL) {
      envCollisions[i] = new CollisionQuery(*geometry[i],mesh);
    }
    else {
      if(envCollisions[i]->b != &mesh) {
	delete envCollisions[i];
	envCollisions[i] = new CollisionQuery(*geometry[i],mesh);
      }
    }
  }
}

bool UnderCollisionMargin(Geometry::AnyCollisionQuery* q,Real adj)
{
  if(IsInf(adj) < 0) return false;
  else if(adj == 0)
    return (q->Collide());
  else if(adj > 0) //require a separation distance
    return (q->WithinDistance(adj));
  else { //allow a bit of penetration
    if(!q->Collide()) return false;
    return (q->PenetrationDepth() >= -adj);
  }
}

bool RobotWithGeometry::SelfCollision(int i, int j, Real d)
{
  if(i > j) std::swap(i,j);
  CollisionQuery* query=selfCollisions(i,j);
  if(query == NULL) return false;
  return UnderCollisionMargin(query,d);
}

bool RobotWithGeometry::SelfCollision(const vector<int>& bodies,Real distance)
{
  //get world space bounding boxes for quick reject test
  vector<int> validbodies;
  validbodies.reserve(bodies.size());
  for(size_t i=0;i<bodies.size();i++) 
    if(!IsGeometryEmpty(i)) validbodies.push_back(bodies[i]);
  vector<AABB3D> bbs(validbodies.size());
  for(size_t i=0;i<validbodies.size();i++) 
    bbs[i] = geometry[validbodies[i]]->GetAABB();
  if(distance != 0) {
    //adjust bounding  boxes
    Vector3 d(distance*0.5);
    for(size_t i=0;i<bbs.size();i++) {
      bbs[i].bmin -= d;
      bbs[i].bmax += d;
    }
  }
  //if |validbodies| > 300, you may want to use the functions in geometry/RangeQuery.h
  //now check collisions, ensuring BBs overlap
  for(size_t i=0;i<validbodies.size();i++) {
    for(size_t j=i+1;j<validbodies.size();j++) {
      CollisionQuery* query=selfCollisions(validbodies[i],validbodies[j]);
      if(query == NULL) continue;
      if(!bbs[i].intersects(bbs[j])) continue;
      if(UnderCollisionMargin(query,distance)) return true;
    }
  }
  return false;
}

bool RobotWithGeometry::SelfCollision(const vector<int>& set1,const vector<int>& set2,Real distance)
{
  //get world space bounding boxes for quick reject test
  vector<int> valid1,valid2;
  valid1.reserve(set1.size());
  valid2.reserve(set2.size());
  for(size_t i=0;i<set1.size();i++) 
    if(!IsGeometryEmpty(set1[i])) valid1.push_back(set1[i]);
  for(size_t i=0;i<set2.size();i++) 
    if(!IsGeometryEmpty(set2[i])) valid2.push_back(set2[i]);
  if(valid1.empty() || valid2.empty()) return false;
  vector<AABB3D> bbs1(valid1.size()),bbs2(valid2.size());
  for(size_t i=0;i<valid1.size();i++) 
    bbs1[i] = geometry[valid1[i]]->GetAABB();
  for(size_t i=0;i<valid2.size();i++) 
    bbs2[i] = geometry[valid2[i]]->GetAABB();
  if(distance != 0) {
    //adjust bounding  boxes
    Vector3 d(distance*0.5);
    for(size_t i=0;i<bbs1.size();i++) {
      bbs1[i].bmin -= d;
      bbs1[i].bmax += d;
    }
    for(size_t i=0;i<bbs2.size();i++) {
      bbs2[i].bmin -= d;
      bbs2[i].bmax += d;
    }
  }
  if(valid1.size() >= 3 && valid2.size() >= 3) {
    //O(m+n) additional fast reject tests: set 1 vs bb of set 2, and vice versa
    AABB3D bb1,bb2;
    bb1 = bbs1[0];
    for(size_t i=1;i<bbs1.size();i++)
      bb1.setUnion(bbs1[i]);
    for(size_t i=0;i<valid2.size();i++)
      if(!bb1.intersects(bbs2[i])) { //quick reject
	bbs2[i] = bbs2.back();
	valid2[i] = valid2.back();
	bbs2.resize(bbs2.size()-1);
	valid2.resize(valid2.size()-1);
	i--;
      }
    bb2 = bbs1[0];
    for(size_t i=1;i<bbs2.size();i++)
      bb2.setUnion(bbs2[i]);
    for(size_t i=0;i<valid1.size();i++)
      if(!bb2.intersects(bbs1[i])) { //quick reject
	bbs1[i] = bbs1.back();
	valid1[i] = valid1.back();
	bbs1.resize(bbs1.size()-1);
	valid1.resize(valid1.size()-1);
	i--;
      }
  }
  //quick reject of all 
  //if |valid1| or |valid2| > 300, you may want to use the functions in geometry/RangeQuery.h
  //now check collisions, ensuring BBs overlap
  for(size_t i=0;i<valid1.size();i++) {
    for(size_t j=0;j<valid2.size();j++) {
      CollisionQuery* query=selfCollisions(valid2[i],valid2[j]);
      if(query == NULL) continue;
      if(!bbs1[i].intersects(bbs2[j])) continue;
      if(UnderCollisionMargin(query,distance)) return true;
    }
  }
  return false;
}

bool RobotWithGeometry::SelfCollision(Real distance)
{
  //get world space bounding boxes for quick reject test
  vector<int> validbodies;
  validbodies.reserve(links.size());
  for(size_t i=0;i<links.size();i++) 
    if(!IsGeometryEmpty(i)) validbodies.push_back(i);
  vector<AABB3D> bbs(validbodies.size());
  for(size_t i=0;i<validbodies.size();i++) {
    bbs[i] = geometry[validbodies[i]]->GetAABB();
  }
  if(distance != 0) {
    //adjust bounding  boxes
    Vector3 d(distance*0.5);
    for(size_t i=0;i<bbs.size();i++) {
      bbs[i].bmin -= d;
      bbs[i].bmax += d;
    }
  }
  //if |validbodies| > 300, you may want to use the functions in geometry/RangeQuery.h
  //now check collisions, ensuring BBs overlap
  for(size_t i=0;i<validbodies.size();i++) {
    for(size_t j=i+1;j<validbodies.size();j++) {
      CollisionQuery* query=selfCollisions(validbodies[i],validbodies[j]);
      if(query == NULL) continue;
      if(!bbs[i].intersects(bbs[j])) continue;
      if(UnderCollisionMargin(query,distance)) return true;
    }
  }
  return false;
}

void RobotWithGeometry::SelfCollisions(vector<pair<int,int> >& pairs,Real distance)
{
  //get world space bounding boxes for quick reject test
  vector<int> validbodies;
  validbodies.reserve(links.size());
  for(size_t i=0;i<links.size();i++) 
    if(!IsGeometryEmpty(i)) validbodies.push_back(i);
  vector<AABB3D> bbs(validbodies.size());
  for(size_t i=0;i<validbodies.size();i++) 
    bbs[i] = geometry[validbodies[i]]->GetAABB();
  if(distance != 0) {
    //adjust bounding  boxes
    Vector3 d(distance*0.5);
    for(size_t i=0;i<bbs.size();i++) {
      bbs[i].bmin -= d;
      bbs[i].bmax += d;
    }
  }
  //if |validbodies| > 300, you may want to use the functions in geometry/RangeQuery.h
  //now check collisions, ensuring BBs overlap
  for(size_t i=0;i<validbodies.size();i++) {
    for(size_t j=i+1;j<validbodies.size();j++) {
      CollisionQuery* query=selfCollisions(validbodies[i],validbodies[j]);
      if(query == NULL) continue;
      if(!bbs[i].intersects(bbs[j])) continue;
      if(UnderCollisionMargin(query,distance)) pairs.push_back(pair<int,int>(validbodies[i],validbodies[j]));
    }
  }

}


bool RobotWithGeometry::MeshCollision(CollisionGeometry& mesh)
{
  if(!envCollisions[0] || envCollisions[0]->b != &mesh) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, MeshCollision() called with a different mesh\n");
    InitMeshCollision(mesh);
  }
  for(size_t i=0;i<links.size();i++)
    if(MeshCollision(i)) return true;
  return false;
}

bool RobotWithGeometry::MeshCollision(int i,Real distance)
{
  if(envCollisions[i]==NULL) return false;
  return (envCollisions[i] && UnderCollisionMargin(envCollisions[i],distance));
}

void RobotWithGeometry::DrawGL()
{
  for(size_t i=0;i<links.size();i++) {
    Matrix4 mat = links[i].T_World;
    glPushMatrix();
    glMultMatrix(mat);
    DrawLinkGL(i);
    glPopMatrix();
  }
}

void RobotWithGeometry::DrawLinkGL(int i)
{
  if(geometry[i]) draw(*geometry[i]);
}



