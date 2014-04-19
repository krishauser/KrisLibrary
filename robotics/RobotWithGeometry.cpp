#include "RobotWithGeometry.h"
#include <meshing/IO.h>
#include <errors.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/GeometryAppearance.h>
#include <fstream>
#include <algorithm>
using namespace GLDraw;
using namespace std;

RobotWithGeometry::RobotWithGeometry()
{}

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
    for(size_t j=0;j<robots[i]->envCollisions.size();j++)
      envCollisions[j+offset[i]] = robots[i]->envCollisions[j];
    //delete self collisions that are not allowed
    for(int j=0;j<robots[i]->selfCollisions.m;j++)
      for(int k=0;k<robots[i]->selfCollisions.n;k++)
	if(!robots[i]->selfCollisions(j,k))
	  selfCollisions(j+offset[i],k+offset[i]) = NULL;
  }
}

bool RobotWithGeometry::LoadGeometry(int i,const char* file)
{
  if(!geometry[i].Load(file)) return false;
  return true;
}

bool RobotWithGeometry::SaveGeometry(int i,const char* file)
{
  if(!geometry[i].Save(file)) return false;
  return true;
}

void RobotWithGeometry::InitCollisions()
{
  for(size_t i=0;i<geometry.size();i++) geometry[i].InitCollisions();
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
  if(!geometry[i].Empty() && !geometry[j].Empty()) 
    selfCollisions(i,j) = new CollisionQuery(geometry[i],geometry[j]);
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
  geometry[i].SetTransform(links[i].T_World);
}

void RobotWithGeometry::InitMeshCollision(CollisionGeometry& mesh)
{
  for(size_t i=0;i<links.size();i++) {
    if(geometry[i].Empty()) continue;
    if(envCollisions[i] == NULL) {
      envCollisions[i] = new CollisionQuery(geometry[i],mesh);
    }
    else {
      if(envCollisions[i]->b != &mesh) {
	delete envCollisions[i];
	envCollisions[i] = new CollisionQuery(geometry[i],mesh);
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
  for(size_t i=0;i<bodies.size();i++) {
    for(size_t j=i+1;j<bodies.size();j++) {
      if(SelfCollision(bodies[i],bodies[j],distance)) return true;
    }
  }
  return false;

}

bool RobotWithGeometry::SelfCollision(const vector<int>& set1,const vector<int>& set2,Real distance)
{
  for(size_t i=0;i<set1.size();i++) {
    for(size_t j=0;j<set2.size();j++) {
      if(SelfCollision(set1[i],set2[j],distance)) return true;
    }
  }
  return false;
}

bool RobotWithGeometry::SelfCollision(Real distance)
{
  for(size_t i=0;i<links.size();i++) {
    for(size_t j=i+1;j<links.size();j++) {
      if(SelfCollision(i,j,distance)) return true;
    }
  }
  return false;
}

bool RobotWithGeometry::MeshCollision(CollisionGeometry& mesh)
{
  if(!envCollisions[0] || envCollisions[0]->b != &mesh) {
    cerr<<"Warning, MeshCollision() called with a different mesh"<<endl;
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
  draw(geometry[i]);
}



