#include "RobotWithGeometry.h"
#include <errors.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
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

bool RobotWithGeometry::LoadGeometry(int i,const char* file)
{
  if(!LoadMultipleTriMeshes(file,geometry[i])) return false;
  return true;
}

bool RobotWithGeometry::SaveGeometry(int i,const char* file)
{
  ofstream out(file);
  if(!out) return false;
  out << geometry[i] << endl;
  out.close();
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
  if(!geometry[i].tris.empty() && !geometry[j].tris.empty()) 
    selfCollisions(i,j) = new CollisionMeshQuery(geometry[i],geometry[j]);
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
  geometry[i].UpdateTransform(links[i].T_World);
}

void RobotWithGeometry::InitMeshCollision(CollisionMesh& mesh)
{
  for(size_t i=0;i<links.size();i++) {
    if(geometry[i].tris.empty()) continue;
    if(envCollisions[i] == NULL) {
      envCollisions[i] = new CollisionMeshQuery(geometry[i],mesh);
    }
    else {
      if(envCollisions[i]->m2 != &mesh) {
	delete envCollisions[i];
	envCollisions[i] = new CollisionMeshQuery(geometry[i],mesh);
      }
    }
  }
}

bool UnderCollisionMargin(Geometry::CollisionMeshQuery* q,Real adj)
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
  CollisionMeshQuery* query=selfCollisions(i,j);
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

bool RobotWithGeometry::MeshCollision(CollisionMesh& mesh)
{
  if(!envCollisions[0] || envCollisions[0]->m2 != &mesh) {
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
  DrawGLTris(geometry[i]);
}



