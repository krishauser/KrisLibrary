#include "MultiRobot2DCSpace.h"
#include "EdgePlanner.h"
#include <GLdraw/GL.h>
#include <math/angle.h>
#include <math/random.h>
#include <math/sample.h>
using namespace std;

MultiRobot2DCSpace::MultiRobot2DCSpace()
{
  allowRotation = true;
  angleDistanceWeight = 0.3;
  visibilityEpsilon = 0.01;
  domain.bmin.set(0,0);
  domain.bmax.set(1,1);
}

void MultiRobot2DCSpace::DrawWorkspaceGL() const
{
  //blank out background (light yellow)
  glColor3f(1.f,1.f,0.5f);
  glBegin(GL_QUADS);
  glVertex2d(domain.bmin.x,domain.bmin.y);
  glVertex2d(domain.bmax.x,domain.bmin.y);
  glVertex2d(domain.bmax.x,domain.bmax.y);
  glVertex2d(domain.bmin.x,domain.bmax.y);
  glEnd();

  //draw obstacles (dark grey)
  glColor3f(0.2f,0.2f,0.2f);
  obstacles.DrawGL();
}

void MultiRobot2DCSpace::DrawRobotGL(int index,const RigidTransform2D& T) const
{
  Assert(index >= 0 && index < (int) robots.size());
  Geometric2DCollection temp = robots[index];
  temp.Transform(T);

  //draw robot interpolating from grey to blue
  Real u = Real(index) / Real(robots.size()-1);
  glColor3d(u*0.5,u*0.5,1.0-u*0.5);
  temp.DrawGL();
}

void MultiRobot2DCSpace::DrawRobotGL(int index,const Config& q) const
{
  DrawRobotGL(index,GetRobotTransform(index,q));
}

RigidTransform2D MultiRobot2DCSpace::GetRobotTransform(int index,const Config& x) const
{
  int base = (allowRotation ? 3 : 2)*index;
  RigidTransform2D T;
  T.t.set(x(base),x(base+1));
  if(allowRotation)
    T.R.setRotate(x(base+2));
  else
    T.R.setIdentity();
  return T;
}


void MultiRobot2DCSpace::DrawGL(const Config& x) const
{
  DrawWorkspaceGL();

  for(size_t i=0;i<robots.size();i++) 
    DrawRobotGL(i,x);
}


/*
void MultiRobot2DCSpace::GetSingleRobotCSpace(const Config& q,int index,RigidRobot2DCSpace& space) const
{
  Assert(allowRotation);
  Assert(index >= 0 && index < (int)robots.size());

  space.Clear();
  space.domain = domain;
  space.angleDistanceWeight = angleDistanceWeight;
  space.visibilityEpsilon = visibilityEpsilon;
  space.robot = robots[index];
  space.workspace = workspace;
  //add fixed robots
  for(size_t i=0;i<robots.size();i++,k+=stride) {
    if((int)i == index) continue;
    RigidTransform2D T=GetRobotTransform(i,q);
    Geometric2DCollection temp = robots[i];
    temp.Transform(T);
    space.workspace.Add(temp);
  }
}

void MultiRobot2DCSpace::GetSingleRobotCSpace(const Config& q,int index,TranslatingRobot2DCSpace& space) const
{
  Assert(index >= 0 && index < (int)robots.size());
  Assert(!allowRotation);
  //TODO: if allowRotation is set, then do we want to rotate the reference robot?

  space.Clear();
  space.domain = domain;
  space.angleDistanceWeight = angleDistanceWeight;
  space.visibilityEpsilon = visibilityEpsilon;
  space.robot = robots[index];
  space.workspace = workspace;
  //add fixed robots
  for(size_t i=0;i<robots.size();i++,k+=stride) {
    if((int)i == index) continue;
    RigidTransform2D T=GetRobotTransform(i,q);
    Geometric2DCollection temp = robots[i];
    temp.Transform(T);
    space.workspace.Add(temp);
  }
}

void MultiRobot2DCSpace::GetSubsetRobotCSpace(const Config& q,const vector<int>& robots,MultiRobot2DCSpace& space) const;
*/


void MultiRobot2DCSpace::Sample(Config& x)
{
  int stride = (allowRotation ? 3 : 2);
  x.resize(stride*robots.size());
  int k=0;
  for(size_t i=0;i<robots.size();i++,k+=stride) {
    x(k) = Rand(domain.bmin.x,domain.bmax.x);
    x(k+1) = Rand(domain.bmin.y,domain.bmax.y);
    if(allowRotation)
      x(k+2) = Rand()*TwoPi;
  }
}

void MultiRobot2DCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  int stride = (allowRotation ? 3 : 2);
  Assert(c.n==stride*(int)robots.size());
  x = c;
  int k=0;
  for(size_t i=0;i<robots.size();i++,k+=stride) {
    Real dx,dy;
    SampleDisk(r,dx,dy);
    x(k) += dx;
    x(k+1) += dy;
    if(allowRotation) {
      x(k+2) += Rand(-r,r)/angleDistanceWeight;
      x(k+2)=AngleNormalize(x(k+2));
    }
  }
}

bool MultiRobot2DCSpace::IsFeasible(const Config& x)
{
  int stride = (allowRotation ? 3 : 2);
  Assert(x.n==stride*(int)robots.size());
  int k=0;
  for(size_t i=0;i<robots.size();i++,k+=stride) {
    if(!domain.contains(Vector2(x(k),x(k+1)))) return false;
    RigidTransform2D T=GetRobotTransform(i,x);
    Geometric2DCollection temp = robots[i];
    temp.Transform(T);
    if(obstacles.Collides(temp)) return false;

    //check self collisions
    int m=0;
    for(size_t j=0;j<i;j++,m+=stride) {
      T=GetRobotTransform(j,x);
      Geometric2DCollection temp2 = robots[j];
      temp2.Transform(T);
      if(temp.Collides(temp2)) return false;
    }
  }
  return true;
}

EdgePlannerPtr MultiRobot2DCSpace::PathChecker(const Config& a,const Config& b)
{
  return make_shared<EpsilonEdgeChecker>(this,a,b,visibilityEpsilon);
}

Real MultiRobot2DCSpace::Distance(const Config& x, const Config& y)
{
  int stride = (allowRotation ? 3 : 2);
  Assert(x.n==stride*(int)robots.size());
  Assert(x.n==y.n);
  int k=0;
  Real d2=0;
  for(size_t i=0;i<robots.size();i++,k+=stride) {
    d2 += Vector2(x(k+0),x(k+1)).distanceSquared(Vector2(y(k+0),y(k+1)));
    d2 += Sqr(angleDistanceWeight)*Sqr(AngleDiff(x(k+2),y(k+2)));
  }
  return Sqrt(d2);
}


void MultiRobot2DCSpace::Properties(PropertyMap& map) const
{
  int stride = (allowRotation ? 3 : 2);
  if(allowRotation)
    map.set("cartesian",0);
  else
    map.set("cartesian",0);
  map.set("geodesic",1);
  map.set("metric","weighted euclidean");
  vector<Real> w(stride*robots.size(),1.0);
  if(allowRotation) {
    for(size_t i=0;i<robots.size();i++)
      w[3*i+2] = angleDistanceWeight;
  }
  map.setArray("metricWeights",w);
  Real v = (domain.bmax.x-domain.bmin.x)*(domain.bmax.y-domain.bmin.y);
  if(allowRotation) v*=angleDistanceWeight*TwoPi;
  v = Pow(v,robots.size());
  map.set("volume",v);
  map.set("diameter",Sqrt(robots.size()*domain.bmin.distanceSquared(domain.bmax) + (allowRotation ? robots.size()*Sqr(angleDistanceWeight*TwoPi) : 0))); 
 vector<Real> bmin(stride*robots.size()),bmax(stride*robots.size());
  for(size_t i=0;i<robots.size();i++) {
    domain.bmin.get(bmin[stride*i+0],bmin[stride*i+1]);
    domain.bmax.get(bmax[stride*i+0],bmax[stride*i+1]);
    if(allowRotation) {
      bmin[stride*i+2]=0;
      bmax[stride*i+2]=TwoPi;
    }
  }
  map.setArray("minimum",bmin);
  map.setArray("maximum",bmax);
}
