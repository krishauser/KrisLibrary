#include "TranslatingRobot2DCSpace.h"
#include "EdgePlanner.h"
#include <GLdraw/GL.h>
#include <math/random.h>
#include <math/sample.h>

TranslatingRobot2DCSpace::TranslatingRobot2DCSpace()
{
  visibilityEpsilon = 0.01;
  domain.bmin.set(0,0);
  domain.bmax.set(1,1);
}

void TranslatingRobot2DCSpace::DrawWorkspaceGL() const
{
#ifndef NO_OPENGL
  //blank out background (light yellow)
  glColor3f(1,1,0.5);
  glBegin(GL_QUADS);
  glVertex2f(domain.bmin.x,domain.bmin.y);
  glVertex2f(domain.bmax.x,domain.bmin.y);
  glVertex2f(domain.bmax.x,domain.bmax.y);
  glVertex2f(domain.bmin.x,domain.bmax.y);
  glEnd();

  //draw obstacles (dark grey)
  glColor3f(0.2,0.2,0.2);
  obstacles.DrawGL();
#endif //NO_OPENGL
}

void TranslatingRobot2DCSpace::DrawRobotGL(const Config& x) const
{
  RigidTransform2D T;
  T.t.set(x(0),x(1));
  T.R.setRotate(x(2));
  Geometric2DCollection temp = robot;
  temp.Transform(T);

  //draw robot greyish blue
  temp.DrawGL();
}

void TranslatingRobot2DCSpace::DrawGL(const Config& x) const
{
#ifndef NO_OPENGL
  DrawWorkspaceGL();

  //draw robot greyish blue
  glColor3f(0.3,0.3,0.7);
  DrawRobotGL(x);
#endif //NO_OPENGL
}

void TranslatingRobot2DCSpace::Sample(Config& x)
{
  x.resize(2);
  x(0) = Rand(domain.bmin.x,domain.bmax.x);
  x(1) = Rand(domain.bmin.y,domain.bmax.y);
}

void TranslatingRobot2DCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  Assert(c.n==2);
  Real dx,dy;
  SampleDisk(r,dx,dy);
  x = c;
  x(0) += dx;
  x(1) += dy;
}

int TranslatingRobot2DCSpace::NumObstacles()
{
  return 1+obstacles.NumObstacles();
}

std::string TranslatingRobot2DCSpace::ObstacleName(int obstacle)
{
  if(obstacle==0) return "domain";
  else {
    obstacle -= 1;
    char buf[64];
    sprintf(buf,"%s[%d]",obstacles.ObstacleTypeName(obstacle),obstacles.ObstacleIndex(obstacle));
    return buf;
  }
}

bool TranslatingRobot2DCSpace::IsFeasible(const Config& x,int obstacle)
{
  if(obstacle==0)
    return domain.contains(Vector2(x(0),x(1)));
  obstacle -= 1;
  RigidTransform2D T;
  T.t.set(x(0),x(1));
  T.R.setIdentity();
  Geometric2DCollection temp = robot;
  temp.Transform(T);
  if(obstacles.Collides(temp,obstacle)) return false;
  return true;
}

bool TranslatingRobot2DCSpace::IsFeasible(const Config& x)
{
  if(!domain.contains(Vector2(x(0),x(1)))) return false;
  RigidTransform2D T;
  T.t.set(x(0),x(1));
  T.R.setIdentity();
  Geometric2DCollection temp = robot;
  temp.Transform(T);
  if(obstacles.Collides(temp)) return false;
  return true;
}

EdgePlanner* TranslatingRobot2DCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new BisectionEpsilonExplicitEdgePlanner(this,a,b,visibilityEpsilon);
}


EdgePlanner* TranslatingRobot2DCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  CSpace* space = new SubsetExplicitCSpace(this,obstacle);
  return new EdgePlannerWithCSpaceContainer(space,new BisectionEpsilonEdgePlanner(space,a,b,visibilityEpsilon));
}

Real TranslatingRobot2DCSpace::Distance(const Config& x, const Config& y)
{
  return Vector2(x(0),x(1)).distance(Vector2(y(0),y(1)));
}


void TranslatingRobot2DCSpace::Properties(PropertyMap& map) const
{
  map.set("cartesian",1);
  map.set("geodesic",1);
  map.set("metric","euclidean");
  Real v = (domain.bmax.x-domain.bmin.x)*(domain.bmax.y-domain.bmin.y);
  map.set("volume",v);
  map.set("diameter",domain.bmin.distance(domain.bmax));
  vector<Real> bmin(2),bmax(2);
  domain.bmin.get(bmin[0],bmin[1]);
  domain.bmax.get(bmax[0],bmax[1]);
  map.setArray("minimum",bmin);
  map.setArray("maximum",bmax);
}
