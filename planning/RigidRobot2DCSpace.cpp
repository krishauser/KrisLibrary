#include "RigidRobot2DCSpace.h"
#include "EdgePlanner.h"
#include <GLdraw/GL.h>
#include <math/angle.h>
#include <math/random.h>
#include <math/sample.h>

RigidRobot2DCSpace::RigidRobot2DCSpace()
{
  angleDistanceWeight = 0.3;
  visibilityEpsilon = 0.01;
  domain.bmin.set(0,0);
  domain.bmax.set(1,1);
}

void RigidRobot2DCSpace::DrawWorkspaceGL() const
{
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
}

void RigidRobot2DCSpace::DrawRobotGL(const Config& x) const
{
  RigidTransform2D T;
  T.t.set(x(0),x(1));
  T.R.setRotate(x(2));
  Geometric2DCollection temp = robot;
  temp.Transform(T);

  //draw robot greyish blue
  temp.DrawGL();
}

void RigidRobot2DCSpace::DrawGL(const Config& x) const
{
  DrawWorkspaceGL();

  glColor3f(0.3,0.3,0.7);
  DrawRobotGL(x);
}

void RigidRobot2DCSpace::Sample(Config& x)
{
  x.resize(3);
  x(0) = Rand(domain.bmin.x,domain.bmax.x);
  x(1) = Rand(domain.bmin.y,domain.bmax.y);
  x(2) = Rand()*TwoPi;
}

void RigidRobot2DCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  Assert(c.n==3);
  Real dx,dy;
  SampleDisk(r,dx,dy);
  x = c;
  x(0) += dx;
  x(1) += dy;
  x(2) += Rand(-r,r)/angleDistanceWeight;
  x(2)=AngleNormalize(x(2));
}

int RigidRobot2DCSpace::NumObstacles()
{
  return 1+obstacles.NumObstacles();
}

std::string RigidRobot2DCSpace::ObstacleName(int obstacle)
{
  if(obstacle==0) return "domain";
  else {
    obstacle -= 1;
    char buf[64];
    sprintf(buf,"%s[%d]",obstacles.ObstacleTypeName(obstacle),obstacles.ObstacleIndex(obstacle));
    return buf;
  }
}

bool RigidRobot2DCSpace::IsFeasible(const Config& x,int obstacle)
{
  if(obstacle == 0) 
    return domain.contains(Vector2(x(0),x(1)));
  obstacle -= 1;
  RigidTransform2D T;
  T.t.set(x(0),x(1));
  T.R.setRotate(x(2));
  Geometric2DCollection temp = robot;
  temp.Transform(T);
  if(obstacles.Collides(temp,obstacle)) return false;
  return true;
}

bool RigidRobot2DCSpace::IsFeasible(const Config& x)
{
  if(!domain.contains(Vector2(x(0),x(1)))) return false;
  RigidTransform2D T;
  T.t.set(x(0),x(1));
  T.R.setRotate(x(2));
  Geometric2DCollection temp = robot;
  temp.Transform(T);
  if(obstacles.Collides(temp)) return false;
  return true;
}

EdgePlanner* RigidRobot2DCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new BisectionEpsilonExplicitEdgePlanner(this,a,b,visibilityEpsilon);
}

EdgePlanner* RigidRobot2DCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  CSpace* space = new SubsetExplicitCSpace(this,obstacle);
  return new EdgePlannerWithCSpaceContainer(space,new BisectionEpsilonEdgePlanner(space,a,b,visibilityEpsilon));
}

Real RigidRobot2DCSpace::Distance(const Config& x, const Config& y)
{
  Real d2 = Vector2(x(0),x(1)).distanceSquared(Vector2(y(0),y(1)));
  return Sqrt(d2 + Sqr(angleDistanceWeight)*Sqr(AngleDiff(AngleNormalize(x(2)),AngleNormalize(y(2)))));
}

void RigidRobot2DCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  CSpace::Interpolate(x,y,u,out);
  out(2) = AngleInterp(AngleNormalize(x(2)),AngleNormalize(y(2)),u);
}

void RigidRobot2DCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  CSpace::Interpolate(x,y,0.5,out);
  out(2) = AngleInterp(AngleNormalize(x(2)),AngleNormalize(y(2)),0.5);
}

void RigidRobot2DCSpace::Properties(PropertyMap& map) const
{
  map.set("cartesian",0);
  map.set("geodesic",1);
  map.set("metric","weighted euclidean");
  vector<Real> w(3,1.0);
  w[2] = angleDistanceWeight;
  map.setArray("metricWeights",w);
  Real v = (domain.bmax.x-domain.bmin.x)*(domain.bmax.y-domain.bmin.y)*angleDistanceWeight*TwoPi;
  map.set("volume",v);
  map.set("diameter",Sqrt(domain.bmin.distanceSquared(domain.bmax)+Sqr(angleDistanceWeight*TwoPi)));
  vector<Real> bmin(3),bmax(3);
  domain.bmin.get(bmin[0],bmin[1]);
  domain.bmax.get(bmax[0],bmax[1]);
  bmin[2]=0;
  bmax[2]=TwoPi;
  map.setArray("minimum",bmin);
  map.setArray("maximum",bmax);
}
