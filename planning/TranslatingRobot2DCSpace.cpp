#include <KrisLibrary/Logger.h>
#include "TranslatingRobot2DCSpace.h"
#include "EdgePlanner.h"
#include "EdgePlannerHelpers.h"
#include <GLdraw/GL.h>
#include <math/random.h>
#include <math/sample.h>

TranslatingRobot2DCSpace::TranslatingRobot2DCSpace()
:BoxCSpace(0,1,2)
{
  visibilityEpsilon = 0.01;
  domain.bmin.set(0,0);
  domain.bmax.set(1,1);
}

void TranslatingRobot2DCSpace::InitConstraints()
{
  BoxCSpace::SetDomain(Vector(2,domain.bmin),Vector(2,domain.bmax));
  for(int i=0;i<obstacles.NumObstacles();i++) {
    char buf[64];
    snprintf(buf,64,"%s[%d]",obstacles.ObstacleTypeName(i),obstacles.ObstacleIndex(i));
    AddConstraint(buf,new Geometric2DObstacleFreeSet(obstacles.Obstacle(i),robot,true));
  }
}

void TranslatingRobot2DCSpace::DrawWorkspaceGL() const
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
  DrawWorkspaceGL();

  //draw robot greyish blue
  glColor3f(0.3f,0.3f,0.7f);
  DrawRobotGL(x);
}


EdgePlannerPtr TranslatingRobot2DCSpace::PathChecker(const Config& a,const Config& b)
{
  return make_shared<EpsilonEdgeChecker>(this,a,b,visibilityEpsilon);
}


EdgePlannerPtr TranslatingRobot2DCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  auto space = make_shared<SubsetConstraintCSpace>(this,obstacle);
  return make_shared<EdgePlannerWithCSpaceContainer>(space,make_shared<EpsilonEdgeChecker>(space.get(),a,b,visibilityEpsilon));
}

