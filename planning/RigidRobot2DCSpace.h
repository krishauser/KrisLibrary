#ifndef RIGID_ROBOT_2D_CSPACE_H
#define RIGID_ROBOT_2D_CSPACE_H

#include "Geometric2DCSpace.h"
#include "RigidBodyCSpace.h"

/** @brief a translating and rotating 2D robot in a 2D workspace.
 */
class RigidRobot2DCSpace : public SE2CSpace
{
public:
  RigidRobot2DCSpace();
  RigidRobot2DCSpace(const AABB2D& domain);
  void InitConstraints();
  void DrawWorkspaceGL() const;
  void DrawRobotGL(const Config& q) const;
  void DrawGL(const Config& q) const;

  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle);
  virtual void Properties(PropertyMap&);

  Real visibilityEpsilon;
  Geometric2DCollection obstacles;
  Geometric2DCollection robot;
  AABB2D domain;
};

#endif
