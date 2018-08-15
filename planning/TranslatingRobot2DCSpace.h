#ifndef TRANSLATING_ROBOT_2D_CSPACE_H
#define TRANSLATING_ROBOT_2D_CSPACE_H

#include "Geometric2DCSpace.h"
#include "CSpaceHelpers.h"

/** @brief a translating 2D robot in a 2D workspace.
 */
class TranslatingRobot2DCSpace : public BoxCSpace
{
public:
  TranslatingRobot2DCSpace();
  void InitConstraints();
  void DrawWorkspaceGL() const;
  void DrawRobotGL(const Config& x) const;
  void DrawGL(const Config& q) const;

  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle);

  Real visibilityEpsilon;
  Geometric2DCollection obstacles;
  Geometric2DCollection robot;
  AABB2D domain;
};

#endif
