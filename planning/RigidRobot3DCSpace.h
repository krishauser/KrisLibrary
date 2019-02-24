#ifndef RIGID_ROBOT_2D_CSPACE_H
#define RIGID_ROBOT_2D_CSPACE_H

#include <KrisLibrary/geometry/AnyGeometry.h>

/** @ingroup Planning
 * @brief a translating and rotating 3D robot in a 3D workspace.
 *
 * Not implemented yet.
 */
class RigidRobot3DCSpace : public SE3CSpace
{
public:
  RigidRobot3DCSpace();
  void DrawWorkspaceGL() const;
  void DrawRobotGL(const Config& q) const;
  void DrawGL(const Config& q) const;

  virtual int NumObstacles();
  virtual std::string ObstacleName(int obstacle);
  virtual bool IsFeasible(const Config& x);
  virtual bool IsFeasible(const Config& x,int obstacle);
  virtual EdgePlannerPtr PathChecker(const InterpolatorPtr& path);
  virtual EdgePlannerPtr PathChecker(const InterpolatorPtr& path,int obstacle);
  virtual void Properties(PropertyMap&) const;

  std::vector<Geometry::AnyCollisionGeometry3D> obstacles;
  Geometry::AnyCollisionGeometry3D robot;
};

#endif
