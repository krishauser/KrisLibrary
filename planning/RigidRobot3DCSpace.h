#ifndef RIGID_ROBOT_2D_CSPACE_H
#define RIGID_ROBOT_2D_CSPACE_H

#include <KrisLibrary/geometry/AnyGeometry.h>

/** @brief a translating and rotating 2D robot in a 2D workspace.
 */
class RigidRobot3DCSpace : public SE3CSpace, ExplicitCSpace
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
  virtual EdgePlanner* PathChecker(const SmartPointer<Interpolator>& path);
  virtual EdgePlanner* PathChecker(const SmartPointer<Interpolator>& path,int obstacle);
  virtual void Properties(PropertyMap&) const;

  std::vector<Geometry::AnyCollisionGeometry3D> obstacles;
  Geometry::AnyCollisionGeometry3D robot;
};

#endif
