#ifndef MULTI_ROBOT_2D_CSPACE_H
#define MULTI_ROBOT_2D_CSPACE_H

#include "Geometric2DCSpace.h"
#include "RigidRobot2DCSpace.h"
#include "TranslatingRobot2DCSpace.h"

/** @brief One or more 2D robots translating and/or rotating in a 2D workspace.
 */
class MultiRobot2DCSpace : public CSpace
{
public:
  MultiRobot2DCSpace();
  void DrawWorkspaceGL() const;
  void DrawRobotGL(int index,const RigidTransform2D& T) const;
  void DrawRobotGL(int index,const Config& q) const;
  void DrawGL(const Config& q) const;
  RigidTransform2D GetRobotTransform(int index,const Config& q) const;

  /* TODO: implement these
  void GetSingleRobotConfig(const Config& q,int index,Config& qrobot) const;
  void GetSingleRobotCSpace(const Config& q,int index,RigidRobot2DCSpace& space) const;
  void GetSingleRobotCSpace(const Config& q,int index,TranslatingRobot2DCSpace& space) const;
  void GetSubsetRobotConfig(const Config& q,const vector<int>& indices,Config& qrobots) const;
  void GetSubsetRobotCSpace(const Config& q,const vector<int>& indices,MultiRobot2DCSpace& space) const;
  */

  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Properties(PropertyMap&) const;

  bool allowRotation;
  Real angleDistanceWeight;
  Real visibilityEpsilon;
  AABB2D domain;
  Geometric2DCollection obstacles;
  vector<Geometric2DCollection> robots;
};

#endif
