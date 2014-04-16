#ifndef OPTIMAL_MOTION_PLANNER_H
#define OPTIMAL_MOTION_PLANNER_H

#include "MotionPlanner.h"
#include <graph/ShortestPaths.h>

class PRMStarPlanner : public RoadmapPlanner
{
 public:
  PRMStarPlanner(CSpace* space);
  ///Initialize with a start and goal configuration
  void Init(const Config& start,const Config& goal);
  ///Perform one planning step
  void PlanMore();
  ///Helper: perform K-nearest neighbor query
  void KNN(const Config& x,int k,vector<int>& nn);
  ///Helper: perform neighbor query limited by radius r
  void Neighbors(const Config& x,Real r,vector<int>& neighbors);
  ///Helper: get path from start to goal
  bool GetPath(MilestonePath& path);
  ///Helper: get path from milestone a to b
  bool GetPath(int a,int b,vector<int>& nodes,MilestonePath& path);
  ///Helper: check feasibility of path from milestone a to b for lazy planning
  bool CheckPath(int a,int b);

  //configuration variables
  ///Set lazy to true if you wish to do lazy planning (default false)
  bool lazy;
  ///Set random neighbors to true if you wish to pick random neighbors
  ///(faster but not asymptotically optimal)
  bool randomNeighbors;
  ///If this is false, uses k-nearest neighbors. If this is true, uses radius
  ///gamma * (log(n)/n)^(1/d) where n is the number of milestones and d is
  ///dimension.
  bool connectByRadius;
  ///Constant term gamma in the above expression. Default 1.
  Real connectRadiusConstant;
  ///Set this value to limit the maximum distance of attempted
  ///connections
  Real connectionThreshold;

  int start,goal;
  typedef Graph::ShortestPathProblem<Config,SmartPointer<EdgePlanner> > ShortestPathProblem;
  ShortestPathProblem spp;

  //statistics
  int numPlanSteps;
  Real tCheck, tKnn, tConnect, tLazy;
  int numEdgeChecks;
};


#endif
