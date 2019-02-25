#ifndef OPTIMAL_MOTION_PLANNER_H
#define OPTIMAL_MOTION_PLANNER_H

#include "MotionPlanner.h"
#include <KrisLibrary/graph/ShortestPaths.h>

/** @brief Implements the asymptotically optimal kinematic motion planners
 * PRM*, RRT* / RRG*, Lazy-PRM*, and Lazy-RRG*.  Also allows the asymptotically
 * sub-optimal method LBT-RRG*.
 *
 * Compared to RRT*, RRG* only has a tiny amount of overhead to maintain edges
 * in the graph structure.  No additional edges are collision checked.
 *
 * Allows unidirectional or bidirectional search.
 */
class PRMStarPlanner : public RoadmapPlanner
{
 public:
  PRMStarPlanner(CSpace* space);
  ///Initialize with a start and goal configuration
  void Init(const Config& start,const Config& goal);
  ///Define a maximum path length.  Must be called after Init.
  void SetMaxCost(Real cmax);
  ///Erases all internal data structures
  virtual void Cleanup();
  ///Perform one planning step
  void PlanMore();
  ///Helper: perform K-nearest neighbor query
  void KNN(const Config& x,int k,vector<int>& nn);
  ///Helper: perform neighbor query limited by radius r
  void Neighbors(const Config& x,Real r,vector<int>& neighbors);
  ///Helper: returns true if there exists a feasible path from start to goal
  bool HasPath() const;
  ///Helper: get path from start to goal
  bool GetPath(MilestonePath& path);
  ///Helper: get path from milestone a to b
  bool GetPath(int a,int b,vector<int>& nodes,MilestonePath& path);
  ///Helper: get path from start to goal that optimizes some cost function
  Real OptimizePath(ObjectiveFunctionalBase* cost,MilestonePath& path);
  ///Helper: get path from milestone a to some goal node in goals that optimizes some cost function
  Real OptimizePath(int a,const vector<int>& goals,ObjectiveFunctionalBase* cost,MilestonePath& path);
  ///Helper: check feasibility of path from milestone a to b for lazy planning
  bool CheckPath(int a,int b);
  ///Helper: add a milestone and update data structures
  virtual int AddMilestone(const Config& x);
  ///Helper: add a (feasible) edge, and update data structures
  virtual void ConnectEdge(int i,int j,const EdgePlannerPtr& e);
  ///Helper: add an unchecked edge, and update data structures
  void ConnectEdgeLazy(int i,int j,const EdgePlannerPtr& e);

  //configuration variables
  ///Set lazy to true if you wish to do lazy planning (default false)
  bool lazy;
  ///Set rrg to true if you wish to use the RRG* algorithm rather than PRM*
  bool rrg;
  ///Set this to true for bidirectional planning
  bool bidirectional;
  ///If this is false, uses k*-nearest neighbors. If this is true, uses radius
  ///gamma * (log(n)/n)^(1/d) where n is the number of milestones and d is
  ///dimension.
  bool connectByRadius;
  ///Constant term gamma in the above expression. Default 1.
  Real connectRadiusConstant;
  ///If connectByRadius is false, the value of k* for k-nearest neighbors is
  ///chosen to be connectNeighborsConstant*e*(1+1/d)*log(n) where n is the number of
  ///milestones (default 1.1)
  Real connectNeighborsConstant;
  ///Set this value to limit the maximum distance of attempted
  ///connections
  Real connectionThreshold;
  ///If lazy planning, check all edges with length greater than this threshold
  Real lazyCheckThreshold;
  ///For suboptimal planning (like LBT-RRT*), default 0
  Real suboptimalityFactor;

  int start,goal;
  typedef Graph::ShortestPathProblem<Config,EdgePlannerPtr> ShortestPathProblem;
  ShortestPathProblem spp,sppGoal,sppLB,sppLBGoal;
  Roadmap LBroadmap;

  //statistics
  int numPlanSteps;
  Real tCheck, tKnn, tConnect, tLazy, tLazyCheck, tShortestPaths;
  int numEdgeChecks;
  int numEdgePrechecks;
};


#endif
