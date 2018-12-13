#ifndef MCR_PLANNER_H
#define MCR_PLANNER_H

#include "MotionPlanner.h"
#include "CSpace.h"
#include <KrisLibrary/utils/Subset.h>

/** @brief A planner that minimizes the the number of violated constraints
 * using a RRT-like strategy.
 * 
 * Usage:
 *   //first, set up ExplicitCSpace cspace.
 *   MCRPlanner planner(&cspace);
 *   planner.Init(start,goal);
 *   
 *   //do planning with a given expansion schedule
 *   vector<int> schedule;
 *   schedule.push_back(limit1);
 *     ...
 *   schedule.push_back(limitN);
 *   Subset cover;
 *   vector<int> bestPlan;
 *   planner.Plan(0,schedule,bestPlan,cover);
 *
 *   //output best path
 *   MilestonePath path;
 *   planner.GetMilestonePath(bestPlan,path);
 */
class MCRPlanner
{
 public:
  struct Milestone {
    Config q;
    int mode;
  };
  struct Edge {
    EdgePlannerPtr e;
    int mode;
  };
  typedef Graph::UndirectedGraph<Milestone,Edge> Roadmap;

  struct Mode {
    Subset subset;      //subset covered by this mode
    std::vector<int> roadmapNodes;
    std::vector<Subset> pathCovers;   //minimal covers leading from the start to this mode
    Real minCost;
  };
  struct Transition {
    std::vector<std::pair<int,int> > connections;
  };
  typedef Graph::UndirectedGraph<Mode,Transition> ModeGraph;

  MCRPlanner(CSpace* space);
  void Init(const Config& start,const Config& goal);
  ///Performs one iteration of planning given a limit on the explanation size
  void Expand(Real maxExplanationCost,std::vector<int>& newNodes);
  void Expand2(Real maxExplanationCost,std::vector<int>& newNodes);
  ///Performs bottom-up planning according to a given limit expansion schedule
  void Plan(int initialLimit,const std::vector<int>& expansionSchedule,std::vector<int>& bestPath,Subset& cover);
  ///Outputs the graph with the given explanation limit
  void BuildRoadmap(Real maxExplanationCost,RoadmapPlanner& prm);
  ///Outputs the CC graph.  Each node is a connected component of the roadmap
  ///within the same subset.
  void BuildCCGraph(Graph::UndirectedGraph<Subset,int>& G);
  ///A search that finds a path subject to a coverage constraint
  bool CoveragePath(int s,int t,const Subset& cover,std::vector<int>& path,Subset& pathCover);
  ///A greedy heuristic that performs smallest cover given predecessor
  bool GreedyPath(int s,int t,std::vector<int>& path,Subset& pathCover);
  ///An optimal search
  bool OptimalPath(int s,int t,std::vector<int>& path,Subset& pathCover);
  ///Returns the cover of the path from s->node + completion(node,goal)
  ///where the path cover is determined using the greedy
  ///heuristic
  void Completion(int s,int node,int t,Subset& pathCover);

  //helpers
  Real Cost(const Subset& s) const;
  int AddNode(const Config& q,int parent=-1);
  int AddNode(const Config& q,const Subset& subset,int parent=-1);
  bool AddEdge(int i,int j,int depth=0);
  int AddEdge(int i,const Config& q,Real maxExplanationCost);  //returns index of q
  void AddEdgeRaw(int i,int j);
  int ExtendToward(int i,const Config& qdest,Real maxExplanationCost);
  void KNN(const Config& q,int k,std::vector<int>& neighbors,std::vector<Real>& distances);
  void KNN(const Config& q,Real maxExplanationCost,int k,std::vector<int>& neighbors,std::vector<Real>& distances);
  void UpdatePathsGreedy();
  void UpdatePathsComplete();
  void UpdatePathsGreedy2(int nstart=-1);
  void UpdatePathsComplete2(int nstart=-1);
  //returns true if a path from mode a to mode b can improve the explanation
  //at b
  bool CanImproveConnectivity(const Mode& ma,const Mode& mb,Real maxExplanationCost);
  //updates the minCost member of m
  void UpdateMinCost(Mode& m);
  //fast checking of whether the cost of the local constraints at q exceed the
  //given limit
  bool ExceedsCostLimit(const Config& q,Real limit,Subset& violations);
  //fast checking of whether the cost of the local constraints violated on 
  //the edge ab exceed the given limit
  bool ExceedsCostLimit(const Config& a,const Config& b,Real limit,Subset& violations);

  ///Computes the cover of the path
  void GetCover(const std::vector<int>& path,Subset& cover) const;
  ///Computes the length of the path
  Real GetLength(const std::vector<int>& path) const;
  ///Returns the MilestonePath
  void GetMilestonePath(const std::vector<int>& path,MilestonePath& mpath) const;
 
  Config start,goal;
  CSpace* space;

  //weighted explanation
  std::vector<Real> obstacleWeights;
 
  //settings for RRT* like expansion
  int numConnections;
  Real connectThreshold,expandDistance,goalConnectThreshold;
  Real goalBiasProbability;  //probability of expanding toward the goal
  bool bidirectional;        //not functional yet
  
  //settings for path update
  ///If true: use the slower complete, exact cover update.
  ///If false: use the faster greedy one.
  bool updatePathsComplete;
  ///If true: do dynamic shortest paths update
  ///If false: do batch updates when needed
  bool updatePathsDynamic;
  ///For complete planning, keep at most this number of covers 
  int updatePathsMax;

  Roadmap roadmap;
  ModeGraph modeGraph;

  //planning statistics
  int numExpands,numRefinementAttempts,numRefinementSuccesses,numExplorationAttempts,
    numEdgeChecks,numConfigChecks,
    numUpdatePaths,numUpdatePathsIterations;
  double timeNearestNeighbors,timeRefine,timeExplore,timeUpdatePaths,timeOverhead;
};

#endif
