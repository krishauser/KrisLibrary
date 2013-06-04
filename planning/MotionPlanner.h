#ifndef ROBOTICS_MOTION_PLANNER_H
#define ROBOTICS_MOTION_PLANNER_H

#include <graph/Tree.h>
#include <graph/UndirectedGraph.h>
#include <graph/ConnectedComponents.h>
#include <utils/SmartPointer.h>
#include <vector>
#include <list>
#include "CSpace.h"
#include "EdgePlanner.h"
#include "Path.h"

/** @defgroup MotionPlanning
 * @brief Classes to assist in motion planning.
 */

/** @ingroup MotionPlanning
 * @brief A base roadmap planner class.
 */
class RoadmapPlanner
{
public:
  typedef Graph::UndirectedGraph<Config,SmartPointer<EdgePlanner> > Roadmap;

  RoadmapPlanner(CSpace*);
  virtual ~RoadmapPlanner();
  
  virtual void Cleanup();
  virtual void GenerateConfig(Config& x);
  virtual int AddMilestone(const Config& x);
  virtual int TestAndAddMilestone(const Config& x);
  virtual void ConnectEdge(int i,int j,const SmartPointer<EdgePlanner>& e);
  virtual SmartPointer<EdgePlanner> TestAndConnectEdge(int i,int j);
  virtual bool HasEdge(int i,int j) { return roadmap.FindEdge(i,j)!=NULL; }
  virtual SmartPointer<EdgePlanner> GetEdge(int i,int j) { return *roadmap.FindEdge(i,j); }
  virtual bool AreConnected(int i,int j) { return ccs.SameComponent(i,j); }
  virtual bool AreConnected(int i,int j) const { return ccs.SameComponent(i,j); }
  virtual void ConnectToNeighbors(int i,Real connectionThreshold,bool ccReject=true);
  virtual void ConnectToNearestNeighbors(int i,int k,bool ccReject=true);
  virtual void Generate(int numSamples,Real connectionThreshold); 
  virtual void CreatePath(int i,int j,MilestonePath& path);

  CSpace* space;
  Roadmap roadmap;
  Graph::ConnectedComponents ccs;
};


/** @ingroup MotionPlanning
 * @brief A base class to be used for tree-based roadmap planners.
 *
 * connectionThreshold is the minimum distance two nodes must be before
 * a connection may be made between them.  This is infinity by default.
 * If it is infinity, connections are attempted to the closest node in
 * a different component.
 */
class TreeRoadmapPlanner
{
public:
  struct Milestone
  {
    Config x;
    int connectedComponent;
  };
  
  typedef Graph::TreeNode<Milestone,SmartPointer<EdgePlanner> > Node;
  
  TreeRoadmapPlanner(CSpace*);
  virtual ~TreeRoadmapPlanner();
  
  virtual void GenerateConfig(Config& x);
  virtual Node* AddMilestone(const Config& x);
  virtual Node* TestAndAddMilestone(const Config& x);
  virtual Node* AddInfeasibleMilestone(const Config& x) { return NULL; }
  virtual Node* Extend(); 
  virtual void Cleanup();
  virtual void ConnectToNeighbors(Node*);
  virtual EdgePlanner* TryConnect(Node*,Node*);
  virtual void DeleteSubtree(Node* n);
  //helpers
  //default implementation is O(n) search
  virtual Node* ClosestMilestone(const Config& x);
  virtual Node* ClosestMilestoneInComponent(int component,const Config& x);
  virtual Node* ClosestMilestoneInSubtree(Node* node,const Config& x);
  Node* Extend(Node* n,const Config& x);
  Node* TryExtend(Node* n,const Config& x);
  void AttachChild(Node* p, Node* c, EdgePlanner* e);   //c will become a child of p
  Node* SplitEdge(Node* p,Node* n,Real u);
  void CreatePath(Node* a, Node* b, MilestonePath& path);
  
  CSpace* space;
  std::vector<Node*> connectedComponents;
  Real connectionThreshold;
  
  //temporary
  std::vector<Node*> milestones;
  Config x;
};


/** @ingroup MotionPlanning
 * @brief A tree-based randomized planner that extends the roadmap by
 * sampling the neighborhood of existing samples.
 *
 * The existing sample picked is selected with probability proportion
 * to its value in the weight vector.
 * By default, a new node is given weight 1.
 */
class PerturbationTreePlanner : public TreeRoadmapPlanner
{
public:
  PerturbationTreePlanner(CSpace*s);
  virtual void GenerateConfig(Config& x);
  virtual Node* AddMilestone(const Config& x); 
  virtual void Cleanup();

  //overrideable 
  virtual Node* SelectMilestone(const std::vector<Node*>& milestones);

  ///Neighborhood distance
  Real delta;
  ///Node selection weights
  std::vector<Real> weights;
};

/** @ingroup MotionPlanning
 * @brief A basic RRT (Rapidly-Exploring Random Tree) planner.
 *
 * Max distance to expand existing nodes is given in delta.
 *
 * Currently this does not attempt to connect separate trees.
 */
class RRTPlanner : public TreeRoadmapPlanner
{
public:
  RRTPlanner(CSpace*s);
  virtual Node* Extend();
  
  Real delta;
};

/** @ingroup MotionPlanning
 * @brief A single-query RRT (Rapidly-Exploring Random Tree) planner.
 *
 * Consists of two trees, one from the start, the other from the goal.
 * Tries to connect the two when an extended config is within
 * connectionThreshold of the other tree.
 *
 * The start and goal configs are stored in milestones 0 and 1, resp.
 */
class BidirectionalRRTPlanner : public RRTPlanner
{
public:
  BidirectionalRRTPlanner(CSpace*s);
  /// Clears the trees, then initializes the start/goal configs
  void Init(const Config& start, const Config& goal);
  /// Performs 1 step of planning, returns true on success
  bool Plan();
  /// Returns the planned path, if successful
  void CreatePath(MilestonePath&) const;
};

/*
class VisibilityPRM : public RandomizedPlanner
{
public:
  VisibilityPRM(CSpace*s);
  //may return NULL for rejected config
  virtual Node* AddMilestone(const Config& x);
  virtual Node* AddInfeasibleMilestone(const Config& x) { return NULL; }
  virtual Node* Extend();
  virtual Node* CanConnectComponent(int i,const Config& x);
};
*/

#endif
