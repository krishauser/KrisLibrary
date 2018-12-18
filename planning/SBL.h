#ifndef ROBOTICS_SBL_H
#define ROBOTICS_SBL_H

#include "SBLTree.h"
#include "Path.h"
#include <KrisLibrary/graph/UndirectedGraph.h>
#include <KrisLibrary/graph/ConnectedComponents.h>

/** @ingroup MotionPlanning
 * @brief The SBL motion planner.
 *
 * Call Init() with the start and goal configuration.  While IsDone()
 * returns false, call Extend() .  Retrieve the path with
 * CreatePath().
 *
 * Parameters are maxExtendDistance, maxExtendIters,
 * edgeConnectionThreshold.  maxExtendDistance is the radius of the
 * neighborhood sampling.  maxExtendIters is the number of iters of 
 * shrinking the neighborhood sampling radius until we quit. 
 * edgeConnectionThreshold is the minimum distance required for a connection
 * between the two trees.
 */
class SBLPlanner
{
public:
  typedef SBLTree::Node Node;
  typedef SBLTree::EdgeInfo EdgeInfo;

  SBLPlanner(CSpace*);
  virtual ~SBLPlanner();
  virtual void Cleanup();
  virtual void Init(const Config& qStart,const Config& qGoal);
  virtual bool Extend();
  virtual Node* PickConnection(SBLTree* t,const Config& x) { return t->FindClosest(x); }

  bool IsDone() const { return !outputPath.empty(); }
  void CreatePath(MilestonePath& path) const;

  //helper, no need to call this
  bool CheckPath(Node* nStart,Node* nGoal);  //path from start->ns->ng->goal

  CSpace* space;
  Real maxExtendDistance;
  int maxExtendIters;
  Real edgeConnectionThreshold;

  int numIters;
  SBLTree *tStart, *tGoal;
  std::list<EdgeInfo> outputPath;
};

/** @brief An SBL planner whose trees use grids for point location.
 *
 * Every numItersPerRandomize Extend() iterations, the grid dimensions are
 * randomized.  They use a 3-d grid with division gridDivision.
 */
class SBLPlannerWithGrid : public SBLPlanner
{
 public:
  SBLPlannerWithGrid(CSpace*);
  virtual void Cleanup();
  virtual void Init(const Config& qStart,const Config& qGoal);
  virtual bool Extend();
  virtual Node* PickConnection(SBLTree* t,const Config& x);
  void RandomizeSubset();

  int numItersPerRandomize;
  Real gridDivision;
};

/** @brief A probabilistic roadmap of trees, where SBL is used for
 * local planning.
 *
 * Permitted connections are represented by edges in an undirected graph.
 * Plans are attempted only along these edges.  An edge has a corresponding
 * MilestonePath which will contain the path between the seeds once planning
 * is complete.
 */
class SBLPRT
{
 public:
  typedef SBLTree::Node Node;
  typedef SBLTree::EdgeInfo EdgeInfo;
  typedef Graph::UndirectedGraph<SBLTree*,MilestonePath> Roadmap;

  SBLPRT(CSpace* s);
  virtual ~SBLPRT();
  virtual void Cleanup();
  int AddSeed(const Config& q);
  std::pair<int,int> Expand();  //picks a random tree, returns connection if made
  int ExpandTree(int t);  //expands tree t, calls PickConnection to attempt a connection, and returns the connected tree if successful, or -1 on failure

  void AddRoadmapEdgesIfBelowThreshold(Real distanceThreshold);
  void AddRoadmapEdge(int i,int j) { roadmap.AddEdge(i,j); }
  bool IsEdgeConnected(int i,int j) const;
  bool IsSeedFullyConnected(int i) const;
  bool AreSeedsConnected(int i,int j) const { return ccs.GetComponent(i) == ccs.GetComponent(j); }
  void CreatePath(int i,int j,MilestonePath& path);

  virtual std::pair<int,Node*> PickConnection(int t,Node* n);
  //helpers for picking a tree/node to connect
  int PickRandomAdjacentTree(int t);
  int PickClosestAdjacentTree(int t,const Config& x);
  Node* GetClosestNode(int t,const Config& x) { return roadmap.nodes[t]->FindClosest(x); }
  Node* PickNode(int t) { return roadmap.nodes[t]->PickExpand(); }

  CSpace* space;
  Real maxExtendDistance;
  int maxExtendIters;
  Real defaultPPickClosestTree,defaultPPickClosestNode;

  int numIters;
  Roadmap roadmap;          //the roadmap nodes
  Graph::ConnectedComponents ccs;  //connected components of the roadmap
};

#endif

