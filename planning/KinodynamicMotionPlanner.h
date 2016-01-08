#ifndef ROBOTICS_KINODYNAMIC_MOTION_PLANNER_H
#define ROBOTICS_KINODYNAMIC_MOTION_PLANNER_H

#include "KinodynamicCSpace.h"
#include "KinodynamicPath.h"
#include <KrisLibrary/graph/Tree.h>
#include <queue>
typedef Vector State;
typedef Vector ControlInput;

/** @brief Data structure for a kinodynamic planning tree.
 *
 * The planning tree has nodes that are states and edges that store
 * control inputs, traces, and local planners.  Specifically, the edge
 * x1->x2 stores the input u s.t.
 * x2=f(x1,u), the trace from x1->x2, and the edge planner for that trace.
 * This data can be retrieved using x2->getEdgeFromParent().
 *
 * Potential improvement can speed up point location using Kd trees.
 */
class KinodynamicTree
{
public:
  struct EdgeData {
    ControlInput u;
    std::vector<State> path;
    SmartPointer<EdgePlanner> e;
  };
  typedef Graph::TreeNode<State,EdgeData> Node;

  KinodynamicTree(KinodynamicCSpace* s);
  ~KinodynamicTree();
  void Init(const State& initialState);
  void Clear();
  Node* AddMilestone(Node* parent,const ControlInput& u,const State& x);
  Node* AddMilestone(Node* parent,const ControlInput& u,const std::vector<State>& path,const SmartPointer<EdgePlanner>& e);
  void AddPath(Node* n0,const KinodynamicMilestonePath& path,std::vector<Node*>& res);
  void Reroot(Node* n);
  Node* PickRandom() const;
  Node* FindClosest(const State& x) const;
  Node* ApproximateRandomClosest(const State& x,int numIters) const;
  void DeleteSubTree(Node* n);

  static void GetPath(Node* start,Node* goal,KinodynamicMilestonePath& path);

  KinodynamicCSpace* space;
  Node* root;
  std::vector<Node*> index;
};


/** @brief The RRT planner for kinodynamic systems.
 */
class RRTKinodynamicPlanner
{
public:
  typedef KinodynamicTree::Node Node;

  RRTKinodynamicPlanner(KinodynamicCSpace* s);
  virtual ~RRTKinodynamicPlanner() {}
  virtual void Init(const State& xinit);
  virtual Node* Plan(int maxIters);
  virtual Node* Extend();
  virtual Node* ExtendToward(const State& xdest);
  virtual void PickDestination(State& xdest);
  bool IsDone() const;
  void CreatePath(KinodynamicMilestonePath& path) const;

  //default move uses space->BiasedSampleControl()
  virtual void PickControl(const State& x0, const State& xDest, ControlInput& u);

  KinodynamicCSpace* space;
  Real goalSeekProbability;
  CSpace* goalSet;
  KinodynamicTree tree;

  //temporary output
  Node* goalNode;
};

/** @brief A lazy version of kinodynamic RRT.
 */
class LazyRRTKinodynamicPlanner : public RRTKinodynamicPlanner
{
public:
  LazyRRTKinodynamicPlanner(KinodynamicCSpace* s);
  virtual ~LazyRRTKinodynamicPlanner() {}
  Node* Plan(int maxIters);
  Node* ExtendToward(const State& xdest);
  bool CheckPath(Node* n);
};

/** @brief A bidirectional RRT planner for kinodynamic systems.
 */
class BidirectionalRRTKP
{
public:
  typedef KinodynamicTree::Node Node;

  BidirectionalRRTKP(KinodynamicCSpace* s);
  virtual ~BidirectionalRRTKP() {}
  void Init(const State& xStart,const State& xGoal);
  bool Plan(int maxIters);
  bool IsDone() const;
  void CreatePath(KinodynamicMilestonePath& path);
  Node* ExtendStart();
  Node* ExtendGoal();

  //if this returns true, there's a visible path between nStart and nGoal.
  //bridge is filled out with the connection information.
  bool ConnectTrees(Node* nStart,Node* nGoal);

  //default move uses space->BiasedSampleControl()
  virtual void PickControl(const State& x0, const State& xDest, ControlInput& u);
  virtual void PickReverseControl(const State& x1, const State& xDest, ControlInput& u);

  KinodynamicCSpace* space;
  KinodynamicTree start,goal;
  Real connectionTolerance;

  struct Bridge 
  {
    Node *nStart,*nGoal;
    ControlInput u;
    std::vector<State> path;
    SmartPointer<EdgePlanner> e;
  };

  Bridge bridge;
};

#endif
