#ifndef ROBOTICS_KINODYNAMIC_MOTION_PLANNER_H
#define ROBOTICS_KINODYNAMIC_MOTION_PLANNER_H

#include "KinodynamicSpace.h"
#include "KinodynamicPath.h"
#include "PointLocation.h"
#include "DensityEstimator.h"
#include "Objective.h"
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
  struct EdgeData
  {
    KinodynamicMilestonePath path;
    EdgePlannerPtr checker;
  };
  typedef Graph::TreeNode<State,EdgeData> Node;

  KinodynamicTree(KinodynamicSpace* s);
  ~KinodynamicTree();
  void Init(const State& initialState);
  void EnablePointLocation(const char* type=NULL);
  void Clear();
  Node* AddMilestone(Node* parent,const ControlInput& u);
  Node* AddMilestone(Node* parent,const ControlInput& u,const InterpolatorPtr& path,const EdgePlannerPtr& e);
  Node* AddMilestone(Node* parent,KinodynamicMilestonePath& path,const EdgePlannerPtr& e=NULL);
  void AddPath(Node* n0,const KinodynamicMilestonePath& path,std::vector<Node*>& res);
  void Reroot(Node* n);
  Node* PickRandom();
  Node* FindClosest(const State& x);
  ///Deletes n and its subtree.  If point location is not enabled or rebuild=true, cost is O(k)
  ///where k is the size of the subtree.  Otherwise, cost is O(N) where N is the number of nodes!
  ///If you plan to make several deletions, delete the subtrees with rebuild=false, then call
  ///RebuildPointLocation before calling FindClosest() or PickRandom() again.
  void DeleteSubTree(Node* n,bool rebuild=true);
  void RebuildPointLocation();

  static void GetPath(Node* start,Node* goal,KinodynamicMilestonePath& path);

  KinodynamicSpace* space;
  Node* root;

  ///If point location is enabled, this will contain a point location data structure
  std::shared_ptr<PointLocationBase> pointLocation;
  std::vector<Node*> index;
  std::vector<Vector> pointRefs;
};

class KinodynamicPlannerBase
{
public:
  KinodynamicPlannerBase(KinodynamicSpace* s);
  virtual ~KinodynamicPlannerBase() {}
  virtual void Init(const State& xinit,CSet* goalSet)=0;
  virtual void Init(const State& xinit,const State& xgoal,Real goalRadius);
  virtual bool Plan(int maxIters)=0;
  virtual bool Done() const=0;
  virtual bool GetPath(KinodynamicMilestonePath& path)=0;
  virtual void GetStats(PropertyMap& stats) const {}

  KinodynamicSpace* space;
  CSet* goalSet;
};

/** @brief The RRT planner for kinodynamic systems.
 */
class RRTKinodynamicPlanner : public KinodynamicPlannerBase
{
public:
  typedef KinodynamicTree::Node Node;

  RRTKinodynamicPlanner(KinodynamicSpace* s);
  virtual ~RRTKinodynamicPlanner() {}
  virtual void Init(const State& xinit,CSet* goalSet);
  virtual bool Plan(int maxIters);
  virtual bool Done() const;
  virtual bool GetPath(KinodynamicMilestonePath& path);

  virtual Node* Extend();
  virtual Node* ExtendToward(const State& xdest);
  virtual void PickDestination(State& xdest);

  //default move uses steering function, if available, and if not, uses a RandomSamplingSteeringFunction
  virtual bool PickControl(const State& x0, const State& xDest,KinodynamicMilestonePath& e);

  ///Subclasses can overload this to eliminate certain extensions of the tree
  virtual bool FilterExtension(Node* n,const KinodynamicMilestonePath& path) { return false; }

  virtual void GetStats(PropertyMap& stats) const;

  Real goalSeekProbability;
  KinodynamicTree tree;
  Real delta;

  //temporary output
  Node* goalNode;
  int numIters,numInfeasibleControls,numInfeasibleEndpoints,numFilteredExtensions,numSuccessfulExtensions;
  Real nnTime,pickControlTime,visibleTime,overheadTime;
};


/** @brief The EST planner for kinodynamic systems.
 *
 * Node selection is performed by a density-weighted sampling
 * from a grid-based density estimator.  (MultiGridDensityEstimator)
 *
 * This has an implementation of a semi-lazy method in that it maintains
 * a difference between "official" and "unofficial" contributions to the tree. 
 * If extensionCacheSize > 0 (0 by default), then the extension cache
 * contains several candidate extensions, and at each iteration tries
 * making one official by sampling one candidate
 * inversely proportional to their density, and then checking collisions. 
 */
class ESTKinodynamicPlanner : public KinodynamicPlannerBase
{
public:
  typedef KinodynamicTree::Node Node;

  ESTKinodynamicPlanner(KinodynamicSpace* s);
  virtual ~ESTKinodynamicPlanner() {}
  void EnableExtensionCaching(int cacheSize=100);
  void SetDensityEstimatorResolution(Real res);
  void SetDensityEstimatorResolution(const Vector& res);
  virtual void Init(const State& xinit,CSet* goalSet);
  virtual bool Plan(int maxIters);
  virtual bool Done() const;
  virtual bool GetPath(KinodynamicMilestonePath& path);
  ///Subclasses can overload this to eliminate certain extensions of the tree
  virtual bool FilterExtension(Node* n,const KinodynamicMilestonePath& path) { return false; }
  virtual void GetStats(PropertyMap& stats) const;
  
  void RebuildDensityEstimator();

  KinodynamicTree tree;
  std::shared_ptr<DensityEstimatorBase> densityEstimator;
  int extensionCacheSize;

  ///These are a list of nodes that have been added to the tree but
  ///not checked for collision nor added to the density estimator.
  std::vector<Node*> extensionCache;
  std::vector<double> extensionWeights;
  
  //temporary output
  Node* goalNode;
  int numIters,numFilteredExtensions,numSuccessfulExtensions;
  Real sampleTime,simulateTime,visibleTime,overheadTime;
};


/** @brief A lazy version of kinodynamic RRT.
 */
class LazyRRTKinodynamicPlanner : public RRTKinodynamicPlanner
{
public:
  LazyRRTKinodynamicPlanner(KinodynamicSpace* s);
  virtual ~LazyRRTKinodynamicPlanner() {}
  virtual bool Plan(int maxIters);
  virtual Node* ExtendToward(const State& xdest);
  bool CheckPath(Node* n);
};

/** @brief A bidirectional RRT planner for kinodynamic systems.
 */
class BidirectionalRRTKP : public KinodynamicPlannerBase
{
public:
  typedef KinodynamicTree::Node Node;

  BidirectionalRRTKP(KinodynamicSpace* s);
  virtual ~BidirectionalRRTKP() {}
  void Init(const State& xStart,const State& xGoal);
  virtual void Init(const State& xinit,CSet* goalSet);
  virtual bool Plan(int maxIters);
  virtual bool Done() const;
  virtual bool GetPath(KinodynamicMilestonePath& path);
  Node* ExtendStart();
  Node* ExtendGoal();

  //if this returns true, there's a visible path between nStart and nGoal.
  //bridge is filled out with the connection information.
  bool ConnectTrees(Node* nStart,Node* nGoal);

  //default move uses steering function
  virtual bool PickControl(const State& x0, const State& xDest, KinodynamicMilestonePath& path);
  virtual bool PickReverseControl(const State& x1, const State& xDest, KinodynamicMilestonePath& path);

  KinodynamicTree start,goal;
  Real connectionTolerance;

  struct Bridge 
  {
    Node *nStart,*nGoal;
    KinodynamicMilestonePath path;
    EdgePlannerPtr checker;
  };

  Bridge bridge;
};


#endif
