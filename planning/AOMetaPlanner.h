#ifndef PLANNING_AO_META_PLANNER_H
#define PLANNING_AO_META_PLANNER_H

#include "KinodynamicMotionPlanner.h"
#include "Objective.h"

/** @brief An optimizing planner that progressively produces better paths using state-cost space.
 * Plan() returns true only if a better path was found in the last maxIters iterations.  Done() and
 * GetPath() return true if there ANY solution has been found.
 *
 * The method is asymptotically optimal under very general assumptions about the space and objective function.
 *
 * See K. Hauser and Y. Zhou, "Asymptotically-Optimal Kinodynamic Motion Planning in State-Cost Space"
 * IEEE Transactions on Robotics, 2016.
 */
class CostSpaceRRTPlanner : public LazyRRTKinodynamicPlanner
{
public:
  typedef Real (*HeuristicFn) (const Config& x);

  CostSpaceRRTPlanner(const std::shared_ptr<KinodynamicSpace>& baseSpace,const std::shared_ptr<ObjectiveFunctionalBase>& objective,Real costMax=Inf);
  virtual ~CostSpaceRRTPlanner() {}
  void EnableCostSpaceBias(bool enabled);
  void SetCostMax(Real cmax);
  void SetCostDistanceWeight(Real weight);
  ///A heuristic function is a lower bound on the cost-to-go of a state, given the current objective function.
  ///This function tells the planner to use this heuristic when sampling / pruning.
  void SetHeuristic(HeuristicFn f);
  virtual void Init(const State& xinit,CSet* goalSet);
  virtual bool Plan(int maxIters);
  virtual bool Done() const;
  virtual bool GetPath(KinodynamicMilestonePath& path);
  virtual void PickDestination(State& xdest);
  virtual bool FilterExtension(Node* n,const KinodynamicMilestonePath& path);
  virtual RRTKinodynamicPlanner::Node* ExtendToward(const State& xdest);
  virtual void OnNewBestPath();
  virtual void GetStats(PropertyMap& stats) const;
  void PruneTree();
  Real PathCost() const;
  Real TerminalCost(const State& xc);

  std::shared_ptr<KinodynamicSpace> baseSpace;
  std::shared_ptr<KinodynamicSpace> costSpace;
  std::shared_ptr<ObjectiveFunctionalBase> objective;
  std::shared_ptr<CSet> costGoalSet;
  HeuristicFn heuristic;
  Real costSpaceDistanceWeight;
  bool lazy;

  //keeps track of the best path and cost
  KinodynamicMilestonePath bestPath;
  Real bestPathCost;

  //temp: keeps track of how much of the tree is suboptimal
  int prunableNodeSampleCount;
  int nodeSampleCount;

  //keeps track of some statistics
  int numGoalsSampled,numPrunedNodes;
};

/** @brief An optimizing planner that progressively produces better paths using state-cost space.
 * Plan() returns true only if a better path was found in the last maxIters iterations.  Done() and
 * GetPath() return true if there ANY solution has been found.
 *
 * The method is asymptotically optimal under very general assumptions about the space and objective function.
 *
 * See K. Hauser and Y. Zhou, "Asymptotically-Optimal Kinodynamic Motion Planning in State-Cost Space"
 * IEEE Transactions on Robotics, 2016.
 */
class CostSpaceESTPlanner : public ESTKinodynamicPlanner
{
public:
  typedef Real (*HeuristicFn) (const Config& x);

  CostSpaceESTPlanner(const std::shared_ptr<KinodynamicSpace>& baseSpace,const std::shared_ptr<ObjectiveFunctionalBase>& objective,Real costMax=Inf);
  virtual ~CostSpaceESTPlanner() {}
  void EnableCostSpaceBias(bool enabled);
  void SetCostMax(Real cmax);
  void SetDensityEstimatorResolution(Real res);
  void SetDensityEstimatorResolution(const Vector& res);
  ///A heuristic function is a lower bound on the cost-to-go of a state, given the current objective function.
  ///This function tells the planner to use this heuristic when sampling / pruning.
  void SetHeuristic(HeuristicFn f);
  virtual void Init(const State& xinit,CSet* goalSet);
  virtual bool Plan(int maxIters);
  virtual bool GetPath(KinodynamicMilestonePath& path);
  virtual bool FilterExtension(Node* n,const KinodynamicMilestonePath& path);
  virtual void OnNewBestPath();
  virtual void GetStats(PropertyMap& stats) const;
  void PruneTree();
  Real PathCost() const;
  Real TerminalCost(const State& xc);

  std::shared_ptr<KinodynamicSpace> baseSpace;
  std::shared_ptr<KinodynamicSpace> costSpace;
  std::shared_ptr<ObjectiveFunctionalBase> objective;
  std::shared_ptr<CSet> costGoalSet;
  HeuristicFn heuristic;
  Real costSpaceResolution;

  //keeps track of the best path + cost
  KinodynamicMilestonePath bestPath;
  Real bestPathCost;

  //temp: keeps track of how much of the tree is suboptimal
  int prunableNodeSampleCount;
  int nodeSampleCount;

  //keeps track of some statistics
  int numGoalsSampled,numPrunedNodes;
};



#endif