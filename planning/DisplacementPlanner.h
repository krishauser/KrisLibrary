#ifndef DISPLACEMENT_PLANNER_H
#define DISPLACEMENT_PLANNER_H

#include <KrisLibrary/Logger.h>
#include "MotionPlanner.h"
#include <KrisLibrary/utils/Subset.h>
#include <KrisLibrary/structs/IndexedPriorityQueue.h>

/** @brief A class that needs to be subclassed in order to implement a
 * minimum constraint displacement problem.
 *
 * Each displaceable constraint accepts a displacement parameter d[i] which
 * lies in a space Di.  Di must be returned via the method DisplacementSpace
 * that should be overridden by the subclass.  If displaceable, the subclass
 * should allocate a new CSpace via new, otherwise it should return NULL. 
 * Di should return a displacement cost in the Distance(a,b) method
 * (when called by the planner, b will always be zero).
 *
 * The space stores a current displacement in obstacleDisplacements. 
 * These are initialized to the zero vector.
 * 
 * Subclasses should override IsFeasible(q,i,d).  They should also override
 * IsFeasibleAll(q,i) if a bound can be placed on the span of all obstacles
 * instantiated over valid displacements (i.e., if IsFeasibleAll(q,i) is true,
 * then IsFeasible(q,i,d) should return true for all feasible d).
 *
 * If the subclass uses special local planners that are not simply based on
 * discretization, then the EdgePlanner should also use the current setting of
 * obstacleDisplacements when checking visibility.
 * 
 * Besides these methods, the subclass also needs to override the CSpace method
 * Sample(), the ExplicitCSpace NumObstacles() and LocalPlanner() method, 
 * and optionally the CSpace Distance(), Interpolate(), and Midpoint() methods.
 */
class ObstacleDisplacementCSpace : public CSpace 
{
public:
  ObstacleDisplacementCSpace();
  virtual ~ObstacleDisplacementCSpace() {}

  //these should be implemented by the subclass
  virtual std::shared_ptr<CSpace> DisplacementSpace(int obstacle) const { return NULL; }
  virtual bool IsFeasible(const Config& q,int obstacle,const Vector& d)=0;
  virtual bool IsFeasibleAll(const Config& q,int obstacle) { return false; }
  virtual bool IsVisibleAll(const Config& a,const Config& b,int obstacle) { return false; }

  virtual void SetDisplacement(int obstacle,const Vector& d);
  virtual void GetDisplacement(int obstacle,Vector& d) const;
  virtual void SetDisplacementRef(int obstacle,const Vector& d);
  virtual void GetDisplacementRef(int obstacle,Vector& d) const;
  bool IsDisplaceable(int obstacle) const;

  //overriding ExplicitCSpace's method
  virtual bool IsFeasible(const Config& q,int obstacle);

  //re-initializes all displacements to 0, gets the displacementSpaeces
  void InitZeroDisplacements();

  std::vector<Vector> obstacleDisplacements;
  std::vector<std::shared_ptr<CSpace> > displacementSpaces;
};

/** @brief A planner that minimizes the the displacement cost of violated
 * constraints using a RRT-like strategy.
 * 
 * Usage:
 *   //first, set up an ObstacleDisplacementCSpace cspace.
 *   DisplacementPlanner planner(&cspace);
 *   planner.Init(start,goal);
 *   
 *   //do planning with a given expansion schedule
 *   vector<Real> limits;
 *   vector<int> numIters;
 *   limits.push_back(limit1);  numIters.push_back(100);
 *     ...
 *   limits.push_back(limitN);  numIters.push_back(100);
 *   vector<int> assignment;
 *   if(planner.Plan(limits,numIters,bestPlan,assignment))
 *      LOG4CXX_INFO(KrisLibrary::logger(),"Success\n");
 *
 * Plan will keep going until all iterations are exhausted.  It will use the given
 * limit until the goal is reached, then it will use the existing cost to goal
 * as the limit in the future.
 *
 * More detailed
 * manual control (e.g., breaking on first path) is possible through calling:
 * - Expand to grow the roadmap
 * - AddNewDisplacement to generate a new displacement sample
 * - Test whether the goal is reached by calling
 *   bool success = (OptimalPathTo(1)!=NULL);
 * - GetMilestonePath(OptimalPathTo(1),path) to extract the path to the goal
 */
class DisplacementPlanner
{
 public:
  ///stores all values for a single obstacle displacement test
  struct TestResult
  {
    int allFeasible;  //0 = false, 1 = true, -1 = unknown
    Subset feasible;
    Subset infeasible;
  };
  typedef std::vector<TestResult> TestResults;

  struct Milestone {
    Config q;
    TestResults tests;
  };
  struct Edge {
    EdgePlannerPtr e;
    TestResults tests;
  };
  typedef Graph::UndirectedGraph<Milestone,Edge> Roadmap;

  struct PathSearchNode
  {
    int vertex;
    std::vector<int> assignment;
    double pathLength,totalCost;
    PathSearchNode* parent;
  };

  DisplacementPlanner(ObstacleDisplacementCSpace* space);
  void Init(const Config& start,const Config& goal);
  ///Performs bottom-up planning according to a given set of parameters
  ///(see publication)
  bool Plan(int numIters,int numExpandsPerDisp,int numLocalOptimize,Real expandLimitStep,std::vector<int>& bestPath,std::vector<int>& bestDisplacements);
  ///Performs one iteration of planning given a limit on the solution cost
  void Expand(Real maxTotalCost,std::vector<int>& newNodes);
  ///Picks an obstacle to sample and samples a displacement, returns -1 if
  ///failed
  int AddNewDisplacement(Real maxTotalCost = Inf);
  ///Picks an obstacle to sample, and a maximum cost bound for the
  ///displacement, based on the current roadmap.
  ///Default implementation picks whether to explore or refine current paths.
  virtual std::pair<int,Real> PickObstacleToSample(Real maxTotalCost);
  ///The explore strategy finds a node that is not expanded and finds
  ///the obstacles that might enable the node to be expanded
  virtual std::pair<int,Real> PickExploreObstacle(Real maxTotalCost);
  ///The goal explore strategy looks for candidate edges into the goal and
  ///picks among obstacles overlapping those edges
  virtual std::pair<int,Real> PickGoalExploreObstacle(Real maxTotalCost);
  ///The refine strategy picks among obstacles that must be moved along
  ///current paths (over the whole roadmap)
  virtual std::pair<int,Real> PickRefineObstacle(Real maxTotalCost);
  ///The refine strategy picks among obstacles that must be moved along
  ///the current path to the goal
  virtual std::pair<int,Real> PickGoalRefineObstacle(Real maxTotalCost);
  ///Returns true if this node is an unexplored candidate
  bool IsCandidateForExploration(int i) const;
  ///Outputs the graph with the given explanation limit
  void BuildRoadmap(Real maxExplanationCost,RoadmapPlanner& prm);

  //helpers
  double Cost(const std::vector<int>& assignment) const;

  int AddNode(const Config& q,int parent=-1);
  void AddEdge(int i,int j);
  void AddEdge(int i,int j,const TestResults& tests);
  int CheckImmovableAndAddNode(const Config& q,int parent=-1);
  int ExtendEdge(int i,const Config& q);  //returns index of q
  void KNN(const Config& q,int k,std::vector<int>& neighbors,std::vector<Real>& distances);
  void KNN(const Config& q,Real maxTotalCost,int k,std::vector<int>& neighbors,std::vector<Real>& distances);
  bool UpdateCoversIn(int nstart,Real maxTotalCost);
  void UpdateCoversOut(int nstart,Real maxTotalCost);
  std::shared_ptr<PathSearchNode> UpdateEdge(int i,int j,PathSearchNode* ni,Real maxTotalCost);
  bool CheckUpstreamConstraints(PathSearchNode* n,int constraint,int sample);
  void PropagateDownstream(PathSearchNode* n,std::vector<std::shared_ptr<PathSearchNode> >& nodes,Real maxTotalCost);
  bool FindMinimumAssignment(PathSearchNode* n,Real maxTotalCost);
  bool CheckNodeConstraint(Milestone& v,int constraint,int sample);
  bool CheckEdgeConstraint(Edge& e,int constraint,int sample);
  Real OptimalCost(int i) const;
  PathSearchNode* OptimalPathTo(int i);
  void PruneSearchNode(PathSearchNode* n,IndexedPriorityQueue<PathSearchNode*,Real>* q=NULL);
  bool Revisited(PathSearchNode* n);
  ///returns true if a dominates b, either in the total cost sense
  ///(greedy) or pareto sense (optimal)
  bool Dominates(PathSearchNode* a,PathSearchNode* b);
  //returns true if all of the displacements of setting b are at least as
  //costly as the corresponding displacements of assignment a
  bool ParetoDominates(const std::vector<int>& a,const std::vector<int>& b) const;

  ///For a given graph node, returns the list of obstacles that would improve
  ///the optimal path to that node if it were removed.
  void LocalImprovementCandidates(int i,std::vector<int>& obstacles);
  ///Generates a new displacement sample that has the potential to improve paths
  ///to nodes that currently collide with the given obstacle.  The displacement
  ///with lowest cost is picked out of the best of numTries attempts
  bool GenerateDisplacementSample(int obstacle, Real maxDispCost, Real maxTotalCost, int numTries);
  ///Adds a new displacement sample, if it could possibly affect the optimal path
  ///to any node.  Returns false if it is irrelevant
  bool AddDisplacementSample(int obstacle,const Vector& disp);
  ///Adds a new displacement sample, without checking or updating paths
  void AddDisplacementSampleRaw(int obstacle,const Vector& disp);
  ///Performs local optimization of the displacements along the goal path. 
  ///Sample-based descent technique
  ///that at each step perturbs the optimum by r*perturbRadiusFrac where r is the
  ///current deviation from the zero displacement.
  ///Returns true if a better solution is found.
  bool RefineGoalDisplacements(int numIters,Real perturbRadiusFrac=1.0);
  ///Performs local optimization of the displacements and the goal path. 
  ///Sample-based descent technique that at each step perturbs the optimum
  ///displacement by r*perturbRadiusFrac where r is the
  ///current deviation from the zero displacement, and then modifies the path 
  ///to achieve that displacement.  Paths are perturbed by radius
  ///lipschitzDisp*r*perturbRadiusFrac, where lipschitzDisp is a lipschitz
  ///constant on how much a C-obstacle moves with a unit displacement.
  ///Returns true if a better solution is found.
  bool RefineGoalPathAndDisplacements(int numIters,Real perturbRadiusFrac=1.0,Real lipschitzDisp=1.0);
  ///Performs local optimization of the path to the goal via shortcutting.
  ///This version considers skip-node shortcutting
  bool ShortcutGoalPath(int skip,int numIters);

  ///Returns the path from the start leading to this node 
  void GetPath(PathSearchNode* n,std::vector<int>& path) const;
  ///Returns the MilestonePath corresponding to a graph path
  void GetMilestonePath(const std::vector<int>& path,MilestonePath& mpath) const;
  ///Returns the MilestonePath leading up to a given PathSearchNode
  ///e.g., GetMilestonePath(OptimalPathTo(1),path) returns the optimal path to
  ///the goal.
  void GetMilestonePath(PathSearchNode* n,MilestonePath& mpath) const;

  ///Do basic sanity checks with the data structures
  bool SanityCheck(bool checkOneCoverGreedy=false);
 
  Config start,goal;
  ObstacleDisplacementCSpace* space;

  //minimize pathCostWeight*length(path) + sum of displacement costs
  //if initialDisplacementCosts is nonempty and displacement i is nonzero,
  //it incurs an extra cost of initialDisplacementCosts[i].
  std::vector<Real> initialDisplacementCosts;
  Real pathCostWeight; 

  //settings for RRT* like expansion
  bool dynamicDomainExpansion;
  int numConnections;
  Real connectThreshold,expandDistance,goalConnectThreshold;
  Real goalBiasProbability;  //probability of expanding toward the goal
  bool bidirectional;        //not functional yet
  
  //settings for path update
  ///If true: use the slower complete, exact cover update.
  ///If false: use the faster greedy one.
  bool updatePathsComplete;
  ///For complete planning, keep at most this number of covers 
  int updatePathsMax;

  //settings for obstacle sampling
  int obstacleSampleCount,obstacleDescendIters;

  Roadmap roadmap;
  std::vector<std::vector<Vector> > displacementSamples;
  std::vector<std::vector<Real> > displacementSampleCosts;
  std::vector<std::vector<int> > displacementSampleOrders;

  struct DynamicShortestPathNode
  {
    //store either a single cover (in case of greedy search)
    //or multiple pareto-optimal covers (in case of optimal search)
    std::vector<std::shared_ptr<PathSearchNode> > covers;
  };
  std::vector<DynamicShortestPathNode> pathCovers;

  //planning statistics
  int numExpands,numRefinementAttempts,numRefinementSuccesses,numExplorationAttempts,
    numEdgeChecks,numConfigChecks,
    numUpdateCovers,numUpdateCoversIterations;
  double timeNearestNeighbors,timeRefine,timeExplore,timeUpdateCoversIn,timeUpdateCoversOut,timeOverhead;
};

#endif
