#ifndef ANY_MOTION_PLANNER_H
#define ANY_MOTION_PLANNER_H

#include "MotionPlanner.h"

class TiXmlElement;

//forward declarations
class HaltingCondition;
class MotionPlanningProblem;
class MotionPlannerInterface;
class PointToSetMotionPlanner;
class MotionPlannerFactory;
class PropertyMap;

/** @ingroup MotionPlanning
 * @brief A termination condition for a planner.  Supports max iteration count,
 * time limit, absolute cost, and cost improvement conditions.
 */
class HaltingCondition
{
 public:
  HaltingCondition();
  ///Load settings from JSON string
  bool LoadJSON(const string& str);
  ///Save settings to JSON string
  string SaveJSON() const;

  ///Stop on the first solution found
  bool foundSolution;
  ///Stop when this number of iterations have been computed (default 1000)
  int maxIters;
  ///Stop when this time limit (in seconds) has expired (default Inf)
  Real timeLimit;
  ///Stop when the solution cost decreases below the threshold
  Real costThreshold;
  ///Stop when the solution cost improvement over the given period of time decreases below the threshold (default inactive)
  Real costImprovementPeriod;
  Real costImprovementThreshold;
};

/** @brief A structure to specify the space, endpoints, and cost function of a 
 * motion planning problem.
 */
class MotionPlanningProblem
{
 public:
  MotionPlanningProblem();
  ///Create a point-to-point problem
  MotionPlanningProblem(CSpace* space,const Config& a,const Config& b);
  ///Create a point-to-set problem
  MotionPlanningProblem(CSpace* space,const Config& a,CSpace* goalSet);
  ///Create a set-to-set problem
  MotionPlanningProblem(CSpace* space,CSpace* startSet,CSpace* goalSet);
  ///Create an initial value problem (placeholder -- objectives not done yet)
  MotionPlanningProblem(CSpace* space,const Config& a,void* objective);

  CSpace* space;
  ///Non-empty if the start/end point is given
  Config qstart,qgoal;
  ///Non-NULL if the start/end point must be in a given set
  CSpace *startSet, *goalSet;
  ///Placeholder -- objectives not done yet
  void* objective;
};

/** @ingroup MotionPlanning
 * @brief An abstract class for a sample-based motion planner. 
 *
 * Standard single-query, feasible motion planning usage is to create
 * a MotionPlannerInterface using
 * a MotionPlannerFactory, then add the start and goal milestones
 * (which have id's 0 and 1, respectively).  Then, Plan(path,cond)
 * is called, with an appropriate termination condition.
 *
 * Execution can be controlled manually, e.g., call PlanMore() 
 * until IsSolved() returns true, at which point the path is
 * retrieved using GetSolution(p).
 *
 * Single-query, optimal motion planning usage is the same as Plan(), except
 * HaltingCondition.foundSolution should be set to false, and termination is
 * determined by the desired number of iterations, time limit, or cost
 * improvement.
 */
class MotionPlannerInterface
{
 public:
  MotionPlannerInterface() {}
  virtual ~MotionPlannerInterface() {}
  ///Plans until a given termination condition holds true, returns the reason for termination
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  ///Performs a planning unit and returns the added milestone
  virtual int PlanMore()=0;
  ///Performs numIters planning units
  virtual void PlanMore(int numIters) { for(int i=0;i<numIters;i++) PlanMore(); }
  ///Returns the number of elaped planning units
  virtual int NumIterations() const=0;
  ///Returns the number of milestones stored by the planner
  virtual int NumMilestones() const=0;
  ///Returns the number of connected components stored by the planner
  virtual int NumComponents() const=0;
  ///Returns true if a milestone can currently be added
  virtual bool CanAddMilestone() const { return false; }
  ///Adds a milestone, if possible.  Returns an id of the milestone.
  virtual int AddMilestone(const Config& q)=0;
  ///For the id of a milestone previously added, returns its configuration
  virtual void GetMilestone(int,Config& q)=0;
  ///Manual indication that the milestone is a good candidate for connecting
  virtual void ConnectHint(int m) { }
  ///Manual indication that the milestones are good candidates for connecting
  virtual bool ConnectHint(int ma,int mb) { return false; }
  ///Returns true if the two milestones are connected with a feasible path
  virtual bool IsConnected(int ma,int mb) const=0;
  ///Returns true if this planner has lazy semantics
  virtual bool IsLazy() const { return false; }
  ///If lazy semantics are used, returns true if the two milestones are
  ///connected by a likely feasible path
  virtual bool IsLazyConnected(int ma,int mb) const { return IsConnected(ma,mb); }
  ///If lazy semantics are used, this will check for a feasible path
  /// between two lazy-connected milestones
  virtual bool CheckPath(int ma,int mb) { return false; }
  ///Returns a feasible path between two connected milestones
  virtual void GetPath(int ma,int mb,MilestonePath& path)=0;
  ///For single-query planners, returns true if the start and goal are connected
  virtual bool IsSolved() { return IsConnected(0,1); }
  ///For single-query planners, returns the solution path
  virtual void GetSolution(MilestonePath& path) { return GetPath(0,1,path); }
  ///Returns a full-blown roadmap representation of the roadmap
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const {}
  ///Returns some named statistics about the planner, implementation-dependent
  virtual void GetStats(PropertyMap& stats) const;
};

/** @brief A motion planner creator.
 * 
 * User calls Create(space) to create a planner. After Create, the planner pointer must be deleted.
 * 
 * The type field can be left as "any", in which a default planning algorithm will be
 * used. Otherwise, a given planner algorithm can be designated as follows:
 * - prm: the Probabilistic Roadmap algorithm
 * - lazyprm: the Lazy-PRM algorithm (interface not implemented yet)
 * - perturbation: the PerturbationTree algorithm (interface not implemented yet)
 * - est: the Expanding Space Trees algorithm (interface not implemented yet)
 * - rrt: the Rapidly Exploring Random Trees algorithm
 * - sbl: the Single-Query Bidirectional Lazy planner
 * - sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as the inter-root planner.
 * - rrt*: the RRT* algorithm for optimal motion planning
 * - prm*: the PRM* algorithm for optimal motion planning
 * - lazyprm*: the Lazy-PRM* algorithm for optimal motion planning
 * - lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning
 * - fmm: the fast marching method algorithm for resolution-complete optimal motion planning
 * - fmm*: an anytime fast marching method algorithm for optimal motion planning
 *
 * Multi-query planners include PRM and SBLPRT. 
 *
 * The only cost function supported for optimal motion planners is the path length function.
 */
class MotionPlannerFactory
{
 public:
  MotionPlannerFactory();
  virtual ~MotionPlannerFactory() {}
  ///Make a motion planner for a given problem
  virtual MotionPlannerInterface* Create(const MotionPlanningProblem& problem);
  ///Make a motion planner
  virtual MotionPlannerInterface* Create(CSpace* space);
  ///Make a point-to-point motion planner (start is milestone 0 and goal is milestone 1)
  virtual MotionPlannerInterface* Create(CSpace* space,const Config& a,const Config& b);
  ///Make a point-to-set motion planner
  virtual PointToSetMotionPlanner* Create(CSpace* space,const Config& a,CSpace* goalSet);
  ///Helper: make a motion planner without shortcut / restart modifiers
  virtual MotionPlannerInterface* CreateRaw(CSpace* space);
  ///Helper: apply shortcut / restart modifiers to a given planner interface
  virtual MotionPlannerInterface* ApplyModifiers(MotionPlannerInterface*,const MotionPlanningProblem& problem);
  ///Load settings from XML
  bool Load(TiXmlElement* e);
  ///Save settings to XML
  bool Save(TiXmlElement* e);
  ///Load settings from JSON string
  bool LoadJSON(const string& str);
  ///Save settings to JSON string
  string SaveJSON() const;

  string type;
  int knn;                 //for PRM (default 10)
  Real connectionThreshold;//for PRM,RRT,SBL,SBLPRT,RRT*,PRM*,LazyPRM*,LazyRRG* (default Inf)
  Real suboptimalityFactor;//for RRT*, LazyPRM*, LazyRRG* (default 0)
  bool ignoreConnectedComponents; //for PRM (default false)
  Real perturbationRadius; //for Perturbation,EST,RRT,SBL,SBLPRT (default 0.1)
  int perturbationIters;   //for SBL (default 5)
  bool bidirectional;      //for RRT (default true)
  bool useGrid;            //for SBL, SBLPRT (default true): for SBL, uses grid-based random point selection
  Real gridResolution;     //for SBL, SBLPRT, FMM, FMM* (default 0): if nonzero, for SBL, specifies point selection grid size (default 0.1), for FMM / FMM*, specifies resolution (default 1/8 of domain)
  int randomizeFrequency;  //for SBL, SBLPRT (default 50): how often the grid projection is randomly perturbed
  string pointLocation;    //for PRM, RRT*, PRM*, LazyPRM*, LazyRRG* (default ""): specifies a point location data structure ("random", "randombest [k]", "kdtree" supported)
  bool storeEdges;         //if local planner data is stored during planning (false may save memory, default)
  bool shortcut;           //if you wish to perform shortcutting afterwards (default false)
  bool restart;            //if you wish to restart the planner to get better paths with the remaining time (default false)
  string restartTermCond;  //used if restart is true, JSON string defining termination condition (default "{foundSolution:1;maxIters:1000}")
};



#endif
