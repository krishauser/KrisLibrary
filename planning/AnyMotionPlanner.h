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
  virtual void GetRoadmap(RoadmapPlanner& roadmap) {}
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
 * - prm*: the PRM* algorithm for optimal motion planning
 * - lazyprm*: the Lazy-PRM* algorithm for optimal motion planning
 * - rrt*: the RRT* algorithm for optimal motion planning (interface not implemented yet)
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
  int knn;                 //for PRM
  Real connectionThreshold;//for PRM,RRT,SBL,PRM*,LazyPRM*
  bool ignoreConnectedComponents; //for PRM
  Real perturbationRadius; //for Perturbation,EST,RRT,SBL,SBLPRT
  int perturbationIters;   //for SBL
  bool bidirectional;      //for RRT
  bool useGrid;            //for SBL
  Real gridResolution;     //for SBL
  int randomizeFrequency;  //for SBL
  bool storeEdges;         //if local planner data is stored during planning
  bool shortcut;           //if you wish to perform shortcutting afterwards
  bool restart;            //if you wish to restart the planner to get better paths with the remaining time
  string restartTermCond;  //used if restart is true, JSON string defining termination condition
};



/** @brief Helper class for higher-order planners -- passes all calls to another motion planner.
 */
class PiggybackMotionPlanner : public MotionPlannerInterface
{
 public:
  PiggybackMotionPlanner(const SmartPointer<MotionPlannerInterface>& mp);
  virtual int PlanMore() { return mp->PlanMore(); }
  virtual void PlanMore(int numIters) { for(int i=0;i<numIters;i++) PlanMore(); }
  virtual int NumIterations() const { return mp->NumIterations(); }
  virtual int NumMilestones() const { return mp->NumMilestones(); }
  virtual int NumComponents() const { return mp->NumComponents(); }
  virtual bool CanAddMilestone() const { return mp->CanAddMilestone(); }
  virtual int AddMilestone(const Config& q) { return mp->AddMilestone(q); }
  virtual void GetMilestone(int i,Config& q) { return mp->GetMilestone(i,q); }
  virtual void ConnectHint(int m) { mp->ConnectHint(m); }
  virtual bool ConnectHint(int ma,int mb) { return mp->ConnectHint(ma,mb); }
  virtual bool IsConnected(int ma,int mb) const { return mp->IsConnected(ma,mb); }
  virtual bool IsLazy() const { return mp->IsLazy(); }
  virtual bool IsLazyConnected(int ma,int mb) const { return mp->IsLazyConnected(ma,mb); }
  virtual bool CheckPath(int ma,int mb) { return mp->CheckPath(ma,mb); }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { mp->GetPath(ma,mb,path); }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) { mp->GetRoadmap(roadmap); }
  virtual bool IsSolved() { return mp->IsSolved(); }
  virtual void GetSolution(MilestonePath& path) { return mp->GetSolution(path); }

  SmartPointer<MotionPlannerInterface> mp;
};

/** @brief Tries to produce a path between a start node and a goal space.
 * Does so via goal sampling.
 */
class PointToSetMotionPlanner : public PiggybackMotionPlanner
{
 public:
  PointToSetMotionPlanner(const SmartPointer<MotionPlannerInterface>& mp,const Config& qstart,CSpace* goalSpace);
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  virtual int PlanMore();
  virtual bool IsSolved();
  virtual void GetSolution(MilestonePath& path);
  virtual bool SampleGoal(Config& q);
  ///User can add goal nodes manually via this method
  virtual int AddMilestone(const Config& q);

  CSpace* goalSpace;

  ///Setting: the planner samples a new goal configuration every n*(|goalNodes|+1) iterations
  int sampleGoalPeriod;
  ///Incremented each iteration, when it hits sampleGoalPeriod a goal should be sampled
  int sampleGoalCounter;
  ///These are the indices of goal configurations
  vector<int> goalNodes;
};

/** @brief A multiple-restart motion planner that turns a feasible motion planner into an anytime
 * optimal motion planner by keeping the best path found so far.
 */
class RestartMotionPlanner : public PiggybackMotionPlanner
{
 public:
  RestartMotionPlanner(const MotionPlannerFactory& factory,const MotionPlanningProblem& problem,const HaltingCondition& iterTermCond);
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  virtual int PlanMore();
  virtual bool IsSolved() { return !bestPath.edges.empty(); }
  virtual void GetSolution(MilestonePath& path) { path = bestPath; }
  virtual int NumIterations() const { return numIters; }

  MotionPlannerFactory factory;
  MotionPlanningProblem problem;
  HaltingCondition iterTermCond;
  MilestonePath bestPath;
  int numIters;
};

/** @brief Plans a path and then tries to shortcut it with the remaining time.
 */
class ShortcutMotionPlanner : public PiggybackMotionPlanner
{
 public:
  ShortcutMotionPlanner(const SmartPointer<MotionPlannerInterface>& mp);
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  virtual int PlanMore();
  virtual bool IsSolved() { return !bestPath.edges.empty(); }
  virtual void GetSolution(MilestonePath& path) { path = bestPath; }
  virtual int NumIterations() const { return numIters; }

  MilestonePath bestPath;
  int numIters;
};



#endif
