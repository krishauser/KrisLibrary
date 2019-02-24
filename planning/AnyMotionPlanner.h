#ifndef ANY_MOTION_PLANNER_H
#define ANY_MOTION_PLANNER_H

#include <KrisLibrary/Logger.h>
#include "MotionPlanner.h"

class TiXmlElement;

//forward declarations
class HaltingCondition;
class MotionPlanningProblem;
class MotionPlannerInterface;
class MotionPlannerFactory;
class PropertyMap;

/** @ingroup MotionPlanning
 * @brief A termination condition for a planner.  Supports max iteration count,
 * time limit, absolute cost, and cost improvement conditions.
 *
 * Can be converted to/from JSON strings.  The format is very simple, simply
 * a dictionary whose keys are the names of the attributes of this structure.
 */
class HaltingCondition
{
 public:
  HaltingCondition();
  ///Load settings from JSON string
  bool LoadJSON(const std::string& str);
  ///Save settings to JSON string
  std::string SaveJSON() const;

  ///Stop on the first solution found
  bool foundSolution;
  ///Stop when this number of iterations have been computed (default 1000)
  int maxIters;
  ///Stop when this time limit (in seconds) has expired (default Inf)
  Real timeLimit;
  ///Stop when the solution cost decreases below the threshold (default 0)
  Real costThreshold;
  ///Stop when the solution cost improvement over the given period of time decreases below costImprovementThreshold (default inactive)
  Real costImprovementPeriod;
  Real costImprovementThreshold;
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
 * the halting condition must be changed:
 * - HaltingCondition.foundSolution must be set to false
 * - Set a termination criterion to the desired number of iterations
 *   (HaltingCondition.maxIters), time limit (HaltingCondition.timeLimit),
 *   or cost improvement (HaltingCondition.costThreshold  or
 *   costImprovementPeriod and costImprovementThreshold).
 */
class MotionPlannerInterface
{
 public:
  typedef Graph::UndirectedGraph<Config,std::shared_ptr<EdgePlanner> > Roadmap;

  MotionPlannerInterface() {}
  virtual ~MotionPlannerInterface() {}
  ///Plans until a given termination condition holds true, returns the
  ///reason for termination.  The return string will be the attribute of
  ///HaltingCondition that caused termination.
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
  ///Returns true if this planner is exclusively point-to-point, i.e., cannot accept
  ///additional goal nodes via AddMilestone()
  virtual bool IsPointToPoint() const { return true; }
  ///Returns true if this planner can optimize the path after the first solution
  virtual bool IsOptimizing() const { return false; }
  ///Returns true if this planner can handle an objective function (in SetObjective
  ///and GetOptimalPath)
  virtual bool CanUseObjective() const { return false; }
  ///Must be implemented if CanUseObjective() = true
  virtual void SetObjective(std::shared_ptr<ObjectiveFunctionalBase> obj) {}
  ///Returns true if this planner has lazy semantics
  virtual bool IsLazy() const { return false; }
  ///If lazy semantics are used, returns true if the two milestones are
  ///connected by a likely feasible path
  ///
  ///Must be implemented if IsLazy()=true
  virtual bool IsLazyConnected(int ma,int mb) const { return IsConnected(ma,mb); }
  ///If lazy semantics are used, this will check for a feasible path
  /// between two lazy-connected milestones
  ///
  ///Must be implemented if IsLazy()=true
  virtual bool CheckPath(int ma,int mb) { return false; }
  ///Retrieve the index of a close milestone
  virtual int GetClosestMilestone(const Config& q);
  ///Returns a feasible path between two connected milestones.  If
  ///IsConnected(ma,mb)=false, this will abort.
  ///
  ///Must be implemented if IsLazy()=true
  virtual void GetPath(int ma,int mb,MilestonePath& path)=0;
  ///Calculates a feasible, minimum cost path, starting at ma and terminating at some node 
  ///in the mb set.  If SetObjective was called before, the objective function is
  ///used as the cost.  Otherwise, the cost is up to the planner, but is typically path length.
  ///The cost is returned.
  ///
  ///Must be implemented if CanUseObjective() = true.
  virtual Real GetOptimalPath(int ma,const std::vector<int>& mb,MilestonePath& path) { return Inf; }
  ///For single-query planners, returns true if the start and goal are connected
  virtual bool IsSolved() { return IsConnected(0,1); }
  ///For single-query planners (IsPointToPoint()=true), returns the solution path.
  ///
  ///For multi-query planners, returns the optimal solution path.
  virtual void GetSolution(MilestonePath& path) { return GetPath(0,1,path); }
  ///Returns a full-blown roadmap representation of the roadmap
  virtual void GetRoadmap(Roadmap& roadmap) const {}
  ///Returns some named statistics about the planner, implementation-dependent
  virtual void GetStats(PropertyMap& stats) const;
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
  MotionPlanningProblem(CSpace* space,const Config& a,CSet* goalSet);
  ///Create a set-to-set problem
  MotionPlanningProblem(CSpace* space,CSet* startSet,CSet* goalSet);
  ///Create an initial value problem
  MotionPlanningProblem(CSpace* space,const Config& a,std::shared_ptr<ObjectiveFunctionalBase> objective);

  CSpace* space;
  ///Non-empty if the start/end point is given
  Config qstart,qgoal;
  ///Non-NULL if the start/end point must be in a given set
  CSet *startSet, *goalSet;
  ///Non-NULL if you'd like to optimize some objective function
  std::shared_ptr<ObjectiveFunctionalBase> objective;
};

/** @brief A motion planner creator.
 * 
 * Standard usage is as follows:
 * - Set up the factory's parameters, either loading by hand or from
 *   JSON strings.
 * - Set up your CSpace / MotionPlanningProblem as needed.
 * - Create a planner via "MotionPlannerInterface* planner =
 *   MotionPlannerFactory.Create(space,[start],[goal])", or "Create(problem)"
 * - Set up your HaltingCondition.
 * - Call planner->Plan(path,condition) and inspect the resulting path.
 * - Delete the planner object
 * 
 * Example (assuming you have a space, start, and goal):
 *
 * @verbatim
 * MotionPlanningFactory factory;
 * factory.type = "any";  //do more setup of parameters here
 * //an alternative way of setting up the parameters is as follows
 * //factory.LoadJSON("{ type:\"rrt\", perturbationRadius:0.5, shortcut:1, restart=1 }");
 * MotionPlannerInterface* planner = factory.Create(space,start,goal);
 *
 * //set up a condition to plan for 10s
 * HaltingCondition cond;
 * cond.foundSolution=false;
 * cond.timeLimit = 10;
 *
 * //do the planning
 * MilestonePath path;
 * string res = planner->Plan(path,cond);
 * if(path.edges.empty())  //failed
 *   LOG4CXX_INFO(KrisLibrary::logger(),"Planning failed\n");
 * else
 *   LOG4CXX_INFO(KrisLibrary::logger(),"Planning succeeded, path has length "<<path.Length());
 *
 * //clean up the planner
 * delete planner;
 * @endverbatim
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
 * If KrisLibrary is built with OMPL support, you can also use the type specifier
 * "ompl:[X]" where [X] is one of:
 * - prm, lazyprm, prm*, lazyprm*, spars
 * - rrt, rrtconnect, birrt, lazyrrt, lbtrrt, rrt*, informedrrt*
 * - est, fmt, sbl, stride
 * The appropriate OMPL planner will be created for that given type, and MotionPlannerFactory
 * parameters will be mapped as closely as possible to the OMPL parameters.
 * (tested with OMPL 1.1.0)
 *
 * Multi-query planners include PRM and SBLPRT. 
 *
 * The only cost function currently supported for optimal motion planners
 * is the path length cost.
 *
 * Standard kinematic planners (e.g., prm,lazyprm,perturbation,est,rrt,sbl,
 * sblprt,fmm) can be adapted into to optimal planners either by a
 * shortcutting technique -- where the path is repeatedy shortened -- or by
 * a random-restart technique -- in which the planner will repeatedly
 * try to find a better solution.  Specifically:
 * - With shortcut=true, the path produced by the planner is shrunk
 *   in a postprocessing stage by repeatedly shortcutting until the
 *   remaining time is up.
 * - With restart=true, planning proceeds in rounds, each round completely
 *   restarting from scratch, and only the best path is stored.
 *   To govern how long each of the rounds lasts, you must set the
 *   restartTermCond field to a JSON string defining the HaltingCondition
 *   for that round.
 * - Good results are often obtained by setting both restart=true and
*    shortcut=true. 
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
  virtual MotionPlannerInterface* Create(CSpace* space,const Config& a,CSet* goalSet);
  ///Helper: make a motion planner without shortcut / restart modifiers
  virtual MotionPlannerInterface* CreateRaw(CSpace* space);
  ///Helper: apply shortcut / restart modifiers to a given planner interface, deleting the prior pointer if necessary
  virtual MotionPlannerInterface* ApplyModifiers(MotionPlannerInterface*,const MotionPlanningProblem& problem);
  ///Load settings from XML
  bool Load(TiXmlElement* e);
  ///Save settings to XML
  bool Save(TiXmlElement* e);
  ///Load settings from JSON string
  bool LoadJSON(const std::string& str);
  ///Save settings to JSON string
  std::string SaveJSON() const;

  std::string type;
  int knn;                 ///<for PRM (default 10)
  Real connectionThreshold;///<for PRM,RRT,SBL,SBLPRT,RRT*,PRM*,LazyPRM*,LazyRRG* (default Inf)
  Real suboptimalityFactor;///<for RRT*, LazyPRM*, LazyRRG* (default 0)
  bool ignoreConnectedComponents; //for PRM (default false)
  Real perturbationRadius; ///<for Perturbation,EST,RRT,SBL,SBLPRT (default 0.1)
  int perturbationIters;   ///<for SBL (default 5)
  bool bidirectional;      ///<for RRT (default true)
  bool useGrid;            ///<for SBL, SBLPRT (default true): for SBL, uses grid-based random point selection
  Real gridResolution;     ///<for SBL, SBLPRT, FMM, FMM* (default 0): if nonzero, for SBL, specifies point selection grid size (default 0.1), for FMM / FMM*, specifies resolution (default 1/8 of domain)
  int randomizeFrequency;  ///<for SBL, SBLPRT (default 50): how often the grid projection is randomly perturbed
  std::string pointLocation;    ///<for PRM, RRT*, PRM*, LazyPRM*, LazyRRG* (default ""): specifies a point location data structure ("random", "randombest [k]", "kdtree", "balltree" supported)
  bool storeEdges;         ///<true if local planner data is stored during planning (false may save memory, default)
  bool shortcut;           ///<true if you wish to perform shortcutting afterwards (default false)
  bool restart;            ///<true if you wish to restart the planner to get better paths with the remaining time (default false)
  std::string restartTermCond;  ///<used if restart is true, JSON string defining termination condition (default "{foundSolution:1;maxIters:1000}")
};



#endif
