#ifndef ANY_MOTION_PLANNER_H
#define ANY_MOTION_PLANNER_H

#include "MotionPlanner.h"

class TiXmlElement;

/** @brief An abstract class for a sample-based motion planner. 
 *
 * Standard single-query usage is to create a MotionPlannerInterface using
 * a MotionPlannerFactory, then add the start and goal milestones
 * (which have id's 0 and 1, respectively).  Then PlanMore() is called
 * until IsConnected(0,1) returns true, at which point the path is
 * retrieved using GetPath(0,1,p).
 */
class MotionPlannerInterface
{
 public:
  MotionPlannerInterface() {}
  virtual ~MotionPlannerInterface() {}
  ///Performs a planning unit
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
  ///Returns a full-blown roadmap representation of the roadmap
  virtual void GetRoadmap(RoadmapPlanner& roadmap) {}
};

class MotionPlannerFactory
{
 public:
  enum Type { PRM,LazyPRM,PerturbationTree,EST,RRT,SBL,SBLPRT};

  MotionPlannerFactory();
  virtual ~MotionPlannerFactory() {}
  virtual MotionPlannerInterface* Create(CSpace* space);
  bool Load(TiXmlElement* e);
  bool Save(TiXmlElement* e);

  Type type;
  int knn;                 //for PRM
  Real connectionThreshold;//for PRM,RRT,SBL
  bool ignoreConnectedComponents; //for PRM
  Real perturbationRadius; //for Perturbation,EST,RRT,SBL,SBLPRT
  int perturbationIters;   //for SBL
  bool bidirectional;      //for RRT
  bool useGrid;            //for SBL
  Real gridResolution;     //for SBL
  int randomizeFrequency;  //for SBL
  bool storeEdges;         //if local planner data is stored during planning
};


#endif
