#ifndef MULTI_MODAL_PLANNER_H
#define MULTI_MODAL_PLANNER_H

#include "MultiModalCSpace.h"
#include "AnyMotionPlanner.h"
#include "SBL.h"
#include <KrisLibrary/graph/ShortestPaths.h>
#include <KrisLibrary/structs/IndexedPriorityQueue.h>
#include <map>

class MultiModalPRM
{
 public:
  MultiModalPRM(MultiModalCSpace<int>* cspace);
  void InitializeExplicit(ExplicitMMCSpace* cspace);
  void SetStart(const Config& q,int mode);
  void SetGoal(const Config& q,int mode);
  void SetStartMode(int mode);
  void SetGoalMode(int mode);
  void ExpandAll();
  void ExpandMode(int mode,int numSamples=-1);
  void ExpandTrans(int m1,int m2,int numSamples=-1);
  void AddTransition(int m1,int m2,const Config& q);
  bool IsStartAndGoalConnected() const;
  
  struct ModeInfo
  {
    std::shared_ptr<MotionPlannerInterface> planner;
    int sampleCount;
  };

  struct TransitionInfo
  {
    int sampleCount;
    std::vector<Config> transitions;
    //index of transition, in roadmap with the lower index
    std::vector<int> prevRoadmapIndices;
    //index of transition, in roadmap with the higher index
    std::vector<int> nextRoadmapIndices;
  };

  struct TransitionIndex
  {
    inline bool operator < (const TransitionIndex& t) const {
      if(m1 < t.m1) return true;
      else if(m1 > t.m1) return false;
      if(m2 < t.m2) return true;
      else if(m2 > t.m2) return false;
      return count < t.count;
    }
    int m1,m2;
    int count;
  };

  //helpers
  TransitionIndex NodeToTransitionIndex(int mode,int roadmapIndex) const;
  void ConnectTransitions(int mode,int t1,int t2);
  void ConnectTransitions(const TransitionIndex& t1,const TransitionIndex& t2);
  void GetTransitionNodes(int mode,std::vector<TransitionIndex>& transitions,std::vector<int>& roadmapIndices) const;

  typedef Graph::UndirectedGraph<ModeInfo,TransitionInfo> PlanningGraph;
  MultiModalCSpace<int>* space;
  PlanningGraph planningGraph;
  int startMode,startRoadmapIndex;
  int goalMode,goalRoadmapIndex;
  int numExpandModeSamples,numExpandTransSamples;
  MotionPlannerFactory plannerFactory;  //for single mode planning

  //transition indices into the connected component graph
  //start/goal config are virtual transitions indexed by (-1,startMode,0)
  //and (goalMode,-1,0), respectively
  std::map<TransitionIndex,int> ccIndex;
  Graph::ConnectedComponents ccs;
  int modeSampleCount;
  int transSampleCount;
};

class IncrementalMMPRM_Search;

struct UpdatePrioritySPP : public Graph::ShortestPathProblem<int,MultiModalPRM::TransitionInfo*>
{
  UpdatePrioritySPP(IncrementalMMPRM_Search& _g);
  virtual void OnDistanceUpdate(int n);

  IncrementalMMPRM_Search& g;
};


class IncrementalMMPRM_Search
{
 public:
  virtual double EdgeLength(int s,int t);
  virtual double EdgeLength(MultiModalPRM::TransitionInfo* trans,int s,int t);
  virtual double Heuristic(int mode);

  IncrementalMMPRM_Search(MultiModalPRM& mmprm);
  virtual ~IncrementalMMPRM_Search() {}
  void Init();
  bool ExpandMore();

  void SampleMore(int n);
  void ExpandAdjacent(int n);
  void AddEdge(int s,int t,MultiModalPRM::TransitionInfo* e);
  void PushFringe(int i);
  void UpdateFringe(int i);
  int PopFringe();
  void RefreshFringe();
  double Priority(int n);
  bool GoalConnected() const;
  double PathDistance(int n) const;
  /// This must be called when an edge length changes
  void UpdateEdgeLength(int s, int t); 
  /// This must be called when a heuristic value changes.
  /// Called internally in UpdateEdgeLength.
  void UpdatePriority(int n);

  MultiModalPRM& mmprm;
  typedef Graph::UndirectedGraph<int,MultiModalPRM::TransitionInfo*> SearchGraph;
  typedef IndexedPriorityQueue<int,double> Fringe;
  typedef UpdatePrioritySPP ShortestPathProblem;
  Fringe fringe;
  ShortestPathProblem spp;
  SearchGraph searchGraph;
  std::vector<bool> reachedModes;
  std::vector<bool> outputModes;

  double transSampleWeight;
};

class IncrementalMMPRM_Explicit
{
 public:
  IncrementalMMPRM_Explicit(ExplicitMMCSpace* space);
  virtual ~IncrementalMMPRM_Explicit() {}
  void Init();
  void PlanMore();
  bool Done() const;
  bool ExpandMore();
  void RefineMore();
  virtual int PickRefineCount();

  ExplicitMMCSpace* space;
  MultiModalPRM mmprm;
  IncrementalMMPRM_Search search;
  int numRefineSamplesPerMode;
  int numRefineSamplesPerOldMode;
  int numRefineSamplesConstant;
  bool evenRefinement;

  //temporary / stats
  std::vector<bool> lastRefineSet;
  int remainingRefineSamples;
  int expandPhaseCount,expandStepCount;
  int refinePhaseCount,refineStepCount;
};

#endif
