#include "AnyMotionPlanner.h"
#include <math/sample.h>
#include <utils/AnyCollection.h>
#include <utils/stringutils.h>
#include <utils/PropertyMap.h>
#include "ExplicitCSpace.h"
#include "OptimalMotionPlanner.h"
#include "OMPLInterface.h"
#include "PointLocation.h"
#include "SBL.h"
#include "FMMMotionPlanner.h"
#include "Timer.h"

#if HAVE_TINYXML
#include <tinyxml.h>
#endif


void ToCollection(const MotionPlannerFactory& factory,AnyCollection& items)
{
  items["type"] = factory.type;
  items["knn"] = factory.knn;
  items["connectionThreshold"] = factory.connectionThreshold;
  items["ignoreConnectedComponents"] = factory.ignoreConnectedComponents;
  items["perturbationRadius"] = factory.perturbationRadius;
  items["perturbationIters"] = factory.perturbationIters;
  items["bidirectional"] = factory.bidirectional;
  items["useGrid"] = factory.useGrid;
  items["gridResolution"] = factory.gridResolution;
  items["randomizeFrequency"] = factory.randomizeFrequency;
  items["pointLocation"] = factory.pointLocation;
  items["storeEdges"] = factory.storeEdges;
  items["shortcut"] = factory.shortcut;
  items["restart"] = factory.restart;
  items["restartTermCond"] = factory.restartTermCond;
}


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
  virtual bool IsPointToPoint() const { return mp->IsPointToPoint(); }
  virtual bool IsOptimizing() const { return mp->IsOptimizing(); }
  virtual bool IsLazy() const { return mp->IsLazy(); }
  virtual bool IsLazyConnected(int ma,int mb) const { return mp->IsLazyConnected(ma,mb); }
  virtual bool CheckPath(int ma,int mb) { return mp->CheckPath(ma,mb); }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { mp->GetPath(ma,mb,path); }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { mp->GetRoadmap(roadmap); }
  virtual bool IsSolved() { return mp->IsSolved(); }
  virtual void GetSolution(MilestonePath& path) { return mp->GetSolution(path); }

  SmartPointer<MotionPlannerInterface> mp;
};

/** @brief Tries to produce a path between a start node and a goal space.
 * Does so via goal sampling.  NOTE: only works on base motion planners that
 * accept dynamic goal insertion, such as PRM or SBLPRT.  For other motion 
 * planners, need to use the PointToSetMotionPlannerAdaptor
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
  virtual bool IsPointToPoint() const { return false; }
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

/** @brief Tries to produce a path between a start node and a goal space.
 * Does so via goal sampling / reinitializing 
 */
class PointToSetMotionPlannerAdaptor : public MotionPlannerInterface
{
 public:
  PointToSetMotionPlannerAdaptor(const MotionPlannerFactory& factory,CSpace* space,const Config& qstart,CSpace* goalSpace);
  virtual int PlanMore();
  virtual int NumIterations() const { return numIters; }
  virtual int NumMilestones() const;
  virtual int NumComponents() const;
  virtual bool CanAddMilestone() const { return true; }
  virtual int AddMilestone(const Config& q);
  virtual void GetMilestone(int i,Config& q);
  virtual bool IsConnected(int ma,int mb) const;
  virtual bool IsPointToPoint() const { return false; }
  virtual bool IsOptimizing() const { return true; }
  virtual bool IsLazy() const;
  virtual bool IsLazyConnected(int ma,int mb) const;
  virtual bool CheckPath(int ma,int mb);
  virtual void GetPath(int ma,int mb,MilestonePath& path);
  virtual void GetRoadmap(RoadmapPlanner& roadmap);
  virtual bool IsSolved();
  virtual void GetSolution(MilestonePath& path);

  MotionPlannerFactory factory;
  CSpace* space;
  Config qstart;
  CSpace* goalSpace;

  ///Setting: the planner samples a new goal configuration every n*|goalNodes| iterations
  int sampleGoalPeriod;
  ///Number of total iterations
  int numIters;
  ///Incremented each iteration, when it hits sampleGoalPeriod a goal should be sampled
  int sampleGoalCounter;
  ///These are plans from qstart to each goal node
  vector<SmartPointer<MotionPlannerInterface> > goalPlanners;
  ///The costs of the path to each planner's goal, or Inf if no solution has been found
  vector<Real> goalCosts;
};

/** @brief A multiple-restart motion planner that turns a feasible motion planner into an anytime
 * optimal motion planner by keeping the best path found so far.
 */
class RestartMotionPlanner : public PiggybackMotionPlanner
{
 public:
  RestartMotionPlanner(const MotionPlannerFactory& factory,const MotionPlanningProblem& problem,const HaltingCondition& iterTermCond);
  virtual bool IsOptimizing() const { return true; }
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  virtual int PlanMore();
  virtual int AddMilestone(const Config& q);
  virtual bool IsConnected(int ma,int mb) const;
  virtual void GetPath(int ma,int mb,MilestonePath& path);
  virtual bool IsSolved() { return !bestPath.edges.empty(); }
  virtual void GetSolution(MilestonePath& path) { path = bestPath; }
  virtual int NumIterations() const { return numIters; }

  MotionPlannerFactory factory;
  MotionPlanningProblem problem;
  HaltingCondition iterTermCond;
  MilestonePath bestPath;
  int numIters;
};


/** @brief A multiple-restart motion planner that turns a feasible motion planner into an anytime
 * optimal motion planner by keeping the best path found so far and
 * shortcutting the best path.
 */
class RestartShortcutMotionPlanner : public RestartMotionPlanner
{
 public:
  RestartShortcutMotionPlanner(const MotionPlannerFactory& factory,const MotionPlanningProblem& problem,const HaltingCondition& iterTermCond);
  virtual bool IsOptimizing() const { return true; }
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  virtual int PlanMore();
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const;

  bool shortcutMode;
  int numShortcutIters;
  vector<MilestonePath> candidatePaths;
};

/** @brief Plans a path and then tries to shortcut it with the remaining time.
 */
class ShortcutMotionPlanner : public PiggybackMotionPlanner
{
 public:
  ShortcutMotionPlanner(const SmartPointer<MotionPlannerInterface>& mp);
  virtual bool IsOptimizing() const { return true; }
  virtual std::string Plan(MilestonePath& path,const HaltingCondition& cond);
  virtual int PlanMore();
  virtual bool IsSolved() { return !bestPath.edges.empty(); }
  virtual void GetSolution(MilestonePath& path) { path = bestPath; }
  virtual int NumIterations() const { return numIters; }

  MilestonePath bestPath;
  int numIters;
};



void ReversePath(MilestonePath& path)
{
  for(size_t k=0;k<path.edges.size()/2;k++) {
    SmartPointer<EdgePlanner> e1 = path.edges[k];
    SmartPointer<EdgePlanner> e2 = path.edges[path.edges.size()-k];
    path.edges[k] = e2->ReverseCopy();
    path.edges[path.edges.size()-k] = e1->ReverseCopy();
  }
  if(path.edges.size()%2 == 1)
    path.edges[path.edges.size()/2] = path.edges[path.edges.size()/2]->ReverseCopy();
  if(!path.IsValid()) fprintf(stderr,"ReversePath : Path invalidated ?!?!\n");
}


HaltingCondition::HaltingCondition()
  :foundSolution(true),maxIters(1000),timeLimit(Inf),costThreshold(0),costImprovementPeriod(Inf),costImprovementThreshold(0)
{}

bool HaltingCondition::LoadJSON(const string& str)
{
  AnyCollection items;
  if(!items.read(str.c_str())) return false;
  items["foundSolution"].as(foundSolution);
  items["maxIters"].as(maxIters);
  items["timeLimit"].as(timeLimit);
  items["costThreshold"].as(costThreshold);
  items["costImprovementPeriod"].as(costImprovementPeriod);
  items["costImprovementThreshold"].as(costImprovementThreshold);
  return true;
}

string HaltingCondition::SaveJSON() const
{
  AnyCollection items;
  items["foundSolution"] = foundSolution;
  items["maxIters"] = maxIters;
  items["timeLimit"] = timeLimit;
  items["costThreshold"] = costThreshold;
  items["costImprovementPeriod"] = costImprovementPeriod;
  items["costImprovementThreshold"] = costImprovementThreshold;
  stringstream ss;
  items.write_inline(ss);
  return ss.str();
}

MotionPlanningProblem::MotionPlanningProblem()
  :space(NULL),startSet(NULL),goalSet(NULL),objective(NULL)
{}

MotionPlanningProblem::MotionPlanningProblem(CSpace* _space,const Config& a,const Config& b)
  :space(_space),qstart(a),qgoal(b),startSet(NULL),goalSet(NULL),objective(NULL)
{}

MotionPlanningProblem::MotionPlanningProblem(CSpace* _space,const Config& a,CSpace* _goalSet)
  :space(_space),qstart(a),startSet(NULL),goalSet(_goalSet),objective(NULL)
{}

MotionPlanningProblem::MotionPlanningProblem(CSpace* _space,CSpace* _startSet,CSpace* _goalSet)
  :space(_space),startSet(_startSet),goalSet(_goalSet),objective(NULL)
{}

MotionPlanningProblem::MotionPlanningProblem(CSpace* _space,const Config& a,void* _objective)
  :space(_space),qstart(a),startSet(NULL),goalSet(NULL),objective(_objective)
{}

std::string MotionPlannerInterface::Plan(MilestonePath& path,const HaltingCondition& cond)
{
  path.edges.clear();
  bool foundPath = false;
  Real lastCheckTime = 0, lastCheckValue = 0;
  Timer timer;
  for(int iters=0;iters<cond.maxIters;iters++) {
    Real t=timer.ElapsedTime();
    if(t > cond.timeLimit) {
      if(foundPath) {
	//get the final path
	GetSolution(path);
      }
      return "timeLimit";
    }
    //check for cost improvements
    if(foundPath && t > lastCheckTime + cond.costImprovementPeriod) {
      GetSolution(path);
      Real len = path.Length();
      if(len < cond.costThreshold)
	return "costThreshold";
      if(lastCheckValue - len < cond.costImprovementThreshold)
	return "costImprovementThreshold";
      lastCheckTime = t;
      lastCheckValue = len;
    }
    //do planning, check if a path is found
    PlanMore();
    if(!foundPath) {
      if(IsSolved()) {
	foundPath = true;
	GetSolution(path);
	if(cond.foundSolution) {
	  return "foundSolution";
	}
	lastCheckTime = t;
	lastCheckValue = path.Length();
      }
    }
  }
  if(foundPath) {
    //get the final path
    GetSolution(path);
  }
  return "maxIters";
}

void MotionPlannerInterface::GetStats(PropertyMap& stats) const
{
  stats.set("numIters",NumIterations());
  stats.set("numMilestones",NumMilestones());
  stats.set("numComponents",NumComponents());
}

class RoadmapPlannerInterface  : public MotionPlannerInterface
{
 public:
  RoadmapPlannerInterface(CSpace* space)
    : prm(space),knn(10),connectionThreshold(Inf),numIters(0),ignoreConnectedComponents(false),storeEdges(true)
    {}
  virtual ~RoadmapPlannerInterface() {}
  virtual bool IsOptimizing() const { return true; }
  virtual bool IsPointToPoint() const { return false; }
  virtual bool CanAddMilestone() const { return true; }
  virtual int AddMilestone(const Config& q) { return prm.AddMilestone(q); }
  virtual void GetMilestone(int i,Config& q) { q=prm.roadmap.nodes[i]; }
  virtual void ConnectHint(int n) { 
    if(knn)
      prm.ConnectToNearestNeighbors(n,knn,!ignoreConnectedComponents);
    else
      prm.ConnectToNeighbors(n,connectionThreshold,!ignoreConnectedComponents);
    if(!storeEdges) {
      RoadmapPlanner::Roadmap::Iterator e;
      for(prm.roadmap.Begin(n,e);!e.end();++e)
	*e = NULL;
    }
  }
  virtual bool ConnectHint(int i,int j) {
    bool res=(prm.TestAndConnectEdge(i,j) != NULL);
    if(res) {
      if(!storeEdges) //delete the edge
	prm.GetEdge(i,j) = NULL;
    }
    return res;
  }
  virtual int PlanMore() { 
    Config q;
    prm.space->Sample(q);
    int n=prm.TestAndAddMilestone(q);
    if(n>=0) {
      ConnectHint(n);
    }
    numIters++;
    return n;
  }
  virtual int NumIterations() const { return numIters; }
  virtual int NumMilestones() const { return prm.roadmap.NumNodes(); }
  virtual int NumComponents() const { return prm.ccs.NumComponents(); }
  virtual bool IsConnected(int ma,int mb) const { return prm.AreConnected(ma,mb); }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { prm.CreatePath(ma,mb,path); }
  virtual bool IsSolved() const { return IsConnected(0,1); }
  virtual void GetSolution(MilestonePath& path) { GetPath(0,1,path); }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { roadmap = prm; }

  RoadmapPlanner prm;
  int knn;
  Real connectionThreshold;
  int numIters;
  bool ignoreConnectedComponents,storeEdges;
};

void GetRoadmapIter(TreeRoadmapPlanner::Node* node,RoadmapPlanner& roadmap,int pindex=-1)
{
  if(!node) return;
  int nindex = roadmap.AddMilestone(node->x);
  if(pindex >= 0)
    roadmap.ConnectEdge(pindex,nindex,node->edgeFromParent());
  for(TreeRoadmapPlanner::Node* c=node->getFirstChild();c!=NULL;c=c->getNextSibling())
    GetRoadmapIter(c,roadmap,nindex);
}

void GetRoadmap(const TreeRoadmapPlanner& planner,RoadmapPlanner& roadmap)
{
  roadmap.Cleanup();
  for(size_t i=0;i<planner.connectedComponents.size();i++) {
    GetRoadmapIter(planner.connectedComponents[i],roadmap);
  }
}

class RRTInterface  : public MotionPlannerInterface
{
 public:
  RRTInterface(CSpace* space)
    : rrt(space),numIters(0)
    {}
  virtual ~RRTInterface() {}
  virtual bool CanAddMilestone() const { return true; }
  virtual int AddMilestone(const Config& q) {
    TreeRoadmapPlanner::Node* n=rrt.TestAndAddMilestone(q);
    if(n) return rrt.milestones.size()-1;
    else return -1;
  }
  virtual void GetMilestone(int i,Config& q) { q=rrt.milestones[i]->x; }
  virtual int PlanMore() { 
    TreeRoadmapPlanner::Node* n=rrt.Extend();
    numIters++;
    if(n) return rrt.milestones.size()-1;
    else return -1;
  }
  virtual void ConnectHint(int m) { rrt.ConnectToNeighbors(rrt.milestones[m]); }
  virtual bool ConnectHint(int ma,int mb) { return rrt.TryConnect(rrt.milestones[ma],rrt.milestones[mb])!=NULL; }
  virtual int NumIterations() const { return numIters; }
  virtual int NumMilestones() const { return rrt.milestones.size(); }
  virtual int NumComponents() const { return rrt.connectedComponents.size(); }
  virtual bool IsConnected(int ma,int mb) const { return rrt.milestones[ma]->connectedComponent == rrt.milestones[mb]->connectedComponent; }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { rrt.CreatePath(rrt.milestones[ma],rrt.milestones[mb],path); }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { ::GetRoadmap(rrt,roadmap); }

  RRTPlanner rrt;
  int numIters;
};

class BiRRTInterface  : public MotionPlannerInterface
{
 public:
  BiRRTInterface(CSpace* space)
    : rrt(space),numIters(0)
    {}
  virtual ~BiRRTInterface() {}
  virtual bool CanAddMilestone() const { return true; }
  virtual int AddMilestone(const Config& q) {
    TreeRoadmapPlanner::Node* n=rrt.TestAndAddMilestone(q);
    if(n) return rrt.milestones.size()-1;
    else return -1;
  }
  virtual void GetMilestone(int i,Config& q) { q=rrt.milestones[i]->x; }
  virtual int PlanMore() { 
    bool res=rrt.Plan();
    numIters++;
    if(res) return (int)rrt.milestones.size()-1;
    return -1;
  }
  virtual int NumIterations() const { return numIters; }
  virtual int NumMilestones() const { return rrt.milestones.size(); }
  virtual int NumComponents() const { return rrt.connectedComponents.size(); }
  virtual bool IsConnected(int ma,int mb) const { return rrt.milestones[ma]->connectedComponent == rrt.milestones[mb]->connectedComponent; }
  virtual void GetPath(int ma,int mb,MilestonePath& path) {
    Assert(ma==0 && mb==1);
    rrt.CreatePath(path);
  }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { ::GetRoadmap(rrt,roadmap); }

  BidirectionalRRTPlanner rrt;
  int numIters;
};



class SBLInterface  : public MotionPlannerInterface
{
 public:
  SBLInterface(CSpace* space) {
    sbl = new SBLPlanner(space);
  }
  SBLInterface(CSpace* space,bool grid,Real gridDivs,int randomizeFrequency) {
    if(grid) {
      SBLPlannerWithGrid* sblgrid = new SBLPlannerWithGrid(space);
      sblgrid->gridDivision=gridDivs;
      sblgrid->numItersPerRandomize = randomizeFrequency;
      sbl = sblgrid;
    }
    else {
      sbl = new SBLPlanner(space);
    }
  }
  void Init(const Config& qStart,const Config& qGoal) {
    sbl->Init(qStart,qGoal);
  }
  virtual bool CanAddMilestone() const { if(qStart.n != 0 && qGoal.n != 0) return false; return true; }
  virtual int AddMilestone(const Config& q) {
    if(qStart.n == 0) {
      qStart = q;
      return 0;
    }
    else if(qGoal.n == 0) {
      qGoal = q;
      sbl->Init(qStart,qGoal);
      return 1;
    }
    AssertNotReached();
    return -1;
  }
  virtual void GetMilestone(int i,Config& q) { if(i==0) q=*sbl->tStart->root; else q=*sbl->tGoal->root; }
  virtual int PlanMore() { 
    if(qStart.n == 0 || qGoal.n == 0) {
      fprintf(stderr,"AnyMotionPlanner::PlanMore(): SBL is a point-to-point planner, AddMilestone() must be called to set the start and goal configuration\n");
      return -1;
    }
    if(!sbl->IsDone()) sbl->Extend();
    return -1;
  }
  virtual int NumIterations() const { return sbl->numIters; }
  virtual int NumMilestones() const {
    Graph::CountCallback<SBLTree::Node*> cb1,cb2;
    if(sbl->tStart && sbl->tStart->root)
      sbl->tStart->root->DFS(cb1);
    if(sbl->tGoal && sbl->tGoal->root)
      sbl->tGoal->root->DFS(cb2);
    return cb1.count+cb2.count;
  }
  virtual int NumComponents() const {
    if(!sbl->tStart) return 0;
    if(!sbl->tGoal) return 1;
    return 2;
  }
  virtual bool IsConnected(int ma,int mb) const { return sbl->IsDone(); }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { sbl->CreatePath(path); if(ma == 1) ReversePath(path); }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const {
    if(qStart.n != 0) 
      roadmap.AddMilestone(qStart);
    if(qGoal.n != 0) 
      roadmap.AddMilestone(qGoal);
    if(sbl->tStart && sbl->tStart->root)
      GetRoadmapRecurse(sbl->tStart->root,roadmap,0);
    if(sbl->tGoal && sbl->tGoal->root)
      GetRoadmapRecurse(sbl->tGoal->root,roadmap,1);
  }
  void GetRoadmapRecurse(SBLTree::Node* n,RoadmapPlanner& roadmap,int nIndex=-1) const
  {
    if(nIndex < 0)
      nIndex = roadmap.AddMilestone(*n);
    SBLTree::Node* c=n->getFirstChild();
    while(c != NULL) {
      int cIndex = roadmap.AddMilestone(*c);
      roadmap.ConnectEdge(nIndex,cIndex,c->edgeFromParent());
      GetRoadmapRecurse(c,roadmap,cIndex);
      c = c->getNextSibling();
    }
  }

  SmartPointer<SBLPlanner> sbl;
  Config qStart,qGoal;
};

class SBLPRTInterface  : public MotionPlannerInterface
{
 public:
  SBLPRTInterface(CSpace* space)
    : sblprt(space)
    {}
  virtual ~SBLPRTInterface() {}
  virtual bool CanAddMilestone() const { return true; }
  virtual bool IsPointToPoint() const { return false; }
  virtual int AddMilestone(const Config& q) { return sblprt.AddSeed(q); }
  virtual void GetMilestone(int i,Config& q) { q=*sblprt.roadmap.nodes[i]->root; }
  virtual void ConnectHint(int i) {
    for(size_t j=0;j<sblprt.roadmap.nodes.size();j++)
      sblprt.AddRoadmapEdge(i,j);
  }
  virtual bool ConnectHint(int i,int j) { sblprt.AddRoadmapEdge(i,j); return false; }
  virtual int PlanMore() { 
    if(sblprt.roadmap.nodes.empty()) {
      fprintf(stderr,"SBLPRTInterface::PlanMore(): no seed configurations set yet\n");
      return -1;
    }
    sblprt.Expand();
    return -1;
  }
  virtual int NumIterations() const { return sblprt.numIters; }
  virtual int NumMilestones() const { 
    int n=0;
    for(size_t i=0;i<sblprt.roadmap.nodes.size();i++) {
      Graph::CountCallback<SBLTree::Node*> callback;
      sblprt.roadmap.nodes[i]->root->DFS(callback);
      n+=callback.count;
    }
    return n;
  }
  virtual int NumComponents() const { return sblprt.ccs.NumComponents(); }
  virtual bool IsConnected(int ma,int mb) const { return sblprt.AreSeedsConnected(ma,mb); }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { sblprt.CreatePath(ma,mb,path); }
  virtual bool IsSolved() const { return false; }
  virtual void GetSolution(MilestonePath& path) { return; }

  SBLPRT sblprt;
};


void CalcCCs(RoadmapPlanner& roadmap)
{
  roadmap.ccs.Clear();
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++)
    roadmap.ccs.AddNode();
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {
    Graph::EdgeIterator<SmartPointer<EdgePlanner> > e;
    for(roadmap.roadmap.Begin(i,e);!e.end();e++)
      roadmap.ccs.AddEdge(i,e.target());
  }
}

class PRMStarInterface  : public MotionPlannerInterface
{
 public:
  PRMStarInterface(CSpace* space) : planner(space)
  {
    //planner.connectByRadius = true;
    planner.connectByRadius = false;
    PropertyMap props;
    space->Properties(props);
    int d;
    if(props.get("intrinsicDimension",d))
      ;
    else {
      Vector q;
      space->Sample(q);
      d = q.n;
    }
    Real v;
    if(props.get("volume",v)) {
      planner.connectRadiusConstant = Pow(v,1.0/Real(d));
    }
    else
      planner.connectRadiusConstant = 1;

    //TEMP: test keeping this at a constant regardless of space volume?
    //planner.connectRadiusConstant = 1;
  }
  virtual ~PRMStarInterface() {}
  virtual bool IsOptimizing() const { return true; }
  virtual bool CanAddMilestone() const { if(qStart.n != 0 && qGoal.n != 0) return false; return true; }
  virtual int AddMilestone(const Config& q) {
    if(qStart.n == 0) {
      qStart = q;
      return 0;
    }
    else if(qGoal.n == 0) {
      qGoal = q;
      planner.Init(qStart,qGoal);
      assert(planner.start == 0 && planner.goal == 1);
      return 1;
    }
    AssertNotReached();
    return -1;
  }
  virtual void GetMilestone(int i,Config& q) { q=planner.roadmap.nodes[i]; }
  virtual int PlanMore() { 
    if(planner.start < 0 || planner.goal < 0) {
      fprintf(stderr,"AnyMotionPlanner::PlanMore(): PRM* is a point-to-point planner, AddMilestone() must be called to set the start and goal configuration\n");
      return -1;
    }
    planner.PlanMore();
    return -1;
  }
  virtual int NumIterations() const { return planner.numPlanSteps; }
  virtual int NumMilestones() const { return planner.roadmap.nodes.size(); }
  virtual int NumComponents() const { return 1; }
  virtual bool IsConnected(int ma,int mb) const { 
    Assert(ma==0 && mb==1);
    return planner.HasPath();
  }
  virtual void GetPath(int ma,int mb,MilestonePath& path) {
    Assert(ma==0 && mb==1);
    planner.GetPath(path);
  }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { 
    if(planner.lazy)
      roadmap.roadmap = planner.LBroadmap;
    else
      roadmap.roadmap = planner.roadmap;
    CalcCCs(roadmap);
  }
  virtual void GetStats(PropertyMap& stats) const {
    MotionPlannerInterface::GetStats(stats);
    stats.set("configCheckTime",planner.tCheck);
    stats.set("knnTime",planner.tKnn);
    stats.set("connectTime",planner.tConnect);
    if(planner.lazy)
      stats.set("lazyPathCheckTime",planner.tLazy);
    stats.set("shortestPathsTime",planner.tShortestPaths);
    stats.set("numEdgeChecks",planner.numEdgeChecks);
    if(planner.lazy)
      stats.set("numEdgesPrechecked",planner.numEdgePrechecks);
  }

  PRMStarPlanner planner;
  Config qStart,qGoal;
};

class FMMInterface  : public MotionPlannerInterface
{
 public:
  FMMInterface(CSpace* space,bool _anytime)
    : planner(space),anytime(_anytime),iterationCount(0)
    {}
  virtual ~FMMInterface() {}
  virtual bool CanAddMilestone() const { if(qStart.n != 0 && qGoal.n != 0) return false; return true; }
  virtual int AddMilestone(const Config& q) {
    if(qStart.n == 0) {
      qStart = q;
      return 0;
    }
    else if(qGoal.n == 0) {
      qGoal = q;
      planner.Init(qStart,qGoal);
      return 1;
    }
    AssertNotReached();
    return -1;
  }
  virtual void GetMilestone(int i,Config& q) { 
    if(i==0) q=qStart;
    if(i==1) q=qGoal;
    return;
  }
  virtual int PlanMore() { 
    if(qStart.n == 0 || qGoal.n == 0) {
      fprintf(stderr,"AnyMotionPlanner::PlanMore(): FMM is a point-to-point planner, AddMilestone() must be called to set the start and goal configuration\n");
      return -1;
    }
    iterationCount++;
    if(anytime) {
      int d=qStart.n;
      Real scaleFactor = Pow(0.5,1.0/Real(d));
      planner.resolution *= scaleFactor;

      planner.SolveFMM();
    }
    else
      planner.SolveFMM();
    return -1;
  }
  virtual int NumIterations() const { return iterationCount; }
  virtual int NumMilestones() const { return 2; }
  virtual int NumComponents() const { return 1; }
  virtual bool IsConnected(int ma,int mb) const { 
    Assert(ma==0 && mb==1);
    if(planner.solution.edges.empty()) return false;
    return true;
  }
  virtual void GetPath(int ma,int mb,MilestonePath& path) {
    Assert(ma==0 && mb==1);
    path = planner.solution;
  }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { 
    //simply output all the visited grid cells
    if(planner.distances.numValues()==0) return;
    vector<int> index(qStart.n,0);
    do {
      if(!IsInf(planner.distances[index])) {
	Vector config = planner.FromGrid(index);
	roadmap.AddMilestone(config);
      }
    } while(IncrementIndex(index,planner.distances.dims)==0);

    if(planner.solution.edges.empty()) {
      //add debugging path
      int prev = -1;
      for(size_t i=0;i<planner.failedCheck.edges.size();i++) {
	if(prev < 0) 
	  prev = roadmap.AddMilestone(planner.failedCheck.GetMilestone(0));
	int n = roadmap.AddMilestone(planner.failedCheck.GetMilestone(i+1));
	roadmap.ConnectEdge(prev,n,planner.failedCheck.edges[i]);
	prev = n;
      }
    }
  }
  virtual void GetStats(PropertyMap& stats) const {
    MotionPlannerInterface::GetStats(stats);
    if(planner.dynamicDomain) {
      vector<Real> bmin(planner.bmin),bmax(planner.bmax);
      stats.setArray("gridMin",bmin);
      stats.setArray("gridMax",bmax);
    }
    stats.set("gridSize",planner.distances.numValues());
  }


  FMMMotionPlanner planner;
  Config qStart,qGoal;
  bool anytime;
  int iterationCount;
};


#if HAVE_OMPL
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
namespace og = ompl::geometric;

map<string,ob::PlannerAllocator> omplAllocators;
map<string,map<string,string> > omplParameterMaps;
bool omplAllocatorsSetup = false;

ob::PlannerPtr BITstarAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::BITstar(si)); }
ob::PlannerPtr ESTAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::EST(si)); }
ob::PlannerPtr PRMAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::PRM(si)); }
ob::PlannerPtr LazyPRMAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::LazyPRM(si)); }
ob::PlannerPtr PRMstarAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::PRMstar(si)); }
ob::PlannerPtr LazyPRMstarAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::LazyPRMstar(si)); }
ob::PlannerPtr PDSTAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::PDST(si)); }
ob::PlannerPtr RRTAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::RRT(si)); }
ob::PlannerPtr RRTConnectAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::RRTConnect(si)); }
ob::PlannerPtr RRTstarAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::RRTstar(si)); }
ob::PlannerPtr LBTRRTAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::LBTRRT(si)); }
ob::PlannerPtr SBLAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::SBL(si)); }
ob::PlannerPtr STRIDEAllocator(const ob::SpaceInformationPtr& si) { return ob::PlannerPtr(new og::STRIDE(si)); }

void SetupOMPLAllocators()
{
  if(omplAllocatorsSetup) return;
  omplAllocatorsSetup = true;
  omplAllocators["bitstar"]  = BITstarAllocator;
  omplAllocators["est"]  = ESTAllocator;
  omplAllocators["prm"]  = PRMAllocator;
  omplAllocators["lazyprm"]  = LazyPRMAllocator;
  omplAllocators["prm*"]  = PRMstarAllocator;
  omplAllocators["lazyprm*"]  = LazyPRMstarAllocator;
  omplAllocators["pdst"]  = PDSTAllocator;
  omplAllocators["rrt"]  = RRTAllocator;
  omplAllocators["rrtconnect"]  = RRTConnectAllocator;
  omplAllocators["rrt*"]  = RRTstarAllocator;
  omplAllocators["lbtrrt"]  = LBTRRTAllocator;
  omplAllocators["sbl"]  = SBLAllocator;
  omplAllocators["stride"]  = STRIDEAllocator;
  omplParameterMaps["prm"]["knn"] = "max_nearest_neighbors";
  omplParameterMaps["lazyprm"]["connectionThreshold"] = "range";
  omplParameterMaps["lazyprm"]["knn"] = "max_nearest_neighbors";
  omplParameterMaps["lazyprm*"]["connectionThreshold"] = "range";
  omplParameterMaps["stride"]["connectionThreshold"] = "range";
  omplParameterMaps["rrt"]["connectionThreshold"] = "range";
  omplParameterMaps["rrt*"]["connectionThreshold"] = "range";
  omplParameterMaps["rrtconnect"]["connectionThreshold"] = "range";
  omplParameterMaps["lbtrrt"]["connectionThreshold"] = "range";
  omplParameterMaps["lbtrrt"]["suboptimalityFactor"] = "epsilon";
  omplParameterMaps["sbl"]["connectionThreshold"] = "range";
}

class OMPLPlannerInterface  : public MotionPlannerInterface
{
 public:
  OMPLPlannerInterface(CSpace* _space,const char* plannerName)
    : type(plannerName),space(new CSpaceOMPLSpaceInformation(_space)),spacePtr(space)
    {
      SetupOMPLAllocators();
      if(omplAllocators.count(plannerName) == 0) {
	fprintf(stderr,"OMPLPlannerInterface: no available OMPL planner of type %s\n",plannerName);
	return;
      }
      planner = omplAllocators[plannerName](spacePtr);
    }
  virtual ~OMPLPlannerInterface()
  { }
  void ReadParameters(const MotionPlannerFactory& factory) {
    if(omplParameterMaps.count(type) > 0) {
      AnyCollection items;
      ToCollection(factory,items);
      const map<string,string>& parammap = omplParameterMaps[type];
      for(map<string,string>::const_iterator i=parammap.begin();i!=parammap.end();i++) {
        string val;
        bool converted = LexicalCast((const AnyValue&)items[i->first],val);
        if(!converted) {
          fprintf(stderr,"OMPLPlannerInterface::ReadParameters: Warning, item %s was not castable to string?\n",i->first.c_str());
          continue;
        }
        if(!SetParameter(i->second,val)) {
          fprintf(stderr,"OMPLPlannerInterface::ReadParameters: Warning, planner %s does not have parameter named %s\n",type.c_str(),i->second.c_str());
          continue;
        }
      }
    }
  }
  bool SetParameter(const string& name,string& val) {
    if(!planner) return false;
    if(!planner->params().hasParam(name)) return false;
    planner->params().setParam(name,val);
    return true;
  }
  virtual bool CanAddMilestone() const { if(qStart.n != 0 && qGoal.n != 0) return false; return true; }
  virtual int AddMilestone(const Config& q) {
    if(!planner) return -1;
    if(qStart.n == 0) {
      qStart = q;
      return 0;
    }
    else if(qGoal.n == 0) {
      qGoal = q;
      //initialize the planner
      ob::State* sStart = space->ToOMPL(qStart);
      ob::State* sGoal = space->ToOMPL(qGoal);
      planner->getProblemDefinition()->addStartState(sStart);
      planner->getProblemDefinition()->setGoalState(sGoal);
      space->freeState(sStart);
      space->freeState(sGoal);
      planner->setup();
      planner->checkValidity();
      return 1;
    }
    AssertNotReached();
    return -1;
  }
  virtual void GetMilestone(int i,Config& q) { 
    if(i==0) q=qStart;
    if(i==1) q=qGoal;
    return;
  }
  virtual int PlanMore() { 
    if(qStart.n == 0 || qGoal.n == 0) {
      fprintf(stderr,"AnyMotionPlanner::PlanMore(): OMPL interfaces onlys upports a point-to-point planners, AddMilestone() must be called to set the start and goal configuration\n");
      return -1;
    }
    if(!planner) return -1;
    //pick some time
    if(planner->solve(0.1)) return 1;
    return -1;
  }
  virtual int NumIterations() const { return iterationCount; }
  virtual int NumMilestones() const { return -1; }
  virtual int NumComponents() const { return -1; }
  virtual bool IsConnected(int ma,int mb) const { 
    Assert(ma==0 && mb==1);
    return ( planner->getProblemDefinition()->getSolutionPath() != NULL); 
  }
  virtual void GetPath(int ma,int mb,MilestonePath& mpath) {
    Assert(ma==0 && mb==1);
    ob::PathPtr path = planner->getProblemDefinition()->getSolutionPath();
    if(path != NULL) {
      og::PathGeometric* gpath = dynamic_cast<og::PathGeometric*>(&*path);
      if(gpath != NULL) {
	std::vector<ob::State*>& states = gpath->getStates();
	if(states.size() >= 1) {
	  mpath.edges.resize(states.size()-1);
	  for(size_t i=0;i+1<states.size();i++)
	    mpath.edges[i] = space->cspace->LocalPlanner(space->FromOMPL(states[i]),space->FromOMPL(states[i+1]));
	}
      }
    }
  }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) const { 
    ob::PlannerData data(spacePtr);
    planner->getPlannerData(data);
    roadmap.roadmap.nodes.resize(data.numVertices());
    for(size_t i=0;i<data.numVertices();i++)
      roadmap.roadmap.nodes[i] = space->FromOMPL(data.getVertex(i).getState());
    for(size_t i=0;i<data.numVertices();i++) {
      vector<unsigned int> edges;
      data.getEdges(i,edges);
      for(size_t j=0;j<edges.size();j++)
      	roadmap.roadmap.AddEdge(i,edges[j],space->cspace->LocalPlanner(roadmap.roadmap.nodes[i],roadmap.roadmap.nodes[edges[i]]));
    }
  }
  virtual void GetStats(PropertyMap& stats) const {
    MotionPlannerInterface::GetStats(stats);
    ob::PlannerData data(spacePtr);
    planner->getPlannerData(data);
    for(map<string,string>::const_iterator i=data.properties.begin();i!=data.properties.end();i++)
      stats[i->first] = i->second;
  }

  string type;
  CSpaceOMPLSpaceInformation* space;
  ob::SpaceInformationPtr spacePtr;
  ob::PlannerPtr planner;
  Config qStart,qGoal;
  int iterationCount;
};

#endif // HAVE_OMPL


PiggybackMotionPlanner::PiggybackMotionPlanner(const SmartPointer<MotionPlannerInterface>& _mp)
  :mp(_mp)
{}

PointToSetMotionPlanner::PointToSetMotionPlanner(const SmartPointer<MotionPlannerInterface>& _mp,const Config& _qstart,CSpace* _goal)
  :PiggybackMotionPlanner(_mp),goalSpace(_goal),sampleGoalPeriod(50),sampleGoalCounter(0)
{
  int mstart = mp->AddMilestone(_qstart);
  Assert(mstart == 0);
}

std::string PointToSetMotionPlanner::Plan(MilestonePath& path,const HaltingCondition& cond)
{
  return MotionPlannerInterface::Plan(path,cond);
}

int PointToSetMotionPlanner::PlanMore()
{
  if(mp->CanAddMilestone()) {
    sampleGoalCounter++;
    if(sampleGoalCounter >= sampleGoalPeriod*((int)goalNodes.size()+1)) {
      sampleGoalCounter = 0;
      Config q;
      if(SampleGoal(q)) {
	return AddMilestone(q);
      }
      else
	return -1;
    }
  }
  int res = mp->PlanMore();
  if(res >= 0) {
    Config q;
    mp->GetMilestone(res,q);
    if(goalSpace->IsFeasible(q))
      goalNodes.push_back(res);
  }
  return res;
}

bool PointToSetMotionPlanner::IsSolved() 
{
  for(size_t i=0;i<goalNodes.size();i++)
    if(mp->IsConnected(0,goalNodes[i])) return true;
  return false;
}

void PointToSetMotionPlanner::GetSolution(MilestonePath& path)
{
  path.edges.clear();
  for(size_t i=0;i<goalNodes.size();i++) {
    MilestonePath temp;
    mp->GetPath(0,goalNodes[i],temp);
    if(!temp.edges.empty()) {
      if(path.edges.empty()) path = temp;
      else {
	if(temp.Length() < path.Length()) 
	  path = temp;
      }
    }
  }   
}
 
bool PointToSetMotionPlanner::SampleGoal(Config& q)
{
  goalSpace->Sample(q);
  return goalSpace->IsFeasible(q);
}

int PointToSetMotionPlanner::AddMilestone(const Config& q)
{
  int n=mp->AddMilestone(q);
  if(goalSpace->IsFeasible(q))
    goalNodes.push_back(n);
  return n;
}

PointToSetMotionPlannerAdaptor::PointToSetMotionPlannerAdaptor(const MotionPlannerFactory& _factory,CSpace* _space,const Config& _qstart,CSpace* _goal)
  :factory(_factory),space(_space),qstart(_qstart),goalSpace(_goal),sampleGoalPeriod(50),numIters(0),sampleGoalCounter(0)
{
}

int PointToSetMotionPlannerAdaptor::PlanMore()
{
  //test to see if there's work to be done on existing paths
  bool anyRemaining = false;
  for(size_t i=0;i<goalPlanners.size();i++) {
    if(goalPlanners[i]->IsOptimizing() || IsInf(goalCosts[i])) 
      anyRemaining = true;
  }
  //if not, sample a new goal
  sampleGoalCounter += 1;
  numIters += 1;
  if(!anyRemaining || sampleGoalCounter >= sampleGoalPeriod*(int)goalPlanners.size()) {
    //printf("Sampling a goal configuration on iteration %d\n",numIters);
    sampleGoalCounter = 0;
    Config q;
    goalSpace->Sample(q);
    return AddMilestone(q);
  }
  for(size_t i=0;i<goalPlanners.size();i++) {
    if(goalPlanners[i]->IsOptimizing() || IsInf(goalCosts[i])) {
      int res = goalPlanners[i]->PlanMore();
      if(goalPlanners[i]->IsConnected(0,1)) {
        MilestonePath path;
        goalPlanners[i]->GetSolution(path);
        goalCosts[i] = path.Length();
      }
    }
  }
  return -1;
}

int PointToSetMotionPlannerAdaptor::NumMilestones() const
{
  int n = 1;
  for(size_t i=0;i<goalPlanners.size();i++)
    n += goalPlanners[i]->NumMilestones()-1;
  return n;
}

int PointToSetMotionPlannerAdaptor::NumComponents() const
{
  int n=1;
  for(size_t i=0;i<goalPlanners.size();i++)
    n += goalPlanners[i]->NumComponents()-1;
  return n; 
}

void PointToSetMotionPlannerAdaptor::GetMilestone(int i,Config& q)
{
  if(i==0) q=qstart;
  if(i<=(int)goalPlanners.size())
    goalPlanners[i-1]->GetMilestone(1,q);
  else {
    //seek for which goal planner contains i
    for(size_t j=0;j<goalPlanners.size();j++) {
      if(i < goalPlanners[j]->NumMilestones()-1) {
        goalPlanners[j]->GetMilestone(i-1,q);
        return;
      }
      i -= goalPlanners[j]->NumMilestones()-1;
    }
  }
}

bool PointToSetMotionPlannerAdaptor::IsConnected(int ma,int mb) const
{
  if(ma == mb) return true;
  if(ma > mb) return IsConnected(mb,ma);
  if(ma != 0) {
    return IsConnected(0,ma) && IsConnected(0,mb);
  }
  if(mb<=(int)goalPlanners.size())
    return goalPlanners[mb-1]->IsConnected(0,1);
  else {
    //seek for which goal planner contains mb
    for(size_t j=0;j<goalPlanners.size();j++) {
      if(mb < goalPlanners[j]->NumMilestones()-1) {
        return goalPlanners[j]->IsConnected(0,mb-1);
      }
      mb -= goalPlanners[j]->NumMilestones()-1;
    }
  }
  return false;
}

bool PointToSetMotionPlannerAdaptor::IsLazy() const
{
  if(goalPlanners.empty()) return false;
  return goalPlanners[0]->IsLazy();
}
bool PointToSetMotionPlannerAdaptor::IsLazyConnected(int ma,int mb) const
{
  if(ma == mb) return true;
  if(ma > mb) return IsLazyConnected(mb,ma);
  if(ma != 0) {
    return IsLazyConnected(0,ma) && IsLazyConnected(0,mb);
  }
  if(mb<=(int)goalPlanners.size())
    return goalPlanners[mb-1]->IsLazyConnected(0,1);
  else {
    //seek for which goal planner contains mb
    for(size_t j=0;j<goalPlanners.size();j++) {
      if(mb < goalPlanners[j]->NumMilestones()-1) {
        return goalPlanners[j]->IsLazyConnected(0,mb-1);
      }
      mb -= goalPlanners[j]->NumMilestones()-1;
    }
  }
  return false;
}

bool PointToSetMotionPlannerAdaptor::CheckPath(int ma,int mb)
{
  if(ma == mb) return true;
  if(ma > mb) return CheckPath(mb,ma);
  if(ma != 0) {
    return CheckPath(0,ma) && CheckPath(0,mb);
  }
  if(mb<=(int)goalPlanners.size())
    return goalPlanners[mb-1]->CheckPath(0,1);
  else {
    //seek for which goal planner contains mb
    for(size_t j=0;j<goalPlanners.size();j++) {
      if(mb < goalPlanners[j]->NumMilestones()-1) {
        return goalPlanners[j]->CheckPath(0,mb-1);
      }
      mb -= goalPlanners[j]->NumMilestones()-1;
    }
  }
  return false;
}

void PointToSetMotionPlannerAdaptor::GetPath(int ma,int mb,MilestonePath& path)
{
  if(ma == mb) return;
  if(ma > mb) {
    GetPath(mb,ma,path);
    ReversePath(path);
    return;
  }
  if(ma != 0) {
    MilestonePath pa,pb;
    GetPath(0,ma,pa);
    GetPath(0,mb,pb);
    ReversePath(pa);
    path = pa;
    path.Concat(pb);
    return;
  }
  if(mb<=(int)goalPlanners.size())
    goalPlanners[mb-1]->GetPath(0,1,path);
  else {
    //seek for which goal planner contains mb
    for(size_t j=0;j<goalPlanners.size();j++) {
      if(mb < goalPlanners[j]->NumMilestones()-1) {
        goalPlanners[j]->GetPath(0,mb-1,path);
        return;
      }
      mb -= goalPlanners[j]->NumMilestones()-1;
    }
  }
}

void PointToSetMotionPlannerAdaptor::GetRoadmap(RoadmapPlanner& roadmap)
{
  FatalError("TODO: get roadmap for point-to-set adaptor");
}

bool PointToSetMotionPlannerAdaptor::IsSolved() 
{
  for(size_t i=0;i<goalCosts.size();i++)
    if(!IsInf(goalCosts[i])) return true;
  return false;
}

void PointToSetMotionPlannerAdaptor::GetSolution(MilestonePath& path)
{
  int best=-1;
  Real bestCost = Inf;
  for(size_t i=0;i<goalCosts.size();i++) {
    if(goalCosts[i] < bestCost) {
      bestCost = goalCosts[i];
      best = (int)i;
    }
  }   
  if(best < 0) return;
  goalPlanners[best]->GetSolution(path);
}
 
int PointToSetMotionPlannerAdaptor::AddMilestone(const Config& q)
{
  if(goalSpace->IsFeasible(q) && space->IsFeasible(q)) {
    goalPlanners.push_back(factory.Create(space,qstart,q));
    goalCosts.push_back(Inf);
    if(goalPlanners.back()->IsConnected(0,1)) { //straight line connection
      MilestonePath path;
      goalPlanners.back()->GetSolution(path);
      goalCosts.back() = path.Length();
    }
    return (int)goalPlanners.size()-1;
  }
  else {
    /*
    ExplicitCSpace* espace = dynamic_cast<ExplicitCSpace*>(goalSpace);
    if(espace) {
      printf("Failures:\n");
      espace->PrintInfeasibleNames(q);
    }
    */
    return -1;
  }
}



MotionPlannerFactory::MotionPlannerFactory()
  :type("any"),
   knn(10),
   connectionThreshold(Inf),
   suboptimalityFactor(0),
   ignoreConnectedComponents(false),
   perturbationRadius(0.1),perturbationIters(5),
   bidirectional(true),
   useGrid(true),gridResolution(0),randomizeFrequency(50),
   storeEdges(false),shortcut(false),restart(false),
   restartTermCond("{foundSolution:1,maxIters:1000}")
{}

MotionPlannerInterface* MotionPlannerFactory::Create(const MotionPlanningProblem& problem)
{
  if(problem.startSet) FatalError("MotionPlannerFactory: Cannot do start-set problems yet");
  if(problem.qstart.empty() && (!problem.qgoal.empty() || problem.goalSet!=NULL)) FatalError("MotionPlannerFactory: Goal set specified but start not specified");
  if(!problem.qstart.empty() && problem.goalSet) { //point-to-goal problem
    //pick a multi-query planner for the underlying planner
    string oldtype = type;
    if(type == "any") type = "sblprt";
    MotionPlannerInterface* mp = Create(problem.space);
    type = oldtype;
    if(mp->IsPointToPoint()) {
      printf("MotionPlannerFactory: warning, motion planner %s does not fully accept point-to-set problems, applying multi-query adaptor\n",type.c_str());
      delete mp;
      return new PointToSetMotionPlannerAdaptor(*this,problem.space,problem.qstart,problem.goalSet);
    }
    else {
      PointToSetMotionPlanner* psmp = new PointToSetMotionPlanner(mp,problem.qstart,problem.goalSet);
      return psmp;
    }
  }
  else {
    MotionPlannerInterface* mp = CreateRaw(problem.space);
    if(!mp) return NULL;
    if(!problem.qstart.empty()) {
      int qstart = mp->AddMilestone(problem.qstart);
      Assert(qstart == 0);
    }
    if(!problem.qgoal.empty()) {
      int qgoal = mp->AddMilestone(problem.qgoal);
      Assert(qgoal == 1);
    }
    return ApplyModifiers(mp,problem);
  }
}

MotionPlannerInterface* MotionPlannerFactory::Create(CSpace* space)
{
  MotionPlanningProblem problem;
  problem.space = space;
  return Create(problem);
}

MotionPlannerInterface* MotionPlannerFactory::ApplyModifiers(MotionPlannerInterface* planner,const MotionPlanningProblem& problem)
{
  if(shortcut && restart) {
    HaltingCondition iterTerm;
    if(!restartTermCond.empty())
      iterTerm.LoadJSON(restartTermCond);
    delete planner;
    MotionPlannerFactory norestart = *this;
    norestart.restart = false;
    norestart.shortcut = false;
    return new RestartShortcutMotionPlanner(norestart,problem,iterTerm);
  }
  else if(restart) {
    HaltingCondition iterTerm;
    if(!restartTermCond.empty())
      iterTerm.LoadJSON(restartTermCond);
    delete planner;
    MotionPlannerFactory norestart = *this;
    norestart.restart = false;
    return new RestartMotionPlanner(norestart,problem,iterTerm);
  }
  else if(shortcut) {
    return new ShortcutMotionPlanner(planner);
  }
  else
    return planner;
}

bool ReadPointLocation(const string& str,RoadmapPlanner& planner)
{
  if(str.empty()) return false; //use default
  stringstream ss(str);
  string type;
  ss>>type;
  if(type=="random") {
    planner.pointLocator = new RandomPointLocation(planner.roadmap.nodes);
    return true;
  }
  else if(type=="randombest") {
    int k;
    ss >> k;
    if(!ss) {
      fprintf(stderr,"Error reading point location string \"randombest [k]\"\n");
      return false;
    }
    planner.pointLocator = new RandomBestPointLocation(planner.roadmap.nodes,planner.space,k);
    return true;
  }
  else if(type=="kdtree") {
    PropertyMap props;
    planner.space->Properties(props);
    int euclidean;
    if(props.get("euclidean",euclidean) && euclidean == 0)
      fprintf(stderr,"MotionPlannerFactory: Warning, requesting K-D tree point location for non-euclidean space\n");

    vector<Real> weights;
    if(props.getArray("metricWeights",weights))
      planner.pointLocator = new KDTreePointLocation(planner.roadmap.nodes,2,weights);
    else
      planner.pointLocator = new KDTreePointLocation(planner.roadmap.nodes);
    return true;
  }
  else {
    fprintf(stderr,"Unsupported point location type %s\n",type.c_str());
    return false;
  }
}

MotionPlannerInterface* MotionPlannerFactory::CreateRaw(CSpace* space)
{
  Lowercase(type);
#if HAVE_OMPL
  if(StartsWith(type.c_str(),"ompl")) {
    string ompltype = type.substr(5,type.size()-5);
    OMPLPlannerInterface* ompl = new OMPLPlannerInterface(space,ompltype.c_str());
    if(!ompl->planner) {
      delete ompl;
      return NULL;
    }
    ompl->ReadParameters(*this);
    return ompl;
  }
  else
#endif //HAVE_OMPL
  if(type=="prm") {
    RoadmapPlannerInterface* prm = new RoadmapPlannerInterface(space);
    prm->knn=knn;
    prm->connectionThreshold = connectionThreshold;
    prm->ignoreConnectedComponents = ignoreConnectedComponents;
    prm->storeEdges=storeEdges;
    ReadPointLocation(pointLocation,prm->prm);
    return prm;
  }
  else if(type=="any" || type=="sbl") {
    Real res=gridResolution;
    if(gridResolution <= 0) res = 0.1;
    SBLInterface* sbl = new SBLInterface(space,useGrid,res,randomizeFrequency);
    sbl->sbl->maxExtendDistance = perturbationRadius;
    sbl->sbl->maxExtendIters = perturbationIters;
    sbl->sbl->edgeConnectionThreshold = connectionThreshold;
    return sbl;
  }
  else if(type=="sblprt") {
    SBLPRTInterface* sblprt = new SBLPRTInterface(space);
    sblprt->sblprt.maxExtendDistance = perturbationRadius;
    sblprt->sblprt.maxExtendIters = perturbationIters;
    //Real defaultPPickClosestTree,defaultPPickClosestNode;
    return sblprt;
  }
  else if(type=="rrt") {
    if(bidirectional) {
      BiRRTInterface* rrt = new BiRRTInterface(space);
      rrt->rrt.connectionThreshold = connectionThreshold;
      rrt->rrt.delta = perturbationRadius;
      return rrt;
    }
    else {
      RRTInterface* rrt = new RRTInterface(space);
      rrt->rrt.connectionThreshold = connectionThreshold;
      rrt->rrt.delta = perturbationRadius;
      return rrt;
    }
  }
  else if(type=="prm*") {
    PRMStarInterface* prm = new PRMStarInterface(space);
    prm->planner.lazy = false;
    prm->planner.connectionThreshold = connectionThreshold;
    ReadPointLocation(pointLocation,prm->planner);
    if(shortcut || restart) 
      printf("MotionPlannerInterface: Warning, shortcut and restart are incompatible with PRM* planner\n");
    return prm;
  }
  else if(type=="rrt*") {
    PRMStarInterface* prm = new PRMStarInterface(space);
    prm->planner.rrg = true;
    prm->planner.lazy = false;
    prm->planner.bidirectional = bidirectional;
    prm->planner.connectionThreshold = connectionThreshold;
    prm->planner.suboptimalityFactor = suboptimalityFactor;
    ReadPointLocation(pointLocation,prm->planner);
    if(shortcut || restart) 
      printf("MotionPlannerInterface: Warning, shortcut and restart are incompatible with RRT* planner\n");
    return prm;
  }
  else if(type=="lazyprm*") {
    PRMStarInterface* prm = new PRMStarInterface(space);
    prm->planner.lazy = true;
    prm->planner.connectionThreshold = connectionThreshold;
    prm->planner.suboptimalityFactor = suboptimalityFactor;
    ReadPointLocation(pointLocation,prm->planner);
    if(shortcut || restart) 
      printf("MotionPlannerInterface: Warning, shortcut and restart are incompatible with Lazy-PRM* planner\n");
    return prm;
  }
  else if(type=="lazyrrg*") {
    PRMStarInterface* prm = new PRMStarInterface(space);
    prm->planner.lazy = true;
    prm->planner.rrg = true;
    prm->planner.bidirectional = bidirectional;
    prm->planner.connectionThreshold = connectionThreshold;
    prm->planner.suboptimalityFactor = suboptimalityFactor;
    ReadPointLocation(pointLocation,prm->planner);
    if(shortcut || restart) 
      printf("MotionPlannerInterface: Warning, shortcut and restart are incompatible with Lazy-RRG* planner\n");
    return prm;
  }
  else if(type=="fmm" || type=="fmm*") {
    FMMInterface* fmm = new FMMInterface(space,(type=="fmm*"));
    int d;
    Vector q;
    space->Sample(q);
    d = q.n;
    vector<Real> domainMin, domainMax;
    PropertyMap props;
    space->Properties(props); 
    int euclidean;
    if(props.get("euclidean",euclidean) && euclidean == 0)
      fprintf(stderr,"MotionPlannerFactory: Warning, FMM used in non-euclidean space\n");
    if(props.getArray("minimum",domainMin)) {
      if(domainMin.size()==1)  //single number
	fmm->planner.bmin.resize(d,domainMin[0]);
      else if((int)domainMin.size() == d)
	fmm->planner.bmin = domainMin;
      else if((int)domainMin.size() > d) {
	fmm->planner.bmin = domainMin;
	fmm->planner.bmin.n = d;
      }
      else {
	printf("MotionPlannerInterface: Warning, domainMin is of incorrect size, ignoring\n");
      }
    }
    if(props.getArray("maximum",domainMax)) {
      if(domainMax.size()==1)  //single number
	fmm->planner.bmax.resize(d,domainMax[0]);
      else if((int)domainMax.size() == d)
	fmm->planner.bmax = domainMax;
      else if((int)domainMax.size() > d) {
	fmm->planner.bmax = domainMax;
	fmm->planner.bmax.n = d;
      }
      else {
	printf("MotionPlannerInterface: Warning, domainMax is of incorrect size, ignoring\n");
      }
    }
    if(gridResolution > 0) {
      fmm->planner.resolution.resize(d,gridResolution);
      vector<Real> weights;
      if(props.getArray("metricWeights",weights)) {
	for(int i=0;i<d;i++)
	  fmm->planner.resolution[i] = d/weights[i];
      }
    }
    //TODO: turn off dynamic domain?
    if(!domainMin.empty() && !domainMax.empty() && gridResolution > 0) {
      fmm->planner.dynamicDomain = false;
    }
    if(restart) 
      printf("MotionPlannerInterface: Warning, restart is incompatible with FMM planner\n");
    return fmm;
  }
  else {
    FatalError("MotionPlannerFactory: Planner type %s is unknown",type.c_str());
    return NULL;
  }
}

MotionPlannerInterface* MotionPlannerFactory::Create(CSpace* space,const Config& a,const Config& b)
{
  return Create(MotionPlanningProblem(space,a,b));
}

MotionPlannerInterface* MotionPlannerFactory::Create(CSpace* space,const Config& a,CSpace* goalSpace)
{
  return Create(MotionPlanningProblem(space,a,goalSpace));
}

bool MotionPlannerFactory::Load(TiXmlElement* e)
{
#if HAVE_TINYXML
  string stype;
  if(e->QueryStringAttribute("type",&stype)!=TIXML_SUCCESS) {
    type = "any";
  }
  else {
    type = stype;
    Lowercase(type);
  }
  e->QueryValueAttribute("knn",&knn);
  e->QueryValueAttribute("connectionThreshold",&connectionThreshold);
  e->QueryValueAttribute("suboptimalityFactor",&suboptimalityFactor);
  e->QueryValueAttribute("ignoreConnectedComponents",&ignoreConnectedComponents);
  e->QueryValueAttribute("perturbationRadius",&perturbationRadius);
  e->QueryValueAttribute("perturbationIters",&perturbationIters);
  e->QueryValueAttribute("bidirectional",&bidirectional);
  e->QueryValueAttribute("useGrid",&useGrid);
  e->QueryValueAttribute("gridResolution",&gridResolution);
  e->QueryValueAttribute("randomizeFrequency",&randomizeFrequency);
  e->QueryValueAttribute("storeEdges",&storeEdges);
  e->QueryValueAttribute("shortcut",&shortcut);
  e->QueryValueAttribute("restart",&restart);
  e->QueryValueAttribute("restartTermCond",&restartTermCond);
  if(e->Attribute("pointLocation"))
    pointLocation = e->Attribute("pointLocation");
  return true;
#else
  return false;
#endif //HAVE_TINYXML
}

bool MotionPlannerFactory::Save(TiXmlElement* e)
{
  FatalError("MotionPlannerFactory::Save not defined\n");
  return false;
}

bool MotionPlannerFactory::LoadJSON(const string& str)
{
  AnyCollection items;
  if(!items.read(str.c_str())) return false;
  string typestr;
  if(!items["type"].as(typestr)) return false;
  type = typestr;
  items["knn"].as(knn);
  items["suboptimalityFactor"].as(suboptimalityFactor);
  items["connectionThreshold"].as(connectionThreshold);
  items["ignoreConnectedComponents"].as(ignoreConnectedComponents);
  items["perturbationRadius"].as(perturbationRadius);
  items["perturbationIters"].as(perturbationIters);
  items["bidirectional"].as(bidirectional);
  items["useGrid"].as(useGrid);
  items["pointLocation"].as(pointLocation);
  items["gridResolution"].as(gridResolution);
  items["randomizeFrequency"].as(randomizeFrequency);
  items["storeEdges"].as(storeEdges);
  items["shortcut"].as(shortcut);
  items["restart"].as(restart);
  items["restartTermCond"].as(restartTermCond);
  return true;
}

string MotionPlannerFactory::SaveJSON() const
{
  AnyCollection items;
  ToCollection(*this,items);
  stringstream ss;
  items.write_inline(ss);
  return ss.str();
}




RestartMotionPlanner::RestartMotionPlanner(const MotionPlannerFactory& _factory,const MotionPlanningProblem& _problem,const HaltingCondition& _iterTermCond)
  :PiggybackMotionPlanner(NULL),factory(_factory),problem(_problem),iterTermCond(_iterTermCond),numIters(0)
{
  mp = factory.Create(problem);
}

int RestartMotionPlanner::AddMilestone(const Config& q)
{
  int m = mp->AddMilestone(q);
  //save for later to support point-to-point initialization paradigm:
  //Create(space), AddMilestone(start), AddMilestone(goal)
  if(m == 0) {
    Assert(problem.qstart.empty());
    problem.qstart = q;
  }
  if(m == 1) {
    Assert(problem.qgoal.empty());
    problem.qgoal = q;
  }
  return m;
}

bool RestartMotionPlanner::IsConnected(int ma,int mb) const
{
  if(ma == 0 && mb == 1) return !bestPath.edges.empty();
  return mp->IsConnected(ma,mb);
}

void RestartMotionPlanner::GetPath(int ma,int mb,MilestonePath& path)
{
  if(ma == 0 && mb == 1) path = bestPath;
  else mp->GetPath(ma,mb,path);
}

std::string RestartMotionPlanner::Plan(MilestonePath& path,const HaltingCondition& cond)
{
  if(cond.foundSolution) {
    fprintf(stderr,"RestartMotionPlanner: warning, termination condition wants to stop at first solution. Ignoring\n");
  }
  bestPath.edges.clear();
  Real lastOuterCheckTime = 0, lastOuterCheckValue = 0;
  Timer timer;
  numIters = 0;
  while(numIters<cond.maxIters) {
    Real t=timer.ElapsedTime();
    if(t > cond.timeLimit) {
      path = bestPath;
      return "timeLimit";
    }
    if(!bestPath.edges.empty() && t > lastOuterCheckTime + cond.costImprovementPeriod) {
      Real len = bestPath.Length();
      if(len < cond.costThreshold)
	return "costThreshold";
      if(lastOuterCheckValue - len < cond.costImprovementThreshold) {
	path = bestPath;
	return "costImprovement";
      }
      lastOuterCheckTime = t;
      lastOuterCheckValue = len;
    }

    HaltingCondition myTermCond = iterTermCond;
    if(t + iterTermCond.timeLimit > cond.timeLimit)
      myTermCond.timeLimit = cond.timeLimit - t;
    if(numIters + iterTermCond.maxIters > cond.maxIters)
      myTermCond.maxIters = cond.maxIters - numIters;
    //TODO: cost improvement checking
    mp = factory.Create(problem);
    cout<<"Starting new sub-plan at time "<<t<<", "<<myTermCond.maxIters<<" iters, "<<myTermCond.timeLimit<<" seconds"<<endl;
    mp->Plan(path,myTermCond);
    cout<<"  Result: ";
    if(path.edges.empty())
      cout<<"Failure"<<endl;
    else
      cout<<"Length "<<path.Length()<<endl;
    numIters += mp->NumIterations();
    cout<<"  Expended "<<mp->NumIterations()<<" iterations"<<endl;
    if(!path.edges.empty() && (bestPath.edges.empty() || path.Length() < bestPath.Length())) {
      //update best path
      bestPath = path;
    }
  }
  path = bestPath;
  return "maxIters";  
}

int RestartMotionPlanner::PlanMore()
{
  int res=mp->PlanMore();
  numIters++;
  if(mp->IsSolved()) {
    //update best path
    MilestonePath path;
    mp->GetSolution(path);
    if(bestPath.edges.empty() || path.Length() < bestPath.Length()) {
      bestPath = path;
    }
    if(iterTermCond.foundSolution) {
      //create a new planner
      mp = NULL;
      mp = factory.Create(problem);
      return res;
    }
  }
  if(mp->NumIterations() > iterTermCond.maxIters) {
    //create a new planner
    mp = NULL;
    mp = factory.Create(problem);    
  }
  return res;
}

RestartShortcutMotionPlanner::RestartShortcutMotionPlanner(const MotionPlannerFactory& _factory,const MotionPlanningProblem& _problem,const HaltingCondition& _iterTermCond)
  :RestartMotionPlanner(_factory,_problem,_iterTermCond),shortcutMode(false),numShortcutIters(0)
{
}

std::string RestartShortcutMotionPlanner::Plan(MilestonePath& path,const HaltingCondition& cond)
{
  if(cond.foundSolution) {
    fprintf(stderr,"RestartShortcutMotionPlanner: warning, termination condition wants to stop at first solution. Ignoring\n");
  }
  bestPath.edges.clear();
  candidatePaths.resize(0);
  Real lastOuterCheckTime = 0, lastOuterCheckValue = 0;
  Timer timer;
  numIters = 0;
  while(numIters<cond.maxIters) {
    shortcutMode = false;
    Real t=timer.ElapsedTime();
    if(t > cond.timeLimit) {
      path = bestPath;
      return "timeLimit";
    }
    if(!bestPath.edges.empty() && t > lastOuterCheckTime + cond.costImprovementPeriod) {
      Real len = bestPath.Length();
      if(len < cond.costThreshold)
	return "costThreshold";
      if(lastOuterCheckValue - len < cond.costImprovementThreshold) {
	path = bestPath;
	return "costImprovement";
      }
      lastOuterCheckTime = t;
      lastOuterCheckValue = len;
    }

    Timer mpTimer;
    HaltingCondition myTermCond = iterTermCond;
    if(t + iterTermCond.timeLimit > cond.timeLimit)
      myTermCond.timeLimit = cond.timeLimit - t;
    if(numIters + iterTermCond.maxIters > cond.maxIters)
      myTermCond.maxIters = cond.maxIters - numIters;
    //TODO: cost improvement checking
    mp = factory.Create(problem);
    cout<<"Starting new sub-plan at time "<<t<<", "<<myTermCond.maxIters<<" iters, "<<myTermCond.timeLimit<<" seconds"<<endl;
    mp->Plan(path,myTermCond);
    cout<<"  Result: ";
    if(path.edges.empty())
      cout<<"Failure"<<endl;
    else
      cout<<"Length "<<path.Length()<<endl;
    cout<<"  Expended "<<mp->NumIterations()<<" iterations"<<endl;
    if(!path.edges.empty()) {
      candidatePaths.push_back(path);
      if(bestPath.edges.empty() || path.Length() < bestPath.Length()) {
	//update best path
	bestPath = path;
      }
    }
    if(candidatePaths.empty()) continue;

    //do shortcutting on all candidate paths
    Real bestPathLength = bestPath.Length();
    if(bestPathLength == 0) {
      continue;
    }

    vector<Real> samplingWeights(candidatePaths.size());
    for(size_t i=0;i<candidatePaths.size();i++) 
      samplingWeights[i] = Exp(-(candidatePaths[i].Length()/bestPathLength - 1.0)*3.0);
    numIters += mp->NumIterations();
    int numMpIters = mp->NumIterations();
    while(mpTimer.ElapsedTime() < myTermCond.timeLimit && numMpIters < myTermCond.maxIters) {
      //round robin
      //int index=numMpIters%candidatePaths.size();
      //sample
      int index=WeightedSample(samplingWeights);
      if(candidatePaths[index].Reduce(1)) {
	if(candidatePaths[index].Length() < bestPathLength) {
	  bestPath = candidatePaths[index];
	  bestPathLength = candidatePaths[index].Length();
	  for(size_t i=0;i<candidatePaths.size();i++) 
	    samplingWeights[i] = Exp(-(candidatePaths[i].Length()/bestPathLength - 1.0)*3.0);
	}
      }
      numMpIters++;
      numIters++;
      shortcutMode = true;
    }
  }
  path = bestPath;
  return "maxIters";  
}

int RestartShortcutMotionPlanner::PlanMore()
{
  if(shortcutMode) {
    Assert(!candidatePaths.empty());
    numShortcutIters ++;
    numIters ++;
    Real bestPathLength = bestPath.Length();
    if(bestPathLength == 0) {
      shortcutMode = false;
      return -1;
    }
    vector<Real> samplingWeights(candidatePaths.size());
    for(size_t i=0;i<candidatePaths.size();i++) 
      samplingWeights[i] = Exp(-(candidatePaths[i].Length()/bestPathLength - 1.0)*3.0);
    int index = WeightedSample(samplingWeights);
    //round robin
    //int index = numShortcutIters % candidatePaths.size();
    if(candidatePaths[index].Reduce(1)) {
      if(candidatePaths[index].Length() < bestPathLength) 
	bestPath = candidatePaths[index];
    }
    if(numShortcutIters /*+ mp->NumIterations()*/ >= iterTermCond.maxIters) {
      //finished planning and shortcutting, create a new planner and switch
      //back to planning mode
      mp = NULL;
      mp = factory.Create(problem);    
      shortcutMode = false;
      numShortcutIters = 0;
    }
    return -1;
  }
  else  {
    //regular planning mode
    int res=mp->PlanMore();
    numIters++;
    if(mp->IsSolved()) {
      //update best path
      MilestonePath path;
      mp->GetSolution(path);
      candidatePaths.push_back(path);
      if(bestPath.edges.empty() || path.Length() < bestPath.Length()) {
	bestPath = path;
      }
      shortcutMode = true;
    }
    if(mp->NumIterations() >= iterTermCond.maxIters && !candidatePaths.empty()) {
      shortcutMode = true;
    }
    /*
    if(mp->NumIterations() >= iterTermCond.maxIters) {
      //create a new planner
      mp = NULL;
      mp = factory.Create(problem);    
    }
    */
    return res;
  }
}

void RestartShortcutMotionPlanner::GetRoadmap(RoadmapPlanner& roadmap) const
{
  if(candidatePaths.empty()) {
    mp->GetRoadmap(roadmap);
    return;
  }
  for(size_t i=0;i<candidatePaths.size();i++) {
    int prev = roadmap.AddMilestone(candidatePaths[i].GetMilestone(0));
    for(size_t j=0;j<candidatePaths[i].edges.size();j++) {
      int n = roadmap.AddMilestone(candidatePaths[i].GetMilestone(j+1));
      roadmap.ConnectEdge(prev,n,candidatePaths[i].edges[j]);
      prev = n;
    }
  }
}


ShortcutMotionPlanner::ShortcutMotionPlanner(const SmartPointer<MotionPlannerInterface>& mp)
  :PiggybackMotionPlanner(mp),numIters(0)
{}

std::string ShortcutMotionPlanner::Plan(MilestonePath& path,const HaltingCondition& cond)
{
  Timer timer;
  string res = mp->Plan(path,cond);
  numIters = mp->NumIterations();
  if(res == "maxIters" || res == "timeLimit") {
    printf("Shortcutting not started\n");
    return res;  //out of time or iterations
  }
  assert(!path.edges.empty());
  int itersLeft = cond.maxIters - mp->NumIterations(); 
  Real lastCheckTime = timer.ElapsedTime(), lastCheckValue = path.Length();
  printf("Beginning shortcutting with %d iters and %g seconds left\n",itersLeft,cond.timeLimit-timer.ElapsedTime());
  for(int iters=0;iters<itersLeft;iters++) {
    Real t = timer.ElapsedTime();
    if(t >= cond.timeLimit) {
      bestPath = path;
      return "timeLimit";
    }
    //check for cost improvements
    if(t > lastCheckTime + cond.costImprovementPeriod) {
      Real len = path.Length();
      if(lastCheckValue - len < cond.costImprovementThreshold)
	return "costImprovementThreshold";
      lastCheckTime = t;
      lastCheckValue = len;
    }
    //do shortcutting
    path.Reduce(1);
    numIters ++;
  }
  bestPath = path;
  return "maxIters";
}

int ShortcutMotionPlanner::PlanMore()
{
  numIters++;
  if(bestPath.edges.empty()) {
    int res = mp->PlanMore();
    if(mp->IsSolved()) {
      mp->GetSolution(bestPath);
    }
    return res;
  }
  else {
    bestPath.Reduce(1);
    return -1;
  }
}
