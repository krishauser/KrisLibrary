#include "AnyMotionPlanner.h"
#include <utils/AnyCollection.h>
#include <utils/stringutils.h>
#include "OptimalMotionPlanner.h"
#include "PointLocation.h"
#include "SBL.h"
#include "Timer.h"

#if HAVE_TINYXML
#include <tinyxml.h>
#endif

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
    if(t > cond.timeLimit) return "timeLimit";
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
  return "maxIters";
}

class RoadmapPlannerInterface  : public MotionPlannerInterface
{
 public:
  RoadmapPlannerInterface(CSpace* space)
    : prm(space),knn(10),connectionThreshold(Inf),numIters(0),ignoreConnectedComponents(false),storeEdges(true)
    {}
  virtual ~RoadmapPlannerInterface() {}
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
    bool res=prm.TestAndConnectEdge(i,j);
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
  virtual bool IsSolved() const { return false; }
  virtual void GetSolution(MilestonePath& path) { return; }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) { roadmap = prm; }

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

void GetRoadmap(TreeRoadmapPlanner& planner,RoadmapPlanner& roadmap)
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
  virtual void GetRoadmap(RoadmapPlanner& roadmap) { ::GetRoadmap(rrt,roadmap); }

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
  virtual void GetRoadmap(RoadmapPlanner& roadmap) { ::GetRoadmap(rrt,roadmap); }

  BidirectionalRRTPlanner rrt;
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
    if(!sbl->IsDone()) sbl->Extend();
    return -1;
  }
  virtual int NumIterations() const { return sbl->numIters; }
  virtual int NumMilestones() const {
    Graph::CountCallback<SBLTree::Node*> cb1,cb2;
    sbl->tStart->root->DFS(cb1);
    sbl->tGoal->root->DFS(cb2);
    return cb1.count+cb2.count;
  }
  virtual int NumComponents() const { return 2; }
  virtual bool IsConnected(int ma,int mb) const { return sbl->IsDone(); }
  virtual void GetPath(int ma,int mb,MilestonePath& path) { sbl->CreatePath(path); if(ma == 1) ReversePath(path); }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) { printf("TODO: get SBL roadmap\n"); }

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
  virtual int AddMilestone(const Config& q) { return sblprt.AddSeed(q); }
  virtual void GetMilestone(int i,Config& q) { q=*sblprt.roadmap.nodes[i]->root; }
  virtual void ConnectHint(int i) {
    for(size_t j=0;j<sblprt.roadmap.nodes.size();j++)
      sblprt.AddRoadmapEdge(i,j);
  }
  virtual bool ConnectHint(int i,int j) { sblprt.AddRoadmapEdge(i,j); return false; }
  virtual int PlanMore() { 
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


class PRMStarInterface  : public MotionPlannerInterface
{
 public:
  PRMStarInterface(CSpace* space)
    : planner(space)
    {}
  virtual ~PRMStarInterface() {}
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
    planner.PlanMore();
    return -1;
  }
  virtual int NumIterations() const { return planner.numPlanSteps; }
  virtual int NumMilestones() const { return planner.roadmap.nodes.size(); }
  virtual int NumComponents() const { return 1; }
  virtual bool IsConnected(int ma,int mb) const { 
    Assert(ma==0 && mb==1);
    if(IsInf(planner.spp.d[mb])) return false;
    return true;
  }
  virtual void GetPath(int ma,int mb,MilestonePath& path) {
    Assert(ma==0 && mb==1);
    if(IsInf(planner.spp.d[mb])) return;
    planner.GetPath(path);
  }
  virtual void GetRoadmap(RoadmapPlanner& roadmap) { roadmap.roadmap = planner.roadmap; }

  PRMStarPlanner planner;
  Config qStart,qGoal;
};

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
	AddMilestone(q);
	int n=mp->AddMilestone(q);
	goalNodes.push_back(n);
	return n;
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

MotionPlannerFactory::MotionPlannerFactory()
  :type("any"),
   knn(10),
   connectionThreshold(Inf),
   ignoreConnectedComponents(false),
   perturbationRadius(0.1),perturbationIters(5),
   bidirectional(true),
   useGrid(true),gridResolution(0.1),randomizeFrequency(50),
   storeEdges(false),shortcut(false),restart(false)
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

    PointToSetMotionPlanner* psmp = new PointToSetMotionPlanner(mp,problem.qstart,problem.goalSet);
    return psmp;
  }
  else {
    MotionPlannerInterface* mp = CreateRaw(problem.space);
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
    return new RestartMotionPlanner(norestart,problem,iterTerm);
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
  else if(shortcut)
    return new ShortcutMotionPlanner(planner);
  else
    return planner;
}

MotionPlannerInterface* MotionPlannerFactory::CreateRaw(CSpace* space)
{
  Lowercase(type);
  if(type=="prm") {
    RoadmapPlannerInterface* prm = new RoadmapPlannerInterface(space);
    prm->knn=knn;
    prm->connectionThreshold = connectionThreshold;
    prm->ignoreConnectedComponents = ignoreConnectedComponents;
    prm->storeEdges=storeEdges;
    return prm;
  }
  else if(type=="any" || type=="sbl") {
    SBLInterface* sbl = new SBLInterface(space,useGrid,gridResolution,randomizeFrequency);
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
    if(shortcut || restart) 
      printf("MotionPlannerInterface: Warning, shortcut and restart are incompatible with PRM* planner\n");
    return prm;
  }
  else if(type=="lazyprm*") {
    PRMStarInterface* prm = new PRMStarInterface(space);
    prm->planner.lazy = true;
    prm->planner.connectionThreshold = connectionThreshold;
    if(shortcut || restart) 
      printf("MotionPlannerInterface: Warning, shortcut and restart are incompatible with Lazy-PRM* planner\n");
    return prm;
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

PointToSetMotionPlanner* MotionPlannerFactory::Create(CSpace* space,const Config& a,CSpace* goalSpace)
{
  return dynamic_cast<PointToSetMotionPlanner*>(Create(MotionPlanningProblem(space,a,goalSpace)));
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
  items["connectionThreshold"].as(connectionThreshold);
  items["ignoreConnectedComponents"].as(ignoreConnectedComponents);
  items["perturbationRadius"].as(perturbationRadius);
  items["perturbationIters"].as(perturbationIters);
  items["bidirectional"].as(bidirectional);
  items["useGrid"].as(useGrid);
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
  items["type"] = type;
  items["knn"] = knn;
  items["connectionThreshold"] = connectionThreshold;
  items["ignoreConnectedComponents"] = ignoreConnectedComponents;
  items["perturbationRadius"] = perturbationRadius;
  items["perturbationIters"] = perturbationIters;
  items["bidirectional"] = bidirectional;
  items["useGrid"] = useGrid;
  items["gridResolution"] = gridResolution;
  items["randomizeFrequency"] = randomizeFrequency;
  items["storeEdges"] = storeEdges;
  items["shortcut"] = shortcut;
  items["restart"] = restart;
  items["restartTermCond"] = restartTermCond;
  stringstream ss;
  items.write_inline(ss);
  return ss.str();
}




RestartMotionPlanner::RestartMotionPlanner(const MotionPlannerFactory& _factory,const MotionPlanningProblem& _problem,const HaltingCondition& _iterTermCond)
  :PiggybackMotionPlanner(NULL),factory(_factory),problem(_problem),iterTermCond(_iterTermCond),numIters(0)
{}

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
  if(!mp)
    mp = factory.Create(problem);
  int res=mp->PlanMore();
  numIters++;
  if(iterTermCond.foundSolution) {
    if(mp->IsSolved()) {
      MilestonePath path;
      mp->GetSolution(path);
      if(path.Length() < bestPath.Length())
	bestPath = path;
    }
    mp = NULL;
  }
  return res;
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
