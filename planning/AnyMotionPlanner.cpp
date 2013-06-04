#include "AnyMotionPlanner.h"
#include "SBL.h"

#if HAVE_TINYXML
#include <tinyxml/tinyxml.h>
#endif

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

  SBLPRT sblprt;
};


MotionPlannerFactory::MotionPlannerFactory()
  :type(PRM),
   knn(10),
   connectionThreshold(Inf),
   ignoreConnectedComponents(false),
   perturbationRadius(0.1),perturbationIters(5),
   bidirectional(true),
   useGrid(true),gridResolution(0.1),randomizeFrequency(50),
   storeEdges(false)
{}

MotionPlannerInterface* MotionPlannerFactory::Create(CSpace* space)
{
  switch(type) {
  case PRM:
    {
    RoadmapPlannerInterface* prm = new RoadmapPlannerInterface(space);
    prm->knn=knn;
    prm->connectionThreshold = connectionThreshold;
    prm->ignoreConnectedComponents = ignoreConnectedComponents;
    prm->storeEdges=storeEdges;
    return prm;
    }
  case SBL:
    {
    SBLInterface* sbl = new SBLInterface(space,useGrid,gridResolution,randomizeFrequency);
    sbl->sbl->maxExtendDistance = perturbationRadius;
    sbl->sbl->maxExtendIters = perturbationIters;
    sbl->sbl->edgeConnectionThreshold = connectionThreshold;
    return sbl;
    }
  case SBLPRT:
    {
      SBLPRTInterface* sblprt = new SBLPRTInterface(space);
      sblprt->sblprt.maxExtendDistance = perturbationRadius;
      sblprt->sblprt.maxExtendIters = perturbationIters;
      //Real defaultPPickClosestTree,defaultPPickClosestNode;
      return sblprt;
    }
  case RRT:
    {
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
  default:
    FatalError("MotionPlannerFactory: That interface is not done");
    return NULL;
  }
}



bool MotionPlannerFactory::Load(TiXmlElement* e)
{
#if HAVE_TINYXML
  string stype;
  if(e->QueryStringAttribute("type",&stype)!=TIXML_SUCCESS) {
    printf("Could not read motion planner factory attribute type\n");
    return false;
  }
  if(stype == "prm")
    type = MotionPlannerFactory::PRM;
  else if(stype == "sbl")
    type = MotionPlannerFactory::SBL;
  else if(stype == "rrt")
    type = MotionPlannerFactory::RRT;
  else if(stype == "sblprt")
    type = MotionPlannerFactory::SBLPRT;
  else if(stype == "est")
    type = MotionPlannerFactory::EST;
  else if(stype == "perturbation")
    type = MotionPlannerFactory::PerturbationTree;
  else if(stype == "lazyprm")
    type = MotionPlannerFactory::LazyPRM;
  else {
    fprintf(stderr,"MotionPlannerFactory::Load: Unsupported motion planner type %s\n",stype.c_str());
    return false;
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
