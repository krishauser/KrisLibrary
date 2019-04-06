#include <KrisLibrary/Logger.h>
#include "KinodynamicMotionPlanner.h"
#include "CSetHelpers.h"
#include <KrisLibrary/geometry/KDTree.h>
#include <KrisLibrary/geometry/BallTree.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/graph/Callback.h>
#include <algorithm>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/utils/EZTrace.h>
#include <KrisLibrary/math/random.h>
#include <string.h>
using namespace std;

typedef KinodynamicTree::Node Node;

struct VectorizeCallback : public Node::Callback
{
  virtual void Visit(Node* n) { 
    nodes.push_back(n);
  }

  vector<Node*> nodes;
};










KinodynamicTree::KinodynamicTree(KinodynamicSpace* s)
  :space(s),root(NULL)
{
}

KinodynamicTree::~KinodynamicTree()
{
  SafeDelete(root);
}

void KinodynamicTree::Init(const State& initialState)
{
  Clear();
  root = new Node(initialState);

  if(pointLocation) {
    index.push_back(root);
    pointRefs.resize(pointRefs.size()+1);
    pointRefs.back().setRef(*root);
    pointLocation->OnAppend();
  }
}

void KinodynamicTree::EnablePointLocation(const char* type)
{ 
  if(type == NULL) 
    pointLocation = make_shared<NaivePointLocation>(pointRefs,space->GetStateSpace().get());
  else if(0==strcmp(type,"kdtree")) {
    PropertyMap props;
    space->Properties(props);
    int euclidean;
    if(props.get("euclidean",euclidean) && euclidean == 0)
            LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicTree: Warning, requesting K-D tree point location for non-euclidean space");

    vector<Real> weights;
    if(props.getArray("metricWeights",weights)) {
      pointLocation = make_shared<KDTreePointLocation>(pointRefs,2,Vector(weights));
    }
    else {
      pointLocation = make_shared<KDTreePointLocation>(pointRefs);
    }
  }
  else if(0==strcmp(type,"balltree")) {
    pointLocation = make_shared<BallTreePointLocation>(space->GetStateSpace().get(),pointRefs);
  }
  else if(0==strcmp(type,"random"))
    pointLocation = make_shared<RandomPointLocation>(pointRefs);
  else
    FatalError("Invalid point location method %s\n",type);
  //rebuild point location data structures
  LOG4CXX_INFO(KrisLibrary::logger(),"Rebuilding point location data structures");
  VectorizeCallback callback;
  if(root) root->DFS(callback);
  index = callback.nodes;
  pointRefs.clear();
  pointRefs.reserve(index.size());
  pointLocation->OnClear();
  for(size_t i=0;i<index.size();i++) {
    pointRefs.resize(pointRefs.size()+1);
    pointRefs[i].setRef(*index[i]);
    pointLocation->OnAppend();
  }
}

void KinodynamicTree::Clear()
{
  index.clear();
  SafeDelete(root);
  pointRefs.clear();
  if(pointLocation)
    pointLocation->OnClear();
}


Node* KinodynamicTree::AddMilestone(Node* parent, const ControlInput& u)
{
  KinodynamicMilestonePath path;
  path.milestones.push_back(*parent);
  path.controls.push_back(u);
  path.SimulateFromControls(space);
  return AddMilestone(parent,path,path.edges[0]);
}

Node* KinodynamicTree::AddMilestone(Node* parent,const ControlInput& u,const std::shared_ptr<Interpolator>& path,const EdgePlannerPtr& e)
{
  Node* c;
  if(e->Start() == *parent) {
    c=parent->addChild(e->End());
  }
  else {
    Assert(e->End() == *parent);
    c=parent->addChild(e->Start());
  }
  c->edgeFromParent().path = KinodynamicMilestonePath(u,path);
  c->edgeFromParent().checker = e;
  if(pointLocation) {
    index.push_back(c);
    pointRefs.resize(pointRefs.size()+1);
    pointRefs.back().setRef(*c);
    pointLocation->OnAppend();
  }
  return c;
}

Node* KinodynamicTree::AddMilestone(Node* parent,KinodynamicMilestonePath& path,const EdgePlannerPtr& e)
{
  assert(*parent == path.milestones[0]);
  Node* c;
  c = parent->addChild(path.End());
  c->edgeFromParent().path = path;
  if(e == NULL) c->edgeFromParent().checker = space->TrajectoryChecker(c->edgeFromParent().path);
  else c->edgeFromParent().checker = e;
  if(pointLocation) {
    index.push_back(c);
    pointRefs.resize(pointRefs.size()+1);
    pointRefs.back().setRef(*c);
    pointLocation->OnAppend();
  }
  return c;
}

void KinodynamicTree::AddPath(Node* n0,const KinodynamicMilestonePath& path,std::vector<Node*>& res)
{
  Assert(*n0 == path.milestones[0]);
  res.resize(0);
  for(size_t i=0;i<path.edges.size();i++) {
    res.push_back(AddMilestone(n0,path.controls[i],path.paths[i],path.edges[i]));
    n0=res.back();
  }
}

void KinodynamicTree::Reroot(Node* n)
{
  FatalError("TODO: KinodynamicTree::Reroot");
}

Node* KinodynamicTree::FindClosest(const State& x)
{
  if(!pointLocation) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicTree::FindClosest: Warning, point location not enabled, now enabling it");
    EnablePointLocation();
  }
  int id;
  Real dist;
  if(!pointLocation->NN(x,id,dist)) return NULL;
  return index[id];
}

Node* KinodynamicTree::PickRandom()
{
  if(!pointLocation) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"KinodynamicTree::PickRandom: Warning, point location not enabled, now enabling it");
    EnablePointLocation();
  }
  return index[RandInt(index.size())];
}

void KinodynamicTree::GetPath(Node* start,Node* goal,KinodynamicMilestonePath& path)
{
  vector<Node*> npath;
  Node* n=goal;
  while(n != start) {
    if(n == NULL) FatalError("KinodynamicTree::GetPath: nodes specified on tree are not valid!");
    npath.push_back(n);

    n = n->getParent();
  }
  npath.push_back(start);
  reverse(npath.begin(),npath.end());

  path.Clear();
  for(size_t i=1;i<npath.size();i++)
    path.Concat(npath[i]->edgeFromParent().path);
}

void KinodynamicTree::DeleteSubTree(Node* n,bool rebuild)
{
  //EZCallTrace tr("KinodynamicTree::DeleteSubTree()");
  if(n == root) root = NULL;
  Node* p=n->getParent();
  if(p) p->detachChild(n);
  Node* c = p->getFirstChild();
  while(c) {
    Assert(c != n);
    c = c->getNextSibling();
  }
  delete n;  //this automatically deletes n and all children

  if(rebuild && pointLocation) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Rebuilding point location data structure on subtree delete..");
    RebuildPointLocation();
  }
}

void KinodynamicTree::RebuildPointLocation()
{
  if(pointLocation) {
    //TODO: faster deletion, if this is a small subtree
    VectorizeCallback callback;
    if(root) root->DFS(callback);
    index = callback.nodes;

    //rebuild point location data structures
    pointRefs.clear();
    pointRefs.reserve(index.size());
    pointLocation->OnClear();
    for(size_t i=0;i<index.size();i++) {
      pointRefs.resize(pointRefs.size()+1);
      pointRefs[i].setRef(*index[i]);
      pointLocation->OnAppend();
    }
  }
}



KinodynamicPlannerBase::KinodynamicPlannerBase(KinodynamicSpace* s)
  :space(s),goalSet(NULL)
{}

void KinodynamicPlannerBase::Init(const State& xinit,const State& xgoal,Real goalRadius)
{
  Init(xinit,new NeighborhoodSet(space->GetStateSpace().get(),xgoal,goalRadius));
}


RRTKinodynamicPlanner::RRTKinodynamicPlanner(KinodynamicSpace* s)
  :KinodynamicPlannerBase(s),goalSeekProbability(0.1),tree(s),delta(Inf),goalNode(NULL),
  numIters(0),numInfeasibleControls(0),numInfeasibleEndpoints(0),numFilteredExtensions(0),numSuccessfulExtensions(0),
  nnTime(0),pickControlTime(0),visibleTime(0),overheadTime(0)
{}

bool RRTKinodynamicPlanner::Plan(int maxIters)
{
  if(goalNode) return true;
  if(!goalSet) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!");
    KrisLibrary::loggerWait();
  }
  for(int i=0;i<maxIters;i++) {
    numIters++;
    Node* n=Extend();
    Timer timer;
    if(n && goalSet && goalSet->Contains(*n)) {
      overheadTime += timer.ElapsedTime();
      goalNode = n;
      return true;
    }
    overheadTime += timer.ElapsedTime();
  }
  return false;
}

void RRTKinodynamicPlanner::GetStats(PropertyMap& stats) const
{
  stats.set("numIters",numIters);
  stats.set("numInfeasibleControls",numInfeasibleControls);
  stats.set("numInfeasibleEndpoints",numInfeasibleEndpoints);
  stats.set("numFilteredExtensions",numFilteredExtensions);
  stats.set("numSuccessfulExtensions",numSuccessfulExtensions);
  stats.set("nnTime",nnTime);
  stats.set("pickControlTime",pickControlTime);
  stats.set("visibleTime",visibleTime);
  stats.set("overheadTime",overheadTime);
  PropertyMap nnstats;
  tree.pointLocation->GetStats(nnstats);
  if(!nnstats.empty())
    stats.set("nnStats",nnstats);
}

void RRTKinodynamicPlanner::Init(const State& xinit,CSet* _goalSet)
{
  Timer timer;
  goalNode = NULL;
  tree.Init(xinit);
  goalSet = _goalSet;
  if(goalSet->Contains(xinit)) goalNode = tree.root;
  if(!tree.pointLocation) {
    //tree.EnablePointLocation("kdtree");
    tree.EnablePointLocation(NULL);
  }
  overheadTime += timer.ElapsedTime();
}

void RRTKinodynamicPlanner::PickDestination(State& xdest)
{
  if(goalSet && goalSet->IsSampleable() && RandBool(goalSeekProbability)) {
    goalSet->Sample(xdest);
    if(!goalSet->Project(xdest)) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: goal set doesnt sample a feasible configuration");
    }
  }
  else {
    space->GetStateSpace()->Sample(xdest);
  }
}

Node* RRTKinodynamicPlanner::Extend()
{
  Timer timer;
  State xdest;
  PickDestination(xdest);
  overheadTime += timer.ElapsedTime();
  return ExtendToward(xdest);
}


Node* RRTKinodynamicPlanner::ExtendToward(const State& xdest)
{
  Timer timer;
  //EZCallTrace tr("RRTKinodynamicPlanner::Extend()");
  Node* n=tree.FindClosest(xdest);
  nnTime += timer.ElapsedTime();
  timer.Reset();
  Vector temp = xdest;
  Real d = space->GetStateSpace()->Distance(*n,xdest);
  if(d > delta)  {
    space->GetStateSpace()->Interpolate(*n,xdest,delta/d,temp);
  }
  KinodynamicMilestonePath path;
  if(!PickControl(*n,temp,path)) {
    numInfeasibleControls++;
    pickControlTime += timer.ElapsedTime();
    return NULL;
  }
  pickControlTime += timer.ElapsedTime();
  timer.Reset();
  if(!space->GetStateSpace()->IsFeasible(path.End())) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Edge endpoint is not feasible");
    visibleTime += timer.ElapsedTime();
    numInfeasibleEndpoints++;
    return NULL;
  }
  visibleTime += timer.ElapsedTime();
  timer.Reset();

  if(FilterExtension(n,path)) {
    numFilteredExtensions++;
    return NULL;
  }

  timer.Reset();
  EdgePlannerPtr e(space->TrajectoryChecker(path));
  if(e->IsVisible()) {
    numSuccessfulExtensions++;
    visibleTime += timer.ElapsedTime();
    timer.Reset();
    Node *c = tree.AddMilestone(n,path,e);
    overheadTime += timer.ElapsedTime();
    return c;
  }
  else {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Edge is not visible");
    visibleTime += timer.ElapsedTime();
    return NULL;
  }
}

bool RRTKinodynamicPlanner::Done() const
{
  return goalNode != NULL;
}

bool RRTKinodynamicPlanner::GetPath(KinodynamicMilestonePath& path)
{
  if(goalNode == NULL) return false;
  tree.GetPath(tree.root,goalNode,path);
  return true;
}

bool RRTKinodynamicPlanner::PickControl(const State& x0, const State& xDest, KinodynamicMilestonePath& path)
{
  std::shared_ptr<SteeringFunction> sf = space->GetControlSpace()->GetSteeringFunction();
  if(!sf)  {
    int nd = space->GetControlSet(x0)->NumDimensions();
    if(nd < 0) nd = space->GetStateSpace()->NumDimensions();
    sf = make_shared<RandomBiasSteeringFunction>(space,3*nd);
  }
  if(sf->Connect(x0,xDest,path)) {
    //steering function is assumed to create valid controls
    for(size_t i=0;i<path.controls.size();i++) 
      assert(space->IsValidControl(path.milestones[i],path.controls[i]));
    path.MakeEdges(space);
    Assert(path.Start() == x0);
    Assert(path.End().n == space->GetStateSpace()->NumDimensions());
    return true;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Failed to connect "<<x0<<" toward "<<xDest);
  }
  return false;
}


ESTKinodynamicPlanner::ESTKinodynamicPlanner(KinodynamicSpace* s)
:KinodynamicPlannerBase(s),tree(s),extensionCacheSize(0), goalNode(NULL),
numIters(0),numFilteredExtensions(0),numSuccessfulExtensions(0),
sampleTime(0),simulateTime(0),visibleTime(0),overheadTime(0)
{
  if(s)
    densityEstimator = make_shared<MultiGridDensityEstimator>(s->GetStateSpace()->NumDimensions(),3,0.1);
}

void ESTKinodynamicPlanner::EnableExtensionCaching(int cacheSize)
{
  extensionCacheSize = cacheSize;
}

void ESTKinodynamicPlanner::SetDensityEstimatorResolution(Real res)
{
  Assert(space != NULL);
  SetDensityEstimatorResolution(Vector(space->GetStateSpace()->NumDimensions(),res));
}

void ESTKinodynamicPlanner::SetDensityEstimatorResolution(const Vector& res)
{
  Assert(space != NULL);
  Assert(res.n == space->GetStateSpace()->NumDimensions());
  if(densityEstimator == NULL)
    densityEstimator = make_shared<MultiGridDensityEstimator>(space->GetStateSpace()->NumDimensions(),3,res);
  else {
    MultiGridDensityEstimator* mgrid = dynamic_cast<MultiGridDensityEstimator*>(densityEstimator.get());
    Assert(mgrid != NULL);
    mgrid->numDims = space->GetStateSpace()->NumDimensions();
    mgrid->h = res;
    mgrid->Randomize();
  }
  RebuildDensityEstimator();
}

void ESTKinodynamicPlanner::RebuildDensityEstimator()
{
  densityEstimator->Clear();
  //Randomize destroys the contents of the estimator, need to re-add the nodes
  VectorizeCallback callback;
  if(tree.root) tree.root->DFS(callback);
  for(size_t i=0;i<callback.nodes.size();i++)
    densityEstimator->Add(*callback.nodes[i],callback.nodes[i]);
}

void ESTKinodynamicPlanner::Init(const State& xinit,CSet* _goalSet)
{
  goalNode = NULL;
  tree.Init(xinit);
  goalSet = _goalSet;
  if(goalSet->Contains(xinit)) goalNode = tree.root;
  densityEstimator->Add(xinit,tree.root);
}

bool ESTKinodynamicPlanner::Plan(int maxIters)
{
  //if(goalNode != NULL) return true;

  const static bool precheckCachedExtensions = false;
  const static int gESTNumControlSamplesPerNode = 1;
  std::shared_ptr<ControlSpace> controlSpace = space->GetControlSpace();
  for(int iters=0;iters<maxIters;iters++) {
    numIters++;
    int numNodeSamples = extensionCacheSize - (int)extensionCache.size();
    if(extensionCacheSize == 0) numNodeSamples = 1;
    for(int nsample=0;nsample<numNodeSamples;nsample++) {
      Node* n = (Node*)densityEstimator->Random();
      if(n == NULL) 
        FatalError("DensityEstimator random selection returned NULL? was the tree not initialized?");
      std::shared_ptr<CSet> uspace = controlSpace->GetControlSet(*n);
      ControlInput u;
      for(int sample=0;sample<gESTNumControlSamplesPerNode;sample++) {
        uspace->Sample(u);
        if(!uspace->Contains(u)) continue;
        Node* c = tree.AddMilestone(n,u);
        if(!c) continue;
        //TODO: test whether prechecking the edges is useful
        if(FilterExtension(n,c->edgeFromParent().path)) {
          numFilteredExtensions++;
          tree.DeleteSubTree(c);
          continue;
        }
        if(!space->GetStateSpace()->IsFeasible(*c) || (precheckCachedExtensions && !c->edgeFromParent().checker->IsVisible())) {
          tree.DeleteSubTree(c);
          continue;
        }
        if(goalSet->Contains(*c)) {
          goalNode = c;
          return true;
        }
        extensionWeights.push_back(1.0/(1.0+densityEstimator->Density(*c)));
        extensionCache.push_back(c);
      }
    }
    if(extensionCache.empty()) {
      continue;
    }

    //sample one of the extensions proportional to weight
    int selection = WeightedSample(extensionWeights);

    Node* c = extensionCache[selection];
    //remove it from the cache
    extensionCache[selection] = extensionCache.back();
    extensionWeights[selection] = extensionWeights.back();
    extensionCache.resize(extensionCache.size()-1);
    extensionWeights.resize(extensionWeights.size()-1);

    //check the path and add it if feasible
    if(!precheckCachedExtensions) {
      if(!c->edgeFromParent().checker->IsVisible()) {
        tree.DeleteSubTree(c);
        continue;
      }
    }
    //it's a good extension, add it as a candidate to be expanded
    densityEstimator->Add(*c,c);
    numSuccessfulExtensions++;
  }
  return false;
}

bool ESTKinodynamicPlanner::Done() const
{
  return goalNode != NULL;
}

bool ESTKinodynamicPlanner::GetPath(KinodynamicMilestonePath& path)
{
  if(goalNode == NULL) return false;
  tree.GetPath(tree.root,goalNode,path);
  return true;
}

void ESTKinodynamicPlanner::GetStats(PropertyMap& stats) const
{
  stats.set("numIters",numIters);
  stats.set("numFilteredExtensions",numFilteredExtensions);
  stats.set("numSuccessfulExtensions",numSuccessfulExtensions);
}




LazyRRTKinodynamicPlanner::LazyRRTKinodynamicPlanner(KinodynamicSpace* s)
  :RRTKinodynamicPlanner(s)
{}

bool LazyRRTKinodynamicPlanner::Plan(int maxIters)
{
  if(!goalSet) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"LazyRRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!");
    KrisLibrary::loggerWait();
  }
  if(goalNode) return true;
  if(goalSet && goalSet->Contains(*tree.root)) {
    goalNode = tree.root;
    return true;
  }
  for(int i=0;i<maxIters;i++) {
    Node* n=Extend();
    Timer timer;
    if(n && goalSet && goalSet->Contains(*n)) {
      if(CheckPath(n)) {
        overheadTime += timer.ElapsedTime();
	goalNode = n;
	return true;
      }
    }
    else {
      overheadTime += timer.ElapsedTime();
    }
  }
  return false;
}

Node* LazyRRTKinodynamicPlanner::ExtendToward(const State& xdest)
{
  Timer timer;
  //EZCallTrace tr("RRTKinodynamicPlanner::Extend()");
  Node* n=tree.FindClosest(xdest);
  nnTime += timer.ElapsedTime();
  timer.Reset();
  Vector temp = xdest;
  Real d = space->GetStateSpace()->Distance(*n,xdest);
  if(d > delta)  {
    space->GetStateSpace()->Interpolate(*n,xdest,delta/d,temp);
  }
  KinodynamicMilestonePath path;
  if(!PickControl(*n,temp,path)) {
    numInfeasibleControls++;
    pickControlTime += timer.ElapsedTime();
    return NULL;
  }
  pickControlTime += timer.ElapsedTime();
  timer.Reset();
  if(!space->GetStateSpace()->IsFeasible(path.End())) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Edge endpoint is not feasible");
    visibleTime += timer.ElapsedTime();
    numInfeasibleEndpoints++;
    return NULL;
  }
  visibleTime += timer.ElapsedTime();
  timer.Reset();

  if(FilterExtension(n,path)) {
    numFilteredExtensions++;
    return NULL;
  }

  EdgePlannerPtr e(space->TrajectoryChecker(path));
  numSuccessfulExtensions++;
  return tree.AddMilestone(n,path,e);
}


struct NodeWithPriority
{
  Node* n;

  NodeWithPriority(Node* _n) :n(_n) {}
  operator Node* () const { return n; }
  Node* operator -> () const { return n; }
  inline bool operator < (const NodeWithPriority& b) const {
    return n->edgeFromParent().checker->Priority() < b->edgeFromParent().checker->Priority();
  }
};

bool LazyRRTKinodynamicPlanner::CheckPath(Node* n)
{
  //EZCallTrace tr("LazyRRTKinodynamicPlanner::CheckPath()");
  //add all nodes up to n, except root
  if(!n->getParent()) return true;
  if(!CheckPath(n->getParent())) return false;
  if(!n->edgeFromParent().checker->IsVisible()) {
    Timer timer;
    tree.DeleteSubTree(n);
    overheadTime += timer.ElapsedTime();
    return false;
  }
  return true;

  priority_queue<NodeWithPriority,vector<NodeWithPriority> > q;
  while(n != tree.root) {
    q.push(n);
    n = n->getParent();
    Assert(n != NULL);
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"LazyRRTKinodynamicPlanner::CheckPath: "<<q.size());
  while(!q.empty()) {
    n=q.top(); q.pop();
    EdgePlannerPtr& e=n->edgeFromParent().checker;
    if(!e->Done()) {
      if(!e->Plan()) {
        LOG4CXX_INFO(KrisLibrary::logger(),"Edge found infeasible, deleting");
	//disconnect n from the rest of the tree
  Timer timer;
	tree.DeleteSubTree(n);
  overheadTime += timer.ElapsedTime();
	return false;
      }
      q.push(n);
    }
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Path checking successful!");
  return true;
}






BidirectionalRRTKP::BidirectionalRRTKP(KinodynamicSpace* s)
  :KinodynamicPlannerBase(s),start(s),goal(s),connectionTolerance(1.0)
{
  bridge.nStart=NULL;
  bridge.nGoal=NULL;
}

void BidirectionalRRTKP::Init(const State& xinit,const State& xgoal)
{
  start.Init(xinit);
  goal.Init(xgoal);

  bridge.nStart=NULL;
  bridge.nGoal=NULL;

  start.EnablePointLocation("kdtree");
  goal.EnablePointLocation("kdtree");
}


void BidirectionalRRTKP::Init(const State& xinit,CSet* goalSet)
{
  if(goalSet->Contains(xinit)) {
    Init(xinit,xinit);
    return;
  }
  Vector x;
  for(int i=0;i<100;i++) {
    goalSet->Sample(x);
    if(goalSet->Contains(x) || goalSet->Project(x)) {
      Init(xinit,x);
      return;
    }
  }
  FatalError("Could not initialize Bidirectional kinodynamic RRT, goal set doesn't seem to have any feasible points");
}

bool BidirectionalRRTKP::Done() const
{
  return bridge.nStart != NULL;
}

bool BidirectionalRRTKP::Plan(int maxIters)
{
  std::shared_ptr<CSpace> stateSpace = space->GetStateSpace();
  for(int iters=0;iters<maxIters;iters++) {
    if(RandBool()) {
      Node* s=ExtendStart();
      if(s) {
	Node* g=goal.FindClosest(*s);
	if(stateSpace->Distance(*s,*g) < connectionTolerance && ConnectTrees(s,g)) return true;
      }
    }
    else {
      Node* g=ExtendGoal();
      if(g) {
	Node* s=start.FindClosest(*g);
	if(stateSpace->Distance(*s,*g) < connectionTolerance && ConnectTrees(s,g)) return true;
      }
    }
  }
  return false;
}

Node* BidirectionalRRTKP::ExtendStart()
{
  EZCallTrace tr("BidirectionalRRTKP::ExtendStart()");
  State xdest;
  space->GetStateSpace()->Sample(xdest);
  Node* n=start.FindClosest(xdest);
  KinodynamicMilestonePath path;
  if(!PickControl(*n,xdest,path)) return NULL;
  if(!space->GetStateSpace()->IsFeasible(path.End()))
    return NULL;
  EdgePlannerPtr e(space->TrajectoryChecker(path));
  if(e->IsVisible())
    return start.AddMilestone(n,path,e);
  return NULL;
}

Node* BidirectionalRRTKP::ExtendGoal()
{
  EZCallTrace tr("BidirectionalRRTKP::ExtendGoal()");
  State xdest;
  space->GetStateSpace()->Sample(xdest);
  Node* n=goal.FindClosest(xdest);
  KinodynamicMilestonePath path;
  PickReverseControl(*n,xdest,path);
  if(!space->GetStateSpace()->IsFeasible(path.End()))
    return NULL;
  EdgePlannerPtr e(space->TrajectoryChecker(path));
  if(e->IsVisible())
    return goal.AddMilestone(n,path,e);
  return NULL;
}

bool BidirectionalRRTKP::ConnectTrees(Node* a,Node* b)
{
  EZCallTrace tr("BidirectionalRRTKP::ConnectTrees()");
  std::shared_ptr<SteeringFunction> sf = space->controlSpace->GetSteeringFunction();
  if(sf && sf->Connect(*a,*b,bridge.path)) {
    Real d=space->GetStateSpace()->Distance(bridge.path.End(),*b);
    if(d >= 1e-3) {
LOG4CXX_ERROR(KrisLibrary::logger(),"BidirectionRRTKP: error detected in CSpace's ConnectionControl() method, distance "<<d);
return false;
    }
    Assert(d < 1e-3);
    bridge.checker = space->TrajectoryChecker(bridge.path);
    if(bridge.checker->IsVisible()) {
      bridge.nStart=a;
      bridge.nGoal=b;
      return true;
    }
    else return false;
  }
  else return false;
}

bool BidirectionalRRTKP::PickControl(const State& x0, const State& xDest, KinodynamicMilestonePath& path) {
  std::shared_ptr<SteeringFunction> sf = space->controlSpace->GetSteeringFunction();
  if(!sf)  
    sf = make_shared<RandomBiasSteeringFunction>(space,10);
  if(sf->Connect(x0,xDest,path)) {
    //steering function is assumed to create valid controls
    for(size_t i=0;i<path.controls.size();i++) 
      assert(space->IsValidControl(path.milestones[i],path.controls[i]));
    path.MakeEdges(space);
    return true;
  }
  return false;
}

bool BidirectionalRRTKP::PickReverseControl(const State& x1, const State& xStart, KinodynamicMilestonePath& path)
{
  ReversibleControlSpace* rspace = dynamic_cast<ReversibleControlSpace*>(&*space->controlSpace);
  std::shared_ptr<SteeringFunction> sf;
  if(rspace)
    sf = rspace->reverseControlSpace->GetSteeringFunction();
  if(!sf)  
    sf = make_shared<RandomBiasReverseSteeringFunction>(space,10);
  if(sf->Connect(x1,xStart,path)) {
    //steering function is assumed to create valid controls
    for(size_t i=0;i<path.controls.size();i++) 
      assert(space->IsValidControl(path.milestones[i],path.controls[i]));
    path.MakeEdges(space);
    return true;
  }
  return false;
}


bool BidirectionalRRTKP::GetPath(KinodynamicMilestonePath& path)
{
  if(!Done()) return false;
  Assert(bridge.nStart != NULL && bridge.nGoal != NULL);  
  KinodynamicMilestonePath pStart,pGoal;
  start.GetPath(start.root,bridge.nStart,pStart);
  goal.GetPath(goal.root,bridge.nGoal,pGoal);


  path = pStart;
  path.Concat(bridge.path);
  FatalError("TODO: reverse goal path");
  path.Concat(pGoal);
  return true;
}
