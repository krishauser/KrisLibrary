#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "KinodynamicMotionPlanner.h"
#include <graph/Callback.h>
#include <algorithm>
#include <errors.h>
#include <utils/EZTrace.h>
#include <math/random.h>
using namespace std;

typedef KinodynamicTree::Node Node;

// ClosestMilestoneCallback: finds the closest milestone to x 
struct ClosestMilestoneCallback : public Node::Callback
{
  ClosestMilestoneCallback(CSpace* s,const Config& _x)
    :space(s),closestDistance(Inf),x(_x),closestMilestone(NULL)
  {}
  virtual void Visit(Node* n) {
    Real d = space->Distance(x,*n);
    if(d < closestDistance) {
      closestDistance = d;
      closestMilestone = n;
    }
  }
  CSpace* space;
  Real closestDistance;
  const Config& x;
  Node* closestMilestone;
};

struct PickCallback : public Node::Callback
{
  PickCallback(int _k) :i(0),k(_k),res(NULL) {}
  virtual void Visit(Node* n) { 
    if(i==k) res=n;
    i++;
  }
  virtual bool Stop() { return (res!=NULL); }

  int i;
  int k;
  Node* res;
};

struct VectorizeCallback : public Node::Callback
{
  virtual void Visit(Node* n) { 
    nodes.push_back(n);
  }

  vector<Node*> nodes;
};










KinodynamicTree::KinodynamicTree(KinodynamicCSpace* s)
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
  index.push_back(root);
}

void KinodynamicTree::Clear()
{
  index.clear();
  SafeDelete(root);
}


Node* KinodynamicTree::AddMilestone(Node* parent, const ControlInput& u,const State& x)
{
  FatalError("AddMilestone(parent,u,x) deprecated");

  Node* c=parent->addChild(x);
  c->edgeFromParent().u = u;
  c->edgeFromParent().e = NULL;
  index.push_back(c);
  return c;
}

Node* KinodynamicTree::AddMilestone(Node* parent,const ControlInput& u,const vector<State>& path,const SmartPointer<EdgePlanner>& e)
{
  Node* c;
  if(e->Start() == *parent) {
    c=parent->addChild(e->Goal());
  }
  else {
    Assert(e->Goal() == *parent);
    c=parent->addChild(e->Start());
  }
  c->edgeFromParent().u = u;
  c->edgeFromParent().path = path;
  c->edgeFromParent().e = e;
  index.push_back(c);
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

Node* KinodynamicTree::FindClosest(const State& x) const
{
  EZCallTrace tr("KinodynamicTree::FindClosest()");
  
  if(!root) return NULL;

  //iterating is faster
  Node* closest=NULL;
  double dclosest=Inf;
  for(size_t i=0;i<index.size();i++) {
    double d=space->Distance(x,*index[i]);
    if(d < dclosest) {
      dclosest=d;
      closest=index[i];
    }
  }
  return closest;
    /*
  ClosestMilestoneCallback callback(space,x);
  root->DFS(callback);
  return callback.closestMilestone;
    */
}

Node* KinodynamicTree::PickRandom() const
{
  EZCallTrace tr("KinodynamicTree::PickRandom()");

  if(!root) return NULL;

  return index[RandInt(index.size())];

  /*
  CountCallback<Node*> count;
  root->DFS(count);
  PickCallback pick(RandInt(count.count));
  root->DFS(pick);
  Assert(pick.res != NULL);
  return pick.res;
  */
}

Node* KinodynamicTree::ApproximateRandomClosest(const State& x,int numIters) const
{
  EZCallTrace tr("KinodynamicTree::ApproximateRandomClosest()");

  if(!root) return NULL;

  Assert(numIters > 0);
  Real dclosest=Inf;
  Node* closest=NULL;
  for(int i=0;i<numIters;i++) {
    Node* n=index[RandInt(index.size())];
    Real d=space->Distance(x,*n);
    if(d < dclosest) {
      dclosest=d;
      closest=n;
    }
  }
  return closest;
  /*
  VectorizeCallback callback;
  root->DFS(callback);
  Assert(numIters > 0);
  Real dclosest=Inf;
  Node* closest=NULL;
  for(int i=0;i<numIters;i++) {
    Node* n=callback.nodes[RandInt(callback.nodes.size())];
    Real d=space->Distance(x,*n);
    if(d < dclosest) {
      dclosest=d;
      closest=n;
    }
  }
  return closest;
  */
}

void KinodynamicTree::GetPath(Node* start,Node* goal,KinodynamicMilestonePath& path)
{
  path.Clear();
  Node* n=goal;
  while(n != start) {
    if(n == NULL) FatalError("KinodynamicTree::GetPath: nodes specified on tree are not valid!");
    path.milestones.push_back(*n);
    path.controls.push_back(n->edgeFromParent().u);
    path.paths.push_back(n->edgeFromParent().path);
    path.edges.push_back(n->edgeFromParent().e);

    n = n->getParent();
  }
  path.milestones.push_back(*n);

  //flip the vectors
  reverse(path.milestones.begin(),path.milestones.end());
  reverse(path.controls.begin(),path.controls.end());
  reverse(path.paths.begin(),path.paths.end());
  reverse(path.edges.begin(),path.edges.end());
}

void KinodynamicTree::DeleteSubTree(Node* n)
{
  EZCallTrace tr("KinodynamicTree::DeleteSubTree()");
  if(n == root) root = NULL;
  Node* p=n->getParent();
  if(p) p->detachChild(n);
  delete n;  //this automatically deletes n and all children

  VectorizeCallback callback;
  if(root) root->DFS(callback);
  index = callback.nodes;
}





RRTKinodynamicPlanner::RRTKinodynamicPlanner(KinodynamicCSpace* s)
  :space(s),goalSeekProbability(0.1),goalSet(NULL),tree(s),goalNode(NULL)
{}

Node* RRTKinodynamicPlanner::Plan(int maxIters)
{
  if(!goalSet) {
        LOG4CXX_ERROR(logger,"RRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!\n");
        LOG4CXX_ERROR(logger,"   Press enter to continue\n");
    if(logger->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
  }
  if(goalSet && goalSet->IsFeasible(*tree.root)) {
    goalNode = tree.root;
    return tree.root;
  }
  for(int i=0;i<maxIters;i++) {
    Node* n=Extend();
    if(n && goalSet && goalSet->IsFeasible(*n)) {
      goalNode = n;
      return n;
    }
  }
  return NULL;
}

void RRTKinodynamicPlanner::Init(const State& xinit)
{
  goalNode = NULL;
  tree.Init(xinit);
}

void RRTKinodynamicPlanner::PickDestination(State& xdest)
{
  if(goalSet && RandBool(goalSeekProbability)) {
    goalSet->Sample(xdest);
  }
  else {
    space->Sample(xdest);
  }
}

Node* RRTKinodynamicPlanner::Extend()
{
  State xdest;
  PickDestination(xdest);
  return ExtendToward(xdest);
}


Node* RRTKinodynamicPlanner::ExtendToward(const State& xdest)
{
  //EZCallTrace tr("RRTKinodynamicPlanner::Extend()");
  Node* n=tree.FindClosest(xdest);
  ControlInput u;
  PickControl(*n,xdest,u);
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);
  if(!space->IsFeasible(path.back())) {
    //LOG4CXX_INFO(logger,"Edge endpoint is not feasible\n");
    return NULL;
  }
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible()) {
    return tree.AddMilestone(n,u,path,e);
  }
  else {
    //LOG4CXX_INFO(logger,"Edge is not visible\n");
    delete e;
    return NULL;
  }
}

bool RRTKinodynamicPlanner::IsDone() const
{
  return goalNode != NULL;
}

void RRTKinodynamicPlanner::CreatePath(KinodynamicMilestonePath& path) const
{
  Assert(goalNode != NULL);
  tree.GetPath(tree.root,goalNode,path);
}

void RRTKinodynamicPlanner::PickControl(const State& x0, const State& xDest, ControlInput& u) {
  space->BiasedSampleControl(x0,xDest,u);
}




LazyRRTKinodynamicPlanner::LazyRRTKinodynamicPlanner(KinodynamicCSpace* s)
  :RRTKinodynamicPlanner(s)
{}

Node* LazyRRTKinodynamicPlanner::Plan(int maxIters)
{
  if(!goalSet) {
        LOG4CXX_ERROR(logger,"LazyRRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!\n");
        LOG4CXX_ERROR(logger,"   Press enter to continue\n");
    if(logger->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
  }
  if(goalSet && goalSet->IsFeasible(*tree.root)) {
    goalNode = tree.root;
    return tree.root;
  }
  for(int i=0;i<maxIters;i++) {
    Node* n=Extend();
    if(n && goalSet && goalSet->IsFeasible(*n)) {
      if(CheckPath(n)) {
	goalNode = n;
	return n;
      }
    }
  }
  return NULL;
}

Node* LazyRRTKinodynamicPlanner::ExtendToward(const State& xdest)
{
  EZCallTrace tr("LazyRRTKinodynamicPlanner::Extend()");
  //Node* n=tree.FindClosest(xdest);
  Node* n=tree.ApproximateRandomClosest(xdest,10);
  ControlInput u;
  PickControl(*n,xdest,u);
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);
  if(space->IsFeasible(path.back())) {
    EdgePlanner* e=space->TrajectoryChecker(path);
    return tree.AddMilestone(n,u,path,e);
  }
  return NULL;
}

struct NodeWithPriority
{
  Node* n;

  NodeWithPriority(Node* _n) :n(_n) {}
  operator Node* () const { return n; }
  Node* operator -> () const { return n; }
  inline bool operator < (const NodeWithPriority& b) const {
    return n->edgeFromParent().e->Priority() < b->edgeFromParent().e->Priority();
  }
};

bool LazyRRTKinodynamicPlanner::CheckPath(Node* n)
{
  EZCallTrace tr("LazyRRTKinodynamicPlanner::CheckPath()");
  //add all nodes up to n, except root
  priority_queue<NodeWithPriority,vector<NodeWithPriority> > q;
  while(n != tree.root) {
    q.push(n);
    n = n->getParent();
    Assert(n != NULL);
  }
  while(!q.empty()) {
    n=q.top(); q.pop();
    EdgePlanner* e=n->edgeFromParent().e;
    if(!e->Done()) {
      if(!e->Plan()) {
	//disconnect n from the rest of the tree
	tree.DeleteSubTree(n);
	return false;
      }
      q.push(n);
    }
  }
  return true;
}






BidirectionalRRTKP::BidirectionalRRTKP(KinodynamicCSpace* s)
  :space(s),start(s),goal(s),connectionTolerance(1.0)
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
}

bool BidirectionalRRTKP::IsDone() const
{
  return bridge.nStart != NULL;
}

bool BidirectionalRRTKP::Plan(int maxIters)
{
  for(int iters=0;iters<maxIters;iters++) {
    if(RandBool()) {
      Node* s=ExtendStart();
      if(s) {
	Node* g=goal.FindClosest(*s);
	if(space->Distance(*s,*g) < connectionTolerance && ConnectTrees(s,g)) return true;
      }
    }
    else {
      Node* g=ExtendGoal();
      if(g) {
	Node* s=start.FindClosest(*g);
	if(space->Distance(*s,*g) < connectionTolerance && ConnectTrees(s,g)) return true;
      }
    }
  }
  return false;
}

Node* BidirectionalRRTKP::ExtendStart()
{
  EZCallTrace tr("BidirectionalRRTKP::ExtendStart()");
  State xdest;
  space->Sample(xdest);
  Node* n=start.FindClosest(xdest);
  ControlInput u;
  PickControl(*n,xdest,u);
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);
  if(!space->IsFeasible(path.back()))
    return NULL;
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible())
    return start.AddMilestone(n,u,path,e);
  else {
    delete e;
    return NULL;
  }
}

Node* BidirectionalRRTKP::ExtendGoal()
{
  EZCallTrace tr("BidirectionalRRTKP::ExtendGoal()");
  State xdest;
  space->Sample(xdest);
  Node* n=goal.FindClosest(xdest);
  ControlInput u;
  PickReverseControl(*n,xdest,u);
  Assert(space->IsValidReverseControl(*n,u));
  vector<State> path;
  space->ReverseSimulate(*n,u,path);
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible())
    return goal.AddMilestone(n,u,path,e);
  else delete e;
  return NULL;
}

bool BidirectionalRRTKP::ConnectTrees(Node* a,Node* b)
{
  EZCallTrace tr("BidirectionalRRTKP::ConnectTrees()");
  if(space->ConnectionControl(*a,*b,bridge.u)) {
    if(space->IsValidControl(*a,bridge.u)) {
      space->Simulate(*a,bridge.u,bridge.path);
      Real d=space->Distance(bridge.path.back(),*b);
      if(d >= 1e-3) {
		LOG4CXX_ERROR(logger,"BidirectionRRTKP: error detected in CSpace's ConnectionControl() method, distance "<<d);;
      }
      Assert(d < 1e-3);
      bridge.e = space->TrajectoryChecker(bridge.path);
      if(bridge.e->IsVisible()) {
	bridge.nStart=a;
	bridge.nGoal=b;
	return true;
      }
      else return false;
    }
    else return false;
  }
  else return false;
}

void BidirectionalRRTKP::PickControl(const State& x0, const State& xDest, ControlInput& u) {
  space->BiasedSampleControl(x0,xDest,u);
}

void BidirectionalRRTKP::PickReverseControl(const State& x1, const State& xStart, ControlInput& u) {
  space->BiasedSampleReverseControl(x1,xStart,u);
}


void BidirectionalRRTKP::CreatePath(KinodynamicMilestonePath& path)
{
  Assert(bridge.nStart != NULL && bridge.nGoal != NULL);  
  KinodynamicMilestonePath pStart,pGoal;
  start.GetPath(start.root,bridge.nStart,pStart);
  goal.GetPath(goal.root,bridge.nGoal,pGoal);

  path.milestones.resize(pStart.milestones.size()+pGoal.milestones.size());
  copy(pStart.milestones.begin(),pStart.milestones.end(),path.milestones.begin());
  reverse_copy(pGoal.milestones.begin(),pGoal.milestones.end(),path.milestones.begin()+pStart.milestones.size());

  path.controls.resize(pStart.controls.size()+pGoal.controls.size()+1);
  copy(pStart.controls.begin(),pStart.controls.end(),path.controls.begin());
  path.controls[pStart.controls.size()] = bridge.u;
  reverse_copy(pGoal.controls.begin(),pGoal.controls.end(),path.controls.begin()+pStart.controls.size()+1);

  path.edges.resize(pStart.edges.size()+pGoal.edges.size()+1);
  copy(pStart.edges.begin(),pStart.edges.end(),path.edges.begin());
  path.edges[pStart.edges.size()] = bridge.e;
  reverse_copy(pGoal.edges.begin(),pGoal.edges.end(),path.edges.begin()+pStart.edges.size()+1);

  path.paths.resize(pStart.paths.size()+pGoal.paths.size()+1);
  copy(pStart.paths.begin(),pStart.paths.end(),path.paths.begin());
  path.paths[pStart.paths.size()] = bridge.path;
  reverse_copy(pGoal.paths.begin(),pGoal.paths.end(),path.paths.begin()+pStart.paths.size()+1);
}
