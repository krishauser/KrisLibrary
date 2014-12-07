#include "MotionPlanner.h"
#include "PointLocation.h"
#include <graph/Path.h>
#include <graph/ShortestPaths.h>
#include <math/random.h>
#include <errors.h>

typedef TreeRoadmapPlanner::Node Node;
using namespace std;

//Graph search callbacks
class EdgeDistance
{
 public:
  Real operator () (const SmartPointer<EdgePlanner>& e,int s,int t)
  {
    if(!e) return 1.0;
    assert(e->Space() != NULL);
    Real res = e->Space()->Distance(e->Start(),e->Goal());
    if(res <= 0) {
      printf("RoadmapPlanner: Warning, edge has nonpositive length %g\n",res);
      return Epsilon;
    }
    return res;
  }
};


// SetComponentCallback: sets all components to c
struct SetComponentCallback : public Node::Callback
{
  SetComponentCallback(int c) { component = c; }
  virtual void Visit(Node* n) { n->connectedComponent = component; }
  int component;
};

// ClosestMilestoneCallback: finds the closest milestone to x 
struct ClosestMilestoneCallback : public Node::Callback
{
  ClosestMilestoneCallback(CSpace* s,const Config& _x)
    :space(s),closestDistance(Inf),x(_x),closestMilestone(NULL)
  {}
  virtual void Visit(Node* n) {
    Real d = space->Distance(n->x,x);
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



RoadmapPlanner::RoadmapPlanner(CSpace* s)
  :space(s)
{
  pointLocator = new NaivePointLocation(roadmap.nodes,s);
}

RoadmapPlanner::~RoadmapPlanner()
{
	Cleanup();
}


void RoadmapPlanner::Cleanup()
{
  roadmap.Cleanup();
  ccs.Clear();
  pointLocator->OnClear();
}

void RoadmapPlanner::GenerateConfig(Config& x)
{
  space->Sample(x);
}

int RoadmapPlanner::AddMilestone(const Config& x)
{
  ccs.AddNode();
  int res=roadmap.AddNode(x);
  pointLocator->OnAppend();
  return res;
}

int RoadmapPlanner::TestAndAddMilestone(const Config& x)
{
  if(!space->IsFeasible(x)) return -1;
  return AddMilestone(x);
}

void RoadmapPlanner::ConnectEdge(int i,int j,const SmartPointer<EdgePlanner>& e)
{
  ccs.AddEdge(i,j);
  roadmap.AddEdge(i,j,e);
}

SmartPointer<EdgePlanner> RoadmapPlanner::TestAndConnectEdge(int i,int j)
{
  SmartPointer<EdgePlanner> e=space->LocalPlanner(roadmap.nodes[i],roadmap.nodes[j]);
  if(e->IsVisible()) {
    ConnectEdge(i,j,e);
    return e;
  }
  else {
    e=NULL;
    return NULL;
  }
}

void RoadmapPlanner::ConnectToNeighbors(int i,Real connectionThreshold,bool ccReject)
{
  vector<int> nn;
  vector<Real> distances;
  if(pointLocator->Close(roadmap.nodes[i],connectionThreshold,nn,distances)) {
    for(size_t k=0;k<nn.size();k++) {
      int j=nn[k];
      if(ccReject) { if(ccs.SameComponent(i,j)) continue; }
      else if(i==(int)j || roadmap.HasEdge(i,j)) continue;
      TestAndConnectEdge(i,j);
    }
  }
  else {
    //fall back on naive point location
    for(size_t j=0;j<roadmap.nodes.size();j++) {
      if(ccReject) { if(ccs.SameComponent(i,j)) continue; }
      else if(i==(int)j || roadmap.HasEdge(i,j)) continue;
      if(space->Distance(roadmap.nodes[i],roadmap.nodes[j]) < connectionThreshold) {
	TestAndConnectEdge(i,j);
      }
    }
  }
}

void RoadmapPlanner::ConnectToNearestNeighbors(int i,int k,bool ccReject)
{
  if(k <= 0) return;
  vector<int> nn;
  vector<Real> distances;
  if(pointLocator->KNN(roadmap.nodes[i],(ccReject?k*4:k),nn,distances)) {
    //assume the k nearest neighbors are sorted by distance
    int numTests=0;
    for(size_t m=0;m<nn.size();m++) {
      int j=nn[m];
      if(ccReject) { if(ccs.SameComponent(i,j)) continue; }
      else if(i==(int)j) continue;
      TestAndConnectEdge(i,j);
      numTests++;
      if(numTests == k) break;
    }
  }
  else {
    //fall back on naive
    set<pair<Real,int> > knn;
    pair<Real,int> node;
    Real worst=Inf;
    for(size_t j=0;j<roadmap.nodes.size();j++) {
      if(ccReject) { if(ccs.SameComponent(i,j)) continue; }
      else if(i==(int)j) continue;
      node.first = space->Distance(roadmap.nodes[i],roadmap.nodes[j]);
      node.second = j;
      if(node.first < worst) {
	knn.insert(node);
	
	if(ccReject) {  //oversample candidate nearest neighbors
	  if((int)knn.size() > k*4)
	    knn.erase(--knn.end());
	}
	else {  //only keep k nearest neighbors
	  if((int)knn.size() > k)
	    knn.erase(--knn.end());
	}
	worst = (--knn.end())->first;
      }
    }
    int numTests=0;
    for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
      if(ccReject && ccs.SameComponent(i,j->second)) continue;
      TestAndConnectEdge(i,j->second);
      numTests++;
      if(numTests == k) break;
    }
  }
}

void RoadmapPlanner::Generate(int numSamples,Real connectionThreshold)
{
  Config x;
  for(int i=0;i<numSamples;i++) {
    GenerateConfig(x);
    int node=TestAndAddMilestone(x);
    if(node >= 0) ConnectToNeighbors(node,connectionThreshold);
  }
}

void RoadmapPlanner::CreatePath(int i,int j,MilestonePath& path)
{
  Assert(ccs.SameComponent(i,j));
  //Graph::PathIntCallback callback(roadmap.nodes.size(),j);
  //roadmap._BFS(i,callback);
  EdgeDistance distanceWeightFunc;
  Graph::ShortestPathProblem<Config,SmartPointer<EdgePlanner> > spp(roadmap);
  spp.InitializeSource(i);
  spp.FindPath_Undirected(j,distanceWeightFunc);
  if(IsInf(spp.d[j])) {
    FatalError("RoadmapPlanner::CreatePath: SameComponent is true, but no shortest path?");
    return;
  }

  list<int> nodes;
  //Graph::GetAncestorPath(callback.parents,j,i,nodes);
  bool res=Graph::GetAncestorPath(spp.p,j,i,nodes);
  if(!res) {
    FatalError("RoadmapPlanner::CreatePath: GetAncestorPath returned false");
    return;
  }
  if(nodes.front() != i || nodes.back() != j) {
    FatalError("RoadmapPlanner::CreatePath: GetAncestorPath didn't return correct path? %d to %d vs %d to %d",nodes.front(),nodes.back(),i,j);
  }
  Assert(nodes.front()==i);
  Assert(nodes.back()==j);
  path.edges.clear();
  path.edges.reserve(nodes.size());
  for(list<int>::const_iterator p=nodes.begin();p!=--nodes.end();++p) {
    list<int>::const_iterator n=p; ++n;
    SmartPointer<EdgePlanner>* e=roadmap.FindEdge(*p,*n);
    Assert(e);
    if(*e == NULL) {
      //edge data not stored
      path.edges.push_back(space->LocalPlanner(roadmap.nodes[*p],roadmap.nodes[*n]));
    }
    else {
      //edge data stored
      if((*e)->Start() == roadmap.nodes[*p]) {
	//path.edges.push_back((*e)->Copy());
	path.edges.push_back(*e);
      }
      else {
	Assert((*e)->Goal() == roadmap.nodes[*p]);
	path.edges.push_back((*e)->ReverseCopy());
      }
    }
  }
  Assert(path.IsValid());
}



TreeRoadmapPlanner::TreeRoadmapPlanner(CSpace* s)
  :space(s),connectionThreshold(Inf)
{
}

TreeRoadmapPlanner::~TreeRoadmapPlanner()
{
  Cleanup();
}


void TreeRoadmapPlanner::Cleanup()
{
  for(size_t i=0;i<connectedComponents.size();i++)
    SafeDelete(connectedComponents[i]);
  connectedComponents.clear();
  milestones.clear();
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::TestAndAddMilestone(const Config& x)
{
  if(space->IsFeasible(x))
    return AddMilestone(x);
  else 
    return AddInfeasibleMilestone(x);
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::AddMilestone(const Config& x)
{
  Milestone m;
  m.x=x;
  int n=(int)connectedComponents.size();
  m.connectedComponent=n;
  connectedComponents.push_back(new Node(m));
  milestones.push_back(connectedComponents[n]);
  return connectedComponents[n];
}

void TreeRoadmapPlanner::GenerateConfig(Config& x)
{
  space->Sample(x);
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::Extend()
{
  GenerateConfig(x);
  Node* n=AddMilestone(x);
  if(n) ConnectToNeighbors(n);
  return n;
}


void TreeRoadmapPlanner::ConnectToNeighbors(Node* n)
{
  if(n->connectedComponent == -1) return;
  if(IsInf(connectionThreshold)==1) {
    //for each other component, attempt a connection to the closest node
    for(size_t i=0;i<connectedComponents.size();i++) {
      if((int)i == n->connectedComponent) continue;
      
      ClosestMilestoneCallback callback(space,n->x);
      connectedComponents[i]->DFS(callback);
      TryConnect(n,callback.closestMilestone);
    }
  }
  else {
    //attempt a connection between this node and all others within the 
    //connection threshold
    for(size_t i=0;i<milestones.size();i++) {
      if(n->connectedComponent != milestones[i]->connectedComponent) {
	if(space->Distance(n->x,milestones[i]->x) < connectionThreshold) {
	  TryConnect(n,milestones[i]);
	}
      }
    }
  }
}

EdgePlanner* TreeRoadmapPlanner::TryConnect(Node* a,Node* b)
{
  Assert(a->connectedComponent != b->connectedComponent);
  EdgePlanner* e=space->LocalPlanner(a->x,b->x);
  if(e->IsVisible()) {
    if(a->connectedComponent < b->connectedComponent) AttachChild(a,b,e);
    else AttachChild(b,a,e);
    return e;
  }
  delete e;
  return NULL;
}

void TreeRoadmapPlanner::AttachChild(Node* p, Node* c, EdgePlanner* e)
{
  Assert(p->connectedComponent != c->connectedComponent);
  if(e) Assert(e->Start() == p->x && e->Goal() == c->x);
  connectedComponents[c->connectedComponent] = NULL;
  c->reRoot();
  SetComponentCallback callback(p->connectedComponent);
  c->DFS(callback);
  p->addChild(c);
  c->edgeFromParent() = e;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::SplitEdge(Node* p,Node* n,Real u)
{
  Assert(p==n->getParent());
  Vector x;
  n->edgeFromParent()->Eval(u,x);
  p->detachChild(n);
  Node* s=Extend(p,x);
  s->addChild(n);
  n->edgeFromParent() = space->LocalPlanner(x,n->x);
  return s;
}

void TreeRoadmapPlanner::DeleteSubtree(Node* n)
{
  if(connectedComponents[n->connectedComponent] == n) {
    connectedComponents[n->connectedComponent] = n->getParent();
  }
  Graph::TopologicalSortCallback<Node*> callback;
  n->DFS(callback);
  for(list<Node*>::iterator i=callback.list.begin();i!=callback.list.end();i++) {
    for(size_t j=0;j<milestones.size();j++) {
      if(milestones[j]==*i) {
	milestones[j]=milestones.back();
	milestones.resize(milestones.size()-1);
	break;
      }
    }
  }
  n->getParent()->eraseChild(n);
}

void TreeRoadmapPlanner::CreatePath(Node* a, Node* b, MilestonePath& path)
{
  Assert(a->connectedComponent == b->connectedComponent);
  Assert(a->LCA(b) != NULL);  //make sure they're on same tree?
  a->reRoot();
  connectedComponents[a->connectedComponent] = a;
  Assert(b->hasAncestor(a) || b==a);
  Assert(a->getParent()==NULL);

  //get path from a to b
  list<Node*> atob;
  while(b != NULL) {
    atob.push_front(b);
    Assert(b->connectedComponent == a->connectedComponent);
    if(b->getParent() != NULL) {
      Assert(b->edgeFromParent()->Goal() == b->x);
      Assert(b->edgeFromParent()->Start() == b->getParent()->x);
    }
    b = b->getParent();
  }
  assert(atob.front() == a);

  path.edges.resize(atob.size()-1);
  int index=0;
  for(list<Node*>::iterator i=++atob.begin();i!=atob.end();i++) {
    b=*i;
    if(b->edgeFromParent()==NULL) {
      //printf("Hmm... constructing new edge?\n");
      //edge data not stored
      list<Node*>::iterator p=i; --p;
      Node* bp = *p;
      path.edges[index]=space->LocalPlanner(bp->x,b->x);
    }
    else {
      //contains edge data
      if(b->x == b->edgeFromParent()->Start())
	path.edges[index]=b->edgeFromParent()->ReverseCopy();
      else {
	Assert(b->x == b->edgeFromParent()->Goal());
	//do we need a copy here?
	//path.edges[index]=b->edgeFromParent()->Copy();
	path.edges[index]=b->edgeFromParent();
      }
    }
    index++;
  }
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::ClosestMilestone(const Config& x)
{
  if(milestones.empty()) return NULL;
  Real dmin=space->Distance(milestones[0]->x,x);
  Node* n=milestones[0];
  for(size_t i=1;i<milestones.size();i++) {
    Real d=space->Distance(milestones[i]->x,x);
    if(d<dmin) {
      dmin=d;
      n=milestones[i];
    }
  }
  return n;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::ClosestMilestoneInComponent(int component,const Config& x)
{
  ClosestMilestoneCallback callback(space,x);
  connectedComponents[component]->DFS(callback);
  return callback.closestMilestone;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::ClosestMilestoneInSubtree(Node* node,const Config& x)
{
  ClosestMilestoneCallback callback(space,x);
  node->DFS(callback);
  return callback.closestMilestone;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::Extend(Node* n,const Config& x)
{    //connect closest to n
  EdgePlanner* e=space->LocalPlanner(n->x,x);
  Assert(e->Start() == n->x);
  Assert(e->Goal() == x);
  Node* c=AddMilestone(x);
  n->addChild(c);
  c->edgeFromParent() = e;
  c->connectedComponent = n->connectedComponent;
  //AddMilestone adds a connected component
  connectedComponents.resize(connectedComponents.size()-1);
  return c;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::TryExtend(Node* n,const Config& x)
{
  if(space->IsFeasible(x)) {
    //connect closest to n
    EdgePlanner* e=space->LocalPlanner(n->x,x);
    if(e->IsVisible()) {
      Node* c=AddMilestone(x);
      n->addChild(c);
      c->edgeFromParent() = e;
      c->connectedComponent = n->connectedComponent;
      //AddMilestone adds a connected component
      connectedComponents.resize(connectedComponents.size()-1);
      return c;
    }
    else {
      delete e;
    }
  }
  return NULL;
}






/*
bool LazyCollisionRP::CheckPath(Node* a, Node* b)
{
  Assert(a->connectedComponent == b->connectedComponent);
  a->reRoot();
  connectedComponents[a->connectedComponent] = a;
  Node* n=b;
  while(n != a) {
    Node* p = n->getParent();
    Assert(p != NULL);
    if(!space->IsVisible(n->x,p->x)) {
      //split at n
      p->detachChild(n);
      SetComponentCallback callback(connectedComponents.size());
      connectedComponents.push_back(n);
      n->DFS(callback);
      return false;
    }
    n = p;
  }
  return true;
}

RandomizedPlanner::Node* LazyCollisionRP::CanConnectComponent(int i,const Config& x)
{
	ClosestMilestoneCallback callback(space,x);
	connectedComponents[i]->DFS(callback);
	if(callback.closestMilestone) {
		if(callback.closestDistance < connectionThreshold) {
			return callback.closestMilestone;
		}
	}
	return NULL;
}
*/


PerturbationTreePlanner::PerturbationTreePlanner(CSpace*s)
  :TreeRoadmapPlanner(s),delta(1)
{}

void PerturbationTreePlanner::Cleanup()
{
  TreeRoadmapPlanner::Cleanup();
  weights.clear();
}

TreeRoadmapPlanner::Node* PerturbationTreePlanner::AddMilestone(const Config& x)
{
  Assert(milestones.size() == weights.size());
  Node* n=TreeRoadmapPlanner::AddMilestone(x);
  Assert(n == milestones.back());
  weights.push_back(1);
  Assert(milestones.size() == weights.size());
  return n;
}

void PerturbationTreePlanner::GenerateConfig(Config& x)
{
  if(milestones.empty()) {
    cerr<<"PerturbationTreePlanner::GenerateConfig(): No nodes to choose from!"<<endl;
    space->Sample(x);
  }
  else {
    Node* n = SelectMilestone(milestones);
    space->SampleNeighborhood(n->x,delta,x);
  }
}

TreeRoadmapPlanner::Node* PerturbationTreePlanner::SelectMilestone(const vector<Node*>& milestones)
{
  Assert(milestones.size()==weights.size());
  Real total=Zero;
  for(unsigned int i=0;i<milestones.size();i++) {
    total += weights[i];
  }
  //pick randomly from the total
  Real val=Rand(Zero,total);
  for(unsigned int i=0;i<milestones.size();i++) {
    val -= weights[i];
    if(val<=Zero) return milestones[i];
  }
  //shouldn't get here
  Assert(false);
  return NULL;
}




RRTPlanner::RRTPlanner(CSpace*s)
  :TreeRoadmapPlanner(s),delta(1)
{}

TreeRoadmapPlanner::Node* RRTPlanner::Extend()
{
  Config dest,x;
  space->Sample(dest);

  //pick closest milestone, step in that direction
  Node* closest=ClosestMilestone(dest);
  Real dist=space->Distance(closest->x,dest);
  if(dist > delta)
    space->Interpolate(closest->x,dest,delta/dist,x);
  else
    x=dest;

  return TryExtend(closest,x);
}


BidirectionalRRTPlanner::BidirectionalRRTPlanner(CSpace*s)
  :RRTPlanner(s)
{}

void BidirectionalRRTPlanner::Init(const Config& start, const Config& goal)
{
  Cleanup();
  Assert(space->IsFeasible(start));
  Assert(space->IsFeasible(goal));
  AddMilestone(start);
  AddMilestone(goal);
  Assert(milestones.size()==2);
  Assert(milestones[0]->x == start);
  Assert(milestones[1]->x == goal);
  Assert(connectedComponents.size()==2);
  Assert(connectedComponents[0] == milestones[0]);
  Assert(connectedComponents[1] == milestones[1]);
}

bool BidirectionalRRTPlanner::Plan()
{
  //If we've already found a path, return true
  if(milestones[0]->connectedComponent == milestones[1]->connectedComponent)
    return true;

  Node* n=Extend();
  if(!n) return false;

  if(n->connectedComponent == milestones[0]->connectedComponent) {
    //attempt to connect to goal, if the distance is < connectionThreshold
    ClosestMilestoneCallback callback(space,n->x);
    milestones[1]->DFS(callback);
    if(callback.closestDistance < connectionThreshold) {
      if(TryConnect(n,callback.closestMilestone)) //connection successful!
	return true;
    }
  }
  else {
    Assert(n->connectedComponent == milestones[1]->connectedComponent);
    //attempt to connect to start, if the distance is < connectionThreshold
    ClosestMilestoneCallback callback(space,n->x);
    milestones[0]->DFS(callback);
    if(callback.closestDistance < connectionThreshold) {
      if(TryConnect(callback.closestMilestone,n)) //connection successful!
	return true;
    }
  }
  return false;
}

void BidirectionalRRTPlanner::CreatePath(MilestonePath& p) const
{
  Assert(milestones[0]->connectedComponent == milestones[1]->connectedComponent);
  Assert(connectedComponents[0] == milestones[0]);
  list<Node*> path;
  Node* n = milestones[1];
  while(n != milestones[0]) {
    path.push_front(n);
    n = n->getParent();
    Assert(n != NULL);
  }
  p.edges.resize(0);
  p.edges.reserve(path.size());
  for(list<Node*>::const_iterator i=path.begin();i!=path.end();i++) {
    Node* n = *i;
    SmartPointer<EdgePlanner> e=n->edgeFromParent();
    if(e->Start() == n->x) {
      p.edges.push_back(e->ReverseCopy());
    }
    else if(e->Goal() == n->x) {
      p.edges.push_back(e);
    }
    else {
      cerr<<"Hmm... edge doesn't have node as its start or its goal!"<<endl;
      Abort();
    }
  }
}

/*
VisibilityPRM::VisibilityPRM(CSpace*s)
  :RandomizedPlanner(s)
{}

VisibilityPRM::Node* VisibilityPRM::AddMilestone(const Config& x)
{
  if(space->IsFeasible(x)) {
    vector<Node*> visibleNodes;

    //add it only if it can connect to 0, or >= 2 nodes
    //pick two closest nodes of entire graph
    vector<Node*> closestNodes;
    vector<Real> closestDistances;
    closestNodes.reserve(connectedComponents.size()/2);
    closestDistances.reserve(connectedComponents.size()/2);

    Real minDist=Inf;
    for(size_t i=0;i<connectedComponents.size();i++) {
      if(connectedComponents[i] == NULL) continue;
      ClosestMilestoneCallback callback(space,x);
      connectedComponents[i]->DFS(callback);
      if(callback.closestMilestone) {
	closestNodes.push_back(callback.closestMilestone);
	closestDistances.push_back(callback.closestDistance);
	minDist = Min(minDist,callback.closestDistance);
      }
    }
    //check visibility of any node that's closer than 2x min dist
    for(size_t i=0;i<closestDistances.size();i++) {
      if(closestDistances[i] < minDist*Two) {
	if(space->IsVisible(x,closestNodes[i]->x)) 
	  visibleNodes.push_back(closestNodes[i]);
      }
    }

    //connect if 0 or >= 2 nodes can be seen
    if(visibleNodes.size() != 1) {
      Node* n = AddMilestone(x);
      for(size_t i=0;i<visibleNodes.size();i++) {
	ConnectComponents(visibleNodes[i],n);
      }
      return n;
    }
  }
  return NULL;
}

RandomizedPlanner::Node* VisibilityPRM::Extend()
{
	GenerateConfig(x);
	Node* n=TestAndAddMilestone(x);
	//if(n) ConnectToNeighbors(n);  (don't connect to neighbors, AddMilestone does that for you)
	return n;
}


RandomizedPlanner::Node* VisibilityPRM::CanConnectComponent(int i,const Config& x)
{
  //return closest visible node
  ClosestMilestoneCallback callback(space,x);
  connectedComponents[i]->DFS(callback);
  if(space->IsVisible(callback.closestMilestone->x,x))
    return callback.closestMilestone;
  return NULL;
}

*/
