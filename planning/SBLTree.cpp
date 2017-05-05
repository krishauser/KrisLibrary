#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "SBLTree.h"
#include <math/random.h>
#include <errors.h>
#include <vector>
#include <queue>
using namespace std;
using namespace Geometry;

typedef SBLTree::Node Node;
typedef EdgePlanner Edge;

//HACK if an edge plan fails, extend the tree with the max
// extensions along that plan
#define USE_PLAN_EXTENSIONS 0
const static Real kMinExtensionLength = 0.01;

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

struct AddPointCallback : public Node::Callback
{
  AddPointCallback(SBLSubdivision& _s) :s(_s) {}
  virtual void Visit(Node* n) { s.AddPoint(n); }
  SBLSubdivision& s;
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

//moves nodes from tree a to b
struct ChangeTreeCallback : public Node::Callback
{
  ChangeTreeCallback(SBLTree* _a,SBLTree* _b)
    :a(_a),b(_b)
  {}
  virtual void Visit(Node* n) { 
    a->RemoveMilestone(n);
    b->AddMilestone(n);
  }

  SBLTree *a, *b;
};

//removes nodes from tree t
struct RemoveTreeCallback : public Node::Callback
{
  RemoveTreeCallback(SBLTree* _t)
    :t(_t)
  {}
  virtual void Visit(Node* n) { 
    t->RemoveMilestone(n);
  }

  SBLTree *t;
};









SBLTree::SBLTree(CSpace* s)
  :space(s),root(NULL)
{}

SBLTree::~SBLTree()
{
  Cleanup();
}

void SBLTree::Cleanup()
{
  SafeDelete(root);
}

void SBLTree::Init(const Config& qRoot)
{
  Assert(!root);
  Assert(space->IsFeasible(qRoot));
  root = AddMilestone(qRoot);
}

Node* SBLTree::Extend(Real maxDistance,int maxIters)
{
  Node* n=PickExpand();
  Config x;
  for(int i=1;i<=maxIters;i++) {
    Real r = maxDistance/i;
    space->SampleNeighborhood(*n,r,x);
    if(space->IsFeasible(x)) {
      //add as child of n
      return AddChild(n,x);
    }
  }
  return NULL;
}

Node* SBLTree::AddChild(Node* n,const Config& x)
{
  Node* c=AddMilestone(x);
  c->edgeFromParent() = space->LocalPlanner(*n,*c);
  return n->addChild(c);
}

bool SBLTree::HasNode(Node* n) const
{
  return n==root || n->hasAncestor(root);
}

void SBLTree::AdjustMilestone(Node* n,const Config& newConfig)
{
  RemoveMilestone(n);
  n->copy(newConfig);
  AddMilestone(n);

  n=n->getFirstChild();
  //adjust all the edges from parents
  while(n != NULL) {
    n->edgeFromParent() = space->LocalPlanner(newConfig,*n);
    n = n->getNextSibling();
  }
}

void SBLTree::DeleteSubtree(Node* n)
{
  if(n->getParent() == NULL) {
    Assert(n==root);
    root = NULL;
  }
  else
    n->getParent()->detachChild(n);

  //remove any records for milestones in the subtree at n 
  RemoveTreeCallback removeCallback(this);
  n->DFS(removeCallback);  
  //delete the subtree
  delete n;
}



struct LessEdgePriority
{
  typedef SBLTree::EdgeInfo EdgeInfo;
  bool operator() (EdgeInfo& a,EdgeInfo& b) const
  {
    return a.e->Priority() < b.e->Priority();
  }
  bool operator() (EdgePlanner* a,EdgePlanner* b) const
  {
    return a->Priority() < b->Priority();
  }
};

bool SBLTree::CheckPath(SBLTree* s,Node* ns,SBLTree* g,Node* ng,std::list<EdgeInfo>& outputPath)
{
  CSpace* space=s->space;
  Assert(s->space == g->space);
  //LOG4CXX_INFO(logger,"Checking path!!!"<<"\n");
  Assert(s->HasNode(ns));
  Assert(g->HasNode(ng));
  Assert(outputPath.empty());
  //start -> ns -> ng -> goal
  SmartPointer<EdgePlanner> bridge = space->LocalPlanner(*ns,*ng);  //edge from ns to ng

  priority_queue<EdgeInfo,vector<EdgeInfo>,LessEdgePriority> q;

  //start->ns
  EdgeInfo temp;
  Node* n=ns;
  while(n->getParent() != NULL) {
    temp.s = n->getParent();
    temp.t = n;
    temp.e = n->edgeFromParent();
    temp.reversed=false;
    outputPath.push_front(temp);
    n=n->getParent();
  }

  //ns->ng
  temp.s = ns;
  temp.t = ng;
  temp.e = bridge;
  temp.reversed=false;
  outputPath.push_back(temp);

  //ng->goal
  n=ng;
  while(n->getParent() != NULL) {
    temp.s = n;
    temp.t = n->getParent();
    temp.e = n->edgeFromParent();
    temp.reversed=true;
    outputPath.push_back(temp);
    n=n->getParent();
  }
  Assert(outputPath.front().s == s->root);
  Assert(outputPath.back().t == g->root);
  for(list<EdgeInfo>::iterator i=outputPath.begin();i!=outputPath.end();i++) {
    list<EdgeInfo>::iterator n=i; n++;
    if(n != outputPath.end())
      Assert(i->t == n->s);
  }

  for(list<EdgeInfo>::iterator i=outputPath.begin();i!=outputPath.end();i++)
    q.push(*i);
  
  //adaptive division of path
  Config x;
  while(!q.empty()) {
    temp=q.top(); q.pop();
    if(temp.e->Done()) continue;
    //Real len=temp.e->Priority();
#if USE_PLAN_EXTENSIONS
    Config *a,*b;
    BisectionEpsilonEdgePlanner* bisectionEdge;
    try {
      bisectionEdge=dynamic_cast<BisectionEpsilonEdgePlanner*>((EdgePlanner*)temp.e);
    }
    catch(exception& e) {
      FatalError("SBLPlanner is unable to cast edge planner to BisectionEpsilonEdgePlanner - turn off USE_PLAN_EXTENSIONS in SBLTree.cpp");
    }
    //assert(len == temp.e->Priority());
    if(!bisectionEdge->Plan(a,b)) {
      Config p=*a;
      Config q=*b;
      assert(bisectionEdge == temp.e);

      //disconnect!
      if(temp.e == bridge) {
	//LOG4CXX_INFO(logger,"Disconnecting edge between connected nodes"<<"\n");
	bridge = NULL;
      }
      else if(s->HasNode(temp.s)) {
	//LOG4CXX_INFO(logger,"Disconnecting edge on start tree"<<"\n");
	//disconnect tree from s->t
	temp.s->detachChild(temp.t);
	Assert(temp.t == ns || temp.t->hasDescendent(ns));
	ns->reRoot();
	ns->edgeFromParent() = bridge;
	
	//move nodes in subtree from start arrays to goal arrays
	ChangeTreeCallback changeCallback(s,g);
	ns->DFS(changeCallback);
	ng->addChild(ns);
	Assert(ns->hasAncestor(g->root));
	Assert(s->root->getParent()==NULL);
      }
      else {     //on goal tree
	//LOG4CXX_INFO(logger,"Disconnecting edge on goal tree"<<"\n");
	Assert(g->HasNode(temp.t));
	//disconnect tree from s->t
	temp.t->detachChild(temp.s);
	Assert(temp.s == ng || temp.s->hasDescendent(ng));
	ng->reRoot();
	ng->edgeFromParent() = bridge;
	
	//move nodes in subtree from goal arrays to start arrays
	ChangeTreeCallback changeCallback(g,s);
	ng->DFS(changeCallback);
	ns->addChild(ng);
	Assert(ng->hasAncestor(s->root));
	Assert(g->root->getParent()==NULL);
      }
      outputPath.clear();

      //extend the segments
      if(space->Distance(p,*temp.s) > kMinExtensionLength) {
	s->AddChild(temp.s,p);
      }
      if(space->Distance(q,*temp.t) > kMinExtensionLength) {
	g->AddChild(temp.t,q);
      }
      //LOG4CXX_INFO(logger,"CheckPath: Failed on edge of length "<<len<<"\n");
      return false;
    }
#else
    if(!temp.e->Plan()) {
      //disconnect!
      if(temp.e == bridge) {
	//LOG4CXX_INFO(logger,"Disconnecting edge between connected nodes"<<"\n");
	//no change in graph
	bridge = NULL;
      }
      else if(s->HasNode(temp.s)) {
	//LOG4CXX_INFO(logger,"Disconnecting edge on start tree"<<"\n");
	//disconnect tree from s->t
	temp.s->detachChild(temp.t);
	Assert(temp.t == ns || temp.t->hasDescendent(ns));
	ns->reRoot();
	ns->edgeFromParent() = bridge;
	
	//move nodes in subtree from start arrays to goal arrays
	ChangeTreeCallback changeCallback(s,g);
	ns->DFS(changeCallback);
	ng->addChild(ns);
	Assert(ns->hasAncestor(g->root));
	Assert(s->root->getParent()==NULL);
      }
      else {     //on goal tree
	//LOG4CXX_INFO(logger,"Disconnecting edge on goal tree"<<"\n");
	Assert(g->HasNode(temp.t));
	//disconnect tree from s->t
	temp.t->detachChild(temp.s);
	Assert(temp.s == ng || temp.s->hasDescendent(ng));
	ng->reRoot();
	ng->edgeFromParent() = bridge;
	
	//move nodes in subtree from goal arrays to start arrays
	ChangeTreeCallback changeCallback(g,s);
	ng->DFS(changeCallback);
	ns->addChild(ng);
	Assert(ng->hasAncestor(s->root));
	Assert(g->root->getParent()==NULL);
      }
      outputPath.clear();
      return false;
    }
#endif //USE_PLAN_EXTENSIONS
    if(!temp.e->Done()) q.push(temp);
  }

  //done!
  //LOG4CXX_INFO(logger,"Path checking success"<<"\n");

  //check the reversed flags for the output path
  for(list<EdgeInfo>::iterator i=outputPath.begin();i!=outputPath.end();i++) {
    if(i->reversed) {
      if(*i->s == i->e->Start()) i->reversed=false;
    }
    else {
      if(*i->s == i->e->Goal()) i->reversed=true;
    }
  }
  return true;
}

bool SBLTree::CheckPath(SBLTree* t,Node* ns,Node* ng,MilestonePath& outputPath)
{
  //LOG4CXX_INFO(logger,"Checking path!!!"<<"\n");
  Assert(t->HasNode(ns));
  Assert(ns->hasAncestor(ng) || ng->hasAncestor(ns));
  if(!ng->hasAncestor(ns))
    std::swap(ns,ng);
  Assert(outputPath.edges.empty());
  
  priority_queue<EdgeInfo,vector<EdgeInfo>,LessEdgePriority> q;

  //ns -> ng
  EdgeInfo temp;
  Node* n=ng;
  while(n != ns) {
    temp.s = n->getParent();
    temp.t = n;
    temp.e = n->edgeFromParent();
    temp.reversed=false;
    q.push(temp);

    outputPath.edges.push_back(n->edgeFromParent());
    n=n->getParent();
  }
  std::reverse(outputPath.edges.begin(),outputPath.edges.end());
  
  //adaptive division of path
  Config x;
  while(!q.empty()) {
    temp=q.top(); q.pop();
    if(temp.e->Done()) continue;
    if(!temp.e->Plan()) {
      //disconnect!
      //LOG4CXX_INFO(logger,"Disconnecting edge on start tree"<<"\n");
      t->DeleteSubtree(temp.t);
      outputPath.edges.clear();
      return false;
    }
    if(!temp.e->Done()) q.push(temp);
  }
  //success!
  return true;
}



Node* SBLTree::PickExpand()
{
  FatalError("PickExpand not implemented by subclass");
  return NULL;
}

Node* SBLTree::FindClosest(const Config& x)
{
  //walk through start tree
  ClosestMilestoneCallback callback(space,x);
  root->DFS(callback);
  return callback.closestMilestone;
}



SBLTreeWithIndex::SBLTreeWithIndex(CSpace* space)
  :SBLTree(space)
{}

void SBLTreeWithIndex::Cleanup()
{
  index.resize(0);
}

void SBLTreeWithIndex::AddMilestone(Node* n)
{
  index.push_back(n);
}

void SBLTreeWithIndex::RemoveMilestone(Node* n)
{
  vector<Node*>::iterator i=find(index.begin(),index.end(),n);
  if(i == index.end()) return;
  *i = index.back();
  index.resize(index.size()-1);
}

Node* SBLTreeWithIndex::PickRandom() const
{
  Graph::CountCallback<Node*> count;
  root->DFS(count);
  PickCallback pick(RandInt(count.count));
  root->DFS(pick);
  Assert(pick.res != NULL);
  return pick.res;
}


SBLSubdivision::SBLSubdivision(int mappedDims)
  :subdiv(mappedDims,0.1)
{
  subsetToFull.mapping.resize(mappedDims);
  for(int i=0;i<mappedDims;i++)
    subsetToFull.mapping[i] = i;
  temp.resize(mappedDims);
}

void SBLSubdivision::Clear()
{
  subdiv.Clear();
}

void SBLSubdivision::RandomizeSubset()
{
  Assert(h.n >= subsetToFull.Size());
  int n=h.n;
  vector<int> p(n); //make a permutation
  for(int i=0;i<n;i++) p[i]=i;
  for(size_t i=0;i<subsetToFull.mapping.size();i++) {
    p[i] = p[i+rand()%(n-i)];
    subsetToFull.mapping[i] = p[i];
  }
  subsetToFull.InvMap(h,subdiv.hinv);
  for(int i=0;i<subdiv.hinv.n;i++)
    subdiv.hinv[i] = 1.0/subdiv.hinv[i];
  temp.resize(subsetToFull.Size());
}

void SBLSubdivision::AddPoint(Node* n)
{
  Assert(temp.n == subsetToFull.Size());
  const Vector& p = *n;
  subsetToFull.InvMap(p,temp);

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  subdiv.Insert(index,(void*)n);
}

void SBLSubdivision::RemovePoint(Node* n)
{
  Assert(temp.n == subsetToFull.Size());
  const Vector& p = *n;
  subsetToFull.InvMap(p,temp);

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  bool res=subdiv.Erase(index,(void*)n);
  Assert(res == true);
}

void* RandomObject(const vector<void*>& objs)
{
  Assert(!objs.empty());
  return objs[RandInt(objs.size())];
}


void* RandomObject(const list<void*>& objs)
{
  //pick a random point from objs
  int n=objs.size();
  Assert(n != 0);
  int k=RandInt(n);
  list<void*>::const_iterator obj=objs.begin();
  for(int i=0;i<k;i++,obj++);
  return (*obj);
}

Node* SBLSubdivision::PickPoint(const Config& x)
{
  Assert(temp.n == subsetToFull.Size());
  subsetToFull.InvMap(x,temp);

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  vector<void*>* objs = subdiv.GetObjectSet(index);
  if(!objs) return NULL;

  return (Node*)RandomObject(*objs);
}

Node* SBLSubdivision::PickRandom()
{
  int n=subdiv.buckets.size();
  Assert(n > 0);
  int k=RandInt(n);
  GridSubdivision::HashTable::iterator bucket=subdiv.buckets.begin();
  for(int i=0;i<k;i++,bucket++);
  return (Node*)RandomObject(bucket->second);
}






SBLTreeWithGrid::SBLTreeWithGrid(CSpace* space)
  :SBLTree(space),A(3)
{
}

void SBLTreeWithGrid::Cleanup()
{
  A.Clear();
  SBLTree::Cleanup();
}

void SBLTreeWithGrid::Init(const Config& qStart)
{
  SBLTree::Init(qStart);
  A.Clear();
  A.AddPoint(root);
}

void SBLTreeWithGrid::InitDefaultGrid(int numDims,Real h)
{
  A.h.resize(numDims,h);
}

void SBLTreeWithGrid::RandomizeSubset()
{
  //LOG4CXX_INFO(logger,"SBLTreeWithGrid: Randomizing subset"<<"\n");
  A.Clear();
  A.RandomizeSubset();

  if(root) {
    AddPointCallback callback(A);
    root->DFS(callback);
  }
}

void SBLTreeWithGrid::AddMilestone(Node* n)
{
  A.AddPoint(n);
}

void SBLTreeWithGrid::RemoveMilestone(Node* n)
{
  A.RemovePoint(n);
}

Node* SBLTreeWithGrid::PickExpand()
{
  return A.PickRandom();
}

Node* SBLTreeWithGrid::FindNearby(const Config& x)
{
  Node* n=A.PickPoint(x);
  if(n) return n;
  return A.PickRandom();
}
