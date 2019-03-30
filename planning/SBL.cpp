#include <KrisLibrary/Logger.h>
#include "SBL.h"
#include <math/random.h>
using namespace std;
typedef SBLPlanner::Node Node;

SBLPlanner::SBLPlanner(CSpace* s)
  :space(s),maxExtendDistance(0.2),maxExtendIters(10),edgeConnectionThreshold(Inf),numIters(0),tStart(NULL),tGoal(NULL)
{}

SBLPlanner::~SBLPlanner()
{
  SafeDelete(tStart);
  SafeDelete(tGoal);
}

void SBLPlanner::Cleanup()
{
  SafeDelete(tStart);
  SafeDelete(tGoal);
  outputPath.clear();
  numIters=0;
}

void SBLPlanner::Init(const Config& qStart,const Config& qGoal)
{
  Assert(!tStart && !tGoal);
  tStart = new SBLTreeWithIndex(space);
  tGoal = new SBLTreeWithIndex(space);
  tStart->Init(qStart);
  tGoal->Init(qGoal);
  //LOG4CXX_INFO(KrisLibrary::logger(),"SBL: Distance "<<space->Distance(qStart,qGoal));
  //LOG4CXX_INFO(KrisLibrary::logger(),"SBL: Distance "<<space->Distance(*tStart,*tGoal));
  if(CheckPath(tStart->root,tGoal->root)) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"SBLPlanner::Init(): Start and goal connected!");
  }
}

bool SBLPlanner::Extend()
{
  numIters++;
  int useStart = RandBool();
  SBLTree *s, *g;
  if(useStart) { s=tStart; g=tGoal; }
  else { s=tGoal; g=tStart; }
  Node* ns=s->Extend(maxExtendDistance,maxExtendIters);
  if(ns) {
    Node* ng=PickConnection(g,*ns);
    if(s == tStart) return CheckPath(ns,ng);
    else return CheckPath(ng,ns);
  }
  return false;
}

bool SBLPlanner::CheckPath(Node* nStart,Node* nGoal)
{
  Assert(outputPath.empty());
  if(!IsInf(edgeConnectionThreshold)) {
    if(space->Distance(*nStart,*nGoal) > edgeConnectionThreshold) return false;
  }
  return SBLTree::CheckPath(tStart,nStart,tGoal,nGoal,outputPath);
}

void CreateMilestonePath(const list<SBLTree::EdgeInfo>& in,MilestonePath& out)
{
  Assert(!in.empty());
  out.edges.resize(in.size());
  int index=0;
  for(list<SBLTree::EdgeInfo>::const_iterator i=in.begin();i!=in.end();i++) {
    if(i->reversed) out.edges[index] = i->e->ReverseCopy();
    else out.edges[index] = i->e->Copy();
    Assert(out.edges[index]->Start() == *i->s);
    Assert(out.edges[index]->End() == *i->t);
    index++;
  }
}

void SBLPlanner::CreatePath(MilestonePath& path) const
{
  CreateMilestonePath(outputPath,path);
  Assert(path.edges.front()->Start() == *tStart->root);
  Assert(path.edges.back()->End() == *tGoal->root);
}

SBLPlannerWithGrid::SBLPlannerWithGrid(CSpace* s)
  :SBLPlanner(s),numItersPerRandomize(50),gridDivision(0.1)
{}

void SBLPlannerWithGrid::Init(const Config& qStart,const Config& qGoal)
{
  SBLTreeWithGrid* s= new SBLTreeWithGrid(space);
  SBLTreeWithGrid* g= new SBLTreeWithGrid(space);
  tStart = s;
  tGoal = g;

  s->A.h.resize(qStart.n,gridDivision);
  g->A.h.resize(qStart.n,gridDivision);
  //this is kinda stupid, should figure out a cleaner way to do this
  if(qStart.n < (int)s->A.mappedDims.size()) {
    s->A.Randomize(qStart.n,qStart.n,gridDivision);
    g->A.Randomize(qStart.n,qStart.n,gridDivision);
  }

  s->Init(qStart);
  g->Init(qGoal);
  if(CheckPath(s->root,g->root)) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"SBLPlanner::Init(): Start and goal connected!");
  }
}

Node* SBLPlannerWithGrid::PickConnection(SBLTree* t,const Config& x)
{
  SBLTreeWithGrid* s=(SBLTreeWithGrid*)t;
  return s->FindNearby(x);
}

void SBLPlannerWithGrid::Cleanup()
{
  SBLPlanner::Cleanup();
}

bool SBLPlannerWithGrid::Extend()
{
  if((numIters+1) % numItersPerRandomize == 0) RandomizeSubset();

  return SBLPlanner::Extend();
}

void SBLPlannerWithGrid::RandomizeSubset()
{
  SBLTreeWithGrid* s=(SBLTreeWithGrid*)tStart;
  SBLTreeWithGrid* g=(SBLTreeWithGrid*)tGoal;
  s->RandomizeSubset();
  g->RandomizeSubset();
}

SBLPRT::SBLPRT(CSpace* s)
  :space(s),maxExtendDistance(0.2),maxExtendIters(10),
   defaultPPickClosestTree(0.2),defaultPPickClosestNode(0.0),
   numIters(0)
{}

SBLPRT::~SBLPRT()
{
  for (size_t i = 0; i<roadmap.nodes.size(); i++)
    delete roadmap.nodes[i];
}

void SBLPRT::Cleanup()
{
  for(size_t i=0;i<roadmap.nodes.size();i++)
    delete roadmap.nodes[i];
  roadmap.Cleanup();
  numIters=0;
}

int SBLPRT::AddSeed(const Config& q)
{
  //SBLTree* t = new SBLTreeWithIndex(space);
  SBLTreeWithGrid* t = new SBLTreeWithGrid(space);
  t->A.h.resize(q.n,0.1);
  t->RandomizeSubset();
  ccs.AddNode();
  t->Init(q);
  Assert(space->IsFeasible(q));
  return roadmap.AddNode(t);
}

pair<int,int> SBLPRT::Expand()
{
  numIters++;
  if(numIters % 50 == 0) {
    for(size_t i=0;i<roadmap.nodes.size();i++)
      ((SBLTreeWithGrid*)roadmap.nodes[i])->RandomizeSubset();
  }
  int t=RandInt(roadmap.nodes.size());
  if(!IsSeedFullyConnected(t)) {
    int g=ExpandTree(t);
    if(g >= 0)  return pair<int,int>(t,g);
  }
  return pair<int,int>(-1,-1);
}

int SBLPRT::ExpandTree(int t)
{
  //LOG4CXX_INFO(KrisLibrary::logger(),"SBLPRT: Extend\n");
  Node* n=roadmap.nodes[t]->Extend(maxExtendDistance,maxExtendIters);
  if(!n) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"SBLPRT: No extend...\n");
    return -1;
  }
  //LOG4CXX_INFO(KrisLibrary::logger(),"SBLPRT: PickConnection\n");
  pair<int,Node*> con=PickConnection(t,n);
  int tg = con.first;
  Node* ng = con.second;
  if(tg < 0 && ng == NULL) { LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, picked a nonexistent connection"); return -1; }

  MilestonePath* p=roadmap.FindEdge(t,tg);
  Assert(p != NULL);
  Assert(p->edges.empty());
  
  list<EdgeInfo> outputPath;
  if(SBLTree::CheckPath(roadmap.nodes[t],n,roadmap.nodes[tg],ng,outputPath)) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Connecting nodes "<<t<<" to "<<tg);
    CreateMilestonePath(outputPath,*p);
    ccs.AddEdge(t,tg);
    return tg;
  }
  //LOG4CXX_INFO(KrisLibrary::logger(),"SBLPRT: No connect...\n");
  return -1;
}

void SBLPRT::AddRoadmapEdgesIfBelowThreshold(Real distanceThreshold)
{
  int n=roadmap.NumNodes();
  for(int i=0;i<n;i++)
    for(int j=i+1;i<n;i++) 
      if(space->Distance(*roadmap.nodes[i]->root,*roadmap.nodes[j]->root) < distanceThreshold)
	AddRoadmapEdge(i,j);
    
}

bool SBLPRT::IsEdgeConnected(int i,int j) const
{
  const MilestonePath*e=roadmap.FindEdge(i,j);
  if(!e) return false;
  //Assert(e!=NULL);
  return !e->edges.empty();
}

bool SBLPRT::IsSeedFullyConnected(int i) const
{
  Roadmap::Iterator e;
  for(roadmap.Begin(i,e);!e.end();e++) {
    if(e->edges.empty()) {
      if(ccs.GetComponent(e.source()) != ccs.GetComponent(e.target()))
	return false;
    }
  }
  return true;
}

struct ConnectedSeedCallback : public Graph::PathIntCallback
{
  SBLPRT* prt;

  ConnectedSeedCallback(SBLPRT* _prt,int seeknode) :Graph::PathIntCallback(_prt->roadmap.NumNodes(),seeknode),prt(_prt) {}
  virtual bool ForwardEdge(int i,int j)
  { 
    MilestonePath* p=prt->roadmap.FindEdge(i,j);
    Assert(p!=NULL);
    if(p->edges.empty()) return false;
    Assert(parents[j]==-1);
    parents[j]=i; 
    return true;
  }
};

/*
bool SBLPRT::AreSeedsConnected(int i,int j)
{
  ConnectedSeedCallback callback(this,j);
  callback.node = j;
  roadmap.NewTraversal();
  roadmap._DFS(i,callback);
  if(callback.found) return true;
  return false;
}
*/

struct PathInfo
{
  int tstart,tgoal;
  MilestonePath* path;
};

void SBLPRT::CreatePath(int i,int j,MilestonePath& path)
{
  Assert(i >= 0 && i < (int)roadmap.nodes.size());
  Assert(j >= 0 && j < (int)roadmap.nodes.size());
  ConnectedSeedCallback callback(this,j);
  callback.node = j;
  roadmap.NewTraversal();
  roadmap._BFS(i,callback);
  if(!callback.found) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SBLPRT::CreatePath: Warning, a path doesn't exist between nodes "<<i<<" and "<<j);
    return;
  }

  list<PathInfo> subpaths;
  int n=j;
  while(n != i) {
    int p=callback.parents[n];
    Assert(p >= 0);
    PathInfo temp;
    temp.tstart = p;
    temp.tgoal = n;
    temp.path = roadmap.FindEdge(n,p);
    Assert(temp.path != NULL);
    Assert(!temp.path->edges.empty());
    subpaths.push_front(temp);
    n=p;
  }
  for(list<PathInfo>::iterator it=subpaths.begin();it!=subpaths.end();it++) {
    int s=it->tstart;
    int g=it->tgoal;
    Assert(s >= 0 && s <(int)roadmap.nodes.size());
    Assert(g >= 0 && g <(int)roadmap.nodes.size());
    MilestonePath& subpath=*it->path;
    Assert(!subpath.edges.empty());
    //forward path or backward path?
    if(subpath.edges.front()->Start() == *roadmap.nodes[s]->root) {
      Assert(subpath.edges.back()->End() == *roadmap.nodes[g]->root);
      //forward path
      path.Concat(subpath);
            if(!path.IsValid()) LOG4CXX_ERROR(KrisLibrary::logger(),"SBLPRT::CreatePath: Path invalidated on "<<s<<" "<<g);
    }
    else {
      Assert(subpath.edges.front()->Start() == *roadmap.nodes[g]->root);
      Assert(subpath.edges.back()->End() == *roadmap.nodes[s]->root);
      //backward path
      for(int k=(int)subpath.edges.size()-1;k>=0;k--) {
	path.edges.push_back(subpath.edges[k]->ReverseCopy());
      }
            if(!path.IsValid()) LOG4CXX_ERROR(KrisLibrary::logger(),"SBLPRT::CreatePath: Path invalidated on backwards "<<s<<" "<<g);
    }
  }
  Assert(path.IsValid());
}

pair<int,Node*> SBLPRT::PickConnection(int t,Node* n)
{
  pair<int,Node*> res(-1,(Node*)NULL);
  if(RandBool(defaultPPickClosestTree)) res.first=PickClosestAdjacentTree(t,*n);
  else res.first=PickRandomAdjacentTree(t);
  if(res.first >= 0) {
    if(RandBool(defaultPPickClosestNode)) res.second=GetClosestNode(res.first,*n);
    else res.second=PickNode(res.first);
  }
  return res;
}

int SBLPRT::PickRandomAdjacentTree(int t)
{
  vector<int> adj;
  Roadmap::Iterator e;
  for(roadmap.Begin(t,e);!e.end();e++) {
    if(e->edges.empty() && (ccs.GetComponent(t) != ccs.GetComponent(e.target()))) adj.push_back(e.target());
  }
  if(adj.empty()) return -1;
  return adj[RandInt(adj.size())];
}

int SBLPRT::PickClosestAdjacentTree(int t,const Config& x)
{
  int closest=-1;
  Real closestDist=Inf;
  Roadmap::Iterator e;
  for(roadmap.Begin(t,e);!e.end();e++) {
    if(e->edges.empty() && (ccs.GetComponent(t) != ccs.GetComponent(e.target()))) {
      const Config& y=*roadmap.nodes[e.target()]->root;
      Real d=space->Distance(x,y);
      if(d < closestDist) {
	closest = e.target();
	closestDist = d;
      }
    }
  }
  return closest;
}


