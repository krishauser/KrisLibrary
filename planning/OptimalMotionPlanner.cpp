#include "OptimalMotionPlanner.h"
#include <math/random.h>
#include <graph/Path.h>
#include <Timer.h>

class EdgeDistance
{
 public:
  Real operator () (const SmartPointer<EdgePlanner>& e,int s,int t)
  {
    assert(e->Space() != NULL);
    return e->Space()->Distance(e->Start(),e->Goal());
  }
};


PRMStarPlanner::PRMStarPlanner(CSpace* space)
  :RoadmapPlanner(space),spp(roadmap),lazy(false),randomNeighbors(false),connectByRadius(false),connectRadiusConstant(1),connectionThreshold(Inf)
{}

void PRMStarPlanner::Init(const Config& qstart,const Config& qgoal)
{
  Cleanup();
  start = AddMilestone(qstart);
  goal = AddMilestone(qgoal);
  spp.InitializeSource(start);  

  numPlanSteps = 0;
  numEdgeChecks = 0;
  tCheck=tKnn=tConnect=tLazy=0;
}
void PRMStarPlanner::PlanMore()
{
  numPlanSteps ++;
  EdgeDistance distanceWeightFunc;
  Vector x;
  Timer timer;
  GenerateConfig(x);
  if(!space->IsFeasible(x)) {
    tCheck += timer.ElapsedTime();
    return;
  }
  tCheck += timer.ElapsedTime();
  timer.Reset();
  int m = -1;

  vector<int> neighbors;
  if(connectByRadius) {
    Real rad = connectRadiusConstant*Pow(Log(Real(roadmap.nodes.size()))/Real(roadmap.nodes.size()),1.0/x.n);
    if(rad > connectionThreshold) rad = connectionThreshold;
    KNN(x,1,neighbors);
    int nearest = neighbors[0];
    Neighbors(x,rad,neighbors);
    if(neighbors.empty())
      neighbors.push_back(nearest);
  }
  else {
    int kmax = int(((1.0+1.0/x.n)*E)*Log(Real(roadmap.nodes.size())));
    assert(kmax >= 1);
    if(kmax > (int)roadmap.nodes.size()-1)
      kmax = roadmap.nodes.size()-1;
    KNN(x,kmax,neighbors);
  }  
  tKnn += timer.ElapsedTime();
  timer.ElapsedTime();

  Real goalDist = spp.d[goal];
  for(size_t i=0;i<neighbors.size();i++) {
    int n = neighbors[i];
    if(n == m || (m >= 0 && roadmap.HasEdge(m,n)))
      continue;
    if(lazy && (n!=goal && IsInf(spp.d[n])))
      continue;
    //check for shorter connections into m and neighbors[i]
    Assert(n >= 0 && n < roadmap.nodes.size() && n != m);
    Real d = space->Distance(x,roadmap.nodes[n]);
    if(d > connectionThreshold) continue;

    if(m==-1) {
      m=AddMilestone(x);  
      Assert(m==(int)spp.p.size());
      spp.p.push_back(-1);
      spp.d.push_back(Inf);
    }

    const Config& xn = roadmap.nodes[n];
    bool add = false;
    SmartPointer<EdgePlanner> e;
    //don't connect points that aren't potentially optimal
    if(!lazy || spp.d[n] + d < spp.d[m] || spp.d[m] + d < spp.d[n]) {
      e = space->LocalPlanner(x,xn);
      Assert(e->Space() != NULL);
      if(!lazy) { numEdgeChecks++; if(e->IsVisible()) add=true; }
      else add=true;
    }
    if(add) {
      roadmap.AddEdge(m,n,e);
      if(lazy) 
	spp.DecreaseUpdate_Undirected(m,n,distanceWeightFunc);
    }
  }
  tConnect += timer.ElapsedTime();
  if(lazy) {
    if(spp.d[goal] < goalDist) {
      //found an improved path to the goal! do checking
      timer.Reset();
      CheckPath(start,goal);
      tLazy += timer.ElapsedTime();
    }
  }
}

void PRMStarPlanner::Neighbors(const Config& x,Real rad,vector<int>& neighbors)
{
  set<pair<Real,int> > nn;
  if(!randomNeighbors) {
    //radius rad neighbors
    for(int i=0;i<roadmap.nodes.size();i++) {
      Real d=space->Distance(roadmap.nodes[i],x);
      if(d > 0 && d < rad) {
	nn.insert(pair<Real,int>(d,i));
      }
    }
  }
  else {
    int num = int(Log(Real(roadmap.nodes.size())));
    for(int k=0;k<num;k++) {
      int i = RandInt(roadmap.nodes.size());
      Real d=space->Distance(roadmap.nodes[i],x);
      if(d > 0 && d < rad) {
	nn.insert(pair<Real,int>(d,i));
      }
    }
  }
  neighbors.resize(0);
  for(set<pair<Real,int> >::const_iterator j=nn.begin();j!=nn.end();j++) {
    neighbors.push_back(j->second);
  }
}

void PRMStarPlanner::KNN(const Config& x,int k,vector<int>& neighbors)
{
  //k nearest vs k random neighbors
  neighbors.resize(k);
  if(!randomNeighbors) {
    set<pair<Real,int> > knn;
    Real dmax = Inf;
    for(int i=0;i<roadmap.nodes.size();i++) {
      Real d=space->Distance(roadmap.nodes[i],x);
      if(d > 0 && d < dmax) {
	pair<Real,int> idx(d,i);
	knn.insert(idx);
	if((int)knn.size() > k)
	  knn.erase(--knn.end());
	dmax = (--knn.end())->first;
      }
    }
    neighbors.resize(0);
    for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
      neighbors.push_back(j->second);
    }
  }
  else {
    set<pair<Real,int> > knn;
    for(int i=0;i<k*2;i++) {
      int n = RandInt(roadmap.nodes.size()-1);
      Real d=space->Distance(roadmap.nodes[n],x);
      knn.insert(pair<Real,int>(d,n));
    }
    neighbors.resize(0);
    for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
      neighbors.push_back(j->second);
    }
    neighbors.resize(k);
  }
}

bool PRMStarPlanner::GetPath(MilestonePath& path)
{
  vector<int> nodes;
  return GetPath(start,goal,nodes,path);
}

bool PRMStarPlanner::GetPath(int a,int b,vector<int>& nodes,MilestonePath& path)
{
  if(!lazy) {
    EdgeDistance distanceWeightFunc;
    spp.InitializeSource(a);
    spp.FindPath_Undirected(b,distanceWeightFunc);
  }
  else
    Assert(a==start);
  if(IsInf(spp.d[b])) return false;
  if(!Graph::GetAncestorPath(spp.p,b,a,nodes)) return false;
  path.edges.resize(0);
  for(size_t i=0;i+1<nodes.size();i++) {
    SmartPointer<EdgePlanner>* e=roadmap.FindEdge(nodes[i],nodes[i+1]);
    Assert(e != NULL);
    if((*e)->Start() == roadmap.nodes[nodes[i]])
      path.edges.push_back(*e);
    else {
      Assert((*e)->Start() == roadmap.nodes[nodes[i+1]]);
      path.edges.push_back((*e)->ReverseCopy());
    }
  }
  return true;
}

bool PRMStarPlanner::CheckPath(int a,int b)
{
  EdgeDistance distanceWeightFunc;
  Assert(lazy);
  Assert(a==start);
  //printf("Done with planning step\n");
  while(!IsInf(spp.d[b])) {
    //printf("Lazy collision checking, existing cost %g, potential %g\n",goalDist,spp.d[b]);
    vector<int> npath;
    MilestonePath path;
    if(!GetPath(start,goal,npath,path)) {
      //may have disconnected goal -- need to refresh
      //printf("GetPath returned false\n");
      return false;
    }
    bool feas = true;
    for(size_t k=0;k<path.edges.size();k++) {
      int i;
      if(k%2==0)
	i=(int)k/2;
      else
	i=(int)path.edges.size()-1-(int)k/2;
      SmartPointer<EdgePlanner>* e = roadmap.FindEdge(npath[i],npath[i+1]);
      numEdgeChecks++;
      if(!(*e)->IsVisible()) {
	//delete edge
	//printf("Deleting edge %d %d...\n",npath[i],npath[i+1]);
	roadmap.DeleteEdge(npath[i],npath[i+1]);
	spp.DeleteUpdate_Undirected(npath[i],npath[i+1],distanceWeightFunc);
	//Assert(spp.HasShortestPaths_Undirected(0,distanceWeightFunc));
	feas = false;
	break;
      }
    }
    /*
    if(!feas) {
      //TODO: make this dynamic shortest path update
      spp.InitializeSource(0);
      spp.FindAllPaths_Undirected(distanceWeightFunc);
      //printf("Resulting cost candidate %g...\n",spp.d[1]);
    }
    */
    
    if(feas) {
      return true;
    }
  }
  return false;
}
