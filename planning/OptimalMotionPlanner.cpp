#include "OptimalMotionPlanner.h"
#include "PointLocation.h"
#include <math/random.h>
#include <graph/Path.h>
#include <Timer.h>

//if this is on, this will check any optimal edges as they are added
#define PRECHECK_OPTIMAL_EDGES 1
#define RRG_BIDIRECTIONAL 0
//Do not do heuristic pruning with RRG_BIDIRECTIONAL
#define HEURISTIC_PRUNING 0
#define ELLIPSOID_PRUNING 1

class EdgeDistance
{
 public:
  Real operator () (const SmartPointer<EdgePlanner>& e,int s,int t)
  {
    assert(e->Space() != NULL);
    Real res = e->Space()->Distance(e->Start(),e->Goal());
    if(res <= 0) {
      printf("PRMStarPlanner: Warning, edge has nonpositive length %g\n",res);
      return Epsilon;
    }
    return res;
  }
};


PRMStarPlanner::PRMStarPlanner(CSpace* space)
  :RoadmapPlanner(space),lazy(false),rrg(false),connectByRadius(false),connectRadiusConstant(1),connectionThreshold(Inf),lazyCheckThreshold(Inf),spp(roadmap),sppGoal(roadmap)
{}

void PRMStarPlanner::Init(const Config& qstart,const Config& qgoal)
{
  Cleanup();
  visibleEdges.clear();
  start = AddMilestone(qstart);
  goal = AddMilestone(qgoal);
  Assert(start == 0 && goal == 1);
  spp.InitializeSource(start);  
  sppGoal.InitializeSource(goal);

  numPlanSteps = 0;
  numEdgeChecks = 0;
  numEdgePrechecks = 0;
  tCheck=tKnn=tConnect=tLazy=tLazyCheck=0;
}
void PRMStarPlanner::PlanMore()
{
  numPlanSteps ++;
  EdgeDistance distanceWeightFunc;
  Vector x;

  bool useSpp = (rrg || lazy);
#if PRECHECK_OPTIMAL_EDGES
  bool useSppGoal = (lazy);
#else
  bool useSppGoal = false;
#endif

  Real goalDist = spp.d[goal];
  Real fudgeFactor = 1.0+spp.epsilon;
  //Real fudgeFactor = 1.0;

  Timer timer;
  int m = -1;
  if(!rrg) {
    //PRM* expansion strategy
    GenerateConfig(x);
#if ELLIPSOID_PRUNING
    if((space->Distance(roadmap.nodes[start],x)+space->Distance(x,roadmap.nodes[goal]))*fudgeFactor >= goalDist) {
      return;
    }
#endif
    if(!space->IsFeasible(x)) {
      tCheck += timer.ElapsedTime();
      return;
    }
    tCheck += timer.ElapsedTime();
  }
  else {
    //RRT/RRG expansion strategy
    GenerateConfig(x);
    //KNN call has a bit of duplication with connectByRadius, below
    int nn;
    vector<int> neighbors;
    if(RRG_BIDIRECTIONAL || lazy) {
      KNN(x,1,neighbors);
      nn = neighbors[0];
      tKnn += timer.ElapsedTime();
    }
    else {
      //this is a really inefficient way of getting the closest neighbor
      //connected to the start
      bool good = false;
      for(int i=1;i<100;i+=10) {
	KNN(x,i,neighbors);
	//only connect to nodes attached to start
	for(size_t j=0;j<neighbors.size();j++) {
	  nn = neighbors[j];
#if HEURISTIC_PRUNING
	  if(spp.d[nn]*fudgeFactor < goalDist) {
	    good = true;
	    break;
	  }
#else
	  if(!IsInf(spp.d[nn])) {
	    good = true;
	    break;
	  }
#endif //HEURISTIC_PRUNING
	}
	if(good) break;
      }
      tKnn += timer.ElapsedTime();
      if(!good) return;
    } //not RRT_BIDIRECTIONAL
    timer.Reset();
    Real rad = connectRadiusConstant*Pow(Log(Real(roadmap.nodes.size()))/Real(roadmap.nodes.size()),1.0/x.n);
    if(rad > connectionThreshold) rad = connectionThreshold;
    Real d = space->Distance(roadmap.nodes[nn],x);
    if(d > rad) {
      Config xnew;
      space->Interpolate(roadmap.nodes[nn],x,rad/d,xnew);
      x = xnew;
      d = rad;
    }
#if ELLIPSOID_PRUNING
    if((space->Distance(roadmap.nodes[start],x)+space->Distance(x,roadmap.nodes[goal]))*fudgeFactor >= goalDist) {
      return;
    }
#endif
    if(!space->IsFeasible(x)) {
      tCheck += timer.ElapsedTime();
      return;
    }
    tCheck += timer.ElapsedTime();
    timer.Reset();

    SmartPointer<EdgePlanner> e = space->LocalPlanner(x,roadmap.nodes[nn]);
    bool efeasible = false;
    if(!lazy || d > lazyCheckThreshold) {
      numEdgeChecks++;
      if(!e->IsVisible()) {
	tConnect += timer.ElapsedTime();
	return;
      }
      efeasible = true;
      tConnect += timer.ElapsedTime();
    }

    m=AddMilestone(x);
    if(efeasible) visibleEdges.insert(pair<int,int>(nn,m));
    roadmap.AddEdge(m,nn,e);
    Assert(m==(int)spp.p.size());
    spp.p.push_back(-1);
    spp.d.push_back(Inf);
    if(useSpp) {
      spp.DecreaseUpdate_Undirected(m,nn,distanceWeightFunc);
    }
    if(useSppGoal) {
      sppGoal.p.push_back(-1);
      sppGoal.d.push_back(Inf);
      sppGoal.DecreaseUpdate_Undirected(m,nn,distanceWeightFunc);
    }
  }

  timer.Reset();
  vector<int> neighbors;
  if(connectByRadius) {
    Real rad = connectRadiusConstant*Pow(Log(Real(roadmap.nodes.size()))/Real(roadmap.nodes.size()),1.0/x.n);
    if(rad > connectionThreshold) rad = connectionThreshold;
    Neighbors(x,rad,neighbors);
    if(!rrg && neighbors.empty()) {
      KNN(x,1,neighbors);
    }
  }
  else {
    int kmax = int(((1.0+1.0/x.n)*E)*Log(Real(roadmap.nodes.size())));
    assert(kmax >= 1);
    if(kmax > (int)roadmap.nodes.size()-1)
      kmax = roadmap.nodes.size()-1;
    KNN(x,kmax,neighbors);
  }  
  tKnn += timer.ElapsedTime();
  timer.Reset();

  //sort neighbors with respect to increasing d[n] + d
  {
    vector<pair<Real,int> > queue;
    for(size_t i=0;i<neighbors.size();i++) {
      int n = neighbors[i];
      if(n == m || (m >= 0 && roadmap.HasEdge(m,n)))
	continue;
      if(rrg && (n!=goal && IsInf(spp.d[n])))
	continue;
      Real d = space->Distance(x,roadmap.nodes[n]);
      if(d > connectionThreshold) continue;
      queue.push_back(pair<Real,int>(spp.d[n]+d,n));
    }
    sort(queue.begin(),queue.end());
    neighbors.resize(queue.size());
    for(size_t i=0;i<queue.size();i++)
      neighbors[i] = queue[i].second;
  }

  for(size_t i=0;i<neighbors.size();i++) {
    //check for shorter connections into m and neighbors[i]
    int n = neighbors[i];
    /*
    //this checking becomes unnecessary due to neighborhood sorting above
    if(n == m || (m >= 0 && roadmap.HasEdge(m,n)))
      continue;
    if(rrg && (n!=goal && IsInf(spp.d[n])))
      continue;
    Assert(n >= 0 && n < (int)roadmap.nodes.size() && n != m);
    Real d = space->Distance(x,roadmap.nodes[n]);
    if(d > connectionThreshold) continue;
    */

    Real d = space->Distance(x,roadmap.nodes[n]);

    if(m==-1) {
      m=AddMilestone(x);  
      Assert(m==(int)spp.p.size());
      spp.p.push_back(-1);
      spp.d.push_back(Inf);
      if(useSppGoal) {
	sppGoal.p.push_back(-1);
	sppGoal.d.push_back(Inf);
      }
    }

    bool add = false;
    SmartPointer<EdgePlanner> e;
    //don't connect points that aren't potentially optimal
    if(!rrg || (spp.d[n] + d)*fudgeFactor < spp.d[m] || (spp.d[m] + d)*fudgeFactor < spp.d[n]) {
      const Config& xn = roadmap.nodes[n];
      e = space->LocalPlanner(x,xn);
      Assert(e->Space() != NULL);

      //non-lazy -- check all edges
      bool doCheck = !lazy;
      if(lazy) {
#if PRECHECK_OPTIMAL_EDGES
	//if this edge will become part of an optimal path -- check it now
	//rather than waiting for the lazy update
	if((spp.d[n] + d + sppGoal.d[m])*fudgeFactor < goalDist || (spp.d[m] + d + sppGoal.d[n])*fudgeFactor < goalDist) {
	  doCheck=true;
	  numEdgePrechecks ++;
	}
#endif
	//long enough -- check it now
	if(d > lazyCheckThreshold) {
	  doCheck = true;
	}
      }

      if(doCheck) {
	//do the check
	numEdgeChecks++;
	if(e->IsVisible()) {
	  pair<int,int> epair(m,n);
	  if(epair.first > epair.second)
	    swap(epair.first,epair.second);
	  visibleEdges.insert(epair);
	  add=true; 
	}
      }
      else
	//add it now without checking, lazy style
	add = true;
    }
    /*else {
      if(n==goal) printf("Candidate neighbor to goal rejected due to cost: %g+%g > %g (fudge factor %g)\n",spp.d[m],d,spp.d[n],fudgeFactor);
    }
    */

    if(add) {
      roadmap.AddEdge(m,n,e);
      if(useSpp) 
	spp.DecreaseUpdate_Undirected(m,n,distanceWeightFunc);
      if(useSppGoal)
	sppGoal.DecreaseUpdate_Undirected(m,n,distanceWeightFunc);
    }
  }
  tConnect += timer.ElapsedTime();
  if(lazy) {    
    static const int batchSize=100;
    static int lastCheckTime = 0;
    if((spp.epsilon > 0 && numPlanSteps >= lastCheckTime+batchSize) || spp.d[goal] < goalDist) {
      lastCheckTime = numPlanSteps;
      //found an improved path to the goal! do checking
      timer.Reset();
      //printf("Candidate improvement to cost %g\n",spp.d[goal]);
      if(CheckPath(start,goal)) {
	//printf("Found an improved path to goal with cost %g (before it was %g)\n",spp.d[goal],goalDist);
      }
      tLazy += timer.ElapsedTime();
    }
  }
}

void PRMStarPlanner::Neighbors(const Config& x,Real rad,vector<int>& neighbors)
{
  vector<Real> distances;
  bool res=pointLocator->Close(x,rad,neighbors,distances);
  Assert(res==true);
  /*
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
  */
}

void PRMStarPlanner::KNN(const Config& x,int k,vector<int>& neighbors)
{
  vector<Real> distances;
  bool res=pointLocator->KNN(x,k,neighbors,distances);
  Assert(res==true);
  /*
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
  */
}

bool PRMStarPlanner::GetPath(MilestonePath& path)
{
  vector<int> nodes;
  return GetPath(start,goal,nodes,path);
}

bool PRMStarPlanner::HasPath() const
{
  bool useSpp = (rrg || lazy);
  if(!useSpp) {
    EdgeDistance distanceWeightFunc;
    ShortestPathProblem spptemp(roadmap);
    spptemp.InitializeSource(0);
    spptemp.FindPath_Undirected(1,distanceWeightFunc);
    if(IsInf(spptemp.d[1])) return false;
    return true;
  }
  if(IsInf(spp.d[1])) return false;
  return true;
}

bool PRMStarPlanner::GetPath(int a,int b,vector<int>& nodes,MilestonePath& path)
{
  bool useSpp = (rrg || lazy);
  if(!useSpp) {
    EdgeDistance distanceWeightFunc;
    spp.InitializeSource(a);
    spp.FindPath_Undirected(b,distanceWeightFunc);
  }
  else {
    Assert(a==start);
    //EdgeDistance distanceWeightFunc;
    //Assert(spp.HasShortestPaths_Undirected(a,distanceWeightFunc));
  }
  if(IsInf(spp.d[b])) return false;
  if(!Graph::GetAncestorPath(spp.p,b,a,nodes)) {
    printf("PRMStarPlanner: Unable to find path from %d to %d\n",a,b);
    printf("node,distance,parent,edge weight,edge feasible\n");
    for(int i=0;i<20;i++) {
      Real w = 0;
      int feas = 0;
      if(spp.p[b] >= 0) {
	SmartPointer<EdgePlanner>* e=roadmap.FindEdge(b,spp.p[b]);
	Assert(e != NULL);
	w=(*e)->Space()->Distance((*e)->Start(),(*e)->Goal());
	feas = (*e)->IsVisible();
      }
      printf("%d,%g,%d,%g,%d\n",b,spp.d[b],spp.p[b],w,feas);
      b = spp.p[b];
      if(b < 0) break;
    }
    return false;
  }
  for(size_t i=0;i+1<nodes.size();i++) 
    Assert(spp.d[nodes[i+1]] > spp.d[nodes[i]]);
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
#if PRECHECK_OPTIMAL_EDGES
  bool useSppGoal = true;
#else
  bool useSppGoal = false;
#endif
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
    Timer timer;
    bool feas = true;
    for(size_t k=0;k<path.edges.size();k++) {
      int i;
      if(k%2==0)
	i=(int)k/2;
      else
	i=(int)path.edges.size()-1-(int)k/2;
      Assert(i >= 0 && i+1 < (int)npath.size());
      //check if it's been determined to be visible
      pair<int,int> epair(npath[i],npath[i+1]);
      if(epair.first > epair.second)
	swap(epair.first,epair.second);
      if(visibleEdges.count(epair)!=0) continue;
      //it's not, now check it
      SmartPointer<EdgePlanner>* e = roadmap.FindEdge(npath[i],npath[i+1]);
      numEdgeChecks++;
      if(!(*e)->IsVisible()) {
	tLazyCheck += timer.ElapsedTime();
	//delete edge
	//printf("Deleting edge %d %d...\n",npath[i],npath[i+1]);
	roadmap.DeleteEdge(npath[i],npath[i+1]);
	spp.DeleteUpdate_Undirected(npath[i],npath[i+1],distanceWeightFunc);
	if(useSppGoal)
	  sppGoal.DeleteUpdate_Undirected(npath[i],npath[i+1],distanceWeightFunc);
	//Assert(spp.HasShortestPaths_Undirected(0,distanceWeightFunc));
	feas = false;
	break;
      }
      else
	visibleEdges.insert(epair);
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
      tLazyCheck += timer.ElapsedTime();
      Assert(path.IsFeasible());
      return true;
    }
  }
  return false;
}
