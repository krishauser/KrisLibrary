#include "OptimalMotionPlanner.h"
#include "PointLocation.h"
#include "GeneralizedAStar.h"
#include <math/random.h>
#include <graph/Path.h>
#include <Timer.h>

//if this is on, this will check any optimal edges as they are added
#define PRECHECK_OPTIMAL_EDGES 1
//Do not do heuristic pruning with bidirectional (this doesn't seem to help much with lazy)
#define HEURISTIC_PRUNING 1
//Prunes new configurations whose straight line distance to start and goal
//is greater than the current goal cost
#define ELLIPSOID_PRUNING 1

//TEST: An incremental version of FMT*
#define TEST_FMT 0


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

PRMStarPlanner* gCurrentOptimalMotionPlanner = NULL;
Real gCurrentDGoal = Inf;
bool connectedToStartFilter(int n)
{
  Assert(gCurrentOptimalMotionPlanner != NULL);
  bool useSppLB = (gCurrentOptimalMotionPlanner->lazy || (gCurrentOptimalMotionPlanner->rrg && gCurrentOptimalMotionPlanner->suboptimalityFactor > 0));
  Real dnn = (useSppLB ? gCurrentOptimalMotionPlanner->sppLB.d[n] : gCurrentOptimalMotionPlanner->spp.d[n]);
  //with lazy, there may be lots of good neighbors that are just poorly connected
  if(!gCurrentOptimalMotionPlanner->lazy && HEURISTIC_PRUNING) {
    if((dnn+gCurrentDGoal)*(1.0+gCurrentOptimalMotionPlanner->suboptimalityFactor) < gCurrentOptimalMotionPlanner->spp.d[gCurrentOptimalMotionPlanner->goal]) {
      return true;
    }
    return false;
  }
  else {
    return !IsInf(dnn);
  }
}

PRMStarPlanner::PRMStarPlanner(CSpace* space)
  :RoadmapPlanner(space),lazy(false),rrg(false),bidirectional(true),connectByRadius(false),connectRadiusConstant(1),connectionThreshold(Inf),lazyCheckThreshold(Inf),suboptimalityFactor(0),spp(roadmap),sppGoal(roadmap),sppLB(LBroadmap),sppLBGoal(LBroadmap)
{}

void PRMStarPlanner::Cleanup()
{
  RoadmapPlanner::Cleanup();
  spp.p.clear();
  spp.d.clear();
  sppGoal.p.clear();
  sppGoal.d.clear();
  sppLB.p.clear();
  sppLB.d.clear();
  sppLBGoal.p.clear();
  sppLBGoal.d.clear();
}

void PRMStarPlanner::Init(const Config& qstart,const Config& qgoal)
{
  Cleanup();
  start = AddMilestone(qstart);
  goal = AddMilestone(qgoal);
  Assert(start == 0 && goal == 1);
  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  spp.InitializeSource(start);  
  if(useSppLB) 
    sppLB.InitializeSource(start);
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));
  if(useSppGoal) {
    sppGoal.InitializeSource(goal);
    if(useSppLB) 
      sppLBGoal.InitializeSource(goal);
  }

  numPlanSteps = 0;
  numEdgeChecks = 0;
  numEdgePrechecks = 0;
  tCheck=tKnn=tConnect=tLazy=tLazyCheck=tShortestPaths=0;
}
void PRMStarPlanner::PlanMore()
{
  numPlanSteps ++;
  Real optCounter = Real(numPlanSteps)+1.0;
  Vector x;

  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));

  Real goalDist = spp.d[goal];
  Real fudgeFactor = 1.0+suboptimalityFactor;

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
    m=AddMilestone(x);  
  }
  else {
    //RRT/RRG expansion strategy
    GenerateConfig(x);
#if ELLIPSOID_PRUNING
    if((space->Distance(roadmap.nodes[start],x)+space->Distance(x,roadmap.nodes[goal]))*fudgeFactor >= goalDist) {
      //printf("Ellipsoid pruned, distance %g\n",space->Distance(roadmap.nodes[start],x)+space->Distance(x,roadmap.nodes[goal]));
      return;
    }
#endif
    //KNN call has a bit of duplication with connectByRadius, below
    int nn;
    vector<int> neighbors;
    if(bidirectional) {
      KNN(x,1,neighbors);
      nn = neighbors[0];
      tKnn += timer.ElapsedTime();
    }
    else {
      Real dGoal = space->Distance(x,roadmap.nodes[goal]);
      gCurrentOptimalMotionPlanner = this;
      gCurrentDGoal = dGoal;
      Real d;
      bool res=pointLocator->FilteredNN(x,connectedToStartFilter,nn,d);
      if(!res) {
	//this is a really inefficient way of getting the closest neighbor
	//connected to the start
	bool good = false;
	for(int i=1;i<22;i+=10) {
	  KNN(x,i,neighbors);
	  //only connect to nodes attached to start
	  for(size_t j=0;j<neighbors.size();j++) {
	    nn = neighbors[j];
	    Real dnn = (useSppLB ? sppLB.d[nn] : spp.d[nn]);
	    //with lazy, there may be lots of good neighbors that are just poorly connected
	    if(!lazy && HEURISTIC_PRUNING) {
	      if((dnn+dGoal)*fudgeFactor < goalDist) {
		good = true;
		break;
	      }
	    }
	    else {
	      if(!IsInf(dnn)) {
		good = true;
		break;
	      }
	    }
	  }
	  if(good) break;
	}
	tKnn += timer.ElapsedTime();
	if(!good) return;
      }
    } //not bidirectional
    timer.Reset();
    Real rad = connectRadiusConstant*Pow(Log(optCounter)/optCounter,1.0/x.n);
    if(rad > connectionThreshold) rad = connectionThreshold;
    Real d = space->Distance(roadmap.nodes[nn],x);
    if(d > rad) {
      Config xnew;
      space->Interpolate(roadmap.nodes[nn],x,rad/d,xnew);
      x = xnew;
      d = rad;
    }
    if(!space->IsFeasible(x)) {
      tCheck += timer.ElapsedTime();
      return;
    }
    tCheck += timer.ElapsedTime();
    timer.Reset();

    //connect to the closest node
    SmartPointer<EdgePlanner> e = space->LocalPlanner(x,roadmap.nodes[nn]);
    bool efeasible = false;
    if(!lazy || d > lazyCheckThreshold) {
      numEdgeChecks++;
      if(!e->IsVisible()) {
	tConnect += timer.ElapsedTime();
	return;
      }
      efeasible = true;
    }

    m=AddMilestone(x);
    if(efeasible)
      ConnectEdge(m,nn,e);
    else
      ConnectEdgeLazy(m,nn,e);
    tConnect += timer.ElapsedTime();
  }

  //determine near neighbors for connection
  timer.Reset();
  vector<int> neighbors;
  if(connectByRadius) {
    Real rad = connectRadiusConstant*Pow(Log(optCounter)/optCounter,1.0/x.n);
    //do we want to halve this for the PRM connection strategy?
    if(rad > connectionThreshold) rad = connectionThreshold;
    Neighbors(x,rad,neighbors);
  }
  else {
    int kmax = int(((1.0+1.0/x.n)*E)*Log(Real(roadmap.nodes.size())));
    //do we want to halve this for the PRM connection strategy?
    if(!rrg) kmax /= 2;
    if(kmax <= 0) kmax = 1;
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
      if(n == m) continue;
      if(useSppLB) {
	Assert(m < (int)LBroadmap.nodes.size());
	Assert(n < (int)LBroadmap.nodes.size());
	if(m >= 0 && LBroadmap.HasEdge(m,n)) continue;
      }
      else {
	if(m >= 0 && roadmap.HasEdge(m,n)) continue;
      }
      Real d = space->Distance(x,roadmap.nodes[n]);
      if(d > connectionThreshold) continue;
      if(useSppLB) 
	queue.push_back(pair<Real,int>(sppLB.d[n]+d,n));
      else
	queue.push_back(pair<Real,int>(spp.d[n]+d,n));
    }
    sort(queue.begin(),queue.end());
    neighbors.resize(queue.size());
    for(size_t i=0;i<queue.size();i++)
      neighbors[i] = queue[i].second;
  }

  //start connecting to neighbors and doing necessary rewiring
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

    //prune out distant neighbors
    if(m==-1) {
      m=AddMilestone(x);  
    }

    SmartPointer<EdgePlanner> e;
    //don't connect points that aren't potentially optimal
    bool doConnect = !rrg;
    if(rrg) {
      //TODO: for lazy, this doesnt necessarily work properly with nonzero suboptimality
      //factor.  Need to prove that it works!
      Real cn_lb = (useSppLB ? sppLB.d[n] : spp.d[n]);
      Real cm_lb = (useSppLB ? sppLB.d[m] : spp.d[m]);
      Real dGoal = space->Distance(x,roadmap.nodes[goal]);
      if((Min(cn_lb + d,cm_lb) + dGoal)*fudgeFactor < goalDist) {
	if((cn_lb + d) < cm_lb)
	  doConnect = true;
	else if((cm_lb + d) < cn_lb)
	  doConnect = true;
      }
      if(bidirectional && !doConnect) { //allow backwards connections
	Real cn_lb = (useSppLB ? sppLBGoal.d[n] : sppGoal.d[n]);
	Real cm_lb = (useSppLB ? sppLBGoal.d[m] : sppGoal.d[m]);
	Real dStart = space->Distance(x,roadmap.nodes[start]);
	if((Min(cn_lb + d,cm_lb) + dStart)*fudgeFactor < goalDist) {
	  if((cn_lb + d) < cm_lb)
	    doConnect = true;
	  else if((cm_lb + d) < cn_lb)
	    doConnect = true;
	}
      }
    }
    if(doConnect) {
      const Config& xn = roadmap.nodes[n];
      e = space->LocalPlanner(x,xn);
      Assert(e->Space() != NULL);

      //non-lazy and no suboptimality: check all edges
      //non-lazy and suboptimality: check edges whose current true cost is 
      //  greater than the potential cost
      bool doCheck = !lazy;
      if(!lazy && suboptimalityFactor > 0) {
	doCheck = false;
	Real cn = spp.d[n];
	Real cm = spp.d[m];
	Real cn_lb = (useSppLB ? sppLB.d[n] : spp.d[n]);
	Real cm_lb = (useSppLB ? sppLB.d[m] : spp.d[m]);
	//printf ("%g %g %g %g\n",cn,cm,cn_lb,cm_lb);
	if((cn_lb + d)*fudgeFactor < cm)
	  doCheck = true;
	else if((cm_lb + d)*fudgeFactor < cn)
	  doCheck = true;
	if(bidirectional && !doCheck) {
	  Real cn = sppGoal.d[n];
	  Real cm = sppGoal.d[m];
	  Real cn_lb = (useSppLB ? sppLBGoal.d[n] : sppGoal.d[n]);
	  Real cm_lb = (useSppLB ? sppLBGoal.d[m] : sppGoal.d[m]);
	  if((cn_lb + d)*fudgeFactor < cm)
	    doCheck = true;
	  else if((cm_lb + d)*fudgeFactor < cn)
	    doCheck = true;
	}
      }
      else if(lazy) {
#if PRECHECK_OPTIMAL_EDGES && ! TEST_FMT
	//if this edge will become part of an optimal lazy path -- check it now
	//rather than waiting for the lazy update
	if((sppLB.d[n] + d + sppLBGoal.d[m])*fudgeFactor < goalDist || (sppLB.d[m] + d + sppLBGoal.d[n])*fudgeFactor < goalDist) {
	  //check edge leading to m, it's on the optimal path too
	  if(!IsInf(sppLB.d[m])) {
	    int p = sppLB.p[m];
	    if(!roadmap.HasEdge(p,m)) {
	      //precheck edge to m
	      numEdgeChecks++;
	      numEdgePrechecks ++;
	      SmartPointer<EdgePlanner>* e = LBroadmap.FindEdge(p,m);
	      if(!(*e)->IsVisible()) {
		LBroadmap.DeleteEdge(p,m);
		timer.Reset();
		EdgeDistance distanceWeightFunc;
		sppLB.DeleteUpdate_Undirected(p,m,distanceWeightFunc);
		if(useSppGoal)
		  sppLBGoal.DeleteUpdate_Undirected(p,m,distanceWeightFunc);
		tShortestPaths += timer.ElapsedTime();
	      }
	    }
	  }
	  //check again, the path to m may have been deleted
	  if((sppLB.d[n] + d + sppLBGoal.d[m])*fudgeFactor < goalDist || (sppLB.d[m] + d + sppLBGoal.d[n])*fudgeFactor < goalDist) {
	    doCheck=true;
	    numEdgePrechecks ++;
	  }
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
	  ConnectEdge(m,n,e);
	}
      }
      else {
	//add it now without checking, lazy style
	ConnectEdgeLazy(m,n,e);
      }
    }
    /*else {
      if(n==goal) printf("Candidate neighbor to goal rejected due to cost: %g+%g > %g (fudge factor %g)\n",spp.d[m],d,spp.d[n],fudgeFactor);
    }
    */
  }
  tConnect += timer.ElapsedTime();
  if(lazy) {    
    if(sppLB.d[goal]*fudgeFactor < goalDist) {
      //found an improved path to the goal! do checking
      timer.Reset();
      //printf("Candidate improvement to cost %g\n",spp.d[goal]);
      if(CheckPath(start,goal)) {
	//printf("Found an improved path to goal with cost %g (before it was %g)\n",spp.d[goal],goalDist);
      }
      //else
	//printf("Failed CheckPath, going back to old distance, d=%g\n",spp.d[goal]);
      tLazy += timer.ElapsedTime();
    }
  }
}

int PRMStarPlanner::AddMilestone(const Config& x)
{
  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));

  int m=RoadmapPlanner::AddMilestone(x);
  if(useSppLB) {
    int mlb = LBroadmap.AddNode(x);
    Assert(mlb == m);
  }
  Assert(m==(int)spp.p.size());
  spp.p.push_back(-1);
  spp.d.push_back(Inf);
  if(useSppLB) {
    sppLB.p.push_back(-1);
    sppLB.d.push_back(Inf);
  }
  if(useSppGoal) {
    sppGoal.p.push_back(-1);
    sppGoal.d.push_back(Inf);
    if(useSppLB) {
      sppLBGoal.p.push_back(-1);
      sppLBGoal.d.push_back(Inf);
    }
  }
  return m;
}

void PRMStarPlanner::ConnectEdge(int i,int j,const SmartPointer<EdgePlanner>& e)
{
  bool useSpp = (rrg || lazy);
  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));

  roadmap.AddEdge(i,j,e);
  Timer timer;
  EdgeDistance distanceWeightFunc;
  if(useSpp) 
    spp.DecreaseUpdate_Undirected(i,j,distanceWeightFunc);
  if(useSppGoal) 
    sppGoal.DecreaseUpdate_Undirected(i,j,distanceWeightFunc);
  if(useSppLB) {
    LBroadmap.AddEdge(i,j,e);
    sppLB.DecreaseUpdate_Undirected(i,j,distanceWeightFunc);
    if(useSppGoal) {
      sppLBGoal.DecreaseUpdate_Undirected(i,j,distanceWeightFunc);
    }
  }
  tShortestPaths += timer.ElapsedTime();
}

void PRMStarPlanner::ConnectEdgeLazy(int i,int j,const SmartPointer<EdgePlanner>& e)
{
  Assert(i >= 0 && j >= 0 && i < (int)roadmap.nodes.size() && j < (int)roadmap.nodes.size());
  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));

  Assert(useSppLB);
  LBroadmap.AddEdge(i,j,e);

  Timer timer;
  EdgeDistance distanceWeightFunc;
  sppLB.DecreaseUpdate_Undirected(i,j,distanceWeightFunc);
  if(useSppGoal) 
    sppLBGoal.DecreaseUpdate_Undirected(i,j,distanceWeightFunc);
  tShortestPaths += timer.ElapsedTime();
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
    int num = int(Log(Real(numPlanSteps+1)));
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
    spptemp.InitializeSource(start);
    spptemp.FindPath_Undirected(goal,distanceWeightFunc);
    if(IsInf(spptemp.d[goal])) return false;
    return true;
  }
  if(IsInf(spp.d[goal])) return false;
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
    //debugging:
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

struct RoadmapEdgeInfo
{
  Real Priority() const { return e->Priority(); }
  //Real Priority() const { return e->Priority()/(1.0+distanceFromEndpoint); }
  //Real Priority() const { return e->Priority()*(1.0+distanceFromEndpoint); }
  //Real Priority() const { return Exp(-e->Priority()*distanceFromEndpoint); }
  //Real Priority() const { return -distanceFromEndpoint; }
  int s,t;
  Real distanceFromEndpoint;
  SmartPointer<EdgePlanner> e;
};

struct LessEdgePriority
{
  typedef RoadmapEdgeInfo EdgeInfo;
  bool operator() (EdgeInfo& a,EdgeInfo& b) const
  {
    return a.Priority() < b.Priority();
  }
};

class FMTAStar : public AI::GeneralizedAStar<int,double>
{
public:
  PRMStarPlanner* planner;
  FMTAStar(PRMStarPlanner* _planner):planner(_planner)
  {
    visited.resize(planner->roadmap.nodes.size(),NULL);
  }
  virtual bool IsGoal(const int& s) { return s == planner->goal; }
  virtual void Successors(const int& s,vector<int>& successors,vector<double>& cost) {
    successors.resize(0);
    cost.resize(0);
    Graph::UndirectedEdgeIterator<SmartPointer<EdgePlanner> > e;
    vector<pair<int,int> > todelete;
    for(planner->LBroadmap.Begin(s,e);!e.end();e++) {
      int t = e.target();
      double c = planner->space->Distance((*e)->Start(),(*e)->Goal());
      if(planner->roadmap.HasEdge(s,t)) {
	successors.push_back(t);
	cost.push_back(c);
      }
      else {
	Node* nt = VisitedStateNode(t);
	if(nt && nt->g <= VisitedStateNode(s)->g+c) 
	  //don't check edges along suboptimal paths
	  continue;
	if((*e)->IsVisible()) {
	  planner->roadmap.AddEdge(s,t,(*e));
	  successors.push_back(t);
	  cost.push_back(c);
	}
	else {
	  todelete.push_back(pair<int,int>(s,t));
	}
      }
    }
    for(size_t i=0;i<todelete.size();i++)
      planner->LBroadmap.DeleteEdge(todelete[i].first,todelete[i].second);
  }
  virtual double Heuristic(const int& s) { return planner->sppLBGoal.d[s]; }
  virtual void ClearVisited() { fill(visited.begin(),visited.end(),(Node*)NULL); }
  virtual void Visit(const int& s,Node* n) { visited[s] = n; }
  virtual Node* VisitedStateNode(const int& s) { return visited[s]; }
  vector<Node*> visited;
};

bool PRMStarPlanner::CheckPath(int a,int b)
{
  EdgeDistance distanceWeightFunc;
  Assert(lazy);

  if(TEST_FMT) {
    Timer timer;
    Assert(PRECHECK_OPTIMAL_EDGES);
    FMTAStar astar(this);
    astar.SetStart(start);
    bool res = astar.Search();
    tLazyCheck += timer.ElapsedTime();

    //update shortest paths
    timer.Reset();
    spp.FindPath_Undirected(goal,distanceWeightFunc);
    sppLB.FindPath_Undirected(goal,distanceWeightFunc);
    sppGoal.FindPath_Undirected(start,distanceWeightFunc);
    sppLBGoal.FindPath_Undirected(start,distanceWeightFunc);
    tShortestPaths += timer.ElapsedTime();

    return res;
  }

#if PRECHECK_OPTIMAL_EDGES
  bool useSppGoal = true;
#else
  bool useSppGoal = false;
#endif
  Assert(a==start);
  //printf("Done with planning step\n");
  while(!IsInf(sppLB.d[b])) {
    //printf("Lazy collision checking, existing cost %g, potential %g\n",goalDist,spp.d[b]);
    vector<int> npath;
    if(!Graph::GetAncestorPath(sppLB.p,goal,start,npath)) {
      //shouldn't happen: may have disconnected goal?
      return false;
    }
    Assert(npath[0] == a);
    Timer timer;
    priority_queue<RoadmapEdgeInfo,vector<RoadmapEdgeInfo>,LessEdgePriority> q;
    for(size_t i=0;i+1<npath.size();i++) {
      if(roadmap.HasEdge(npath[i],npath[i+1])) continue;
      RoadmapEdgeInfo e;
      e.s = npath[i];
      e.t = npath[i+1];
      e.distanceFromEndpoint = (int)i;
      if(bidirectional && ((int)npath.size()-(int)i-1 < e.distanceFromEndpoint))
	e.distanceFromEndpoint = (int)npath.size()-(int)i-1;
      e.e = *LBroadmap.FindEdge(npath[i],npath[i+1]);
      q.push(e);
    }
    //adaptive division of path
    bool feas = true;
    while(!q.empty()) {
      RoadmapEdgeInfo temp=q.top(); q.pop();
      if(!temp.e->Done()) 
	temp.e->Plan(); 
      if(!temp.e->Done()) {
	q.push(temp);
	continue;
      }
      //it's done planning
      numEdgeChecks++;
      if(temp.e->Failed()) {
	tLazyCheck += timer.ElapsedTime();
	//delete edge from lazy roadmap
	//printf("Deleting edge %d %d...\n",npath[i],npath[i+1]);
	LBroadmap.DeleteEdge(temp.s,temp.t);

	//update shortest paths
	timer.Reset();
	sppLB.DeleteUpdate_Undirected(temp.s,temp.t,distanceWeightFunc);
	if(useSppGoal)
	  sppLBGoal.DeleteUpdate_Undirected(temp.s,temp.t,distanceWeightFunc);
	tShortestPaths += timer.ElapsedTime();
	//Assert(sppLB.HasShortestPaths_Undirected(0,distanceWeightFunc));
	feas = false;
      }
      else {
	roadmap.AddEdge(temp.s,temp.t,temp.e);
	timer.Reset();
	spp.DecreaseUpdate_Undirected(temp.s,temp.t,distanceWeightFunc);
	if(useSppGoal)
	  sppGoal.DecreaseUpdate_Undirected(temp.s,temp.t,distanceWeightFunc);
	tShortestPaths += timer.ElapsedTime();
      }
    }
    
    if(feas) {
      /*
	//SANITY CHECK
      MilestonePath path;
      bool res = GetPath(path);
      Assert(res == true);
      Assert(path.IsFeasible());
      */
      tLazyCheck += timer.ElapsedTime();
      return true;
    }
  }
  return false;
}
