#include <KrisLibrary/Logger.h>
#include "OptimalMotionPlanner.h"
#include "PointLocation.h"
#include "GeneralizedAStar.h"
#include "Objective.h"
#include <math/random.h>
#include <graph/Path.h>
#include <Timer.h>

//if this is on, this will check any optimal edges as they are added
#define PRECHECK_OPTIMAL_EDGES 1
//Do not do heuristic pruning with bidirectional (this doesn't seem to help much with lazy)
#define HEURISTIC_PRUNING 0
//Prunes new configurations whose straight line distance to start and goal
//is greater than the current goal cost
#define ELLIPSOID_PRUNING 1

//TEST: An incremental version of FMT*
#define TEST_FMT 0

//TEST: discount edges that are already collision checked
#define TEST_EDGE_DISCOUNTING 0
//TEST: when this is lower, the planner tries to reuse checked edges
#define EDGE_DISCOUNT_FACTOR 0.01

//regular distance function used in shortest paths update
#define DISTANCE_FUNC EdgeDistance()

//if 1, the path checking is done adaptively so that edges are placed in a priority queue
//and bisected until an infeasible solution is found.  If 0, the path checking is done by
//walking along the path and calling IsVisible() along each unchecked edge
#define ADAPTIVE_SUBDIVISION 0
#define ADAPTIVE_SUBDIVISION_CHECK_COUNT 10

#if TEST_EDGE_DISCOUNTING
  #if TEST_FMT
  //undefined results when using discounting
  #define LB_DISTANCE_FUNC EdgeDistance()
  #else
  #define LB_DISTANCE_FUNC DiscountedEdgeDistance(EDGE_DISCOUNT_FACTOR)
  #endif //TEST_FMT 
#else
  #define LB_DISTANCE_FUNC EdgeDistance()
#endif //TEST_EDGE_DISCOUNTING

class EdgeDistance
{
 public:
  Real operator () (const EdgePlannerPtr& e,int s,int t)
  {
    assert(e->Space() != NULL);
    Real res = e->Space()->Distance(e->Start(),e->End());
    if(res <= 0) {
      LOG4CXX_WARN(KrisLibrary::logger(),"PRMStarPlanner: Warning, edge has nonpositive length "<<res);
      return Epsilon;
    }
    return res;
  }
};

class EdgeObjectiveCost
{
 public:
  ObjectiveFunctionalBase* objective;
  int terminalNode;  //the indicator for the special terminal node

  EdgeObjectiveCost(ObjectiveFunctionalBase* _objective,int _terminalNode=-1)
    :objective(_objective),terminalNode(_terminalNode)
  {}
  Real operator () (const EdgePlannerPtr& e,int s,int t)
  {
    if(!e) return 1.0;  //this must be a lazy planner
    if(t==terminalNode)
      return objective->TerminalCost(e->Start());
    return objective->IncrementalCost(e.get());
  }
};

class DiscountedEdgeDistance
{
 public:
  Real factor;
  DiscountedEdgeDistance(Real _factor):factor(_factor) {}
  Real operator () (const EdgePlannerPtr& e,int s,int t)
  {
    assert(e->Space() != NULL);
    Real res = e->Space()->Distance(e->Start(),e->End());
    if(e->Done()) {
      Assert(!e->Failed());
      res *= factor;
    }
    if(res <= 0) {
      LOG4CXX_WARN(KrisLibrary::logger(),"PRMStarPlanner: Warning, edge has nonpositive length "<<res);
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
  :RoadmapPlanner(space),lazy(false),rrg(false),bidirectional(true),connectByRadius(false),connectRadiusConstant(1),connectNeighborsConstant(1.1),connectionThreshold(Inf),lazyCheckThreshold(Inf),suboptimalityFactor(0),spp(roadmap),sppGoal(roadmap),sppLB(LBroadmap),sppLBGoal(LBroadmap)
{
  start = goal = -1;
}

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
void PRMStarPlanner::SetMaxCost(Real cmax)
{
  if(goal < 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"PRMStarPlanner::SetMaxCost(): Init() must be called first");
    return;
  }
  spp.d[goal] = Min(spp.d[goal],cmax);
}
void PRMStarPlanner::PlanMore()
{
  if(start < 0 || goal < 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"PRMStarPlanner::PlanMore(): Init() must be called before planning");
    return;
  }
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
      //LOG4CXX_INFO(KrisLibrary::logger(),"Ellipsoid pruned, distance "<<space->Distance(roadmap.nodes[start],x)+space->Distance(x,roadmap.nodes[goal]))
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
    EdgePlannerPtr e = space->LocalPlanner(x,roadmap.nodes[nn]);
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
    int kmax = int(connectNeighborsConstant*((1.0+1.0/x.n)*E)*Log(Real(roadmap.nodes.size())));
    //do we want to halve this for the PRM connection strategy?
    //if(!rrg) kmax /= 2;
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

    EdgePlannerPtr e;
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
        //LOG4CXX_INFO(KrisLibrary::logger()  ,""<<cn<<" "<<cm<<" "<<cn_lb<<" "<<cm_lb);
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
              EdgePlannerPtr* e = LBroadmap.FindEdge(p,m);
              if(!(*e)->IsVisible()) {
                LBroadmap.DeleteEdge(p,m);
                timer.Reset();
                sppLB.DeleteUpdate_Undirected(p,m,LB_DISTANCE_FUNC);
                if(useSppGoal)
                  sppLBGoal.DeleteUpdate_Undirected(p,m,LB_DISTANCE_FUNC);
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
      if(n==goal) LOG4CXX_INFO(KrisLibrary::logger(),"Candidate neighbor to goal rejected due to cost: "<<spp.d[m]<<"+"<<d<<" > "<<spp.d[n]<<" (fudge factor "<<fudgeFactor);
    }
    */
  }
  tConnect += timer.ElapsedTime();
  if(lazy) {    
    if(sppLB.d[goal]*fudgeFactor < goalDist) {
      //found an improved path to the goal! do checking
      timer.Reset();
      //LOG4CXX_INFO(KrisLibrary::logger(),"Candidate improvement to cost "<<spp.d[goal]);
      if(CheckPath(start,goal)) {
        //LOG4CXX_INFO(KrisLibrary::logger(),"Found an improved path to goal with cost "<<spp.d[goal]<<" (before it was "<<goalDist);
      }
      //else
        //LOG4CXX_INFO(KrisLibrary::logger(),"Failed CheckPath, going back to old distance, d="<<spp.d[goal]);
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

void PRMStarPlanner::ConnectEdge(int i,int j,const EdgePlannerPtr& e)
{
  bool useSpp = (rrg || lazy);
  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));

  roadmap.AddEdge(i,j,e);
  Timer timer;
  if(useSpp) 
    spp.DecreaseUpdate_Undirected(i,j,DISTANCE_FUNC);
  if(useSppGoal) 
    sppGoal.DecreaseUpdate_Undirected(i,j,DISTANCE_FUNC);
  if(useSppLB) {
    LBroadmap.AddEdge(i,j,e);
    sppLB.DecreaseUpdate_Undirected(i,j,LB_DISTANCE_FUNC);
    if(useSppGoal) {
      sppLBGoal.DecreaseUpdate_Undirected(i,j,LB_DISTANCE_FUNC);
    }
  }
  tShortestPaths += timer.ElapsedTime();
}

void PRMStarPlanner::ConnectEdgeLazy(int i,int j,const EdgePlannerPtr& e)
{
  Assert(i >= 0 && j >= 0 && i < (int)roadmap.nodes.size() && j < (int)roadmap.nodes.size());
  bool useSppLB = (lazy || (rrg && suboptimalityFactor > 0));
  bool useSppGoal = (bidirectional || (lazy && PRECHECK_OPTIMAL_EDGES));

  Assert(useSppLB);
  LBroadmap.AddEdge(i,j,e);

  Timer timer;
  sppLB.DecreaseUpdate_Undirected(i,j,LB_DISTANCE_FUNC);
  if(useSppGoal) 
    sppLBGoal.DecreaseUpdate_Undirected(i,j,LB_DISTANCE_FUNC);
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
    ShortestPathProblem spptemp(roadmap);
    spptemp.InitializeSource(start);
    spptemp.FindPath_Undirected(goal,DISTANCE_FUNC);
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
    spp.InitializeSource(a);
    spp.FindPath_Undirected(b,DISTANCE_FUNC);
  }
  else {
    Assert(a==start);
    //Assert(spp.HasShortestPaths_Undirected(a,DISTANCE_FUNC));
  }
  if(IsInf(spp.d[b])) return false;
  if(!Graph::GetAncestorPath(spp.p,b,a,nodes)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"PRMStarPlanner: Unable to find path from "<<a<<" to "<<b);
    //debugging:
    LOG4CXX_INFO(KrisLibrary::logger(),"node,distance,parent,edge weight,edge feasible");
    for(int i=0;i<20;i++) {
      Real w = 0;
      int feas = 0;
      if(spp.p[b] >= 0) {
        EdgePlannerPtr* e=roadmap.FindEdge(b,spp.p[b]);
        Assert(e != NULL);
        w=(*e)->Space()->Distance((*e)->Start(),(*e)->End());
        feas = (*e)->IsVisible();
      }
      LOG4CXX_INFO(KrisLibrary::logger(),""<<b<<","<<spp.d[b]<<","<<spp.p[b]<<","<<w<<","<<feas);
      b = spp.p[b];
      if(b < 0) break;
    }
    return false;
  }
  for(size_t i=0;i+1<nodes.size();i++) 
    Assert(spp.d[nodes[i+1]] > spp.d[nodes[i]]);
  path.edges.resize(0);
  for(size_t i=0;i+1<nodes.size();i++) {
    EdgePlannerPtr* e=roadmap.FindEdge(nodes[i],nodes[i+1]);
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

Real PRMStarPlanner::OptimizePath(ObjectiveFunctionalBase* cost,MilestonePath& path)
{
  vector<int> goals(1,goal);
  return OptimizePath(start,goals,cost,path);
}
  
Real PRMStarPlanner::OptimizePath(int a,const vector<int>& goals,ObjectiveFunctionalBase* cost,MilestonePath& path)
{
  ShortestPathProblem spptemp(roadmap);
  spptemp.InitializeSource(a);
  set<int> goalset;
  goalset.insert(goals.begin(),goals.end());
  EdgeObjectiveCost costfunc(cost);
  spptemp.FindAPath_Undirected(goalset,costfunc);
  Real dmin = Inf;
  int optgoal = -1;
  for(auto g: goals) {
    if(spptemp.d[g] < dmin) {
      dmin = spptemp.d[g];
      optgoal = g;
    }
  }
  path.edges.clear();
  if(optgoal < 0) return Inf;

  vector<int> nodes;
  if(!Graph::GetAncestorPath(spptemp.p,optgoal,a,nodes)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"PRMStarPlanner: Unable to find path from "<<a<<" to "<<optgoal);
    return Inf;
  }
  path.edges.resize(0);
  for(size_t i=0;i+1<nodes.size();i++) {
    EdgePlannerPtr* e=roadmap.FindEdge(nodes[i],nodes[i+1]);
    Assert(e != NULL);
    if((*e)->Start() == roadmap.nodes[nodes[i]])
      path.edges.push_back(*e);
    else {
      Assert((*e)->Start() == roadmap.nodes[nodes[i+1]]);
      path.edges.push_back((*e)->ReverseCopy());
    }
  }
  return dmin;
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
  EdgePlannerPtr e;
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
    Graph::UndirectedEdgeIterator<EdgePlannerPtr > e;
    vector<pair<int,int> > todelete;
    for(planner->LBroadmap.Begin(s,e);!e.end();e++) {
      int t = e.target();
      double c = planner->space->Distance((*e)->Start(),(*e)->End());
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
    spp.FindPath_Undirected(goal,DISTANCE_FUNC);
    sppLB.FindPath_Undirected(goal,LB_DISTANCE_FUNC);
    sppGoal.FindPath_Undirected(start,DISTANCE_FUNC);
    sppLBGoal.FindPath_Undirected(start,LB_DISTANCE_FUNC);
    tShortestPaths += timer.ElapsedTime();

    return res;
  }

#if PRECHECK_OPTIMAL_EDGES
  bool useSppGoal = true;
#else
  bool useSppGoal = false;
#endif
  Assert(a==start);
  //LOG4CXX_INFO(KrisLibrary::logger(),"Done with planning step");
  while(!IsInf(sppLB.d[b])) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Lazy collision checking, existing cost "<<goalDist<<", potential "<<spp.d[b]);
    vector<int> npath;
    if(!Graph::GetAncestorPath(sppLB.p,goal,start,npath)) {
      //shouldn't happen: may have disconnected goal?
      return false;
    }
    Assert(npath[0] == a);
    Timer timer;
    bool feas = true;
#if ADAPTIVE_SUBDIVISION
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
    while(!q.empty()) {
      RoadmapEdgeInfo temp=q.top(); q.pop();
      int cnt = ADAPTIVE_SUBDIVISION_CHECK_COUNT;
      if(!temp.e->Done() && cnt-- > 0) 
        temp.e->Plan(); 
      if(!temp.e->Done()) {
        q.push(temp);
        continue;
      }
      bool edgeInfeasible = temp.e->Failed();
#else
    //non-adaptive subdivision -- just march along path
    for(size_t i=0;i+1<npath.size();i++) {
      if(roadmap.HasEdge(npath[i],npath[i+1])) continue;
      RoadmapEdgeInfo temp;
      temp.s = npath[i];
      temp.t = npath[i+1];
      temp.e = *LBroadmap.FindEdge(npath[i],npath[i+1]);
      bool edgeInfeasible = !temp.e->IsVisible();
#endif //ADAPTIVE_SUBDIVISION

      //it's done planning
      numEdgeChecks++;
      if(edgeInfeasible) {
        tLazyCheck += timer.ElapsedTime();
        //delete edge from lazy roadmap
        //LOG4CXX_INFO(KrisLibrary::logger(),"Deleting edge "<<npath[i]<<" "<<npath[i+1]);
        LBroadmap.DeleteEdge(temp.s,temp.t);

        //update shortest paths
        timer.Reset();
        sppLB.DeleteUpdate_Undirected(temp.s,temp.t,LB_DISTANCE_FUNC);
        if(useSppGoal)
          sppLBGoal.DeleteUpdate_Undirected(temp.s,temp.t,LB_DISTANCE_FUNC);
        tShortestPaths += timer.ElapsedTime();
        //Assert(sppLB.HasShortestPaths_Undirected(0,distanceWeightFunc));
        feas = false;
      }
      else {
        roadmap.AddEdge(temp.s,temp.t,temp.e);
        timer.Reset();
        spp.DecreaseUpdate_Undirected(temp.s,temp.t,DISTANCE_FUNC);
        if(useSppGoal)
          sppGoal.DecreaseUpdate_Undirected(temp.s,temp.t,DISTANCE_FUNC);

        if(TEST_EDGE_DISCOUNTING) {
          //if discounted, revise LB roadmap: the cost got smaller!
          sppLB.DecreaseUpdate_Undirected(temp.s,temp.t,LB_DISTANCE_FUNC);
          if(useSppGoal)
            sppLBGoal.DecreaseUpdate_Undirected(temp.s,temp.t,LB_DISTANCE_FUNC);
        }
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
