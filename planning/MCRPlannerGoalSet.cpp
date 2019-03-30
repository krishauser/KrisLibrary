#include <KrisLibrary/Logger.h>
#include "MCRPlannerGoalSet.h"
#include "GeneralizedAStar.h"
#include <math/random.h>
#include <structs/FixedSizeHeap.h>
#include <structs/Heap.h>
#include <graph/Path.h>
#include <optimization/NonlinearProgram.h>
#include <Timer.h>
#include <algorithm>
using namespace AI;
using namespace std;

const static int gMaxExtendTowardIters = 1;
const static double gSubdivideThreshold = 1e-3;
//on some platforms, timing takes a non-negligible amount of time
#define DO_TIMING 1


//defined in MCRPlanner.cpp
Subset Violations(CSpace* space,const Config& q);
Subset Violations(CSpace* space,const Config& a,const Config& b);


inline Real WeightedCost(const Subset& s,const vector<Real>& weights)
{
  Real sum=0.0;
  //for(set<int>::const_iterator i=s.items.begin();i!=s.items.end();i++) {
  for(vector<int>::const_iterator i=s.items.begin();i!=s.items.end();i++) {
    sum += weights[*i];
    if(IsInf(sum)) return sum;
  }
  return sum;
}

bool WithinThreshold(const MCRPlannerGoalSet::Mode& mode,Real maxExplanationCost)
{
  return mode.minCost <= maxExplanationCost;
}

//checks whether the mode's cover + the extra cover exceed the given cost
bool WithinThreshold(const MCRPlannerGoalSet::Mode& mode,const Subset& extra,Real maxExplanationCost,const vector<Real>& weights)
{
  if(mode.minCost > maxExplanationCost) return false;
  for(size_t i=0;i<mode.pathCovers.size();i++) {
    if(WeightedCost((mode.pathCovers[i]+extra),weights) <= maxExplanationCost) return true;
  }
  return false;
}

Real MaxError(Optimization::NonlinearProgram* p,const Config& x)
{
  Real maxError = 0;
  if(p->c) {
    Vector v(p->c->NumDimensions());
    (*p->c)(x,v);
    maxError = v.maxAbsElement();
  }
  if(p->d) {
    Vector v(p->d->NumDimensions());
    (*p->d)(x,v);
    if(p->inequalityLess)
      maxError = Max(maxError,v.maxElement());
    else 
      maxError = Max(maxError,-v.minElement());
  }
  return maxError;
}


MCRPlannerGoalSet::MCRPlannerGoalSet(CSpace* _space)
  :space(_space),goalSet(NULL),goalNumeric(NULL),
   updatePathsComplete(false),updatePathsDynamic(true),updatePathsMax(INT_MAX),
   numExpands(0),numRefinementAttempts(0),numRefinementSuccesses(0),numExplorationAttempts(0),
   numEdgeChecks(0),numConfigChecks(0),
   numUpdatePaths(0),numUpdatePathsIterations(0),
   timeNearestNeighbors(0),timeRefine(0),timeExplore(0),timeUpdatePaths(0),timeOverhead(0)
{
  numConnections=10;
  //numConnections=-1;
  connectThreshold=Inf;
  expandDistance = 0.1;
  goalConnectThreshold = 0.5;
  goalBiasProbability = 0.0;
  bidirectional = false;
}

Real MCRPlannerGoalSet::Cost(const Subset& s) const
{
  if(obstacleWeights.empty()) return Real(s.items.size());
  return WeightedCost(s,obstacleWeights);
}

void MCRPlannerGoalSet::Init(const Config& _start,CSet* _goal)
{
  roadmap.Cleanup();
  modeGraph.Cleanup();
  numExpands=0;
  numExplorationAttempts=0;
  numRefinementAttempts=0;
  numRefinementSuccesses=0;
  numEdgeChecks=0;
  numConfigChecks=0;
  numUpdatePaths=0;
  numUpdatePathsIterations=0;

  start=_start; 
  goalSet = _goal; 
  if(_goal) goalNumeric = _goal->Numeric();
  goalNodes.resize(0);
  AddNode(start);
  if(updatePathsComplete) UpdatePathsComplete();
  else UpdatePathsGreedy();
}

void MCRPlannerGoalSet::UpdateMinCost(Mode& m)
{
  if(m.pathCovers.empty())
    m.minCost = DBL_MAX;
  else {
    m.minCost = Cost(m.pathCovers[0]);
    for(size_t j=1;j<m.pathCovers.size();j++) {
      m.minCost = Min(m.minCost,Cost(m.pathCovers[j]));
    }
  }
}

void MCRPlannerGoalSet::UpdatePathsGreedy()
{
  numUpdatePaths++; 

  FixedSizeHeap<Real> q(modeGraph.nodes.size());

  //reset min covers
  for(size_t i=0;i<modeGraph.nodes.size();i++)
    modeGraph.nodes[i].minCost = DBL_MAX;

  int m0=roadmap.nodes[0].mode;
  set<int> mg;
  for(size_t i=0;i<goalNodes.size();i++)
    mg.insert(roadmap.nodes[goalNodes[i]].mode);
  Subset sgCover = modeGraph.nodes[m0].subset;
  modeGraph.nodes[m0].pathCovers.resize(1);
  modeGraph.nodes[m0].pathCovers[0] = sgCover;
  Real c0=Cost(sgCover);
  modeGraph.nodes[m0].minCost = c0;
  q.push(m0,-c0);

  while(!q.empty()) {
    numUpdatePathsIterations++;

    int m = q.top();
    q.pop();

    //reached goal
    if(mg.count(m) != 0) break;

    //ModeGraph::Iterator e;
    Graph::UndirectedEdgeIterator<Transition> e;
    for(modeGraph.Begin(m,e);!e.end();e++) {
      //compute propagated cost
      Mode& modet=modeGraph.nodes[e.target()];
      Subset s = modet.subset+modeGraph.nodes[m].pathCovers[0];
      Real cs=Cost(s);
      if(modet.minCost <= cs)
	continue;
      q.adjust(e.target(),-cs);

      //propagate the path subset
      modet.pathCovers.resize(1);
      modet.pathCovers[0] = s;
      modet.minCost = cs;
    }
  }
  /*

    LOG4CXX_INFO(KrisLibrary::logger(),"Mode "<<i<<": subset "<<modeGraph.nodes[i].subset<<", size "<<modeGraph.nodes[i].roadmapNodes.size()<<", cover: ");
    if(modeGraph.nodes[i].pathCovers.size() > 0)
      LOG4CXX_INFO(KrisLibrary::logger(),modeGraph.nodes[i].pathCovers[0]);
    else
      LOG4CXX_INFO(KrisLibrary::logger(),"(not reached)");
  }
  */
}

void MCRPlannerGoalSet::UpdatePathsGreedy2(int nstart)
{
  numUpdatePaths++; 

  Heap<int,Real> q;

  set<int> mg;
  for(size_t i=0;i<goalNodes.size();i++)
    mg.insert(roadmap.nodes[goalNodes[i]].mode);
  if(nstart <= 0) {
    int m0=roadmap.nodes[0].mode;
    Subset sgCover = modeGraph.nodes[m0].subset;
    modeGraph.nodes[m0].pathCovers.resize(1);
    modeGraph.nodes[m0].pathCovers[0] = sgCover;
    Real c0=Cost(sgCover);
    modeGraph.nodes[m0].minCost = c0;
    q.push(m0,-c0);
  }
  else {
    int m0=roadmap.nodes[nstart].mode;
    Mode& mode0 = modeGraph.nodes[m0];
    assert(!mode0.pathCovers.empty());

    //start looking for better paths into nstart
    ModeGraph::Iterator e;
    for(modeGraph.Begin(m0,e);!e.end();e++) {
      Mode& modet = modeGraph.nodes[e.target()];
      if(modet.minCost >= mode0.minCost) continue;
      Subset s = mode0.subset+modet.pathCovers[0];
      Real cs=Cost(s);
      if(cs < mode0.minCost) {
	mode0.pathCovers[0] = s;
	mode0.minCost = cs; 
      }
    }
    q.push(m0,-mode0.minCost);
  }
  while(!q.empty()) {
    numUpdatePathsIterations++;

    int m = q.top();
    q.pop();

    //reached goal
    if(mg.count(m)==0) break;

    ModeGraph::Iterator e;
    for(modeGraph.Begin(m,e);!e.end();e++) {
      //compute propagated cost
      Mode& modet=modeGraph.nodes[e.target()];
      Subset s = modet.subset+modeGraph.nodes[m].pathCovers[0];
      Real cs=Cost(s);

      if(!modet.pathCovers.empty()) {
	if(cs >= modet.minCost)
	  continue;
      }
      q.adjust(e.target(),-cs);

      //propagate the path subset
      modet.pathCovers.resize(1);
      modet.pathCovers[0] = s;
      modet.minCost = cs;
    }
  }

  /*
  //SANITY CHECK
  if(numUpdatePaths % 1000 == 0) {
    vector<Real> modeMinCost(modeGraph.nodes.size());
    for(size_t i=0;i<modeGraph.nodes.size();i++)
      modeMinCost[i] = modeGraph.nodes[i].minCost;
    UpdatePathsGreedy();
    for(size_t i=0;i<modeGraph.nodes.size();i++) 
      if(modeMinCost[i] != modeGraph.nodes[i].minCost) 
	LOG4CXX_WARN(KrisLibrary::logger(),"Warning, dynamic path error, mode with cover "<<modeMinCost[i]<<" actually "<<modeGraph.nodes[i].minCost);
  }
  */
}

void MCRPlannerGoalSet::UpdatePathsComplete()
{
  numUpdatePaths++; 

  vector<bool> visited(modeGraph.nodes.size(),0);
  //index is a (mode index,pathSubset index) pair
  IndexedPriorityQueue<pair<int,int>,Real> q;

  //start from scratch
  for(size_t i=0;i<modeGraph.nodes.size();i++) {
    modeGraph.nodes[i].pathCovers.resize(0);
    modeGraph.nodes[i].minCost = DBL_MAX;
  }

  int m0=roadmap.nodes[0].mode;
  set<int> mg;
  for(size_t i=0;i<goalNodes.size();i++)
    mg.insert(roadmap.nodes[goalNodes[i]].mode);
  Subset sgCover = modeGraph.nodes[m0].subset;
  modeGraph.nodes[m0].pathCovers.resize(1);
  modeGraph.nodes[m0].pathCovers[0] = sgCover;
  q.insert(pair<int,int>(m0,0),Cost(sgCover));
  vector<pair<int,int> > reducible;
  while(!q.empty()) {
    numUpdatePathsIterations++;

    int m = q.top().second.first;
    int subsetIndex = q.top().second.second;
    q.pop();
    visited[m] = 1;

    //reached goal
    if(mg.count(m)!=0) break;

    ModeGraph::Iterator e;
    for(modeGraph.Begin(m,e);!e.end();e++) {
      //compute propagated cost
      Mode& modet = modeGraph.nodes[e.target()];
      //hit the max number of path covers
      if((int)modet.pathCovers.size() >= updatePathsMax)
	continue;

      Subset s = modet.subset+modeGraph.nodes[m].pathCovers[subsetIndex];

      if(visited[e.target()] == 1) { //visited already, look to see if the mode contains a subset of s
	bool skip=false;
	vector<int> replace;
	for(size_t i=0;i<modet.pathCovers.size();i++) {
	  if(modet.pathCovers[i].is_subset(s)) {
	    skip=true;
	    break;
	  }
	  else if(s.is_subset(modet.pathCovers[i])) {
	    replace.push_back((int)i);
	  }
	}
	if(skip) continue;


	//add it in
	int index = (int)modet.pathCovers.size();
	modet.pathCovers.push_back(s);
	q.insert(pair<int,int>(e.target(),index),Cost(s));
	//remove all the nodes that this subset replaces
	for(size_t i=0;i<replace.size();i++) {
	  reducible.push_back(pair<int,int>(e.target(),replace[i]));
	  IndexedPriorityQueue<pair<int,int>,Real>::iterator it;
	  it=q.find(pair<int,int>(e.target(),replace[i]));
	  if(it !=q.end())
	    q.erase(it);
	}
      }
      else {
	//propagate the path subset
	modet.pathCovers.resize(1);
	modet.pathCovers[0] = s;
	q.insert(pair<int,int>(e.target(),0),Cost(s));
	visited[e.target()]=1;
      }
    }
  }
  //erase all the reducible paths (go from end to keep indices valid)
  sort(reducible.begin(),reducible.end());
  reverse(reducible.begin(),reducible.end());
  for(size_t i=0;i<reducible.size();i++) {
    int m=reducible[i].first;
    int path=reducible[i].second;
    modeGraph.nodes[m].pathCovers.erase(modeGraph.nodes[m].pathCovers.begin()+path);
    //update the min cover, below
    visited[i] = 1;
  }
  for(size_t i=0;i<modeGraph.nodes.size();i++) {
    if(visited[i]) UpdateMinCost(modeGraph.nodes[i]);
  }
}

void MCRPlannerGoalSet::UpdatePathsComplete2(int nstart)
{
  numUpdatePaths++; 

  vector<bool> visited(modeGraph.nodes.size(),0);
  //index is a (mode index,pathSubset index) pair
  Heap<pair<int,int>,Real> q;

  set<int> mg;
  for(size_t i=0;i<goalNodes.size();i++)
    mg.insert(roadmap.nodes[goalNodes[i]].mode);
  if(nstart <= 0) {
    for(size_t i=0;i<modeGraph.nodes.size();i++)
      modeGraph.nodes[i].pathCovers.resize(0);
    int m0=roadmap.nodes[0].mode;
    Subset sgCover = modeGraph.nodes[m0].subset;
    modeGraph.nodes[m0].pathCovers.resize(1);
    modeGraph.nodes[m0].pathCovers[0] = sgCover;
    Real c0=Cost(sgCover);
    modeGraph.nodes[m0].minCost = c0;
    q.push(pair<int,int>(m0,0),-c0);
  }
  else {
    int m0 = roadmap.nodes[nstart].mode;
    Mode& mode0 = modeGraph.nodes[m0];
    assert(!mode0.pathCovers.empty());

    //start looking for better paths into nstart
    ModeGraph::Iterator e;
    for(modeGraph.Begin(m0,e);!e.end();e++) {
      Mode& modet = modeGraph.nodes[e.target()];
      if(modet.minCost > mode0.minCost) continue;
      for(size_t j=0;j<modet.pathCovers.size();j++) {
	Subset s = mode0.subset+modet.pathCovers[j];
	bool skip=false;
	for(size_t i=0;i<mode0.pathCovers.size();i++) {
	  if(s.is_subset(mode0.pathCovers[i])) {
	    mode0.pathCovers.erase(mode0.pathCovers.begin()+i);
	    i--;
	  }
	  else if(mode0.pathCovers[i].is_subset(s)) {
	    skip=true;
	    break;
	  }
	}
	if(!skip) mode0.pathCovers.push_back(s);
      }
    }
    UpdateMinCost(mode0);

    for(size_t i=0;i<mode0.pathCovers.size();i++)
      q.push(pair<int,int>(m0,(int)i),-Cost(mode0.pathCovers[i]));
  }

  vector<pair<int,int> > reducible;
  while(!q.empty()) {
    numUpdatePathsIterations++;

    int m = q.top().first;
    int subsetIndex = q.top().second;
    q.pop();

    //reached goal
    if(mg.count(m)!=0) break;

    ModeGraph::Iterator e;
    for(modeGraph.Begin(m,e);!e.end();e++) {
      //compute propagated cost
      Mode& modet = modeGraph.nodes[e.target()];
      //hit the max number of path covers
      if((int)modet.pathCovers.size() >= updatePathsMax)
	continue;

      Subset s = modet.subset+modeGraph.nodes[m].pathCovers[subsetIndex];

      bool skip=false;
      vector<int> replace;
      for(size_t i=0;i<modet.pathCovers.size();i++) {
	if(modet.pathCovers[i].is_subset(s)) {
	  skip=true;
	  break;
	}
	else if(s.is_subset(modet.pathCovers[i])) {
	  replace.push_back((int)i);
	}
      }
      if(skip) continue;
	
      //add it in
      int index = (int)modet.pathCovers.size();
      modet.pathCovers.push_back(s);
      Real cs=Cost(s);
      modet.minCost = Min(modet.minCost,cs);
      q.push(pair<int,int>(e.target(),index),-cs);
      //remove all the nodes that this subset replaces
      for(size_t i=0;i<replace.size();i++) {
	reducible.push_back(pair<int,int>(e.target(),replace[i]));
      }
    }
  }
  //erase all the reducible paths (go from end to keep indices valid)
  sort(reducible.begin(),reducible.end());
  reverse(reducible.begin(),reducible.end());
  for(size_t i=0;i<reducible.size();i++) {
    int m=reducible[i].first;
    int path=reducible[i].second;
    modeGraph.nodes[m].pathCovers.erase(modeGraph.nodes[m].pathCovers.begin()+path);
    UpdateMinCost(modeGraph.nodes[m]);
  }
}

bool MCRPlannerGoalSet::CanImproveConnectivity(const Mode& ma,const Mode& mb,Real maxExplanationCost)
{
  //return true;
  if(&ma == &mb) return false;
  if(ma.pathCovers.empty() || mb.pathCovers.empty()) return true;
  for(size_t i=0;i<ma.pathCovers.size();i++) {
    Subset next=(ma.pathCovers[i] + mb.subset);
    Real cnext = Cost(next);
    if(cnext <= maxExplanationCost) {
      if(updatePathsComplete) {
	for(size_t j=0;j<mb.pathCovers.size();j++) {
	  if(!mb.pathCovers[j].is_subset(next))
	    return true;
	}
      }
      else {
	if(cnext < mb.minCost) return true;
      }
    }
  }
  for(size_t i=0;i<mb.pathCovers.size();i++) {
    Subset next=(mb.pathCovers[i] + ma.subset);
    Real cnext = Cost(next);
    if(cnext <= maxExplanationCost) {
      if(updatePathsComplete) {
	for(size_t j=0;j<ma.pathCovers.size();j++) {
	  if(!ma.pathCovers[j].is_subset(next))
	    return true;
	}
      }
      else {
	if(cnext < ma.minCost) return true;
      }
    }
  }
  return false;
}


//If there are more violations than the limit, return true.
//Otherwise, return false and compute the subset of violations
bool MCRPlannerGoalSet::ExceedsCostLimit(const Config& q,Real limit,Subset& violations)
{
  int n=(int)space->constraints.size();
  /*
  if(!space->IsFeasible(q)) return true;
  violations.maxItem = n;
  return false;
  */

  vector<bool> vis(n);
  Real vcount = 0;
  for(int i=0;i<n;i++) {
    if(!space->IsFeasible(q,i)) {
      if(obstacleWeights.empty()) vcount += 1.0;
      else vcount += obstacleWeights[i];
      vis[i] = true;
      if(vcount > limit) return true;
    }
    else
      vis[i] = false;
  }
  violations = Subset(vis);
  return false;
}

//If there are more violations than the limit, return true.
//Otherwise, return false and compute the subset of violations
bool MCRPlannerGoalSet::ExceedsCostLimit(const Config& a,const Config& b,Real limit,Subset& violations)
{
  int n=(int)space->constraints.size();
  /*
  EdgePlannerPtr e=space->LocalPlanner(a,b);
  if(!e->IsVisible()) {
    return true;
  }
  delete e;
  violations.maxItem = n;
  return false;
  */

  vector<bool> vis(n);
  Real vcount = 0;
  for(int i=0;i<n;i++) {
    EdgePlannerPtr e=space->PathChecker(a,b,i);
    vis[i] = !e->IsVisible();
    if(vis[i]) {
      if(obstacleWeights.empty()) vcount += 1.0;
      else vcount += obstacleWeights[i];
      if(vcount > limit) return true;
    }
  }
  violations = Subset(vis);
  return false;
}


int MCRPlannerGoalSet::AddNode(const Config& q,int parent)
{
  return AddNode(q,Violations(space,q),parent);
}

int MCRPlannerGoalSet::AddNode(const Config& q,const Subset& subset,int parent)
{
#if DO_TIMING
  Timer timer;
#endif
  int index=(int)roadmap.nodes.size();
  roadmap.AddNode(Milestone());
  roadmap.nodes[index].q = q;

  /*
  //Sanity check?
  assert(subset == Violations(space,q));
  */

  if(parent < 0 || modeGraph.nodes[roadmap.nodes[parent].mode].subset != subset)  {
    //LOG4CXX_INFO(KrisLibrary::logger(),"New mode graph node:"<<subset);
    //if(parent >= 0)
    //  LOG4CXX_INFO(KrisLibrary::logger(),"Parent mode: "<<modeGraph.nodes[roadmap.nodes[parent].mode].subset);
    //add a new mode
    int mode = (int)modeGraph.nodes.size();
    modeGraph.AddNode(Mode());
    modeGraph.nodes.back().subset = subset;
    modeGraph.nodes.back().roadmapNodes.push_back(index);
    modeGraph.nodes.back().minCost = DBL_MAX;

    roadmap.nodes[index].mode = mode;

    if(parent >= 0) {
      AddEdgeRaw(parent,index);

      //initialize this mode's path cover
      int pmode = roadmap.nodes[parent].mode;
      modeGraph.nodes.back().pathCovers.resize(modeGraph.nodes[pmode].pathCovers.size());
      for(size_t i=0;i<modeGraph.nodes[pmode].pathCovers.size();i++)
	modeGraph.nodes.back().pathCovers[i] = modeGraph.nodes[pmode].pathCovers[i] + subset;
      UpdateMinCost(modeGraph.nodes.back());
      //LOG4CXX_INFO(KrisLibrary::logger(),"New mode min cost: "<<modeGraph.nodes.back().minCost);
    }
  }
  else {
    //add to the parent's mode
    int mode = roadmap.nodes[parent].mode;
    roadmap.nodes[index].mode = mode;
    modeGraph.nodes[mode].roadmapNodes.push_back(index);
    
    Edge e;
    e.e = space->LocalPlanner(roadmap.nodes[parent].q,roadmap.nodes[index].q);
    e.mode = mode;
    roadmap.AddEdge(parent,index,e);
  }


#if DO_TIMING
  timeOverhead += timer.ElapsedTime();
#endif
  return index;
}

bool MCRPlannerGoalSet::AddEdge(int i,int j,int depth)
{
  if(depth >= 10) return false;
  assert(i != j);
  assert(i >= 0 && i < (int)roadmap.nodes.size());
  assert(j >= 0 && j < (int)roadmap.nodes.size());
  assert(!roadmap.HasEdge(i,j));
  numEdgeChecks++;
  Subset ev=Violations(space,roadmap.nodes[i].q,roadmap.nodes[j].q);
  int mi = roadmap.nodes[i].mode;
  int mj = roadmap.nodes[j].mode;
  assert(mi >= 0 && mi < (int)modeGraph.nodes.size());
  assert(mj >= 0 && mj < (int)modeGraph.nodes.size());
  if(ev != (modeGraph.nodes[mi].subset + modeGraph.nodes[mj].subset)) {
    return false;
    //TODO: what to do here?
    //subdivide until only 1 CC boundary is crossed
    Config qm;
    space->Midpoint(roadmap.nodes[i].q,roadmap.nodes[j].q,qm);
    int k=AddNode(qm);
    if(!AddEdge(i,k,depth+1)) return false;
    if(!AddEdge(k,j,depth+1)) return false;
  }
  else {
    AddEdgeRaw(i,j);
  }
  return true;
}

int MCRPlannerGoalSet::AddEdge(int i,const Config& q,Real maxExplanationCost)
{
  numEdgeChecks++;
  Subset ev,qv;
  numConfigChecks++;
  if(ExceedsCostLimit(q,maxExplanationCost,qv))
    return -1;
  int mi = roadmap.nodes[i].mode;
  if(!WithinThreshold(modeGraph.nodes[mi],qv,maxExplanationCost,obstacleWeights)) 
    return -1;
  numEdgeChecks++;
  if(ExceedsCostLimit(roadmap.nodes[i].q,q,maxExplanationCost,ev))
    return -1;
  if(!WithinThreshold(modeGraph.nodes[mi],ev,maxExplanationCost,obstacleWeights)) 
    return -1;
  const Subset& qiv = modeGraph.nodes[mi].subset;
  if(Cost((qiv + qv)) < Cost(ev)) {
    if(space->Distance(roadmap.nodes[i].q,q) < gSubdivideThreshold)
      return -1;

    //need to subdivide
    Config qm;
    space->Midpoint(roadmap.nodes[i].q,q,qm);
    int j=AddEdge(i,qm,maxExplanationCost);
    if(j < 0) return -1;
    return AddEdge(j,q,maxExplanationCost);
  }
  else {
    //int j=AddNode(q,ev);
    //AddEdgeRaw(i,j);
    int j=AddNode(q,qv,i);
    return j;
  }
}


void MCRPlannerGoalSet::AddEdgeRaw(int i,int j)
{
#if DO_TIMING
  Timer timer;
#endif
  int mi = roadmap.nodes[i].mode;
  int mj = roadmap.nodes[j].mode;
  assert(mi >= 0 && mi < (int)modeGraph.nodes.size());
  assert(mj >= 0 && mj < (int)modeGraph.nodes.size());
  if(modeGraph.nodes[mi].subset != modeGraph.nodes[mj].subset) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Adding a transition: "<<mi<<"->"<<mj);
    //add a transition
    Transition* t=modeGraph.FindEdge(mi,mj);
    if(!t) {
      t=&modeGraph.AddEdge(mi,mj);
    }
    t->connections.push_back(pair<int,int>(i,j));
  }
  else if(mi != mj) {

    Mode& ma=modeGraph.nodes[mi];
    Mode& mb=modeGraph.nodes[mj];
    size_t nmi = ma.roadmapNodes.size();
    size_t nmj = mb.roadmapNodes.size();

    /*
    if(nmi != 1 && nmj != 1){
      LOG4CXX_INFO(KrisLibrary::logger(),"Merging two modes: "<<mi<<"->"<<mj<<", sizes "<<nmi<<" "<<nmj);
      LOG4CXX_INFO(KrisLibrary::logger(),ma.subset);
      LOG4CXX_INFO(KrisLibrary::logger(),mb.subset);
    }
    */

    /*
    //sanity check
    for(size_t m=0;m<ma.roadmapNodes.size();m++)
      assert(ma.roadmapNodes[m] >= 0 && ma.roadmapNodes[m] < (int)roadmap.nodes.size());
    for(size_t m=0;m<mb.roadmapNodes.size();m++)
      assert(mb.roadmapNodes[m] >= 0 && mb.roadmapNodes[m] < (int)roadmap.nodes.size());
    */
    //merge the modes
    ma.roadmapNodes.insert(ma.roadmapNodes.end(),mb.roadmapNodes.begin(),mb.roadmapNodes.end());
    /*
    //sanity check
    for(size_t m=0;m<ma.roadmapNodes.size();m++)
      assert(ma.roadmapNodes[m] >= 0 && ma.roadmapNodes[m] < (int)roadmap.nodes.size());
    */
    if(updatePathsComplete) {
      //do a proper update of the irreducible covers
      vector<Subset> newCovers;
      for(size_t p=0;p<ma.pathCovers.size();p++) {
	bool subset = false;
	bool equal = false;
	for(size_t q=0;q<mb.pathCovers.size();q++) {
	  if(mb.pathCovers[q] == ma.pathCovers[p]) {
	    equal=true;
	    break;
	  }
	  if(mb.pathCovers[q].is_subset(ma.pathCovers[p])) {
	    subset=true;
	  }
	}
	if(!subset || equal) {
	  newCovers.push_back(ma.pathCovers[p]);
	}
      }
      for(size_t p=0;p<mb.pathCovers.size();p++) {
	bool subset = false;
	for(size_t q=0;q<ma.pathCovers.size();q++)
	  if(ma.pathCovers[q].is_subset(mb.pathCovers[p])) {
	    subset=true;
	    break;
	  }
	if(!subset) {
	  newCovers.push_back(mb.pathCovers[p]);
	}
      }
      if(!ma.pathCovers.empty() || !mb.pathCovers.empty())
	assert(!newCovers.empty());
      ma.pathCovers = newCovers;
    }
    else {
      if(mb.minCost < ma.minCost) {
	if(mb.pathCovers.empty()) { 
	  LOG4CXX_WARN(KrisLibrary::logger(),"Warning, empty cover but minCost="<<mb.minCost);
	  mb.minCost = DBL_MAX;
	}
	else {
	  assert(mb.pathCovers.size() >= 1);
	  ma.pathCovers.resize(1);
	  ma.pathCovers[0] = mb.pathCovers[0];
	}
      }
    }
    ma.minCost = Min(ma.minCost,mb.minCost);
    ModeGraph::Iterator e;
    for(modeGraph.Begin(mj,e);!e.end();e++) {
      Transition* t=modeGraph.FindEdge(mi,e.target());
      if(t) {
	t->connections.insert(t->connections.end(),e->connections.begin(),e->connections.end());
      }
      else {
	t=&modeGraph.AddEdge(mi,e.target());
	t->connections = e->connections;
      }
    }
    //LOG4CXX_INFO(KrisLibrary::logger(),"Deleting mode "<<mj<<" / "<<modeGraph.nodes.size()<<" and merging into "<<mi);
    modeGraph.DeleteNode(mj);
    if(mj < (int)modeGraph.nodes.size()) {
      //the n-1'th mode was moved into the mj'th spot
      //move all roadmap node indices there
      for(size_t k=0;k<modeGraph.nodes[mj].roadmapNodes.size();k++)
	roadmap.nodes[modeGraph.nodes[mj].roadmapNodes[k]].mode=mj;
    }
    if(mi == (int)modeGraph.nodes.size()) {
      //the delete moved mi into the mj spot
      mi = mj;
    }
    else {
      mj = mi;
      //switch mj's roadmaps to mi
      for(size_t k=nmi;k<modeGraph.nodes[mi].roadmapNodes.size();k++)
	roadmap.nodes[modeGraph.nodes[mi].roadmapNodes[k]].mode = mi;
    }
    /*
    LOG4CXX_INFO(KrisLibrary::logger(),""<<roadmap.nodes.size());
    for(size_t k=0;k<modeGraph.nodes.size();k++) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Mode %d: ");
      for(size_t m=0;m<modeGraph.nodes[k].roadmapNodes.size();m++) 
	LOG4CXX_INFO(KrisLibrary::logger(),""<<modeGraph.nodes[k].roadmapNodes[m]);
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    }
    */
  }

  /*
  //SANITY CHECK
  for(size_t k=0;k<roadmap.nodes.size();k++) 
    assert(roadmap.nodes[k].mode >= 0 && roadmap.nodes[k].mode < (int)modeGraph.nodes.size());
  for(size_t k=0;k<modeGraph.nodes.size();k++) {
    for(size_t m=0;m<modeGraph.nodes[k].roadmapNodes.size();m++)  
      assert(modeGraph.nodes[k].roadmapNodes[m] >= 0 && modeGraph.nodes[k].roadmapNodes[m] < (int)roadmap.nodes.size());
    set<int> all(modeGraph.nodes[k].roadmapNodes.begin(),modeGraph.nodes[k].roadmapNodes.end());
    assert(all.size() == modeGraph.nodes[k].roadmapNodes.size());
    int oldCover = modeGraph.nodes[k].minCost;
    UpdateMinCost(modeGraph.nodes[k]);
    assert(oldCover == modeGraph.nodes[k].minCost);
  }
  for(size_t k=0;k<modeGraph.nodes.size();k++) 
    for(size_t m=0;m<modeGraph.nodes[k].roadmapNodes.size();m++) 
      assert(roadmap.nodes[modeGraph.nodes[k].roadmapNodes[m]].mode==(int)k);
  */

  Edge e;
  e.e = space->LocalPlanner(roadmap.nodes[i].q,roadmap.nodes[j].q);
  e.mode = (Cost(modeGraph.nodes[mi].subset) > Cost(modeGraph.nodes[mj].subset) ? mi : mj);
  roadmap.AddEdge(i,j,e);
#if DO_TIMING
  timeOverhead += timer.ElapsedTime();
#endif //DO_TIMING
}

int MCRPlannerGoalSet::ExtendToward(int i,const Config& qdest,Real maxExplorationCost)
{
  int mi = roadmap.nodes[i].mode;
  const Subset& ei = modeGraph.nodes[mi].subset;

  assert(gMaxExtendTowardIters == 1);

  numConfigChecks++;
  Subset qv,ev;
  if(ExceedsCostLimit(qdest,maxExplorationCost,qv))
    return -1;
  if(!WithinThreshold(modeGraph.nodes[mi],qv,maxExplorationCost,obstacleWeights)) 
    return -1;
  numEdgeChecks++;
  if(ExceedsCostLimit(roadmap.nodes[i].q,qdest,maxExplorationCost,ev)) 
    return -1;
  if(!WithinThreshold(modeGraph.nodes[mi],ev,maxExplorationCost,obstacleWeights)) 
    return -1;
  if(Cost((qv + modeGraph.nodes[mi].subset)) < Cost(ev)) {
    //crosses extra obstacles... do we want to subdivide?
    if(space->Distance(roadmap.nodes[i].q,qdest) < gSubdivideThreshold)
      return -1;

    //need to subdivide
    Config qm;
    space->Midpoint(roadmap.nodes[i].q,qdest,qm);
    int j=AddEdge(i,qm,maxExplorationCost);
    if(j < 0) return -1;
    return AddEdge(j,qdest,maxExplorationCost);
  }
  //done
  int j = AddNode(qdest,qv,i);
  //int j = AddNode(qdest,ev);
  //AddEdgeRaw(i,j);
  return j;
}


void MCRPlannerGoalSet::KNN(const Config& q,int k,vector<int>& neighbors,vector<Real>& distances)
{
  //int maxIdx=0;
  distances.resize(0);
  neighbors.resize(0);
  distances.reserve(k);
  neighbors.reserve(k);
  set<pair<Real,int> > knn;
  Real dmax = Inf;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    Real d=space->Distance(roadmap.nodes[i].q,q);
    if(d < connectThreshold) {
      /*
	if(distances.size()==k) {
	  if(d < distances[maxIdx]) {
	    distances[maxIdx] = d;
	    neighbors[maxIdx] = (int)i;
	    for(int j=0;j<k;j++)
	      if(distances[j] > distances[maxIdx])
		maxIdx=j;
	  }
	}
	else {
	  distances.push_back(d);
	  neighbors.push_back((int)i);
	  if(d > distances[maxIdx])
	    maxIdx = (int)distances.size()-1;
	}
	*/
      if(d < dmax) {
	pair<Real,int> idx(d,(int)i);
	knn.insert(idx);
	if((int)knn.size() > k)
	  knn.erase(--knn.end());
	dmax = (--knn.end())->first;
      }
    }
  }
  for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
    distances.push_back(j->first);
    neighbors.push_back(j->second);
  }
}

void MCRPlannerGoalSet::KNN(const Config& q,Real maxExplanationCost,int k,vector<int>& neighbors,vector<Real>& distances)
{
  //int maxIdx=0;
  distances.resize(0);
  neighbors.resize(0);
  distances.reserve(k);
  neighbors.reserve(k);
  set<pair<Real,int> > knn;
  Real dmax = Inf;
  for(size_t m=0;m<modeGraph.nodes.size();m++) {
    if(!WithinThreshold(modeGraph.nodes[m],maxExplanationCost)) continue;
    for(size_t idx=0;idx<modeGraph.nodes[m].roadmapNodes.size();idx++) {
      int i=modeGraph.nodes[m].roadmapNodes[idx];
      Real d=space->Distance(roadmap.nodes[i].q,q);
      if(d < connectThreshold) {
	/*
	if(distances.size()==k) {
	  if(d < distances[maxIdx]) {
	    distances[maxIdx] = d;
	    neighbors[maxIdx] = (int)i;
	    for(int j=0;j<k;j++)
	      if(distances[j] > distances[maxIdx])
		maxIdx=j;
	  }
	}
	else {
	  distances.push_back(d);
	  neighbors.push_back((int)i);
	  if(d > distances[maxIdx])
	    maxIdx = (int)distances.size()-1;
	}
	*/
	if(d < dmax) {
	  pair<Real,int> idx(d,i);
	  knn.insert(idx);
	  if((int)knn.size() > k)
	    knn.erase(--knn.end());
	  dmax = (--knn.end())->first;
	}
      }
    }
  }
  for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
    distances.push_back(j->first);
    neighbors.push_back(j->second);
  }
}

void MCRPlannerGoalSet::Expand(Real maxExplanationCost,vector<int>& newNodes)
{
  numExpands++;
#if DO_TIMING
  Timer timer;
#endif //DO_TIMING

  newNodes.resize(0);
  Config q;
  space->Sample(q);
  int kmax = numConnections;
  if(numConnections < 0) {
    kmax = int(((1.0+1.0/q.n)*E)*Log(Real(roadmap.nodes.size())));
    assert(kmax >= 1);
  }
  //do nearest neighbors query
  vector<Real> kclosest;
  vector<int> kneighbors;
  KNN(q,maxExplanationCost,kmax,kneighbors,kclosest);

#if DO_TIMING
  timeNearestNeighbors += timer.ElapsedTime();
  timer.Reset();
#endif // DO_TIMING

  if(Rand() < goalBiasProbability && !goalNodes.empty()) 
    q = roadmap.nodes[goalNodes[RandInt(goalNodes.size())]].q;

  if(kneighbors.empty()) return;

  //attempt connections
  bool didRefine = false;
  newNodes.resize(0);
  Subset qsubset;
  Real minDist = Inf;
  size_t closestIndex = 0;
  for(size_t j=0;j<kclosest.size();j++) 
    if(kclosest[j] < minDist) {
      minDist = kclosest[j];
      closestIndex = j;
    }
  if(minDist < expandDistance) {
    numRefinementAttempts++;
    
    //do a direct connection
    numConfigChecks++;
    qsubset = Violations(space,q);
    
    int nearmode = roadmap.nodes[kneighbors[closestIndex]].mode;
    if(WithinThreshold(modeGraph.nodes[nearmode],qsubset,maxExplanationCost,obstacleWeights)) { //if config itself violates too many constraints, we're not going to connect any nodes
      int n=AddEdge(kneighbors[closestIndex],q,maxExplanationCost);
      if(n >= 0) {
	newNodes.push_back(n); 
	numRefinementSuccesses++;
	int nmode = roadmap.nodes[n].mode;
	
	//check the other close nodes
	for(size_t j=0;j<kclosest.size();j++) {
	  if(j == closestIndex) continue;
	  if(kclosest[j] >= expandDistance) continue;
	  int mode = roadmap.nodes[kneighbors[j]].mode;
	  //check if it will make a difference

	  //the hypothetical path to this node is within the limit?
	  if(CanImproveConnectivity(modeGraph.nodes[nmode],modeGraph.nodes[mode],maxExplanationCost)) {
	    if(AddEdge(kneighbors[j],n)) {
	      didRefine = true;
	      //occasionally see a crash above -- may need up update mode?
	      nmode = roadmap.nodes[n].mode;
	    }
	  }
	}
      }
    }
  }
#if DO_TIMING
  timeRefine += timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  if(newNodes.empty()) {
    numExplorationAttempts++;

    int n=kneighbors[closestIndex];
    /*if(validModes[roadmap.nodes[n].mode])*/ {
      //LOG4CXX_INFO(KrisLibrary::logger(),"ExtendTowards from "<<n);
      //do an RRT-style extension
      Real u=expandDistance/kclosest[closestIndex];
      Config qu;
      space->Interpolate(roadmap.nodes[n].q,q,u,qu);
      int res=ExtendToward(n,qu,maxExplanationCost);
      if(res >= 0)
	newNodes.push_back(res);
    }
  }
#if DO_TIMING
  timeExplore += timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  if(!bidirectional) {
    //check for connection with goal
    for(size_t i=0;i<newNodes.size();i++) {
      Config qgoal = roadmap.nodes[newNodes[i]].q;
      int goalindex = -1;
      if(goalSet->Contains(qgoal)) {
        goalindex = newNodes[i];
        didRefine = true;
      }
      else {
        bool project = false;
        if(IsInf(goalConnectThreshold)) project = true;
        else {
          if(goalNumeric) {
            if(MaxError(goalNumeric,qgoal) < goalConnectThreshold)
              project = true;
          }
        }
        if(project) {
          if(goalSet->Project(qgoal)) {
            int res=AddEdge(newNodes[i],qgoal,maxExplanationCost);
            didRefine = true;
            if(res >= 0) {
              if(goalSet->Contains(qgoal)) 
                goalindex = res;
              else 
                LOG4CXX_INFO(KrisLibrary::logger(),"Note: Goal projection got closer to the goal, but failed to reach it exactly\n");
            }
          }
        }
        if(goalindex >= 0)
    	    goalNodes.push_back(goalindex);
      }
    }
  }
  else
    abort();

#if DO_TIMING
  timeOverhead += timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  if(didRefine) {
    if(updatePathsDynamic) {
      for(size_t i=0;i<newNodes.size();i++)
	if(updatePathsComplete) UpdatePathsComplete2(newNodes[i]);
	else UpdatePathsGreedy2(newNodes[i]);
    }
    else {
      if(updatePathsComplete) UpdatePathsComplete();
      else UpdatePathsGreedy();
    }

#if DO_TIMING
    timeUpdatePaths += timer.ElapsedTime();
#endif //DO_TIMING
  }
}

void MCRPlannerGoalSet::Expand2(Real maxExplanationCost,vector<int>& newNodes)
{
  numExpands++;
#if DO_TIMING
  Timer timer;
#endif //DO_TIMING

  newNodes.resize(0);
  Config q;
  space->Sample(q);

  if(Rand() < goalBiasProbability && !goalNodes.empty()) 
    q = roadmap.nodes[goalNodes[RandInt(goalNodes.size())]].q;

  int kmax = numConnections;
  if(numConnections < 0) {
    kmax = int(((1.0+1.0/q.n)*E)*Log(Real(roadmap.nodes.size())));
    assert(kmax >= 1);
  }
  //do nearest neighbors query
  vector<Real> closest;
  vector<int> neighbor;
  KNN(q,maxExplanationCost,1,neighbor,closest);
  assert(neighbor.size() <= 1);
#if DO_TIMING
  timeNearestNeighbors += timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  if(neighbor.empty()) return;

  //attempt connections
  bool didRefine = false;
  newNodes.resize(0);
  Subset qsubset;
  if(closest[0] < expandDistance) {
    numRefinementAttempts++;
    
    //do a direct connection
    numConfigChecks++;
    qsubset = Violations(space,q);

    int nearmode = roadmap.nodes[neighbor[0]].mode;    
    if(WithinThreshold(modeGraph.nodes[nearmode],qsubset,maxExplanationCost,obstacleWeights)) { //if config itself violates too many constraints, we're not going to connect any nodes
      numRefinementSuccesses++;
      int n=AddEdge(neighbor[0],q,maxExplanationCost);
      if(n >= 0) {
	newNodes.push_back(n); 
	assert(!modeGraph.nodes[roadmap.nodes[n].mode].pathCovers.empty());
      }
    }
  }
#if DO_TIMING
  timeRefine += timer.ElapsedTime();
  timer.Reset();
#endif

  if(newNodes.empty()) {
    numExplorationAttempts++;

    int n=neighbor[0];
    //do an RRT-style extension
    Real u=expandDistance/closest[0];
    Config qu;
    space->Interpolate(roadmap.nodes[n].q,q,u,qu);
    int res=ExtendToward(n,qu,maxExplanationCost);
    if(res >= 0) {
      newNodes.push_back(res);
    }
  }
#if DO_TIMING
  timeExplore += timer.ElapsedTime();
  timer.Reset();
#endif

  if(!newNodes.empty()) {
    int n = newNodes[0];
    int nmode = roadmap.nodes[n].mode;

    vector<Real> kclosest;
    vector<int> kneighbors;
    KNN(roadmap.nodes[n].q,kmax*5+1,kneighbors,kclosest);
#if DO_TIMING
    timeNearestNeighbors += timer.ElapsedTime();
    timer.Reset();    
#endif

    //check the other close nodes
    int numadded=0;
    for(size_t j=0;j<kclosest.size();j++) {
      if(kneighbors[j] == n) continue;
      if(kclosest[j] >= expandDistance) continue;
      if(roadmap.HasEdge(n,kneighbors[j])) continue;
      int mode = roadmap.nodes[kneighbors[j]].mode;
      //check if it will make a difference
      
      //the hypothetical path to this node is within the limit?
      if(CanImproveConnectivity(modeGraph.nodes[nmode],modeGraph.nodes[mode],maxExplanationCost)) {
	if(AddEdge(kneighbors[j],n)) {
	  didRefine = true;
	  //occasionally see a crash above -- may need up update mode?
	  nmode = roadmap.nodes[n].mode;
	  assert(!modeGraph.nodes[nmode].pathCovers.empty());
	  numadded++;
	  if(numadded == kmax) break;
	}
      }
    }
  }

  if(!bidirectional) {
    for(size_t i=0;i<newNodes.size();i++) {
      Config qgoal = roadmap.nodes[newNodes[i]].q;
      int goalindex = -1;
      if(goalSet->Contains(qgoal)) {
        goalindex = newNodes[i];
        didRefine = true;
      }
      else {
        bool project = false;
        if(IsInf(goalConnectThreshold)) project = true;
        else {
          if(goalNumeric) {
            if(MaxError(goalNumeric,qgoal) < goalConnectThreshold)
              project = true;
          }
        }
        if(project) {
          if(goalSet->Project(qgoal)) {
            int res=AddEdge(newNodes[i],qgoal,maxExplanationCost);
            didRefine = true;
            if(res >= 0) {
              if(goalSet->Contains(qgoal)) 
                goalindex = res;
              else 
                LOG4CXX_INFO(KrisLibrary::logger(),"Note: Goal projection got closer to the goal, but failed to reach it exactly\n");
            }
          }
        }
        if(goalindex >= 0)
          goalNodes.push_back(goalindex);
      }
    }
  }
  else
    abort();

#if DO_TIMING
  timeRefine += timer.ElapsedTime();
  timer.Reset();
#endif

  if(didRefine) {
    if(updatePathsDynamic) {
      for(size_t i=0;i<newNodes.size();i++)
	if(updatePathsComplete) UpdatePathsComplete2(newNodes[i]);
	else UpdatePathsGreedy2(newNodes[i]);
    }
    else {
      if(updatePathsComplete) UpdatePathsComplete();
      else UpdatePathsGreedy();
    }
#if DO_TIMING
    timeUpdatePaths += timer.ElapsedTime();
#endif //DO_TIMING
  }
}

void MCRPlannerGoalSet::Plan(int initialLimit,const vector<int>& expansionSchedule,vector<int>& bestPath,Subset& bestCover)
{
  bestCover.maxItem = space->constraints.size();
  bestCover.items.resize(bestCover.maxItem);
  for(int i=0;i<bestCover.maxItem;i++)
    bestCover.items[i] = (int)i;

  Subset lowerCover;
  numConfigChecks += 1;
  lowerCover=Violations(space,start);

  Real lowerCost = Cost(lowerCover);
  Real bestCost = Cost(bestCover);
  Real costEpsilon = 1.0;
  if(!obstacleWeights.empty()) {
    vector<Real> wsorted = obstacleWeights;
    sort(wsorted.begin(),wsorted.end());
    size_t i=0;
    while(wsorted[i] <= 0 && i<wsorted.size()) i++;
    if(i<wsorted.size()) {
      costEpsilon = wsorted[i];
      i++;
      for(;i<wsorted.size();i++)
	costEpsilon = Min(costEpsilon,wsorted[i+1]-wsorted[i]);
    }
  }
  int expansionIndex = 0;
  Real limit = initialLimit;
  if(limit < lowerCost) limit = lowerCost;
  Timer timer;
  vector<Real> progress_covers;
  vector<int> progress_iters;
  vector<double> progress_times;
  for(int iters=0;iters<expansionSchedule.back();iters++) {
    if(iters == expansionSchedule[expansionIndex]) {
      limit += (bestCost-lowerCost)/double(expansionSchedule.size()-expansionIndex);
      if(limit >= bestCost)
	limit = bestCost-costEpsilon;
      if(limit < lowerCost) limit = lowerCost;
      //LOG4CXX_INFO(KrisLibrary::logger(),"Iter "<<iters<<", now searching at limit "<<limit);
      expansionIndex++;
    }
    if(FuzzyEquals(bestCost,lowerCost)) break;
    
    vector<int> newnodes;
    //Expand(limit,newnodes);
    Expand2(limit,newnodes);
    
    set<int> mg;
    for(size_t i=0;i<goalNodes.size();i++)
      mg.insert(roadmap.nodes[goalNodes[i]].mode);
    for(set<int>::const_iterator i=mg.begin();i!=mg.end();i++) {
      int mgoal = *i;
      for(size_t k=0;k<modeGraph.nodes[mgoal].pathCovers.size();k++)
	if(Cost(modeGraph.nodes[mgoal].pathCovers[k]) < Cost(bestCover)) {
	  //bool res=GreedyPath(0,1,bestPath,bestCover);
	  //assert(res);
	  bestCover = modeGraph.nodes[mgoal].pathCovers[k];
	  bestCost = Cost(bestCover);
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Iter "<<iters<<": improved cover to "<<bestCost);
	  progress_covers.push_back(bestCost);
	  progress_iters.push_back(iters);
	  progress_times.push_back(timer.ElapsedTime());
	  
	  if(limit >= bestCost)
	    limit = bestCost-costEpsilon;
	  if(limit < lowerCost) limit = lowerCost;
	}
    }
    if(FuzzyEquals(bestCost,lowerCost)) break;
  }

  if(!progress_iters.empty()) {
    /*
    LOG4CXX_INFO(KrisLibrary::logger(),"Cover: ");
    for(size_t i=0;i<progress.size();i++)
      LOG4CXX_INFO(KrisLibrary::logger(),""<< "    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Iters: ");
    for(size_t i=0;i<progress.size();i++)
      LOG4CXX_INFO(KrisLibrary::logger(),""<< "    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    */
    LOG4CXX_INFO(KrisLibrary::logger(),"Cover "<<progress_covers.back()<<", best time "<<progress_times.back());
  }
  else
    LOG4CXX_INFO(KrisLibrary::logger(),"Cover "<<Cost(bestCover)<<", best time "<<timer.ElapsedTime());
  /*
  if(GreedyPath(0,1,bestPath,bestCover)) {
  }
  else {
    //straight line path from start to goal
    bestPath.resize(2);
    bestPath[0] = 0; bestPath[1] = 1;
  }
  */
}

void MCRPlannerGoalSet::BuildRoadmap(Real maxExplanationCost,RoadmapPlanner& prm)
{
  vector<bool> useMode(modeGraph.nodes.size());
  for(size_t i=0;i<modeGraph.nodes.size();i++) 
    useMode[i] = (Cost(modeGraph.nodes[i].subset) <= maxExplanationCost);
  vector<int> subsetMap(roadmap.nodes.size(),-1);
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    if(useMode[roadmap.nodes[i].mode]) 
      subsetMap[i] = prm.roadmap.AddNode(roadmap.nodes[i].q);
  }
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    if(useMode[roadmap.nodes[i].mode]) {
      Graph::EdgeIterator<Edge> e;
      for(roadmap.Begin(i,e);!e.end();e++) {
	if(useMode[roadmap.nodes[e.target()].mode]) {
	  prm.roadmap.AddEdge(subsetMap[i],subsetMap[e.target()],e->e);
	}
      }
    }
  }
}

void UniformModeDFS(MCRPlannerGoalSet* eep,int mode,int vertex,vector<bool>& marked,vector<int>& visited)
{
  if(marked[vertex]) return;
  if(eep->roadmap.nodes[vertex].mode != mode) return;

  marked[vertex]=true;
  visited.push_back(vertex);
  Graph::UndirectedEdgeIterator<MCRPlannerGoalSet::Edge> e;
  for(eep->roadmap.Begin(vertex,e);!e.end();++e) {
    UniformModeDFS(eep,mode,e.target(),marked,visited);
  }
}

void MCRPlannerGoalSet::BuildCCGraph(Graph::UndirectedGraph<Subset,int>& G)
{
  vector<int> nodeCCs(roadmap.nodes.size(),-1);
  vector<bool> marked(roadmap.nodes.size(),false);
  for(size_t i=0;i<modeGraph.nodes.size();i++) {
    for(size_t j=0;j<modeGraph.nodes[i].roadmapNodes.size();j++) {
      int nj=modeGraph.nodes[i].roadmapNodes[j];
      if(!marked[nj]) {
	vector<int> visited;
	UniformModeDFS(this,i,nj,marked,visited);
	assert(visited.size() >= 1);
	G.AddNode(modeGraph.nodes[i].subset);
	for(size_t k=0;k<visited.size();k++) {
	  int n=visited[k];
	  nodeCCs[n] = (int)G.nodes.size()-1;
	}
      }
    }
  }
  for(size_t i=0;i<modeGraph.nodes.size();i++) {
    Graph::EdgeIterator<Transition> e;
    for(modeGraph.Begin(i,e);!e.end();e++) {
      for(size_t j=0;j<e->connections.size();j++) {
	int i1=e->connections[j].first;
	int i2=e->connections[j].second;
	int cc1 = nodeCCs[i1];
	int cc2 = nodeCCs[i2];
	if(!G.HasEdge(cc1,cc2)) {
	  G.AddEdge(cc1,cc2);
	}
      }
    }
  }
}



struct CoverageLimitedPathCallback: public Graph::PathIntCallback
{
  MCRPlannerGoalSet* planner;
  const Subset& cover;

  CoverageLimitedPathCallback(MCRPlannerGoalSet* _planner,const Subset& _cover,int _target=-1)
    :PathIntCallback(_planner->roadmap.nodes.size(),_target),planner(_planner),cover(_cover)
  {}

  virtual bool ForwardEdge(int i,int j) {
    int modej = planner->roadmap.nodes[j].mode;
    const Subset& subj = planner->modeGraph.nodes[modej].subset;
    return subj.is_subset(cover);
  }
};

bool MCRPlannerGoalSet::CoveragePath(int s,int t,const Subset& cover,vector<int>& path,Subset& pathCover)
{
  CoverageLimitedPathCallback callback(this,cover,t);
  roadmap._DFS(s,callback);
  if(Graph::GetAncestorPath(callback.parents,t,s,path)) {
    pathCover = Subset();
    for(size_t i=0;i<path.size();i++)
      pathCover = pathCover + modeGraph.nodes[roadmap.nodes[path[i]].mode].subset;
    return true;
  }
  return false;
}


/** Uses size comparisons for a partial ordering,
 * rather than element comparisons
 */
struct SubsetCost
{
  Subset subset;
  Real pathCost;
  vector<Real>* weights;

  SubsetCost(int maxItem=0,Real cost=0,vector<Real>* _weights=NULL)
    :subset(maxItem),pathCost(cost),weights(_weights)
  {}

  SubsetCost(const Subset& s,Real cost=0,vector<Real>* _weights=NULL)
    :subset(s),pathCost(cost),weights(_weights)
  {}

  SubsetCost(const SubsetCost& s)
    :subset(s.subset),pathCost(s.pathCost),weights(s.weights)
  {}

  bool operator < (const SubsetCost& s) const
  {
    assert(weights==s.weights);
    if(weights) {
      if(WeightedCost(subset,*weights) < WeightedCost(s.subset,*weights)) return true;
      if(WeightedCost(subset,*weights) == WeightedCost(s.subset,*weights)) return pathCost < s.pathCost;
      return false;
    }
    else {
      if(subset.size() < s.subset.size()) return true;
      if(subset.size() == s.subset.size()) return pathCost < s.pathCost;
      return false;
    }
  }

  bool operator > (const SubsetCost& s) const
  {
    return s < *this;
  }

  bool operator == (const SubsetCost& s) const
  {
    if(weights)
      return WeightedCost(subset,*weights) == WeightedCost(s.subset,*weights) && pathCost == s.pathCost;
    else
      return subset.size() == s.subset.size() && pathCost == s.pathCost;
  }
  
  bool operator != (const SubsetCost& s) const
  {
    return !(operator == (s));
  }

  SubsetCost operator + (const SubsetCost& s) const
  {
    return SubsetCost(subset+s.subset,pathCost + s.pathCost,weights);
  }

  SubsetCost operator - () const
  {
    return SubsetCost (-subset,-pathCost,weights);
  }
};


struct GreedySubsetAStar2 : public GeneralizedAStar<int,SubsetCost>
{
  typedef GeneralizedAStar<int,SubsetCost>::Node Node;
  MCRPlannerGoalSet* planner;
  int startNode;
  set<int> targetNodes;
  vector<Node*> visited;

  GreedySubsetAStar2(MCRPlannerGoalSet* _planner,int _start,const vector<int> _targets)
    :planner(_planner),startNode(_start),targetNodes(_targets.begin(),_targets.end())
  {
    SetStart(_start);
    root.g = SubsetCost(planner->modeGraph.nodes[planner->roadmap.nodes[startNode].mode].subset,0,&planner->obstacleWeights);
    root.f = root.g;
  }

  virtual bool IsGoal(const int& s) { return targetNodes.count(s)!=0; }

  virtual void Successors(const int& s,vector<int>& successors,vector<SubsetCost>& cost) {
    Graph::UndirectedEdgeIterator<MCRPlannerGoalSet::Edge> e;
    successors.resize(0);
    cost.resize(0);
    for(planner->roadmap.Begin(s,e);!e.end();e++) {
      Real dist = planner->space->Distance(planner->roadmap.nodes[s].q,planner->roadmap.nodes[e.target()].q);

      successors.push_back(e.target());
      int mode=planner->roadmap.nodes[e.target()].mode;
      if(mode != planner->roadmap.nodes[s].mode) { //transition
	SubsetCost c(planner->modeGraph.nodes[mode].subset,dist,&planner->obstacleWeights);
	cost.push_back(c);
      }
      else {
	//same subset
	SubsetCost c(planner->space->constraints.size(),dist);
	cost.push_back(c);
      }
    }
  }

  virtual void ClearVisited()
  {
    visited.clear();
    visited.resize(planner->roadmap.nodes.size(),NULL);
  }

  virtual void Visit(const int& s,Node* n)
  {
    visited[s] = n;
  }

  virtual Node* VisitedStateNode(const int& s)
  {
    return visited[s];
  }
};


struct OptimalSubsetAStar2 : public GeneralizedAStar<pair<int,Subset>,SubsetCost>
{
  typedef pair<int,Subset> State;
  typedef GeneralizedAStar<State,SubsetCost>::Node Node;
  MCRPlannerGoalSet* planner;
  int startNode;
  set<int> targetNodes;
  vector<vector<pair<Subset,Node*> > > visited;

  OptimalSubsetAStar2(MCRPlannerGoalSet* _planner,int _start,const vector<int>& _targets)
    :planner(_planner),startNode(_start),targetNodes(_targets.begin(),_targets.end())
  {
    const Subset& m0=planner->modeGraph.nodes[planner->roadmap.nodes[startNode].mode].subset;
    SetStart(pair<int,Subset>(_start,m0));
    root.g = SubsetCost(m0,0,&planner->obstacleWeights);
    root.f = root.g;
  }

  virtual bool IsGoal(const State& s) { return targetNodes.count(s.first)!=0; }

  virtual void Successors(const State& s,vector<State>& successors,vector<SubsetCost>& cost) {
    Graph::UndirectedEdgeIterator<MCRPlannerGoalSet::Edge> e;
    successors.resize(0);
    cost.resize(0);
    for(planner->roadmap.Begin(s.first,e);!e.end();e++) {
      Real dist = planner->space->Distance(planner->roadmap.nodes[s.first].q,planner->roadmap.nodes[e.target()].q);

      int mode=planner->roadmap.nodes[e.target()].mode;
      if(mode != planner->roadmap.nodes[s.first].mode) { //transition
	SubsetCost c(planner->modeGraph.nodes[mode].subset,dist,&planner->obstacleWeights);
	cost.push_back(c);
      }
      else {
	//same subset
	SubsetCost c(planner->space->constraints.size(),dist,&planner->obstacleWeights);
	cost.push_back(c);
      }
      successors.push_back(pair<int,Subset>(e.target(),cost.back().subset+s.second));
    }
  }

  virtual void ClearVisited()
  {
    visited.clear();
    visited.resize(planner->roadmap.nodes.size());
  }

  virtual void Visit(const State& s,Node* n)
  {
    visited[s.first].push_back(pair<Subset,Node*>(s.second,n));
  }

  virtual Node* VisitedStateNode(const State& s)
  {
    for(size_t i=0;i<visited[s.first].size();i++) {
      if(visited[s.first][i].first.is_subset(s.second))
	return visited[s.first][i].second;
    }
    return NULL;
  }
};


bool MCRPlannerGoalSet::GreedyPath(int s,int t,vector<int>& path,Subset& pathCover)
{
  vector<int> tgts(1,t);
  GreedySubsetAStar2 astar(this,s,tgts);
  if(!astar.Search()) {
    path.clear();
    return false;
  }
  else {
    GreedySubsetAStar2::Node* n=astar.goal;
    path.resize(0);
    while(n) {
      path.push_back(n->data);
      n = n->parent;
    }
    reverse(path.begin(),path.end());
    pathCover = astar.goal->g.subset;
    return true;
  }
}


bool MCRPlannerGoalSet::OptimalPath(int s,int t,vector<int>& path,Subset& pathCover)
{
  vector<int> tgts(1,t);
  OptimalSubsetAStar2 astar(this,s,tgts);
  if(!astar.Search()) {
    path.clear();
    return false;
  }
  else {
    OptimalSubsetAStar2::Node* n=astar.goal;
    path.resize(0);
    while(n) {
      path.push_back(n->data.first);
      n = n->parent;
    }
    reverse(path.begin(),path.end());
    pathCover = astar.goal->g.subset;
    return true;
  }
}


bool MCRPlannerGoalSet::GreedyPath(vector<int>& path,Subset& pathCover)
{
  GreedySubsetAStar2 astar(this,0,goalNodes);
  if(!astar.Search()) {
    path.clear();
    return false;
  }
  else {
    GreedySubsetAStar2::Node* n=astar.goal;
    path.resize(0);
    while(n) {
      path.push_back(n->data);
      n = n->parent;
    }
    reverse(path.begin(),path.end());
    pathCover = astar.goal->g.subset;
    return true;
  }
}


bool MCRPlannerGoalSet::OptimalPath(vector<int>& path,Subset& pathCover)
{
  OptimalSubsetAStar2 astar(this,0,goalNodes);
  if(!astar.Search()) {
    path.clear();
    return false;
  }
  else {
    OptimalSubsetAStar2::Node* n=astar.goal;
    path.resize(0);
    while(n) {
      path.push_back(n->data.first);
      n = n->parent;
    }
    reverse(path.begin(),path.end());
    pathCover = astar.goal->g.subset;
    return true;
  }
}


void MCRPlannerGoalSet::GetMilestonePath(const std::vector<int>& path,MilestonePath& mpath) const
{
  mpath.edges.resize(path.size()-1);
  for(size_t i=0;i+1<path.size();i++) {
    assert(roadmap.HasEdge(path[i],path[i+1]));
    if(path[i] < path[i+1]) 
      mpath.edges[i] = roadmap.FindEdge(path[i],path[i+1])->e->Copy();
    else
      mpath.edges[i] = roadmap.FindEdge(path[i],path[i+1])->e->ReverseCopy();
    assert(mpath.edges[i]->Start()==roadmap.nodes[path[i]].q);
    assert(mpath.edges[i]->End()==roadmap.nodes[path[i+1]].q);
  }
}
