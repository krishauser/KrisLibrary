#define NOMINMAX
#include "MultiModalPlanner.h"
#include <utils/IntTriple.h>
#include <graph/Operations.h>
#include <graph/Path.h>
#include <utils/StatCollector.h>
using namespace std;

const static bool kDebugSearch = false;

MultiModalPRM::MultiModalPRM(MultiModalCSpace<int>* cspace)
  :space(cspace),startMode(-1),startRoadmapIndex(-1),
   goalMode(-1),goalRoadmapIndex(-1),
   numExpandModeSamples(1),numExpandTransSamples(1),
   modeSampleCount(0),transSampleCount(0)
{}

void MultiModalPRM::InitializeExplicit(ExplicitMMCSpace* cspace)
{
  Assert(cspace == space);
  startMode = goalMode = -1;
  startRoadmapIndex=goalRoadmapIndex = -1;
  ccIndex.clear();
  ccs.Clear();
  CopyStructure(cspace->modeGraph,planningGraph);
  for(size_t i=0;i<cspace->modeGraph.nodes.size();i++) {
    planningGraph.nodes[i].sampleCount = 0;
    planningGraph.nodes[i].planner = plannerFactory.Create(cspace->GetModeCSpace(i));
    //planningGraph.nodes[i].sbl->maxExtendDistance = 0.5;
  }
  for(list<TransitionInfo>::iterator i=planningGraph.edgeData.begin();i!=planningGraph.edgeData.end();i++)
    i->sampleCount = 0;
}

void MultiModalPRM::SetStart(const Config& q,int mode)
{
  startMode = mode;
  startRoadmapIndex = planningGraph.nodes[mode].planner->AddMilestone(q);

  //add virtual transition
  TransitionIndex index;
  index.m1 = -1;
  index.m2 = mode;
  index.count = 0;
  printf("Adding virtual trans (%d,%d)#%d\n",index.m1,index.m2,index.count);
  int n=(int)ccIndex.size();
  ccs.AddNode();
  ccIndex[index] = n;

  if(mode == goalMode && goalRoadmapIndex >= 0) {
    planningGraph.nodes[mode].planner->ConnectHint(startRoadmapIndex,goalRoadmapIndex);
  }
}

void MultiModalPRM::SetGoal(const Config& q,int mode)
{
  goalMode = mode;
  goalRoadmapIndex = planningGraph.nodes[mode].planner->AddMilestone(q);

  //add virtual transition
  TransitionIndex index;
  index.m1 = mode;
  index.m2 = -1;
  index.count = 0;
  printf("Adding virtual trans (%d,%d)#%d\n",index.m1,index.m2,index.count);
  int n=(int)ccIndex.size();
  ccs.AddNode();
  ccIndex[index] = n;

  if(mode == startMode && startRoadmapIndex >= 0) {
    planningGraph.nodes[mode].planner->ConnectHint(startRoadmapIndex,goalRoadmapIndex);
  } 
}

void MultiModalPRM::SetStartMode(int mode)
{
  startMode = mode;
  startRoadmapIndex = -1;
}

void MultiModalPRM::SetGoalMode(int mode)
{
  goalMode = mode;
  goalRoadmapIndex = -1;
}

void MultiModalPRM::ExpandAll()
{
  //printf("MultiModalPRM Expand iteration...\n");
  for(size_t i=0;i<planningGraph.nodes.size();i++) {
    PlanningGraph::Iterator e;
    for(planningGraph.Begin(i,e);!e.end();++e) {
      if((int)i == e.source()) {
	if((int)i<e.target())
	  ExpandTrans(i,e.target());
      }
      else {
	if((int)i<e.source())
	  ExpandTrans(i,e.source());
      }
    }
  }
  for(size_t i=0;i<planningGraph.nodes.size();i++) {
    ExpandMode(i);
  }
  //printf("Done.\n");
}

MultiModalPRM::TransitionIndex MultiModalPRM::NodeToTransitionIndex(int mode,int roadmapIndex) const
{
  TransitionIndex index;
  if(mode == startMode && roadmapIndex == startRoadmapIndex) {
    index.m1 = -1;
    index.m2 = startMode;
    index.count = 0;
    return index;
  }
  else if(mode == goalMode && roadmapIndex == goalRoadmapIndex) {
    index.m1 = goalMode;
    index.m2 = -1;
    index.count = 0;
    return index;
  }
  PlanningGraph::Iterator e;
  for(planningGraph.Begin(mode,e);!e.end();++e) {
    index.m1=e.source();
    index.m2=e.target();
    if(index.m2 < index.m1) std::swap(index.m1,index.m2);
    if(mode == index.m1) {
      for(size_t i=0;i<e->prevRoadmapIndices.size();i++) {
	if(e->prevRoadmapIndices[i] == roadmapIndex) {
	  index.count=(int)i;
	  return index;
	}
      }
    }
    else {
      Assert(mode == index.m2);
      for(size_t i=0;i<e->nextRoadmapIndices.size();i++) {
	if(e->nextRoadmapIndices[i] == roadmapIndex) {
	  index.count = (int)i;
	  return index;
	}
      }
    }
  }
  FatalError("Transition not found");
  return index;
}

void MultiModalPRM::ConnectTransitions(int mode,int t1,int t2)
{
  /*
  PlanningGraph::Iterator e;
  int e1_target=-1;
  int e2_target=-1;
  int e1_count=-1;
  int e2_count=-1;
  if((mode == startMode && t1 == startRoadmapIndex) || (mode == goalMode && t1 == goalRoadmapIndex)) {
    e1_target = mode;
    e1_count = 0;
  }
  if((mode == startMode && t2 == startRoadmapIndex) || (mode == goalMode && t2 == goalRoadmapIndex)) {
    e2_target = mode;
    e2_count = 0;
  }
  for(planningGraph.Begin(mode,e);!e.end();++e) {
    if(mode < e.target()) {
      for(size_t i=0;i<e->prevRoadmapIndices.size();i++) {
	if(e->prevRoadmapIndices[i] == t1) {
	  e1_target=e.target();
	  e1_count=(int)i;
	}
	if(e->prevRoadmapIndices[i] == t2) {
	  e2_target=e.target();
	  e2_count=(int)i;
	}
	if(e1_target >= 0 && e2_target >= 0) break;
      }
    }
    else {
      for(size_t i=0;i<e->nextRoadmapIndices.size();i++) {
	if(e->nextRoadmapIndices[i] == t1) {
	  e1_target=e.target();
	  e1_count=(int)i;
	}
	if(e->nextRoadmapIndices[i] == t2) {
	  e2_target=e.target();
	  e2_count=(int)i;
	}
      }
      if(e1_target >= 0 && e2_target >= 0) break;
    }
    if(e1_target >= 0 && e2_target >= 0) break;
  }
  Assert(e1_target >= 0 && e2_target >= 0);
  TransitionIndex index1,index2;
  index1.m1 = mode;
  index1.m2 = e1_target;
  index1.count = e1_count;
  if(index1.m1 > index1.m2) swap(index1.m1,index1.m2);
  else if(index1.m1 == index1.m2) {
    if(mode == startMode) index1.m1 = -1;  //virtual start
    else index1.m2 = -1;   //virtual goal
  }
  index2.m1 = mode;
  index2.m2 = e2_target;
  index2.count = e2_count;
  if(index2.m1 > index2.m2) swap(index2.m1,index2.m2);
  else if(index2.m1 == index2.m2) {
    if(mode == startMode) index2.m1 = -1;  //virtual start
    else index2.m2 = -1;  //virtual goal
  }
  */
  TransitionIndex index1,index2;
  index1 = NodeToTransitionIndex(mode,t1);
  index2 = NodeToTransitionIndex(mode,t2);
  ConnectTransitions(index1,index2);
}

void MultiModalPRM::ConnectTransitions(const TransitionIndex& index1,const TransitionIndex& index2)
{
  map<TransitionIndex,int>::const_iterator i1,i2;
  i1 = ccIndex.find(index1);
  i2 = ccIndex.find(index2);
  Assert(i1 != ccIndex.end());
  Assert(i2 != ccIndex.end());
  //printf("Adding edge between connected components (%d,%d)#%d and (%d,%d)#%d\n",index1.m1,index1.m2,index1.count,index2.m1,index2.m2,index2.count);
  ccs.AddEdge(i1->second,i2->second);
}

void MultiModalPRM::GetTransitionNodes(int mode,vector<TransitionIndex>& transitions,vector<int>& roadmapIndices) const
{
  transitions.resize(0);
  roadmapIndices.resize(0);
  //may be easier if useSBL is true
  PlanningGraph::Iterator e;
  for(planningGraph.Begin(mode,e);!e.end();++e) {
    TransitionIndex ti;
    ti.m1 = e.source();
    ti.m2 = e.target();
    if(ti.m2 < ti.m1) std::swap(ti.m1,ti.m2);
    if(mode == ti.m2) {
      for(size_t i=0;i<e->nextRoadmapIndices.size();i++) {
	ti.count = (int)i;
	transitions.push_back(ti);
	roadmapIndices.push_back(e->nextRoadmapIndices[i]);
      }
    }
    else {
      Assert(mode == ti.m1);
      for(size_t i=0;i<e->prevRoadmapIndices.size();i++) {
	ti.count = (int)i;
	transitions.push_back(ti);
	roadmapIndices.push_back(e->prevRoadmapIndices[i]);
      }
    }
  }
}

void MultiModalPRM::ExpandMode(int mode,int numSamples)
{
  if(numSamples < 0) numSamples = numExpandModeSamples;
  planningGraph.nodes[mode].sampleCount += numSamples;
  modeSampleCount += numSamples;
  //if(planningGraph.nodes[mode].planner->NumMilestones()==0) return;  //no transitions, doesn't make sense to sample yet
  /* TODO: this is faster than the generic version
  if(useSBL) {
    if(planningGraph.nodes[mode].sbl->roadmap.nodes.empty()) return;
    for(int iters=0;iters<numSamples;iters++) {
      pair<int,int> res = planningGraph.nodes[mode].sbl->Expand();
      if(res.first >= 0)  //connected two transitions
	ConnectTransitions(mode,res.first,res.second);
    }
  }
  else */ {
    MotionPlannerInterface* planner = planningGraph.nodes[mode].planner;
    planner->PlanMore(numSamples);
    //look at adjacent modes, and update transition ccs if necessary
    //get all transition nodes in roadmap
    vector<TransitionIndex> transitions;
    vector<int> roadmapIndices;
    GetTransitionNodes(mode,transitions,roadmapIndices);
    for(size_t i=0;i<transitions.size();i++) {
      for(size_t j=0;j<i;j++) {
	if(planner->IsConnected(roadmapIndices[i],roadmapIndices[j]) &&
	   !ccs.SameComponent(ccIndex[transitions[i]],ccIndex[transitions[j]])) {
	  ConnectTransitions(transitions[i],transitions[j]);
	}
      }
    }
  }
}

void MultiModalPRM::ExpandTrans(int m1,int m2,int numSamples)
{
  if(numSamples < 0) numSamples = numExpandTransSamples;
  if(m1 > m2) std::swap(m1,m2);
  TransitionInfo* trans = planningGraph.FindEdge(m1,m2);
  CSpace* tspace=space->GetTransitionCSpace(m1,m2);
  Assert(trans != NULL);
  Assert(tspace != NULL);
  trans->sampleCount += numSamples;
  transSampleCount += numSamples;
  Config temp;
  for(int iters=0;iters<numSamples;iters++) {
    tspace->Sample(temp);
    if(tspace->IsFeasible(temp)) {
      AddTransition(m1,m2,temp);
    }
  }
}

void MultiModalPRM::AddTransition(int m1,int m2,const Config& q)
{
  if(m1 > m2) std::swap(m1,m2);
  TransitionInfo* trans = planningGraph.FindEdge(m1,m2);
  Assert(trans != NULL);
  trans->transitions.push_back(q);
  int pindex,nindex;
  pindex = planningGraph.nodes[m1].planner->AddMilestone(q);
  nindex = planningGraph.nodes[m2].planner->AddMilestone(q);
  trans->prevRoadmapIndices.push_back(pindex);
  trans->nextRoadmapIndices.push_back(nindex);

  TransitionIndex index;
  index.m1 = m1;
  index.m2 = m2;
  index.count = (int)trans->transitions.size()-1;
  //printf("Adding transition (%d,%d)#%d\n",m1,m2,index.count);
  int n=(int)ccIndex.size();
  ccs.AddNode();
  ccIndex[index] = n;

  /*
  if(useSBL) {
    //printf("Mode %d has %d seeds, mode %d has %d seeds\n",m1,planningGraph.nodes[m1].sbl->roadmap.nodes.size(),m2,planningGraph.nodes[m2].sbl->roadmap.nodes.size());
    //add PRT edges
    PlanningGraph::Iterator e;      
    if(m1 == startMode && startRoadmapIndex >= 0) 
      planningGraph.nodes[m1].sbl->AddRoadmapEdge(pindex,startRoadmapIndex);
    if(m1 == goalMode && goalRoadmapIndex >= 0) 
      planningGraph.nodes[m1].sbl->AddRoadmapEdge(pindex,goalRoadmapIndex);
    for(planningGraph.Begin(m1,e);!e.end();e++) {
      if(e.target() == m2) continue;
      if(e.target() > m1) {
	for(size_t i=0;i<e->prevRoadmapIndices.size();i++) 
	  planningGraph.nodes[m1].sbl->AddRoadmapEdge(pindex,e->prevRoadmapIndices[i]);
      }
      else {
	for(size_t i=0;i<e->nextRoadmapIndices.size();i++) 
	  planningGraph.nodes[m1].sbl->AddRoadmapEdge(pindex,e->nextRoadmapIndices[i]);
      }
    }
    if(m2 == startMode && startRoadmapIndex >= 0) 
      planningGraph.nodes[m2].sbl->AddRoadmapEdge(nindex,startRoadmapIndex);
    if(m2 == goalMode && goalRoadmapIndex >= 0) 
      planningGraph.nodes[m2].sbl->AddRoadmapEdge(nindex,goalRoadmapIndex);
    for(planningGraph.Begin(m2,e);!e.end();e++) {
      if(e.target() == m1) continue;
      if(e.target() > m2) {
	for(size_t i=0;i<e->prevRoadmapIndices.size();i++) 
	  planningGraph.nodes[m2].sbl->AddRoadmapEdge(nindex,e->prevRoadmapIndices[i]);
      }
      else {
	for(size_t i=0;i<e->nextRoadmapIndices.size();i++) 
	  planningGraph.nodes[m2].sbl->AddRoadmapEdge(nindex,e->nextRoadmapIndices[i]);
      }
    }
  }
  else */ {
    //printf("Maintaining CCs of transition graph... \n");
    planningGraph.nodes[m1].planner->ConnectHint(pindex);
    planningGraph.nodes[m2].planner->ConnectHint(nindex);
    vector<TransitionIndex> transitions;
    vector<int> roadmapIndices;
    //printf("pindex %d, nindex %d\n",pindex,nindex);
    GetTransitionNodes(m1,transitions,roadmapIndices);
    //printf("m1=%d has %d transitions\n",m1,transitions.size());
    for(size_t i=0;i<transitions.size();i++) {
      //printf("testing m1 [%d,%d]#%d = %d\n",transitions[i].m1,transitions[i].m2,transitions[i].count,roadmapIndices[i]);
      Assert(roadmapIndices[i] < planningGraph.nodes[m1].planner->NumMilestones());
      if(roadmapIndices[i] == pindex) continue;
      if(planningGraph.nodes[m1].planner->IsConnected(pindex,roadmapIndices[i]) &&
	 !ccs.SameComponent(n,ccIndex[transitions[i]])) {
	ConnectTransitions(index,transitions[i]);
      }
    }
    GetTransitionNodes(m2,transitions,roadmapIndices);
    //printf("m2=%d has %d transitions\n",m2,transitions.size());
    for(size_t i=0;i<transitions.size();i++) {
      //printf("testing m2 [%d,%d]#%d = %d\n",transitions[i].m1,transitions[i].m2,transitions[i].count,roadmapIndices[i]);
      Assert(roadmapIndices[i] < planningGraph.nodes[m2].planner->NumMilestones());
      if(roadmapIndices[i] == nindex) continue;
      if(planningGraph.nodes[m2].planner->IsConnected(nindex,roadmapIndices[i]) &&
	 !ccs.SameComponent(n,ccIndex[transitions[i]])) {
	ConnectTransitions(index,transitions[i]);
      }
    }
  }
}

bool MultiModalPRM::IsStartAndGoalConnected() const
{
  if(startMode < 0 || goalMode < 0) return false;
  TransitionIndex istart,igoal;
  istart.m1 = -1;
  istart.m2 = startMode;
  istart.count = 0;
  igoal.m1 = goalMode;
  igoal.m2 = -1;
  igoal.count = 0;
  map<TransitionIndex,int>::const_iterator i1,i2;
  set<int> startCCs,goalCCs;
  if(startRoadmapIndex != -1) {
    i1=ccIndex.find(istart);
    Assert(i1!=ccIndex.end());
    startCCs.insert(ccs.GetComponent(i1->second));
  }
  else {
    //go through each transition from the start mode
    PlanningGraph::Iterator e;      
    for(planningGraph.Begin(startMode,e);!e.end();e++) {
      istart.m1 = startMode;
      istart.m2 = e.target();
      if(istart.m1 > istart.m2) swap(istart.m1,istart.m2);
      istart.count = 0;
      while((i1=ccIndex.find(istart))!=ccIndex.end()) {
	startCCs.insert(ccs.GetComponent(i1->second));
	istart.count++;
      }
      Assert(istart.count == (int)e->transitions.size());
    }
  }
  if(goalRoadmapIndex != -1) {
    i2=ccIndex.find(igoal);
    Assert(i2!=ccIndex.end());
    goalCCs.insert(ccs.GetComponent(i2->second));
  }
  else {
    //go through each transition from the goal mode
    PlanningGraph::Iterator e;      
    for(planningGraph.Begin(goalMode,e);!e.end();e++) {
      igoal.m1 = goalMode;
      igoal.m2 = e.target();
      if(igoal.m1 > igoal.m2) swap(igoal.m1,igoal.m2);
      igoal.count = 0;
      while((i2=ccIndex.find(igoal))!=ccIndex.end()) {
	goalCCs.insert(ccs.GetComponent(i2->second));
	igoal.count++;
      }
      Assert(igoal.count == (int)e->transitions.size());
    }
  }
  //TODO: faster version
  for(set<int>::const_iterator i=goalCCs.begin();i!=goalCCs.end();i++)
    if(startCCs.count(*i) != 0) return true;
  return false;
}




// Callbacks / helper structs
struct EdgeLengthFunction
{
  EdgeLengthFunction(IncrementalMMPRM_Search* _g) : g(_g) {}
  double operator () (MultiModalPRM::TransitionInfo* e,int s,int t) const 
  { return g->EdgeLength(e,s,t); }
  double operator () (int s, int t) const
  { return g->EdgeLength(s,t); }
  IncrementalMMPRM_Search* g;
};

UpdatePrioritySPP::UpdatePrioritySPP(IncrementalMMPRM_Search& _g)
  :Graph::ShortestPathProblem<int,MultiModalPRM::TransitionInfo*>(_g.searchGraph),g(_g)
{}

void UpdatePrioritySPP::OnDistanceUpdate(int n) { g.UpdatePriority(n); }

IncrementalMMPRM_Search::IncrementalMMPRM_Search(MultiModalPRM& _mmprm)
  :mmprm(_mmprm),spp(*this),transSampleWeight(1)
{
}

void IncrementalMMPRM_Search::Init()
{
  fringe.clear();

  searchGraph.Resize(mmprm.planningGraph.NumNodes());
  for(int i=0;i<mmprm.planningGraph.NumNodes();i++)
    searchGraph.nodes[i] = i;
  spp.InitializeSource(mmprm.startMode);
  PushFringe(mmprm.startMode);

  reachedModes.resize(mmprm.planningGraph.NumNodes());
  fill(reachedModes.begin(),reachedModes.end(),false);
  reachedModes[mmprm.startMode] = true;

  outputModes.resize(mmprm.planningGraph.NumNodes());
  fill(outputModes.begin(),outputModes.end(),false);
}

bool IncrementalMMPRM_Search::ExpandMore()
{
  if(fringe.empty()) {
    if(kDebugSearch) printf("Fringe is now empty -- returning all modes\n");
    fill(outputModes.begin(),outputModes.end(),true);
    return true;
  }
  int n=PopFringe();
  int prev=spp.p[n];
  if(prev < 0 || !mmprm.planningGraph.FindEdge(prev,n)->transitions.empty()) {
    reachedModes[n]=true;
    if(n == mmprm.goalMode || outputModes[n]) {
      if(kDebugSearch) printf("Found path: ");
      list<int> path;
      Graph::GetAncestorPath(spp.p,n,-1,path);
      //add to output modes
      bool addedNew = false;
      for(list<int>::const_iterator i=path.begin();i!=path.end();i++) {
	if(!outputModes[*i]) addedNew = true;
	outputModes[*i]=true;
	if(kDebugSearch) printf("%d ",*i);
      }
      if(kDebugSearch) printf("\n");
      if(kDebugSearch && !addedNew) printf("Warning: Incremental-MMPRM didn't add a new mode to the output modes (%d is output and fringe)\n",n);
      //clip out the active path from the search graph
      for(list<int>::const_iterator i=path.begin();i!=path.end();i++) {
	//freeze all distances derived from children of the path
	SearchGraph::Iterator e;
	vector<int> toDelete;
	for(searchGraph.Begin(*i,e);!e.end();++e) {
	  //printf("Edge (%d,%d)\n",e.source(),e.target());
	  if(outputModes[e.source()]  && outputModes[e.target()]) continue;
	  if(e.source() == *i) {
	    if(spp.p[e.target()] == *i) {
	      spp.p[e.target()] = -1;
	      toDelete.push_back(e.target());
	    }
	  }
	  else {
	    Assert(e.target() == *i);
	    if(spp.p[e.source()] == *i) {
	      spp.p[e.source()] = -1;
	      toDelete.push_back(e.source());
	    }
	  }
	}
	//freeze the parent
	for(size_t k=0;k<toDelete.size();k++) {
	  //printf("Deleting %d %d,freezing parent\n",*i,toDelete[k]);
	  searchGraph.DeleteEdge(*i,toDelete[k]);
	}
	//cut out all edges along the path
	if(i!=path.begin()) { //remove edge
	  EdgeLengthFunction w(this);
	  list<int>::const_iterator p=i; --p;
	  searchGraph.DeleteEdge(*p,*i);
	  spp.DeleteUpdate_Undirected(*p,*i,w);
	  //printf("Deleting %d %d and redirecting\n",*i,*p);
	}
      }

      //reactivate modes
      for(list<int>::const_iterator i=path.begin();i!=path.end();i++) {
	reachedModes[*i] = false;
	if(spp.p[*i] >= 0) {
	  //printf("Reactivating %d, parent %d\n",*i,spp.p[*i]);
	  UpdateFringe(*i);
	}
      }
	
      if(kDebugSearch) {
	printf("Fringe: ");
	for(Fringe::const_iterator i=fringe.begin();i!=fringe.end();i++)
	  printf("%d ",i->second);
	printf("\n");
      }
      return true;
    }
    else {  //not a verified goal node
      int oldFringe = fringe.size();
      ExpandAdjacent(n);
      int newFringe = fringe.size();
      if(kDebugSearch) printf("Fringe from size %d -> %d\n",oldFringe,newFringe);
    }
  }
  else {
    SampleMore(n);
    //PushFringe(n);
    UpdateFringe(n);
  }
  return false;
}

void IncrementalMMPRM_Search::SampleMore(int n)
{
  int prev=spp.p[n];
  Assert(prev >= 0);
  mmprm.ExpandTrans(prev,n);
  UpdateEdgeLength(prev,n);
}

void IncrementalMMPRM_Search::ExpandAdjacent(int n)
{
  MultiModalPRM::PlanningGraph::Iterator e;
  int numEdges = 0;
  int numAdded = 0;
  for(mmprm.planningGraph.Begin(n,e);!e.end();++e) {
    numEdges++;
    if(n == e.source()) {
      if(reachedModes[e.target()] && searchGraph.FindEdge(e.source(),e.target())) { 
      } //new edge
      else {
	AddEdge(e.source(),e.target(),&(*e));
	UpdateFringe(e.target());
	//reachedModes[e.target()]=true;
	numAdded++;
      }
    }
    else {
      if(reachedModes[e.source()] && searchGraph.FindEdge(e.source(),e.target())) { 
      } 
      else { //new edge
	AddEdge(e.source(),e.target(),&(*e));
	UpdateFringe(e.source());
	//reachedModes[e.source()]=true;
	numAdded++;
      }
    }
  }
  //printf("Added %d/%d adjacencies to node %d\n",numAdded,numEdges,n);
}

double IncrementalMMPRM_Search::EdgeLength(int s,int t) 
{
  MultiModalPRM::TransitionInfo** e=searchGraph.FindEdge(s,t);
  if(e) return EdgeLength(*e,s,t);
  return Graph::inf;
}

double IncrementalMMPRM_Search::EdgeLength(MultiModalPRM::TransitionInfo* trans,int s,int t)
{
  if(trans->transitions.empty())
    return 1+transSampleWeight*double(trans->sampleCount);
  else return 1;
}

double IncrementalMMPRM_Search::Heuristic(int n)
{
  return 0;
}

void IncrementalMMPRM_Search::UpdateEdgeLength(int s,int t)
{
  //uncomment for dynamic graph
  //spp.d.resize(searchGraph.NumNodes(),Graph::inf);
  //spp.p.resize(searchGraph.NumNodes(),-1);
  EdgeLengthFunction w(this);
  //if(increase)
    spp.IncreaseUpdate_Undirected(s,t,w);
    //else
    spp.DecreaseUpdate_Undirected(s,t,w);
}

void IncrementalMMPRM_Search::UpdatePriority(int n)
{
  Fringe::iterator i=fringe.find(n);
  if(i==fringe.end()) return;
  fringe.erase(i);
  fringe.insert(n,Priority(n));
}

void IncrementalMMPRM_Search::AddEdge(int s,int t,MultiModalPRM::TransitionInfo* e)
{
  if(searchGraph.FindEdge(s,t)) {
    printf("Warning: I-MMPRM_Search::AddEdge: edge already exists\n");
    UpdateEdgeLength(s,t);
    return;
  }
  searchGraph.AddEdge(s,t,e);
  UpdateEdgeLength(s,t);
}

double IncrementalMMPRM_Search::Priority(int n)
{
  return spp.d[n]+Heuristic(n);
}

void IncrementalMMPRM_Search::PushFringe(int node)
{
  fringe.insert(node,Priority(node));
}

void IncrementalMMPRM_Search::UpdateFringe(int node)
{
  fringe.refresh(node,Priority(node));
}

int IncrementalMMPRM_Search::PopFringe()
{
  int n=fringe.top().second;
  fringe.pop();
  return n;
}

void IncrementalMMPRM_Search::RefreshFringe()
{
  vector<pair<double,int> > temp(fringe.size());
  copy(fringe.begin(),fringe.end(),temp.begin());
  for(size_t i=0;i<temp.size();i++) {
    int n=temp[i].second;
    temp[i].first = Priority(n);
  }
  fringe.clear();
  fringe.insert(temp.begin(),temp.end());
}

double IncrementalMMPRM_Search::PathDistance(int n) const
{
  return spp.d[n];
}



IncrementalMMPRM_Explicit::IncrementalMMPRM_Explicit(ExplicitMMCSpace* _space)
  :space(_space),mmprm(_space),search(mmprm),numRefineSamplesPerMode(100),numRefineSamplesPerOldMode(0),numRefineSamplesConstant(0),evenRefinement(true)
{
  mmprm.InitializeExplicit(space);
}

void IncrementalMMPRM_Explicit::Init()
{
  search.Init();
  remainingRefineSamples = 0;
  expandPhaseCount = expandStepCount = 0;
  refinePhaseCount = refineStepCount = 0;
  lastRefineSet.resize(0);
  expandPhaseCount++;
}

void IncrementalMMPRM_Explicit::PlanMore()
{
  if(remainingRefineSamples > 0) {
    RefineMore();
    if(remainingRefineSamples == 0) {
      lastRefineSet = search.outputModes;
      //printf("Failed to find path -- going back to expand\n");
      expandPhaseCount++;
    }
  }
  else {
    bool res=ExpandMore();
    if(res) {
      remainingRefineSamples=PickRefineCount();
      refinePhaseCount++;
    }
  }
}

bool IncrementalMMPRM_Explicit::Done() const
{
  return mmprm.IsStartAndGoalConnected();
}

bool IncrementalMMPRM_Explicit::ExpandMore()
{
  expandStepCount++;
  bool res=search.ExpandMore();
  if(res) return true;
  return false;
}

void IncrementalMMPRM_Explicit::RefineMore()
{
  refineStepCount++;
  if(evenRefinement) {
    //TODO: parameterize number of samples drawn this round
    int numSamplesThisRound = std::min(remainingRefineSamples,100);
    //int numSamplesThisRound = remainingRefineSamples;

    set<IntTriple> priorityQueue;
    IntTriple index;
    for(size_t i=0;i<search.outputModes.size();i++) {
      if(!search.outputModes[i]) continue;
      //add mode
      index.a = mmprm.planningGraph.nodes[i].sampleCount*mmprm.numExpandTransSamples;
      index.b = index.c = (int)i;
      priorityQueue.insert(index);
      //add transitions
      MultiModalPRM::PlanningGraph::Iterator e;
      for(mmprm.planningGraph.Begin(i,e);!e.end();++e) {
	if(search.outputModes[e.source()] && search.outputModes[e.target()]) {
	  index.a = e->sampleCount*mmprm.numExpandModeSamples;
	  if((int)i == e.source()) {
	    if((int)i < e.target()) {
	      index.b = i;
	      index.c = e.target();
	      priorityQueue.insert(index);
	    }
	  }
	  else {
	    if((int)i < e.source()) {
	      index.b = i;
	      index.c = e.source();
	      priorityQueue.insert(index);
	    }
	  }
	}
      }
    }
    vector<int> numModeSamples(search.outputModes.size(),0);
    while(numSamplesThisRound > 0) {
      IntTriple top = *priorityQueue.begin();
      priorityQueue.erase(priorityQueue.begin());
      numSamplesThisRound--;
      remainingRefineSamples--;
      if(top.b == top.c) { //it's a mode
	numModeSamples[top.b]++;
	top.a += mmprm.numExpandTransSamples;
      }
      else {
	mmprm.ExpandTrans(top.b,top.c,1);
	top.a += mmprm.numExpandModeSamples;
      }
      priorityQueue.insert(top);
    }
    for(size_t i=0;i<search.outputModes.size();i++)
      if(numModeSamples[i])
	mmprm.ExpandMode(i,numModeSamples[i]);

    /*
    StatCollector numSamples;
    for(size_t i=0;i<search.outputModes.size();i++) 
    if(search.outputModes[i])
    numSamples.collect(mmprm.planningGraph.nodes[i].sampleCount);
    cout<<"Num samples per mode: "; numSamples.Print(cout); cout<<endl;
    getchar();
    */
  }
  else {  //uniform refinement
    for(size_t i=0;i<search.outputModes.size();i++) {
      if(!search.outputModes[i]) continue;

      int ns = std::min(remainingRefineSamples,mmprm.numExpandModeSamples);
      mmprm.ExpandMode(i,ns);
      remainingRefineSamples -= ns;
      if(remainingRefineSamples <= 0) return;
      
      MultiModalPRM::PlanningGraph::Iterator e;
      for(mmprm.planningGraph.Begin(i,e);!e.end();++e) {
	if(search.outputModes[e.source()] && search.outputModes[e.target()]) {
	  if((int)i == e.source()) {
	    if((int)i < e.target()) {
	      ns = std::min(remainingRefineSamples,mmprm.numExpandTransSamples);
	      mmprm.ExpandTrans(e.source(),e.target(),ns);
	      remainingRefineSamples -= ns;
	      if(remainingRefineSamples <= 0) return;
	    }
	  }
	  else {
	    if((int)i < e.source()) {
	      ns = std::min(remainingRefineSamples,mmprm.numExpandTransSamples);
	      mmprm.ExpandTrans(e.source(),e.target(),ns);
	      remainingRefineSamples -= ns;
	      if(remainingRefineSamples <= 0) return;
	    }
	  }
	}
      }
    }
  }
}

int IncrementalMMPRM_Explicit::PickRefineCount()
{
  int nLast=0;
  int nCur=0;
  for(size_t i=0;i<lastRefineSet.size();i++)
    if(lastRefineSet[i]) nLast++;
  for(size_t i=0;i<search.outputModes.size();i++)
    if(search.outputModes[i]) nCur++;
  if(nCur == (int)search.outputModes.size())
    return INT_MAX;
  return numRefineSamplesPerMode*(nCur-nLast)+numRefineSamplesPerOldMode*nLast+numRefineSamplesConstant;
}
