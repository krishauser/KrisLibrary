#include <KrisLibrary/Logger.h>
#include "DisplacementPlanner.h"
#include <math/random.h>
#include <structs/IndexedPriorityQueue.h>
#include <graph/Path.h>
#include <graph/Operations.h>
#include <Timer.h>
#include <algorithm>
using namespace std;

//on some platforms, timing takes a non-negligible amount of time
#define DO_TIMING 1
#define DEBUG 0

//just sample displacements uniformly at random
#define TEST_RANDOM_DISPLACEMENTS 0

const static int gMaxExtendBisectIters = 3;
const static int gNumOptimizePathSamples = 50;

//picks uniformly among items that have nonzero counts
int PickByPresence(const vector<int>& count_candidate)
{
  int numCandidates = 0;
  for(size_t i=0;i<count_candidate.size();i++)
    if(count_candidate[i]>0) numCandidates ++;
  if(numCandidates==0) return -1;
  int cand = RandInt(numCandidates);
  for(size_t i=0;i<count_candidate.size();i++)
    if(count_candidate[i]>0) {
      if(cand == 0) return i;
      cand--;
    }
  AssertNotReached();
  return -1;
}

//picks an item with probability proportional to its count
int PickByVote(const vector<int>& count_candidate)
{
  int numCandidates = 0;
  for(size_t i=0;i<count_candidate.size();i++)
    numCandidates += count_candidate[i];
  if(numCandidates==0) return -1;
  int cand = RandInt(numCandidates);
  for(size_t i=0;i<count_candidate.size();i++) {
    if(cand < count_candidate[i]) return i;
    cand-=count_candidate[i];
  }
  AssertNotReached();
  return -1;
}

bool IsVisibleAll(ObstacleDisplacementCSpace* cspace,const Config& a,const Config& b,int obstacle)
{
  return cspace->IsVisibleAll(a,b,obstacle);
}


//assuming a is feasible for the given obstacle, checks whether b is feasible
//and whether the intermediate path is feasible.
bool IsVisible(ObstacleDisplacementCSpace* cspace,const Config& a,const Config& b,int obstacle,const Vector& displacement)
{
  cspace->SetDisplacement(obstacle,displacement);
  shared_ptr<EdgePlanner> e = cspace->PathChecker(a,b,obstacle);
  return e->IsVisible();
}

bool IsVisible(ObstacleDisplacementCSpace* cspace,const Config& a,const Config& b,int obstacle)
{
  shared_ptr<EdgePlanner> e = cspace->PathChecker(a,b,obstacle);
  return e->IsVisible();
}

ObstacleDisplacementCSpace::ObstacleDisplacementCSpace()
{}

bool ObstacleDisplacementCSpace::IsDisplaceable(int obstacle) const
{
  if(displacementSpaces.empty()) {
    shared_ptr<CSpace> Di = DisplacementSpace(obstacle); 
    return Di != NULL;
  }
  else return displacementSpaces[obstacle]!=NULL;
}

void ObstacleDisplacementCSpace::SetDisplacement(int obstacle,const Vector& d)
{
  if(obstacleDisplacements.empty()) 
    InitZeroDisplacements();
  obstacleDisplacements[obstacle] = d;
}

void ObstacleDisplacementCSpace::GetDisplacement(int obstacle,Vector& d) const
{
  if(obstacleDisplacements.empty()) 
    d.resize(0);
  else
    d = obstacleDisplacements[obstacle];
}

void ObstacleDisplacementCSpace::SetDisplacementRef(int obstacle,const Vector& d)
{
  if(obstacleDisplacements.empty()) 
    InitZeroDisplacements();
  obstacleDisplacements[obstacle].setRef(d);
}

void ObstacleDisplacementCSpace::GetDisplacementRef(int obstacle,Vector& d) const
{
  if(obstacleDisplacements.empty()) 
    d.resize(0);
  else
    d.setRef(obstacleDisplacements[obstacle]);
}

bool ObstacleDisplacementCSpace::IsFeasible(const Config& q,int obstacle)
{
  if(obstacleDisplacements.empty()) 
    InitZeroDisplacements();
  return IsFeasible(q,obstacle,obstacleDisplacements[obstacle]);
}


void ObstacleDisplacementCSpace::InitZeroDisplacements()
{
  obstacleDisplacements.resize(NumConstraints());
  displacementSpaces.resize(NumConstraints());
  for(size_t i=0;i<obstacleDisplacements.size();i++) {
    displacementSpaces[i] = DisplacementSpace(i);
    if(displacementSpaces[i]) {
      displacementSpaces[i]->Sample(obstacleDisplacements[i]);
      obstacleDisplacements[i].setZero();
    }
  }
}




DisplacementPlanner::DisplacementPlanner(ObstacleDisplacementCSpace* _space)
  :space(_space),
   pathCostWeight(0),
   dynamicDomainExpansion(true),
   updatePathsComplete(false),updatePathsMax(INT_MAX),
   obstacleSampleCount(10),obstacleDescendIters(1),
   numExpands(0),numRefinementAttempts(0),numRefinementSuccesses(0),numExplorationAttempts(0),
   numEdgeChecks(0),numConfigChecks(0),
   numUpdateCovers(0),numUpdateCoversIterations(0),
   timeNearestNeighbors(0),timeRefine(0),timeExplore(0),timeUpdateCoversIn(0),timeUpdateCoversOut(0),timeOverhead(0)
{
  numConnections=10;
  //numConnections=-1;
  connectThreshold=Inf;
  expandDistance = 0.1;
  goalConnectThreshold = 0.5;
  goalBiasProbability = 0.0;
  bidirectional = false;
}

bool DisplacementPlanner::SanityCheck(bool checkOneCoverGreedy)
{
  Assert(pathCovers.size() == roadmap.nodes.size());
  for(size_t i=0;i<pathCovers.size();i++) {
    if(!updatePathsComplete && checkOneCoverGreedy) {
      if(pathCovers[i].covers.size() > 1) {
		LOG4CXX_ERROR(KrisLibrary::logger(),""<<pathCovers[i].covers.size()<<" > 1 paths to node "<<i);
	return false;
      }
    }
    for(size_t j=0;j<pathCovers[i].covers.size();j++) {
      if(pathCovers[i].covers[j]->vertex != (int)i) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Path search node doesn't match vertex, "<<pathCovers[i].covers[j]->vertex<<" "<<i);
	return false;
      }
      if(pathCovers[i].covers[j]->parent != NULL) {
	if(roadmap.FindEdge(i,pathCovers[i].covers[j]->parent->vertex)==NULL) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid path search parent "<<pathCovers[i].covers[j]->parent->vertex<<" -> "<<i);
	  return false;
	}
	if(pathCovers[i].covers[j]->parent->totalCost > pathCovers[i].covers[j]->totalCost) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"Path cost appears to decrease "<<pathCovers[i].covers[j]->parent->totalCost<<" -> "<<pathCovers[i].covers[j]->totalCost<<" on edge "<<pathCovers[i].covers[j]->parent->vertex<<" -> "<<i);
	  return false;
	}
      }
      //check for cycles
      set<PathSearchNode*> visited;
      PathSearchNode* n=pathCovers[i].covers[j].get();
      while(n != NULL) {
	if(visited.count(n) != 0) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"Cycle in search tree starting at node "<<i);
	  return false;
	}
	visited.insert(n);
	n = n->parent;
      }
    }
  }
  return true;
}

void DisplacementPlanner::Init(const Config& _start,const Config& _goal)
{
  roadmap.Cleanup();
  displacementSamples.resize(0);
  pathCovers.resize(0);

  //reset statistics
  numExpands=0;
  numExplorationAttempts=0;
  numRefinementAttempts=0;
  numRefinementSuccesses=0;
  numEdgeChecks=0;
  numConfigChecks=0;
  numUpdateCovers=0;
  numUpdateCoversIterations=0;
  
  //reset to only the zero displacement sample
  displacementSamples.resize(space->NumConstraints());
  displacementSampleCosts.resize(displacementSamples.size());
  displacementSampleOrders.resize(displacementSamples.size());
  space->InitZeroDisplacements();
  for(size_t i=0;i<displacementSamples.size();i++) {
    displacementSamples[i].resize(1);
    displacementSampleCosts[i].resize(1);
    displacementSampleCosts[i][0] = 0;
    if(space->IsDisplaceable(i)) 
      displacementSamples[i][0] = space->obstacleDisplacements[i];
    else
      displacementSamples[i][0].resize(0);
    displacementSampleOrders[i].resize(1);
    displacementSampleOrders[i][0] = 0;
  }

  start=_start;
  goal=_goal;
  AddNode(start);
  AddNode(goal);
  UpdateCoversIn(0,Inf);
  UpdateCoversIn(1,Inf);
  for(size_t i=0;i<displacementSamples.size();i++)
    if(space->displacementSpaces[i]==NULL) {
      if(!roadmap.nodes[0].tests[i].infeasible.empty()) {
	LOG4CXX_WARN(KrisLibrary::logger(),"Warning, Start configuration violates fixed constraint "<<i);
	KrisLibrary::loggerWait();
      }
      if(!roadmap.nodes[1].tests[i].infeasible.empty()) {
	LOG4CXX_WARN(KrisLibrary::logger(),"Warning, Goal configuration violates fixed constraint "<<i);
	KrisLibrary::loggerWait();
      }
    }
}

bool DisplacementPlanner::Plan(int numIters,int numExpandsPerDisp,int numLocalOptimize,Real expandLimitStep,vector<int>& bestPath,vector<int>& bestDisplacements)
{
  if(expandLimitStep <= 0) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Invalid expand limit step "<<expandLimitStep);
    return false;
  }
  Real maxExplanationCost = 0;
  PathSearchNode* n=OptimalPathTo(1);
  Real bestCost = OptimalCost(1);
  bestPath.resize(0);
  if(n != NULL) maxExplanationCost = bestCost;
  for(int iters=0;iters<numIters;iters++) {
    if(n == NULL) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Increasing explanation cost from "<<maxExplanationCost<<" by "<<expandLimitStep<<" to "<<maxExplanationCost+expandLimitStep);
      maxExplanationCost += expandLimitStep;
    }
    if((iters+1) % numExpandsPerDisp == 0) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Displacement sample iter "<<iters);
      AddNewDisplacement(maxExplanationCost);
    }
    vector<int> newnodes;
    Expand(maxExplanationCost,newnodes);
    if(OptimalPathTo(1) != n) {
      if(OptimalCost(1) < bestCost) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Got a new path, optimizing");
	if(numLocalOptimize>0) {
	  RefineGoalPathAndDisplacements(numLocalOptimize,0.5,1.0);
	  ShortcutGoalPath(1,5);
	}

	n = OptimalPathTo(1);
	bestCost = n->totalCost;

	bestDisplacements = n->assignment;
	bestPath.resize(0);
	while(n!=NULL) {
	  bestPath.push_back(n->vertex);
	  n = n->parent;
	}
	reverse(bestPath.begin(),bestPath.end());
      }
    }
  }
  if(bestPath.empty()) return false;
  return true;
}

int DisplacementPlanner::AddNewDisplacement(Real maxTotalCost)
{
  pair<int,Real> des = PickObstacleToSample(maxTotalCost);
  if(des.first < 0) {
    LOG4CXX_INFO(KrisLibrary::logger(),"DisplacementPlanner::AddNewDisplacement: No obstacle needs sampling");
    return -1;
  }
  if(!space->displacementSpaces[des.first]) {
    LOG4CXX_INFO(KrisLibrary::logger(),"DisplacementPlanner::AddNewDisplacement: Obstacle picker returned immovable obstacle");
    return -1;    
  }
  if(GenerateDisplacementSample(des.first,des.second,maxTotalCost,obstacleSampleCount))
    return des.first;
  return -1;
}

///Picks an obstacle to sample, and a maximum cost bound for the
///displacement, based on the current roadmap.
///Default implementation picks whether to explore or refine current paths.
pair<int,Real> DisplacementPlanner::PickObstacleToSample(Real maxTotalCost)
{
#if TEST_RANDOM_DISPLACEMENTS
  //do uniform sampling
  return pair<int,Real>(RandInt(displacementSamples.size()),maxTotalCost);
#endif

  pair<int,Real> res;
  if(!pathCovers[1].covers.empty()) { //goal has been reached
    if(Rand() < 0.5) {
      LOG4CXX_INFO(KrisLibrary::logger(),"  Goal reached, refining goal path");
      res = PickGoalRefineObstacle(maxTotalCost);
      if(res.first >= 0) return res;
    }
    if(Rand() < 0.2) {
      LOG4CXX_INFO(KrisLibrary::logger(),"  Goal reached, refining new candidate goal paths");
      res = PickGoalExploreObstacle(maxTotalCost);
      if(res.first >= 0) return res;
    }
    if(Rand()<0.9) {
      LOG4CXX_INFO(KrisLibrary::logger(),"  Goal reached, exploring");
      res = PickExploreObstacle(maxTotalCost);
      if(res.first >= 0) return res;
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"  Goal reached, refining overall");
    return PickRefineObstacle(maxTotalCost);
  }
  if(Rand() < 0.2) {
    LOG4CXX_INFO(KrisLibrary::logger(),"  Goal not reached, refining goal path");
    res = PickGoalExploreObstacle(maxTotalCost);
    if(res.first >= 0) return res;
  }
  //goal has not been reached
  if(Rand() < 0.9) {
    LOG4CXX_INFO(KrisLibrary::logger(),"  Goal not reached, exploring");
    res=PickExploreObstacle(maxTotalCost);
    if(res.first >= 0) return res;
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"  Goal not reached, refining");
  return PickRefineObstacle(maxTotalCost);
}

void DisplacementPlanner::LocalImprovementCandidates(int i,vector<int>& obstacles)
{
  obstacles.resize(0);
  if(pathCovers[i].covers.empty() || pathCovers[i].covers[0]->parent==NULL) {
    //look through infeasible tests
    Subset hitObstacles(displacementSamples.size());
    for(size_t j=0;j<roadmap.nodes[i].tests.size();j++)
      if(space->displacementSpaces[j]!=NULL && !roadmap.nodes[i].tests[j].infeasible.empty())
	hitObstacles.insert_end(j);
    Roadmap::Iterator e;
    for(roadmap.Begin(i,e);!e.end();e++) {
      for(size_t j=0;j<roadmap.nodes[i].tests.size();j++)
	if(space->displacementSpaces[j]!=NULL && !e->tests[j].infeasible.empty())
	  hitObstacles.insert(j);
    }
    obstacles = hitObstacles.items;
  }
  else {
    for(size_t obs=0;obs<displacementSamples.size();obs++) {
      if(space->displacementSpaces[obs]==NULL) continue;
      for(size_t j=0;j<pathCovers[i].covers.size();j++) {
	Assert(pathCovers[i].covers[j]->parent != NULL);
	if(pathCovers[i].covers[j]->assignment[obs] != pathCovers[i].covers[j]->parent->assignment[obs]) {
	  obstacles.push_back(obs);
	  break;
	}
      }
    }
  }
}

bool DisplacementPlanner::IsCandidateForExploration(int i) const
{
  if(!pathCovers[i].covers.empty()) return false;
  for(size_t c=0;c<roadmap.nodes[i].tests.size();c++) {
    if(space->displacementSpaces[c]==NULL) {
      if(!roadmap.nodes[i].tests[c].infeasible.empty()) return false;
      Roadmap::Iterator e;
      for(roadmap.Begin(i,e);!e.end();e++)
	if(!e->tests[c].infeasible.empty()) return false;
    }
  }
  return true;
}

///The explore strategy finds a node that is not reached and finds
///the obstacles that might make the node reached.
pair<int,Real> DisplacementPlanner::PickExploreObstacle(Real maxTotalCost)
{
  vector<int> count_candidate(displacementSamples.size(),0);
  vector<Real> candidate_bound(displacementSamples.size(),0);
  vector<int> candidatelist;
  for(size_t i=0;i<roadmap.nodes.size();i++)
    if(IsCandidateForExploration(i)) {
      LocalImprovementCandidates(i,candidatelist);
      if(i==0) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Root improvement candidates: ");
	for(size_t j=0;j<candidatelist.size();j++)
	  LOG4CXX_INFO(KrisLibrary::logger(),""<<candidatelist[j]);
	LOG4CXX_INFO(KrisLibrary::logger(),"");
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"boundary node "<<i<<" feasible, has "<<candidatelist.size());
      for(size_t j=0;j<candidatelist.size();j++) {
	int obs = candidatelist[j];
	//look through costs of possible parent paths, assignments - obs
	bool foundCandidate = false;
	if(i==0) {
	  //root
	  foundCandidate=true;
	  candidate_bound[obs] = maxTotalCost;
	  count_candidate[obs] ++;
	}
	else {
	  Real closestParent = maxTotalCost;
	  Roadmap::Iterator e;
	  for(roadmap.Begin(i,e);!e.end();e++) {
	    PathSearchNode* n=OptimalPathTo(e.target());
	    if(!n) continue;
	    Real cmin = n->totalCost - displacementSampleCosts[obs][n->assignment[obs]];
	    if(cmin < closestParent) {
	      foundCandidate = true;
	      closestParent = cmin;
	    }
	  }
	  if(foundCandidate) {
	    candidate_bound[obs] = maxTotalCost-closestParent;
	    count_candidate[obs] ++;
	  }
	  else {
	    //LOG4CXX_INFO(KrisLibrary::logger(),"A node "<< but no parent is reachable\n"<<" has an infeasibility at obstacle "<<i);
    }
	}
      }
    }
  /*
  for(size_t i=0;i<candidate_bound.size();i++)
    LOG4CXX_INFO(KrisLibrary::logger(),"Candidate obstacle "<<i<<": count "<<count_candidate[i]<<", bound "<<candidate_bound[i]);
  */
  for(size_t i=0;i<candidate_bound.size();i++)
    if(candidate_bound[i] <= 0)
      count_candidate[i] = 0;
  //Strategy: pick any touched candidate, or prioritize by the count?
  //int i = PickByVote(count_candidate);
  int i = PickByPresence(count_candidate);
  if(i<0) return pair<int,Real>(-1,0);
  return pair<int,Real>(i,candidate_bound[i]);
}

pair<int,Real> DisplacementPlanner::PickGoalExploreObstacle(Real maxTotalCost)
{
  vector<int> count_candidate(displacementSamples.size(),0);
  vector<Real> candidate_bound(displacementSamples.size(),maxTotalCost);
  //look through any reachable neighbors with infeasible edges to goal
  Roadmap::Iterator e;
  bool anyValid = false;
  for(roadmap.Begin(1,e);!e.end();e++) {
    int t=e.target();
    //look for infeasibilities in non-movable obstacles
    bool invalid=false;
    for(size_t i=0;i<displacementSamples.size();i++)
      if(space->displacementSpaces[i] == NULL) {
	if(!e->tests[i].infeasible.empty()) {
	  invalid = true;
	  break;
	}
      }
    if(invalid) continue;
    Real c = OptimalCost(t);
    if(c > maxTotalCost) continue;
    Real len = space->Distance(roadmap.nodes[t].q,roadmap.nodes[1].q);
    c += len*pathCostWeight;
    if(c > maxTotalCost) continue;
    for(size_t i=0;i<displacementSamples.size();i++) {
      if(!space->displacementSpaces[i]) continue;
      if(!roadmap.nodes[t].tests[i].infeasible.empty()) {
	anyValid = true;
	count_candidate[i]++;
	candidate_bound[i] = Min(candidate_bound[i],maxTotalCost-c);
      }
      else if(!e->tests[i].infeasible.empty()) {
	anyValid = true;
	count_candidate[i]++;
	candidate_bound[i] = Min(candidate_bound[i],maxTotalCost-c);
      }
    } 
  }
  if(!anyValid) return pair<int,Real>(-1,0);
  //int i = PickByVote(count_candidate);
  int i = PickByPresence(count_candidate);
  if(i<0) return pair<int,Real>(-1,0);
  return pair<int,Real>(i,candidate_bound[i]);
}

pair<int,Real> DisplacementPlanner::PickRefineObstacle(Real maxTotalCost)
{
  vector<int> count_candidate(displacementSamples.size(),0);
  vector<Real> candidate_bound(displacementSamples.size(),maxTotalCost);
  vector<int> candidatelist;
  for(size_t i=0;i<roadmap.nodes.size();i++)
    if(!pathCovers[i].covers.empty()) {
      LocalImprovementCandidates(i,candidatelist);
      for(size_t j=0;j<candidatelist.size();j++) {
	int obs = candidatelist[j];
	count_candidate[obs] ++;
	candidate_bound[obs] = Min(candidate_bound[obs],displacementSampleCosts[obs][OptimalPathTo(i)->assignment[obs]]);
      }
    }
  //Strategy: pick any touched candidate, or prioritize by the count?
  int i = PickByVote(count_candidate);
  //int i = PickByPresence(count_candidate);
  if(i<0) return pair<int,Real>(-1,0);
  return pair<int,Real>(i,candidate_bound[i]);
}

pair<int,Real> DisplacementPlanner::PickGoalRefineObstacle(Real maxTotalCost)
{
  vector<int> count_candidate(displacementSamples.size(),0);
  vector<Real> candidate_bound(displacementSamples.size(),maxTotalCost);
  PathSearchNode* n = OptimalPathTo(1);
  Assert(n!=NULL);
  while(n->parent != NULL) {
    for(size_t i=0;i<n->assignment.size();i++)
      if(n->assignment[i] != n->parent->assignment[i]) {
	count_candidate[i]++;
	candidate_bound[i] = Min(candidate_bound[i],displacementSampleCosts[i][n->assignment[i]]);
      }
    n = n->parent;
  }
  int i = PickByPresence(count_candidate);
  if(i<0) return pair<int,Real>(-1,0);
  return pair<int,Real>(i,candidate_bound[i]);
}


bool DisplacementPlanner::GenerateDisplacementSample(int obstacle, Real maxDispCost, Real maxTotalCost, int numTries)
{
  Real c0 = (initialDisplacementCosts.empty()?0:initialDisplacementCosts[obstacle]);
  if(c0 >= maxDispCost) {
    LOG4CXX_INFO(KrisLibrary::logger(),"DisplacementPlanner::GenerateDisplacementSample: Cost bound "<<maxDispCost);
    return false;
  }

  vector<int> potential_changes;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    if(roadmap.nodes[i].tests[obstacle].allFeasible==1 || roadmap.nodes[i].tests[obstacle].infeasible.empty()) {
      //TODO: look at infeasibilities on edges eminating for i
      continue;
    }
    int bestold = -1;
    Real bestcost = Inf;
    for(size_t j=0;j<pathCovers[i].covers.size();j++) {
      int oldassignment = pathCovers[i].covers[j]->assignment[obstacle];
      if(displacementSampleCosts[obstacle][oldassignment] < bestcost) {
	bestcost = displacementSampleCosts[obstacle][oldassignment];
	bestold = oldassignment;
      }
    }
    if(c0 < bestcost) {
      //room for improvement
      potential_changes.push_back((int)i);
    }
  }
  if(potential_changes.empty() && pathCovers[0].covers.empty())
    potential_changes.push_back(0);
  if(potential_changes.empty()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Obstacle "<<obstacle<<" potential displacement was irrelevant for all nodes");
    return false;
  }

  //now sample a displacement to lead to an improved path to at least one of
  //those nodes
  LOG4CXX_INFO(KrisLibrary::logger(),"Generating a displacement on obs "<<obstacle<<" to satisfy "<<potential_changes.size());

  //combine # of satisfied node changes + cost of sample
  Real maxRad = maxDispCost-c0;
  Real bestSampleScore = -Inf;
  Vector bestSample;

  auto dspace = space->displacementSpaces[obstacle];
  Vector sample;
  for(int iters=0;iters<numTries;iters++) {
    dspace->SampleNeighborhood(displacementSamples[obstacle][0],maxRad,sample);
    Real dist = dspace->Distance(sample,displacementSamples[obstacle][0]);
    if(dist > maxRad) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, D-space's SampleNeighborhood failed to return a sample with a proper cost, "<<dist<<" > "<<maxRad);
      continue;
    }
    if(!dspace->IsFeasible(sample)) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, D-space's SampleNeighborhood failed to return a feasible sample");
      continue;
    }

    int numfeas = 0;
    //first make sure goal is reachable, then do checks
    if(!space->IsFeasible(roadmap.nodes[1].q,obstacle,sample) || !space->IsFeasible(roadmap.nodes[0].q,obstacle,sample)) {
    }
    else {
      for(size_t i=0;i<potential_changes.size();i++) {
	numConfigChecks++;
	if(space->IsFeasible(roadmap.nodes[potential_changes[i]].q,obstacle,sample)) numfeas++;
      }
    }
    Real score = Real(numfeas) / potential_changes.size() - dist/maxRad;
    //LOG4CXX_INFO(KrisLibrary::logger(),"Sampled candidate feas "<<numfeas<<" dist "<<dist);
    if(score > bestSampleScore) {
      bestSampleScore = score;
      bestSample = sample;
    }

    //try shrinking to get a lower cost displacement
    Real origScore = score;
    Vector dispTemp=sample,dispMid;
    for(int diters=0;diters<obstacleDescendIters;diters++) {
      dspace->Midpoint(displacementSamples[obstacle][0],dispTemp,dispMid);
      Assert(dspace->Distance(dispMid,displacementSamples[obstacle][0]) <= maxRad);
      dist *= 0.5;
      int numfeas = 0;
      if(!dspace->IsFeasible(dispMid)) break;
      if(!space->IsFeasible(roadmap.nodes[1].q,obstacle,dispMid)||!space->IsFeasible(roadmap.nodes[0].q,obstacle,dispMid)) break;
      for(size_t i=0;i<potential_changes.size();i++) {
	numConfigChecks++;
	if(space->IsFeasible(roadmap.nodes[potential_changes[i]].q,obstacle,dispMid)) numfeas++;
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"  Bisect "<<diters<<" candidate feas "<<numfeas<<" dist "<<dist);
      Real score = Real(numfeas) / potential_changes.size() - dist/maxRad;
      if(score > bestSampleScore) {
	bestSampleScore = score;
	bestSample = dispMid;
      }
      else if(score < origScore)
	break;
      dispTemp = dispMid;
    }
  }
  if(IsInf(bestSampleScore )) {
    LOG4CXX_INFO(KrisLibrary::logger(),"No good samples for obs "<<obstacle);
    return false;
  }

  Assert(dspace->Distance(bestSample,displacementSamples[obstacle][0]) <= maxRad);
  LOG4CXX_INFO(KrisLibrary::logger(),"Adding displacement sample to obstacle "<<obstacle<<", dist "<<dspace->Distance(bestSample,displacementSamples[obstacle][0])<<", bound "<<maxRad);
  //add the displacement sample
  AddDisplacementSampleRaw(obstacle,bestSample);

  //clear all covers and do search from scratch
  for(size_t i=0;i<pathCovers.size();i++) {
    pathCovers[i].covers.resize(0);
  }
  UpdateCoversIn(0,Inf);
  UpdateCoversOut(0,Inf);
  return true;
}

bool DisplacementPlanner::AddDisplacementSample(int obstacle,const Vector& disp)
{
  auto dspace = space->displacementSpaces[obstacle];
  Real cost = dspace->Distance(disp,displacementSamples[obstacle][0]);
  if(!initialDisplacementCosts.empty())
    cost += initialDisplacementCosts[obstacle];

  vector<int> potential_changes;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    if(roadmap.nodes[i].tests[obstacle].allFeasible==1) continue;
    if(roadmap.nodes[i].tests[obstacle].infeasible.empty()) continue;
    int bestold = -1;
    Real bestcost = Inf;
    for(size_t j=0;j<pathCovers[i].covers.size();j++) {
      int oldassignment = pathCovers[i].covers[j]->assignment[obstacle];
      if(displacementSampleCosts[obstacle][oldassignment] < bestcost) {
	bestcost = displacementSampleCosts[obstacle][oldassignment];
	bestold = oldassignment;
      }
    }
    //if(bestold < 0) continue;
    if(cost < bestcost) {
      numConfigChecks++;
      if(space->IsFeasible(roadmap.nodes[i].q,obstacle,disp)) {
	potential_changes.push_back((int)i);
      }
    }
  }
  if(potential_changes.empty()) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Obstacle "<<obstacle<<" potential displacement "<<disp<<" was infeasible for all nodes");
    return false;
  }

  //try shrinking to get a better displacement
  Vector dispTemp=disp,dispMid;
  for(int iters=0;iters<obstacleDescendIters;iters++) {
    dspace->Midpoint(displacementSamples[obstacle][0],dispTemp,dispMid);
    if(!dspace->IsFeasible(dispMid)) break;
    bool feasible = true;
    for(size_t i=0;i<potential_changes.size();i++) {
      numConfigChecks++;
      if(!space->IsFeasible(roadmap.nodes[potential_changes[i]].q,obstacle,dispMid)) {
	feasible=false;
	break;
      }
    }
    if(!feasible) break;
    dispTemp = dispMid;
  }

  //add the displacement sample
  int newSampleIndex = (int)displacementSamples[obstacle].size();
  AddDisplacementSampleRaw(obstacle,dispTemp);

  //LOG4CXX_INFO(KrisLibrary::logger(),"New displacement sample, making "<<potential_changes.size());
  //go through all touched nodes to add this sample
  for(size_t i=0;i<potential_changes.size();i++) {
    int node=potential_changes[i];
    roadmap.nodes[node].tests[obstacle].feasible.insert_end(newSampleIndex);
  }
  //update all paths from scratch? or do dynamic shortest paths
  //bool updateAll = (potentialChanges.size() > 100);
  bool updateAll = true;
  for(size_t i=0;i<potential_changes.size();i++) {
    int node=potential_changes[i];
    bool changed=false;
    for(size_t j=0;j<pathCovers[node].covers.size();j++)
      if(CheckUpstreamConstraints(pathCovers[node].covers[j].get(),obstacle,newSampleIndex)) {
        changed = true;
        if(!updateAll) {
          if(pathCovers[node].covers[j]->parent) 
            UpdateEdge(pathCovers[node].covers[j]->parent->vertex,pathCovers[node].covers[j]->vertex,pathCovers[node].covers[j]->parent,Inf);
          else
            FindMinimumAssignment(pathCovers[node].covers[j].get(),Inf);
        }
      }
    if(!updateAll) {
      if(!changed) 
        if(UpdateCoversIn(node,Inf)) changed=true;
      if(changed) {
        UpdateCoversOut(node,Inf);
      }
    }
    //else
    //LOG4CXX_INFO(KrisLibrary::logger(),"Node "<<node<<" ("<<pathCovers[node].covers.size());
  }
  if(updateAll) {
    //clear all covers and do search from scratch
    Real goalCost = OptimalCost(1);
    for(size_t i=0;i<pathCovers.size();i++) {
      pathCovers[i].covers.resize(0);
    }
    UpdateCoversIn(0,Inf);
    UpdateCoversOut(0,Inf);
    if(!IsInf(goalCost)) {
      if(OptimalCost(1) > goalCost)
		LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: optimal cost to goal was increased! "<<OptimalCost(1)<<" > "<<goalCost);
      //Assert(OptimalCost(1) <= goalCost);
    }
  }
  return true;
}

void DisplacementPlanner::AddDisplacementSampleRaw(int obstacle,const Vector& disp)
{
  Real cost = space->displacementSpaces[obstacle]->Distance(disp,displacementSamples[obstacle][0]);
  if(!initialDisplacementCosts.empty())
    cost += initialDisplacementCosts[obstacle];

  displacementSamples[obstacle].push_back(disp);
  displacementSampleCosts[obstacle].push_back(cost);
  //get monotonic ordering
  vector<pair<Real,int> > sorter(displacementSampleCosts[obstacle].size());
  for(size_t i=0;i<displacementSampleCosts[obstacle].size();i++) {
    sorter[i].first = displacementSampleCosts[obstacle][i];
    sorter[i].second = (int)i;
  }
  sort(sorter.begin(),sorter.end());
  displacementSampleOrders[obstacle].resize(sorter.size());
  for(size_t i=0;i<sorter.size();i++)
    displacementSampleOrders[obstacle][i] = sorter[i].second;
}

bool DisplacementPlanner::RefineGoalDisplacements(int numIters,Real perturbRadiusFrac)
{
  PathSearchNode* n=OptimalPathTo(1);
  if(!n) return false;
  vector<Real> curCost(displacementSamples.size());
  vector<Vector> optima(displacementSamples.size());
  for(size_t i=0;i<n->assignment.size();i++) {
    curCost[i] = displacementSampleCosts[i][n->assignment[i]];
    optima[i] = displacementSamples[i][n->assignment[i]];
  }
  vector<int> path;
  GetPath(n,path);
  vector<bool> changed(displacementSamples.size(),false);
  Vector temp;
  for(size_t i=0;i<n->assignment.size();i++) {
    if(curCost[i] <= 0) continue;
    for(int iters=0;iters<numIters;iters++) {
      Real c0 = (initialDisplacementCosts.empty()? 0 : initialDisplacementCosts[i]);
      Real r = (curCost[i]-c0)*perturbRadiusFrac;
      space->displacementSpaces[i]->SampleNeighborhood(optima[i],r,temp);
      //test for a decrease in cost
      if(space->displacementSpaces[i]->Distance(temp,displacementSamples[i][0]) > curCost[i]-c0) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Sample for obstacle "<<i);
	continue;
      }
      if(!space->displacementSpaces[i]->IsFeasible(temp)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Sample for obstacle "<<i);
	continue;
      }

      //test obstacle i, displacement temp for all nodes along the path
      bool feasible = true;
      for(size_t v=0;v<path.size();v++) {
	if(roadmap.nodes[path[v]].tests[i].allFeasible==1) continue;
	numConfigChecks++;
	if(!space->IsFeasible(roadmap.nodes[path[v]].q,i,temp)) {
	  feasible = false;
	  break;
	}
      }
      if(!feasible) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Displacement sample "<<i); LOG4CXX_INFO(KrisLibrary::logger(),temp<<" vert infeasible");
	continue;
      }
      for(size_t v=0;v+1<path.size();v++) {
	Edge* e=roadmap.FindEdge(path[v],path[v+1]);
	if(e->tests[i].allFeasible==1) continue;
	numEdgeChecks++;
	if(!IsVisible(space,e->e->Start(),e->e->End(),i,temp)) {
	  feasible = false;
	  break;
	}
      }
      if(!feasible) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Displacement sample "<<i); LOG4CXX_INFO(KrisLibrary::logger(),temp<<" edge infeasible");
	continue;
      }
      optima[i] = temp;
      changed[i] = true;
      curCost[i] = space->displacementSpaces[i]->Distance(temp,displacementSamples[i][0])+c0;
      LOG4CXX_INFO(KrisLibrary::logger(),"Displacement sample "<<i);
    }
  }
  bool anyChanged = false;
  for(size_t i=0;i<changed.size();i++) {
    if(changed[i]) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Cost for obstacle "<<i<<" changed to "<<curCost[i]);
      anyChanged=true;
      AddDisplacementSampleRaw(i,optima[i]);

      //mark all tests for the optimum feasible
      for(size_t v=0;v<path.size();v++)
	roadmap.nodes[path[v]].tests[i].feasible.insert_end(displacementSamples[i].size()-1);
      for(size_t v=0;v+1<path.size();v++) {
	Edge* e = roadmap.FindEdge(path[v],path[v+1]);
	Assert(e != NULL);
	e->tests[i].feasible.insert_end(displacementSamples[i].size()-1);
      }
    }
  }
  if(anyChanged) {
    //update all paths
    for(size_t i=0;i<pathCovers.size();i++) {
      pathCovers[i].covers.resize(0);
    }
    UpdateCoversIn(0,Inf);
    UpdateCoversOut(0,Inf);
  }
  return anyChanged;
}

bool DisplacementPlanner::RefineGoalPathAndDisplacements(int numIters,Real perturbRadiusFrac,Real lipschitzDisp)
{
  PathSearchNode* n=OptimalPathTo(1);
  if(!n) return false;
  vector<Real> curCost(displacementSamples.size());
  vector<Vector> optima(displacementSamples.size());
  for(size_t i=0;i<n->assignment.size();i++) {
    curCost[i] = displacementSampleCosts[i][n->assignment[i]];
    optima[i] = displacementSamples[i][n->assignment[i]];
  }
  vector<int> path;
  GetPath(n,path);
  Real curLen = n->pathLength;
  vector<bool> pathChanged(path.size(),false);
  vector<Config> pathConfigs(path.size());
  for(size_t i=0;i<path.size();i++)
    pathConfigs[i] = roadmap.nodes[path[i]].q;
  vector<Vector> temps(path.size());
  vector<bool> changed(displacementSamples.size(),false);
  for(size_t i=0;i<n->assignment.size();i++) {
    if(curCost[i] <= 0) continue;
    for(int iters=0;iters<numIters;iters++) {
      Real c0 = (initialDisplacementCosts.empty()? 0 : initialDisplacementCosts[i]);
      Real r = (curCost[i]-c0)*perturbRadiusFrac;
      Vector temp;
      space->displacementSpaces[i]->SampleNeighborhood(optima[i],r,temp);
      //test for a decrease in cost
      Real c=space->displacementSpaces[i]->Distance(temp,displacementSamples[i][0])+c0;
      if(c >= curCost[i]) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Sample for obstacle "<<i);
	continue;
      }
      if(!space->displacementSpaces[i]->IsFeasible(temp)) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Sample for obstacle "<<i);
	continue;
      }

      //test obstacle i, displacement temp for all nodes along the path
      int numActive=0;
      vector<bool> activeVertices(path.size(),false);
      for(size_t v=0;v<path.size();v++) {
	numConfigChecks++;
	if(!space->IsFeasible(pathConfigs[v],i,temp)) {
	  activeVertices[v] = true;
	  numActive++;
	}
      }
      //made the start or goal infeasible
      if(activeVertices[0] || activeVertices.back()) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Displacement sample "<<i); LOG4CXX_INFO(KrisLibrary::logger(),temp<<" made start/goal infeasible");
	continue;
      }
      for(size_t v=0;v+1<path.size();v++) {
	if(activeVertices[v] || activeVertices[v+1]) continue;
	numEdgeChecks++;
	if(!IsVisible(space,pathConfigs[v],pathConfigs[v+1],i,temp)) {
	  activeVertices[v] = activeVertices[v+1] = true;
	  numActive++;
	}
      }

      if(numActive==0) { //just accept the sample
	optima[i] = temp;
	changed[i] = true;
	curCost[i] = space->displacementSpaces[i]->Distance(temp,displacementSamples[i][0])+c0;
	//LOG4CXX_INFO(KrisLibrary::logger(),"Displacement sample "<<i);
	continue;
      }

      //perturb configs
      int numPathSamples = gNumOptimizePathSamples;  //TODO: make this a parameter
      bool vertsfeas = true;
      Real movedist = space->displacementSpaces[i]->Distance(temp,optima[i]);
      for(size_t v=0;v<path.size();v++) {
	if(!activeVertices[v]) continue;
	temps[v].clear();
	bool feas = false;
	for(int k=0;k<numPathSamples;k++) {
	  space->SampleNeighborhood(pathConfigs[v],2.0*movedist*lipschitzDisp,temps[v]);
	  //check feasibility w.r.t to the current disp
	  if(space->IsFeasible(temps[v],i,temp)) {
	    bool allfeas = true;
	    for(size_t j=0;j<n->assignment.size();j++) {
	      if(j==i) continue;
	      if(!space->IsFeasible(temps[v],j,optima[j])) {
		allfeas = false;
		break;
	      }
	    }
	    if(allfeas) {
	      feas = true;
	      break;
	    }
	  }
	}
	if(!feas) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Obs "<<i<<": path sample "<<v);
	  vertsfeas = false;
	  break;
	}
      }
      if(!vertsfeas) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Obs "<<i);
	continue;
      }
      for(size_t v=0;v<path.size();v++) 
	if(!activeVertices[v]) {
	  temps[v].clear();
	  temps[v].setRef(pathConfigs[v]);
	}
      //check path length constraint
      Real newlen = 0;
      for(size_t v=0;v+1<path.size();v++)
	newlen += space->Distance(temps[v],temps[v+1]);
      if((newlen - curLen)*pathCostWeight >= curCost[i] - c) { //path too long
	//LOG4CXX_INFO(KrisLibrary::logger(),"Obs "<<i);
	continue;
      }
      //made a candidate displacement: check feasibility w.r.t to other
      //obstacles
      for(size_t v=1;v+1<path.size();v++) {
	if(activeVertices[v]) {
	  bool feasible = true;
	  for(size_t j=0;j<n->assignment.size();j++) {
	    if(j==i) continue;
	    if(!space->IsFeasible(temps[v],j,optima[j])) {
	      feasible = false;
	      break;
	    }
	  }
	  if(!feasible) vertsfeas = false;
	}
      }
      if(!vertsfeas) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Obs "<<i);
	continue;
      }
      //now check edge feasibility
      for(size_t v=0;v+1<path.size();v++) {
	if(activeVertices[v] || activeVertices[v+1]) {
	  if(!IsVisible(space,temps[v],temps[v+1],i,temp)) {
	    vertsfeas = false;
	    break;
	  }
	}
      }
      if(!vertsfeas) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Obs "<<i);
	continue;
      }
      //check other edges
      for(size_t v=0;v+1<path.size();v++) {
	if(activeVertices[v] || activeVertices[v+1]) {
	  for(size_t j=0;j<n->assignment.size();j++) {
	    if(i==j) continue;
	    if(!IsVisible(space,temps[v],temps[v+1],j,optima[j])) {
	      vertsfeas = false;
	      break;
	    }
	  }
	}
      }
      if(!vertsfeas) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Obs "<<i);
	continue;
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"Path successfully changed");
      //we are done.  now, store this in the current optimization variable
      for(size_t v=0;v<path.size();v++)
	if(activeVertices[v])
	  pathChanged[v] = true;
      pathConfigs = temps;
      curLen = newlen;
      optima[i] = temp;
      changed[i] = true;
      curCost[i] = space->displacementSpaces[i]->Distance(temp,displacementSamples[i][0])+c0;
    }
  }
  //add the new edges for the path
  for(size_t i=1;i+1<path.size();i++) {
    if(pathChanged[i]) {
      int j=AddNode(pathConfigs[i],path[i-1]);
      path[i] = j;
    }
    else if(pathChanged[i-1]) {
      AddEdge(path[i-1],path[i]);
    }
  }
  if(pathChanged[path.size()-2]) 
    AddEdge(path[path.size()-2],1);
  bool anyChanged = false;
  for(size_t i=0;i<changed.size();i++) {
    if(changed[i]) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Cost for obstacle "<<i<<" changed to "<<curCost[i]);
      anyChanged=true;
      AddDisplacementSampleRaw(i,optima[i]);
      //mark all tests for the optimum feasible
      for(size_t v=0;v<path.size();v++)
	roadmap.nodes[path[v]].tests[i].feasible.insert_end(displacementSamples[i].size()-1);
      for(size_t v=0;v+1<path.size();v++) {
	Edge* e = roadmap.FindEdge(path[v],path[v+1]);
	Assert(e != NULL);
	e->tests[i].feasible.insert_end(displacementSamples[i].size()-1);
      }
    }
  }

  if(anyChanged) {
    //update all paths
    for(size_t i=0;i<pathCovers.size();i++) {
      pathCovers[i].covers.resize(0);
    }
    UpdateCoversIn(0,Inf);
    UpdateCoversOut(0,Inf);
  }
  return anyChanged;
}


bool DisplacementPlanner::ShortcutGoalPath(int skip,int numIters)
{
  PathSearchNode* n=OptimalPathTo(1);
  if(!n) return false;
  if(skip <= 0) return false;
  vector<int> path;
  GetPath(n,path);
  int numChanges = 0;
  for(int iters=0;iters<numIters;iters++) {
    //n->assignment is feasible at all path vertices, so just check the 
    //edges
    if(skip+1>= (int)path.size()) break;
    int origChanges = numChanges;
    for(size_t i=0;i+skip+1<path.size();i++) {
      size_t j=i+skip+1;
      if(roadmap.HasEdge(path[i],path[j])) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Shortcut "<<path[i]<<"->"<<path[j]);
	continue;
      }
      bool feasible = true;
      for(size_t k=0;k<displacementSamples.size();k++)
	if(!IsVisible(space,roadmap.nodes[path[i]].q,roadmap.nodes[path[j]].q,k,displacementSamples[k][n->assignment[k]])) {  
	  feasible = false;
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Shortcut "<<i<<"->"<<j<<" failed feasibility check "<<k<<" = "<<n->assignment[k]);
	  break;
	}
      //else 
      //LOG4CXX_INFO(KrisLibrary::logger(),"Shortcut "<<i<<"->"<<j<<" satisfied feasibility check "<<k<<" = "<<n->assignment[k]);
      if(!feasible) continue;
      TestResults tests(displacementSamples.size());
      for(size_t k=0;k<displacementSamples.size();k++) {
	tests[k].allFeasible = -1;
	tests[k].feasible.maxItem = INT_MAX;
	tests[k].infeasible.maxItem = INT_MAX;
	tests[k].feasible.insert_end(n->assignment[k]);
      }
      AddEdge(path[i],path[j],tests);
      numChanges++;
    }
    if(origChanges == numChanges) break;
  }
  if(numChanges==0) return false;

  //now update all paths
  for(size_t i=0;i<pathCovers.size();i++) {
    pathCovers[i].covers.resize(0);
  }
  UpdateCoversIn(0,Inf);
  UpdateCoversOut(0,Inf);
  return true;
}


void DisplacementPlanner::Expand(Real maxExplanationCost,vector<int>& newNodes)
{
  numExpands++;
#if DO_TIMING
  Timer timer;
#endif //DO_TIMING

  newNodes.resize(0);
  Config q;
  if(RandBool(goalBiasProbability))
    q = goal;
  else if(dynamicDomainExpansion) {
    //dynamic domain RRT-style
    Config center = roadmap.nodes[RandInt(roadmap.nodes.size())].q;
    //Real rad = space->Distance(roadmap.nodes[0].q,roadmap.nodes[1].q);
    Real rad = expandDistance;
    for(size_t i=0;i<roadmap.nodes.size();i++)
      if(OptimalCost(i) < maxExplanationCost) {
	rad = Max(rad,space->Distance(center,roadmap.nodes[i].q));
	if(rad > expandDistance*2) {
	  rad = expandDistance*2;
	  break;
	}
      }
    space->SampleNeighborhood(center,rad,q);
  }
  else
    space->Sample(q);
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

  if(neighbor.empty()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Expand(): no vertices reachable within the cost limit "<<maxExplanationCost);
    return;
  }

  //attempt connections
  bool didRefine = false;
  newNodes.resize(0);
  if(closest[0] <= expandDistance) {
    numRefinementAttempts++;
    
    //do a direct connection
    int n=ExtendEdge(neighbor[0],q);
    if(n >= 0) {
      numRefinementSuccesses++;
      newNodes.push_back(n); 
    }
  }
#if DO_TIMING
  timeRefine += timer.ElapsedTime();
  timer.Reset();
#endif

  if(closest[0] > expandDistance) {
    //now try to connect a short extension
    numExplorationAttempts++;

    int n=neighbor[0];
    //do an RRT-style extension
    Real u=expandDistance/closest[0];
    Config qu;
    space->Interpolate(roadmap.nodes[n].q,q,u,qu);
    int res=ExtendEdge(n,qu);
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

    vector<Real> kclosest;
    vector<int> kneighbors;
    KNN(roadmap.nodes[n].q,maxExplanationCost,kmax,kneighbors,kclosest);
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
      //check if the hypothetical path to this node is within the limit?
      AddEdge(kneighbors[j],n);
      didRefine = true;
      numadded++;
      if(numadded == kmax) break;
    }
  }

#if DO_TIMING
  timeRefine += timer.ElapsedTime();
  timer.Reset();
#endif

  for(size_t i=0;i<newNodes.size();i++) {
    UpdateCoversIn(newNodes[i],maxExplanationCost);
  }
#if DO_TIMING
  timeUpdateCoversIn += timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  if(didRefine) {
    for(size_t i=0;i<newNodes.size();i++) {
      UpdateCoversOut(newNodes[i],maxExplanationCost);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Expand: New node "<<newNodes[i]<<" cost "<<OptimalCost(newNodes[i]));
    }
#if DO_TIMING
    timeUpdateCoversOut += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING
  }

  if(!bidirectional) {
    didRefine = false;
    for(size_t i=0;i<newNodes.size();i++) {
      if(pathCovers[newNodes[i]].covers.empty()) continue;
      Real d=space->Distance(goal,roadmap.nodes[newNodes[i]].q);
      if(d > goalConnectThreshold)
	continue;
      if(roadmap.HasEdge(1,newNodes[i])) 
	continue;
      if(OptimalCost(newNodes[i]) + d*pathCostWeight >= maxExplanationCost) 
	continue;
      //test all non-movable constraints
      //look for infeasibilities in non-movable obstacles
      bool invalid=false;
      for(size_t c=0;c<displacementSamples.size();c++)
	if(space->displacementSpaces[c] == NULL) 
	  if(!IsVisible(space,goal,roadmap.nodes[newNodes[i]].q,c)) {
	    invalid = true;
	    break;
	  }
      if(!invalid) {
	AddEdge(1,newNodes[i]);
	didRefine = true;
      }
    }
#if DO_TIMING
  timeRefine += timer.ElapsedTime();
  timer.Reset();
#endif

  if(didRefine) {
    //eventually the goal has lots of covers leading into it
    //UpdateCoversIn(1,maxExplanationCost);
    for(size_t i=0;i<newNodes.size();i++) {
      if(!roadmap.HasEdge(newNodes[i],1)) continue;
      for(size_t j=0;j<pathCovers[newNodes[i]].covers.size();j++) {
	UpdateEdge(newNodes[i],1,pathCovers[newNodes[i]].covers[j].get(),maxExplanationCost);
      }
    }
  }

#if DO_TIMING
  timeUpdateCoversIn += timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING
  }
  else
    FatalError("Can't do bidirectional planning yet");

  if(DEBUG) Assert(SanityCheck());
}

void DisplacementPlanner::BuildRoadmap(Real maxTotalCost,RoadmapPlanner& prm)
{
  //copy into a traditional roadmap
  RoadmapPlanner::Roadmap copy;
  copy.Resize(roadmap.nodes.size());
  Graph::CopyStructure(roadmap,copy);
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    copy.nodes[i].setRef(roadmap.nodes[i].q);
    for(Roadmap::EdgeListIterator e=roadmap.edges[i].begin();e!=roadmap.edges[i].end();e++) {
      shared_ptr<EdgePlanner>* ecopy = copy.FindEdge(i,e->first);
      Assert(ecopy != NULL);
      *ecopy = e->second->e;
    }
  }
  //get the reachable subgraph
  vector<int> reachable;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    if(OptimalCost(i) <= maxTotalCost)
      reachable.push_back(i);
  }
  Graph::GetSubGraph(copy,reachable,prm.roadmap);
}

int DisplacementPlanner::AddNode(const Config& q,int parent)
{
#if DO_TIMING
  Timer timer;
#endif
  int index=(int)roadmap.nodes.size();
  roadmap.AddNode(Milestone());
  roadmap.nodes[index].q = q;
  int n=space->NumConstraints();
  //initialize empty test results
  roadmap.nodes[index].tests.resize(n);
  for(int i=0;i<n;i++) {
    roadmap.nodes[index].tests[i].allFeasible = -1;
    roadmap.nodes[index].tests[i].feasible.maxItem = INT_MAX;
    roadmap.nodes[index].tests[i].feasible.items.clear();
    roadmap.nodes[index].tests[i].infeasible.maxItem = INT_MAX;
    roadmap.nodes[index].tests[i].infeasible.items.clear();
  }

  //add path cover structure
  pathCovers.resize(roadmap.nodes.size());

  if(parent >=0) {
    AddEdge(parent,index);
  }

#if DO_TIMING
  timeOverhead += timer.ElapsedTime();
#endif
  return index;
}

int DisplacementPlanner::CheckImmovableAndAddNode(const Config& q,int parent)
{
  //check non-movable constraints
  for(size_t c=0;c<displacementSamples.size();c++) 
    if(space->displacementSpaces[c]==NULL) {
      numConfigChecks++;
      if(!space->IsFeasible(q,c)) return -1;
    }
  for(size_t c=0;c<displacementSamples.size();c++)
    if(space->displacementSpaces[c]==NULL) {
      Vector blah;
      numEdgeChecks++;
      if(!IsVisible(space,roadmap.nodes[parent].q,q,c,blah)) return -1;
    }
  //passed tests, add the node
  int j = AddNode(q,parent);
  //cache the feasibility tests
  Edge* e=roadmap.FindEdge(j,parent);
  for(size_t c=0;c<displacementSamples.size();c++)
    if(space->displacementSpaces[c]==NULL) {
      roadmap.nodes[j].tests[c].allFeasible = 1;
      e->tests[c].allFeasible = 1;
    }
  return j;
}

int DisplacementPlanner::ExtendEdge(int i,const Config& q)
{
  int res=CheckImmovableAndAddNode(q,i);
  if(res >= 0) return res;
  Vector qtemp=q,qmid;
  for(int iters=0;iters<gMaxExtendBisectIters;iters++) {
    space->Midpoint(roadmap.nodes[i].q,qtemp,qmid);
    res = CheckImmovableAndAddNode(qmid,i);
    if(res >= 0) {
      return res;
    }
    qtemp = qmid;
  }
  return -1;
}


void DisplacementPlanner::AddEdge(int i,int j)
{
  assert(i != j);
  assert(i >= 0 && i < (int)roadmap.nodes.size());
  assert(j >= 0 && j < (int)roadmap.nodes.size());
  assert(!roadmap.HasEdge(i,j));
  TestResults ev;
  int n=space->NumConstraints();
  ev.resize(n);
  for(int c=0;c<n;c++) {
    ev[c].allFeasible = -1;
    ev[c].feasible.maxItem = INT_MAX;
    ev[c].feasible.items.clear();
    ev[c].infeasible.maxItem = INT_MAX;
    ev[c].infeasible.items.clear();
  }
  AddEdge(i,j,ev);
}

void DisplacementPlanner::AddEdge(int i,int j,const TestResults& tests)
{
  Edge e;
  e.e = space->LocalPlanner(roadmap.nodes[i].q,roadmap.nodes[j].q);
  e.tests = tests;
  roadmap.AddEdge(i,j,e);
}

void DisplacementPlanner::KNN(const Config& q,int k,vector<int>& neighbors,vector<Real>& distances)
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
      if(d < dmax) {
	pair<Real,int> idx(d,i);
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

void DisplacementPlanner::KNN(const Config& q,Real maxTotalCost,int k,vector<int>& neighbors,vector<Real>& distances)
{
  //int maxIdx=0;
  distances.resize(0);
  neighbors.resize(0);
  distances.reserve(k);
  neighbors.reserve(k);
  set<pair<Real,int> > knn;
  Real dmax = Inf;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    if(OptimalCost(i) < maxTotalCost) {
      Real d=space->Distance(roadmap.nodes[i].q,q);
      if(d < connectThreshold) {
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

double DisplacementPlanner::Cost(const vector<int>& assignment) const
{
  double sumCost = 0;
  for(size_t i=0;i<assignment.size();i++)
    sumCost += displacementSampleCosts[i][assignment[i]];
  return sumCost;
}

bool DisplacementPlanner::CheckNodeConstraint(Milestone& v,int i,int j)
{
  if(j == -1) { //indicate all-feasible check
    if(v.tests[i].allFeasible == -1) {
      numConfigChecks++;
      v.tests[i].allFeasible = (space->IsFeasibleAll(v.q,i)?1:0);
    }
    return (v.tests[i].allFeasible == 1);
  }
  //normal check
  if(v.tests[i].allFeasible == 1) return true;
  else if(v.tests[i].feasible.count(j) == 1) return true;
  else if(v.tests[i].infeasible.count(j) == 1) return false;
  //untested
  numConfigChecks++;
  if(space->IsFeasible(v.q,i,displacementSamples[i][j])) {
    v.tests[i].feasible.insert(j);
    return true;
  }
  else {
    //if(i != 0) LOG4CXX_INFO(KrisLibrary::logger(),"Found infeasible vertex for obstacle "<<i<<", index "<<j);
    v.tests[i].infeasible.insert(j);
    return false;
  }
}

bool DisplacementPlanner::CheckEdgeConstraint(Edge& e,int i,int j)
{
  if(j == -1) { //indicate all-feasible check
    if(e.tests[i].allFeasible == -1) {
      numEdgeChecks++;
      e.tests[i].allFeasible = (space->IsVisibleAll(e.e->Start(),e.e->End(),i)?1:0);
    }
    return (e.tests[i].allFeasible == 1);
  }
  //normal check
  if(e.tests[i].allFeasible == 1) return true;
  else if(e.tests[i].feasible.count(j) == 1) return true;
  else if(e.tests[i].infeasible.count(j) == 1) return false;
  //untested
  numEdgeChecks++;
  if(IsVisible(space,e.e->Start(),e.e->End(),i,displacementSamples[i][j])) {
    e.tests[i].feasible.insert(j);
    return true;
  }
  else {
    //if(i != 0) LOG4CXX_INFO(KrisLibrary::logger(),"Found infeasible edge for obstacle "<<i<<", index "<<j);
    e.tests[i].infeasible.insert(j);
    return false;
  }
}




/*
  pathCovers stores all candidate paths passing through each
  node.  By a "candidate path" we mean a path to n with a cover for
  which there is no other path that is pareto-larger.

  Cover updates are called when:
  1. the start and goal nodes are added to the graph (disembodied nodes)
  2. the graph is extended from p to a new node n
  3. a new edge (i,j) is added to the graph
  4. a new displacement sample is added, which could possibly affect some
     subset of nodes n1,...,nk

  Case 1: goal node has no cover. Start node begins with the optimal
  assignment.

  Case 2: The optimal path to n needs to be computed. 
  If the constraints from p->n are compatible with the optimal path
  to p, then the optimal path to the parent is also optimal to n.
  Otherwise, we need to search for a new candidate solution path to p
  that is compatible with a feasible partial setting along from p->n.
  If the setting is compatible with *some* candidate path at p,
  can just extend that candidate path to n.  So it makes sense to look at the
  settings already feasible at p to get a quick upper bound.
  Otherwise, we may need to construct a completely new candidate path at p
  that is explicitly compatible with the setting. 

  For each obstacle, find the next best constraint setting, and perform a
  backward search from p along its optimal path.

  Case 3: The edge may improve a solution path passing through i->j or j->i.
  A completely new candidate path can be constructed! 

  Go through all path search nodes at i, propagate foward to j.  
  If any are pareto, optimal add them.  If it is found that existing nodes
  are pareto suboptimal, update/delete them down-stream.

  Do the same from j to i.
  
  Case 4: Do the upstream update for all paths leading to those affected
  nodes.  If changed, propagate downstream.
 */


bool DisplacementPlanner::CheckUpstreamConstraints(PathSearchNode* node,int obstacle,int setting)
{
  //check vertices first
  PathSearchNode* node0 = node;
  while(node != NULL) {
    if(!CheckNodeConstraint(roadmap.nodes[node->vertex],obstacle,setting)) {
      return false;
    }
    node = node->parent;
  }
  //now check edges
  node = node0;
  while(node->parent != NULL) {
    int s = node->parent->vertex;
    int t = node->vertex;
    Edge* e = roadmap.FindEdge(s,t);
    Assert(e != NULL);
    if(!CheckEdgeConstraint(*e,obstacle,setting)) {
      return false;
    }
    node = node->parent;
  }
  return true;
}

bool DisplacementPlanner::Dominates(PathSearchNode* a,PathSearchNode* b)
{
  if(updatePathsComplete)
    return ParetoDominates(a->assignment,b->assignment);
  else
    return a->totalCost <= b->totalCost;
}

bool DisplacementPlanner::Revisited(PathSearchNode* n)
{
  int i=n->vertex;
  if(updatePathsComplete) {
    for(size_t j=0;j<pathCovers[i].covers.size();j++) {
      //if assignment is pareto-dominated then it should be discarded
      if(ParetoDominates(pathCovers[i].covers[j]->assignment,n->assignment)) {
	return true;
      }
    }
    return false;
  }
  else {
    return (n->totalCost >= OptimalCost(i));
  }
}

shared_ptr<DisplacementPlanner::PathSearchNode> DisplacementPlanner::UpdateEdge(int s,int t,PathSearchNode* ns,Real maxTotalCost)
{
  Assert(ns->vertex==s);
  if(DEBUG) 
    Assert(roadmap.FindEdge(s,t) != NULL);
  //create new candidate child node
  shared_ptr<PathSearchNode> c(new PathSearchNode);
  c->vertex = t;
  c->parent = ns;
  c->assignment = ns->assignment;
  Real dst=space->Distance(roadmap.nodes[s].q,roadmap.nodes[t].q);
  c->pathLength = ns->pathLength + dst;
  c->totalCost = ns->totalCost + dst*pathCostWeight;
  //adjust its assignment (propagating upstream)
  if(!FindMinimumAssignment(c.get(),maxTotalCost)) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Unable to find feasible assignment "<<s<<"->"<<t<<" under cost "<<maxTotalCost);
    //LOG4CXX_INFO(KrisLibrary::logger(),"  no edge update "<< fail assignment\n"<<"->"<<s);
    return NULL;
  }
  c->totalCost = Cost(c->assignment) + c->pathLength*pathCostWeight;

  //check for revisited states
  bool revisited = Revisited(c.get());
  if(revisited) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"  no edge update "<< revisited\n"<<"->"<<s);
    return NULL;
  }
  //if(t==1) 
  //LOG4CXX_INFO(KrisLibrary::logger(),"New path to goal has displacement cost "<<Cost(c->assignment)<<", total "<<c->totalCost);

  //allow intermediate between greedy and optimal
  if((int)pathCovers[t].covers.size()+1>updatePathsMax) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"  no edge update "<< too many paths\n"<<"->"<<s;
    return NULL;
  }

  //add to the covers at t
  pathCovers[t].covers.push_back(c);
  return c;
}

void DisplacementPlanner::PropagateDownstream(PathSearchNode* n,vector<shared_ptr<PathSearchNode> >& nodes,Real maxTotalCost)
{
  Roadmap::Iterator e;
  int s=n->vertex;
  nodes.resize(0);
  for(roadmap.Begin(s,e);!e.end();e++) {
    int t=e.target();
    shared_ptr<PathSearchNode> newNode = UpdateEdge(s,t,n,maxTotalCost);
    if(DEBUG) Assert(SanityCheck());

    if(newNode) {
      Assert(newNode->vertex == t);
      nodes.push_back(newNode);
    }
  }
}

bool DisplacementPlanner::FindMinimumAssignment(PathSearchNode* n,Real maxTotalCost)
{
  //test for compatibility
  vector<int> infeasible;
  for(size_t i=0;i<n->assignment.size();i++) {
    if(!CheckNodeConstraint(roadmap.nodes[n->vertex],i,n->assignment[i])) {
      infeasible.push_back(i);
    }
    else if(n->parent) {
      //check edge
      Edge* e=roadmap.FindEdge(n->vertex,n->parent->vertex);
      Assert(e != NULL);
      if(!CheckEdgeConstraint(*e,i,n->assignment[i]))
	infeasible.push_back(i);
    }
  }
  //existing assignment is compatible
  if(infeasible.empty()) return true;

  //minor branching enhancement
  //ensure that candidates have potential optimality at n->vertex
  //pareto optimal means that at least one of bestcosts must be improved upon
  double bestCost = maxTotalCost;
  vector<double> bestCosts;
  if(!updatePathsComplete) {
    bestCost = Min(bestCost,OptimalCost(n->vertex));
  }
  else {
    bestCosts.resize(infeasible.size(),Inf);
    for(size_t j=0;j<pathCovers[n->vertex].covers.size();j++)
      for(size_t i=0;i<infeasible.size();i++)
	bestCosts[i] = Min(bestCosts[i],displacementSampleCosts[infeasible[i]][pathCovers[n->vertex].covers[j]->assignment[infeasible[i]]]);
  }
  
  //check for new settings
  Real sumCost = n->totalCost;
  //start with sum of cost up to the feasible obstacles
  for(size_t i=0;i<infeasible.size();i++)
    sumCost -= displacementSampleCosts[infeasible[i]][n->assignment[infeasible[i]]];
  vector<int> origAssignment = n->assignment;
  bool paretoImprovement = false;
  for(size_t i=0;i<infeasible.size();i++) {
    int obs = infeasible[i];
    int setting = n->assignment[obs];
    Real origcost = displacementSampleCosts[obs][setting];
    bool found = false;
    int numBranched = 0, numFailedUpstream = 0;
    for(vector<int>::iterator j=displacementSampleOrders[obs].begin();j!=displacementSampleOrders[obs].end();j++) {
      int nextSetting = *j;
      //TODO: store all settings infeasible at path from parent for easier branching?
      if(setting == nextSetting) continue;
      //better branching
      if(sumCost + displacementSampleCosts[obs][nextSetting] >= bestCost) {
	numBranched++;
	continue;
      }
      if(updatePathsComplete) {
	//branch only on the last assignment if it isn't a pareto improvement
	if(i+1==infeasible.size() && !paretoImprovement &&  displacementSampleCosts[obs][nextSetting] >= bestCosts[i]) {
	  numBranched++;
	  continue;
	}
      }

      //here are the guts
      if(CheckUpstreamConstraints(n,obs,nextSetting)) {
	n->assignment[obs] = nextSetting;
	found = true;
	break;
      }
      else
	numFailedUpstream++;
    }
    if (!found) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"  Failed finding assignment to obs "<<obs<<" vertex "<<n->vertex);
      //LOG4CXX_INFO(KrisLibrary::logger(),"  "<<numBranched<<" branched, "<<numFailedUpstream<<" failed of "<<displacementSamples[obs].size());
      return false;
    }
    Real newcost = displacementSampleCosts[obs][n->assignment[obs]];
    if(!updatePathsComplete) 
      sumCost += newcost;
    else if(newcost < bestCosts[i])
      paretoImprovement = true;
  }
  if(n->vertex==1 && !infeasible.empty()) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Found a path to goal, sumcost "<<sumCost);
    /*
    for(size_t i=0;i<infeasible.size();i++)
      LOG4CXX_INFO(KrisLibrary::logger(),"  assignment["<<infeasible[i]<<"] "<<origAssignment[infeasible[i]]<<" -> "<<n->assignment[infeasible[i]]);
    */
  }
  return true;
}

void DisplacementPlanner::PruneSearchNode(PathSearchNode* n,IndexedPriorityQueue<PathSearchNode*,Real>* q)
{
  int v=n->vertex;
  //need to erase all downstream nodes
  Roadmap::Iterator e;
  for(roadmap.Begin(v,e);!e.end();e++) {
    int t=e.target();
    for(size_t j=0;j<pathCovers[t].covers.size();j++) 
      if(pathCovers[t].covers[j]->parent == n) 
	PruneSearchNode(pathCovers[t].covers[j].get(),q);
  }
  if(q) {
    IndexedPriorityQueue<PathSearchNode*,Real>::iterator it=q->find(n);
    if(it != q->end()) {
      q->erase(it);
      //LOG4CXX_INFO(KrisLibrary::logger(),"  ...in heap");
    }
  }
  for(size_t j=0;j<pathCovers[v].covers.size();j++)
    if(pathCovers[v].covers[j].get()==n) {
      n->parent = NULL;
      pathCovers[v].covers.erase(pathCovers[v].covers.begin()+j);
    }
  if(DEBUG) Assert(SanityCheck());
}

//returns true if changed
bool DisplacementPlanner::UpdateCoversIn(int n,Real maxTotalCost)
{
  if(n == 0) {
    shared_ptr<PathSearchNode> c(new PathSearchNode);
    c->vertex = n;
    c->assignment.resize(displacementSamples.size(),0);
    c->pathLength = 0;
    c->totalCost = 0;
    c->parent = NULL;
    if(!FindMinimumAssignment(c.get(),maxTotalCost)) {
      return false;
    }
    c->totalCost = Cost(c->assignment);
    //test if it's revisited -- if so, no update needed
    if(Revisited(c.get())) return false;
    for(size_t i=0;i<pathCovers[n].covers.size();i++) 
      PruneSearchNode(pathCovers[n].covers[i].get());
    pathCovers[n].covers.resize(1);
    pathCovers[n].covers[0] = c;
    if(DEBUG) Assert(SanityCheck(true));
    return true;
  }
  else {
    //find the optimal path into n, possibly requesting more parent
    //constraints to be checked
    int numNew = 0;
    Roadmap::Iterator e;
    for(roadmap.Begin(n,e);!e.end();e++) {
      int t=e.target();
      if(t==1) continue;
      vector<shared_ptr<PathSearchNode> > newNodes;
      for(size_t i=0;i<pathCovers[t].covers.size();i++) {
	shared_ptr<PathSearchNode> c=UpdateEdge(t,n,pathCovers[t].covers[i].get(),maxTotalCost);
	if(c) {
	  newNodes.push_back(c);
	  //do pruning
	  bool changed = true;
	  while(changed) {
	    changed = false;
	    for(size_t i=0;i<pathCovers[n].covers.size();i++) {
	      if(c.get() == pathCovers[n].covers[i].get()) continue;
	      if(Dominates(c.get(),pathCovers[n].covers[i].get())) {
		changed = true;
		PruneSearchNode(pathCovers[n].covers[i].get());
		break;
	      }
	    }
	  }
	}
      }
      numNew += (int)newNodes.size();
    }
    if(DEBUG) Assert(SanityCheck(true));
    return (numNew!=0);
  }
}

void DisplacementPlanner::UpdateCoversOut(int nstart,Real maxTotalCost)
{  
  if(nstart==1) return;
  //start propagating changes to neighbors
  IndexedPriorityQueue<PathSearchNode*,Real> q;
  for(size_t i=0;i<pathCovers[nstart].covers.size();i++) {
    Real c=pathCovers[nstart].covers[i]->totalCost;
    if(c<=maxTotalCost)
      q.insert(pathCovers[nstart].covers[i].get(),c);
  }
  vector<shared_ptr<PathSearchNode> > newNodes;

  vector<Real> oldCosts;
  vector<int> changedNodes;
  if(DEBUG) {
    oldCosts.resize(roadmap.nodes.size());
    for(size_t i=0;i<roadmap.nodes.size();i++) {
      oldCosts[i] = OptimalCost(i);
    }
  }
  //LOG4CXX_INFO(KrisLibrary::logger(),"UpdateCoversOut");
  while(!q.empty()) {
    numUpdateCoversIterations++;
    PathSearchNode* n = q.top().second;   q.pop();

    PropagateDownstream(n,newNodes,maxTotalCost);
    if(DEBUG) Assert(SanityCheck());

    //prune other dominated nodes at the same vertex
    for(size_t i=0;i<newNodes.size();i++) {
      //a node in newNodes can be erased by PruneSearchNode -- this causes
      //an occasional crash if you're not careful
      if(newNodes[i]->parent == NULL) //was pruned
	continue;
      Real c=newNodes[i]->totalCost;
      int v = newNodes[i]->vertex;
      if(DEBUG) changedNodes.push_back(v);
      bool changed = true;
      while(changed) {
	changed = false;
	for(size_t j=0;j<pathCovers[v].covers.size();j++) {
	  if(pathCovers[v].covers[j].get() == newNodes[i].get()) continue;
	  if(Dominates(newNodes[i].get(),pathCovers[v].covers[j].get())) {
	    changed = true;
	    //LOG4CXX_INFO(KrisLibrary::logger(),"  Erasing node "<<v<<" cover "<<j<<", cost "<<pathCovers[v].covers[j]->totalCost<<"->"<<newNodes[i]->totalCost);
	    PruneSearchNode(pathCovers[v].covers[j].get(),&q);
	  }
	}
      }

      //add new node to queue
      if(c<=maxTotalCost)
	q.insert(newNodes[i].get(),c);
      //else
      //LOG4CXX_INFO(KrisLibrary::logger(),"  Skipping node "<<newNodes[i]->vertex);
    }
    if(DEBUG) Assert(SanityCheck(true));

    //reached goal
    if(n->vertex == 1) break;
  }
  if(DEBUG) {
    if(!changedNodes.empty()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"  Changed: ");
      for(vector<int>::iterator i=changedNodes.begin();i!=changedNodes.end();i++)
	LOG4CXX_INFO(KrisLibrary::logger(),""<<*i);
      LOG4CXX_INFO(KrisLibrary::logger(),"");
    }
    for(size_t i=0;i<roadmap.nodes.size();i++) {
      if(oldCosts[i] <= maxTotalCost) {
	if(OptimalCost(i) > oldCosts[i])
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: cost to node "<<i<<" increased "<<OptimalCost(i)<<" > "<<oldCosts[i]);
	//Assert(OptimalCost(i) <= oldCosts[i]);
      }
    }
  }
}

bool DisplacementPlanner::ParetoDominates(const vector<int>& a,const vector<int>& b) const
{
  Assert(a.size()==b.size());
  Assert(a.size()==displacementSampleCosts.size());
  for(size_t i=0;i<a.size();i++)
    if(displacementSampleCosts[i][a[i]] > displacementSampleCosts[i][b[i]])
      return false;
  return true;
}

Real DisplacementPlanner::OptimalCost(int n) const
{
  Real bestCost = Inf;
  for(size_t i=0;i<pathCovers[n].covers.size();i++) {
    Real c=pathCovers[n].covers[i]->totalCost;
    if(c < bestCost) {
      bestCost = c;
    }
  }
  return bestCost;
}

DisplacementPlanner::PathSearchNode* DisplacementPlanner::OptimalPathTo(int n)
{
  Real bestCost = Inf;
  PathSearchNode* best=NULL;
  for(size_t i=0;i<pathCovers[n].covers.size();i++) {
    Real c=pathCovers[n].covers[i]->totalCost;
    if(c < bestCost) {
      bestCost = c;
      best = pathCovers[n].covers[i].get();
    }
  }
  return best;
}


void DisplacementPlanner::GetPath(PathSearchNode* n,std::vector<int>& path) const
{
  while(n != NULL) {
    path.push_back(n->vertex);
    n=n->parent;
  }
  reverse(path.begin(),path.end());
}

void DisplacementPlanner::GetMilestonePath(const std::vector<int>& path,MilestonePath& mpath) const
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

void DisplacementPlanner::GetMilestonePath(PathSearchNode* n,MilestonePath& mpath) const
{
  mpath.edges.resize(0);
  while(n->parent != NULL) {
    if(n->parent->vertex < n->vertex) 
      mpath.edges.push_back(roadmap.FindEdge(n->parent->vertex,n->vertex)->e->Copy());
    else
      mpath.edges.push_back(roadmap.FindEdge(n->vertex,n->parent->vertex)->e->ReverseCopy());
    n=n->parent;
  }
  reverse(mpath.edges.begin(),mpath.edges.end());
}
