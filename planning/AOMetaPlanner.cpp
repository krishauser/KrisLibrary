#include <KrisLibrary/Logger.h>
#include "AOMetaPlanner.h"
#include "CostSpace.h"
#include "InterpolatorHelpers.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/Timer.h>
using namespace std;

class CostGoal : public CSet
{
public:
  CostGoal(CSet* _baseGoal,const std::shared_ptr<ObjectiveFunctionalBase>& _objective,Real _costMax)
  :baseGoal(_baseGoal),objective(_objective),costMax(_costMax)
  {}


  virtual bool IsSampleable() const { return baseGoal->IsSampleable(); }
  virtual bool Contains(const Config& xc) {
    Vector x;
    Real c;
    StateCostControlSpace::SplitRef(xc,x,c);
    if(!baseGoal->Contains(x)) return false;
    if(IsInf(costMax)) return true;
    return c + objective->TerminalCost(x) <= costMax;
  }
  virtual void Sample(Config& xc) {
    Config x;
    baseGoal->Sample(x);
    if(x.empty()) { xc.clear(); return; }
    Real cmax = 0.0;
    if(!IsInf(costMax)) cmax = Max(0.0,costMax-objective->TerminalCost(x));
    Real c = Rand()*cmax;
    StateCostControlSpace::Join(x,c,xc);
  }
  virtual bool Project(Config& xc) {
    Config x;
    Real c;
    StateCostControlSpace::SplitRef(xc,x,c);
    if(!baseGoal->Project(x)) return false;
    c = Min(costMax-objective->TerminalCost(x),c);
    if(c < 0) return false;
    xc[xc.n-1] = c;
    return true;
  }
  CSet* baseGoal;
  std::shared_ptr<ObjectiveFunctionalBase> objective;
  Real costMax;
};


CostSpaceRRTPlanner::CostSpaceRRTPlanner(const std::shared_ptr<KinodynamicSpace>& _baseSpace,const std::shared_ptr<ObjectiveFunctionalBase>& _objective,Real costMax)
:LazyRRTKinodynamicPlanner(NULL),baseSpace(_baseSpace),costSpace(make_shared<StateCostKinodynamicSpace>(_baseSpace,_objective,costMax)),objective(_objective),costGoalSet(NULL),
 heuristic(NULL),costSpaceDistanceWeight(1),lazy(false),bestPathCost(Inf),
 prunableNodeSampleCount(0),nodeSampleCount(0),
 numGoalsSampled(0),numPrunedNodes(0)
{
  RRTKinodynamicPlanner::space = costSpace.get();
  RRTKinodynamicPlanner::tree.space = costSpace.get();
  if(IsInf(costMax)) EnableCostSpaceBias(false);
  else {
    bestPathCost = costMax;
    SetCostMax(costMax);
    EnableCostSpaceBias(true);
  }
}

void CostSpaceRRTPlanner::Init(const State& xinit,CSet* goalSet)
{
  costGoalSet = make_shared<CostGoal>(goalSet,objective,bestPathCost);
  Vector cx0;
  StateCostControlSpace::Join(xinit,0.0,cx0);
  RRTKinodynamicPlanner::Init(cx0,costGoalSet.get());
}

bool CostSpaceRRTPlanner::Done() const
{
  return !bestPath.Empty();
}

void CostSpaceRRTPlanner::EnableCostSpaceBias(bool enabled)
{
  if(!enabled) {
    SetCostMax(Inf);
    SetCostDistanceWeight(0);
  }
  else {
    SetCostMax(bestPathCost);
    SetCostDistanceWeight(costSpaceDistanceWeight);
  }
}

void CostSpaceRRTPlanner::SetCostMax(Real cmax)
{
  dynamic_cast<StateCostKinodynamicSpace*>(&*costSpace)->SetCostMax(cmax);
  if(costGoalSet != NULL) {
    CostGoal* cg = dynamic_cast<CostGoal*>(&*costGoalSet);
    Assert(cg != NULL);
    cg->costMax = cmax;
  }
}

void CostSpaceRRTPlanner::SetCostDistanceWeight(Real w)
{
  if(w != 0)
    costSpaceDistanceWeight = w;
  StateCostKinodynamicSpace* scspace = dynamic_cast<StateCostKinodynamicSpace*>(&*costSpace);
  if(scspace->GetCostDistanceWeight() != 0)
    scspace->SetCostDistanceWeight(w);
  //need to update point location data structure
  KDTreePointLocation* kdtree = dynamic_cast<KDTreePointLocation*>(&*tree.pointLocation);
  if(kdtree != NULL) {
    Assert(kdtree->weights.n == scspace->GetStateSpace()->NumDimensions());
    kdtree->weights[kdtree->weights.n-1] = w;
  }
}

void CostSpaceRRTPlanner::SetHeuristic(HeuristicFn f)
{
  heuristic = f;
}


void CostSpaceRRTPlanner::PickDestination(State& xdest)
{
  if(!heuristic || IsInf(bestPathCost)) {
    RRTKinodynamicPlanner::PickDestination(xdest);
  }
  else { 
    //sample cost with heursitic bias
    for(int iters=0;iters<1000;iters++) {
      State x;
      Real c;
      RRTKinodynamicPlanner::PickDestination(xdest);
      StateCostControlSpace::SplitRef(xdest,x,c);
      c = Rand()*(bestPathCost-heuristic(x));
      if(c < 0) continue;
      xdest[x.n] = c;
      return;
    }
    RRTKinodynamicPlanner::PickDestination(xdest);
  }
}

Real CostSpaceRRTPlanner::PathCost() const
{
  return bestPathCost;
}

bool CostSpaceRRTPlanner::GetPath(KinodynamicMilestonePath& path)
{
  if(bestPath.Empty()) return false;

  KinodynamicMilestonePath& cpath = bestPath;
  //project to the first n-1 dimensions
  path.milestones.resize(cpath.milestones.size());
  path.controls = cpath.controls;
  Real c;
  for(size_t i=0;i<cpath.milestones.size();i++)
    StateCostControlSpace::Split(cpath.milestones[i],path.milestones[i],c);
  path.paths.resize(cpath.paths.size());
  for(size_t i=0;i<cpath.paths.size();i++) {
    if(dynamic_cast<MultiInterpolator*>(&*cpath.paths[i]) != NULL)
      path.paths[i] = dynamic_cast<MultiInterpolator*>(cpath.paths[i].get())->components[0];
    else
      path.paths[i].reset(new SubsetInterpolator(cpath.paths[i],0,path.milestones[0].n));
  }
  return true;
}

RRTKinodynamicPlanner::Node* CostSpaceRRTPlanner::ExtendToward(const State& xdest)
{
  if(heuristic && !IsInf(bestPathCost) && heuristic(xdest) > bestPathCost) {
    numFilteredExtensions++;
    return NULL;
  }
  Timer timer;
  nodeSampleCount+=1;
  Node* n=tree.FindClosest(xdest);
  nnTime += timer.ElapsedTime();
  timer.Reset();
  Real cn = StateCostControlSpace::Cost(*n);
  if(!IsInf(bestPathCost)) {
    if(cn > bestPathCost) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Closest node is prunable");
      prunableNodeSampleCount+=1;
      return NULL;
    }
    if(heuristic && cn + heuristic(*n) > bestPathCost) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Closest node is prunable w/ heuristic");
      prunableNodeSampleCount+=1;
      return NULL;
    }
  }
  Real d = space->GetStateSpace()->Distance(*n,xdest);
  //LOG4CXX_INFO(KrisLibrary::logger(),"Source "<<*n<<" dest "<<xdest<<"");
  //LOG4CXX_INFO(KrisLibrary::logger(),"Distance to closest "<<d<<"");
  Vector temp = xdest;
  if(d > delta)  {
    space->GetStateSpace()->Interpolate(*n,xdest,delta/d,temp);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Interpolated point "<<temp<<"");
  }

  KinodynamicMilestonePath path;
  if(!PickControl(*n,temp,path)) {
    pickControlTime += timer.ElapsedTime();
    numInfeasibleControls++;
    //LOG4CXX_INFO(KrisLibrary::logger(),"PickControl failed");
    return NULL;
  }
  pickControlTime += timer.ElapsedTime();
  timer.Reset();
  Assert(path.Start().n == space->GetStateSpace()->NumDimensions());
  Assert(path.End().n == space->GetStateSpace()->NumDimensions());
  if(FilterExtension(n,path)) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Extension filtered");
    numFilteredExtensions++;
    return NULL;
  }
  if(!space->GetStateSpace()->IsFeasible(path.End())) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Edge endpoint is not feasible");
    visibleTime += timer.ElapsedTime();
    //LOG4CXX_INFO(KrisLibrary::logger(),"Infeasible endpoint");
    numInfeasibleEndpoints++;
    return NULL;
  }

  Assert(path.Start().n == space->GetStateSpace()->NumDimensions());
  Assert(path.End().n == space->GetStateSpace()->NumDimensions());

  EdgePlannerPtr e(space->TrajectoryChecker(path));
  Assert(e->Start().n == space->GetStateSpace()->NumDimensions());
  Assert(e->End().n == space->GetStateSpace()->NumDimensions());
  if(lazy || e->IsVisible()) {
    visibleTime += timer.ElapsedTime();
    numSuccessfulExtensions++;
    return tree.AddMilestone(n,path,e);
  }
  else {
    visibleTime += timer.ElapsedTime();
    //LOG4CXX_INFO(KrisLibrary::logger(),"Edge is not visible");
    return NULL;
  }
}

bool CostSpaceRRTPlanner::Plan(int maxIters)
{
  bool foundNewPath = false;
  for(int i=0;i<maxIters;i++) {
    numIters++;
    bool prune = false;
    Node* n=RRTKinodynamicPlanner::Extend();
    if(n && goalSet && goalSet->Contains(*n)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Got a new node in goal");
      //new goal node found
      Real cn = TerminalCost(*n);
      numGoalsSampled++;
      if(goalNode == NULL || cn < bestPathCost) {
        if(!lazy || CheckPath(n)) {
          goalNode = n;
          foundNewPath = true;
          if(IsInf(bestPathCost)) {
            LOG4CXX_INFO(KrisLibrary::logger(),"Got initial best path, cost "<<cn);
            bestPathCost = cn;
            RRTKinodynamicPlanner::GetPath(bestPath);
            EnableCostSpaceBias(true);
            nodeSampleCount = prunableNodeSampleCount = 0;
            prune = true;
          }
          else {
            //improves upon previous solution
            LOG4CXX_INFO(KrisLibrary::logger(),"Improved best path cost from "<<bestPathCost<<" to "<<cn);
            //if(cn < 0.9*bestPathCost) prune = true;
            //TEST: always prune
            prune = true;
            bestPathCost = cn;
            RRTKinodynamicPlanner::GetPath(bestPath);
            SetCostMax(cn);
            OnNewBestPath();
          }
          //there's the problem of history dependence
          //delete the path leading up to goalNode
          int numSteps = 4;
          n = goalNode;
          while(goalNode && numSteps-- >= 0) {
            n = goalNode;
            goalNode = goalNode->getParent();
          }
          if(n != tree.root)
            tree.DeleteSubTree(n);
        }
      }
    }
    if(prune || (prunableNodeSampleCount > 10 && prunableNodeSampleCount*100 > nodeSampleCount)) {
      //prune the tree
      LOG4CXX_INFO(KrisLibrary::logger(),"Pruning the RRT tree");
      PruneTree();
      nodeSampleCount = prunableNodeSampleCount = 0;
    }
  }
  return foundNewPath;
}

struct PruneCallback : public Graph::CallbackBase<CostSpaceRRTPlanner::Node*>
{
  typedef CostSpaceRRTPlanner::Node Node;
  CostSpaceRRTPlanner* planner;
  PruneCallback(CostSpaceRRTPlanner* _planner):planner(_planner) {}
  virtual void Visit(Node* n) { 
    Real cn = StateCostControlSpace::Cost(*n);
    if(cn > planner->bestPathCost) {
      deleteNodes.push_back(n);
    }
    else if(planner->heuristic && cn + planner->heuristic(*n) > planner->bestPathCost) {
      if(find(prevent.begin(),prevent.end(),n) != prevent.end()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"CostSpaceRRTPlanner: Warning, heuristic function is not admissible, trying to prune path to goal node");
        return;
      }
      deleteNodes.push_back(n);
    }
  }
  virtual bool Descend(Node* n) {
    if(!deleteNodes.empty() && n == deleteNodes.back()) return false;
    return true;
  }

  vector<Node*> prevent;
  vector<Node*> deleteNodes;
};

void CostSpaceRRTPlanner::PruneTree()
{
  PruneCallback callback(this);
  /*
  Node* n = goalNode;
  while(n != NULL) {
    callback.prevent.push_back(n);
    n = n->getParent();
  }
  */
  tree.root->DFS(callback);
  if(callback.deleteNodes.empty()) return;
  LOG4CXX_INFO(KrisLibrary::logger(),"CostSpaceRRTPlanner::PruneTree() Pruning "<<(int)callback.deleteNodes.size());
  numPrunedNodes += (int)callback.deleteNodes.size();
  for(size_t i=0;i+1<callback.deleteNodes.size();i++) {
    Node* p = callback.deleteNodes[i]->getParent();
    p->detachChild(callback.deleteNodes[i]);
    delete callback.deleteNodes[i];
  }
  //this will rebuild the point location data structure
  tree.DeleteSubTree(callback.deleteNodes.back());
}

bool CostSpaceRRTPlanner::FilterExtension(Node* n,const KinodynamicMilestonePath& path)
{
  if(IsInf(bestPathCost)) return false;
  Real c = StateCostControlSpace::Cost(path.End());
  if(!heuristic) return (c > bestPathCost);
  else {
    Vector x;
    StateCostControlSpace::SplitRef(path.End(),x,c);
    return (c + heuristic(x) > bestPathCost);
  }
}

void CostSpaceRRTPlanner::OnNewBestPath()
{
  //TODO: pruning suboptimal branches of RRT tree?
}


Real CostSpaceRRTPlanner::TerminalCost(const State& xc)
{
  State x;
  Real c;
  StateCostControlSpace::SplitRef(xc,x,c);
  return c + objective->TerminalCost(x);
}

void CostSpaceRRTPlanner::GetStats(PropertyMap& stats) const
{
  RRTKinodynamicPlanner::GetStats(stats);
  stats.set("numGoalsSampled",numGoalsSampled);
  stats.set("numPrunedNodes",numPrunedNodes);
}





CostSpaceESTPlanner::CostSpaceESTPlanner(const std::shared_ptr<KinodynamicSpace>& _baseSpace,const std::shared_ptr<ObjectiveFunctionalBase>& _objective,Real costMax)
:ESTKinodynamicPlanner(NULL),baseSpace(_baseSpace),costSpace(make_shared<StateCostKinodynamicSpace>(_baseSpace,_objective,costMax)),objective(_objective),costGoalSet(NULL),
 costSpaceResolution(0.1),bestPathCost(Inf),
 prunableNodeSampleCount(0),nodeSampleCount(0),
 numGoalsSampled(0),numPrunedNodes(0)
{
  ESTKinodynamicPlanner::space = costSpace.get();
  ESTKinodynamicPlanner::tree.space = costSpace.get();
  if(IsInf(costMax)) EnableCostSpaceBias(false);
  else {
    bestPathCost = costMax;
    SetCostMax(costMax);
    EnableCostSpaceBias(true);
  }
}

void CostSpaceESTPlanner::Init(const State& xinit,CSet* goalSet)
{
  costGoalSet = make_shared<CostGoal>(goalSet,objective,bestPathCost);
  Vector cx0;
  StateCostControlSpace::Join(xinit,0.0,cx0);
  ESTKinodynamicPlanner::Init(cx0,costGoalSet.get());
}

void CostSpaceESTPlanner::EnableCostSpaceBias(bool enabled)
{
  if(!enabled) {
    SetCostMax(Inf);
    int nbase = baseSpace->GetStateSpace()->NumDimensions();
    MultiGridDensityEstimator* mde = dynamic_cast<MultiGridDensityEstimator*>(&*densityEstimator);
    Vector h;
    if(mde) h = mde->h;
    else h.resize(nbase,0.1);
    if(h.n > nbase) h.n = nbase;
    densityEstimator = make_shared<MultiGridDensityEstimator>(nbase,3,h);
  }
  else {
    SetCostMax(bestPathCost);
    int nsc = baseSpace->GetStateSpace()->NumDimensions()+1;
    MultiGridDensityEstimator* mde = dynamic_cast<MultiGridDensityEstimator*>(&*densityEstimator);
    Vector h(nsc);
    if(mde) h.copySubVector(0,mde->h);
    else h.set(0.1);
    h(nsc-1) = costSpaceResolution;
    densityEstimator = make_shared<MultiGridDensityEstimator>(nsc,3,h);
  }
  RebuildDensityEstimator();
}

void CostSpaceESTPlanner::SetCostMax(Real cmax)
{
  dynamic_cast<StateCostKinodynamicSpace*>(&*costSpace)->SetCostMax(cmax);
  if(costGoalSet != NULL) {
    CostGoal* cg = dynamic_cast<CostGoal*>(&*costGoalSet);
    Assert(cg != NULL);
    cg->costMax = cmax;
  }
}


void CostSpaceESTPlanner::SetDensityEstimatorResolution(Real res)
{
  costSpaceResolution = res;
  ESTKinodynamicPlanner::SetDensityEstimatorResolution(res);
}

void CostSpaceESTPlanner::SetDensityEstimatorResolution(const Vector& res)
{
  costSpaceResolution = res[res.n-1];
  ESTKinodynamicPlanner::SetDensityEstimatorResolution(res);
}

void CostSpaceESTPlanner::SetHeuristic(HeuristicFn f)
{
  heuristic = f;
}

Real CostSpaceESTPlanner::PathCost() const
{
  return bestPathCost;
}

bool CostSpaceESTPlanner::FilterExtension(Node* n,const KinodynamicMilestonePath& path)
{
  if(IsInf(bestPathCost)) return false;
  Real c = StateCostControlSpace::Cost(path.End());
  if(!heuristic) return (c > bestPathCost);
  else {
    Vector x;
    StateCostControlSpace::SplitRef(path.End(),x,c);
    return (c + heuristic(x) > bestPathCost);
  }
}

bool CostSpaceESTPlanner::GetPath(KinodynamicMilestonePath& path)
{
  if(bestPath.Empty()) return false;
  KinodynamicMilestonePath& cpath = bestPath;
  //project to the first n-1 dimensions
  path.milestones.resize(cpath.milestones.size());
  path.controls = cpath.controls;
  Real c;
  for(size_t i=0;i<cpath.milestones.size();i++)
    StateCostControlSpace::Split(cpath.milestones[i],path.milestones[i],c);
  path.paths.resize(cpath.paths.size());
  for(size_t i=0;i<cpath.paths.size();i++) {
    MultiInterpolator* mi = dynamic_cast<MultiInterpolator*>(&*cpath.paths[i]);
    if(mi != NULL)
      path.paths[i] = mi->components[0];
    else
      path.paths[i].reset(new SubsetInterpolator(cpath.paths[i],0,path.milestones[0].n));
  }
  return true;
}

bool CostSpaceESTPlanner::Plan(int maxIters)
{
  bool foundNewPath = false;
  for(int i=0;i<maxIters;i++) {
    bool prune = false;
    bool res=ESTKinodynamicPlanner::Plan(1);
    if(res) {
      //new goal node found
      numGoalsSampled++;
      Real cn = TerminalCost(*goalNode);
      if(cn < bestPathCost) {
        bool gotpath = ESTKinodynamicPlanner::GetPath(bestPath);
        Assert(gotpath);

        foundNewPath = true;
        if(IsInf(bestPathCost)) {
          LOG4CXX_INFO(KrisLibrary::logger(),"Got initial best path, cost "<<cn);
          bestPathCost = cn;
          EnableCostSpaceBias(true);
          nodeSampleCount = prunableNodeSampleCount = 0;
          prune = true;
        }
        else {
          //improves upon previous solution
          LOG4CXX_INFO(KrisLibrary::logger(),"Improved best path cost from "<<bestPathCost<<" to "<<cn);
          //if(cn < 0.9*bestPathCost) prune = true;
          //TEST: always prune
          prune = true;
          bestPathCost = cn;
          SetCostMax(cn);
          OnNewBestPath();
        }
      }
    }
    if(prune || (prunableNodeSampleCount > 10 && prunableNodeSampleCount*10 > nodeSampleCount)) {
      //prune the tree
      LOG4CXX_INFO(KrisLibrary::logger(),"Pruning the EST tree");
      PruneTree();
      nodeSampleCount = prunableNodeSampleCount = 0;
    }
  }
  return foundNewPath;
}

struct PruneCallback2 : public Graph::CallbackBase<CostSpaceESTPlanner::Node*>
{
  typedef CostSpaceESTPlanner::Node Node;
  CostSpaceESTPlanner* planner;
  PruneCallback2(CostSpaceESTPlanner* _planner):planner(_planner) {}
  virtual void Visit(Node* n) { 
    Real cn = StateCostControlSpace::Cost(*n);
    if(cn > planner->bestPathCost) {
      deleteNodes.push_back(n);
    }
    else if(planner->heuristic && cn + planner->heuristic(*n) > planner->bestPathCost) {
      if(find(prevent.begin(),prevent.end(),n) != prevent.end()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"CostSpaceRRTPlanner: Warning, heuristic function is not admissible, trying to prune path to goal node");
        return;
      }
      deleteNodes.push_back(n);
    }
  }
  virtual bool Descend(Node* n) {
    if(!deleteNodes.empty() && n == deleteNodes.back()) return false;
    return true;
  }

  vector<Node*> prevent;
  vector<Node*> deleteNodes;
};

void CostSpaceESTPlanner::PruneTree()
{
  PruneCallback2 callback(this);
  Node* n = goalNode;
  while(n != NULL) {
    callback.prevent.push_back(n);
    n = n->getParent();
  }
  tree.root->DFS(callback);
  if(callback.deleteNodes.empty()) return;
  numPrunedNodes += (int)callback.deleteNodes.size();
  LOG4CXX_INFO(KrisLibrary::logger(),"CostSpaceESTPlanner::PruneTree() Pruning "<<(int)callback.deleteNodes.size());
  for(size_t i=0;i+1<callback.deleteNodes.size();i++) {
    Node* p = callback.deleteNodes[i]->getParent();
    p->detachChild(callback.deleteNodes[i]);
    delete callback.deleteNodes[i];
  }
  //this will rebuild the point location data structure
  tree.DeleteSubTree(callback.deleteNodes.back());

  RebuildDensityEstimator();
}

void CostSpaceESTPlanner::OnNewBestPath()
{
  //TODO: pruning suboptimal branches of EST tree?
}


Real CostSpaceESTPlanner::TerminalCost(const State& xc)
{
  State x;
  Real c;
  StateCostControlSpace::SplitRef(xc,x,c);
  return c + objective->TerminalCost(x);
}

void CostSpaceESTPlanner::GetStats(PropertyMap& stats) const
{
  ESTKinodynamicPlanner::GetStats(stats);
  stats.set("numGoalsSampled",numGoalsSampled);
  stats.set("numPrunedNodes",numPrunedNodes);
}

