#include <KrisLibrary/Logger.h>
#include "FMM.h"
#include <structs/FixedSizeHeap.h>
#include <utils/indexing.h>
#include <utils/combination.h>
#include <math/math.h>
#include <math/misc.h>
#include <math/vector.h>
#include <utils/ioutils.h>
#include <fstream>
#include <Timer.h>
#include <iostream>
#include <algorithm>
using namespace std;
using namespace Math;

#define DO_TIMING 0

Real Sum(const Vector& x)
{
  Real val=0.0;
  for(int i=0;i<x.n;i++) val += x(i);
  return val;
}

/**
   for three adjacent points at (1,0,0), (0,1,0), (0,0,1) with distances
   d1, d2, and d3 from the source, finds the minimum distance point to
   the origin on the diagonal
   min d(u,v,w) = u*d1 + v*d2 + w*d3 + ||u,v,w||
   s.t. u+v+w=1
*/
Real best_diag_distanceN(const Vector& d)
{
  /* Lagrange equation: d'(u,v,w) + lambda (1,1,1) = 0
    d' = (d1,d2,d3) + (u,v,w)/||u,v,w||
    (d1+lambda,d2+lambda,d3+lambda)||u,v,w|| = (-u,-v,-w)
    (d1+lambda)^2 ||u,v,w||^2 = u^2
    (d2+lambda)^2 ||u,v,w||^2 = v^2
    (d3+lambda)^2 ||u,v,w||^2 = w^2
    [(d1+lambda)^2 + (d2+lambda)^2 + (d3+lambda)^2]||u,v,w||^2 = ||u,v,w||^2
    (d1+lambda)^2 + (d2+lambda)^2 + (d3+lambda)^2 = 1
    3 lambda^2 + (2d1+2d2+2d3) lambda + (d1^2+d2^2+d3^2-1) = 0
    lambda = -(d1+d2+d3)/3 +/- 1/3 sqrt((d1+d2+d3)^2 - 3 (d1^2+d2^2+d3^2-1))
  */

  Real dsum = Sum(d);
  int n=d.n;
  Real dsumsq = Sqr(dsum);
  Real dsqsum = d.normSquared();
  Real det = dsumsq - n*(dsqsum-1.0);
  if(det < 0.0) {
        //LOG4CXX_ERROR(KrisLibrary::logger(),"Negative determinant: "<<det);
    //LOG4CXX_ERROR(KrisLibrary::logger(),"D: "<<d);
    return d.minElement()+1.0;
  }
  Real sqdet = Sqrt(det);
  Real lambda1 = (-dsum + sqdet)/n;
  Real lambda2 = (-dsum - sqdet)/n;
  Vector u1=d,u2=d;
  for(int i=0;i<d.n;i++)
    u1[i] += lambda1;
  for(int i=0;i<d.n;i++)
    u2[i] += lambda2;
  
  u1 *= 1.0/(dsum+lambda1*n);
  u2 *= 1.0/(dsum+lambda2*n);
  Real val1=Inf,val2=Inf;
  if(u1.minElement() >= 0.0 && u1.maxElement() <= 1.0)
    val1 = u1.dot(d)+u1.norm();
  if(u2.minElement() >= 0.0 && u2.maxElement() <= 1.0)
    val2 = u2.dot(d)+u2.norm();
  return Min(val1,val2);
}



//returns the offsets of all adjacent nodes in the grid
void Adjacent(vector<int>& index,int offset,const ArrayND<Real>& grid,vector<int>& noffsets)
{
  noffsets.resize(0);
  noffsets.reserve(index.size()*2);  
  vector<int> temp = index;
  for(size_t i=0;i<index.size();i++) {
    temp[i] += 1;
    if(temp[i] < grid.dims[i]) {
      noffsets.push_back(offset + grid.strides[i]);
    }
    temp[i] -= 2;
    if(temp[i] >= 0) {
      noffsets.push_back(offset - grid.strides[i]);
    }
    temp[i] += 1;
  }
}


struct SimplexEnumerator
{
  vector<int> node;
  int nodeOffset;
  const ArrayND<int>& grid;
  //valid axis offsets for each axis (subsets of {-1,1})
  vector<vector<int> > candidates;
  //list of occupied axes
  vector<int> occupied;

  //iteration data
  int level;
  vector<int> axisIndex;
  vector<int> candidateIndex;
  //size of occupied candidates on the current axis
  vector<int> candidateSizes;

  //constructor.
  //if the value of a neighbor in the grid is equal to gridfilter, this adds it to the potential simplex
  SimplexEnumerator(const vector<int>& _node,const ArrayND<int>& _grid,int gridfilter)
    :grid(_grid)
  {
    node = _node;
    nodeOffset = grid.indexToOffset(_node);

    //axis aligned candidates
    candidates.resize(node.size());
    vector<int> temp = node;
    for(size_t i=0;i<node.size();i++) {
      temp[i] += 1;
      int index = nodeOffset + grid.strides[i];
      if (temp[i] < grid.dims[i] && grid.values[index]==gridfilter)
	candidates[i].push_back(1);
      temp[i] -= 2;
      index = nodeOffset - grid.strides[i];
      if(temp[i] >= 0 && grid.values[index]==gridfilter) 
	candidates[i].push_back(-1);
      temp[i] += 1;
    }
    //enumerate combinations of k candidates
    for(size_t i=0;i<node.size();i++)
      if(!candidates[i].empty()) occupied.push_back(i);

    reset();
  }

  void reset() {
    level = 1;
    if(occupied.empty()) return;
    axisIndex.resize(1);
    candidateIndex.resize(1);
    axisIndex[0] = 0;
    candidateIndex[0] = 0;
    candidateSizes.resize(1);
    candidateSizes[0] = (int)candidates[occupied[0]].size();
  }

  void get(vector<vector<int> >& res)
  {
    res.resize(level);
    for(size_t i=0;i<axisIndex.size();i++) {
      res[i] = node;
      int axis = axisIndex[i];
      int candidate = candidateIndex[i];
      int origaxis = occupied[axis];
      res[i][origaxis] += candidates[origaxis][candidate];
    }
  }

  void getOffsets(vector<int>& res)
  {
    res.resize(level);
    for(size_t i=0;i<axisIndex.size();i++) {
      int axis = axisIndex[i];
      int candidate = candidateIndex[i];
      int origaxis = occupied[axis];
      res[i] = nodeOffset+candidates[origaxis][candidate]*grid.strides[origaxis];
    }
  }

  bool next() {
    if(level > (int)occupied.size()) return false;
    int nextAxis = IncrementIndex(candidateIndex,candidateSizes);
    if(nextAxis) {
      fill(candidateIndex.begin(),candidateIndex.end(),0);
      int nextLevel = NextCombination(axisIndex,occupied.size());
      if(nextLevel) {
	level++;
	if(level > (int)occupied.size()) return false;
	axisIndex.resize(level);
	FirstCombination(axisIndex,occupied.size());
	candidateIndex.resize(level);
	fill(candidateIndex.begin(),candidateIndex.end(),0);
      }
      candidateSizes.resize(level);
      for(size_t i=0;i<candidateSizes.size();i++)
	candidateSizes[i] = candidates[occupied[axisIndex[i]]].size();
    }
    return true;
  }

  void prune(Real upperBound,const ArrayND<Real>& distances) {
    vector<int> prunedAxes;
    for(size_t i=0;i<candidates.size();i++) {
      if(candidates[i].empty()) continue;
      //pruned index must not be active in current enumerator state!
      bool isActive = (std::find(axisIndex.begin(),axisIndex.end(),(int)i) != axisIndex.end());
      if(isActive) continue;

      vector<int> n = node;
      for(size_t j=0;j<candidates[i].size();j++) {
	n[i] = node[i] + candidates[i][j];
	if(distances[n] >= upperBound) {
	  //delete candidate
	  candidates[i].erase(candidates[i].begin()+j);
	  j--;
	}
      }
      if(candidates[i].empty()) prunedAxes.push_back(i);
    }
    if(!prunedAxes.empty()) {
      //need to change current status
      //occupied is a list of occupied axes
      //axisIndex is a subset of the indices of the occupied axes
      occupied.resize(0);
      for(size_t i=0;i<node.size();i++)
	if(!candidates[i].empty()) occupied.push_back(i);
      /*
      LOG4CXX_INFO(KrisLibrary::logger(),"Pruned: ");
      for(size_t i=0;i<prunedAxes.size();i++)
	LOG4CXX_INFO(KrisLibrary::logger(),""<<prunedAxes[i]);
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"AxisIndex: ");
      for(size_t i=0;i<axisIndex.size();i++)
	LOG4CXX_INFO(KrisLibrary::logger(),""<<axisIndex[i]);
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
      */
      int k = 0;
      for(size_t i=0;i<axisIndex.size();i++) {
	while(k < (int)prunedAxes.size() && axisIndex[i] >= prunedAxes[k]) {
	  Assert(axisIndex[i] != prunedAxes[k]);
	  k++;
	}
	Assert(axisIndex[i] >= k);
	axisIndex[i] -= k;
	Assert(axisIndex[i] >= 0 && axisIndex[i] < (int)occupied.size());
      }
      for(size_t i=0;i<candidateSizes.size();i++)
	Assert(candidateSizes[i] == (int)candidates[occupied[axisIndex[i]]].size());
    }
  }
};

bool FMMSearch(const vector<int>& start,const vector<int>& goal,const ArrayND<Real>& costs,ArrayND<Real>& distances)
{
#if DO_TIMING
  Timer timer;
  double initTime=0,estimateTime=0,propagateTime=0,overheadTime=0;
#endif
  int numVisited = 0,numSimplices = 0;
  assert(start.size() == costs.dims.size());

  //initialize search status
  ArrayND<int> status;
  status.resize(costs.dims);
  status.set(0);
  distances.resize(costs.dims);
  distances.set(Inf);

  int sindex = costs.indexToOffset(start);
  int gindex = -1;
  if(goal.size()==start.size()) gindex=costs.indexToOffset(goal);  

  //build queue
  FixedSizeHeap<Real> q(status.numValues());
  q.push(sindex,0.0); 
  status.values[sindex] = 1;
  vector<int> adjacent;
  vector<int> simplex;
  Vector d;

#if DO_TIMING
  initTime = timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  while(!q.empty()) {
    int nindex = q.top();
    q.pop();
    numVisited++;

    //set to visited
    status.values[nindex] = -1;

    //look at visited neighbors
    Real best = Inf;
    if (nindex == sindex)
      best = 0.0;
    Real c = costs.values[nindex];
    vector<int> node=costs.offsetToIndex(nindex);

#if DO_TIMING
    overheadTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING
    if(nindex != sindex) {
      SimplexEnumerator s(node,status,-1);
      do {
	numSimplices++;
	s.getOffsets(simplex);
	Real dcand;
	if(simplex.size() == 1)
	  dcand = distances.values[simplex[0]]+c;
	else {
	  d.resize(simplex.size());
	  for(size_t i=0;i<simplex.size();i++)
	    d[i] = distances.values[simplex[i]];
	  dcand = best_diag_distanceN(d)*c;
	}
	if(dcand < best) {
	  best = dcand;
	  s.prune(best,distances);
	}
      } while(s.next());
#if DO_TIMING
      estimateTime += timer.ElapsedTime();
      timer.Reset();
#endif //DO_TIMING
    }

    distances.values[nindex] = best;

    //check if goal is reached
    if(nindex==gindex) {
#if DO_TIMING
      LOG4CXX_INFO(KrisLibrary::logger(),"Time for init "<<initTime<<", propagate "<<propagateTime<<", estimate "<<estimateTime<<", overhead "<<overheadTime);
#endif //DO_TIMING
      LOG4CXX_INFO(KrisLibrary::logger(),""<<numVisited<<" nodes visited, avg # of simplices "<<Real(numSimplices)/Real(numVisited));
      return true;
    }

    //otherwise, propagate to children
    Adjacent(node,nindex,costs,adjacent);
    for(size_t i=0;i<adjacent.size();i++) {
      int next = adjacent[i];
      Real ncost = best+costs.values[next];
      if(IsInf(ncost)) continue;
      if(status.values[next]==0) {
	q.push(next,-ncost);
	distances.values[next] = ncost;
	status.values[next] = 1;
      }
      else if(status.values[next]==1) {
	//do we have a better distance than the existing?
	if (ncost < distances.values[next]) {
	  if(!q.find(next)) {
	    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, want to adjust "<<next);
	    q.push(next,-ncost);
	  }
	  else {
	    q.adjust(next,-ncost);
	  }
	}
      }
      else {
	if(ncost < distances.values[next]) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Hmm... better path to complete node!\n");
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Difference: "<<ncost<<" vs "<<distances.values[next]);
	  distances.values[next] = ncost;
	}
      }
    }
#if DO_TIMING
    propagateTime += timer.ElapsedTime();
    timer.Reset();
#endif // DO_TIMING
  }
#if DO_TIMING
  LOG4CXX_INFO(KrisLibrary::logger(),"Time for init "<<initTime<<", propagate "<<propagateTime<<", estimate "<<estimateTime<<", overhead "<<overheadTime);
#endif //DO_TIMING

  //couldn't find goal
  return false;
}

Real Distance(const Vector& x,const vector<int>& gridpt)
{
  Assert(x.size()==(int)gridpt.size());
  Real d2 = 0;
  for(int i=0;i<x.size();i++)
    d2 += Sqr(x[i] - Real(gridpt[i]));
  return Sqrt(d2);
}

//computes the set of grid points that are adjacent to a coordinate x.
//if axisbehavior = 0, if an axis of x is identically on a grid line,
//it only enumerates along that grid line.
//if axisbehavior = 1, if an axis of x is identically on a grid line, it
//enumerates the line, the line above, and the line below
void CoordinatesToGridPoints(const Vector& x,const vector<int>& gridDims,vector<vector<int> >& cells,int axisBehavior = 0)
{
  cells.resize(0);
  vector<vector<int> > axisvals(x.n);
  vector<int> numAxisVals(x.n);
  for(int i=0;i<x.n;i++) {
    int lo = (int)Floor(x(i));
    int hi = lo+1;
    if(x(i) == Real(lo)) {
      //on axis
      if(axisBehavior == 0)
	hi=lo; 
      else
	lo = lo-1;
    }
    if(hi < 0 || lo >= gridDims[i])
      return;
    if(lo < 0) lo = 0;
    if(hi >= gridDims[i]) hi = gridDims[i]-1;
    while (lo <= hi) {
      axisvals[i].push_back(lo);
      lo++;
    }
    numAxisVals[i] = (int)axisvals[i].size();
  }
  //return cartesian product of axis values
  vector<int> index(x.n,0);
  do {
    vector<int> cell(x.n);
    for(int i=0;i<x.n;i++)
      cell[i] = axisvals[i][index[i]];
    cells.push_back(cell);
  } while(!IncrementIndex(index,numAxisVals));
}

//tests whether the grid cell is adjacent to the coordinate x
bool CoordinatesAreAdjacent(const Vector& x,const vector<int>& cell)
{
  Assert(x.size()==(int)cell.size());
  for(int i=0;i<x.n;i++) {
    int lo = (int)Floor(x(i));
    int hi = lo+1;
    if(x(i) == Real(lo)) hi=lo; //on axis
    if(cell[i] != lo && cell[i] != hi) return false;
  }
  return true;
}

bool FMMSearch(const Vector& start,const Vector& goal,const ArrayND<Real>& costs,ArrayND<Real>& distances)
{
#if DO_TIMING
  Timer timer;
  double initTime=0,estimateTime=0,propagateTime=0,overheadTime=0;
#endif
  int numVisited = 0,numSimplices = 0;
  assert(start.size() == (int)costs.dims.size());

  //initialize search status
  ArrayND<int> status;
  status.resize(costs.dims);
  status.set(0);
  distances.resize(costs.dims);
  distances.set(Inf);

  //build queue
  //int goalVirtualIndex = (int)status.numValues();
  //FixedSizeHeap<Real> q(status.numValues()+1);
  FixedSizeHeap<Real> q(status.numValues());
  //add all grid nodes adjacent to the start
  vector<vector<int> > scells,gcells;
  CoordinatesToGridPoints(start,costs.dims,scells);
  CoordinatesToGridPoints(goal,costs.dims,gcells);
  for(size_t i=0;i<scells.size();i++) {
    int sindex = costs.indexToOffset(scells[i]);
    q.push(sindex,0);
    status.values[sindex] = 1;
  }
  //new method: reach all goal cells
  set<int> gIndices;
  for(size_t i=0;i<gcells.size();i++) {
    int gindex = distances.indexToOffset(gcells[i]);
    gIndices.insert(gindex);
  }
  vector<int> adjacent;
  vector<int> simplex;
  Vector d;

#if DO_TIMING
  initTime = timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  while(!q.empty()) {
    int nindex = q.top();
    Real npriority = q.topPriority();
    q.pop();
    numVisited++;

    //if(nindex == goalVirtualIndex) {
    if(gIndices.count(nindex) != 0) {
      gIndices.erase(gIndices.find(nindex));
      if(gIndices.empty()) {
#if DO_TIMING
	LOG4CXX_INFO(KrisLibrary::logger(),"Time for init "<<initTime<<", propagate "<<propagateTime<<", estimate "<<estimateTime<<", overhead "<<overheadTime);
#endif //DO_TIMING
	LOG4CXX_INFO(KrisLibrary::logger(),""<<numVisited<<" nodes visited, avg # of simplices "<<Real(numSimplices)/Real(numVisited));
	return true;
      }
    }

    //set to visited
    status.values[nindex] = -1;

    //look at visited neighbors
    Real best = Inf;
    Real c = costs.values[nindex];
    vector<int> node=costs.offsetToIndex(nindex);
    if (npriority == 0) //it's a start node
      best = Distance(start,node)*c; 

#if DO_TIMING
    overheadTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING
    if(npriority != 0) {
      SimplexEnumerator s(node,status,-1);
      do {
	numSimplices++;
	s.getOffsets(simplex);
	Real dcand;
	if(simplex.size() == 1)
	  dcand = distances.values[simplex[0]]+c;
	else {
	  d.resize(simplex.size());
	  for(size_t i=0;i<simplex.size();i++)
	    d[i] = distances.values[simplex[i]];
	  dcand = best_diag_distanceN(d)*c;
	}
	if(dcand < best) {
	  best = dcand;
	  s.prune(best,distances);
	}
      } while(s.next());
#if DO_TIMING
      estimateTime += timer.ElapsedTime();
      timer.Reset();
#endif //DO_TIMING
    }

    distances.values[nindex] = best;

    //otherwise, propagate to children
    Adjacent(node,nindex,costs,adjacent);
    for(size_t i=0;i<adjacent.size();i++) {
      int next = adjacent[i];
      Real ncost = best+costs.values[next];
      if(IsInf(ncost)) continue;
      if(status.values[next]==0) {
	q.push(next,-ncost);
	distances.values[next] = ncost;
	status.values[next] = 1;
      }
      else if(status.values[next]==1) {
	//do we have a better distance than the existing?
	if (ncost < distances.values[next]) {
	  if(!q.find(next)) {
	    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, want to adjust "<<next);
	    q.push(next,-ncost);
	  }
	  else {
	    q.adjust(next,-ncost);
	  }
	}
      }
      else {
	if(ncost < distances.values[next]) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Hmm... better path to complete node!\n");
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Difference: "<<ncost<<" vs "<<distances.values[next]);
	  distances.values[next] = ncost;
	}
      }
      /*
      //test whether this node is at the goal cell
      if(goal.size() == start.size()) {
	bool inGoal = CoordinatesAreAdjacent(goal,node);
	if(inGoal) {
	  Real ncost = distances.values[nindex] + c*Distance(goal,node);
	  if(!q.find(next)) 
	    q.push(goalVirtualIndex,-ncost);
	  else
	    q.adjust(goalVirtualIndex,-ncost);
	}
      }
      */
    }
#if DO_TIMING
    propagateTime += timer.ElapsedTime();
    timer.Reset();
#endif // DO_TIMING
  }
#if DO_TIMING
  LOG4CXX_INFO(KrisLibrary::logger(),"Time for init "<<initTime<<", propagate "<<propagateTime<<", estimate "<<estimateTime<<", overhead "<<overheadTime);
#endif //DO_TIMING

  //couldn't find goal
  return false;
}



bool FMMSearch(const Vector& startorig,const Vector& goalorig,
	       const Vector& bmin,const Vector& bmax,const Vector& res,
	       Real (*costFn)(const Vector& coords),ArrayND<Real>& distances)
{
#if DO_TIMING
  Timer timer;
  double initTime=0,estimateTime=0,propagateTime=0,overheadTime=0;
#endif
  int numVisited = 0,numSimplices = 0;
  assert(startorig.size() == res.size());
  assert(startorig.size() == bmin.size());
  assert(startorig.size() == bmax.size());
  vector<int> dims(res.size());
  for(int i=0;i<res.n;i++) {
    dims[i] = (int)Ceil((bmax[i]-bmin[i])/res[i]);
    if(dims[i] == ((bmax[i]-bmin[i])/res[i])) //upper bound is identically an integer
      dims[i] ++;
  }
  //normalize start and goal
  Vector start,goal;
  start.resize(startorig.n);
  for(int i=0;i<res.n;i++)
    start[i] = (startorig[i] - bmin[i])/res[i];
  goal.resize(goalorig.n);
  for(int i=0;i<goal.n;i++)
    goal[i] = (goalorig[i] - bmin[i])/res[i];

  //initialize search status
  ArrayND<int> status;
  status.resize(dims);
  status.set(0);
  distances.resize(dims);
  distances.set(Inf);

  //build queue
  //int goalVirtualIndex = (int)status.numValues();
  //FixedSizeHeap<Real> q(status.numValues()+1);
  FixedSizeHeap<Real> q(status.numValues());
  //add all grid nodes adjacent to the start
  vector<vector<int> > scells, gcells;
  CoordinatesToGridPoints(start,dims,scells);
  CoordinatesToGridPoints(goal,dims,gcells);
  for(size_t i=0;i<scells.size();i++) {
    int sindex = distances.indexToOffset(scells[i]);
    q.push(sindex,0);
    status.values[sindex] = 1;
  }
  //new method: reach all goal cells
  set<int> gIndices;
  for(size_t i=0;i<gcells.size();i++) {
    int gindex = distances.indexToOffset(gcells[i]);
    gIndices.insert(gindex);
  }
  vector<int> adjacent;
  vector<int> simplex;
  Vector d;

#if DO_TIMING
  initTime = timer.ElapsedTime();
  timer.Reset();
#endif //DO_TIMING

  while(!q.empty()) {
    int nindex = q.top();
    Real npriority = q.topPriority();
    q.pop();
    numVisited++;

    //if(nindex == goalVirtualIndex) {
    if(gIndices.count(nindex) != 0) {
      gIndices.erase(gIndices.find(nindex));
      if(gIndices.empty()) {
#if DO_TIMING
	LOG4CXX_INFO(KrisLibrary::logger(),"Time for init "<<initTime<<", propagate "<<propagateTime<<", estimate "<<estimateTime<<", overhead "<<overheadTime);
#endif //DO_TIMING
	LOG4CXX_INFO(KrisLibrary::logger(),""<<numVisited<<" nodes visited, avg # of simplices "<<Real(numSimplices)/Real(numVisited));
	return true;
      }
    }

    //set to visited
    status.values[nindex] = -1;
    vector<int> node=distances.offsetToIndex(nindex);
    //compute node
    Vector npt(node.size());
    for(int i=0;i<npt.n;i++)
      npt[i] = bmin(i) + node[i]*res[i];

    //look at visited neighbors
    Real best = Inf;
    Real c = costFn(npt);
    if (npriority == 0) //it's a start node
      best = Distance(start,node)*c; 

#if DO_TIMING
    overheadTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING
    if(npriority != 0) {
      SimplexEnumerator s(node,status,-1);
      do {
	numSimplices++;
	s.getOffsets(simplex);
	Real dcand;
	if(simplex.size() == 1)
	  dcand = distances.values[simplex[0]]+c;
	else {
	  d.resize(simplex.size());
	  for(size_t i=0;i<simplex.size();i++)
	    d[i] = distances.values[simplex[i]];
	  dcand = best_diag_distanceN(d)*c;
	}
	if(dcand < best) {
	  best = dcand;
	  s.prune(best,distances);
	}
      } while(s.next());
#if DO_TIMING
      estimateTime += timer.ElapsedTime();
      timer.Reset();
#endif //DO_TIMING
    }

    distances.values[nindex] = best;

    //otherwise, propagate to children
    Adjacent(node,nindex,distances,adjacent);
    for(size_t i=0;i<adjacent.size();i++) {
      int next = adjacent[i];
      //compute node coodinates to get cost
      vector<int> nextnode=distances.offsetToIndex(next);
      Vector nextpt(node.size());
      for(int i=0;i<nextpt.n;i++)
	nextpt[i] = bmin(i) + nextnode[i]*res[i];
      Real ncost = best+costFn(nextpt);
      if(IsInf(ncost)) continue;
      if(status.values[next]==0) {
	q.push(next,-ncost);
	distances.values[next] = ncost;
	status.values[next] = 1;
      }
      else if(status.values[next]==1) {
	//do we have a better distance than the existing?
	if (ncost < distances.values[next]) {
	  if(!q.find(next)) {
	    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, want to adjust "<<next);
	    q.push(next,-ncost);
	  }
	  else {
	    q.adjust(next,-ncost);
	  }
	}
      }
      else {
	if(ncost < distances.values[next]) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Hmm... better path to complete node!\n");
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Difference: "<<ncost<<" vs "<<distances.values[next]);
	  distances.values[next] = ncost;
	}
      }
      /*
      //test whether this node is at the goal cell
      if(goal.size() == start.size()) {
	bool inGoal = CoordinatesAreAdjacent(goal,node);
	if(inGoal) {
	  Real ncost = distances.values[nindex] + c*Distance(goal,node);
	  if(!q.find(goalVirtualIndex)) 
	    q.push(goalVirtualIndex,-ncost);
	  else
	    q.adjust(goalVirtualIndex,-ncost);
	}
      }
      */
    }
#if DO_TIMING
    propagateTime += timer.ElapsedTime();
    timer.Reset();
#endif // DO_TIMING
  }
#if DO_TIMING
  LOG4CXX_INFO(KrisLibrary::logger(),"Time for init "<<initTime<<", propagate "<<propagateTime<<", estimate "<<estimateTime<<", overhead "<<overheadTime);
#endif //DO_TIMING

  //couldn't find goal
  return false;
}





/** Multilinear interpolation of an ND field.
* Sensitive to Inf's in the field -- will ignore them
*/
Real EvalMultilinear(const ArrayND<Real>& field,const Vector& point)
{
  vector<int> low(point.size());
  Vector u(point.n);
  for(size_t i=0;i<low.size();i++) {
    Real fp = Floor(point[i]);
    low[i] = (int)fp;
    u[i] = point[i] - fp;

    if(low[i] < 0) {
      low[i] = 0;
      u[i] = 0;
    }
    else if(low[i] >= field.dims[i]-1) {
      low[i] = field.dims[i]-2;
      u[i] = 1;
    }
  }
  vector<int> high(point.size());
  for(size_t i=0;i<low.size();i++) {
    high[i] = low[i]+1;
    if(low[i] < 0) //grid on that dimension has size 1
      low[i] = 0;
  }

  vector<int> vertex(low);
  Real s = 0.0;
  Real sumcoeff = 0.0;
  do {
    Real f=field[vertex];
    Real coeff = 1.0;
    for(int i=0;i<u.size();i++) {
      Real axisCoeff = (vertex[i]==low[i]? 1.0-u[i] : u[i]);
      coeff *= axisCoeff;
    }
    if(IsInf(f)) {
      if(coeff != 0) return Inf;
      else continue;
    }
    s += coeff*f;
    sumcoeff += coeff;
  } while(!IncrementIndex(vertex,low,high));
  if (sumcoeff == 0.0) return Inf;
  return s/sumcoeff;
}

#define INF_POS 1
#define INF_NEG 2

Vector FiniteDifference(const ArrayND<Real>& field,const Vector& x,vector<int>& infDirs)
{
  infDirs.resize(x.n);
  fill(infDirs.begin(),infDirs.end(),0);
  Real h = 0.25;
  Vector grad(x.size());
  Vector tmp = x;
  for(int i=0;i<x.size();i++) {
    Real cell = Floor(x[i]);
    Real cellmin = Max((x[i] == cell ? cell-1.0 : cell),0.0);
    Real cellmax = Min(cell+1.0,Real(field.dims[i])-1.0);
    //Real cellmin = 0.0;
    //Real cellmax = Real(field.dims[i])-1.0;
    tmp[i] = Min(x[i]+h,cellmax);
    Real f2 = EvalMultilinear(field,tmp);
    Real d = tmp[i] - x[i];
    if(IsInf(f2)) {
      infDirs[i] |= INF_POS;
      f2 = EvalMultilinear(field,x);
      d = 0.0;
    }
    tmp[i] = Max(x[i]-h,cellmin);
    Real f1 = EvalMultilinear(field,tmp);
    if(IsInf(f1)) {
      infDirs[i] |= INF_NEG;
      f1 = EvalMultilinear(field,x);
    }
    else
      d += x[i] - tmp[i];
    tmp[i] = x[i];
    if(d == 0.0) grad[i] = 0.0;
    else grad[i] = (f2-f1)/d;
  }
  return grad;
}

/** Gradient descent of an ND field */
vector<Vector> GradientDescent(const ArrayND<Real>& field,const Vector& start)
{
  Vector pt = start;
  vector<Vector> path;
  while(1) {
    path.push_back(pt);
    vector<int> infDirs;
    Vector grad = FiniteDifference(field,pt,infDirs);
    Real t=Inf;
    //do a line search to find the step size that steps out of the cell
    int bdry = 0;
    for(int i=0;i<pt.size();i++) {
      if(pt[i] <= 0 && grad[i] > 0) grad[i]=0.0;
      if(pt[i] >= field.dims[i]-1 && grad[i] < 0) grad[i]=0.0;
      if(infDirs[i] & INF_NEG && grad[i] > 0) grad[i]=0;
      if(infDirs[i] & INF_POS && grad[i] < 0) grad[i]=0;
      if(grad[i] == 0) continue;
      Real cell = Floor(pt[i]);
      if(pt[i] == cell) { //on this boundary, allow backwards movement
                if(pt[i]-t*grad[i] < cell-1) {
                  t = (pt[i]-cell+1)/grad[i];
                  bdry = i;
                }
      }
      else {
                if(pt[i]-t*grad[i] < cell) {
                  t = (pt[i]-cell)/grad[i];
                  bdry = i;
                }
      }
      if(pt[i]-t*grad[i] > cell+1) {
                t = (pt[i]-cell-1)/grad[i];
                bdry = i;
      }
    }
    if(IsInf(t)) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Terminated search at "<<pt<<", gradient is zero");
      return path;
    }
    Vector next = pt-grad*t;
    //round to nearest grid cell
    next[bdry] = Real(int(next[bdry]));
    /*
    for(size_t i=0;i<next.size();i++) {
      if(next[i] < 0.0) next[i] = 0.0;
      if(next[i] >= field.dims[i]) next[i] = field.dims[i]-1;
    }
    */
    Real curValue = EvalMultilinear(field,pt);
    if (EvalMultilinear(field,next) >= curValue) {
      //just go to the best nearest gridpoint
      vector<vector<int> > gridpts;
      CoordinatesToGridPoints(pt,field.dims,gridpts,1);
      Real bestVal = curValue;
      int best = -1;
      for(size_t i=0;i<gridpts.size();i++) {
	Real v=field[gridpts[i]];
	if(v < bestVal) {
	  bestVal = v;
	  best = (int)i;
	}
      }
      if(best < 0) {
	/*
	LOG4CXX_INFO(KrisLibrary::logger(),"Terminated, next point "<<next<<" has cost "<<EvalMultilinear(field,next)<<", increase from "<<EvalMultilinear(field,pt)<<" at "<<pt);
	LOG4CXX_INFO(KrisLibrary::logger(),"Gradient: "<<grad);
	LOG4CXX_INFO(KrisLibrary::logger(),"inf dirs: ");
	for(size_t i=0;i<infDirs.size();i++)
	  LOG4CXX_INFO(KrisLibrary::logger(),infDirs[i]<<" ");
	LOG4CXX_INFO(KrisLibrary::logger(),"\n");
	LOG4CXX_INFO(KrisLibrary::logger(),"Adjacent cell values: ");
	for(size_t i=0;i<gridpts.size();i++) {
	  for(size_t k=0;k<gridpts[i].size();k++)
	    LOG4CXX_INFO(KrisLibrary::logger(),gridpts[i][k]<<" ");
	  LOG4CXX_INFO(KrisLibrary::logger(),": "<<field[gridpts[i]]);
	}
	LOG4CXX_INFO(KrisLibrary::logger(),"\n");
	*/
	return path;
      }
      else {
	for(int i=0;i<next.n;i++)
	  next[i] = Real(gridpts[best][i]);
      }
    }
    pt = next;
  }
  return path;
}


/*

struct GridFeature
{
  void SetPoint(const vector<int>& index);
  void SetEdge(const vector<int>& indexlow,int axis);
  void SetQuad(const vector<int>& indexlow,int axis1,int axis2);
  void SetFacet(const vector<int>& indexlow,vector<int>& axes);
  void GetAxisParticipation(vector<bool>& participation) const;
  void GetParents(vector<GridFeature>& parents) const; 
  void GetChildren(vector<GridFeature>& children) const; 
  void GetAdjacentCells(vector<vector<int> >& cells) const;

  //bottom corner of the feature
  vector<int> indexlow;
  vector<int> axes;
};

void GridFeature::SetPoint(const vector<int>& index)
{
  indexlow = index;
  axes.resize(0);
}


void GridFeature::SetEdge(const vector<int>& _indexlow,int axis)
{
  indexlow = _indexlow;
  axes.resize(1);
  axes[0] = axis;
}

void GridFeature::SetQuad(const vector<int>& _indexlow,int axis1,int axis2)
{
  indexlow = _indexlow;
  axes.resize(2);
  axes[0] = axis1;
  axes[1] = axis2;
}

void GridFeature::SetFacet(const vector<int>& _indexlow,vector<int>& _axes)
{
  indexlow = _indexlow;
  axes = _axes;
}

void GridFeature::GetAxisParticipation(vector<bool>& participation) const
{
  participation.resize(indexlow.size());
  fill(participation.begin(),participation.end(),false);
  for(size_t i=0;i<axes.size();i++)
    participation[axes[i]] = true;
}

void GridFeature::GetParents(vector<GridFeature>& parents) const
{
  size_t navail = indexlow.size()-axes.size();
  parents.resize(navail*2);
  int k=0;
  vector<bool> participation;
  GetAxisParticipation(participation);
  for(size_t i=0;i<indexlow.size();i++)
    if(!participation[i]) {
      parents[k*2].indexlow = indexlow;
      parents[k*2+1].indexlow = indexlow;
      parents[k*2+1].indexlow[i] -= 1;
      parents[k*2].axes = axes;
      parents[k*2].axes.push_back(i);
      parents[k*2+1].axes = parents[k*2].axes;
      k++;
    }
}

void GridFeature::GetChildren(vector<GridFeature>& children) const
{
  children.resize(axes.size()*2);
  for(size_t i=0;i<axes.size();i++) {
    children[i*2].indexlow = indexlow;
    children[i*2+1].indexlow = indexlow;
    children[i*2+1].indexlow[axes[i]] += 1;
    children[i*2].axes.reserve(axes.size()-1);
    children[i*2+1].axes.reserve(axes.size()-1);
    for(size_t j=0;j<axes.size();j++) {
      if(j!=i) {
	children[i*2].axes.push_back(j);
	children[i*2+1].axes.push_back(j);
      }
    }
  }
}

void GridFeature::GetAdjacentCells(vector<vector<int> >& cells) const
{
  vector<bool> participation;
  GetAxisParticipation(participation);
  vector<int> low=indexlow,high=indexlow;
  for(size_t i=0;i<participation.size();i++) {
    if(!participation[i])
      low[i] -= 1;
  }

  cells.resize(0);
  //already maximal
  if(low == high) return;
  //enumerate adjacent
  vector<int> vertex = low;
  do {
    cells.push_back(vertex);
  } while(!IncrementIndex(vertex,low,high));
}

*/
