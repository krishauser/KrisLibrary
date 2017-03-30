#include "FMMMotionPlanner.h"
#include <utils/indexing.h>
#include <Timer.h>

///binding for fMM
static CSpace* currentFMMSpace = NULL;
Real FMMCost(const Vector& coords)
{
  if(!currentFMMSpace) return 1;
  if(currentFMMSpace->IsFeasible(coords)) return 1;
  return Inf;
}

FMMMotionPlanner::FMMMotionPlanner(CSpace* _space)
  :space(_space),dynamicDomain(true)
{}

FMMMotionPlanner::FMMMotionPlanner(CSpace* _space,const Vector& _bmin,const Vector& _bmax,int divs)
  :space(_space),bmin(_bmin),bmax(_bmax),dynamicDomain(false)
{
  resolution = bmax-bmin;
  resolution *= 1.0/divs;
}

void FMMMotionPlanner::Init(const Config& a,const Config& b)
{
  start = a;
  goal = b;
  distances.clear();
  solution.edges.clear();

  if(dynamicDomain) {
    Real d = space->Distance(a,b);
    bmin = a;
    bmax = b;
    for(int i=0;i<bmin.n;i++)
      bmin[i] = Min(a[i],b[i])-d*0.5;
    for(int i=0;i<bmax.n;i++)
      bmax[i] = Max(a[i],b[i])+d*0.5;
    if(resolution.empty()) {
      PropertyMap props;
      space->Properties(props);
      vector<Real> weights;
      if(props.getArray("metricWeights",weights)) {
	Assert((int)weights.size()==a.n);
	resolution.resize(a.n);
	for(int i=0;i<bmin.n;i++)
	  resolution[i] = d*0.25/weights[i];
      }
      else {
	resolution.resize(a.n);
	resolution.set(d * 0.25);
      }
    }
  }
}


bool FreeLower(const ArrayND<Real>& distances,int axis)
{
  //iterate over the a cartesian product of ranges
  vector<int> ranges = distances.dims;
  ranges[axis] = 1;
  vector<int> index(distances.dims.size(),0),indexlo;
  do {
    indexlo = index;
    indexlo[axis] = 0;
    if(!IsInf(distances[index])) return true;
  } while(!IncrementIndex(index,ranges));
  return false;
}

bool FreeUpper(const ArrayND<Real>& distances,int axis)
{
  //iterate over the a cartesian product of ranges
  vector<int> ranges = distances.dims;
  ranges[axis] = 1;
  vector<int> index(distances.dims.size(),0),indexhi;
  do {
    indexhi = index;
    indexhi[axis] = distances.dims[axis]-1;
    if(!IsInf(distances[index])) return true;
  } while(!IncrementIndex(index,ranges));
  return false;
}

Vector FMMMotionPlanner::ToGrid(const Vector& q) const
{
  Vector res = q-bmin;
  for(int i=0;i<resolution.n;i++)
    res[i] /= resolution[i];
  return res;
}

Vector FMMMotionPlanner::FromGrid(const Vector& q) const
{
  Vector res=q;
  for(int i=0;i<q.n;i++)
    res[i] *= resolution[i];
  res += bmin;
  return res;
}

Vector FMMMotionPlanner::FromGrid(const vector<int>& cell) const
{
  Vector res(cell.size());
  for(int i=0;i<res.n;i++)
    res[i] = cell[i]*resolution[i] + bmin[i];
  return res;
}


bool FMMMotionPlanner::SolveFMM()
{
  Assert(start.n == goal.n);
  Assert(start.n == bmin.n);
  Assert(start.n == bmax.n);
  Assert(start.n == resolution.n);

  if(dynamicDomain && distances.numValues() > 0) {
    //check distances, if there are any non-inf along an edge then that edge should be expanded
    for(size_t i=0;i<distances.dims.size();i++) {
      Real w=(bmax[i]-bmin[i]);
      if(FreeLower(distances,i)) {
	bmin[i] -= w*0.25;
	printf("Decreasing bottom domain %d by %g\n",i,w*0.25);
      }
      if(FreeUpper(distances,i)) {
	bmax[i] += w*0.25;
	printf("Increasing top domain %d by %g\n",i,w*0.25);
      }
    }
  }

  currentFMMSpace = space;
  if(!FMMSearch(start,goal,bmin,bmax,resolution,FMMCost,distances)) {
    printf("FMM search failed\n");
    return false;
  }
  vector<Vector> pts = GradientDescent(distances,ToGrid(goal));
  reverse(pts.begin(),pts.end());
  //convert these grid-space coordinates to configuration space coordinates
  for(size_t i=0;i<pts.size();i++) {
    pts[i] = FromGrid(pts[i]);
  }
  //do the intermediate edge checking
  MilestonePath pathCheck;
  int istart=0;
  Vector* last = &start;
  if(space->Distance(start,pts[0]) < Epsilon) 
    istart++;
  for(size_t i=istart;i<pts.size();i++) {
    pathCheck.edges.push_back(space->LocalPlanner(*last,pts[i]));
    last = &pts[i];
  }
  if(space->Distance(goal,*last) > Epsilon) 
    pathCheck.edges.push_back(space->LocalPlanner(*last,goal));

  bool feasible = true;
  for(size_t i=0;i<pathCheck.edges.size();i++) {
    if(!pathCheck.edges[i]->IsVisible()) {
      cout<<"Path check failed on edge "<<pathCheck.edges[i]->Start()<<" -> "<<pathCheck.edges[i]->End()<<endl;
      feasible = false;
      break;
    }
  }
  if(!feasible) {
    failedCheck = pathCheck;
    return false;
  }
  else {
    if(!solution.edges.empty() && solution.Length() < pathCheck.Length()) {
      printf("SolveFMM: Warning, higher resolution found a longer feasible path: %g vs %g\n",pathCheck.Length(),solution.Length());
      return false;
    }
    else {
      printf("SolveFMM: Found a solution, length %g!\n",pathCheck.Length());
      solution = pathCheck;
    }
  }
  return true;
}

bool FMMMotionPlanner::SolveAnytime(Real time)
{
  int d=start.n;
  Real scaleFactor = Pow(0.5,1.0/Real(d));
  bool hasSolution = false;
  Timer timer;
  while(timer.ElapsedTime() < time) {
    resolution *= scaleFactor;
    if(SolveFMM())
      hasSolution = true;
  }
  return hasSolution;
}
