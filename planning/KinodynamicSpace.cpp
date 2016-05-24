#include "KinodynamicSpace.h"
#include "KinodynamicPath.h"
#include "EdgePlannerHelpers.h"
#include "InterpolatorHelpers.h"
#include <math/diffeq.h>
#include <math/random.h>
#include <Timer.h>
using namespace std;


KinodynamicSpace::KinodynamicSpace(const SmartPointer<CSpace>& xspace,const SmartPointer<ControlSpace>& uspace)
:stateSpace(xspace),controlSpace(uspace)
{}


IntegratedKinodynamicSpace::IntegratedKinodynamicSpace(const SmartPointer<CSpace>& xspace,IntegratedControlSpace* _controlSpace)
:KinodynamicSpace(xspace,_controlSpace),controlSpace(_controlSpace)
{}

EdgePlanner* IntegratedKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path)
{
  Real udt = u(0);
  int numSteps = int(controlSpace->dt / udt);
  return new EpsilonEdgeChecker(GetStateSpace(),path,path->Length()/numSteps);
}

RandomBiasSteeringFunction::RandomBiasSteeringFunction(KinodynamicSpace* _space,int _sampleCount)
:space(_space),sampleCount(_sampleCount)
{}

bool RandomBiasSteeringFunction::Connect(const State& x,const State& xGoal,KinodynamicMilestonePath& path)
{
  //Timer timer;
  Real closest=Inf;
  ControlInput temp;
  
  ControlInput u;
  SmartPointer<Interpolator> bestpath;
  SmartPointer<Interpolator> pathtemp;
  for(int i=0;i<sampleCount;i++) {
    SmartPointer<CSet> uspace = space->controlSpace->GetControlSet(x);
    if(!uspace) continue;
    uspace->Sample(temp);
    if(temp.empty()) { fprintf(stderr,"Warning, control space does not have Sample() method implemented\n"); return false; }
    if(!uspace->Contains(temp)) continue;
    pathtemp = space->controlSpace->Simulate(x,temp);
    if(!pathtemp) continue;
    Real dist = space->GetStateSpace()->Distance(pathtemp->End(),xGoal);
    if(dist < closest) {
      closest = dist;
      u = temp;
      bestpath = pathtemp;
    }
  }
  if(bestpath == NULL) {
    //printf("Failed to connect in time %g\n",timer.ElapsedTime());
    return false;
  }
  path = KinodynamicMilestonePath(u,bestpath);
  path.SimulateFromControls(space);
  //printf("Connect in time %g\n",timer.ElapsedTime());
  return true;
}

RandomBiasReverseSteeringFunction::RandomBiasReverseSteeringFunction(KinodynamicSpace* _space,int _sampleCount)
:space(_space),sampleCount(_sampleCount)
{}

bool RandomBiasReverseSteeringFunction::Connect(const State& x,const State& xGoal,KinodynamicMilestonePath& path)
{
  Real closest=Inf;
  ControlInput temp;
  
  ReversibleControlSpace* rspace = dynamic_cast<ReversibleControlSpace*>(&(*space->controlSpace));
  if(!rspace) return false;

  ControlInput u;
  SmartPointer<Interpolator> bestpath;
  SmartPointer<Interpolator> pathtemp;
  for(int i=0;i<sampleCount;i++) {
    SmartPointer<CSet> uspace = rspace->reverseControlSpace->GetControlSet(x);
    if(!uspace) continue; 
    uspace->Sample(temp);
    if(temp.empty()) { fprintf(stderr,"Warning, control space does not have Sample() method implemented\n"); return false; }
    if(!uspace->Contains(temp)) continue;
    pathtemp = rspace->reverseControlSpace->Simulate(x,temp);
    if(!pathtemp) continue;
    Real dist = space->GetStateSpace()->Distance(xGoal,pathtemp->End());
    if(dist < closest) {
      closest = dist;
      u = temp;
      bestpath = pathtemp;
    }
  }
  if(bestpath == NULL) return false;
  path = KinodynamicMilestonePath(u,bestpath);
  path.MakePaths(rspace->reverseControlSpace);
  return true;
}

KinodynamicSpace::~KinodynamicSpace() {}

EdgePlanner* KinodynamicSpace::TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path)
{
  if(dynamic_cast<const LinearInterpolator*>(&*path) != NULL)
    return GetStateSpace()->PathChecker(path->Start(),path->Start());
  else if(dynamic_cast<const CSpaceInterpolator*>(&*path) != NULL)
    return GetStateSpace()->PathChecker(path->Start(),path->Start());
  else if(dynamic_cast<const PiecewiseLinearInterpolator*>(&*path) != NULL) {
    const PiecewiseLinearInterpolator* interp = dynamic_cast<const PiecewiseLinearInterpolator*>(&*path);
    vector<SmartPointer<EdgePlanner> > epath(interp->configs.size()-1);
    for(size_t i=0;i+1<interp->configs.size();i++)
      epath[i] = GetStateSpace()->PathChecker(interp->configs[i],interp->configs[i+1]);
    return new PathEdgeChecker(GetStateSpace(),epath);
  }
  else if(dynamic_cast<const PiecewiseLinearCSpaceInterpolator*>(&*path) != NULL) {
    const PiecewiseLinearCSpaceInterpolator* interp = dynamic_cast<const PiecewiseLinearCSpaceInterpolator*>(&*path);
    vector<SmartPointer<EdgePlanner> > epath(interp->configs.size()-1);
    for(size_t i=0;i+1<interp->configs.size();i++)
      epath[i] = GetStateSpace()->PathChecker(interp->configs[i],interp->configs[i+1]);
    return new PathEdgeChecker(GetStateSpace(),epath);
  }
  else FatalError("KinodynamicSpace::TrajectoryChecker: being asked to produce a trajectory checker for a non-piecewise linear path.  Subclass needs to overload this");
  return NULL;
}

EdgePlanner* KinodynamicSpace::TrajectoryChecker(const KinodynamicMilestonePath& path)
{
  vector<SmartPointer<EdgePlanner> > individualCheckers(path.paths.size());
  for(size_t i=0;i<path.paths.size();i++)
    individualCheckers[i] = TrajectoryChecker(path.controls[i],path.paths[i]);
  return new PathEdgeChecker(GetStateSpace(),individualCheckers);
}

bool KinodynamicSpace::NextState(const State& x0,const ControlInput& u,State& x)
{
  SmartPointer<Interpolator> p = controlSpace->Simulate(x0,u);
  x = p->End();
  if(!GetStateSpace()->IsFeasible(x)) return false;
  EdgePlanner* e=TrajectoryChecker(u,p);
  bool res=e->IsVisible();
  delete e;
  return res;
}

bool KinodynamicSpace::PreviousState(const State& x1,const ControlInput& u,State& x0)
{
  ReversibleControlSpace* rspace = dynamic_cast<ReversibleControlSpace*>(&(*controlSpace));
  if(!rspace) return false;
  SmartPointer<Interpolator> p = rspace->reverseControlSpace->Simulate(x1,u);
  x0 = p->End();
  if(!GetStateSpace()->IsFeasible(x0)) return false;
  EdgePlanner* e=TrajectoryChecker(u,p);
  bool res=e->IsVisible();
  delete e;
  return res;
}

void KinodynamicSpace::Properties(PropertyMap& map) const
{
  GetStateSpace()->Properties(map);
  map.set("dynamic",1);
}







KinematicCSpaceAdaptor::KinematicCSpaceAdaptor(const SmartPointer<CSpace>& base,Real maxNeighborhoodRadius)
:KinodynamicSpace(base,new KinematicControlSpace(base,maxNeighborhoodRadius))
{}




KinodynamicSteeringCSpaceAdaptor::KinodynamicSteeringCSpaceAdaptor(const SmartPointer<KinodynamicSpace>& _kinodynamicSpace)
:PiggybackCSpace(_kinodynamicSpace->GetStateSpace()),kinodynamicSpace(_kinodynamicSpace)
{
  steeringFunction = kinodynamicSpace->GetControlSpace()->GetSteeringFunction();
  Assert(steeringFunction != NULL);
  Assert(steeringFunction->IsExact());
}

EdgePlanner* KinodynamicSteeringCSpaceAdaptor::PathChecker(const Config& a,const Config& b)
{
  KinodynamicMilestonePath path;
  if(!steeringFunction->Connect(a,b,path)) return new FalseEdgeChecker(this,a,b);
  return kinodynamicSpace->TrajectoryChecker(path);
}

EdgePlanner* KinodynamicSteeringCSpaceAdaptor::PathChecker(const Config& a,const Config& b,int constraint)
{
  FatalError("Can't do single obstacle path checker for a KinodynamicSteeringCSpaceAdaptor yet");
  return NULL;
}

Real KinodynamicSteeringCSpaceAdaptor::Distance(const Config& x, const Config& y)
{
  KinodynamicMilestonePath path;
  if(!steeringFunction->Connect(x,y,path)) return Inf;
  return path.Length();
}

void KinodynamicSteeringCSpaceAdaptor::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  KinodynamicMilestonePath path;
  if(!steeringFunction->Connect(x,y,path)) {
    out.resize(x.n);
    out.set(Inf);
    return;
  }
  path.Eval(u,out);
  return;
}

void KinodynamicSteeringCSpaceAdaptor::Properties(PropertyMap& map) 
{
  PiggybackCSpace::Properties(map);
  map.set("euclidean",0);
  map.set("geodesic",steeringFunction->IsOptimal());
  map.set("metric","steering function length");
  map.remove("diameter");
}
