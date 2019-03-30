#include <KrisLibrary/Logger.h>
#include "KinodynamicSpace.h"
#include "KinodynamicPath.h"
#include "EdgePlannerHelpers.h"
#include "InterpolatorHelpers.h"
#include <math/diffeq.h>
#include <math/random.h>
#include <Timer.h>
using namespace std;


KinodynamicSpace::KinodynamicSpace(const std::shared_ptr<CSpace>& xspace,const std::shared_ptr<ControlSpace>& uspace)
:stateSpace(xspace),controlSpace(uspace)
{}


IntegratedKinodynamicSpace::IntegratedKinodynamicSpace(const std::shared_ptr<CSpace>& xspace,const std::shared_ptr<IntegratedControlSpace>& _controlSpace)
:KinodynamicSpace(xspace,_controlSpace)
{}

EdgePlannerPtr IntegratedKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const InterpolatorPtr& path)
{
  auto ics = dynamic_cast<IntegratedControlSpace*>(controlSpace.get());
  Real udt = u(0);
  int numSteps = int(ics->dt / udt);
  return make_shared<EpsilonEdgeChecker>(GetStateSpace().get(),path,path->Length()/numSteps);
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
  InterpolatorPtr bestpath;
  InterpolatorPtr pathtemp;
  std::shared_ptr<CSet> uspace = space->controlSpace->GetControlSet(x);
  if(!uspace) {
    LOG4CXX_WARN(KrisLibrary::logger(),"RandomBiasSteeringFunction::Connect(): Warning, no control set at "<<x<<"?");
    return false;
  }
  for(int i=0;i<sampleCount;i++) {
    uspace->Sample(temp);
        if(temp.empty()) { LOG4CXX_ERROR(KrisLibrary::logger(),"RandomBiasSteeringFunction::Connect(): Warning, control space does not have Sample() method implemented\n"); return false; }
    if(!uspace->Contains(temp)) {
      LOG4CXX_WARN(KrisLibrary::logger(),"RandomBiasSteeringFunction::Connect(): Warning, sampled infeasible control "<<temp<<"?");
      continue;
    }
    pathtemp = space->controlSpace->Simulate(x,temp);
    if(!pathtemp) {
      LOG4CXX_WARN(KrisLibrary::logger(),"RandomBiasSteeringFunction::Connect(): Warning, simulated path is null?");
      continue;
    }
    Real dist = space->GetStateSpace()->Distance(pathtemp->End(),xGoal);
    if(dist < closest) {
      closest = dist;
      u = temp;
      bestpath = pathtemp;
    }
  }
  if(bestpath == NULL) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Failed to connect in time "<<timer.ElapsedTime());
    return false;
  }
  path = KinodynamicMilestonePath(u,bestpath);
  path.SimulateFromControls(space);
  //LOG4CXX_INFO(KrisLibrary::logger(),"Connect in time "<<timer.ElapsedTime());
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
  InterpolatorPtr bestpath;
  InterpolatorPtr pathtemp;
  for(int i=0;i<sampleCount;i++) {
    std::shared_ptr<CSet> uspace = rspace->reverseControlSpace->GetControlSet(x);
    if(!uspace) continue; 
    uspace->Sample(temp);
        if(temp.empty()) { LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, control space does not have Sample() method implemented\n"); return false; }
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
  path.MakePaths(rspace->reverseControlSpace.get());
  return true;
}

KinodynamicSpace::~KinodynamicSpace() {}

EdgePlannerPtr KinodynamicSpace::TrajectoryChecker(const ControlInput& u,const InterpolatorPtr& path)
{
  if(dynamic_cast<const LinearInterpolator*>(&*path) != NULL)
    return GetStateSpace()->PathChecker(path->Start(),path->Start());
  else if(dynamic_cast<const CSpaceInterpolator*>(&*path) != NULL)
    return GetStateSpace()->PathChecker(path->Start(),path->Start());
  else if(dynamic_cast<const PiecewiseLinearInterpolator*>(&*path) != NULL) {
    const PiecewiseLinearInterpolator* interp = dynamic_cast<const PiecewiseLinearInterpolator*>(&*path);
    vector<EdgePlannerPtr> epath(interp->configs.size()-1);
    for(size_t i=0;i+1<interp->configs.size();i++)
      epath[i] = GetStateSpace()->PathChecker(interp->configs[i],interp->configs[i+1]);
    return make_shared<PathEdgeChecker>(GetStateSpace().get(),epath);
  }
  else if(dynamic_cast<const PiecewiseLinearCSpaceInterpolator*>(&*path) != NULL) {
    const PiecewiseLinearCSpaceInterpolator* interp = dynamic_cast<const PiecewiseLinearCSpaceInterpolator*>(&*path);
    vector<EdgePlannerPtr > epath(interp->configs.size()-1);
    for(size_t i=0;i+1<interp->configs.size();i++)
      epath[i] = GetStateSpace()->PathChecker(interp->configs[i],interp->configs[i+1]);
    return make_shared<PathEdgeChecker>(GetStateSpace().get(),epath);
  }
  else FatalError("KinodynamicSpace::TrajectoryChecker: being asked to produce a trajectory checker for a non-piecewise linear path.  Subclass needs to overload this");
  return NULL;
}

EdgePlannerPtr KinodynamicSpace::TrajectoryChecker(const KinodynamicMilestonePath& path)
{
  vector<EdgePlannerPtr> individualCheckers(path.paths.size());
  for(size_t i=0;i<path.paths.size();i++)
    individualCheckers[i] = TrajectoryChecker(path.controls[i],path.paths[i]);
  return make_shared<PathEdgeChecker>(GetStateSpace().get(),individualCheckers);
}

bool KinodynamicSpace::NextState(const State& x0,const ControlInput& u,State& x)
{
  InterpolatorPtr p(controlSpace->Simulate(x0,u));
  x = p->End();
  if(!GetStateSpace()->IsFeasible(x)) return false;
  EdgePlannerPtr e=TrajectoryChecker(u,p);
  return e->IsVisible();
}

bool KinodynamicSpace::PreviousState(const State& x1,const ControlInput& u,State& x0)
{
  ReversibleControlSpace* rspace = dynamic_cast<ReversibleControlSpace*>(&(*controlSpace));
  if(!rspace) return false;
  InterpolatorPtr p(rspace->reverseControlSpace->Simulate(x1,u));
  x0 = p->End();
  if(!GetStateSpace()->IsFeasible(x0)) return false;
  EdgePlannerPtr e=TrajectoryChecker(u,p);
  return e->IsVisible();
}

void KinodynamicSpace::Properties(PropertyMap& map) const
{
  GetStateSpace()->Properties(map);
  map.set("dynamic",1);
}







KinematicCSpaceAdaptor::KinematicCSpaceAdaptor(const std::shared_ptr<CSpace>& base,Real maxNeighborhoodRadius)
:KinodynamicSpace(base,make_shared<KinematicControlSpace>(base,maxNeighborhoodRadius))
{}




KinodynamicSteeringCSpaceAdaptor::KinodynamicSteeringCSpaceAdaptor(const std::shared_ptr<KinodynamicSpace>& _kinodynamicSpace)
:PiggybackCSpace(_kinodynamicSpace->GetStateSpace().get()),kinodynamicSpace(_kinodynamicSpace)
{
  steeringFunction = kinodynamicSpace->GetControlSpace()->GetSteeringFunction();
  Assert(steeringFunction != NULL);
  Assert(steeringFunction->IsExact());
}

EdgePlannerPtr KinodynamicSteeringCSpaceAdaptor::PathChecker(const Config& a,const Config& b)
{
  KinodynamicMilestonePath path;
  if(!steeringFunction->Connect(a,b,path)) return make_shared<FalseEdgeChecker>(this,a,b);
  return kinodynamicSpace->TrajectoryChecker(path);
}

EdgePlannerPtr KinodynamicSteeringCSpaceAdaptor::PathChecker(const Config& a,const Config& b,int constraint)
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
