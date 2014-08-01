#include "KinodynamicCSpace.h"
#include <math/diffeq.h>
#include <math/random.h>
#include <Timer.h>
using namespace std;

GivenPathEdgePlanner::GivenPathEdgePlanner(CSpace* _space,const vector<State>& _path,Real _epsilon)
  :space(_space),path(_path),epsilon(_epsilon)
{
  dists.resize(path.size());
  dists[0] = 0;
  for(size_t i=1;i<dists.size();i++)
    dists[i] = dists[i-1] + space->Distance(path[i-1],path[i]);
  bool feasible = true;
  for(size_t i=0;i<dists.size();i++) {
    if(dists[i] < 0 || IsInf(dists[i])) {
      cout<<"Invalid distance "<<i<<": "<<dists[i]<<endl;
      if(i > 0) cout<<path[i-1]<<endl;
      cout<<path[i]<<endl;
      feasible = false;
    }
  }
  Assert(feasible);

  if(!_space->IsFeasible(path.back())) {
    //endpoint is not feasible, need some way of indicating infeasibility
    PathBisector temp;
    temp.a=0;
    temp.b=1;
    temp.adist = 0;
    temp.bdist = Inf;
    edgeQueue.push(temp);
    return;
  }

  PathBisector temp;
  temp.a=0;
  temp.b=1;
  temp.adist = dists.front();
  temp.bdist = dists.back();
  edgeQueue.push(temp);
}

GivenPathEdgePlanner::~GivenPathEdgePlanner()
{
}

bool GivenPathEdgePlanner::IsVisible() 
{
  while(!Done()) {
    if(!Plan()) return false;
  }
  return !Failed();
}

int GivenPathEdgePlanner::Segment(Real u,Real& t) const
{
  if(u <= 0) { t=0; return 0; }
  if(u >= 1) { t=1; return path.size()-2; }
  u*=Real(path.size()-1);
  Real uindex=Floor(u);
  t=u-uindex;
  return (int)uindex;
}

Real GivenPathEdgePlanner::ArcLength(Real u) const
{
  Real t;
  int i=Segment(u,t);
  return (1-t)*dists[i]+t*dists[i+1];
}

void GivenPathEdgePlanner::Eval(Real u,Config& x) const
{
  Real t;
  int i=Segment(u,t);
  space->Interpolate(path[i],path[i+1],t,x);
}

EdgePlanner* GivenPathEdgePlanner::Copy() const
{
  return new GivenPathEdgePlanner(space,path,epsilon);
}

EdgePlanner* GivenPathEdgePlanner::ReverseCopy() const
{
  vector<State> rpath=path;
  reverse(rpath.begin(),rpath.end());
  return new GivenPathEdgePlanner(space,rpath,epsilon);
}

Real GivenPathEdgePlanner::Priority() const
{
  if(edgeQueue.empty()) return 0;
  return edgeQueue.top().bdist - edgeQueue.top().adist;
}

bool GivenPathEdgePlanner::Plan()
{
  if(edgeQueue.empty()) return true;
  PathBisector e=edgeQueue.top(); edgeQueue.pop();
  if(e.bdist-e.adist < epsilon) return true;
  Config x;
  Eval(0.5*(e.a+e.b),x);
  if(!space->IsFeasible(x)) {
    //fprintf(stderr,"Infeasibility at %g\n",0.5*(e.a+e.b));
    e.bdist = Inf;
    //make sure subsequent calls to Plan also return false
    edgeQueue.push(e);
    return false;
  }

  PathBisector e1,e2;
  e1.a = e.a;
  e1.b = 0.5*(e.a+e.b);
  e1.adist = e.adist;
  e1.bdist = ArcLength(e1.b);
  e2.a = 0.5*(e.a+e.b);
  e2.b = e.b;
  e2.adist = e1.bdist;
  e2.bdist = e.bdist;
  edgeQueue.push(e1);
  edgeQueue.push(e2);
  assert(e1.bdist-e1.adist < (e.bdist-e.adist));
  assert(e2.bdist-e2.adist < (e.bdist-e.adist));
  assert(e1.bdist-e1.adist >= 0);
  assert(e2.bdist-e2.adist >= 0);
  return true;
}

bool GivenPathEdgePlanner::Done() const
{
  if(edgeQueue.empty()) return true;
  double d=(edgeQueue.top().bdist-edgeQueue.top().adist); 
  return IsInf(d) || d < epsilon;
}

bool GivenPathEdgePlanner::Failed() const
{
  return !edgeQueue.empty() && IsInf(edgeQueue.top().bdist);
}




void KinodynamicCSpace::ChoiceBiasedSampleControl(const State& x,const State& xGoal,ControlInput& u,int numSamples)
{
  Real closest=Inf;
  ControlInput temp;
  /*
  vector<State> xtrace;
  for(int i=0;i<numSamples;i++) {
    SampleControl(x,temp);
    xtrace.resize(0);
    Simulate(x,temp,xtrace);
    Assert(!xtrace.empty());
    Real dist = Distance(xGoal,xtrace.back());
    if(dist < closest) {
      closest = dist;
      u = temp;
    }
  }
  */
  State x2;
  for(int i=0;i<numSamples;i++) {
    SampleControl(x,temp);
    SimulateEndpoint(x,temp,x2);
    Real dist = Distance(xGoal,x2);
    if(dist < closest) {
      closest = dist;
      u = temp;
    }
  }
}

void KinodynamicCSpace::ChoiceBiasedSampleReverseControl(const State& x,const State& xStart,ControlInput& u,int numSamples)
{
  Real closest=Inf;
  ControlInput temp;
  vector<State> xtrace;
  for(int i=0;i<numSamples;i++) {
    SampleReverseControl(x,temp);
    xtrace.resize(0);
    bool res=ReverseSimulate(x,temp,xtrace);
    Assert(res==true);
    Assert(!xtrace.empty());
    Real dist = Distance(xStart,xtrace.front());
    if(dist < closest) {
      closest = dist;
      u = temp;
    }
  }
}

bool KinodynamicCSpace::NextState(const State& x0,const ControlInput& u,State& x)
{
  vector<State> p;
  Simulate(x0,u,p);
  x = p.back();
  if(!IsFeasible(x)) return false;
  EdgePlanner* e=TrajectoryChecker(p);
  bool res=e->IsVisible();
  delete e;
  return res;
}

bool KinodynamicCSpace::PreviousState(const State& x1,const ControlInput& u,State& x0)
{
  vector<State> p;
  ReverseSimulate(x1,u,p);
  x0 = p.front();
  if(!IsFeasible(x0)) return false;
  EdgePlanner* e=TrajectoryChecker(p);
  bool res=e->IsVisible();
  delete e;
  return res;
}




IntegratedKinodynamicCSpace::IntegratedKinodynamicCSpace(Integrator _type)
  :type(_type)
{}

//hooks for the math library integrators
struct IntegrationFunction : public DiffEqFunction
{
  IntegrationFunction(IntegratedKinodynamicCSpace* _space,const ControlInput& _u)
    :space(_space),u(_u)
  {}

  virtual void Eval(Real t,const Vector& y,Vector& dy) {
    space->XDerivative(y,u,dy);
  }

  IntegratedKinodynamicCSpace* space;
  const ControlInput& u;
};

void IntegratedKinodynamicCSpace::Simulate(const State& x0, const ControlInput& u,vector<State>& p)
{
  IntegrationFunction func(this,u);
  Real dt,h;
  int i,numSteps;
  Parameters(x0,u,dt,numSteps);
  h = dt/numSteps;
  State temp;
  p.push_back(x0);
  switch(type) {
  case Euler:
    for(i=0;i<numSteps;i++) {
      Euler_step(&func,0,h,p.back(),temp);
      p.push_back(temp);
    }
    break;
  case RK4:
    for(i=0;i<numSteps;i++) {
      RungeKutta4_step(&func,0,h,p.back(),temp);
      p.push_back(temp);
    }
    break;
  default:
    FatalError("Unknown integrator type!");
    break;
  }
}







KinematicCSpaceAdaptor::KinematicCSpaceAdaptor(CSpace* _base)
  :base(_base),maxNeighborhoodRadius(0.1)
{}

void KinematicCSpaceAdaptor::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
{
  p.resize(2);
  p[0]=x0;
  p[1]=u;
}

void KinematicCSpaceAdaptor::SimulateEndpoint(const State& x0, const ControlInput& u,State& x1)
{
  x1 = u;
}

EdgePlanner* KinematicCSpaceAdaptor::TrajectoryChecker(const std::vector<State>& p)
{
  return base->LocalPlanner(p.front(),p.back());
}

bool KinematicCSpaceAdaptor::IsValidControl(const State& x,const ControlInput& u)
{ 
  return (base->Distance(x,u)<=maxNeighborhoodRadius); 
}

void KinematicCSpaceAdaptor::SampleControl(const State& x,ControlInput& u)
{
  base->SampleNeighborhood(x,maxNeighborhoodRadius,u);
}

void KinematicCSpaceAdaptor::BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u) 
{
  Real dist=base->Distance(x,xGoal);
  base->Interpolate(x,xGoal,Max(maxNeighborhoodRadius/dist,One),u);
}

bool KinematicCSpaceAdaptor::ConnectionControl(const State& x,const State& xGoal,ControlInput& u) 
{
  u=xGoal;
  return true;
}


//NOTE: not sure if these reverse functions work!
bool KinematicCSpaceAdaptor::ReverseControl(const State& x0,const State& x1,ControlInput& u)
{ 
  u=x0;
  return true; 
}

bool KinematicCSpaceAdaptor::ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p) 
{
  p.resize(2);
  p[0] = u;
  p[1] = x1;
  return true;
}

bool KinematicCSpaceAdaptor::IsValidReverseControl(const State& x1,const ControlInput& u) { return IsValidControl(x1,u); }

void KinematicCSpaceAdaptor::SampleReverseControl(const State& x,ControlInput& u) { SampleControl(x,u); }

void KinematicCSpaceAdaptor::BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u) { BiasedSampleControl(x1,xDest,u); }

void KinematicCSpaceAdaptor::Properties(PropertyMap& map) const { base->Properties(map); map.set("dynamic",1); }
