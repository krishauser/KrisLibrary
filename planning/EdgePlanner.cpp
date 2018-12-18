#include <KrisLibrary/Logger.h>
#include "EdgePlanner.h"
#include "EdgePlannerHelpers.h"
#include "InterpolatorHelpers.h"
#include <errors.h>
using namespace std;

EdgeChecker::EdgeChecker(CSpace* _space,const InterpolatorPtr& _path)
:space(_space),path(_path)
{}

EdgeChecker::EdgeChecker(CSpace* _space,const Config& a,const Config& b)
:space(_space),path(make_shared<CSpaceInterpolator>(_space,a,b))
{}


TrueEdgeChecker::TrueEdgeChecker(CSpace* _space,const InterpolatorPtr& _path)
  :EdgeChecker(_space,path)
{}

TrueEdgeChecker::TrueEdgeChecker(CSpace* _space,const Config& x,const Config& y)
  :EdgeChecker(_space,x,y)
{}


FalseEdgeChecker::FalseEdgeChecker(CSpace* _space,const InterpolatorPtr& _path)
  :EdgeChecker(_space,path)
{}

FalseEdgeChecker::FalseEdgeChecker(CSpace* _space,const Config& x,const Config& y)
  :EdgeChecker(_space,x,y)
{}


EndpointEdgeChecker::EndpointEdgeChecker(CSpace* _space,const InterpolatorPtr& _path)
  :EdgeChecker(_space,path)
{}

EndpointEdgeChecker::EndpointEdgeChecker(CSpace* _space,const Config& x,const Config& y)
  :EdgeChecker(_space,x,y)
{}

bool EndpointEdgeChecker::IsVisible() { return space->IsFeasible(path->End()); }

PiggybackEdgePlanner::PiggybackEdgePlanner(EdgePlannerPtr _e)
:EdgeChecker(_e->Space(),NULL),e(_e)
{
  EdgeChecker* ec = dynamic_cast<EdgeChecker*>(&*e);
  if(ec)
    path = ec->path;
}

PiggybackEdgePlanner::PiggybackEdgePlanner(CSpace* _space,const InterpolatorPtr& _path,EdgePlannerPtr _e)
  :EdgeChecker(_space,_path),e(_e)
{}

PiggybackEdgePlanner::PiggybackEdgePlanner(CSpace* _space,const Config& _a,const Config& _b,EdgePlannerPtr _e)
  :EdgeChecker(_space,_a,_b),e(_e)
{}

EdgePlannerPtr PiggybackEdgePlanner::Copy() const
{
  return make_shared<PiggybackEdgePlanner>(space,path,e);
}

EdgePlannerPtr PiggybackEdgePlanner::ReverseCopy() const
{
  if(path)
   return make_shared<PiggybackEdgePlanner>(space,make_shared<ReverseInterpolator>(path),e->ReverseCopy());
 else
  return make_shared<PiggybackEdgePlanner>(e->ReverseCopy());
}


void PiggybackEdgePlanner::Eval(Real u,Config& x) const
{
  if(path) 
    EdgeChecker::Eval(u,x);
  else 
    e->Eval(u,x);
}

Real PiggybackEdgePlanner::Length() const
{
  if(path) return EdgeChecker::Length();
  else return e->Length();
}

const Config& PiggybackEdgePlanner::Start() const
{
  if(path) return EdgeChecker::Start();
  else return e->Start();
}
const Config& PiggybackEdgePlanner::End() const
{
  if(path) return EdgeChecker::End();
  else return e->End();
}

CSpace* PiggybackEdgePlanner::Space() const
{
  if(space) return space;
  return e->Space();
}

IncrementalizedEdgePlanner::IncrementalizedEdgePlanner(const EdgePlannerPtr& e)
:PiggybackEdgePlanner(e),checked(false),visible(false)
{}

Real IncrementalizedEdgePlanner::Priority() const { return (checked ? 0.0 : e->Length()); }
bool IncrementalizedEdgePlanner::Plan() { if(!checked) visible = e->IsVisible(); checked=true; return false; }
bool IncrementalizedEdgePlanner::Done() const { return checked; }
bool IncrementalizedEdgePlanner::Failed() const { return checked && !visible; }
EdgePlannerPtr IncrementalizedEdgePlanner::Copy() const {
  auto ie = make_shared<IncrementalizedEdgePlanner>(e);
  ie->checked = checked;
  ie->visible = visible;
  return ie;
}
EdgePlannerPtr IncrementalizedEdgePlanner::ReverseCopy() const
{
  auto ie = make_shared<IncrementalizedEdgePlanner>(e->ReverseCopy());
  ie->checked = checked;
  ie->visible = visible;
  return ie;
}

EpsilonEdgeChecker::EpsilonEdgeChecker(CSpace* _space,const Config& _a,const Config& _b,Real _epsilon)
  :EdgeChecker(_space,_a,_b),epsilon(_epsilon)
{
  foundInfeasible = false;
  dist = Length();
  depth = 0;
  segs = 1;
  if(dist < 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"EpsilonEdgeChecker: Warning, path has negative length?\n");
  }
}

EpsilonEdgeChecker::EpsilonEdgeChecker(CSpace* _space,const InterpolatorPtr& path,Real _epsilon)
  :EdgeChecker(_space,path),epsilon(_epsilon)
{
  foundInfeasible = false;
  dist = Length();
  depth = 0;
  segs = 1;
  if(dist < 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"EpsilonEdgeChecker: Warning, path has negative length?\n");
  }
}


Real Log2(Real r) { return Log(r)*Log2e; }

bool EpsilonEdgeChecker::IsVisible()
{
  if(foundInfeasible) return false;
  while(dist > epsilon) {
    depth++;
    segs *= 2;
    dist *= Half;
    Real du2 = 2.0 / (Real)segs;
    Real u = du2*Half;
    for(int k=1;k<segs;k+=2,u+=du2) {
      path->Eval(u,m);
      if(!space->IsFeasible(m)) {
	foundInfeasible = true;
	return false;
      }
    }
  }
  return true;
}


EdgePlannerPtr EpsilonEdgeChecker::Copy() const
{
  auto p=make_shared<EpsilonEdgeChecker>(space,path,epsilon);
  p->depth=depth;
  p->segs=segs;
  p->dist=dist;
  p->foundInfeasible = foundInfeasible;
  return p;
}

EdgePlannerPtr EpsilonEdgeChecker::ReverseCopy() const
{
  auto p=make_shared<EpsilonEdgeChecker>(space,make_shared<ReverseInterpolator>(path),epsilon);
  p->depth=depth;
  p->segs=segs;
  p->dist=dist;
  p->foundInfeasible = foundInfeasible;
  return p;
}

Real EpsilonEdgeChecker::Priority() const { return dist; }

bool EpsilonEdgeChecker::Plan() 
{
  if(foundInfeasible || dist <= epsilon) return false;
  depth++;
  segs *= 2;
  dist *= Half;
  Real du2 = 2.0 / (Real)segs;
  Real u = du2*Half;
  for(int k=1;k<segs;k+=2,u+=du2) {
    path->Eval(u,m);
    if(!space->IsFeasible(m)) {
      dist = 0;
      foundInfeasible=true;
      return false;
    }
  }
  return true;
}

bool EpsilonEdgeChecker::Done() const { return dist <= epsilon; }

bool EpsilonEdgeChecker::Failed() const { return foundInfeasible; }  






ObstacleDistanceEdgeChecker::ObstacleDistanceEdgeChecker(CSpace* _space,const Config& _a,const Config& _b)
  :EdgeChecker(_space,_a,_b)
{}

ObstacleDistanceEdgeChecker::ObstacleDistanceEdgeChecker(CSpace* _space,const InterpolatorPtr& path)
  :EdgeChecker(_space,path)
{}

bool ObstacleDistanceEdgeChecker::IsVisible()
{
  const Config& a = path->Start();
  const Config& b = path->End();
  return CheckVisibility(path->ParamStart(),path->ParamStart(),a,b,space->ObstacleDistance(a),space->ObstacleDistance(b));
}

bool ObstacleDistanceEdgeChecker::CheckVisibility(Real ua,Real ub,const Config& a,const Config& b,Real da,Real db)
{
  Real dmin = Min(da,db);
  if(dmin <= 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"ObstacleDistanceEdgeChecker: being used when space doesn't properly implement ObstacleDistance()\n");
    return false;
  }
  if(dmin < Epsilon) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, da or db is close to zero");
    return false;
  }
  Real r = space->Distance(a,b);
  Assert(r >= Zero);
  if(dmin > r) return true;
  Config m;
  Real um = (ua+ub)*0.5;
  path->Eval(um,m);
  if(!space->IsFeasible(m)) return false;
  Real ram = space->Distance(a,m);
  Real rbm = space->Distance(b,m);
  Assert(ram < r*0.9 && ram > r*0.1);
  Assert(rbm < r*0.9 && rbm > r*0.1);
  Real dm = space->ObstacleDistance(m);
  Assert(dm >= Zero);
  return CheckVisibility(ua,um,a,m,da,dm)
    && CheckVisibility(um,ub,m,b,dm,db);
}

EdgePlannerPtr ObstacleDistanceEdgeChecker::Copy() const
{
  return make_shared<ObstacleDistanceEdgeChecker>(space,path);
}

EdgePlannerPtr ObstacleDistanceEdgeChecker::ReverseCopy() const
{
  return make_shared<ObstacleDistanceEdgeChecker>(space,make_shared<ReverseInterpolator>(path));
}






BisectionEpsilonEdgePlanner::BisectionEpsilonEdgePlanner(CSpace* _space,const Config& a,const Config& b,Real _epsilon)
  :space(_space),epsilon(_epsilon)
{
  path.push_back(a);
  path.push_back(b);
  Segment s;
  s.prev = path.begin();
  s.length = space->Distance(a,b);
  q.push(s);
}


BisectionEpsilonEdgePlanner::BisectionEpsilonEdgePlanner(CSpace* _space,Real _epsilon)
  :space(_space),epsilon(_epsilon)
{}

Real BisectionEpsilonEdgePlanner::Length() const
{
  Real len = 0.0;
  const Config* prev = &(*path.begin());
  for(list<Config>::const_iterator i=++path.begin();i!=path.end();i++) {
    len += space->Distance(*prev,*i);
    prev = &(*i);
  }
  return len;
}

const Config& BisectionEpsilonEdgePlanner::Start() const { return path.front(); }
const Config& BisectionEpsilonEdgePlanner::End() const { return path.back(); }

bool BisectionEpsilonEdgePlanner::IsVisible()
{
  while(!Done()) {
    if(!Plan()) return false;
  }
  return true;
}

void BisectionEpsilonEdgePlanner::Eval(Real u,Config& x) const
{
  //if(!Done()) LOG4CXX_WARN(KrisLibrary::logger(),"Warning, edge planner not done!");
  if(IsNaN(u) || u < 0 || u > 1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Uh... evaluating path outside of [0,1] range");
    LOG4CXX_INFO(KrisLibrary::logger(),"u="<<u);
    KrisLibrary::loggerWait();
  }
  Assert(u >= Zero && u <= One);
  Real dt = One/(Real)(path.size()-1);
  Real t=Zero;
  list<Config>::const_iterator i=path.begin();
  while(t+dt < u) {
    t+=dt;
    i++;
    if(i == path.end()) { LOG4CXX_INFO(KrisLibrary::logger(),"End of path, u="<<u); x=path.back(); return; }
  }
  Assert(t<=u);
  if(t==u) { x=*i; }
  else {
    list<Config>::const_iterator n=i; n++;
    if(n != path.end()) {
      space->Interpolate(*i,*n,(u-t)/dt,x);
    }
    else {
      x = *i;
    }
  }
}

EdgePlannerPtr BisectionEpsilonEdgePlanner::Copy() const
{
  if(path.size() == 2 && q.size()==1) {  //uninitialized
    return make_shared<BisectionEpsilonEdgePlanner>(space,path.front(),path.back(),epsilon);
  }
  else {
    shared_ptr<BisectionEpsilonEdgePlanner> p(new BisectionEpsilonEdgePlanner(space,epsilon));
    p->path = path;
    if(!Done()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: making a copy of a bisection edge planner that is not done!");
      Segment s;
      s.prev = p->path.begin();
      s.length = space->Distance(path.front(),path.back());
      p->q.push(s);
      Assert(!p->Done());
      //LOG4CXX_INFO(KrisLibrary::logger(),"Press any key to continue...");
      //KrisLibrary::loggerWait();
    }
    return p;
  }
}

EdgePlannerPtr BisectionEpsilonEdgePlanner::ReverseCopy() const
{
  shared_ptr<BisectionEpsilonEdgePlanner> p(new BisectionEpsilonEdgePlanner(space,epsilon));
  p->path.resize(path.size());
  reverse_copy(path.begin(),path.end(),p->path.begin());
  return p;
}

Real BisectionEpsilonEdgePlanner::Priority() const
{
  if(q.empty()) return 0;
  return q.top().length;
}

bool BisectionEpsilonEdgePlanner::Plan()
{
  Segment s=q.top(); q.pop();
  list<Config>::iterator a=s.prev, b=a; b++;
  space->Midpoint(*a,*b,x);
  if(!space->IsFeasible(x)) { 
    //LOG4CXX_INFO(KrisLibrary::logger(),"Midpoint was not feasible\n");
    s.length=Inf; q.push(s); return false;
  }
  list<Config>::iterator m=path.insert(b,x);

  if(q.size()%100 == 0 &&
     Real(q.size())*epsilon > 4.0*space->Distance(Start(),End())) {
    s.length = Inf;
    q.push(s);
    LOG4CXX_INFO(KrisLibrary::logger(),"BisectionEpsilonEdgePlanner: Over 4 times as many iterations as needed, quitting.");
    LOG4CXX_INFO(KrisLibrary::logger(),"Original length "<<space->Distance(Start(),End())<<", epsilon "<<epsilon);
    return false;
  }
  //insert the split segments back in the queue
  Real l1=space->Distance(*a,x);
  Real l2=space->Distance(x,*b);
  if(l1 > 0.9*s.length || l2 > 0.9*s.length) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Midpoint exceeded 0.9x segment distance: "<<l1<<", "<<l2<<" > 0.9*"<<s.length);
    //LOG4CXX_INFO(KrisLibrary::logger(),"a = "<<*a);
    //LOG4CXX_INFO(KrisLibrary::logger(),"b = "<<*b);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Midpoint "<<x);
    //KrisLibrary::loggerWait();
    s.length = Inf;
    q.push(s);
    return false;
  }
  s.prev = a;
  s.length = l1;
  if(s.length > epsilon) q.push(s);

  s.prev = m;
  s.length = l2;
  if(s.length > epsilon) q.push(s);
  return true;
}

bool BisectionEpsilonEdgePlanner::Plan(Config*& pre,Config*& post)
{
  Segment s=q.top(); q.pop();
  list<Config>::iterator a=s.prev, b=a; b++;
  space->Midpoint(*a,*b,x);

  //in case there's a failure...
  pre = &(*a); 
  post = &(*b);

  if(!space->IsFeasible(x))  { s.length=Inf; q.push(s); return false; }
  list<Config>::iterator m=path.insert(b,x);

  //insert the split segments back in the queue
  Real l1=space->Distance(*a,x);
  Real l2=space->Distance(x,*b);
  if(Abs(l1-l2) > 0.9*s.length) {
    s.length = Inf;
    q.push(s); 
    return false;
  }
  s.prev = a;
  s.length = l1;
  if(s.length > epsilon) q.push(s);

  s.prev = m;
  s.length = l2;
  if(s.length > epsilon) q.push(s);
  return true;
}

bool BisectionEpsilonEdgePlanner::Done() const
{
  return q.empty() || q.top().length <= epsilon || IsInf(q.top().length);
}

bool BisectionEpsilonEdgePlanner::Failed() const
{
  if(q.empty()) return false;
  return IsInf(q.top().length);
}








PathEdgeChecker::PathEdgeChecker(CSpace* _space,const std::vector<EdgePlannerPtr >& _path)
:space(_space),path(_path),progress(0),foundInfeasible(false)
{}
void PathEdgeChecker::Eval(Real u,Config& x) const
{
  Real s =Floor(u*path.size());
  int seg = int(s);
  if(seg < 0) x = path.front()->Start();
  else if(seg >= (int)path.size()) x = path.back()->End();
  else
    path[seg]->Eval(s - seg,x);
}
Real PathEdgeChecker::Length() const
{
  Real l = 0;
  for(size_t i=0;i<path.size();i++) l += path[i]->Length();
  return l;
}
const Config& PathEdgeChecker::Start() const { return path.front()->Start(); }
const Config& PathEdgeChecker::End() const { return path.back()->End(); }
bool PathEdgeChecker::IsVisible()
{
  while(progress < path.size()) {
    if(!path[progress]->IsVisible()) {
      foundInfeasible = true;
      return false;
    }
    progress++;
  }
  return true;
}
EdgePlannerPtr PathEdgeChecker::Copy() const
{
  return make_shared<PathEdgeChecker>(space,path);
}

EdgePlannerPtr PathEdgeChecker::ReverseCopy() const
{
  vector<EdgePlannerPtr > rpath(path.size());
  for(size_t i=0;i<path.size();i++)
    rpath[path.size()-i-1] = path[i]->ReverseCopy();
  return make_shared<PathEdgeChecker>(space,rpath);
}

Real PathEdgeChecker::Priority() const
{
  return path.size()-progress;
}
bool PathEdgeChecker::Plan()
{
  if(foundInfeasible) return false;
  if(progress < path.size()) {
    if(!path[progress]->IsVisible()) {
      foundInfeasible = true;
      return false;
    }
    progress++;
  }
  return (progress < path.size());
}
bool PathEdgeChecker::Done() const
{
  return progress >= path.size() || foundInfeasible;
}

bool PathEdgeChecker::Failed() const
{
  return progress < path.size() && foundInfeasible;
}


MultiEdgePlanner::MultiEdgePlanner(CSpace* space,const InterpolatorPtr& path,const std::vector<EdgePlannerPtr >& components)
:PiggybackEdgePlanner(space,path,make_shared<PathEdgeChecker>(space,components))
{}



EdgePlannerWithCSpaceContainer::EdgePlannerWithCSpaceContainer(const std::shared_ptr<CSpace>& space,const EdgePlannerPtr& e)
  :PiggybackEdgePlanner(e),spacePtr(space)
{
}

EdgePlannerPtr EdgePlannerWithCSpaceContainer::Copy() const
{
  return make_shared<EdgePlannerWithCSpaceContainer>(spacePtr,e->Copy());
}

EdgePlannerPtr EdgePlannerWithCSpaceContainer::ReverseCopy() const
{
  return make_shared<EdgePlannerWithCSpaceContainer>(spacePtr,e->ReverseCopy());
}

