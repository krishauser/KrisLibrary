#include "EdgePlanner.h"
#include <errors.h>
using namespace std;


TrueEdgePlanner::TrueEdgePlanner(CSpace* _space,const Config& x,const Config& y)
  :a(x),b(y),space(_space)
{}

FalseEdgePlanner::FalseEdgePlanner(CSpace* _space,const Config& x,const Config& y)
  :a(x),b(y),space(_space)
{}

PiggybackEdgePlanner::PiggybackEdgePlanner(CSpace* _space,const Config& _a,const Config& _b,const SmartPointer<EdgePlanner>& _e)
  :a(_a),b(_b),space(_space),e(_e)
{}

StraightLineEpsilonPlanner::StraightLineEpsilonPlanner(CSpace* _space,const Config& _a,const Config& _b,Real _epsilon)
  :a(_a),b(_b),space(_space),epsilon(_epsilon)
{
  foundInfeasible = false;
  dist = space->Distance(a,b);
  depth = 0;
  segs = 1;
}

Real Log2(Real r) { return Log(r)*Log2e; }

bool StraightLineEpsilonPlanner::IsVisible()
{
  while(dist > epsilon) {
    depth++;
    segs *= 2;
    dist *= Half;
    Real du2 = 2.0 / (Real)segs;
    Real u = du2*Half;
    for(int k=1;k<segs;k+=2,u+=du2) {
      space->Interpolate(a,b,u,m);
      if(!space->IsFeasible(m)) {
	foundInfeasible = true;
	return false;
      }
    }
  }
  return !foundInfeasible;
}

void StraightLineEpsilonPlanner::Eval(Real u,Config& x) const
{
  space->Interpolate(a,b,u,x);
}

EdgePlanner* StraightLineEpsilonPlanner::Copy() const
{
  StraightLineEpsilonPlanner* p=new StraightLineEpsilonPlanner(space,a,b,epsilon);
  p->depth=depth;
  p->segs=segs;
  p->dist=dist;
  p->foundInfeasible = foundInfeasible;
  return p;
}

EdgePlanner* StraightLineEpsilonPlanner::ReverseCopy() const
{
  StraightLineEpsilonPlanner* p=new StraightLineEpsilonPlanner(space,b,a,epsilon);
  p->depth=depth;
  p->segs=segs;
  p->dist=dist;
  p->foundInfeasible = foundInfeasible;
  return p;
}

Real StraightLineEpsilonPlanner::Priority() const { return dist; }

bool StraightLineEpsilonPlanner::Plan() 
{
  if(foundInfeasible || dist <= epsilon) return false;
  depth++;
  segs *= 2;
  dist *= Half;
  Real du2 = 2.0 / (Real)segs;
  Real u = du2*Half;
  for(int k=1;k<segs;k+=2,u+=du2) {
    space->Interpolate(a,b,u,m);
    if(!space->IsFeasible(m)) {
      dist = 0;
      foundInfeasible=true;
      return false;
    }
  }
  return true;
}

bool StraightLineEpsilonPlanner::Done() const { return dist <= epsilon; }

bool StraightLineEpsilonPlanner::Failed() const { return foundInfeasible; }  






StraightLineObstacleDistancePlanner::StraightLineObstacleDistancePlanner(CSpace* _space,const Config& _a,const Config& _b)
  :a(_a),b(_b),space(_space)
{}

bool StraightLineObstacleDistancePlanner::IsVisible()
{
  return CheckVisibility(a,b,space->ObstacleDistance(a),space->ObstacleDistance(b));
}

bool StraightLineObstacleDistancePlanner::CheckVisibility(const Config& a,const Config& b,Real da,Real db)
{
  Real dmin = Min(da,db);
  if(dmin < Epsilon) {
    cout<<"Warning, da or db is close to zero"<<endl;
    return false;
  }
  Real r = space->Distance(a,b);
  Assert(r >= Zero);
  if(dmin > r) return true;
  Config m;
  space->Midpoint(a,b,m);
  if(!space->IsFeasible(m)) return false;
  Real ram = space->Distance(a,m);
  Real rbm = space->Distance(b,m);
  Assert(ram < r*0.9 && ram > r*0.1);
  Assert(rbm < r*0.9 && rbm > r*0.1);
  Real dm = space->ObstacleDistance(m);
  Assert(dm >= Zero);
  return CheckVisibility(a,m,da,dm)
    && CheckVisibility(m,b,dm,db);
}

void StraightLineObstacleDistancePlanner::Eval(Real u,Config& x) const
{
  space->Interpolate(a,b,u,x);
}

EdgePlanner* StraightLineObstacleDistancePlanner::Copy() const
{
  StraightLineObstacleDistancePlanner* p=new StraightLineObstacleDistancePlanner(space,a,b);
  return p;
}

EdgePlanner* StraightLineObstacleDistancePlanner::ReverseCopy() const
{
  StraightLineObstacleDistancePlanner* p=new StraightLineObstacleDistancePlanner(space,b,a);
  return p;
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

bool BisectionEpsilonEdgePlanner::IsVisible()
{
  while(!Done()) {
    if(!Plan()) return false;
  }
  return true;
}

void BisectionEpsilonEdgePlanner::Eval(Real u,Config& x) const
{
  //if(!Done()) cout<<"Warning, edge planner not done!"<<endl;
  if(IsNaN(u) || u < 0 || u > 1) {
    cout<<"Uh... evaluating path outside of [0,1] range"<<endl;
    cout<<"u="<<u<<endl;
    getchar();
  }
  Assert(u >= Zero && u <= One);
  Real dt = One/(Real)(path.size()-1);
  Real t=Zero;
  list<Config>::const_iterator i=path.begin();
  while(t+dt < u) {
    t+=dt;
    i++;
    if(i == path.end()) { cout<<"End of path, u="<<u<<endl; x=path.back(); return; }
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

EdgePlanner* BisectionEpsilonEdgePlanner::Copy() const
{
  if(path.size() == 2 && q.size()==1) {  //uninitialized
    return new BisectionEpsilonEdgePlanner(space,path.front(),path.back(),epsilon);
  }
  else {
    BisectionEpsilonEdgePlanner* p=new BisectionEpsilonEdgePlanner(space,epsilon);
    p->path = path;
    if(!Done()) {
      cout<<"Warning: making a copy of a bisection edge planner that is not done!"<<endl;
      Segment s;
      s.prev = p->path.begin();
      s.length = space->Distance(path.front(),path.back());
      p->q.push(s);
      Assert(!p->Done());
      //cout<<"Press any key to continue..."<<endl;
      //getchar();
    }
    return p;
  }
}

EdgePlanner* BisectionEpsilonEdgePlanner::ReverseCopy() const
{
  BisectionEpsilonEdgePlanner* p=new BisectionEpsilonEdgePlanner(space,epsilon);
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
    //printf("Midpoint was not feasible\n");
    s.length=Inf; q.push(s); return false;
  }
  list<Config>::iterator m=path.insert(b,x);

  if(q.size()%100 == 0 &&
     Real(q.size())*epsilon > 4.0*space->Distance(Start(),Goal())) {
    s.length = Inf;
    q.push(s);
    cout<<"BisectionEpsilonEdgePlanner: Over 4 times as many iterations as needed, quitting."<<endl;
    cout<<"Original length "<<space->Distance(Start(),Goal())<<", epsilon "<<epsilon<<endl;
    return false;
  }
  //insert the split segments back in the queue
  Real l1=space->Distance(*a,x);
  Real l2=space->Distance(x,*b);
  if(l1 > 0.9*s.length || l2 > 0.9*s.length) {
    printf("Midpoint exceeded 0.9 time segment distance: %g, %g > 0.9*%g\n",l1,l2,s.length);
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





