#include "Interpolator.h"
#include "InterpolatorHelpers.h"
#include <KrisLibrary/spline/TimeSegmentation.h>
#include <KrisLibrary/math/stacking.h>
#include <set>
using namespace Spline;
using namespace std;

LinearInterpolator::LinearInterpolator(Real ax,Real bx,Real ap,Real bp)
:a(1,ax),b(1,bx),ta(ap),tb(bp)
{}

LinearInterpolator::LinearInterpolator(const Config& _a,const Config& _b,Real ap,Real bp)
:a(_a),b(_b),ta(ap),tb(bp)
{}

void LinearInterpolator::Eval(Real u,Config& x) const
{
  if(tb==1 && ta==0)
    interpolate(a,b,u,x);
  else if(ta == tb) {
    if(u<ta)
      x=a;
    else
      x=b;
  }
  else
    interpolate(a,b,(u-ta)/(tb-ta),x);
}

GeodesicInterpolator::GeodesicInterpolator(GeodesicSpace* _space,const Config& _a,const Config& _b)
:space(_space),a(_a),b(_b)
{}

CSpaceInterpolator::CSpaceInterpolator(CSpace* _space,const Config& _a,const Config& _b)
:space(_space),a(_a),b(_b)
{}

ReverseInterpolator::ReverseInterpolator(const InterpolatorPtr& _base)
:base(_base)
{}

TimeRemappedInterpolator::TimeRemappedInterpolator(const InterpolatorPtr& _base,Real _a,Real _b,Real _pstart,Real _pend)
:base(_base),a(_a),b(_b),pstart(_pstart),pend(_pend)
{}

void TimeRemappedInterpolator::Eval(Real u,Config& x) const
{
  base->Eval(a + (b-a)*(u-pstart)/(pend-pstart),x);
}

Real TimeRemappedInterpolator::Length() const
{ 
  return base->Length()*Abs((b-a)/(base->ParamEnd() - base->ParamStart())); 
}

PathInterpolator::PathInterpolator()
{}

PathInterpolator::PathInterpolator(const InterpolatorPtr& interp)
{
  segments.push_back(interp);
  durations.push_back(1.0);
  times.push_back(0);
  times.push_back(1);
}

PathInterpolator::PathInterpolator(const std::vector<InterpolatorPtr > & _segments,Real totalTime)
{
  segments = _segments;
  durations.resize(segments.size(),totalTime / Real(segments.size()));
  times.resize(segments.size()+1);
  times[0] = 0;
  for(size_t i=0;i<segments.size();i++)
    times[i+1]=times[i]+durations[i];
  times.back()=totalTime;
}

void PathInterpolator::Append(const InterpolatorPtr& interp,Real duration)
{
  if(segments.empty()) {
    *this = PathInterpolator(interp);
    if(duration != 0 && duration != 1.0) ScaleDuration(duration);
    return;
  }
  Real totalTime = times.back();
  segments.push_back(interp);
  Real sduration = (duration == 0? durations.back():duration);
  durations.push_back(sduration);
  times.push_back(times.back()+sduration);
  if(duration == 0) 
    SetTotalTime(totalTime);
}

void PathInterpolator::Concat(const PathInterpolator& suffix)
{
  segments.insert(segments.end(),suffix.segments.begin(),suffix.segments.end());
  durations.insert(durations.end(),suffix.durations.begin(),suffix.durations.end());
  Real totalTime = times.back();
  for(size_t i=1;i<suffix.times.size();i++)
    times.push_back(suffix.times[i]+totalTime);
}

void PathInterpolator::ScaleDuration(Real scale) { for(size_t i=0;i<durations.size();i++) durations[i] *= scale; for(size_t i=0;i<times.size();i++) times[i] *= scale; }
void PathInterpolator::SetTotalTime(Real ttotal) { ScaleDuration(ttotal / times.back()); }
void PathInterpolator::Eval(Real u,Config& x) const
{
  Real s;
  int seg = TimeSegmentation::Map(times,u,s);
  if(seg < 0) { x=Start(); return; }
  else if(seg >= (int)segments.size()) { x=End(); return; }
  segments[seg]->Eval(s,x);
}

Real PathInterpolator::Length() const
{
  Real l = 0.0;
  for(size_t i=0;i<segments.size();i++)
    l += segments[i]->Length();
  return l;
}

PiecewiseLinearInterpolator::PiecewiseLinearInterpolator(const std::vector<Config>& _configs)
:configs(_configs)
{}

PiecewiseLinearInterpolator::PiecewiseLinearInterpolator(const std::vector<Config>& _configs,const std::vector<Real>& _times)
:configs(_configs),times(_times)
{}

void PiecewiseLinearInterpolator::Eval(Real u,Config& x) const
{
  if(times.empty()) {
    Real s = u*Real(configs.size()-1);
    int seg = (int)Floor(s);
    if(seg < 0) x = Start();
    else if(seg+1 >= (int)configs.size()) x = End();
    else x = configs[seg] + (s-Real(seg))*(configs[seg+1]-configs[seg]);
  }
  else {
    Real s;
    int seg = TimeSegmentation::Map(times,u,s);
    if(seg < 0) x = Start();
    else if(seg+1 >= (int)configs.size()) x = End();
    else x = configs[seg] + s*(configs[seg+1]-configs[seg]);
  }
}

Real PiecewiseLinearInterpolator::Length() const
{
  Real l=0;
  for(size_t i=0;i+1<configs.size();i++)
    l += configs[i].distance(configs[i+1]);
  return l;
}

PiecewiseLinearCSpaceInterpolator::PiecewiseLinearCSpaceInterpolator(CSpace* _space,const std::vector<Config>& configs)
:PiecewiseLinearInterpolator(configs),space(_space)
{}

PiecewiseLinearCSpaceInterpolator::PiecewiseLinearCSpaceInterpolator(CSpace* _space,const std::vector<Config>& _configs,const std::vector<Real>& _times)
:PiecewiseLinearInterpolator(_configs,_times),space(_space)
{}

void PiecewiseLinearCSpaceInterpolator::Eval(Real u,Config& x) const
{
  if(times.empty()) {
    Real s = u*Real(configs.size()-1);
    int seg = (int)Floor(s);
    if(seg < 0) x = Start();
    else if(seg+1 >= (int)configs.size()) x = End();
    else space->Interpolate(configs[seg],configs[seg+1],(s-Real(seg)),x);
  }
  else {
    Real s;
    int seg = TimeSegmentation::Map(times,u,s);
    if(seg < 0) x = Start();
    else if(seg+1 >= (int)configs.size()) x = End();
    else space->Interpolate(configs[seg],configs[seg+1],s,x);
  }
}

Real PiecewiseLinearCSpaceInterpolator::Length() const
{
  Real l=0;
  for(size_t i=0;i+1<configs.size();i++)
    l += space->Distance(configs[i],configs[i+1]);
  Assert(l >= 0);
  return l;
}

PiecewisePolynomialInterpolator::PiecewisePolynomialInterpolator(const Spline::PiecewisePolynomialND& _path)
:path(_path),start(path.Start()),end(path.End())
{}

void PiecewisePolynomialInterpolator::Eval(Real u,Config& x) const
{
  x = path.Evaluate(u);
}

Real PiecewisePolynomialInterpolator::Length() const
{
  set<double> divs;
  for(size_t i=0;i<path.elements.size();i++) {
    for(size_t j=0;j<path.elements[i].times.size();j++)
      divs.insert(path.elements[i].times[j]);
  }
  Real s = 0;
  Vector prev,next;
  for(set<double>::const_iterator i=divs.begin();i!=divs.end();i++) {
    next = path.Evaluate(*i);
    if(!prev.empty()) 
      s += prev.distance(next);
    swap(prev,next);
  }
  return s;
}


SubsetInterpolator::SubsetInterpolator(const InterpolatorPtr& _base,int _start,int _end)
:base(_base),start(_start),end(_end)
{
  a.resize(end-start);
  b.resize(end-start);
  for(int i=start;i<end;i++) a[i-start] = base->Start()[i];
  for(int i=start;i<end;i++) b[i-start] = base->End()[i];
}

void SubsetInterpolator::Eval(Real u,Config& x) const
{
  Vector temp;
  base->Eval(u,temp);
  x.resize(end-start);
  for(int i=start;i<end;i++) x[i-start] = temp[i];
}



  MultiInterpolator::MultiInterpolator(const InterpolatorPtr& component1,const InterpolatorPtr& component2)
  {
    components.push_back(component1);
    components.push_back(component2);
    Assert(component1->ParamStart() == component2->ParamStart());
    Assert(component1->ParamEnd() == component2->ParamEnd());

    int n1=component1->Start().n;
    int n=n1+component2->Start().n;
    a.resize(n);
    b.resize(n);
    a.copySubVector(0,component1->Start());
    a.copySubVector(n1,component2->Start());
    b.copySubVector(0,component1->End());
    b.copySubVector(n1,component2->End());
  }

  MultiInterpolator::MultiInterpolator(const std::vector<InterpolatorPtr > & _components)
  :components(_components)
  {
    int n=0;
    for(size_t i=0;i<_components.size();i++) 
      n += _components[i]->Start().n;
    a.resize(n);
    b.resize(n);
    n = 0;
    for(size_t i=0;i<_components.size();i++) {
      a.copySubVector(n,_components[i]->Start());
      b.copySubVector(n,_components[i]->End());
      n += _components[i]->Start().n;
    }
  }

  void MultiInterpolator::Split(const Vector& x,std::vector<Vector>& items) const
  {
    FatalError("Not used");
  }

  void MultiInterpolator::Join(const std::vector<Vector>& items,Vector& x) const
  {
    Stack(items,x);
  }

  void MultiInterpolator::Eval(Real u,Config& x) const
  {
    vector<Vector> xsub(components.size());
    for(size_t i=0;i<components.size();i++)
      components[i]->Eval(u,xsub[i]);
    Join(xsub,x);
  }

  Real MultiInterpolator::Length() const
  {
    Real l=0;
    for(size_t i=0;i<components.size();i++)
      l += Sqr(components[i]->Length());
    return Sqrt(l);
  }
