#include <KrisLibrary/Logger.h>
#include "LinearPath.h"
#include "metric.h"
#include <File.h>
#include <algorithm>
#include <errors.h>
#include <iostream>
using namespace Math;
using namespace std;

bool LessTime(const PiecewiseLinearPath::ControlPoint& a, const PiecewiseLinearPath::ControlPoint& b)
{
  return a.t < b.t;
}

void PiecewiseLinearPath::PreEval(Real t)
{
  //TODO: find the segment for t so we don't have to later
}

void PiecewiseLinearPath::Eval(Real t,Vector& x)
{
  if(t<=points.front().t) {
    x = points.front().x;
    return;    
  }
  if(t>=points.back().t) {
    x = points.back().x;
    return;    
  }
  vector<ControlPoint>::iterator prev=GetSegment(t);
  Assert(t >= prev->t);
  vector<ControlPoint>::iterator next=prev; next++;
  if(next == points.end()) {
    x = prev->x;
    return;
  }
  Real dt = next->t-prev->t;
  if(dt <= Zero) {
    LOG4CXX_INFO(KrisLibrary::logger(),"PiecewiseLinearPath: Invalid range of times: ["<<prev->t<<","<<next->t<<"]");
    LOG4CXX_INFO(KrisLibrary::logger(),"Size of path: "<<points.size());
    LOG4CXX_INFO(KrisLibrary::logger(),"Time range: "<<BeginTime()<<" "<<EndTime());
    LOG4CXX_INFO(KrisLibrary::logger(),"Query time: "<<t);
    Abort();
  }
  Real u = (t-prev->t)/dt;
  Interpolate(prev->x,next->x,u,x);
}

void PiecewiseLinearPath::Interpolate(const Vector& a,const Vector& b,Real u,Vector& x) const
{
  x.mul(a,(One-u));
  x.madd(b,u);
}

void PiecewiseLinearPath::Deriv(Real t,Vector& dx)
{
  if(t<=points.front().t) {
    dx.resize(points.front().x.n);
	dx.setZero();
    return;    
  }
  if(t>=points.back().t) {
    dx.resize(points.front().x.n);
	dx.setZero();
    return;    
  }
  vector<ControlPoint>::iterator prev=GetSegment(t);
  Assert(t >= prev->t);
  vector<ControlPoint>::iterator next=prev; next++;
  Real dt = next->t-prev->t;
  if(dt <= Zero) {
    LOG4CXX_INFO(KrisLibrary::logger(),"PiecewiseLinearPath: Invalid range of times: ["<<prev->t<<","<<next->t<<"]");
    Abort();
  }
  Difference(next->x,prev->x,dx);
  dx *= One/dt;
}

void PiecewiseLinearPath::Difference(const Vector& a,const Vector& b,Vector& dx) const
{
  dx.sub(a,b);
}

Real PiecewiseLinearPath::Distance(const Vector& a,const Vector& b) const
{
  return Distance_L2(a,b);
}

vector<PiecewiseLinearPath::ControlPoint>::iterator PiecewiseLinearPath::GetSegment(Real t)
{
  ControlPoint temp;
  temp.t = t;
  return --std::upper_bound(points.begin(),points.end(),temp,LessTime);
}

void PiecewiseLinearPath::ArcLengthParameterize()
{
  if(points.empty()) return;
  points[0].t = Zero;
  for(size_t i=1;i<points.size();i++) {
    points[i].t = points[i-1].t + Distance(points[i].x,points[i-1].x);
  }
}

Real PiecewiseLinearPath::Length() const
{
  if(points.empty()) return Zero;
  Real len=Zero;
  for(size_t i=0;i+1<points.size();i++) {
    len += Distance(points[i].x,points[i+1].x);
  }
  return len;
}

void PiecewiseLinearPath::ScaleTime(Real s)
{
  for(size_t i=0;i<points.size();i++) points[i].t*=s;
}

void PiecewiseLinearPath::OffsetTime(Real off)
{
  for(size_t i=0;i<points.size();i++) points[i].t+=off;
}

void PiecewiseLinearPath::Concat(const PiecewiseLinearPath& p)
{
  size_t offset = points.size();
  Real toffset = (offset==0?Zero:EndTime());
  LOG4CXX_INFO(KrisLibrary::logger(),"Concat, offset index "<<offset);
  LOG4CXX_INFO(KrisLibrary::logger(),"     offset time  "<<toffset);
  LOG4CXX_INFO(KrisLibrary::logger(),"     new points "<<p.points.size());
  points.resize(points.size()+p.points.size());
  for(size_t i=0;i<p.points.size();i++) {
    points[i+offset].t = p.points[i].t+toffset;
    points[i+offset].x = p.points[i].x;
  }
}

void PiecewiseLinearPath::Append(const Vector& x,Real dt)
{
  ControlPoint cp;
  cp.x=x;
  if(!points.empty()) cp.t=EndTime()+dt;
  else cp.t=dt;
  points.push_back(cp);
}

bool PiecewiseLinearPath::Read(File& f)
{
  int numPoints=0;
  if(!ReadFile(f,numPoints)) return false;
  points.resize(numPoints);
  for(int i=0;i<numPoints;i++) {
    if(!ReadFile(f,points[i].t)) return false;
    if(!points[i].x.Read(f)) return false;
  } 
  return true;
}

bool PiecewiseLinearPath::Write(File& f) const
{
  int numPoints=(int)points.size();
  if(!WriteFile(f,numPoints)) return false;
  for(int i=0;i<numPoints;i++) {
    if(!WriteFile(f,points[i].t)) return false;
    if(!points[i].x.Write(f)) return false;
  } 
  return true;
}


