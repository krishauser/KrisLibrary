#include <KrisLibrary/Logger.h>
#include "PiecewisePolynomial.h"
#include <KrisLibrary/File.h>
#include <math.h>
#include <stdio.h>
#include <algorithm>

namespace Spline {

typedef std::vector<double> Vector;

PiecewisePolynomial::PiecewisePolynomial()
{
  times.resize(1);
  times[0] = 0.0;
}

PiecewisePolynomial::PiecewisePolynomial(const Poly& p,double a,double b)
{
  segments.resize(1);
  times.resize(2);
  segments[0] = p;
  times[0] = a;
  times[1] = b;
  timeShift.resize(1,0.0);
}

PiecewisePolynomial::PiecewisePolynomial(const std::vector<Poly>& _segments,const std::vector<double>& _times,bool relative)
{
  if(relative) {
    assert(_times.size()==_segments.size());
    segments.resize(_segments.size());
    timeShift.resize(_times.size());
    times.resize(_times.size()+1);
    times[0] = 0;
   for(size_t i=0;i<_segments.size();i++) {
      timeShift[i] = times[i];
      segments[i] = _segments[i];
      times[i+1] = times[i]+_times[0];
    }
  }
  else {
    assert(_times.size()==_segments.size()+1);
    segments = _segments;
    timeShift.resize(_segments.size(),0.0);
    times = _times;
  }
}

PiecewisePolynomial::PiecewisePolynomial(const std::vector<Poly>& _segments,const std::vector<double>& _times,const std::vector<double>& _timeShifts)
{
  assert(_times.size()==_segments.size()+1);
  assert(_timeShifts.size()==_segments.size());
  segments = _segments;
  timeShift = _timeShifts;
  times = _times;
}


int PiecewisePolynomial::FindSegment(double t) const
{
  if(t < times[0]) 
    return -1;
  // TODO: speed this up using a binary search
  std::vector<double>::const_iterator it = --std::upper_bound(times.begin(),times.end(),t);
  if(it == times.end()) return times.size()-1;
  return it-times.begin();
  /*
  for(size_t i=0;i+1<times.size();i++)
    if(t < times[i+1]) 
      return (int)i;
  */
  return times.size()-1;
}

double PiecewisePolynomial::Evaluate(double t) const
{
  assert(!segments.empty());
  int i = FindSegment(t);
  if(i < 0) return Start();
  else if (i >= (int)segments.size()) return End();
  else {
    assert(t >= times[i] && t < times[i+1]);
    return segments[i](t-timeShift[i]);
  }
}

double PiecewisePolynomial::Derivative(double t) const
{
  assert(!segments.empty());
  int i = FindSegment(t);
  if(i < 0) return 0;
  else if (i >= (int)segments.size()) {
	if(t == times.back())
		return segments.back().Derivative(times.back()-timeShift.back());
	return 0;
  }
  else return segments[i].Derivative(t-timeShift[i]);
}

double PiecewisePolynomial::Derivative(double t,int n) const
{
  assert(!segments.empty());
  assert(n >= 0);
  if(n == 0) return Evaluate(t);
  int i = FindSegment(t);
  if(i < 0) return 0;
  else if (i >= (int)segments.size()) return 0;
  else return segments[i].Derivative(t-timeShift[i],n);
}

PiecewisePolynomial PiecewisePolynomial::Differentiate(int n) const
{
  std::vector<Poly> dsegments(segments.size());
  for(size_t i=0;i<segments.size();i++)
    dsegments[i] = segments[i].Differentiate(n);
  return PiecewisePolynomial(dsegments,times,timeShift);
}
    
void PiecewisePolynomial::Append(const Poly& p,double t,bool relative)
{
  if (relative) {
    assert(t >= 0);
    double et = EndTime();
    segments.push_back(p);
    timeShift.push_back(et);
    times.push_back(et+t);
  }
  else {
    assert(t >= EndTime());
    segments.push_back(p);
    timeShift.push_back(0);
    times.push_back(t);
  }
}

void PiecewisePolynomial::Concat(const PiecewisePolynomial& traj,bool relative)
{
  assert(this != &traj);
  if(relative) {
    assert(fabs(traj.Evaluate(traj.StartTime())-traj.Start()) < 1e-6);
    //double v0 = End();
    //double v1 = traj.Evaluate(0);
    PiecewisePolynomial trajshift = traj;
    double et = EndTime();
    trajshift.TimeShift(et);
    //double v1s = trajshift.Evaluate(et);
    Concat(trajshift,false);
  }
  else {
    assert(traj.StartTime() >= EndTime());
    if(traj.StartTime() > EndTime()) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Adding constant with len "<<traj.StartTime()-EndTime());
      //add a constant segment of the trajectory
      segments.push_back(Poly(End()));
      timeShift.push_back(0);
      times.push_back(traj.times[0]);
    }
    segments.insert(segments.end(),traj.segments.begin(),traj.segments.end());
    times.insert(times.end(),++traj.times.begin(),traj.times.end());
    timeShift.insert(timeShift.end(),traj.timeShift.begin(),traj.timeShift.end());
    assert(traj.times.size() == traj.segments.size()+1);
    assert(traj.timeShift.size() == traj.segments.size());
  }
}


void PiecewisePolynomial::TimeShift(double dt)
{
  for(size_t i=0;i<times.size();i++)
    times[i] += dt;
  for(size_t i=0;i<timeShift.size();i++)
    timeShift[i] += dt;
}

void PiecewisePolynomial::ZeroTimeShift()
{
  for(size_t i=0;i<timeShift.size();i++)
    if(timeShift[i] != 0.0) {
      Poly shift;
      shift.Resize(2);
      shift.coef[0] = -timeShift[i];
      shift.coef[1] = 1.0;
      segments[i] = segments[i].Evaluate(shift);
      timeShift[i] = 0.0;
    }
}

void PiecewisePolynomial::Split(double t,PiecewisePolynomial& front,PiecewisePolynomial& back) const
{
  assert(!segments.empty());
  int i = FindSegment(t);
  if(i < 0) {
    front = PiecewisePolynomial(Poly(Start()),t,t);
    back = PiecewisePolynomial(Poly(Start()),t,StartTime());
    back.Concat(*this);
  }
  else if(i >= (int)segments.size()) {
    front = *this;
    front.Concat(PiecewisePolynomial(Poly(End()),EndTime(),t));
    back = PiecewisePolynomial(Poly(End()),t,t);
  }
  else {
    front.segments = std::vector<Poly>(segments.begin(),segments.begin()+i+1);
    front.timeShift = std::vector<double>(timeShift.begin(),timeShift.begin()+i+1);
    front.times = std::vector<double>(times.begin(),times.begin()+i+2);
    front.times.back()= t;
    back.segments = std::vector<Poly>(segments.begin()+i,segments.end());
    back.timeShift = std::vector<double>(timeShift.begin()+i,timeShift.end());
    back.times = std::vector<double>(times.begin()+i,times.end());
    back.times.front() = t;
  }
}

void PiecewisePolynomial::TrimFront(double tstart)
{
  assert(!segments.empty());
  int i = FindSegment(tstart);
  if(i < 0) {
    //add a new constant function
    PiecewisePolynomial self(*this);
    *this = PiecewisePolynomial(Poly(self.Start()),tstart,self.StartTime());
    Concat(self);
  }
  else if (i >= (int)segments.size()) {
    *this = PiecewisePolynomial(End(),tstart,tstart);
  }
  else {
    times = std::vector<double>(times.begin()+i,times.end());
    segments = std::vector<Poly>(segments.begin()+i,segments.end());
    timeShift = std::vector<double>(timeShift.begin()+i,timeShift.end());
    times[0] = tstart;
  }
}

void PiecewisePolynomial::TrimBack(double tend)
{
  assert(!segments.empty());
  int i = FindSegment(tend);
  if(i < 0) {
    *this = PiecewisePolynomial(Poly(Start()),tend,tend);
  }
  else if(i >= (int)segments.size()) {
    Concat(PiecewisePolynomial(Poly(End()),EndTime(),tend));
  }
  else {
    segments = std::vector<Poly>(segments.begin(),segments.begin()+i+1);
    timeShift = std::vector<double>(timeShift.begin(),timeShift.begin()+i+1);
    times = std::vector<double>(times.begin(),times.begin()+i+2);
    times.back()= tend;
  }
}

PiecewisePolynomial PiecewisePolynomial::Select(double a,double b) const
{
  PiecewisePolynomial temp = *this;
  temp.TrimFront(a);
  temp.TrimBack(b);
  return temp;
}

std::pair<double,double> PiecewisePolynomial::MaxDiscontinuity(int derivative) const
{
  assert(derivative >= 0);
  double tmax=0;
  double dmax=0;
  for(size_t i=0;i+1<segments.size();i++) {
    double t=times[i+1];
    double f1 = segments[i].Derivative(t-timeShift[i],derivative);
    double f2 = segments[i+1].Derivative(t-timeShift[i+1],derivative);
    if(fabs(f1-f2) > dmax) {
      dmax = fabs(f1-f2);
      tmax = t;
    }
  }
  return std::pair<double,double>(tmax,dmax);
}

void PiecewisePolynomial::operator += (double val)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] += val;
}

void PiecewisePolynomial::operator -= (double val)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] -= val;
}

void PiecewisePolynomial::operator *= (double val)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] *= val;
}

void PiecewisePolynomial::operator /= (double val)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] /= val;
}

void PiecewisePolynomial::operator += (const Polynomial<double>& b)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] += b;
}

void PiecewisePolynomial::operator -= (const Polynomial<double>& b)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] -= b;
}

void PiecewisePolynomial::operator *= (const Polynomial<double>& b)
{
  for(size_t i=0;i<segments.size();i++)
    segments[i] *= b;
}



PiecewisePolynomialND::PiecewisePolynomialND() {}

PiecewisePolynomialND::PiecewisePolynomialND(const std::vector<Poly>& _elements,double a,double b)
{
  elements.resize(_elements.size());
  for(size_t i=0;i<_elements.size();i++)
    elements[i] = PiecewisePolynomial(_elements[i],a,b);
}

PiecewisePolynomialND::PiecewisePolynomialND(const std::vector<PiecewisePolynomial>& _elements)
  :elements(_elements)
{}

Vector PiecewisePolynomialND::Evaluate(double t) const
{
  Vector res(elements.size());
  for(size_t i=0;i<elements.size();i++) 
    res[i] = elements[i].Evaluate(t);
  return res;
}

Vector PiecewisePolynomialND::Derivative(double t) const
{
  Vector res(elements.size());
  for(size_t i=0;i<elements.size();i++)
    res[i] = elements[i].Derivative(t);
  return res;
}

Vector PiecewisePolynomialND::Derivative(double t,int n) const
{
  assert(n >= 0);
  Vector res(elements.size());
  for(size_t i=0;i<elements.size();i++)
    res[i] = elements[i].Derivative(t,n);
  return res;
}

PiecewisePolynomialND PiecewisePolynomialND::Differentiate(int n) const
{
  PiecewisePolynomialND res;
  res.elements.resize(elements.size());
  for(size_t i=0;i<elements.size();i++)
    res.elements[i] = elements[i].Differentiate(n);
  return res;
}


Vector PiecewisePolynomialND::Start() const
{
  Vector res(elements.size());
  for(size_t i=0;i<elements.size();i++)
    res[i] = elements[i].Start();
  return res;
}

Vector PiecewisePolynomialND::End() const
{
  Vector res(elements.size());
  for(size_t i=0;i<elements.size();i++)
    res[i] = elements[i].End();
  return res;
}

double PiecewisePolynomialND::StartTime() const
{
  double res=elements[0].StartTime();
  for(size_t i=1;i<elements.size();i++)
    res = std::min(res,elements[i].StartTime());
  return res;
}

double PiecewisePolynomialND::EndTime() const
{
  double res=elements[0].EndTime();
  for(size_t i=1;i<elements.size();i++)
    res = std::max(res,elements[i].EndTime());
  return res;
}

void PiecewisePolynomialND::Concat(const PiecewisePolynomialND& traj,bool relative)
{
  double tend=EndTime();
  for(size_t i=0;i<elements.size();i++) {
    //can't concatenate onto an empty polynomial
    assert(!elements[i].segments.empty());
    if(relative)
      //make all elements end at the same time
      elements[i].TrimBack(tend);
    elements[i].Concat(traj.elements[i],relative);
  }
}


void PiecewisePolynomialND::Append(const std::vector<Polynomial<double> >& polys,double t,bool relative)
{
  double tend=EndTime();
  for(size_t i=0;i<elements.size();i++) {
    //can't concatenate onto an empty polynomial
    assert(!elements[i].segments.empty());
    if(relative)
      elements[i].TrimBack(tend);
    elements[i].Append(polys[i],t,relative);  
  }
}

void PiecewisePolynomialND::TimeShift(double dt)
{
  for(size_t i=0;i<elements.size();i++)
    elements[i].TimeShift(dt);
}

void PiecewisePolynomialND::Split(double t,PiecewisePolynomialND& front,PiecewisePolynomialND& back) const
{
  front.elements.resize(elements.size());
  back.elements.resize(elements.size());
  for(size_t i=0;i<elements.size();i++)
    elements[i].Split(t,front.elements[i],back.elements[i]);
}

void PiecewisePolynomialND::TrimFront(double tstart)
{
  for(size_t i=0;i<elements.size();i++)
    elements[i].TrimFront(tstart);
}

void PiecewisePolynomialND::TrimBack(double tend)
{
  for(size_t i=0;i<elements.size();i++)
    elements[i].TrimBack(tend);
}

PiecewisePolynomialND PiecewisePolynomialND::Select(double a,double b) const
{
  PiecewisePolynomialND res;
  res.elements.resize(elements.size());
  for(size_t i=0;i<elements.size();i++)
    res.elements[i] = elements[i].Select(a,b);
  return res;
}

std::pair<Vector,Vector> PiecewisePolynomialND::MaxDiscontinuity(int derivative) const
{
  Vector tres(elements.size()),res(elements.size());
  for(size_t i=0;i<elements.size();i++) {
    std::pair<double,double> d = elements[i].MaxDiscontinuity(derivative);
    tres[i] = d.first;
    res[i] = d.second;
  }
  return std::pair<Vector,Vector>(tres,res);
}


PiecewisePolynomial Constant(double x,double ta,double tb)
{
  return PiecewisePolynomial(Polynomial<double>(x),ta,tb);
}

PiecewisePolynomial Linear(double a,double b,double ta,double tb)
{
  assert(tb >= ta);
  if(tb == ta) {
    if(a != b) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Linear path wants instantaneous jump from "<<a<<" to "<<b<<" at time "<<ta);
    }
    assert(a==b);
    return Constant(a,ta,tb);
  }
  std::vector<double> coefs(2);
  coefs[0] = (a*tb - ta*b)/(tb-ta);
  coefs[1] = (b-a)/(tb-ta);
  return PiecewisePolynomial(Polynomial<double>(coefs),ta,tb);
}

PiecewisePolynomial PiecewiseLinear(const std::vector<double>& milestones,const std::vector<double>& times)
{
  assert(milestones.size() == times.size());
  assert(milestones.size() >= 1);
  if(milestones.size() == 1) {
    return Constant(milestones[0],times[0],times[0]);
  }
  std::vector<Polynomial<double> > segments(milestones.size()-1);
  for(size_t i=0;i+1<milestones.size();i++) {
    double a = milestones[i];
    double b = milestones[i+1];
    double ta = times[i];
    double tb = times[i+1];
    assert(tb > ta); //TODO: allow discontinuities?
    std::vector<double> coefs(2);
    coefs[0] = (a*tb - ta*b)/(tb-ta);
    coefs[1] = (b-a)/(tb-ta);
    segments[i].coef = coefs;
  }
  return PiecewisePolynomial(segments,times);
}

PiecewisePolynomialND Constant(const Vector& q,double ta,double tb)
{
  PiecewisePolynomialND res;
  res.elements.resize(q.size());
  for(size_t i=0;i<q.size();i++)
    res.elements[i] = PiecewisePolynomial(Polynomial<double>(q[i]),ta,tb);
  return res;
}

PiecewisePolynomialND Linear(const Vector& a,const Vector& b,double ta,double tb)
{
  assert(a.size() == b.size());
  PiecewisePolynomialND res;
  res.elements.resize(a.size());
  for(size_t i=0;i<a.size();i++)
    res.elements[i] = Linear(a[i],b[i],ta,tb);
  return res;
}

PiecewisePolynomialND PiecewiseLinear(const std::vector<Vector>& milestones,const std::vector<double>& times)
{
  PiecewisePolynomialND res;
  res.elements.resize(milestones[0].size());
  std::vector<double> ivals(milestones.size());
  for(size_t i=0;i<milestones[0].size();i++) {
    for(size_t j=0;j<milestones.size();j++)
      ivals[j] = milestones[j][i];
    res.elements[i] = PiecewiseLinear(ivals,times);
  }
  return res;
}

PiecewisePolynomialND Subspace(const Vector& x0,const Vector& dx,PiecewisePolynomial& poly)
{
  assert(x0.size() == dx.size());
  PiecewisePolynomialND res;
  res.elements.resize(x0.size());
  for(size_t i=0;i<res.elements.size();i++) {
    res.elements[i] = poly*dx[i]+x0[i];
  }
  return res;
}

template <class T>
bool ReadVectorFile(File& f,std::vector<T>& v)
{
  int n;
  if(!ReadFile(f,n)) return false;
  if(n < 0) return false;
  v.resize(n);
  for(int i=0;i<n;i++)
    if(!ReadFile(f,v[i])) return false;
  return true;
}

template <class T>
bool WriteVectorFile(File& f,const std::vector<T>& v)
{
  int n=int(v.size());
  if(!WriteFile(f,n)) return false;
  for(int i=0;i<n;i++)
    if(!WriteFile(f,v[i])) return false;
  return true;
}

bool PiecewisePolynomial::Read(File& f)
{
  int np;
  if(!ReadFile(f,np)) return false;
  if(np < 0) return false;
  segments.resize(np);
  for(size_t i=0;i<segments.size();i++) 
    if(!ReadVectorFile(f,segments[i].coef)) return false;      
  if(!ReadVectorFile(f,timeShift)) return false;
  if(!ReadVectorFile(f,times)) return false;
  if(segments.size()+1 != times.size()) return false;
  if(segments.size() != timeShift.size()) return false;
  return true;
}

bool PiecewisePolynomial::Write(File& f) const
{
  int np=int(segments.size());
  if(!WriteFile(f,np)) return false;
  for(size_t i=0;i<segments.size();i++) 
    if(!WriteVectorFile(f,segments[i].coef)) return false;      
  if(!WriteVectorFile(f,timeShift)) return false;
  if(!WriteVectorFile(f,times)) return false;
  return true;
}

bool PiecewisePolynomialND::Read(File& f)
{
  return ReadVectorFile(f,elements);
}

bool PiecewisePolynomialND::Write(File& f) const
{
  return WriteVectorFile(f,elements);
}

} //namespace Spline
