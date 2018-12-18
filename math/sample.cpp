#include <KrisLibrary/Logger.h>
#include "math.h"
#include "sample.h"
#include "IntervalSet.h"
#include <math/random.h>
#include <algorithm>
#include <iostream>
#include <errors.h>
using namespace std;

namespace Math {

int WeightedSample(const vector<Real>& weights)
{
  Real totWeight=Zero;
  for(size_t i=0;i<weights.size();i++)  totWeight+=weights[i];
  return WeightedSample(weights,totWeight);
}

int WeightedSample(const vector<Real>& weights,Real totWeight)
{
  Real tmp = Rand()*totWeight;
  for(size_t i=0;i<weights.size();i++) {
    tmp -= weights[i];
    if(tmp <= 0) {
      return (int) i;
    }
  }
  AssertNotReached();
  return -1;
}

int CumulativeWeightedSample(const std::vector<Real>& partialSumWeights)
{
  Real max=partialSumWeights.back();
  Real val=Rand()*max;
  return std::lower_bound(partialSumWeights.begin(),partialSumWeights.end(),val)-partialSumWeights.begin();
}

///allocates total samples within the buckets in num, roughly evenly
void RandomAllocate(vector<int>& num,size_t total)
{
  Assert(!num.empty());
  vector<Real> psampled(num.size());
  Real mean=Real(total)/Real(num.size());
  for(size_t i=0;i<num.size();i++) 
    psampled[i] = mean*Real(i+1);  //cumulative distribution function
  fill(num.begin(),num.end(),0);
  size_t begin=0;
  size_t i=0;
  while(i<total) {
    size_t f=(size_t)floor(psampled[begin]);   //if i < f, we're necessarily allocating more than 1 sample to begin
    if(i < f) {                      //this is here so the algorithm  is O(n) where n is the number of buckets
      num[begin] += (f-i);
      i += (f-i);
    }
    Real val = Real(i)+Rand();
    size_t j;
    for(j=begin;j<num.size();j++) {
      if(val < psampled[j]) {
	//allocate sample i to j, begin at j
	num[j]++;
	begin=j;
	i++;
	break;
      }
    }
    if(j==num.size() && i < total) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"RandomAllocate: should not reach end of list, sample "<<i<<" of "<<total);
      return;
    }
  }
}

///allocates total samples within the buckets in num
void RandomAllocate(vector<int>& num,size_t total,const vector<Real>& weights)
{
  Assert(!num.empty());
  Assert(num.size() == weights.size());
  vector<Real> psampled(num.size());
  for(size_t i=0;i<num.size();i++) 
    psampled[i] = weights[i];
  for(size_t i=1;i<num.size();i++) psampled[i]+=psampled[i-1]; //make this a cumulative distribution function
  Real scale = Real(total)/psampled.back();
  for(size_t i=0;i<num.size();i++) psampled[i] *= scale;  //normalize to sum to # total samples
  psampled.back() = Real(total);

  fill(num.begin(),num.end(),0);
  size_t begin=0;
  size_t i=0;
  while(i<total) {
    size_t f=(size_t)floor(psampled[begin]);   //if i < f, we're necessarily allocating more than 1 sample to begin
    if(i < f) {                      //this is here so the algorithm  is O(n) where n is the number of buckets
      num[begin] += (f-i);
      i += (f-i);
    }
    Real val = Real(i)+Rand();
    size_t j;
    for(j=begin;j<num.size();j++) {
      if(val < psampled[j]) {
	//allocate sample i to j, begin at j
	num[j]++;
	begin=j;
	i++;
	break;
      }
    }
    if(j==num.size() && i < total) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"RandomAllocate: should not reach end of list, sample "<<i<<" of "<<total);
      return;
    }
  }
}


Real Sample(const Interval& s)
{
  return Rand(s.a,s.b);
}

Real Sample(const ClosedIntervalSet& s)
{
  Assert(!s.empty());
  Real max=0;
  for(size_t i=0;i<s.size();i++) {
    if(s[i].b < s[i].a) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Sample(ClosedInterval): interval "<<i<<" is invalid!");
      LOG4CXX_ERROR(KrisLibrary::logger(),"["<<s[i].a<<","<<s[i].b<<"]");
      Abort();
    }
    max += s[i].b-s[i].a;
  }
  if(IsInf(max)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Sample(ClosedInterval): interval has infinite size! aborting");
    Abort();
  }
  if(max==0) {  //a bunch of single points
    int n=rng.randInt(s.size());
    return s[n].a;
  }
  Real u=Rand()*max;
  for(size_t i=0;i<s.size();i++) {
    u -= (s[i].b-s[i].a);
    if(u < 0) return s[i].b+u;
  }
  LOG4CXX_ERROR(KrisLibrary::logger(),"Shouldn't get here!");
  AssertNotReached();
  return 0;
}


void SampleCircle(Real r, Real& x, Real& y)
{
  Real angle = Rand()*TwoPi;
  x = Cos(angle)*r;
  y = Sin(angle)*r;
}

void SampleDisk(Real r, Real& x, Real& y)
{
  Real angle = Rand()*TwoPi;
  Real dist = Sqrt(Rand());
  x = Cos(angle)*dist*r;
  y = Sin(angle)*dist*r;
}

void SampleTriangle(Real& x, Real& y)
{
  x = Rand();
  y = Rand();
  if(x+y > One) {
    x = One-x;
    y = One-y;
  }
  //x = One - Sqrt(One-Rand());
  //y = Rand()*(One-x);
}

void SampleSphere(Real r, Real& x, Real& y, Real& z)
{
  Real theta=Rand()*TwoPi;
  z = Rand()*Two-One;
  Real rad=r*Sqrt(One-Sqr(z));
  x = rad*Cos(theta);
  y = rad*Sin(theta);
  z *= r;
}

void SampleBall(Real r, Real& x, Real& y, Real& z)
{
  SampleSphere(r,x,y,z);
  Real dist = Pow(Rand(),(Real)(1.0/3.0));
  x*=dist;
  y*=dist;
  z*=dist;
}

void SampleHyperSphere(Real r,std::vector<Real>& v)
{
  while(true) {
    Assert(!v.empty());
    for(size_t i=0;i<v.size();i++) 
      v[i] = RandGaussian();
    Real ss=0.0;
    for(size_t i=0;i<v.size();i++) 
      ss += Sqr(v[i]);
    Real norm=Sqrt(ss);
    if(norm == 0.0) continue;
    for(size_t i=0;i<v.size();i++) 
      v[i] *= r/norm;
    return;
  }
}

void SampleHyperBall(Real r,std::vector<Real>& v)
{
  Assert(!v.empty());
  Real dist = Pow(Rand(),1.0/v.size())*r;
  SampleHyperSphere(dist,v);
}


} //namespace Math
