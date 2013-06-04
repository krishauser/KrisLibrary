#include "CategoricalDistribution.h"
#include <math/sample.h>
#include <algorithm>
using namespace Statistics;
using namespace std;

CategoricalDistribution::CategoricalDistribution()
{}

CategoricalDistribution::CategoricalDistribution(const vector<Real>& w)
{
  SetWeights(w);
}

void CategoricalDistribution::SetWeights(const vector<Real>& w)
{
  p.resize(w.size());
  cp.resize(w.size());
  Real sumw=0;
  for(size_t i=0;i<w.size();i++)
    sumw += w[i];
  Real sump=0;
  for(size_t i=0;i<w.size();i++) {
    p[i] = w[i]/sumw;
    sump += p[i];
    cp[i] = sump;
  }
}

void CategoricalDistribution::SetValues(const vector<Real>& _values)
{
  values=_values;
}

void CategoricalDistribution::GetParameters(Vector& parameters)
{
  parameters.resize(p.size()-1); 
  for(size_t i=0;i<p.size();i++)
    parameters(i)=p[i];
}

void CategoricalDistribution::SetParameters(const Vector& parameters) 
{
  p.resize(parameters.n+1); 
  Real pn=1;
  for(int i=0;i<parameters.n;i++) {
    p[i]=parameters(i);
    pn -= p[i];
  }
  p.back()=pn;
}

Real CategoricalDistribution::PDF(Real value) 
{
  if(values.empty()) {
    int index=iFloor(value);
    if(0 <= index && index < (int)p.size()) return p[index];
    else return 0;
  }
  else {
	  vector<Real>::iterator i=std::lower_bound(values.begin(),values.end(),value);
    if(i != values.end() && *i==value) return p[i-values.begin()];
    return 0;
  }
}

Real CategoricalDistribution::CDF(Real value) 
{
  if(values.empty()) {
    int index=iFloor(value);
    if(index < 0) return 0;
    else if(index < (int)p.size()) return cp[index];
    else return cp.back();
  }
  else {
    vector<Real>::iterator i=lower_bound(values.begin(),values.end(),value);
    if(i == values.begin()) return 0;
    else if(i == values.end()) return cp.back();
    else return cp[i-values.begin()];
  }
}

Real CategoricalDistribution::Minimum() 
{
  if(values.empty()) return 0; 
  return values.front();
}

Real CategoricalDistribution::Maximum() 
{
  if(values.empty()) return p.size();
  return values.back();
}

Real CategoricalDistribution::Mean()
{ 
  if(values.empty()) {
    Real sum=0;
    for(size_t i=0;i<p.size();i++)
      sum += p[i]*i;
    return sum;
  }
  else {
    Real sum=0;
    for(size_t i=0;i<p.size();i++)
      sum += p[i]*values[i];
    return sum;
  }
}

Real CategoricalDistribution::Variance()
{ 
  return Inf;
}

Real CategoricalDistribution::Skewness() 
{
  return Inf;
}

Real CategoricalDistribution::Kurtosis()
{
  return Inf;
}

Real CategoricalDistribution::Sample()
{
  int index=CumulativeWeightedSample(cp);
  if(values.empty()) return index;
  return values[index];
}
