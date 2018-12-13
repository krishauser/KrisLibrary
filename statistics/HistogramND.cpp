#include "HistogramND.h"
#include <errors.h>
#include <algorithm>
using namespace Statistics;
using namespace std;


IndexHash::IndexHash(size_t _pow)
  :pow(_pow)
{}

size_t IndexHash::operator () (const std::vector<int>& x) const
{
  size_t res=0;
  int p=1;
  for(size_t i=0;i<x.size();i++) {
    res ^= p*x[i];
    p *= pow;
  }
  return res;
}


HistogramND::HistogramND(int n)
{
  Resize(n);
}

void HistogramND::Resize(int numDims)
{
  divs.resize(numDims);
  for(int i=0;i<numDims;i++) 
    divs[i].resize(0);
  buckets.clear();
}

void HistogramND::Clear()
{
  divs.resize(0);
  buckets.clear();
}

void HistogramND::Resize(const Size& _dims,const Point& min,const Point& max)
{
  Assert(_dims.size()==min.size() && _dims.size()==max.size());
  buckets.clear();
  divs.resize(_dims.size());
  for(size_t d=0;d<divs.size();d++) {
    divs[d].resize(_dims[d]+1);
    Real x=min[d], h=(max[d]-min[d])/_dims[d];
    for(size_t i=0;i<=_dims[d];i++) {
      divs[d][i] = x;
      x+=h;
    }
  }
}

void HistogramND::Resize(const Index& _dims,const Point& min,const Point& max)
{
  Assert(_dims.size()==min.size() && _dims.size()==max.size());
  buckets.clear();
  divs.resize(_dims.size());
  for(size_t d=0;d<divs.size();d++) {
    divs[d].resize(_dims[d]+1);
    Real x=min[d], h=(max[d]-min[d])/_dims[d];
    for(int i=0;i<=_dims[d];i++) {
      divs[d][i] = x;
      x+=h;
    }
  }
}

void HistogramND::ResizeToFit(const std::vector<Point>& data,const Size& dims)
{
  if(data.empty()) {
    Clear();
    return;
  }
  size_t n=data[0].size();
  Point bmin=data[0],bmax=data[0];
  for(size_t i=1;i<data.size();i++) {
    for(size_t j=0;j<n;j++) {
      if(data[i][j] < bmin[j])
	bmin[j]=data[i][j];
    else if(data[i][j] > bmax[j])
	bmax[j]=data[i][j];
    }
  }
  for(int k=0;k<3;k++)
    if(bmin[k]==bmax[k]) bmax[k] += 1;
  Resize(dims,bmin,bmax);
}

void HistogramND::Fill(Real val)
{
  if(val==0) buckets.clear();
  else {
    FatalError("TODO: Not done with filling multidimensional array with nonzero value");
  }
}

void HistogramND::Calculate(const std::vector<Point>& data)
{
  Fill(0);
  for(size_t i=0;i<data.size();i++) {
    AddBucket(data[i],1);
  }
}

void HistogramND::GetRange(const Index& bucket,Point& min,Point& max) const
{
  Assert(bucket.size()==divs.size());
  min.resize(bucket.size());
  max.resize(bucket.size());
  for(size_t i=0;i<divs.size();i++) {
    min[i]=(bucket[i] == 0? -Inf: divs[i][bucket[i]-1]);
    max[i]=(bucket[i] == divs[i].size()+1? Inf : divs[i][bucket[i]]);
  }
}

void HistogramND::GetBucket(const Point& val,Index& index) const
{
  Assert(val.size()==divs.size());
  index.resize(val.size());
  for(size_t i=0;i<val.size();i++) {
    if(val[i] < divs[i].front()) index[i]=0;
    else if(val[i] >= divs[i].back()) index[0]=divs[i].size()+1;
    else {
      vector<Real>::const_iterator it = std::upper_bound(divs[i].begin(),divs[i].end(),val[i]);
      index[i]=(it-divs[i].begin());
    }
  }
}

void HistogramND::AddBucket(const Point& val,Real num)
{
  Index i;
  GetBucket(val,i);
  BucketHash::iterator it=buckets.find(i);
  if(it == buckets.end()) buckets[i] = num;
  else it->second += num;
}

Real HistogramND::GetBucketCount(const Index& bucket) const
{
  BucketHash::const_iterator i=buckets.find(bucket);
  if(i == buckets.end()) return 0.0;
  return i->second;
}

Real HistogramND::NumObservations() const
{
  Real sum=0;
  for(BucketHash::const_iterator i=buckets.begin();i!=buckets.end();i++)
    sum += i->second;
  return sum;
}
