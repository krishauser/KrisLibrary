#include "DensityEstimator.h"  
#include <KrisLibrary/utils/permutation.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/math/random.h>
using namespace std;
using namespace Geometry;
using namespace Math;

GridDensityEstimator::GridDensityEstimator()
:subdiv(1,1)
{}

GridDensityEstimator::GridDensityEstimator(const std::vector<int>& _mappedDims,Math::Real _h)
:mappedDims(_mappedDims),h(_mappedDims.size(),_h),subdiv(h)
{}

GridDensityEstimator::GridDensityEstimator(const std::vector<int>& _mappedDims,const Math::Vector& _h)
:mappedDims(_mappedDims),h(_h),subdiv(_h)
{}

void GridDensityEstimator::Randomize(int numSourceDims,int numMapped,const Vector& hsource)
{
  subdiv.Clear();
  flattenedBuckets.clear();
  if(numSourceDims <= numMapped) {
    //just do identity
    mappedDims.resize(numSourceDims);
    IdentityPermutation(mappedDims);
    h = hsource;
    subdiv = GridSubdivision(hsource);
    return;
  }
  vector<int> dims(numSourceDims);
  IdentityPermutation(dims);
  RandomlyPermute(dims);
  mappedDims.resize(numMapped);
  copy(dims.begin(),dims.begin()+numMapped,mappedDims.begin());
  h.resize(numMapped);
  for(int i=0;i<numMapped;i++) 
    h[i] = hsource[mappedDims[i]];
  subdiv = GridSubdivision(h);
}

void GridDensityEstimator::Randomize(int numSourceDims,int numMapped,Real hsource)
{
  Vector vh(numSourceDims,hsource);
  Randomize(numSourceDims,numMapped,vh);
}

void GridDensityEstimator::Clear()
{
  subdiv.Clear();
  flattenedBuckets.clear();
}

void GridDensityEstimator::Add(const Math::Vector& x,void* data)
{
  temp.resize(mappedDims.size());
  for(size_t i=0;i<mappedDims.size();i++)
    temp[i] = x[mappedDims[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  subdiv.Insert(index,data);
}

void GridDensityEstimator::Remove(const Math::Vector& x,void* data)
{
  temp.resize(mappedDims.size());
  for(size_t i=0;i<mappedDims.size();i++)
    temp[i] = x[mappedDims[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  bool res=subdiv.Erase(index,data);
  Assert(res == true);
  if(subdiv.buckets.count(index)==0) //deleted, may need to futz around with flattenedBuckets
    flattenedBuckets.resize(0);
}

double GridDensityEstimator::Density(const Config& x)
{
  temp.resize(mappedDims.size());
  for(size_t i=0;i<mappedDims.size();i++)
    temp[i] = x[mappedDims[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  GridSubdivision::ObjectSet* objs = subdiv.GetObjectSet(index);
  if(!objs) return 0;
  return objs->size();
}

void* RandomObject(const list<void*>& objs)
{
  //pick a random point from objs
  int n=objs.size();
  Assert(n != 0);
  int k=RandInt(n);
  list<void*>::const_iterator obj=objs.begin();
  for(int i=0;i<k;i++,obj++);
  return (*obj);
}

void* RandomObject(const vector<void*>& objs)
{
  //pick a random point from objs
  int n=objs.size();
  Assert(n != 0);
  int k=RandInt(n);
  return objs[k];
}

void* GridDensityEstimator::RandomNear(const Math::Vector& x)
{
  temp.resize(mappedDims.size());
  for(size_t i=0;i<mappedDims.size();i++)
    temp[i] = x[mappedDims[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  GridSubdivision::ObjectSet* objs = subdiv.GetObjectSet(index);
  if(!objs) return NULL;
  if(objs->empty()) return NULL;
  return RandomObject(*objs); 
}

void* GridDensityEstimator::Random()
{
  size_t n=subdiv.buckets.size();
  Assert(n > 0);
  int k=RandInt((int)n);
  if(n != flattenedBuckets.size()) {
    //should we flatten buckets to make this faster?
    //if k is greater than n/log n (probability of this is 1-log n)
    if(k >= int(Real(n)/Log(Real(n)))) {
      flattenedBuckets.resize(n);
      GridSubdivision::HashTable::iterator bucket=subdiv.buckets.begin();
      for(size_t i=0;i<n;i++,bucket++)
        flattenedBuckets[i] = &bucket->second;
    }
    else  {
      //advance
      GridSubdivision::HashTable::iterator bucket=subdiv.buckets.begin();
      for(int i=0;i<k;i++,bucket++);
      if(bucket->second.empty()) return NULL;
      return RandomObject(bucket->second);
    }
  }
  if(flattenedBuckets[k]->empty()) 
    return NULL;
  return RandomObject(*flattenedBuckets[k]);
}


MultiGridDensityEstimator::MultiGridDensityEstimator(int _numDims,int _numMappedDims,Real _h)
:numDims(_numDims),numMappedDims(_numMappedDims),h(numDims,_h)
{
  Randomize();
}

MultiGridDensityEstimator::MultiGridDensityEstimator(int _numDims,int _numMappedDims,const Math::Vector& _h)
:numDims(_numDims),numMappedDims(_numMappedDims),h(_h)
{
  Assert(h.n == numDims);
  Randomize();
}

void MultiGridDensityEstimator::Randomize()
{
  int numGrids = numDims;
  grids.resize(numGrids);
  for(size_t i=0;i<grids.size();i++)
    grids[i].Randomize(numGrids,numMappedDims,h);
}

void MultiGridDensityEstimator::Clear()
{
  for(size_t i=0;i<grids.size();i++)
    grids[i].Clear();
}
void MultiGridDensityEstimator::Add(const Math::Vector& x,void* data)
{
  for(size_t i=0;i<grids.size();i++)
    grids[i].Add(x,data);
}

void MultiGridDensityEstimator::Remove(const Math::Vector& x,void* data)
{
  for(size_t i=0;i<grids.size();i++)
    grids[i].Remove(x,data);
}

double MultiGridDensityEstimator::Density(const Config& x)
{
  //multiplicative or additive?
  double s = 1;
  for(size_t i=0;i<grids.size();i++)
    s *= grids[i].Density(x);
  return s;
}

void* MultiGridDensityEstimator::RandomNear(const Math::Vector& x)
{
  vector<double> densities(grids.size());
  for(size_t i=0;i<densities.size();i++)
    densities[i] = grids[i].Density(x);
  int g = WeightedSample(densities);
  return grids[g].RandomNear(x);
}

void* MultiGridDensityEstimator::Random()
{
  return grids[RandInt(grids.size())].Random();
}

KernelDensityEstimator::KernelDensityEstimator(Math::Real _kernelRadius,Math::Real _kernelTruncationFactor)
:kernelType(KernelGaussian),kernelRadius(_kernelRadius),kernelTruncationFactor(_kernelTruncationFactor)
{
  pointLocation = make_shared<KDTreePointLocation>(pointList);
}

void KernelDensityEstimator::Clear()
{
  pointList.clear();
  dataList.clear();
  pointLocation->OnClear();
}

void KernelDensityEstimator::Add(const Math::Vector& x,void* data)
{
  pointList.push_back(x);
  dataList.push_back(data);
  pointLocation->OnAppend();
}

void KernelDensityEstimator::Remove(const Math::Vector& x,void* data)
{
  vector<int> ids;
  vector<double> distances;
  bool res=pointLocation->Close(x,0,ids,distances);
  if(!res) FatalError("Point locator doesn't implement the Close function?");
  for(size_t i=0;i<ids.size();i++) {
    if(dataList[ids[i]] == data) {
      pointLocation->OnDelete(ids[i]);
      pointList.erase(pointList.begin()+ids[i]);
      dataList.erase(dataList.begin()+ids[i]);
    }
  }
}

double KernelDensityEstimator::Density(const Config& x)
{
  double rad = kernelRadius;
  if(kernelType == KernelGaussian)
    rad *= kernelTruncationFactor;
  vector<int> ids;
  vector<double> distances;
  bool res=pointLocation->Close(x,rad,ids,distances);
  if(!res) FatalError("Point locator doesn't implement the Close function?");
  double d = 0;
  for(size_t i=0;i<ids.size();i++) {
    if(kernelType == KernelUniform)
      d += 1;
    else if(kernelType == KernelTriangular)
      d += 1.0-distances[i]/rad;
    else 
      d += Exp(-0.5*Sqr(distances[i]/kernelRadius));
  }
  return d;
}

void* KernelDensityEstimator::RandomNear(const Math::Vector& x)
{
  double rad = kernelRadius;
  if(kernelType == KernelGaussian)
    rad *= kernelTruncationFactor;
  vector<int> ids;
  vector<double> distances;
  bool res=pointLocation->Close(x,rad,ids,distances);
  if(!res) FatalError("Point locator doesn't implement the Close function?");
  if(ids.empty()) return NULL;
  vector<double> ps(ids.size());
  for(size_t i=0;i<ids.size();i++) {
    if(kernelType == KernelUniform)
      ps[i] = 1;
    else if(kernelType == KernelTriangular)
      ps[i] = 1.0-distances[i]/rad;
    else 
      ps[i] = Exp(-0.5*Sqr(distances[i]/kernelRadius));
  }
  int selection = WeightedSample(ps);
  return dataList[ids[selection]];
}

void* KernelDensityEstimator::Random()
{
  vector<double> ps(pointList.size());
  for(size_t i=0;i<pointList.size();i++)
    ps[i] = Density(pointList[i]);
  int selection = WeightedSample(ps);
  return dataList[selection];
}

void KernelDensityEstimator::Random(Math::Vector& x)
{
  vector<double> ps(pointList.size());
  for(size_t i=0;i<pointList.size();i++)
    ps[i] = Density(pointList[i]);
  int selection = WeightedSample(ps);
  //TODO: perturb according to kernel
  x = pointList[selection];
}