#include "KMeans.h"
#include <utils/permutation.h>
using namespace Statistics;
using namespace std;

KMeans::KMeans(const vector<Vector>& _data)
  :data(_data),weights(NULL),labels(_data.size(),-1)
{
}

KMeans::KMeans(const vector<Vector>& _data,int k)
  :data(_data),weights(NULL),labels(_data.size(),-1),centers(k)
{
}

void KMeans::SetK(int k)
{
  centers.resize(k);
  for(size_t i=0;i<labels.size();i++)
    if(labels[i] >= k) labels[i] = -1;
}

void KMeans::RandomInitialCenters()
{
  if(data.empty()) return;
  vector<int> perm(data.size());
  RandomPermutation(perm);
  if(data.size() < centers.size()) {
    //cyclical centers
    size_t n=data.size();
    for(size_t i=0;i<centers.size();i++)
      centers[i] = data[perm[i%n]];
  }
  else {
    for(size_t i=0;i<centers.size();i++)
      centers[i] = data[perm[i]];
  }
}

void KMeans::ClearLabels()
{
  fill(labels.begin(),labels.end(),-1);
}

void KMeans::Iterate(int& iters)
{
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    if(!CalcLabelsFromCenters()) return;
    CalcCentersFromLabels();
  }
}

bool KMeans::CalcLabelsFromCenters()
{
  assert(data.size()==labels.size());
  if(data.empty()) return false;
  for(size_t c=0;c<centers.size();c++)
    assert(centers[c].n == data[0].n);

  bool changed=false;
  for(size_t i=0;i<data.size();i++) {
    int closest=-1;
    Real closestDist=Inf;
    for(size_t c=0;c<centers.size();c++) {
      Real dist = Distance(data[i],centers[c]);
      if(dist < closestDist) {
	closestDist = dist;
	closest = (int)c;
      }
    }
    if(closest != labels[i]) changed=true;
    labels[i] = closest;
  }
  return changed;
}

void KMeans::CalcCentersFromLabels()
{
  assert(data.size()==labels.size());
  if(data.empty()) return;

  for(size_t c=0;c<centers.size();c++)
    centers[c].setZero();

  vector<Real> num(centers.size(),0);
  if(weights) {
    for(size_t i=0;i<data.size();i++) {
      if(labels[i] >= 0 && labels[i] < (int)centers.size()) {
	num[labels[i]]+=(*weights)[i];
	centers[labels[i]].madd(data[i],(*weights)[i]);
      }
    }
  }
  else {
    //weights are assumed to be 1
    for(size_t i=0;i<data.size();i++) {
      if(labels[i] >= 0 && labels[i] < (int)centers.size()) {
	num[labels[i]] += One;
	centers[labels[i]] += data[i];
      }
    }
  }

  for(size_t c=0;c<centers.size();c++) {
    if(num[c] == 0) //set a random datapoint
      centers[c] = data[rand()%data.size()];
    else 
      centers[c] /= num[c];
  }
}

Real KMeans::AverageDistance(int c)
{
  assert(labels.size() == data.size());
  Real sum=0;
  Real num=0;
  for(size_t i=0;i<labels.size();i++) {
    if((int)i == c) {
      sum += Distance(data[i],centers[c]);
      if(weights) num += (*weights)[i];
      else num += One;
    }
  }
  if(num == 0) return 0;
  return sum/num;
}

void KMeans::AverageDistance(std::vector<Real>& dist)
{
  assert(labels.size() == data.size());
  dist.resize(centers.size());
  fill(dist.begin(),dist.end(),0);
  vector<Real> num(centers.size(),0);
  for(size_t i=0;i<labels.size();i++) {
    int c=labels[i];
    dist[c] += Distance(data[i],centers[c]);
    if(weights) num[c] += (*weights)[i];
    else num[c] += One;
  }
  for(size_t c=0;c<dist.size();c++) {
    if(num[c] == 0) dist[c] = 0;
    else dist[c]/=num[c];
  }
}
