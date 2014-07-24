#include "PointLocation.h"
#include <math/random.h>
#include <set>
using namespace std;

PointLocationBase::PointLocationBase(vector<Vector>& _points)
  :points(_points)
{}

NaivePointLocation::NaivePointLocation(vector<Vector>& points,CSpace* _space) 
  :PointLocationBase(points),space(_space)
{}

bool NaivePointLocation::NN(const Vector& p,int& nn,Real& distance)
{ 
  nn = -1;
  distance = Inf;
  for(size_t i=0;i<points.size();i++) {
    Real d=space->Distance(points[i],p);
    if(d < distance) {
      nn = (int)i;
      distance = d;
    }
  }
  return true;
}

bool NaivePointLocation::KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances) 
{ 
  set<pair<Real,int> > knn;
  Real dmax = Inf;
  for(size_t i=0;i<points.size();i++) {
    Real d=space->Distance(points[i],p);
    if(d > 0 && d < dmax) {
      pair<Real,int> idx(d,i);
      knn.insert(idx);
      if((int)knn.size() > k)
	knn.erase(--knn.end());
      dmax = (--knn.end())->first;
    }
  }
  nn.resize(0);
  distances.resize(0);
  for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
    nn.push_back(j->second);
    distances.push_back(j->first);
  }
  return true;
}

bool NaivePointLocation::Close(const Vector& p,Real r,std::vector<int>& nn,std::vector<Real>& distances) 
{ 
  nn.resize(0);
  distances.resize(0);
  for(size_t i=0;i<points.size();i++) {
    Real d=space->Distance(points[i],p);
    if(d < r) {
      nn.push_back((int)i);
      distances.push_back(d);
    }
  }
  return true;
}

RandomPointLocation::RandomPointLocation(vector<Vector>& points) 
  :PointLocationBase(points)
{}

bool RandomPointLocation::NN(const Vector& p,int& nn,Real& distance) 
{ 
  nn = RandInt(points.size());
  distance = 0;
  return true;
}
bool RandomPointLocation::KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances) 
{ 
  nn.resize(k);
  distances.resize(k);
  for(int i=0;i<k;i++) {
    nn[i] = RandInt(points.size());
    distances[i] = 0;
  }
  return true;
}

KDTreePointLocation::KDTreePointLocation(vector<Vector>& points) 
  :PointLocationBase(points),norm(2.0)
{}


KDTreePointLocation::KDTreePointLocation(vector<Vector>& points,Real _norm,const Vector& _weights) 
  :PointLocationBase(points),norm(_norm),weights(_weights)
{}

void KDTreePointLocation::OnAppend()
{
  int id=(int)points.size()-1;
  tree.Insert(points.back(),id);
  /*
  if(points.size() % 100 == 0)
    printf("K-D Tree size %d, depth %d\n",tree.TreeSize(),tree.MaxDepth());
  */
}

bool KDTreePointLocation::OnClear()
{
  tree.Clear();
  return true;
}

bool KDTreePointLocation::NN(const Vector& p,int& nn,Real& distance)
{ 
  nn = tree.ClosestPoint(p,distance);
  return true;
}

bool KDTreePointLocation::KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances) 
{ 
  nn.resize(k);
  distances.resize(k);
  tree.KClosestPoints(p,k,norm,weights,&distances[0],&nn[0]);
  //may have fewer than k points
  for(size_t i=0;i<nn.size();i++)
    if(nn[i] < 0) {
      nn.resize(i);
      distances.resize(i);
      break;
    }
  vector<pair<Real,int> > items(nn.size());
  for(size_t i=0;i<nn.size();i++)
    items[i] = pair<Real,int>(distances[i],nn[i]);
  sort(items.begin(),items.end());
  for(size_t i=0;i<nn.size();i++) {
    nn[i] = items[i].second;
    distances[i] = items[i].first;
  }
  return true;
}

bool KDTreePointLocation::Close(const Vector& p,Real r,std::vector<int>& nn,std::vector<Real>& distances) 
{ 
  tree.ClosePoints(p,r,norm,weights,distances,nn);
  return true;
}
