#include "KDTree.h"
#include <math/metric.h>
#include <utils/arrayutils.h>
#include <errors.h>
using namespace Geometry;
using namespace std;

struct DDimensionCmp
{
  typedef KDTree::Point Point;
  DDimensionCmp(int _d) : d(_d) {}
  inline bool operator ()(const Point& a, const Point& b) const{
    return (*a.pt)[d] < (*b.pt)[d];
  }
  int d;
};

inline bool Pos(const Vector& x,int dim,Real val) { return x(dim)>val; }

inline int _MaxDist(Real* dist, int k)
{
  int imax=0;
  for(int i=1;i<k;i++)
    if(dist[i] > dist[imax]) imax=i;
  return imax;
} 


KDTree* KDTree::Create(const std::vector<Vector>& p, int k, int depth)
{
  std::vector<Point> pts(p.size());
  for(size_t i=0; i<p.size();i++) {
    pts[i].pt=&p[i];
    pts[i].index=(int)i;
  }
  return new KDTree(pts,k,depth);
}

KDTree::KDTree(std::vector<Point>& p, int k, int depth, int _d)
{
  //the while loop goes through the possibility that the data are all in
  //the d-plane
  if(depth > 0 && p.size()>=2) {
    for(int i=0;i<k;i++) {   //test out different dimensions to split
      dim = (_d+i) % k;
      val = Select(p,dim,p.size()/2);
      std::vector<Point> p1,p2;
      for(size_t i=0;i<p.size();i++) {
	if(Pos(p[i],dim,val)) p1.push_back(p[i]);
	else p2.push_back(p[i]);
      }
      if(!p1.empty() && !p2.empty()) { //go down another level
	p.clear();
	pos = new KDTree(p1,k,depth-1,dim+1);
	neg = new KDTree(p2,k,depth-1,dim+1);
	return;
      }
    }
    //can't split these points
  }
  dim=-1;
  pos=neg=NULL;
  pts=p;
}

int KDTree::MaxDepth() const
{
  if(IsLeaf()) return 1;
  int d1 = pos->MaxDepth();
  int d2 = neg->MaxDepth();
  return Max(d1,d2)+1;
}

int KDTree::MinDepth() const
{
  if(IsLeaf()) return 1;
  int d1 = pos->MinDepth();
  int d2 = neg->MinDepth();
  return Min(d1,d2)+1;
}

int KDTree::MaxLeafSize() const
{
  if(IsLeaf()) return pts.size();
  return Max(pos->MaxLeafSize(),neg->MaxLeafSize());
}

int KDTree::MinLeafSize() const
{
  if(IsLeaf()) return pts.size();
  return Min(pos->MinLeafSize(),neg->MinLeafSize());
}


KDTree* KDTree::Locate(const Vector& p)
{
  if(IsLeaf()) return this;
  if(Pos(p,dim,val)) return pos->Locate(p);
  else return neg->Locate(p);
} 

bool KDTree::Remove(int i) {
  Assert(IsLeaf());
  for(size_t k=0;k<pts.size();k++)
    if(i == pts[k].index) {
      pts.erase(pts.begin()+k);
      return true;
    }
  return false;
}

int KDTree::ClosestPoint(const Vector& pt,Real& dist) const
{
  dist = Inf;
  int idx=-1;
  _ClosestPoint(pt,dist,idx);
  return idx;
}

int KDTree::ClosestPoint(const Vector& pt,Real n,const Vector& w,Real& dist) const
{
  dist = Inf;
  int idx=-1;
  _ClosestPoint2(pt,dist,idx,n,w);
  return idx;
}

int KDTree::PointWithin(const Vector& pt,Real& dist) const
{
  int idx=-1;
  _ClosestPoint(pt,dist,idx);
  return idx;
}

void KDTree::KClosestPoints(const Vector& pt,int k,Real* dist,int* idx) const
{
  for(int i=0;i<k;i++) {
    dist[i] = Inf;
    idx[i] = -1;
  }
  int maxdist=0;
  _KClosestPoints(pt,k,dist,idx,maxdist);
}

void KDTree::KClosestPoints(const Vector& pt,int k,Real n,const Vector& w,Real* dist,int* idx) const
{
  for(int i=0;i<k;i++) {
    dist[i] = Inf;
    idx[i] = -1;
  }
  int maxdist=0;
  _KClosestPoints2(pt,k,dist,idx,maxdist,n,w);
}

Real KDTree::Select(const std::vector<Point>& S, int d, int k)
{
  return (*ArrayUtils::nth_element(S,k,DDimensionCmp(d)).pt)[d];
  /*
  Assert(k >= 0 && k < S.size());
  int i=rand()%S.size();
  Real m=(*S[i].pt)[d];
  std::vector<Point>S1,S2;
  S1.reserve(k);
  S2.reserve(k);
  for(size_t i=0;i<S.size();i++) {
    if((*S[i].pt)[d] < m) S1.push_back(S[i]);
    else if((*S[i].pt)[d] > m) S2.push_back(S[i]);
  }
  if(S1.size() > k) return Select(S1,d,k);
  else if(S.size()-S2.size()>=k) return m;
  else return Select(S2,d,k-(S.size()-S2.size()));
  */
}

void KDTree::_ClosestPoint(const Vector& pt,Real& dist,int& idx) const
{
  if(IsLeaf()) {
    //go through list,return closest pt if less than dist
    for(size_t i=0;i<pts.size();i++) {
      Real d=Distance_L2((const Vector&)pts[i],pt);
      if(d < dist) {
	idx=pts[i].index;
	dist = d;
      }
    }
    return;
  }

  Real d = pt(dim)-val;
  if(d >= Zero) { //probably on pos side, check that first
    pos->_ClosestPoint(pt,dist,idx);
    if(d <= dist) //check - if necessary
      neg->_ClosestPoint(pt,dist,idx);
  }
  else if(d <= Zero) { //probably on neg side, check that first
    neg->_ClosestPoint(pt,dist,idx);
    if(-d <= dist) //check + if necessary
      pos->_ClosestPoint(pt,dist,idx);
  }
}

void KDTree::_KClosestPoints(const Vector& pt,int k,Real* dist,int* idx,int& maxdist) const
{
  if(IsLeaf()) {
    //go through list,return closest pts if less than dist
    for(size_t i=0;i<pts.size();i++) {
      Real d=Distance_L2((const Vector&)pts[i],pt);
      if(d < dist[maxdist]) {
	idx[maxdist]=pts[i].index;
	dist[maxdist]=d;
	maxdist = _MaxDist(dist,k);
      }
    }
    return;
  }

  Real d = pt(dim)-val;
  if(d >= Zero) { //probably on pos side, check that first
    pos->_KClosestPoints(pt,k,dist,idx,maxdist);
    if(d <= dist[maxdist]) //check - if necessary
      neg->_KClosestPoints(pt,k,dist,idx,maxdist);
  }
  else if(d <= Zero) { //probably on neg side, check that first
    neg->_KClosestPoints(pt,k,dist,idx,maxdist);
    if(-d <= dist[maxdist]) //check + if necessary
      pos->_KClosestPoints(pt,k,dist,idx,maxdist);
  }
}


Real Distance(const Vector& a,const Vector& b,Real norm,const Vector& weights)
{
  if(weights.empty()) 
    return Distance(a,b,norm);
  else 
    return Distance_Weighted(a,b,norm,weights);
}

void KDTree::_ClosestPoint2(const Vector& pt,Real& dist,int& idx,Real norm,const Vector& weights) const
{
  if(IsLeaf()) {
    //go through list,return closest pt if less than dist
    for(size_t i=0;i<pts.size();i++) {
      Real d=Distance((const Vector&)pts[i],pt,norm,weights);
      if(d < dist) {
	idx=pts[i].index;
	dist = d;
      }
    }
    return;
  }

  //d should be a lower bound on the distance from pt to points in the cell
  Real d=pt(dim)-val;
  if(!weights.empty())  d *= weights(dim);
  if(d >= Zero) { //probably on pos side, check that first
    pos->_ClosestPoint2(pt,dist,idx,norm,weights);
    if(d <= dist) //check - if necessary
      neg->_ClosestPoint2(pt,dist,idx,norm,weights);
  }
  else if(d <= Zero) { //probably on neg side, check that first
    neg->_ClosestPoint2(pt,dist,idx,norm,weights);
    if(-d <= dist) //check + if necessary
      pos->_ClosestPoint2(pt,dist,idx,norm,weights);
  }
}

void KDTree::_KClosestPoints2(const Vector& pt,int k,Real* dist,int* idx,int& maxdist,Real norm,const Vector& weights) const
{
  if(IsLeaf()) {
    //go through list,return closest pts if less than dist
    for(size_t i=0;i<pts.size();i++) {
      Real d=Distance((const Vector&)pts[i],pt,norm,weights);
      if(d < dist[maxdist]) {
	idx[maxdist]=pts[i].index;
	dist[maxdist]=d;
	maxdist = _MaxDist(dist,k);
      }
    }
    /*
    cout<<"dist: ";
    for(int i=0;i<k;i++)
      cout<<dist[i]<<" ";
    cout<<", max "<<maxDist<<endl;
    getchar();
    */
    return;
  }

  Real d = pt(dim)-val;
  if(!weights.empty()) d*=weights(dim);
  //cout<<"plane dist: "<<d<<endl;
  if(d >= Zero) { //probably on pos side, check that first
    pos->_KClosestPoints2(pt,k,dist,idx,maxdist,norm,weights);
    if(d <= dist[maxdist]) //check - if necessary
      neg->_KClosestPoints2(pt,k,dist,idx,maxdist,norm,weights);
  }
  else { //probably on neg side, check that first
    neg->_KClosestPoints2(pt,k,dist,idx,maxdist,norm,weights);
    if(-d <= dist[maxdist]) //check + if necessary
      pos->_KClosestPoints2(pt,k,dist,idx,maxdist,norm,weights);
  }
}
