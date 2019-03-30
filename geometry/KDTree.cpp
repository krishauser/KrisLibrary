#include <KrisLibrary/Logger.h>
#include "KDTree.h"
#include <math/metric.h>
#include <math/random.h>
#include <utils/arrayutils.h>
#include <errors.h>
using namespace Geometry;
using namespace std;

#define REFERENCE_VECTORS 0

#if REFERENCE_VECTORS
  #define STORE_POINT(a,b) a.setRef(b)
#else
  #define STORE_POINT(a,b) a = b
#endif //REFERENCE_VECTORS

Real Distance(const Vector& a,const Vector& b,Real norm,const Vector& weights)
{
  if(weights.empty()) 
    return Distance(a,b,norm);
  else 
    return Distance_Weighted(a,b,norm,weights);
}

struct DDimensionCmp
{
  int d;
  DDimensionCmp(int _d) : d(_d) {}
  bool operator () (const KDTree::Point& a,const KDTree::Point& b) const {
    return a.pt[d] < b.pt[d];
  }
};

inline bool Pos(const Vector& x,int dim,Real val) { return x(dim)>val; }
inline bool Pos(const KDTree::Point& x,int dim,Real val) { return x.pt(dim)>val; }

inline int _MaxDist(Real* dist, int k)
{
  int imax=0;
  for(int i=1;i<k;i++)
    if(dist[i] > dist[imax]) imax=i;
  return imax;
} 

KDTree::Point::Point()
:id(-1)
{}

KDTree::Point::Point(const Point& p)
{
  STORE_POINT(pt,p.pt);
  id = p.id;
}

const KDTree::Point& KDTree::Point::operator = (const Point& p)
{
  STORE_POINT(pt,p.pt);
  id = p.id;
  return *this;
}

KDTree* KDTree::Create(const std::vector<Vector>& p, int k, int maxDepth)
{
  std::vector<Point> pts(p.size());
  for(size_t i=0; i<p.size();i++) {
    STORE_POINT(pts[i].pt,p[i]);
    pts[i].pt = p[i];
    pts[i].id=(int)i;
  }
  return new KDTree(pts,k,0,maxDepth);
}

KDTree::KDTree()
{
  depth = 0;
  splitDim = -1;
  splitVal = 0;
  pos = neg = NULL;
  visits = 0;
}

KDTree::KDTree(const std::vector<Point>& _pts,int k, int _depth, int maxDepth)
{
  depth = _depth;
  //the while loop goes through the possibility that the data are all in
  //the d-plane
  if(maxDepth > depth && _pts.size()>=2) {
    for(int i=0;i<k;i++) {   //test out different dimensions to split
      splitDim = (_depth+i) % k;
      splitVal = Select(_pts,splitDim,_pts.size()/2);
      std::vector<Point> p1,p2;
      std::vector<int> i1,i2;
      for(size_t i=0;i<_pts.size();i++) {
	if(Pos(_pts[i],splitDim,splitVal)) {
	  p1.push_back(_pts[i]);
  }
	else  {
    p2.push_back(_pts[i]);
  }
      }
      if(!p1.empty() && !p2.empty()) { //go down another level
	pos = new KDTree(p1,k,depth+1,maxDepth);
	neg = new KDTree(p2,k,depth+1,maxDepth);
	return;
      }
    }
    //can't split these points
  }
  splitDim=-1;
  pos=neg=NULL;
  pts = _pts;
  visits = 0;
}

KDTree::~KDTree()
{
  SafeDelete(pos);
  SafeDelete(neg);
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


int KDTree::TreeSize() const
{
  if(IsLeaf()) return 1;
  return pos->TreeSize() + neg->TreeSize();
}

bool KDTree::Split(int dimension)
{
  Assert(IsLeaf());
  if(pts.size() <= 1) {
    return false;
  }
  splitDim = dimension;
  if(dimension < 0)
    splitDim = RandInt(pts[0].pt.n);
  splitVal = Select(pts,splitDim,pts.size()/2);
  std::vector<Point> p1,p2;
  for(size_t i=0;i<pts.size();i++) {
    if(Pos(pts[i],splitDim,splitVal)) p1.push_back(pts[i]);
    else p2.push_back(pts[i]);
  }
  if(!p1.empty() && !p2.empty()) { //go down another level
    pts.clear();
    pos = new KDTree();
    neg = new KDTree();
    pos->pts = p1;
    neg->pts = p2;
    pos->depth = depth+1;
    neg->depth = depth+1;
    return true;
  }
  else {
    splitDim = -1;
    return false;
  }
}

void KDTree::Join()
{
  if(IsLeaf()) return;
  pos->Join();
  neg->Join();
  pts = pos->pts;
  pts.insert(pts.end(),neg->pts.begin(),neg->pts.end());
  splitDim = -1;
  SafeDelete(pos);
  SafeDelete(neg);
}

void KDTree::Clear()
{
  splitDim = -1;
  splitVal = 0;
  depth = 0;
  pts.clear();
  SafeDelete(pos);
  SafeDelete(neg);
}

KDTree* KDTree::Insert(const Vector& p,int id,int maxLeafPoints)
{
  KDTree* node = Locate(p);
  Assert(node->IsLeaf());
  //just add to the node
  node->pts.resize(node->pts.size()+1);
  STORE_POINT(node->pts.back().pt,p);
  node->pts.back().id = id;
  if((int)node->pts.size() >= maxLeafPoints) {
    //split
    if(node->Split(node->depth % p.n))
      return node->Locate(p);
  }
  return node;
}

KDTree* KDTree::Locate(const Vector& p)
{
  if(IsLeaf()) return this;
  if(Pos(p,splitDim,splitVal)) return pos->Locate(p);
  else return neg->Locate(p);
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

void KDTree::ClosePoints(const Vector& pt,Real radius,std::vector<Real>& distances,std::vector<int>& ids) const
{
  if(IsLeaf()) {
    Real r2 = Sqr(radius);
    for(size_t i=0;i<pts.size();i++) {
      Real d2=pt.distanceSquared(pts[i].pt);
      if(d2 < r2) {
	distances.push_back(Sqrt(d2));
	ids.push_back(pts[i].id);
      }
    }
  }
  else {
    if(splitVal - pt[splitDim] <= radius)
      pos->ClosePoints(pt,radius,distances,ids);
    if(pt[splitDim] - splitVal <= radius)    
      neg->ClosePoints(pt,radius,distances,ids);
  }
}

void KDTree::ClosePoints(const Vector& pt,Real radius,Real n,const Vector& w,std::vector<Real>& distances,std::vector<int>& ids) const
{
  if(IsLeaf()) {
    for(size_t i=0;i<pts.size();i++) {
      Real d=Distance(pts[i].pt,pt,n,w);
      if(d < radius) {
	distances.push_back(d);
	ids.push_back(pts[i].id);
      }
    }
  }
  else {
    Real weight=(w.empty()?1.0:w[splitDim]);
    if((splitVal - pt[splitDim])*weight <= radius)
      pos->ClosePoints(pt,radius,n,w,distances,ids);
    if((pt[splitDim] - splitVal)*weight <= radius)    
      neg->ClosePoints(pt,radius,n,w,distances,ids);
  }
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
  return (ArrayUtils::nth_element(S,k,DDimensionCmp(d)).pt)[d];
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
      Real d=Distance_L2(pts[i].pt,pt);
      if(d < dist) {
	idx=pts[i].id;
	dist = d;
      }
    }
    return;
  }

  Real d = pt(splitDim)-splitVal;
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
      Real d=Distance_L2(pts[i].pt,pt);
      if(d < dist[maxdist]) {
	idx[maxdist]=pts[i].id;
	dist[maxdist]=d;
	maxdist = _MaxDist(dist,k);
      }
    }
    return;
  }

  Real d = pt(splitDim)-splitVal;
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

void KDTree::_ClosestPoint2(const Vector& pt,Real& dist,int& idx,Real norm,const Vector& weights) const
{
  if(IsLeaf()) {
    //go through list,return closest pt if less than dist
    for(size_t i=0;i<pts.size();i++) {
      Real d=Distance(pts[i].pt,pt,norm,weights);
      if(d < dist) {
	idx=pts[i].id;
	dist = d;
      }
    }
    return;
  }

  //d should be a lower bound on the distance from pt to points in the cell
  Real d=pt(splitDim)-splitVal;
  if(!weights.empty())  d *= weights(splitDim);
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
      Real d=Distance(pts[i].pt,pt,norm,weights);
      if(d < dist[maxdist]) {
	idx[maxdist]=pts[i].id;
	dist[maxdist]=d;
	maxdist = _MaxDist(dist,k);
      }
    }
    /*
    for(int i=0;i<depth;i++) LOG4CXX_INFO(KrisLibrary::logger()," ");
    LOG4CXX_INFO(KrisLibrary::logger(),"dist: ");
    for(int i=0;i<k;i++)
      LOG4CXX_INFO(KrisLibrary::logger(),dist[i]<<" ");
    LOG4CXX_INFO(KrisLibrary::logger(),", max index "<<maxdist);
    KrisLibrary::loggerWait();
    */
    return;
  }

  Real d = pt(splitDim)-splitVal;
  if(!weights.empty()) d*=weights(splitDim);
  /*
  for(int i=0;i<depth;i++) LOG4CXX_INFO(KrisLibrary::logger()," ");
  LOG4CXX_INFO(KrisLibrary::logger(),"plane dist: "<<d);
  */
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
