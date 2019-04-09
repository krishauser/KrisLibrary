#include "BallTree.h"
#include <KrisLibrary/statistics/statistics.h>
#include <KrisLibrary/math/AABB.h>
#include <iostream>
#include <climits>
#include <algorithm>
using namespace Geometry;
using namespace std;


#define REFERENCE_VECTORS 0

#if REFERENCE_VECTORS
  #define STORE_POINT(a,b) a.setRef(b)
#else
  #define STORE_POINT(a,b) a = b
#endif //REFERENCE_VECTORS



BallTreeNode::BallTreeNode()
:radius(0),parent(NULL)
{}

int BallTreeNode::MaxDepth() const
{
  if(IsLeaf()) return 0;
  int d = 0;
  for(const auto& c : children)
    d = Max(d,1+c->MaxDepth());
  return d;
}

int BallTreeNode::MinDepth() const
{
  if(IsLeaf()) return 0;
  int d = INT_MAX;
  for(const auto& c : children)
    d = Min(d,1+c->MinDepth());
  return d;
}

int BallTreeNode::MaxLeafSize() const
{
  if(IsLeaf()) return (int)pts.size();
  int s = 0;
  for(const auto& c : children)
    s = Max(s,c->MaxLeafSize());
  return s;
}
int BallTreeNode::MinLeafSize() const
{
  if(IsLeaf()) return (int)pts.size();
  int s = INT_MAX;
  for(const auto& c : children)
    s = Min(s,c->MinLeafSize());
  return s;
}

int BallTreeNode::TreeSize() const
{
  int n=1;
  for(const auto& c : children)
    n += c->TreeSize();
  return n;
}

BallTree::BallTree(Metric _metric,int numSplitsPerNode)
:metric(_metric),cartesian(true),splitsPerNode(numSplitsPerNode)
{}

BallTree::BallTree(Metric _metric,Interpolator _interpolator,int numSplitsPerNode)
:metric(_metric),interpolator(_interpolator),cartesian(false),splitsPerNode(numSplitsPerNode)
{}

///Creates the data structure with the given points and max depth
void BallTree::Build(const std::vector<Vector>& p,int maxDepth)
{
  Clear();
  root.pts.resize(p.size());
  for(size_t i=0;i<p.size();i++) {
    STORE_POINT(root.pts[i].pt,p[i]);
    root.pts[i].id = (int)i;
  }
  Fit(&root,true);
  vector<BallTreeNode*> q;
  vector<int> qdepth;
  q.push_back(&root);
  qdepth.push_back(0);
  while(!q.empty()) {
    BallTreeNode* n = q.back(); q.pop_back();
    int d = qdepth.back(); qdepth.pop_back();
    if(d >= maxDepth) continue;
    Split(n);
    for(auto& c:n->children) {
      q.push_back(c.get());
      qdepth.push_back(d+1);
    }
  }
}

///inserts a point, splitting leaf nodes with the indicated number of points
BallTreeNode* BallTree::Insert(const Vector& p,int id,int maxLeafPoints)
{
  Real dmax=Inf;
  BallTreeNode* n = _LookupClosestLeaf(&root,p,dmax);
  if(n->center.n == 0) {
    n->center = p;
    n->radius = 0;
  }
  BallTreeNode::Point pt;
  STORE_POINT(pt.pt,p);
  pt.id = id;
  BallTreeNode* nodeiter = n;
  //also add to all parents
  int numupdates = 0;
  while(nodeiter != NULL) {
    nodeiter->pts.push_back(pt);
    Real d = metric(nodeiter->center,p);
    if(d > nodeiter->radius) {
      nodeiter->radius = d;
    }
    nodeiter = nodeiter->parent;
    numupdates += 1;
  }
  if(maxLeafPoints <= 0) maxLeafPoints = 2*splitsPerNode;
  if((int)n->pts.size() > maxLeafPoints) {
    Split(n);
  }
  return n;
}

BallTreeNode* BallTree::_LookupClosestLeaf(BallTreeNode* node,const Vector& pt,Real& dmin)
{
  if(node->IsLeaf()) return node;
  vector<pair<Real,BallTreeNode*> > searchOrder;
  for(auto& c:node->children) {
    Real d = metric(c->center,pt) - c->radius;
    //pruning
    if(d < dmin) {
      searchOrder.push_back(make_pair(d,c.get()));
    }
  }
  sort(searchOrder.begin(),searchOrder.end());

  BallTreeNode* closest = NULL;
  Real dclosest = Inf;
  for(const auto& dc : searchOrder) {
    BallTreeNode* c = _LookupClosestLeaf(dc.second,pt,dmin);
    if(dmin <= 0) {
      return c;
    }
    Real d = metric(c->center,pt) - c->radius;
    //HACK: how much would this need to expand to contain the new point?
    d = Pow(d+c->radius,c->center.n) - Pow(c->radius,c->center.n);
    if(d < dclosest) {
      dclosest = d;
      closest = c;
    }
  }
  return closest;
}

///Splits the points in a leaf node.  right now this is really simple, and only splits on the longest axis.
///More sophisticated schemes would use SVD, or K-means
bool BallTree::Split(BallTreeNode* node)
{
  Assert(node->IsLeaf());
  if((int)node->pts.size() < splitsPerNode) return false;
  if(splitsPerNode == 2) {
    node->children.resize(splitsPerNode);
    for(auto& c:node->children) {
      c.reset(new BallTreeNode());
      c->parent = node;
    }
    
    //just find axis with max spread
    Vector bmin,bmax;
    bmin = bmax = node->pts[0].pt;
    for(const auto& pt:node->pts) {
      AABBGrow(bmin,bmax,pt.pt);
    }
    int index;
    Real dim = (bmax-bmin).maxElement(&index);
    if(dim == 0) { //all coincident
      node->children.resize(0);
      return false;
    }
    Real split = 0.5*(bmin[index]+bmax[index]);
    for(const auto& pt:node->pts) {
      if(pt.pt[index] < split) 
        node->children[0]->pts.push_back(pt);
      else
        node->children[1]->pts.push_back(pt);
    }
  }
  else {
    FatalError("Can't do k-way splits yet");
  }
  for(auto& c:node->children)
    Fit(c.get(),true);
  Assert(!node->IsLeaf());
  return true;
}

void BallTree::Join(BallTreeNode* node)
{
  node->children.resize(0);
}

void BallTree::Fit(BallTreeNode* node,bool tight)
{
  if(node->pts.size() == 1) {
    node->center = node->pts[0].pt;
    node->radius = 0;
    return;
  }
  if(tight || node->IsLeaf()) {
    if(cartesian) {
      node->center = node->pts[0].pt;
      for(size_t i=1;i<node->pts.size();i++) 
        node->center += node->pts[i].pt;
      node->center /= node->pts.size();
    }
    else {
      //use geodesic averaging
      node->center = node->pts[0].pt;
      Vector temp;
      for(size_t i=1;i<node->pts.size();i++) {
        interpolator(node->center,node->pts[i].pt,1.0/(i+1),temp);
        node->center = temp;
      }
    }
    node->radius = 0;
    for(const auto& p : node->pts)
      node->radius = Max(node->radius,metric(p.pt,node->center));
  }
  else if(node->children.empty()) {
    node->center.clear();
    node->radius = 0;
  }
  else {
    if(cartesian) {
      Real sumweights = 0;
      node->center.resize(node->children[0]->center.n);
      node->center.setZero();
      for(const auto& c : node->children) {
        Real w = Pow(c->radius,c->center.n);
        sumweights += w;
        node->center.madd(c->center,w);
      }
      if(sumweights > 0)
        node->center /= sumweights;
    }
    else {
      //use geodesic averaging
      Real sumweights = Pow(node->children[0]->radius,node->children[0]->center.n);
      node->center = node->children[0]->center;
      Vector temp;
      for(const auto& c : node->children) {
        Real w = Pow(c->radius,c->center.n);
        interpolator(node->center,c->center,w/(sumweights+w),temp);
        sumweights += w;
        node->center = temp;
      }
    }
    node->radius = 0;
    for(const auto& c : node->children) 
      node->radius = Max(node->radius,metric(node->center,c->center)+c->radius);
  }
}

void BallTree::Clear()
{
  root.children.clear();
  root.pts.clear();
  root.center.clear();
  root.radius = 0;
}

int BallTree::ClosestPoint(const Vector& pt,Real& dist) const
{
  if(root.center.n == 0) return -1;
  dist = Inf;
  int idx = -1;
  _ClosestPoint(&root,pt,dist,idx);
  return idx;
}

int BallTree::PointWithin(const Vector& pt,Real& dist) const
{
  if(root.center.n == 0) return -1;
  int idx = -1;
  _ClosestPoint(&root,pt,dist,idx);
  return idx;
}

void BallTree::ClosePoints(const Vector& pt,Real radius,std::vector<Real>& distances,std::vector<int>& ids) const
{
  distances.clear();
  ids.clear();
  if(root.center.n == 0) return;
  return _ClosePoints(&root,pt,radius,distances,ids);
}

void BallTree::KClosestPoints(const Vector& pt,int k,Real* dist,int* idx) const
{
  if(root.center.n == 0) return;
  for(int i=0;i<k;i++) {
    dist[i] = Inf;
    idx[i] = -1;
  }
  int maxidx = 0;
  return _KClosestPoints(&root,pt,k,dist,idx,maxidx);
}

void BallTree::_ClosestPoint(const BallTreeNode* node,const Vector& pt,Real& dist,int& idx) const
{
  //if leaf node, brute force it
  if(node->IsLeaf()) {
    for(const auto& p:node->pts) {
      Real d=metric(p.pt,pt);
      if(d < dist) {
        dist = d;
        idx = p.id;
      }
    }
    return;
  }
  //descend otherwise
  vector<pair<Real,BallTreeNode*> > searchOrder;
  for(auto& c:node->children) {
    Real d = metric(c->center,pt) - c->radius;
    //pruning
    if(d < dist) {
      searchOrder.push_back(make_pair(d,c.get()));
    }
  }
  sort(searchOrder.begin(),searchOrder.end());
  for(const auto& dc : searchOrder) {
    _ClosestPoint(dc.second,pt,dist,idx);
  }
}

void BallTree::_ClosePoints(const BallTreeNode* node,const Vector& pt,Real radius,std::vector<Real>& distances,std::vector<int>& ids) const
{
  if(metric(node->center,pt) - node->radius > radius) return;
  if(node->IsLeaf()) {
    for(const auto& p:node->pts) {
      Real d=metric(p.pt,pt);
      if(d < radius) {
        distances.push_back(d);
        ids.push_back(p.id);
      }
    }
    return;
  }
  for(auto& c:node->children) {
    _ClosePoints(c.get(),pt,radius,distances,ids);
  }
}

void BallTree::_KClosestPoints(const BallTreeNode* node,const Vector& pt,int k,Real* dist,int* idx,int& maxdist) const
{
  //if leaf node, brute force it
  if(node->IsLeaf()) {
    for(const auto& p:node->pts) {
      Real d=metric(p.pt,pt);
      if(d < dist[maxdist]) {
        dist[maxdist] = d;
        idx[maxdist] = p.id;
        //revise maximum distance
        for(int j=0;j<k;j++) {
          if(dist[j] > dist[maxdist])
            maxdist = j;
        }
      }
    }
    return;
  }
  //descend otherwise
  vector<pair<Real,BallTreeNode*> > searchOrder;
  for(auto& c:node->children) {
    Real d = metric(c->center,pt) - c->radius;
    //pruning
    if(d < dist[maxdist]) {
      searchOrder.push_back(make_pair(d,c.get()));
    }
  }
  sort(searchOrder.begin(),searchOrder.end());
  for(const auto& dc : searchOrder) {
    _KClosestPoints(dc.second,pt,k,dist,idx,maxdist);
  }
}
