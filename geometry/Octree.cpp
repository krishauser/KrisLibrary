#include <KrisLibrary/Logger.h>
#include "Octree.h"
#include <math3d/Sphere3D.h>
#include <math3d/Box3D.h>
#include <errors.h>
#include <algorithm> //for sort
#include <Timer.h>
using namespace Geometry;
using namespace Math3D;

Octree::Octree(const AABB3D& bb)
{
  AddNode(-1);
  nodes[0].bb = bb;
}

int Octree::MaxDepth() const
{
  vector<int> d(nodes.size(),0);
  for(size_t i=0;i<nodes.size();i++)
    if(nodes[i].parentIndex >= 0)
      d[i] = d[nodes[i].parentIndex]+1;
  return *std::max_element(d.begin(),d.end());
}

void Octree::SplitToDepth(int d)
{
  SplitToDepth(nodes[0],d);
}

void Octree::SplitToDepth(OctreeNode& n,int d)
{
  if(d > 0) {
    int nindex = Index(n);
    if(IsLeaf(n)) 
      Split(nindex);
    for(int i=0;i<8;i++) {
      int c = nodes[nindex].childIndices[i];
      SplitToDepth(nodes[c],d-1);
    }
  }
}

OctreeNode* Octree::SplitToDepth(OctreeNode& n,const Vector3& point,int d)
{
  OctreeNode* c = Lookup(n,point,d);
  if(c) {
    d -= Depth(*c) - Depth(n);
    if(d > 0) {
      int cindex = Index(*c);
      Split(cindex);
      return SplitToDepth(nodes[cindex],point,d);
    }
    return c;
  }
  return NULL;
}


void Octree::SplitToResolution(const Vector3& res)
{
  SplitToResolution(nodes[0],res);
}

void Octree::SplitToResolution(OctreeNode& n,const Vector3& res)
{
  Vector3 size = n.bb.bmax-n.bb.bmin;
  if(size.x > res.x || size.y > res.y || size.z > res.z) {
    int nindex = Index(n);
    if(IsLeaf(n)) 
      Split(nindex);
    for(int i=0;i<8;i++) {
      int c = nodes[nindex].childIndices[i];
      SplitToResolution(nodes[c],res);
    }
  }
}

OctreeNode* Octree::SplitToResolution(OctreeNode& n,const Vector3& point,const Vector3& res)
{
  OctreeNode* c = Lookup(n,point);
  if(c) {
    Vector3 size = c->bb.bmax-c->bb.bmin;
    if(size.x > res.x || size.y > res.y || size.z > res.z) {
      int cindex = Index(*c);
      Split(cindex);
      return SplitToResolution(nodes[cindex],point,res);
    }
    return c;
  }
  return NULL;
}

void Octree::Split(int nindex)
{
  for(int i=0;i<8;i++) {
    int cindex = AddNode(nindex);
    nodes[nindex].childIndices[i] = cindex;
    OctreeNode& c=nodes[cindex];
    Range(nodes[nindex],i,c.bb);
  }
}

void Octree::Join(int id)
{
  if(IsLeaf(nodes[id])) return;
  for(int i=0;i<8;i++)
    DeleteNode(nodes[id].childIndices[i]);
}

OctreeNode* Octree::Lookup(const Vector3& point)
{
  return Lookup(nodes[0],point);
}

OctreeNode* Octree::Lookup(OctreeNode& root,const Vector3& point)
{
  OctreeNode* n=&root;
  if(!n->bb.contains(point))
    return NULL;
  while(!IsLeaf(*n)) {
    n = &nodes[n->childIndices[Child(*n,point)]];
  }
  Assert(n->bb.contains(point));
  return n;
}

OctreeNode* Octree::Lookup(OctreeNode& root,const Vector3& point,int maxDepth)
{
  OctreeNode* n=&root;
  if(!n->bb.contains(point))
    return NULL;
  while(!IsLeaf(*n) && maxDepth > 0) {
    int c=Child(*n,point);
    n = &nodes[n->childIndices[c]];
    maxDepth--;
  }
  if(!n->bb.contains(point))
    //this can occur if the bounding boxes are shrunk
    return NULL;
  return n;
}

void Octree::BoxLookup(const Vector3& bmin,const Vector3& bmax,vector<int>& nodeIndices) const
{
  AABB3D bb(bmin,bmax);
  list<int> q;
  q.push_back(0);
  while(!q.empty()) {
    int n=q.front(); q.pop_front();
    if(!bb.intersects(nodes[n].bb)) continue;
    if(IsLeaf(nodes[n])) 
      nodeIndices.push_back(n);
    else {
      for(int i=0;i<8;i++)
	q.push_back(nodes[n].childIndices[i]);
    }
  }
}

void Octree::BoxLookup(const Box3D& b,vector<int>& nodeIndices) const
{
  list<int> q;
  q.push_back(0);
  while(!q.empty()) {
    int n=q.front(); q.pop_front();
    if(!b.intersects(nodes[n].bb)) continue;
    if(IsLeaf(nodes[n])) 
      nodeIndices.push_back(n);
    else {
      for(int i=0;i<8;i++)
	q.push_back(nodes[n].childIndices[i]);
    }
  }  
}

void Octree::BallLookup(const Vector3& c,Real r,vector<int>& nodeIndices) const
{
  Sphere3D s;
  s.center = c;
  s.radius = r;
  list<int> q;
  q.push_back(0);
  while(!q.empty()) {
    int n=q.front(); q.pop_front();
    if(nodes[n].bb.distance(c) > r) continue;
    if(IsLeaf(nodes[n])) 
      nodeIndices.push_back(n);
    else {
      for(int i=0;i<8;i++)
	q.push_back(nodes[n].childIndices[i]);
    }
  }
}

void Octree::RayLookup(const Ray3D& ray,vector<int>& nodeindices) const
{
  nodeindices.resize(0);
  Real tmin=0,tmax=Inf;
  if(!ray.intersects(nodes[0].bb,tmin,tmax)) 
    return;
  _RayLookup(0,ray,nodeindices);
}

void Octree::FattenedRayLookup(const Ray3D& ray,Real radius,vector<int>& nodeindices) const
{
  nodeindices.resize(0);
  Real tmin=0,tmax=Inf;
  AABB3D fattened = nodes[0].bb;
  fattened.bmin -= Vector3(radius);
  fattened.bmax += Vector3(radius);
  if(!ray.intersects(fattened,tmin,tmax)) 
    return;
  _FattenedRayLookup(0,ray,radius,nodeindices);
}


void Octree::_RayLookup(int nindex,const Ray3D& ray,vector<int>& nodeindices) const
{
  const OctreeNode& node = nodes[nindex];
  //this is kinda slower than it needs to be... can determine intersecting
  //children directly without testing all bb's
  if(IsLeaf(node)) nodeindices.push_back(nindex);
  else {
    vector<pair<Real,int> > children;
    for(int i=0;i<8;i++) {
      Real tmin = 0;
      Real tmax = Inf;
      if(ray.intersects(nodes[node.childIndices[i]].bb,tmin,tmax))
	children.push_back(pair<Real,int>(tmin,node.childIndices[i]));
    }
    //loop through children, sorted by distance
    sort(children.begin(),children.end());
    for(size_t i=0;i<children.size();i++)
      _RayLookup(children[i].second,ray,nodeindices);
  }
}

void Octree::_FattenedRayLookup(int nindex,const Ray3D& ray,Real radius,vector<int>& nodeindices) const
{
  const OctreeNode& node = nodes[nindex];
  //this is kinda slower than it needs to be... can determine intersecting
  //children directly without testing all bb's
  if(IsLeaf(node)) {
    //fine grained test
    if(ray.distance(node.bb) <= radius)
      nodeindices.push_back(nindex);
  }
  else {
    vector<pair<Real,int> > children;
    for(int i=0;i<8;i++) {
      Real tmin = 0;
      Real tmax = Inf;
      AABB3D fattened = nodes[node.childIndices[i]].bb;
      fattened.bmin -= Vector3(radius);
      fattened.bmax += Vector3(radius);
      if(ray.intersects(fattened,tmin,tmax))
	children.push_back(pair<Real,int>(tmin,node.childIndices[i]));
    }
    //loop through children, sorted by distance
    sort(children.begin(),children.end());
    for(size_t i=0;i<children.size();i++)
      _FattenedRayLookup(children[i].second,ray,radius,nodeindices);
  }
}


int Octree::Child(const OctreeNode& n,const Vector3& point) const
{
  Vector3 mid;
  n.bb.getMidpoint(mid);
  int c=0;
  if(point.x >= mid.x) c |= 0x1;
  if(point.y >= mid.y) c |= 0x2;
  if(point.z >= mid.z) c |= 0x4;
  return c;
}

void Octree::Range(const OctreeNode& n,int child,AABB3D& bb) const
{
  Assert(child >= 0 && child < 8);
  bb=n.bb;
  Vector3 mid;
  n.bb.getMidpoint(mid);
  if(child&0x1)
    bb.bmin[0] = mid[0];
  else
    bb.bmax[0] = mid[0];
  if(child&0x2)
    bb.bmin[1] = mid[1];
  else
    bb.bmax[1] = mid[1];
  if(child&0x4)
    bb.bmin[2] = mid[2];
  else
    bb.bmax[2] = mid[2];
}

int Octree::AddNode(int parent)
{
  int index;
  if(freeNodes.empty()) {
    index=(int)nodes.size();
    nodes.resize(nodes.size()+1);
    //TODO: allow deletions / additions while keeping topological sorted order
    Assert(index > parent);
  }
  else {
    index=freeNodes.front();
    freeNodes.pop_front();
  }
  OctreeNode& n = nodes[index];
  n.childIndices[0] = -1;
  n.parentIndex = parent;
  return index;
}

void Octree::DeleteNode(int id)
{
  Assert(id >= 0 && id < (int)nodes.size());
  freeNodes.push_back(id);
  if(nodes[id].childIndices[0] >= 0) {
    for(int i=0;i<8;i++)
      DeleteNode(nodes[i].childIndices[i]);
    nodes[id].childIndices[0] = -1;
  }
}


OctreePointSet::OctreePointSet(const AABB3D& bbox,int _maxPointsPerCell,Real _minCellSize)
  :Octree(bbox),maxPointsPerCell(_maxPointsPerCell),minCellSize(_minCellSize),fit(false)
{
  //TODO: the Octree constructor calls the virtual AddNode function, which is a no-no in C++
  if (nodes.size() > (int)indexLists.size()) {
    indexLists.resize(nodes.size());
  }
}

size_t OctreePointSet::NumPoints(int node) const
{
  return indexLists[node].size();
}

void OctreePointSet::GetPoints(int node,vector<Vector3>& pts) const
{
  pts.resize(indexLists[node].size());
  for(size_t i=0;i<indexLists[node].size();i++)
    pts[i] = points[indexLists[node][i]];
}

void OctreePointSet::GetPointIDs(int node,vector<int>& ids) const
{
  ids.resize(indexLists[node].size());
  for(size_t i=0;i<indexLists[node].size();i++)
    ids[i] = this->ids[indexLists[node][i]];
}


void OctreePointSet::Add(const Vector3& pt,int id)
{
  if(fit) FatalError("OctreePointSet: Cannot call Add() after FitToPoints()");
  //Timer timer;
  int pindex=(int)points.size();
  points.push_back(pt);
  ids.push_back(id);
  OctreeNode* node = Lookup(pt);
  if(node == NULL) FatalError("OctreePointSet: adding point outside range");
  int nindex = Index(*node);
  //LOG4CXX_INFO(KrisLibrary::logger(),"Lookup time "<<timer.ElapsedTime());
  //timer.Reset();
  Assert(nindex >= 0 && nindex<(int)nodes.size());
  if(nindex >= (int)indexLists.size()) {
    indexLists.push_back(vector<int>());
    indexLists[nindex].reserve(maxPointsPerCell);
  }
  indexLists[nindex].push_back(pindex);
  //timer.Reset();
  if((int)indexLists[nindex].size() > maxPointsPerCell) {
    //split, if bounding box containing points is greater than the maximum cell size
    AABB3D bbox(pt,pt);
    for(size_t i=0;i+1<indexLists[nindex].size();i++) {
      bbox.expand(points[indexLists[nindex][i]]);
      if(bbox.bmin.x + minCellSize < bbox.bmax.x || bbox.bmin.y + minCellSize < bbox.bmax.y || bbox.bmin.z + minCellSize < bbox.bmax.z) {
	Split(nindex);
	//LOG4CXX_INFO(KrisLibrary::logger(),"Split time "<<timer.ElapsedTime());
	//KrisLibrary::loggerWait();
	break;
      }
    }
  }
}


int OctreePointSet::AddNode(int parent)
{
  int res=Octree::AddNode(parent);
  if(res >= (int)indexLists.size()) {
    indexLists.resize(res+1);
  }
  return res;
}

void OctreePointSet::DeleteNode(int id)
{
  Octree::DeleteNode(id);
  indexLists[id].resize(0);
}

void OctreePointSet::Split(int nindex)
{
  Assert(nindex >= 0 && nindex < (int)nodes.size());
  Octree::Split(nindex);
  OctreeNode& node = nodes[nindex];
  for(int i=0;i<8;i++)
    indexLists[node.childIndices[i]].reserve(maxPointsPerCell);
  //distribute points among children
  for(size_t i=0;i<indexLists[nindex].size();i++) {
    const Vector3& pt = points[indexLists[nindex][i]];
    int cindex = node.childIndices[Child(node,pt)];
    indexLists[cindex].push_back(indexLists[nindex][i]);
  }
  indexLists[nindex].clear();
}

void OctreePointSet::Join(int nindex)
{
  if(IsLeaf(nodes[nindex])) return;
  for(int c=0;c<8;c++) {
    int cindex = nodes[nindex].childIndices[c];
    indexLists[nindex].insert(indexLists[nindex].end(),indexLists[cindex].begin(),indexLists[cindex].end());
  }
  Octree::Join(nindex);
}


void OctreePointSet::BoxQuery(const Vector3& bmin,const Vector3& bmax,vector<Vector3>& points,vector<int>& ids) const
{
  points.resize(0);
  ids.resize(0);
  vector<int> boxnodes;
  BoxLookup(bmin,bmax,boxnodes);
  AABB3D bb(bmin,bmax);
  for(size_t i=0;i<boxnodes.size();i++) {
    const vector<int>& pindices = indexLists[boxnodes[i]];
    for(size_t k=0;k<pindices.size();k++)
      if(bb.contains(this->points[pindices[k]])) {
	points.push_back(this->points[pindices[k]]);
	ids.push_back(this->ids[pindices[k]]);
      }
  }
}

void OctreePointSet::BoxQuery(const Box3D& b,vector<Vector3>& points,vector<int>& ids) const
{
  points.resize(0);
  ids.resize(0);
  vector<int> boxnodes;
  BoxLookup(b,boxnodes);
  for(size_t i=0;i<boxnodes.size();i++) {
    const vector<int>& pindices = indexLists[boxnodes[i]];
    for(size_t k=0;k<pindices.size();k++)
      if(b.contains(this->points[pindices[k]])) {
	points.push_back(this->points[pindices[k]]);
	ids.push_back(this->ids[pindices[k]]);
      }
  }
}

void OctreePointSet::BallQuery(const Vector3& c,Real r,vector<Vector3>& points,vector<int>& ids) const
{
  points.resize(0);
  ids.resize(0);
  vector<int> ballnodes;
  BallLookup(c,r,ballnodes);
  Sphere3D s;
  s.center = c;
  s.radius = r;
  for(size_t i=0;i<ballnodes.size();i++) {
    const vector<int>& pindices = indexLists[ballnodes[i]];
    for(size_t k=0;k<pindices.size();k++)
      if(s.contains(this->points[pindices[k]])) {
	points.push_back(this->points[pindices[k]]);
	ids.push_back(this->ids[pindices[k]]);
      }
  }
}

void OctreePointSet::RayQuery(const Ray3D& r,Real radius,vector<Vector3>& points,vector<int>& ids) const
{
  points.resize(0);
  ids.resize(0);
  vector<int> raynodes;
  FattenedRayLookup(r,radius,raynodes);
  Vector3 temp;
  Real r2 = radius*radius;
  for(size_t i=0;i<raynodes.size();i++) {
    const vector<int>& pindices = indexLists[raynodes[i]];
    for(size_t k=0;k<pindices.size();k++) {
      const Vector3& pt=this->points[pindices[k]];
      Real t=r.closestPoint(pt,temp);
      if(pt.distanceSquared(temp) <= r2) {
	points.push_back(pt);
	ids.push_back(this->ids[pindices[k]]);
      }
    }
  }
}

int OctreePointSet::RayCast(const Ray3D& r,Real radius) const
{
  vector<int> raynodes;
  FattenedRayLookup(r,radius,raynodes);
  Vector3 temp;
  Real r2 = radius*radius;
  for(size_t i=0;i<raynodes.size();i++) {
    const vector<int>& pindices = indexLists[raynodes[i]];
    Real closest = Inf;
    int result = -1;
    for(size_t k=0;k<pindices.size();k++) {
      const Vector3& pt=this->points[pindices[k]];
      Real t=r.closestPoint(pt,temp);
      if(pt.distanceSquared(temp) <= r2) {
	if(t < closest) {
	  closest = t;
	  result = ids[k];
	}
      }
    }
    if(result >= 0)
      return result;
  }
  return -1;
}

bool OctreePointSet::NearestNeighbor(const Vector3& c,Vector3& closest,int& id) const
{
  return !IsInf(_NearestNeighbor(nodes[0],c,closest,id,Inf));
}

Real OctreePointSet::_NearestNeighbor(const OctreeNode& n,const Vector3& c,Vector3& closest,int& id,Real minDist2) const
{
  if(!IsInf(minDist2)) {
    //pruning
    Vector3 temp;
    Real d2 = n.bb.distanceSquared(c,temp);
    if(d2 > minDist2) return minDist2;
  }
  if(IsLeaf(n)) {
    int index = Index(n);
    for(size_t i=0;i<indexLists[index].size();i++) {
      const Vector3& pt=points[indexLists[index][i]];
      Real d2 = c.distanceSquared(pt);
      if(d2 < minDist2) {
	closest = pt;
	id = ids[indexLists[index][i]];
	minDist2 = d2;
      }
    }
  }
  else {
    //first choose the child closest to c
    int cindex = Child(n,c);
    minDist2 = _NearestNeighbor(nodes[n.childIndices[cindex]],c,closest,id,minDist2);
    for(int i=0;i<8;i++) {
      if(i == cindex) continue;
      minDist2 = _NearestNeighbor(nodes[n.childIndices[i]],c,closest,id,minDist2);
    }
  }
  return minDist2;
}

void OctreePointSet::KNearestNeighbors(const Vector3& c,int k,vector<Vector3>& closest,vector<int>& ids) const
{
  closest.resize(k);
  ids.resize(k);
  vector<Real> distances(k,Inf);
  _KNearestNeighbors(nodes[0],c,closest,ids,distances,0);
}

int OctreePointSet::_KNearestNeighbors(const OctreeNode& n,const Vector3& c,vector<Vector3>& closest,vector<int>& ids,vector<Real>& distances,int kmin) const
{
  Real minDist2 = distances[kmin];
  if(!IsInf(minDist2)) {
    //pruning
    Vector3 temp;
    Real d2 = n.bb.distanceSquared(c,temp);
    if(d2 > minDist2) return kmin;
  }
  if(IsLeaf(n)) {
    int index = Index(n);
    for(size_t i=0;i<indexLists[index].size();i++) {
      const Vector3& pt = this->points[indexLists[index][i]];
      Real d2 = c.distanceSquared(pt);
      if(d2 < minDist2) {
	closest[kmin] = pt;
	ids[kmin] = this->ids[indexLists[index][i]];
	distances[kmin] = d2;
	//find the next farthest point
	for(size_t k=0;k<distances.size();k++) {
	  if(distances[k] > d2)
	    kmin = k;
	}
      }
    }
  }
  else {
    //first choose the child closest to c
    int cindex = Child(n,c);
    kmin = _KNearestNeighbors(nodes[n.childIndices[cindex]],c,closest,ids,distances,kmin);
    for(int i=0;i<8;i++) {
      if(i == cindex) continue;
      kmin = _KNearestNeighbors(nodes[n.childIndices[i]],c,closest,ids,distances,kmin);
    }
  }
    return kmin;
}

void OctreePointSet::FitToPoints()
{
  fit = true;
  balls.resize(nodes.size());
  //assume topological sort, go backwards
  for(size_t i=0;i<nodes.size();i++) {
    int index=(int)nodes.size()-1-(int)i;
    OctreeNode& n = nodes[index];
    Sphere3D& s=balls[index];
    s.center.setZero();
    s.radius=0;
    if(IsLeaf(n)) {
      const vector<int>& ptidx = indexLists[index];
      n.bb.minimize();
      for(size_t j=0;j<ptidx.size();j++) {
        n.bb.expand(points[ptidx[j]]);
        s.center += points[ptidx[j]];
      }
      Real r = 0;
      if(!ptidx.empty()) {
        s.center /= (Real)ptidx.size();
        for(size_t j=0;j<ptidx.size();j++) 
          r = Max(r,s.center.distanceSquared(points[ptidx[j]]));
        s.radius = Sqrt(r);
      }
    }
    else{
      //form bb from child bb's
      n.bb.minimize();
      for(int c=0;c<8;c++) 
        n.bb.setUnion(nodes[n.childIndices[c]].bb);
      Real sumw = 0;
      for(int c=0;c<8;c++) {
        //Real w=pow(balls[n.childIndices[c]].radius,3);
        Real w=1;
        if(balls[n.childIndices[c]].radius == 0) w = 0;
        s.center.madd(balls[n.childIndices[c]].center,w);
        sumw += w;
      }
      s.center /= sumw;
      Real r=0;
      for(int c=0;c<8;c++) {
        Real d = s.center.distance(balls[n.childIndices[c]].center) + balls[n.childIndices[c]].radius;
        r = Max(d,r);
      }
      s.radius = r;
    }
  }
}

void OctreePointSet::Collapse(int maxSize)
{
  //assume topological sort, go backwards
  for(size_t i=0;i<nodes.size();i++) {
    OctreeNode& n = nodes[nodes.size()-1-i];
    if(!IsLeaf(n)) {
      bool allempty = true;
      size_t sumsizes = 0;
      for(int c=0;c<8;c++) {
	if(!IsLeaf(nodes[n.childIndices[c]])) {
	  allempty = false;
	  break;
	}
	sumsizes += indexLists[n.childIndices[c]].size();
	if((int)sumsizes > maxSize) {
	  allempty = false;
	  break;
	}
      }
      if(allempty)
	Join(Index(n));
    }
  }
}


OctreeScalarField::OctreeScalarField(const AABB3D& bb,Real _defaultValue)
  :Octree(bb),defaultValue(_defaultValue)
{}

void OctreeScalarField::Set(const Vector3& pt,Real value,int id)
{
  OctreeNode* n=Lookup(pt);
  if(!n) return;
  Data* d=&data[Index(*n)];
  Real oldvalue = d->value;
  d->value = value;
  if(d->valueMin == defaultValue)
    d->valueMin = d->valueMax = value;
  else {
    if(value < d->valueMin)
      d->valueMin = value;
    else if(value > d->valueMax)
      d->valueMax = value;
  }
  d->id = id;
  //do backup
  while(n->parentIndex >= 0) {
    n = &nodes[n->parentIndex];
    Data* dp=&data[n->parentIndex];
    Real oldpvalue = dp->value;
    if(dp->valueMin == defaultValue) {
      dp->value = d->value;
      dp->valueMin = d->valueMin;
      dp->valueMax = d->valueMax;
    }
    else {
      dp->value += (d->value-oldvalue)/8.0;
      if(value < dp->valueMin)
	dp->valueMin = value;
      else if(value > dp->valueMax)
	dp->valueMax = value;
    }
    d = dp;
    oldvalue = oldpvalue;
  }
}

Real OctreeScalarField::Value(const Vector3& point) const
{
  const OctreeNode* n=&nodes[0];
  if(!n->bb.contains(point))
    return defaultValue;
  while(!IsLeaf(*n)) {
    int c=Child(*n,point);
    n = &nodes[n->childIndices[c]];
  }
  Assert(n->bb.contains(point));
  const Data& d =data[Index(*n)];
  return d.value;
}

bool OctreeScalarField::ValueIn(const Vector3& point,Real vmin,Real vmax) const
{
  const OctreeNode* n=&nodes[0];
  if(!n->bb.contains(point))
    return vmin <= defaultValue && defaultValue <= vmax;
  while(!IsLeaf(*n)) {
    const Data& d =data[Index(*n)];
    if(d.valueMax < vmin || d.valueMin > vmax) return false;
    if(d.valueMax < vmax && d.valueMin > vmin) return true;
    int c=Child(*n,point);
    n = &nodes[n->childIndices[c]];
  }
  Assert(n->bb.contains(point));
  const Data& d =data[Index(*n)];
  return vmin <= d.value && d.value <= vmax;  
}


bool OctreeScalarField::ValueGreater(const Vector3& point,Real bound) const
{
  return !ValueIn(point,-Inf,bound);
}

bool OctreeScalarField::ValueLess(const Vector3& point,Real bound) const
{
  return !ValueIn(point,bound,Inf);
}

int OctreeScalarField::AddNode(int parent)
{
  int n=Octree::AddNode(parent);
  if(n >= (int)data.size()) data.resize(n+1);
  data[n].value = data[n].valueMin = data[n].valueMax = defaultValue;
  data[n].id = -1;
  return n;
}

void OctreeScalarField::DeleteNode(int id)
{
  Octree::DeleteNode(id);
  data[id].value = data[id].valueMin = data[id].valueMax = defaultValue;
  data[id].id = -1;
}

void OctreeScalarField::Split(int nindex)
{
  Octree::Split(nindex);
  const OctreeNode& node=nodes[nindex];
  Data& d=data[nindex];
  for(int i=0;i<8;i++) {
    data[node.childIndices[i]].value = d.value;
    data[node.childIndices[i]].valueMin = d.valueMin;
    data[node.childIndices[i]].valueMax = d.valueMax;
    data[node.childIndices[i]].id = d.id;
  }
}

void OctreeScalarField::Join(int nindex)
{
  if(IsLeaf(nodes[nindex])) return;
  Data& d=data[nindex];
  const OctreeNode& node=nodes[nindex];
  d.value = 0.0;
  for(int i=0;i<8;i++)
    d.value += data[node.childIndices[i]].value;
  d.value /= 8.0;
  Octree::Join(nindex);
}

void OctreeScalarField::BoxLookupRange(const Vector3& bmin,const Vector3& bmax,Real valueMin,Real valueMax,vector<int>& nodeIndices,bool inclusive) const
{
  AABB3D bb(bmin,bmax);
  list<int> q;
  q.push_back(0);
  while(!q.empty()) {
    int n=q.front(); q.pop_front();
    if(inclusive) {
      if(data[n].valueMax < valueMin || data[n].valueMin > valueMax) continue;
    }
    else {
      if(data[n].valueMax <= valueMin || data[n].valueMin >= valueMax) continue;
    }
    if(!bb.intersects(nodes[n].bb)) continue;
    if(IsLeaf(nodes[n])) 
      nodeIndices.push_back(n);
    else {
      for(int i=0;i<8;i++)
	q.push_back(nodes[n].childIndices[i]);
    }
  }
}

void OctreeScalarField::BoxLookupGreater(const Vector3& bmin,const Vector3& bmax,Real bound,vector<int>& nodeIndices) const
{
  BoxLookupRange(bmin,bmax,bound,Inf,nodeIndices,false);
}

void OctreeScalarField::BoxLookupLess(const Vector3& bmin,const Vector3& bmax,Real bound,vector<int>& nodeIndices) const
{
  BoxLookupRange(bmin,bmax,-Inf,bound,nodeIndices,false);
}

void OctreeScalarField::Collapse(Real tolerance)
{
  for(size_t i=0;i<nodes.size();i++) {
    if(!IsLeaf(nodes[i])) {
      if(data[i].valueMax - data[i].valueMin <= tolerance)
	Join(i);
    }
  }
}
