#include "GridSubdivision.h"
#include "Grid.h"
#include <stdio.h>
#include <utils/indexing.h>
using namespace Geometry;
using namespace std;

typedef GridSubdivision::ObjectSet ObjectSet;
typedef GridSubdivision::Index Index;
typedef GridSubdivision::QueryCallback QueryCallback;

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



GridHash::GridHash(int numDims,Real _h)
  :h(numDims,_h)
{}

GridHash::GridHash(const Vector& _h)
  :h(_h)
{}

void GridHash::Set(const Index& i,void* data)
{
  assert((int)i.size()==h.n);
  buckets[i] = data;
}

void* GridHash::Erase(const Index& i)
{
  assert((int)i.size()==h.n);
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    void* v=bucket->second;
    buckets.erase(bucket);
    return v;
  }
  return NULL;;
}

void* GridHash::Get(const Index& i) const
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    return bucket->second;
  }
  return NULL;
}

void GridHash::Clear()
{
  buckets.clear();
}

void GridHash::Enumerate(std::vector<Value>& items) const
{
  items.resize(0);
  for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++)
    items.push_back(i->second);
}

void GridHash::PointToIndex(const Vector& p,Index& i) const
{
  assert(p.n == h.n);
  i.resize(p.n);
  for(int k=0;k<p.n;k++) {
    assert(h(k) > 0);
    i[k] = (int)Floor(p(k)/h(k));
  }
}

void GridHash::PointToIndex(const Vector& p,Index& i,Vector& u) const
{
  assert(p.n == h.n);
  i.resize(p.n);
  u.resize(p.n);
  for(int k=0;k<p.n;k++) {
    assert(h(k) > 0);
    u(k) = p(k)-Floor(p(k)/h(k));
    i[k] = (int)Floor(p(k)/h(k));
  }
}

void GridHash::IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const
{
  assert((int)i.size() == h.n);
  bmin.resize(h.n);
  bmax.resize(h.n);
  for(int k=0;k<h.n;k++) {
    bmin(k) = h(k)*(Real)i[k];
    bmax(k) = bmin(k) + h(k);
  }
}

void GridHash::GetRange(Index& imin,Index& imax) const
{
  if(buckets.empty()) {
    imin.resize(h.n);
    imax.resize(h.n);
    fill(imin.begin(),imin.end(),0);
    fill(imax.begin(),imax.end(),0);
    return;
  }
  imin=imax=buckets.begin()->first;
  for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
    const Index& idx=i->first;
    assert((int)idx.size() == h.n);
    for(size_t k=0;k<idx.size();i++) {
      if(idx[k] < imin[k]) imin[k] = idx[k];
      else if(idx[k] > imax[k]) imax[k] = idx[k];
    }
  }
}

void GridHash::GetRange(Vector& bmin,Vector& bmax) const
{
  if(buckets.empty()) {
    bmin.resize(h.n,0);
    bmax.resize(h.n,0);
    return;
  }
  Index imin,imax;
  GetRange(imin,imax);
  bmin.resize(h.n);
  bmax.resize(h.n);
  for(int k=0;k<h.n;k++) {
    bmin(k) = h(k)*(Real)imin[k];
    bmax(k) = h(k)*(Real)(imax[k]+1);
  }
}

bool GridHash::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  assert(h.n == (int)imin.size());
  assert(h.n == (int)imax.size());
  for(size_t k=0;k<imin.size();k++)
    assert(imin[k] <= imax[k]);

  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<imin.size();k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
      bool test = true;
      for(size_t k=0;k<imin.size();k++)
	if(i->first[k] < imin[k] || i->first[k] > imax[k]) {
	  test=false;
	  break;
	}
      if(test) {
	if(!f(i->second)) return false;
      }
    }
	return true;
  }
  else {
    Index i=imin;
    for(;;) {
      HashTable::const_iterator item = buckets.find(i);
      if(item != buckets.end())
	if(!f(item->second)) return false;
      if(IncrementIndex(i,imin,imax)!=0) break;
    }
    return true;
  }
}

bool GridHash::BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

bool GridHash::BallQuery(const Vector& c,Real r,QueryCallback f) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector u;
  PointToIndex(c,imin,u);
  for(int k=0;k<c.n;k++) {
    int ik=imin[k];
    Real r_over_h = r/h(k);
    imin[k] = ik - (int)Floor(u(k)-r_over_h);
    imax[k] = ik + (int)Floor(u(k)+r_over_h);
  }
  return IndexQuery(imin,imax,f);
}

//TODO: do this with a DDA algorithm
//bool GridHash::SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;



void GridHash::IndexItems(const Index& imin,const Index& imax,vector<void*>& objs) const
{
  assert(h.n == (int)imin.size());
  assert(h.n == (int)imax.size());
  for(size_t k=0;k<imin.size();k++)
    assert(imin[k] <= imax[k]);

  objs.clear();
  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<imin.size();k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
      bool test = true;
      for(size_t k=0;k<imin.size();k++)
	if(i->first[k] < imin[k] || i->first[k] > imax[k]) {
	  test=false;
	  break;
	}
      if(test) {
	objs.push_back(i->second);
      }
    }
  }
  else {
    Index i=imin;
    for(;;) {
      HashTable::const_iterator item=buckets.find(i);
      if(item != buckets.end())
	objs.push_back(item->second);
      if(IncrementIndex(i,imin,imax)!=0) break;
    }
  }
}

void GridHash::BoxItems(const Vector& bmin,const Vector& bmax,vector<void*>& objs) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  IndexItems(imin,imax,objs);
}

void GridHash::BallItems(const Vector& c,Real r,vector<void*>& objs) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector u;
  PointToIndex(c,imin,u);
  imax = imin;
  for(int k=0;k<c.n;k++) {
    int ik=imin[k];
    Real r_over_h = r/h(k);
    imin[k] = ik - (int)Floor(u(k)-r_over_h);
    imax[k] = ik + (int)Floor(u(k)+r_over_h);
  }
  IndexItems(imin,imax,objs);
}





bool EraseObject(ObjectSet& b,void* data)
{
  for(ObjectSet::iterator i=b.begin();i!=b.end();i++) {
    if(*i == data) {
      b.erase(i);
      return true;
    }
  }
  return false;
}

bool QueryObjects(const ObjectSet& b,QueryCallback f)
{
  for(ObjectSet::const_iterator i=b.begin();i!=b.end();i++) {
    if(!f(*i)) return false;
  }
  return true;
}

GridSubdivision::GridSubdivision(int numDims,Real _h)
  :h(numDims,_h)
{}

GridSubdivision::GridSubdivision(const Vector& _h)
  :h(_h)
{}

void GridSubdivision::Insert(const Index& i,void* data)
{
  assert((int)i.size()==h.n);
  buckets[i].push_back(data);
}

bool GridSubdivision::Erase(const Index& i,void* data)
{
  assert((int)i.size()==h.n);
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    bool res=EraseObject(bucket->second,data);
    if(bucket->second.empty())
      buckets.erase(bucket);
    return res;
  }
  return false;
}

GridSubdivision::ObjectSet* GridSubdivision::GetObjectSet(const Index& i)
{
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    return &bucket->second;
  }
  return NULL;
}

const GridSubdivision::ObjectSet* GridSubdivision::GetObjectSet(const Index& i) const
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    return &bucket->second;
  }
  return NULL;
}

void GridSubdivision::Clear()
{
  buckets.clear();
}

void GridSubdivision::PointToIndex(const Vector& p,Index& i) const
{
  assert(p.n == h.n);
  i.resize(p.n);
  for(int k=0;k<p.n;k++) {
    assert(h(k) > 0);
    i[k] = (int)Floor(p(k)/h(k));
  }
}

void GridSubdivision::PointToIndex(const Vector& p,Index& i,Vector& u) const
{
  assert(p.n == h.n);
  i.resize(p.n);
  u.resize(p.n);
  for(int k=0;k<p.n;k++) {
    assert(h(k) > 0);
    u(k) = p(k)-Floor(p(k)/h(k));
    i[k] = (int)Floor(p(k)/h(k));
  }
}

void GridSubdivision::IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const
{
  assert((int)i.size() == h.n);
  bmin.resize(h.n);
  bmax.resize(h.n);
  for(int k=0;k<h.n;k++) {
    bmin(k) = h(k)*(Real)i[k];
    bmax(k) = bmin(k) + h(k);
  }
}

void GridSubdivision::GetRange(Index& imin,Index& imax) const
{
  if(buckets.empty()) {
    imin.resize(h.n);
    imax.resize(h.n);
    fill(imin.begin(),imin.end(),0);
    fill(imax.begin(),imax.end(),0);
    return;
  }
  imin=imax=buckets.begin()->first;
  for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
    const Index& idx=i->first;
    assert((int)idx.size() == h.n);
    for(size_t k=0;k<idx.size();k++) {
      if(idx[k] < imin[k]) imin[k] = idx[k];
      else if(idx[k] > imax[k]) imax[k] = idx[k];
    }
  }
}

void GridSubdivision::GetRange(Vector& bmin,Vector& bmax) const
{
  if(buckets.empty()) {
    bmin.resize(h.n,0);
    bmax.resize(h.n,0);
    return;
  }
  Index imin,imax;
  GetRange(imin,imax);
  bmin.resize(h.n);
  bmax.resize(h.n);
  for(int k=0;k<h.n;k++) {
    bmin(k) = h(k)*(Real)imin[k];
    bmax(k) = h(k)*(Real)(imax[k]+1);
  }
}

bool GridSubdivision::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  assert(h.n == (int)imin.size());
  assert(h.n == (int)imax.size());
  for(size_t k=0;k<imin.size();k++)
    assert(imin[k] <= imax[k]);

  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<imin.size();k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    printf("GridSubdivision: All-bucket iterating\n");
    for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
      bool test = true;
      for(size_t k=0;k<imin.size();k++)
	if(i->first[k] < imin[k] || i->first[k] > imax[k]) {
	  test=false;
	  break;
	}
      if(test) {
	if(!QueryObjects(i->second,f)) return false;
      }
    }
  }
  else {
    printf("GridSubdivision: Range iterating\n");
    Index i=imin;
    for(;;) {
      HashTable::const_iterator item = buckets.find(i);
      if(item != buckets.end())
	if(!QueryObjects(item->second,f)) return false;
      if(IncrementIndex(i,imin,imax)!=0) break;
    }
  }
  return true;
}

bool GridSubdivision::BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

bool GridSubdivision::BallQuery(const Vector& c,Real r,QueryCallback f) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector bmin=c,bmax=c;
  for(int k=0;k<c.n;k++) bmin[k] -= r;
  for(int k=0;k<c.n;k++) bmax[k] += r;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

//TODO: do this with a DDA algorithm
//bool GridSubdivision::SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;



void GridSubdivision::IndexItems(const Index& imin,const Index& imax,ObjectSet& objs) const
{
  assert(h.n == (int)imin.size());
  assert(h.n == (int)imax.size());
  for(size_t k=0;k<imin.size();k++)
    assert(imin[k] <= imax[k]);

  objs.clear();
  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<imin.size();k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
      bool test = true;
      for(size_t k=0;k<imin.size();k++)
	if(i->first[k] < imin[k] || i->first[k] > imax[k]) {
	  test=false;
	  break;
	}
      if(test) {
	objs.insert(objs.end(),i->second.begin(),i->second.end());
      }
    }
  }
  else {
    Index i=imin;
    for(;;) {
      HashTable::const_iterator item=buckets.find(i);
      if(item != buckets.end())
	objs.insert(objs.end(),item->second.begin(),item->second.end());
      if(IncrementIndex(i,imin,imax)!=0) break;
    }
  }
}

void GridSubdivision::BoxItems(const Vector& bmin,const Vector& bmax,ObjectSet& objs) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  IndexItems(imin,imax,objs);
}

void GridSubdivision::BallItems(const Vector& c,Real r,ObjectSet& objs) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector bmin=c,bmax=c;
  for(int k=0;k<c.n;k++) bmin[k] -= r;
  for(int k=0;k<c.n;k++) bmax[k] += r;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  IndexItems(imin,imax,objs);
}
