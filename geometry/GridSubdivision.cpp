#include <KrisLibrary/Logger.h>
#include <KrisLibrary/errors.h>
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

size_t IndexHash::operator () (const IntTriple& x) const
{
  size_t res=0;
  int p=1;
  for(int i=0;i<3;i++) {
    res ^= p*x[i];
    p *= pow;
  }
  return res;
}

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
  :hinv(numDims,1.0/_h)
{}

GridHash::GridHash(const Vector& _h)
  :hinv(_h.n)
{
  for(int i=0;i<hinv.n;i++) hinv[i] = 1.0/_h[i];
}

Vector GridHash::GetResolution() const
{
  Vector h(hinv.n);
  for(int i=0;i<hinv.n;i++) h[i] = 1.0/hinv[i];
  return h;
}

void GridHash::SetResolution(const Vector& h)
{
  Assert(buckets.empty());
  hinv.resize(h.n);
  for(int i=0;i<hinv.n;i++) hinv[i] = 1.0/h[i];
}

void GridHash::SetResolution(Real h)
{
  for(int i=0;i<hinv.n;i++) hinv[i] = 1.0/h;
}

void GridHash::Set(const Index& i,void* data)
{
  assert((int)i.size()==hinv.n);
  buckets[i] = data;
}

void* GridHash::Erase(const Index& i)
{
  assert((int)i.size()==hinv.n);
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
  for(auto i : buckets)
    items.push_back(i.second);
}

void GridHash::PointToIndex(const Vector& p,Index& i) const
{
  assert(p.n == hinv.n);
  i.resize(p.n);
  for(int k=0;k<p.n;k++) {
    assert(hinv[k] > 0);
    i[k] = (int)Floor(p[k]*hinv[k]);
  }
}

void GridHash::PointToIndex(const Vector& p,Index& i,Vector& u) const
{
  assert(p.n == hinv.n);
  i.resize(p.n);
  u.resize(p.n);
  for(int k=0;k<p.n;k++) {
    assert(hinv[k] > 0);
    u[k] = p[k]-Floor(p[k]*hinv[k]);
    i[k] = (int)Floor(p[k]*hinv[k]);
  }
}

void GridHash::IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const
{
  assert((int)i.size() == hinv.n);
  bmin.resize(hinv.n);
  bmax.resize(hinv.n);
  for(int k=0;k<hinv.n;k++) {
    Real hk = 1.0/hinv[k];
    bmin[k] = (Real)i[k]*hk;
    bmax[k] = bmin[k] + hk;
  }
}

void GridHash::GetRange(Index& imin,Index& imax) const
{
  if(buckets.empty()) {
    imin.resize(hinv.n);
    imax.resize(hinv.n);
    fill(imin.begin(),imin.end(),0);
    fill(imax.begin(),imax.end(),0);
    return;
  }
  imin=imax=buckets.begin()->first;
  for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
    const Index& idx=i->first;
    assert((int)idx.size() == hinv.n);
    for(size_t k=0;k<idx.size();k++) {
      if(idx[k] < imin[k]) imin[k] = idx[k];
      else if(idx[k] > imax[k]) imax[k] = idx[k];
    }
  }
}

void GridHash::GetRange(Vector& bmin,Vector& bmax) const
{
  if(buckets.empty()) {
    bmin.resize(hinv.n,0);
    bmax.resize(hinv.n,0);
    return;
  }
  Index imin,imax;
  GetRange(imin,imax);
  bmin.resize(hinv.n);
  bmax.resize(hinv.n);
  for(int k=0;k<hinv.n;k++) {
    bmin[k] = (Real)imin[k] / hinv[k];
    bmax[k] = (Real)(imax[k]+1) / hinv[k];
  }
}

bool GridHash::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  assert(hinv.n == (int)imin.size());
  assert(hinv.n == (int)imax.size());
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
    Real r_over_h = r*hinv[k];
    imin[k] = ik - (int)Floor(u[k]-r_over_h);
    imax[k] = ik + (int)Floor(u[k]+r_over_h);
  }
  return IndexQuery(imin,imax,f);
}

//TODO: do this with a DDA algorithm
//bool GridHash::SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;



void GridHash::IndexItems(const Index& imin,const Index& imax,vector<void*>& objs) const
{
  assert(hinv.n == (int)imin.size());
  assert(hinv.n == (int)imax.size());
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
    Real r_over_h = r*hinv[k];
    imin[k] = ik - (int)Floor(u[k]-r_over_h);
    imax[k] = ik + (int)Floor(u[k]+r_over_h);
  }
  IndexItems(imin,imax,objs);
}



GridHash3D::GridHash3D(Real _h)
  :hinv(1.0/_h)
{}

GridHash3D::GridHash3D(const Vector3& _h)
{
  for(int i=0;i<3;i++) hinv[i] = 1.0/_h[i];
}

Vector3 GridHash3D::GetResolution() const
{
  Vector3 h;
  for(int i=0;i<3;i++) h[i] = 1.0/hinv[i];
  return h;
}

void GridHash3D::SetResolution(const Vector3& h)
{
  Assert(buckets.empty());
  for(int i=0;i<3;i++) hinv[i] = 1.0/h[i];
}

void GridHash3D::SetResolution(Real h)
{
  for(int i=0;i<3;i++) hinv[i] = 1.0/h;
}

void GridHash3D::Set(const Index& i,void* data)
{
  buckets[i] = data;
}

void* GridHash3D::Erase(const Index& i)
{
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    void* v=bucket->second;
    buckets.erase(bucket);
    return v;
  }
  return NULL;;
}

void* GridHash3D::Get(const Index& i) const
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    return bucket->second;
  }
  return NULL;
}

void GridHash3D::Clear()
{
  buckets.clear();
}

void GridHash3D::Enumerate(std::vector<Value>& items) const
{
  items.resize(0);
  for(auto i : buckets)
    items.push_back(i.second);
}

void GridHash3D::PointToIndex(const Vector3& p,Index& i) const
{
  for(int k=0;k<3;k++) {
    i[k] = (int)Floor(p[k]*hinv[k]);
  }
}

void GridHash3D::PointToIndex(const Vector3& p,Index& i,Vector3& u) const
{
  for(int k=0;k<3;k++) {
    u[k] = p[k]-Floor(p[k]*hinv[k]);
    i[k] = (int)Floor(p[k]*hinv[k]);
  }
}

void GridHash3D::IndexBucketBounds(const Index& i,Vector3& bmin,Vector3& bmax) const
{
  for(int k=0;k<3;k++) {
    Real hk = 1.0/hinv[k];
    bmin[k] = (Real)i[k]*hk;
    bmax[k] = bmin[k] + hk;
  }
}

void GridHash3D::GetRange(Index& imin,Index& imax) const
{
  if(buckets.empty()) {
    imin.set(0,0,0);
    imax.set(0,0,0);
    return;
  }
  imin=imax=buckets.begin()->first;
  for(auto i:buckets) {
    const Index& idx=i.first;
    for(size_t k=0;k<3;k++) {
      if(idx[k] < imin[k]) imin[k] = idx[k];
      else if(idx[k] > imax[k]) imax[k] = idx[k];
    }
  }
}

void GridHash3D::GetRange(Vector3& bmin,Vector3& bmax) const
{
  if(buckets.empty()) {
    bmin.set(0.0);
    bmax.set(0.0);
    return;
  }
  Index imin,imax;
  GetRange(imin,imax);
  for(int k=0;k<3;k++) {
    bmin[k] = (Real)imin[k] / hinv[k];
    bmax[k] = (Real)(imax[k]+1) / hinv[k];
  }
}

bool GridHash3D::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<3;k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    for(auto i : buckets) {
      bool test = true;
      for(size_t k=0;k<3;k++)
        if(i.first[k] < imin[k] || i.first[k] > imax[k]) {
          test=false;
          break;
        }
      if(test) {
        if(!f(i.second)) return false;
      }
    }
    return true;
  }
  else {
    for(int i=imin.a;i<=imax.a;i++) {
      for(int j=imin.b;j<=imax.b;j++) {
        for(int k=imin.c;k<=imax.c;k++) {
          IntTriple index(i,j,k);
          auto b = buckets.find(index);
          if(b==buckets.end()) continue;
          if(!f(b->second)) return false;
        }
      }
    }
    return true;
  }
}

bool GridHash3D::BoxQuery(const Vector3& bmin,const Vector3& bmax,QueryCallback f) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

bool GridHash3D::BallQuery(const Vector3& c,Real r,QueryCallback f) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector3 u;
  PointToIndex(c,imin,u);
  for(int k=0;k<3;k++) {
    int ik=imin[k];
    Real r_over_h = r*hinv[k];
    imin[k] = ik - (int)Floor(u[k]-r_over_h);
    imax[k] = ik + (int)Floor(u[k]+r_over_h);
  }
  return IndexQuery(imin,imax,f);
}

//TODO: do this with a DDA algorithm
//bool GridHash3D::SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;


void GridHash3D::IndexItems(const Index& imin,const Index& imax,vector<void*>& objs) const
{
  objs.clear();
  int numBuckets = 1;
  for(size_t k=0;k<3;k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    for(auto i : buckets) {
      bool test = true;
      for(size_t k=0;k<3;k++)
        if(i.first[k] < imin[k] || i.first[k] > imax[k]) {
          test=false;
          break;
        }
      if(test) {
        objs.push_back(i.second);
      }
    }
  }
  else {
    for(int i=imin.a;i<=imax.a;i++) {
      for(int j=imin.b;j<=imax.b;j++) {
        for(int k=imin.c;k<=imax.c;k++) {
          IntTriple index(i,j,k);
          auto b = buckets.find(index);
          if(b==buckets.end()) continue;
          objs.push_back(b->second);
        }
      }
    }
  }
}

void GridHash3D::BoxItems(const Vector3& bmin,const Vector3& bmax,vector<void*>& objs) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  IndexItems(imin,imax,objs);
}

void GridHash3D::BallItems(const Vector3& c,Real r,vector<void*>& objs) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector3 u;
  PointToIndex(c,imin,u);
  imax = imin;
  for(int k=0;k<3;k++) {
    int ik=imin[k];
    Real r_over_h = r*hinv[k];
    imin[k] = ik - (int)Floor(u[k]-r_over_h);
    imax[k] = ik + (int)Floor(u[k]+r_over_h);
  }
  IndexItems(imin,imax,objs);
}








bool EraseObject(list<void*>& b,void* data)
{
  for(list<void*>::iterator i=b.begin();i!=b.end();i++) {
    if(*i == data) {
      b.erase(i);
      return true;
    }
  }
  return false;
}

bool EraseObject(vector<void*>& b,void* data)
{
  for(size_t i=0;i<b.size();i++) {
    if(b[i] == data) {
      b[i] = b.back();
      b.resize(b.size()-1);
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
  :hinv(numDims,1.0/_h)
{
  for(int i=0;i<hinv.n;i++) 
    assert(hinv[i] > 0 && IsFinite(hinv[i]));
}

GridSubdivision::GridSubdivision(const Vector& _h)
  :hinv(_h.n)
{
   for(int i=0;i<hinv.n;i++) {
    hinv[i] = 1.0/_h[i];
    assert(hinv[i] > 0 && IsFinite(hinv[i]));
  }
}

void GridSubdivision::Insert(const Index& i,void* data)
{
  assert((int)i.size()==hinv.n);
  buckets[i].push_back(data);
}

bool GridSubdivision::Erase(const Index& i,void* data)
{
  assert((int)i.size()==hinv.n);
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
  assert(p.n == hinv.n);
  i.resize(p.n);
  for(int k=0;k<p.n;k++) {
    i[k] = (int)Floor(p[k]*hinv[k]);
  }
}

void GridSubdivision::PointToIndex(const Vector& p,Index& i,Vector& u) const
{
  assert(p.n == hinv.n);
  i.resize(p.n);
  u.resize(p.n);
  for(int k=0;k<p.n;k++) {
    Real s = Floor(p[k]*hinv[k]);
    u[k] = p[k]-s;
    i[k] = (int)s;
  }
}

void GridSubdivision::IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const
{
  assert((int)i.size() == hinv.n);
  bmin.resize(hinv.n);
  bmax.resize(hinv.n);
  for(int k=0;k<hinv.n;k++) {
    bmin[k] = (Real)i[k]/hinv[k];
    bmax[k] = bmin[k] + 1.0/hinv[k];
  }
}

void GridSubdivision::GetRange(Index& imin,Index& imax) const
{
  if(buckets.empty()) {
    imin.resize(hinv.n);
    imax.resize(hinv.n);
    fill(imin.begin(),imin.end(),0);
    fill(imax.begin(),imax.end(),0);
    return;
  }
  imin=imax=buckets.begin()->first;
  for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
    const Index& idx=i->first;
    assert((int)idx.size() == hinv.n);
    for(size_t k=0;k<idx.size();k++) {
      if(idx[k] < imin[k]) imin[k] = idx[k];
      else if(idx[k] > imax[k]) imax[k] = idx[k];
    }
  }
}

void GridSubdivision::GetRange(Vector& bmin,Vector& bmax) const
{
  if(buckets.empty()) {
    bmin.resize(hinv.n,0);
    bmax.resize(hinv.n,0);
    return;
  }
  Index imin,imax;
  GetRange(imin,imax);
  bmin.resize(hinv.n);
  bmax.resize(hinv.n);
  for(int k=0;k<hinv.n;k++) {
    bmin[k] = (Real)imin[k]/hinv[k];
    bmax[k] = (Real)(imax[k]+1)/hinv[k];
  }
}

bool GridSubdivision::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  assert(hinv.n == (int)imin.size());
  assert(hinv.n == (int)imax.size());
  for(size_t k=0;k<imin.size();k++)
    assert(imin[k] <= imax[k]);

  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<imin.size();k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"GridSubdivision: All-bucket iterating\n");
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
    LOG4CXX_INFO(KrisLibrary::logger(),"GridSubdivision: Range iterating\n");
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
  assert(hinv.n == (int)imin.size());
  assert(hinv.n == (int)imax.size());
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






GridSubdivision3D::GridSubdivision3D(Real _h)
  :hinv(1.0/_h)
{}

GridSubdivision3D::GridSubdivision3D(const Vector3& _h)
{
   for(int i=0;i<3;i++) hinv[i] = 1.0/_h[i];
}

void GridSubdivision3D::Insert(const Index& i,void* data)
{
  buckets[i].push_back(data);
}

bool GridSubdivision3D::Erase(const Index& i,void* data)
{
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    bool res=EraseObject(bucket->second,data);
    if(bucket->second.empty())
      buckets.erase(bucket);
    return res;
  }
  return false;
}

GridSubdivision3D::ObjectSet* GridSubdivision3D::GetObjectSet(const Index& i)
{
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    return &bucket->second;
  }
  return NULL;
}

const GridSubdivision3D::ObjectSet* GridSubdivision3D::GetObjectSet(const Index& i) const
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    return &bucket->second;
  }
  return NULL;
}

void GridSubdivision3D::Clear()
{
  buckets.clear();
}

void GridSubdivision3D::PointToIndex(const Vector3& p,Index& i) const
{
  for(int k=0;k<3;k++) {
    assert(hinv[k] > 0);
    i[k] = (int)Floor(p[k]*hinv[k]);
  }
}

void GridSubdivision3D::PointToIndex(const Vector3& p,Index& i,Vector3& u) const
{
  for(int k=0;k<3;k++) {
    assert(hinv[k] > 0);
    Real s = Floor(p[k]*hinv[k]);
    u[k] = p[k]-s;
    i[k] = (int)s;
  }
}

void GridSubdivision3D::IndexBucketBounds(const Index& i,Vector3& bmin,Vector3& bmax) const
{
  for(int k=0;k<3;k++) {
    bmin[k] = (Real)i[k]/hinv[k];
    bmax[k] = bmin[k] + 1.0/hinv[k];
  }
}

void GridSubdivision3D::GetRange(Index& imin,Index& imax) const
{
  if(buckets.empty()) {
    imin.set(0,0,0);
    imax.set(0,0,0);
    return;
  }
  imin=imax=buckets.begin()->first;
  for(auto i: buckets) {
    const Index& idx=i.first;
    for(size_t k=0;k<3;k++) {
      if(idx[k] < imin[k]) imin[k] = idx[k];
      else if(idx[k] > imax[k]) imax[k] = idx[k];
    }
  }
}

void GridSubdivision3D::GetRange(Vector3& bmin,Vector3& bmax) const
{
  if(buckets.empty()) {
    bmin.set(0.0);
    bmax.set(0.0);
    return;
  }
  Index imin,imax;
  GetRange(imin,imax);
  for(int k=0;k<3;k++) {
    bmin[k] = (Real)imin[k]/hinv[k];
    bmax[k] = (Real)(imax[k]+1)/hinv[k];
  }
}

bool GridSubdivision3D::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  for(size_t k=0;k<3;k++)
    assert(imin[k] <= imax[k]);

  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<3;k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"GridSubdivision3D: All-bucket iterating\n");
    for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
      bool test = true;
      for(size_t k=0;k<3;k++)
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
    LOG4CXX_INFO(KrisLibrary::logger(),"GridSubdivision3D: Range iterating\n");
    for(int i=imin.a;i<=imax.a;i++) {
      for(int j=imin.b;j<=imax.b;j++) {
        for(int k=imin.c;k<=imax.c;k++) {
          HashTable::const_iterator item=buckets.find(IntTriple(i,j,k));
          if(item != buckets.end())
            if(!QueryObjects(item->second,f)) return false;
        }
      }
    }
  }
  return true;
}

bool GridSubdivision3D::BoxQuery(const Vector3& bmin,const Vector3& bmax,QueryCallback f) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

bool GridSubdivision3D::BallQuery(const Vector3& c,Real r,QueryCallback f) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector3 bmin=c,bmax=c;
  for(int k=0;k<3;k++) bmin[k] -= r;
  for(int k=0;k<3;k++) bmax[k] += r;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

//TODO: do this with a DDA algorithm
//bool GridSubdivision3D::SegmentQuery(const Vector3& a,const Vector3& b,QueryCallback f) const;



void GridSubdivision3D::IndexItems(const Index& imin,const Index& imax,ObjectSet& objs) const
{
  for(size_t k=0;k<3;k++)
    assert(imin[k] <= imax[k]);

  objs.clear();
  //check to see if we should just loop through all buckets
  int numBuckets = 1;
  for(size_t k=0;k<3;k++)
    numBuckets *= (imax[k]-imin[k]+1);
  if(numBuckets >= (int)buckets.size()) {
    for(HashTable::const_iterator i=buckets.begin();i!=buckets.end();i++) {
      bool test = true;
      for(size_t k=0;k<3;k++)
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
    for(int i=imin.a;i<=imax.a;i++) {
      for(int j=imin.b;j<=imax.b;j++) {
        for(int k=imin.c;k<=imax.c;k++) {
          HashTable::const_iterator item=buckets.find(IntTriple(i,j,k));
          if(item != buckets.end())
            objs.insert(objs.end(),item->second.begin(),item->second.end());
        }
      }
    }
  }
}

void GridSubdivision3D::BoxItems(const Vector3& bmin,const Vector3& bmax,ObjectSet& objs) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  IndexItems(imin,imax,objs);
}

void GridSubdivision3D::BallItems(const Vector3& c,Real r,ObjectSet& objs) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector3 bmin=c,bmax=c;
  for(int k=0;k<3;k++) bmin[k] -= r;
  for(int k=0;k<3;k++) bmax[k] += r;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  IndexItems(imin,imax,objs);
}
