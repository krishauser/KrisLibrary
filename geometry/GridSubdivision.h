#ifndef GEOMETRY_GRID_SUBDIVISION_H
#define GEOMETRY_GRID_SUBDIVISION_H

#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/IntTuple.h>
#include <KrisLibrary/utils/IntTriple.h>
#include <list>
#include <stdint.h>
#include <KrisLibrary/errors.h>
//this is needed for unordered_map compatibility across compilers / OSs
#include <KrisLibrary/utils/stl_tr1.h>


namespace Geometry {

using namespace Math;
using namespace Math3D;

struct IndexHash
{
  explicit IndexHash(size_t pow=257);
  size_t operator () (const IntTriple& x) const;
  size_t operator () (const std::vector<int>& x) const;
  size_t pow;
};

/** @ingroup Geometry
 * @brief A grid containing objects (referenced by void pointers)
 *
 * The map from a point x to an index is floor(x./h) where ./ indicates
 * element-wise division, and h is the resolution parameter defined on
 * construction (see Grid).
 */
class GridHash3D
{
public:
  typedef IntTriple Index;
  typedef void* Value;
  ///called once per value in the query range, return false to stop enumerating
  typedef bool (*QueryCallback)(void* value);

  explicit GridHash3D(Real h=1);
  explicit GridHash3D(const Vector3& h);
  size_t GetBucketCount() const {  return buckets.bucket_count(); }
  void SetBucketCount(size_t n) {  buckets.rehash(n); }
  Vector3 GetResolution() const;
  ///Changes the resolution after construction -- need to have all buckets empty
  void SetResolution(const Vector3& h);
  ///Changes the resolution after construction -- need to have all buckets empty
  void SetResolution(Real h);
  ///Sets the data at a given index
  void Set(const Index& i,void* data);
  ///Retrieves the data at a given index
  void* Get(const Index& i) const;
  ///Important: this method just removes the item from the hash, but does not delete its memory
  void* Erase(const Index& i); 
  ///Returns true if the hash contains the given index
  bool Contains(const Index& i);
  void Clear();
  void Enumerate(std::vector<Value>& items) const;

  //returns the index of the point
  void PointToIndex(const Vector3& p,Index& i) const;
  //same, but with the local coordinates in the bucket [0,1]^n
  void PointToIndex(const Vector3& p,Index& i,Vector3& u) const;
  //returns the lower/upper corner of the bucket
  void IndexBucketBounds(const Index& i,Vector3& bmin,Vector3& bmax) const;

  //returns the min/max indices of all occupied cells
  void GetRange(Index& imin,Index& imax) const;
  //returns the min/max bound of all occupied cells
  void GetRange(Vector3& bmin,Vector3& bmax) const;

  //range imin to imax
  bool IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const;
  //bounding box from bmin to bmax
  bool BoxQuery(const Vector3& bmin,const Vector3& bmax,QueryCallback f) const;
  //ball with center c, radius r
  bool BallQuery(const Vector3& c,Real r,QueryCallback f) const;
  //segment from a to b
  bool SegmentQuery(const Vector3& a,const Vector3& b,QueryCallback f) const;

  //range imin to imax
  void IndexItems(const Index& imin,const Index& imax,std::vector<Value>& items) const;
  //bounding box from bmin to bmax
  void BoxItems(const Vector3& bmin,const Vector3& bmax,std::vector<Value>& items) const;
  //ball with center c, radius r
  void BallItems(const Vector3& c,Real r,std::vector<Value>& items) const;
  //segment from a to b
  void SegmentItems(const Vector3& a,const Vector3& b,std::vector<Value>& items) const;

  Vector3 hinv;
  typedef UNORDERED_MAP_TEMPLATE<Index,Value,IndexHash> HashTable;
  HashTable buckets;
};

/** @ingroup Geometry
 * @brief A grid with a list of arbitrary objects (given by void pointers)
 */
class GridSubdivision3D
{
public:
  typedef IntTriple Index;
  typedef std::vector<void*> ObjectSet;
  //called once per object in the query range, return false to stop enumerating
  typedef bool (*QueryCallback)(void* obj);

  explicit GridSubdivision3D(Real h=1);
  explicit GridSubdivision3D(const Vector3& h);
  size_t GetBucketCount() const {  return buckets.bucket_count(); }
  void SetBucketCount(size_t n) {  buckets.rehash(n); }
  //this doesn't work -- hash power can't currently be changed.
  //void SetHashPower(size_t n) {  buckets.hash_function().pow=n; }
  ///Inserts a new item.  Important: this doesn't take ownership of the object
  void Insert(const Index& i,void* data);
  ///Important: this method just removes the item from the hash, but does not delete its memory
  bool Erase(const Index& i,void* data);
  ObjectSet* GetObjectSet(const Index& i);
  const ObjectSet* GetObjectSet(const Index& i) const;
  void Clear();
  void Enumerate(std::vector<void*>& items) const;

  //returns the index of the point
  void PointToIndex(const Vector3& p,Index& i) const;
  //same, but with the local coordinates in the bucket [0,1]^n
  void PointToIndex(const Vector3& p,Index& i,Vector3& u) const;
  //returns the lower/upper corner of the bucket
  void IndexBucketBounds(const Index& i,Vector3& bmin,Vector3& bmax) const;

  //returns the min/max indices of all occupied cells
  void GetRange(Index& imin,Index& imax) const;
  //returns the min/max bound of all occupied cells
  void GetRange(Vector3& bmin,Vector3& bmax) const;

  //range imin to imax
  bool IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const;
  //bounding box from bmin to bmax
  bool BoxQuery(const Vector3& bmin,const Vector3& bmax,QueryCallback f) const;
  //ball with center c, radius r
  bool BallQuery(const Vector3& c,Real r,QueryCallback f) const;
  //segment from a to b
  bool SegmentQuery(const Vector3& a,const Vector3& b,QueryCallback f) const;

  //range imin to imax
  void IndexItems(const Index& imin,const Index& imax,ObjectSet& objs) const;
  //bounding box from bmin to bmax
  void BoxItems(const Vector3& bmin,const Vector3& bmax,ObjectSet& objs) const;
  //ball with center c, radius r
  void BallItems(const Vector3& c,Real r,ObjectSet& objs) const;
  //segment from a to b
  void SegmentItems(const Vector3& a,const Vector3& b,ObjectSet& objs) const;

  Vector3 hinv;
  typedef UNORDERED_MAP_TEMPLATE<Index,ObjectSet,IndexHash> HashTable;
  HashTable buckets;
};


/** @ingroup Geometry
 * @brief An ND grid containing objects (referenced by void pointers)
 *
 * The map from a point x to an index is floor(x./h) where ./ indicates
 * element-wise division, and h is the resolution parameter defined on
 * construction (see Grid).
 */
class GridHash
{
public:
  typedef IntTuple Index;
  typedef void* Value;
  //called once per value in the query range, return false to stop enumerating
  typedef bool (*QueryCallback)(void* value);

  explicit GridHash(int numDims,Real h=1);
  explicit GridHash(const Vector& h);
  size_t GetBucketCount() const {  return buckets.bucket_count(); }
  void SetBucketCount(size_t n) {  buckets.rehash(n); }
  Vector GetResolution() const;
  ///Changes the resolution after construction -- need to have all buckets empty
  void SetResolution(const Vector& h);
  ///Changes the resolution after construction -- need to have all buckets empty
  void SetResolution(Real h);
  ///Sets the data at a given index
  void Set(const Index& i,void* data);
  ///Retrieves the data at a given index
  void* Get(const Index& i) const;
  ///Important: this method just removes the item from the hash, but does not delete its memory
  void* Erase(const Index& i); 
  ///Returns true if the hash contains the given index
  bool Contains(const Index& i);
  void Clear();
  void Enumerate(std::vector<Value>& items) const;

  //returns the index of the point
  void PointToIndex(const Vector& p,Index& i) const;
  //same, but with the local coordinates in the bucket [0,1]^n
  void PointToIndex(const Vector& p,Index& i,Vector& u) const;
  //returns the lower/upper corner of the bucket
  void IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const;

  //returns the min/max indices of all occupied cells
  void GetRange(Index& imin,Index& imax) const;
  //returns the min/max bound of all occupied cells
  void GetRange(Vector& bmin,Vector& bmax) const;

  //range imin to imax
  bool IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const;
  //bounding box from bmin to bmax
  bool BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const;
  //ball with center c, radius r
  bool BallQuery(const Vector& c,Real r,QueryCallback f) const;
  //segment from a to b
  bool SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;

  //range imin to imax
  void IndexItems(const Index& imin,const Index& imax,std::vector<Value>& items) const;
  //bounding box from bmin to bmax
  void BoxItems(const Vector& bmin,const Vector& bmax,std::vector<Value>& items) const;
  //ball with center c, radius r
  void BallItems(const Vector& c,Real r,std::vector<Value>& items) const;
  //segment from a to b
  void SegmentItems(const Vector& a,const Vector& b,std::vector<Value>& items) const;

  Vector hinv;
  typedef UNORDERED_MAP_TEMPLATE<Index,Value,IndexHash> HashTable;
  HashTable buckets;
};


/** @ingroup Geometry
 * @brief An ND grid with a list of arbitrary objects (given by void pointers)
 */
class GridSubdivision
{
public:
  typedef IntTuple Index;
  typedef std::vector<void*> ObjectSet;
  //called once per object in the query range, return false to stop enumerating
  typedef bool (*QueryCallback)(void* obj);

  explicit GridSubdivision(int numDims,Real h=1);
  explicit GridSubdivision(const Vector& h);
  size_t GetBucketCount() const {  return buckets.bucket_count(); }
  void SetBucketCount(size_t n) {  buckets.rehash(n); }
  //this doesn't work -- hash power can't currently be changed.
  //void SetHashPower(size_t n) {  buckets.hash_function().pow=n; }
  ///Inserts a new item.  Important: this doesn't take ownership of the object
  void Insert(const Index& i,void* data);
  ///Erases an object. Runs in O(k) time where k is the # of items in bucket[i].
  ///Important: this method just removes the item from the hash, but does not delete its memory
  bool Erase(const Index& i,void* data);
  ObjectSet* GetObjectSet(const Index& i);
  const ObjectSet* GetObjectSet(const Index& i) const;
  void Clear();
  void Enumerate(std::vector<void*>& items) const;

  //returns the index of the point
  void PointToIndex(const Vector& p,Index& i) const;
  //same, but with the local coordinates in the bucket [0,1]^n
  void PointToIndex(const Vector& p,Index& i,Vector& u) const;
  //returns the lower/upper corner of the bucket
  void IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const;

  //returns the min/max indices of all occupied cells
  void GetRange(Index& imin,Index& imax) const;
  //returns the min/max bound of all occupied cells
  void GetRange(Vector& bmin,Vector& bmax) const;

  //range imin to imax
  bool IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const;
  //bounding box from bmin to bmax
  bool BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const;
  //ball with center c, radius r
  bool BallQuery(const Vector& c,Real r,QueryCallback f) const;
  //segment from a to b
  bool SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;

  //range imin to imax
  void IndexItems(const Index& imin,const Index& imax,ObjectSet& objs) const;
  //bounding box from bmin to bmax
  void BoxItems(const Vector& bmin,const Vector& bmax,ObjectSet& objs) const;
  //ball with center c, radius r
  void BallItems(const Vector& c,Real r,ObjectSet& objs) const;
  //segment from a to b
  void SegmentItems(const Vector& a,const Vector& b,ObjectSet& objs) const;

  Vector hinv;
  typedef UNORDERED_MAP_TEMPLATE<Index,ObjectSet,IndexHash> HashTable;
  HashTable buckets;
};


/** @ingroup Geometry
 * @brief A container class that makes it easier to work with hashes. Each object
 * is allocated with Insert() and deleted on destruction / Clear / Erase. 
 * 
 * Items must be copy-constructable and assignable;
 */
template <class Data>
class GridHash3DContainer : public GridHash3D
{
 public:
 typedef GridHash3DContainer<Data> MyT;
  ///called once per value in the query range, return false to stop enumerating
  typedef bool (*DataQueryCallback)(Data* value);

  explicit GridHash3DContainer(Real h=1): GridHash3D(h) {}
  explicit GridHash3DContainer(const Vector3& h): GridHash3D(h) {}
  explicit GridHash3DContainer(const MyT& rhs) { operator=(rhs); }
  ~GridHash3DContainer() { Clear(); }
  const MyT& operator = (const MyT& rhs);
  ///Inserts a new item, if it doesnt exist, and returns it
  Data* Insert(const Index& i,bool* created=NULL);
  ///Note: this container takes ownership of data.
  void Set(const Index& i,Data* data);
  Data* Get(const Index& i) { return Cast(GridHash3D::Get(i)); }
  const Data* Get(const Index& i) const { return Cast(GridHash3D::Get(i)); }
  bool Erase(const Index& i);
  void Clear();
  void Enumerate(std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::Enumerate(ptrs); Cast(ptrs,items); }
  void IndexItems(const Index& imin,const Index& imax,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::IndexItems(imin,imax,ptrs); Cast(ptrs,items); }
  void BoxItems(const Vector3& bmin,const Vector3& bmax,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::BoxItems(bmin,bmax,ptrs); Cast(ptrs,items); }
  void BallItems(const Vector3& c,Real r,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::BallItems(c,r,ptrs); Cast(ptrs,items); }
  void SegmentItems(const Vector3& a,const Vector3& b,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::SegmentItems(a,b,ptrs); Cast(ptrs,items); }

  inline Data* Cast(void* ptr) const { return reinterpret_cast<Data*>(ptr); }
  inline void Cast(const std::vector<void*>& ptrs,std::vector<Data*>& items) const {
    items.resize(ptrs.size());
    for(size_t i=0;i<ptrs.size();i++) items[i] = Cast(ptrs[i]);
  }

  class iterator {
    public:
      GridHash3DContainer<Data>& container;
      GridHash3D::HashTable::iterator it;

      iterator(GridHash3DContainer<Data>& _container,GridHash3D::HashTable::iterator _it)
      :container(_container),it(_it)
      {}
      iterator(const iterator& rhs)=default;
      bool operator==(const iterator& rhs) const { return it==rhs.it; }
      bool operator!=(const iterator& rhs) const { return it!=rhs.it; }
      iterator& operator ++() { ++it; return *this; }
      iterator operator ++(int) { iterator res(container,it); ++it; return res; }
      std::pair<IntTriple,Data*> operator *() const { return std::make_pair(it->first,container.Cast(it->second)); }
      std::pair<IntTriple,Data*> operator ->() const { return std::make_pair(it->first,container.Cast(it->second)); }
  };

  class const_iterator {
    public:
      const GridHash3DContainer<Data>& container;
      GridHash3D::HashTable::const_iterator it;

      const_iterator(const GridHash3DContainer<Data>& _container,GridHash3D::HashTable::const_iterator _it)
      :container(_container),it(_it)
      {}
      const_iterator(const const_iterator& rhs)=default;
      const_iterator(const iterator& rhs)
      :container(rhs.container),it(rhs.it)
      {}
      bool operator==(const const_iterator& rhs) const { return it==rhs.it; }
      bool operator!=(const const_iterator& rhs) const { return it!=rhs.it; }
      const_iterator& operator ++() { ++it; return *this; }
      const_iterator operator ++(int) { const_iterator res(container,it); ++it; return res; }
      std::pair<IntTriple,Data*> operator *() const { return std::make_pair(it->first,container.Cast(it->second)); }
      std::pair<IntTriple,Data*> operator ->() const { return std::make_pair(it->first,container.Cast(it->second)); }
  };

  iterator begin() { return iterator(*this,buckets.begin()); }
  iterator end() { return iterator(*this,buckets.end()); }
  const_iterator begin() const { return const_iterator(*this,buckets.begin()); }
  const_iterator end() const { return const_iterator(*this,buckets.end()); }
};


/** @ingroup Geometry
 * @brief A container class that makes it easier to work with subdivisions. Each object
 * is allocated with Insert() and deleted on destruction / Clear / Erase.
 */
template <class Data>
class GridSubdivision3DContainer : public GridSubdivision3D
{
 public:
  typedef GridSubdivision3DContainer<Data> MyT;
  typedef std::vector<Data*> DataSet;
  //called once per object in the query range, return false to stop enumerating
  typedef bool (*DataQueryCallback)(Data* obj);

  explicit GridSubdivision3DContainer(Real h=1) :GridSubdivision3D(h) {}
  explicit GridSubdivision3DContainer(const Vector3& h) :GridSubdivision3D(h) {}
  explicit GridSubdivision3DContainer(const MyT& rhs) { operator=(rhs); }
  ~GridSubdivision3DContainer() { Clear(); }
  const MyT& operator = (const MyT& rhs);
  ///Inserts a new item.  Takes ownership of the object. 
  ///Important: data cannot be added to two different indices!
  void Insert(const Index& i,Data* data) { GridSubdivision3D::Insert(i,data); }
  ///This erases AND deletes an item. (After calling this, data should no longer be used)
  bool Erase(const Index& i,Data* data);
  void GetObjectSet(const Index& i,std::vector<Data*>& objects) const;
  void Clear();
  void Enumerate(std::vector<Data*>& items) const { std::vector<void*> ptrs; GridSubdivision3D::Enumerate(ptrs); Cast(ptrs,items); }
  void IndexItems(const Index& imin,const Index& imax,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridSubdivision3D::IndexItems(imin,imax,ptrs); Cast(ptrs,items); }
  void BoxItems(const Vector3& bmin,const Vector3& bmax,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridSubdivision3D::BoxItems(bmin,bmax,ptrs); Cast(ptrs,items); }
  void BallItems(const Vector3& c,Real r,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridSubdivision3D::BallItems(c,r,ptrs); Cast(ptrs,items); }
  void SegmentItems(const Vector3& a,const Vector3& b,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridSubdivision3D::SegmentItems(a,b,ptrs); Cast(ptrs,items); }


  inline Data* Cast(void* ptr) const { return reinterpret_cast<Data*>(ptr); }
  inline void Cast(const std::vector<void*>& ptrs,std::vector<Data*>& items) const {
    items.resize(ptrs.size());
    for(size_t i=0;i<ptrs.size();i++) items[i] = Cast(ptrs[i]);
  }
};

/** @ingroup Geometry
 * @brief A container class that makes it easier to work with hashes, similar
 * to GridHash3DContainer except data allocation is performed in bulk.
 * 
 * Items must be copy-constructable and assignable;
 * 
 * Note: the void* pointers in the GridHash3D are actually integer indices 
 * into the values array.
 */
template <class Data>
class GridHash3DContiguousContainer : public GridHash3D
{
 public:
  typedef GridHash3DContiguousContainer<Data> MyT;
  ///called once per value in the query range, return false to stop enumerating
  typedef bool (*DataQueryCallback)(Data* value);

  explicit GridHash3DContiguousContainer(Real h=1): GridHash3D(h) {}
  explicit GridHash3DContiguousContainer(const Vector3& h): GridHash3D(h) {}
  explicit GridHash3DContiguousContainer(const MyT& rhs) { operator=(rhs); }
  ~GridHash3DContiguousContainer() {}
  const MyT& operator = (const MyT& rhs);
  ///Inserts a new item, if it doesnt exist, and returns it.
  ///Note: should not keep these around, since the vector can be reallocated and
  ///kill previously allocated pointers.
  Data* Insert(const Index& i,bool* created=NULL);
  ///Should not be used.
  void Set(const Index& i,Data* data) { FatalError("Set is not implemented"); }
  Data* Get(const Index& i);
  const Data* Get(const Index& i) const;
  bool Erase(const Index& i);
  void Clear();
  void Enumerate(std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::Enumerate(ptrs); Cast(ptrs,items); }
  void IndexItems(const Index& imin,const Index& imax,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::IndexItems(imin,imax,ptrs); Cast(ptrs,items); }
  void BoxItems(const Vector3& bmin,const Vector3& bmax,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::BoxItems(bmin,bmax,ptrs); Cast(ptrs,items); }
  void BallItems(const Vector3& c,Real r,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::BallItems(c,r,ptrs); Cast(ptrs,items); }
  void SegmentItems(const Vector3& a,const Vector3& b,std::vector<Data*>& items) const { std::vector<void*> ptrs; GridHash3D::SegmentItems(a,b,ptrs); Cast(ptrs,items); }

  inline const Data* Cast(void* ptr) const {
    int index = static_cast<int>(reinterpret_cast<intptr_t>(ptr));
    Assert(index >= 0 && index < (int)values.size());
    return &values[index];
  }
  inline Data* Cast(void* ptr) {
    int index = static_cast<int>(reinterpret_cast<intptr_t>(ptr));
    Assert(index >= 0 && index < (int)values.size());
    return &values[index];
  }
  inline void Cast(const std::vector<void*>& ptrs,std::vector<Data*>& items) const {
    items.resize(ptrs.size());
    for(size_t i=0;i<ptrs.size();i++) items[i] = Cast(ptrs[i]);
  }
  
  class iterator {
    public:
      GridHash3DContiguousContainer<Data>& container;
      GridHash3D::HashTable::iterator it;

      iterator(GridHash3DContiguousContainer<Data>& _container,GridHash3D::HashTable::iterator _it)
      :container(_container),it(_it)
      {}
      iterator(const iterator& rhs)=default;
      bool operator==(const iterator& rhs) const { return it==rhs.it; }
      bool operator!=(const iterator& rhs) const { return it!=rhs.it; }
      iterator& operator ++() { ++it; return *this; }
      iterator operator ++(int) { iterator res(container,it); ++it; return res; }
      std::pair<IntTriple,Data*> operator *() const { return std::make_pair(it->first,container.Cast(it->second)); }
      std::pair<IntTriple,Data*> operator ->() const { return std::make_pair(it->first,container.Cast(it->second)); }
  };

  class const_iterator {
    public:
      const GridHash3DContiguousContainer<Data>& container;
      GridHash3D::HashTable::const_iterator it;

      const_iterator(const GridHash3DContiguousContainer<Data>& _container,GridHash3D::HashTable::const_iterator _it)
      :container(_container),it(_it)
      {}
      const_iterator(const const_iterator& rhs)=default;
      const_iterator(const iterator& rhs)
      :container(rhs.container),it(rhs.it)
      {}
      bool operator==(const const_iterator& rhs) const { return it==rhs.it; }
      bool operator!=(const const_iterator& rhs) const { return it!=rhs.it; }
      const_iterator& operator ++() { ++it; return *this; }
      const_iterator operator ++(int) { const_iterator res(container,it); ++it; return res; }
      std::pair<IntTriple,const Data*> operator *() const { return std::make_pair(it->first,container.Cast(it->second)); }
      std::pair<IntTriple,const Data*> operator ->() const { return std::make_pair(it->first,container.Cast(it->second)); }
  };

  iterator begin() { return iterator(*this,buckets.begin()); }
  iterator end() { return iterator(*this,buckets.end()); }
  const_iterator begin() const { return const_iterator(*this,buckets.begin()); }
  const_iterator end() const { return const_iterator(*this,buckets.end()); }

  std::vector<Data> values;
  std::list<size_t> freeIndices;
};


template <class Data>
const GridHash3DContainer<Data>& GridHash3DContainer<Data>::operator = (const GridHash3DContainer<Data>& rhs)
{
  hinv = rhs.hinv;
  Clear();
  for(const auto& it:rhs.buckets) {
    GridHash3D::Set(it.first,new Data(*Cast(it.second)));
  }
  return *this;
}

template <class Data>
Data* GridHash3DContainer<Data>::Insert(const Index& i,bool* created)
{
  void* ptr = GridHash3D::Get(i);
  if(ptr) {
    if(created) *created=false;
    return Cast(ptr);
  }
  if(created) *created=true;
  Data* res=new Data();
  GridHash3D::Set(i,res);
  return res;
}

template <class Data>
void GridHash3DContainer<Data>::Set(const Index& i,Data* data)
{
  void* ptr = GridHash3D::Get(i);
  if(ptr) {
    Data* olddata = Cast(ptr);
    if(olddata == data) return;
    delete olddata;
  }
  GridHash3D::Set(i,data);
}

template <class Data>
bool GridHash3DContainer<Data>::Erase(const Index& i)
{
  void* res = GridHash3D::Erase(i);
  if(res) {
    delete Cast(res);
    return true;
  }
  return false;
}

template <class Data>
void GridHash3DContainer<Data>::Clear()
{
  for(auto& i:buckets) 
    delete Cast(i.second);
  GridHash3D::Clear();
}



template <class Data>
const GridSubdivision3DContainer<Data>& GridSubdivision3DContainer<Data>::operator = (const GridSubdivision3DContainer<Data>& rhs)
{
  hinv = rhs.hinv;
  Clear();
  for(const auto& it:rhs.buckets) {
    for(auto j:it.second)
      GridSubdivision3D::Insert(it.first,new Data(*Cast(j)));
  }
  return *this;
}

template <class Data>
bool GridSubdivision3DContainer<Data>::Erase(const Index& i,Data* data)
{
  if(GridSubdivision3D::Erase(i,data)) {
    delete data;
    return true;
  }
  return false;
}

template <class Data>
void GridSubdivision3DContainer<Data>::GetObjectSet(const Index& i,std::vector<Data*>& objects) const
{
  const ObjectSet* ptrs=GridSubdivision3D::GetObjectSet(i);
  objects.resize(0);
  if(!ptrs) return;
  Cast(*ptrs,objects);
}


template <class Data>
void GridSubdivision3DContainer<Data>::Clear()
{
  for(auto& i:buckets) {
    for(auto j:i.second) {
      delete reinterpret_cast<Data*>(j);
    }
  }
  GridSubdivision3D::Clear();
}


template <class Data>
const GridHash3DContiguousContainer<Data>& GridHash3DContiguousContainer<Data>::operator =(const MyT& rhs)
{
  hinv = rhs.hinv;
  buckets = rhs.buckets;
  values = rhs.values;
  freeIndices = rhs.freeIndices;
  return *this;
}

template <class Data>
Data* GridHash3DContiguousContainer<Data>::Insert(const Index& i,bool* created)
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    if(created) *created=false;
    void* ptr = bucket->second;
    size_t index = static_cast<size_t>(reinterpret_cast<intptr_t>(ptr));
    Assert(index >= 0 && index < values.size());
    return &values[index];
  }
  else {
    if(created) *created=true;
    size_t index;
    if(freeIndices.empty()) {
        index = values.size();
        values.resize(values.size()+1);
    }
    else {
        index = freeIndices.front();
        freeIndices.pop_front();
    }

    GridHash3D::Set(i,(void*)index);
    return &values[index];
  }
}

template <class Data>
Data* GridHash3DContiguousContainer<Data>::Get(const Index& i)
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket == buckets.end()) return NULL;
  else return Cast(bucket->second);
}

template <class Data>
const Data* GridHash3DContiguousContainer<Data>::Get(const Index& i) const
{
  HashTable::const_iterator bucket = buckets.find(i);
  if(bucket == buckets.end()) return NULL;
  else return Cast(bucket->second);
}

template <class Data>
bool GridHash3DContiguousContainer<Data>::Erase(const Index& i)
{
  HashTable::iterator bucket = buckets.find(i);
  if(bucket != buckets.end()) {
    void* ptr=bucket->second;
    size_t index = static_cast<size_t>(reinterpret_cast<intptr_t>(ptr));
    freeIndices.push_back(index);
    return true;
  }
  return false;
}

template <class Data>
void GridHash3DContiguousContainer<Data>::Clear()
{
  GridHash3D::Clear();
  values.clear();
  freeIndices.clear();
}

} //namespace Geometry

#endif
