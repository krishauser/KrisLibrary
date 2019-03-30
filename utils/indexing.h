#ifndef INDEXING_H
#define INDEXING_H

#include <vector>
#include <list>
#include <set>
#include "IntPair.h"
#include "IntTriple.h"
#include "IntTuple.h"

/// Increments the tuple i in the range imin to imax (inclusive). Returns the
/// carry bit (0 normally, 1 if i exits the range)
int IncrementIndex(std::vector<int>& i,const std::vector<int>& imin,const std::vector<int>& imax);
/// Increments the tuple i in the range 0 to imax (exclusive). Returns the
/// carry bit (0 normally, 1 if i exits the range)
int IncrementIndex(std::vector<int>& i,const std::vector<int>& imax);

///An iterator class that is passed through a mapping whenever the * operator
///is called.
template <class T,class IterT,class MapT>
struct MapIterator : public IterT
{
  typedef MapIterator<T,IterT,MapT> MyT;

  explicit MapIterator(MapT& _mapping):mapping(_mapping) {}
  explicit MapIterator(IterT& _iter,MapT& _mapping):IterT(_iter),mapping(_mapping) {}
  inline T operator*() { return mapping(IterT::operator *()); }
  inline bool operator == (const MyT& rhs) const { return &mapping==&rhs.mapping && IterT::operator==(rhs); }
  inline bool operator != (const MyT& rhs) const { return !operator==(rhs); }
  inline bool operator < (const MyT& rhs) const {
    if(&mapping!=&rhs.mapping) return true;
    return IterT::operator < (rhs); 
  }

  MapT& mapping;
};

/** @brief A regular range of indices ik = start+k*stride, for k=0 to size-1.
 * 
 * Element k (domain) is given by the index start+k*stride (range).
 * Indices be enumerated into a vector, or with an iterator interface similar
 * to STL. With the iterator class, the index can be retrieved efficiently
 * with the * operator.  The element can be randomly accessed.
 */
struct RangeIndices
{
  RangeIndices();
  RangeIndices(const RangeIndices& rhs);
  explicit RangeIndices(int max);   //range from 0 to max-1
  explicit RangeIndices(int min,int max);  //range from min to max-1
  explicit RangeIndices(int min,int max,int stride);  //range from min to max-1, skipping stride
  void enumerate(std::vector<int>& indices);
  int operator [] (int i) const { return start+stride*i; }
  bool contains(int index) const;
  int indexToElement(int index) const;
  bool operator == (const RangeIndices& range) const;
  inline bool operator != (const RangeIndices& range) const { return !operator==(range); }

  struct iterator
  {
    iterator();
    explicit iterator(const RangeIndices* range);
    explicit iterator(const RangeIndices*, int invalid);
    iterator(const iterator& rhs);
    inline int operator*() const { return index; }
    inline int getElement() const { return i; }
    void setElement(int i);
    iterator& operator ++ ();
    iterator& operator -- ();
    iterator& operator += (int skip);
    iterator& operator -= (int skip);
    bool isInvalid() const;
    bool operator == (const iterator& rhs) const;
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    bool operator < (const iterator& rhs) const;
    
    //parameters
    const RangeIndices *range;
    //temp
    int i,index;
  };

  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(this,-1); }

  int start,size,stride;
};

/*
//Note: Use IntTuple instead of this
struct Indices : public std::vector<int>
{
  typedef std::vector<int> ParentT;

  inline Indices() {}
  inline Indices(const ParentT& rhs) :ParentT(rhs) {}
  inline Indices(int element) :ParentT(1,element) {}
  inline Indices(int e1,int e2) :ParentT(2)
  { operator[](0)=e1; operator[](1)=e2; }
  inline Indices(int e1,int e2,int e3) :ParentT(3)
  { operator[](0)=e1; operator[](1)=e2; operator[](2)=e3; }
  inline Indices(const std::list<int>& rhs):ParentT(rhs.size()) 
  { std::copy(rhs.begin(),rhs.end(),begin()); }
  inline Indices(const std::set<int>& rhs):ParentT(rhs.size()) 
  { std::copy(rhs.begin(),rhs.end(),begin()); }
  inline Indices(const RangeIndices& indices) { indices.enumerate(*this); }
};
*/

/** @brief A 2D lattice of regular ranges.
 * 
 * Can be accessed with an iterator class, which iterates over the second
 * coordinate first, then the first (lexicographically ordered).  Efficient
 * index computation.
 */
struct Range2Indices
{
  Range2Indices();
  Range2Indices(const Range2Indices& rhs);
  Range2Indices(const RangeIndices& irange,const RangeIndices& jrange);
  Range2Indices(int imax,int jmax);   //range [0,imax)x[0,jmax)
  Range2Indices(int imin,int imax,int jmin,int jmax);  //range [imin,imax)x[jmin,jmax)
  //range [imin,imax)x[jmin,jmax) with strides istride,jstride
  Range2Indices(int imin,int imax,int istride,int jmin,int jmax,int jstride);
  void enumerate(std::vector<IntPair>& indices);
  inline IntPair operator () (int i,int j) const { return IntPair(irange[i],jrange[j]); }
  inline IntPair operator () (const IntPair& t) const { return IntPair(irange[t.a],jrange[t.b]); }
  inline bool contains(int iindex,int jindex) const { return irange.contains(iindex) && jrange.contains(jindex); }
  inline bool contains(const IntPair& t) const { return contains(t.a,t.b); }
  inline IntPair indexToElement(int iindex,int jindex) const { return IntPair(irange.indexToElement(iindex),jrange.indexToElement(jindex)); }

  struct iterator 
  {
    iterator();
    iterator(const RangeIndices& irange,const RangeIndices& jrange);
    iterator(const RangeIndices& irange,const RangeIndices& jrange,int invalid);
    iterator(const iterator& rhs);
    inline IntPair operator*() { return IntPair(*i,*j); }
    inline int getElement() const { return element; }
    void setElement(int k);
    void setElement(int i,int j);
    void setElement(const IntPair& t) { setElement(t.a,t.b); }
    iterator& operator ++ ();
    iterator& operator -- ();
    iterator& operator += (int skip);
    iterator& operator -= (int skip);
    inline int getFirst() const { return *i; }
    inline int getSecond() const { return *j; }
    inline int getFirstElement() const { return i.getElement(); }
    inline int getSecondElement() const { return j.getElement(); }
    inline void incFirst(int skip=1) { i+=skip; }
    inline void incSecond(int skip=1) { j+=skip; }
    inline bool isInvalid() const { return i.isInvalid() || j.isInvalid(); }
    inline bool operator == (const iterator& rhs) const { return i==rhs.i && j==rhs.j; }
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    bool operator < (const iterator& rhs) const;
    
    //temp
    RangeIndices::iterator i,j;
    int element;   //the absolute index
  };

  iterator begin() const { return iterator(irange,jrange); }
  iterator end() const { return iterator(irange,jrange,-1); }

  RangeIndices irange,jrange;
};

/** @brief A 3D lattice of regular ranges.
 * 
 * Can be accessed with an iterator class, which iterates over the second
 * coordinate first, then the first (lexicographically ordered).  Efficient
 * index computation.
 */
struct Range3Indices
{
  Range3Indices();
  Range3Indices(const Range3Indices& rhs);
  Range3Indices(const RangeIndices& irange,const RangeIndices& jrange,const RangeIndices& krange);
  Range3Indices(int imax,int jmax,int kmax);   //range [0,imax)x[0,jmax)x[0,kmax)
  Range3Indices(int imin,int imax,int jmin,int jmax,int kmin,int kmax);  //range [imin,imax)x[jmin,jmax)x[kmin,kmax)
  void enumerate(std::vector<IntTriple>& indices);
  inline IntTriple operator () (int i,int j,int k) const
  { return IntTriple(irange[i],jrange[j],krange[k]); }
  inline IntTriple operator () (const IntTriple& t) const
  { return IntTriple(irange[t.a],jrange[t.b],krange[t.c]); }
  inline bool contains(int iindex,int jindex,int kindex) const { return irange.contains(iindex) && jrange.contains(jindex) && krange.contains(kindex); }
  inline bool contains(const IntTriple& t) const
  { return contains(t.a,t.b,t.c); }
  inline IntTriple indexToElement(int iindex,int jindex,int kindex) const {
    return IntTriple(irange.indexToElement(iindex),
		     jrange.indexToElement(jindex),
		     krange.indexToElement(kindex)); }

  struct iterator 
  {
    iterator();
    iterator(const RangeIndices& irange,const RangeIndices& jrange,const RangeIndices& krange);
    iterator(const RangeIndices& irange,const RangeIndices& jrange,const RangeIndices& krange,int invalid);
    iterator(const iterator& rhs);
    inline IntTriple operator*() { return IntTriple(*i,*j,*k); }
    inline int getElement() const { return element; }
    void setElement(int m);
    void setElement(int i,int j,int k);
    void setElement(const IntTriple& t) { setElement(t.a,t.b,t.c); }
    iterator& operator ++ ();
    iterator& operator -- ();
    iterator& operator += (int skip);
    iterator& operator -= (int skip);
    inline int getFirst() const { return *i; }
    inline int getSecond() const { return *j; }
    inline int getThird() const { return *k; }
    inline int getFirstElement() const { return i.getElement(); }
    inline int getSecondElement() const { return j.getElement(); }
    inline int getThirdElement() const { return k.getElement(); }
    inline void incFirst(int skip=1) { i+=skip; }
    inline void incSecond(int skip=1) { j+=skip; }
    inline void incThird(int skip=1) { k+=skip; }
    inline bool isInvalid() const { return i.isInvalid() || j.isInvalid() || k.isInvalid(); }
    inline bool operator == (const iterator& rhs) const { return i==rhs.i && j==rhs.j && k==rhs.k; }
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    bool operator < (const iterator& rhs) const;
    
    //temp
    RangeIndices::iterator i,j,k;
    int element;   //the absolute index
  };

  iterator begin() const { return iterator(irange,jrange,krange); }
  iterator end() const { return iterator(irange,jrange,krange,-1); }

  RangeIndices irange,jrange,krange;
};

/** @brief A 2D lattice indexing into a linear sequence, in a
 * striping pattern.
 * 
 * Typically used for 2D arrays laid out by row/column.
 * Iterator class gives fast iteration over the range.
 */
struct Stripe2Indices
{
  Stripe2Indices();
  Stripe2Indices(const Stripe2Indices& rhs);
  Stripe2Indices(int isize,int jsize,int base=0,int istride=1,int jstride=1);
  Stripe2Indices(const Range2Indices& rhs);  //assigns a row-major ordering
  Stripe2Indices(int isize,int jsize,const Range2Indices& subRange);  //assigns a row-major ordering
  Stripe2Indices(const RangeIndices& irange,const RangeIndices& jrange);
  void enumerate(std::vector<int>& indices);
  inline int operator () (int i,int j) const { return base+i*istride+j*jstride; }
  inline int operator () (const IntPair& t) const { return operator()(t.a,t.b); }
  bool contains(int index) const;
  IntPair indexToElement(int index) const;
  void setRange(const Stripe2Indices& stripe,const Range2Indices& inds);
  bool operator == (const Stripe2Indices& stripe) const;
  inline bool operator != (const Stripe2Indices& stripe) const { return !operator==(stripe); }

  struct iterator
  {
    explicit iterator(const Stripe2Indices*);
    explicit iterator(const Stripe2Indices*,int invalid);
    iterator(const iterator& i);
    inline int operator*() const { return index; }
    inline IntPair getElement() const { return IntPair(i,j); }
    iterator& operator ++();
    iterator& operator --();
    iterator& operator +=(int skip);
    iterator& operator -=(int skip);
    void incFirst(int skip=1);
    void incSecond(int skip=1);
    bool isInvalid() const;
    bool operator == (const iterator& rhs) const;
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    bool operator < (const iterator& rhs) const;

    const Stripe2Indices* stripe;
    int i,j;
    int index,stripeIndex;
  };

  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(this,-1); }

  int base;
  int isize,jsize;
  int istride,jstride;
};

/** @brief A 3D lattice indexing into a linear sequence, in a
 * striping pattern.
 * 
 * Typically used for 3D arrays laid out by face/row/column.
 * Iterator class gives fast iteration over the range.
 */
struct Stripe3Indices
{
  Stripe3Indices();
  Stripe3Indices(const Stripe3Indices& rhs);
  Stripe3Indices(int isize,int jsize,int ksize,int base=0,int istride=1,int jstride=1,int kstride=1);
  explicit Stripe3Indices(const Range3Indices& rhs);  //assigns a row-major ordering
  Stripe3Indices(int isize,int jsize,int ksize,const Range3Indices& subRange);  //assigns a row-major ordering
  Stripe3Indices(const RangeIndices& irange,const RangeIndices& jrange,const RangeIndices& krange);
  void enumerate(std::vector<int>& indices);
  inline int operator () (int i,int j,int k) const { return base+i*istride+j*jstride+k*kstride; }
  inline int operator () (const IntTriple& t) const { return operator()(t.a,t.b,t.c); }
  bool contains(int index) const;
  IntTriple indexToElement(int index) const;
  void setRange(const Stripe3Indices& stripe,const Range3Indices& inds);
  bool operator == (const Stripe3Indices& stripe) const;
  inline bool operator != (const Stripe3Indices& stripe) const { return !operator==(stripe); }

  struct iterator
  {
    explicit iterator(const Stripe3Indices*);
    explicit iterator(const Stripe3Indices*,int invalid);
    iterator(const iterator& i);
    inline int operator*() const { return index; }
    inline IntTriple getElement() const { return IntTriple(i,j,k); }
    iterator& operator ++();
    iterator& operator --();
    iterator& operator +=(int skip);
    iterator& operator -=(int skip);
    void incFirst(int skip=1);
    void incSecond(int skip=1);
    void incThird(int skip=1);
    bool isInvalid() const;
    bool operator == (const iterator& rhs) const;
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    bool operator < (const iterator& rhs) const;

    const Stripe3Indices* stripe;
    int i,j,k;
    int index,firstIndex,secondIndex;
  };

  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(this,-1); }

  int base;
  int isize,jsize,ksize;
  int istride,jstride,kstride;
};

/** @brief A 2D lattice of irregular ranges.
 * 
 * Can be accessed with an iterator class, which iterates over the second
 * coordinate first, then the first (lexicographically ordered).  Efficient
 * index computation.
 */
struct Grid2Indices
{
  void enumerate(std::vector<IntPair>& indices);
  inline IntPair operator () (int i,int j) const { return IntPair(iindices[i],jindices[j]); }
  inline IntPair operator () (const IntPair& t) const { return operator() (t.a,t.b); }
  inline bool contains(int iindex,int jindex) const { return iindices.contains(iindex) && jindices.contains(jindex); }

  struct iterator : public Range2Indices::iterator
  {
    typedef Range2Indices::iterator ParentT;
    iterator(const Grid2Indices* grid);
    iterator(const Grid2Indices* grid,int invalid);
    iterator(const Grid2Indices* grid,const Range2Indices& range);
    iterator(const Grid2Indices* grid,const Range2Indices& range,int invalid);
    inline IntPair operator*() {
      IntPair i=ParentT::operator *();
      return (*grid)(i);
    }
    inline int getFirst() const { return grid->iindices[ParentT::getFirst()]; }
    inline int getSecond() const { return grid->jindices[ParentT::getSecond()]; }
    inline bool operator == (const iterator& rhs) const { return grid==rhs.grid && ParentT::operator==(rhs); }
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    inline bool operator < (const iterator& rhs) const { return ParentT::operator < (rhs); }

    const Grid2Indices* grid;
    Range2Indices range; 
    Range2Indices::iterator e;
  };

  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(this,-1); }
  iterator beginRange(const Range2Indices& r) const { return iterator(this,r); }
  iterator endRange(const Range2Indices& r) const { return iterator(this,r,-1); }

  IntTuple iindices,jindices;
};

#endif
