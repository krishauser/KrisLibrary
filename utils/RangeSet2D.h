#ifndef RANGE_SET_2D_H
#define RANGE_SET_2D_H

#include "IntTuple.h"
#include <KrisLibrary/structs/array2d.h>
#include <set>
using namespace std;

/** @ingroup Utils
 * @brief A set of 2d indices within a range.  Operates in two modes,
 * set or bit matrix mode.  In bit-matrix mode, allows O(1) membership
 * testing (but the range is fixed).  Set mode is just like a regular set.
 *
 * If you need to do n membership tests, the best mode to use can be given
 * by choosing the minimum cost:
 * c(set mode) = n*log(n)*c(set query constant)
 * c(vector mode) = n*c(vector query time) + c(malloc time for SIZE bits)
 */
class RangeSet2D
{
 public:
  typedef set<IntPair>::iterator iterator;
  typedef set<IntPair>::const_iterator const_iterator;

  RangeSet2D();
  void clear();
  inline bool empty() { return items.empty(); }
  void setRange(const IntPair& indexmin,const IntPair& indexmax);
  void expandRange(const IntPair& index);
  void insert(const IntPair& index);
  template <class It>
  void insert(It first,It last);
  void erase(iterator it);
  void erase(const IntPair& item);
  template <class It>
  void erase(It first,It last);
  inline iterator begin() { return items.begin(); }
  inline const_iterator begin() const { return items.begin(); }
  inline iterator end() { return items.end(); }
  inline const_iterator end() const { return items.end(); }
  size_t size() const { return items.size(); }
  int count(const IntPair& item);
  iterator find(const IntPair& item);
  const_iterator find(const IntPair& item) const;
  inline bool isRangeEmpty() const { return imax.a < imin.a; }
  inline const IntPair& minimum() const { return imin; }
  inline const IntPair& maximum() const { return imax; }
  inline bool inRange(const IntPair& item) const { return item.a >= imin.a && item.a <= imax.a && item.b >= imin.b && item.b  <= imax.b; }
  inline bool cacheGet(const IntPair& item) const { return contains(item.a-imin.a,item.b-imin.b); }
  inline void cacheSet(const IntPair& item,bool value) { contains(item.a-imin.a,item.b-imin.b)=value; }

  void BuildCache();
  void ClearCache();
  inline bool IsCacheBuilt() const { return hasContainmentCache; }

 private:
  set<IntPair> items;
  IntPair imin,imax;
  bool hasContainmentCache;
  Array2D<bool> contains;
};

template <class It>
void RangeSet2D::insert(It first,It last)
{
  for(It i=first;i!=last;i++)
    insert(*i);
}

template <class It>
void RangeSet2D::erase(It first,It last)
{
  for(It i=first;i!=last;i++)
    erase(*i);
}

#endif
