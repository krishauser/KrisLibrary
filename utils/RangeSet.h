#ifndef RANGE_SET_H
#define RANGE_SET_H

#include <set>
#include <vector>
using namespace std;

/** @ingroup Utils
 * @brief A set of integers within a range.  Operates in two modes,
 * set or bit vector mode.  In bit-vector mode, allows O(1) membership
 * testing (but the range is fixed).  Set mode is just like a regular set.
 *
 * If you need to do n membership tests, the best mode to use can be given
 * by choosing the minimum cost:
 * c(set mode) = n*log(n)*c(set query constant)
 * c(vector mode) = n*c(vector query time) + c(malloc time for max-min+1 bits)
 */
class RangeSet
{
 public:
  typedef set<int>::iterator iterator;
  typedef set<int>::const_iterator const_iterator;

  RangeSet();
  void clear();
  inline bool empty() { return items.empty(); }
  void setRange(int imin,int imax);
  void expandRange(int item);
  void insert(int item);
  template <class It>
  void insert(It first,It last);
  void erase(iterator it);
  void erase(int item);
  template <class It>
  void erase(It first,It last);
  inline iterator begin() { return items.begin(); }
  inline const_iterator begin() const { return items.begin(); }
  inline iterator end() { return items.end(); }
  inline const_iterator end() const { return items.end(); }
  size_t size() const { return items.size(); }
  int count(int item);
  iterator find(int item);
  const_iterator find(int item) const;
  inline bool isRangeEmpty() const { return imax < imin; }
  inline int minimum() const { return imin; }
  inline int maximum() const { return imax; }
  inline bool inRange(int item) const { return item >= imin && item <= imax; }
  inline bool cacheGet(int item) const { return contains[item-imin]; }
  inline void cacheSet(int item,bool value) { contains[item-imin]=value; }

  void BuildCache();
  void ClearCache();
  inline bool IsCacheBuilt() const { return hasContainmentCache; }

 private:
  set<int> items;
  int imin,imax;
  bool hasContainmentCache;
  vector<bool> contains;
};

template <class It>
void RangeSet::insert(It first,It last)
{
  for(It i=first;i!=last;i++)
    insert(*i);
}

template <class It>
void RangeSet::erase(It first,It last)
{
  for(It i=first;i!=last;i++)
    erase(*i);
}

#endif
