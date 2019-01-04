#ifndef RANGE_MAP_H
#define RANGE_MAP_H

#include <map>
#include <vector>
#include <KrisLibrary/errors.h>
using namespace std;

/** @ingroup Utils
 * @brief Same as RangeSet, but a map rather than a set.  O(1) find as well.
 */
template<class T>
class RangeMap
{
 public:
  typedef map<int,T>::iterator iterator;
  typedef map<int,T>::const_iterator const_iterator;

  RangeMap();
  void clear();
  inline bool empty() { return items.empty(); }
  void setRange(int imin,int imax);
  void expandRange(int item);
  void insert(int item,const T& value);
  void erase(iterator it);
  void erase(int item);
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
  inline iterator cacheGet(int item) const { return contains[item-imin]; }
  inline void cacheSet(int item,iterator it) { contains[item-imin]=it; }

  void BuildCache();
  void ClearCache();
  inline bool IsCacheBuilt() const { return hasContainmentCache; }

 private:
  map<int,T> items;
  int imin,imax;
  bool hasContainmentCache;
  vector<iterator> contains;
};

template <class T>
RangeMap<T>::RangeMap()
  :imin(1),imax(0),hasContainmentCache(false)
{}

template <class T>
void RangeMap<T>::clear()
{
  imin=1;
  imax=0;
  hasContainmentCache=false;
  items.clear();
  contains.clear();
}

template <class T>
void RangeMap<T>::setRange(int _imin,int _imax)
{
  if(!isRangeEmpty() && !items.empty()) //already has a range
    Assert(_imin <= imin && _imax >= imax);
  if(hasContainmentCache) {
    FatalError("Already have cache set up");
  }
  imin=_imin;
  imax=_imax;
}

template <class T>
void RangeMap<T>::expandRange(int item)
{
  if(isRangeEmpty()) { //has no range
    imin = imax = item;
    return;
  }
  if(hasContainmentCache) {
    if(item < imin) {
      FatalError("RangeMap<T>::expandRange(): error, item lower than the range bound");
    }
    if(item > imax) {
      //issue a warning, perhaps?
      contains.resize(item-imin+1,end());
      imax = item;
    }
  }
  else {
    if(item < imin) imin=item;
    else if(item > imax) imax=item;
  }
}

template <class T>
void RangeMap<T>::insert(int item)
{
  if(hasContainmentCache) {
    if(item < imin) {
      FatalError("RangeMap<T>::insert(): error, item lower than the range bound");
    }
    if(item > imax) {
      //issue a warning, perhaps?
      contains.resize(item-imin+1,false);
      imax = item;
    }
    iterator it=cacheGet(item);
    if(it == end()) {
      iterator it=items.insert(item);
      cacheSet(item,it);
    }
  }
  else {
    items.insert(item);
    if(item < imin) imin=item;
    if(item > imax) imax=item;
  }
}

template <class T>
void RangeMap<T>::erase(int item)
{
  if(!inRange(item)) return;
  if(hasContainmentCache) {
    iterator it=cacheGet(item);
    if(it != end()) {
      items.erase(it);
      cacheSet(item,end());
    }
  }
  else {
    items.erase(items.find(item));
  }
}

template <class T>
void RangeMap<T>::erase(iterator it)
{
  if(it == end()) return;
  if(hasContainmentCache) cacheSet(*it,end());
  items.erase(it);
}

template <class T>
int RangeMap<T>::count(int item)
{
  if(!inRange(item)) return 0;
  if(hasContainmentCache) {
    if(cacheGet(item) != end()) return 1;
    return 0;
  }
  return items.count(item);
}

template <class T>
RangeMap<T>::iterator RangeMap<T>::find(int item)
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    return cacheGet(item);
  else
    return items.find(item);
}

template <class T>
RangeMap<T>::const_iterator RangeMap<T>::find(int item) const
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    return cacheGet(item);
  else 
    return items.find(item);
}

template <class T>
void RangeMap<T>::BuildCache()
{
  if(imax < imin) return;  //no items
  if(hasContainmentCache) return;
  Assert(imax >= imin);
  contains.resize(imax-imin+1);
  fill(contains.begin(),contains.end(),items.end());
  for(iterator i=items.begin();i!=items.end();i++)
    cacheSet(*i,i);
  hasContainmentCache = true;
}

template <class T>
void RangeMap<T>::ClearCache()
{
  hasContainmentCache=false;
  contains.clear();
}

#endif
