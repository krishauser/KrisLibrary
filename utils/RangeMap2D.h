#ifndef RANGE_MAP_2D_H
#define RANGE_MAP_2D_H

#include "IntTuple.h"
#include <KrisLibrary/structs/array2d.h>
#include <map>
using namespace std;

/** @ingroup Utils
 * @brief Same as RangeSet2D but a map.  O(1) find as well as count.
 */
template <class T>
class RangeMap2D
{
 public:
  typedef map<IntPair>::iterator iterator;
  typedef map<IntPair>::const_iterator const_iterator;

  RangeMap2D();
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
  inline iterator cacheGet(const IntPair& item) const { return contains(item.a-imin.a,item.b-imin.b); }
  inline void cacheSet(const IntPair& item,iterator it) { contains(item.a-imin.a,item.b-imin.b)=it; }

  void BuildCache();
  void ClearCache();
  inline bool IsCacheBuilt() const { return hasContainmentCache; }

 private:
  map<IntPair,T> items;
  IntPair imin,imax;
  bool hasContainmentCache;
  Array2D<iterator> contains;
};

RangeMap2D::RangeMap2D()
  :imin(1,1),imax(0,0),hasContainmentCache(false)
{}

void RangeMap2D::clear()
{
  imin.set(1,1);
  imax.set(0,0);
  hasContainmentCache=false;
  items.clear();
  contains.clear();
}

void RangeMap2D::setRange(const IntPair& _imin,const IntPair& _imax)
{
  if(imin < imax) //already has a range
    Assert(_imin.a <= imin.a && _imin.b <= imin.b &&
	   _imax.a >= imax.a && _imax.b >= imax.b);
  if(hasContainmentCache) {
    FatalError("Already have cache set up");
  }
  imin=_imin;
  imax=_imax;
}

void RangeMap2D::expandRange(const IntPair& item)
{
  if(isRangeEmpty()) { //has no range
    imin = imax = item;
    return;
  }
  if(hasContainmentCache) {
    if(!inRange(item))
      FatalError("RangeMap2D::expandRange(): error, item out of bounds");
  }
  else {
    if(item.a < imin.a) imin.a=item.a;
    else if(item.a > imax.a) imax.a=item.a;
    if(item.b < imin.b) imin.b=item.b;
    else if(item.b > imax.b) imax.b=item.b;
  }
}

void RangeMap2D::insert(const IntPair& item)
{
  if(hasContainmentCache) {
    if(!inRange(item)) 
      FatalError("RangeMap2D::insert(): error, item out of bounds");
    iterator it=cacheGet(item);
    if(it == end()) {
      iterator it=items.insert(item);
      cacheSet(item,it);
    }
  }
  else {
    items.insert(item);
    if(item.a < imin.a) imin.a=item.a;
    else if(item.a > imax.a) imax.a=item.a;
    if(item.b < imin.b) imin.b=item.b;
    else if(item.b > imax.b) imax.b=item.b;
  }
}

void RangeMap2D::erase(const IntPair& item)
{
  if(!inRange(item)) return;
  if(hasContainmentCache) {
    iterator it=cacheGet(item);
    if(it != end()) {
      items.erase(it);
      cacheSet(item,items.end());
    }
  }
  else {
    items.erase(items.find(item));
  }
}

int RangeMap2D::count(const IntPair& item)
{
  if(!inRange(item)) return 0;
  if(hasContainmentCache) {
    if(cacheGet(item)!=items.end()) return 1;
    return 0;
  }
  return items.count(item);
}

RangeMap2D::iterator RangeMap2D::find(const IntPair& item)
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    return cacheGet(item);
  else
    return items.find(item);
}

RangeMap2D::const_iterator RangeMap2D::find(const IntPair& item) const
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    return cacheGet(item);
  else
    return items.find(item);
}

void RangeMap2D::BuildCache()
{
  if(imax < imin) return;  //no items
  if(hasContainmentCache) return;
  contains.resize(imax.a-imin.a+1,imax.b-imin.b+1);
  contains.set(items.end());
  for(const_iterator i=items.begin();i!=items.end();i++)
    cacheSet(*i,i);
  hasContainmentCache = true;
}

void RangeMap2D::ClearCache()
{
  hasContainmentCache=false;
  contains.clear();
}


#endif
