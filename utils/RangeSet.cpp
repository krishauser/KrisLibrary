#include "RangeSet.h"
#include <errors.h>

RangeSet::RangeSet()
  :imin(1),imax(0),hasContainmentCache(false)
{}

void RangeSet::clear()
{
  imin=1;
  imax=0;
  hasContainmentCache=false;
  items.clear();
  contains.clear();
}

void RangeSet::setRange(int _imin,int _imax)
{
  if(!isRangeEmpty() && !items.empty()) //already has a range
    Assert(_imin <= imin && _imax >= imax);
  if(hasContainmentCache) {
    FatalError("Already have cache set up");
  }
  imin=_imin;
  imax=_imax;
}

void RangeSet::expandRange(int item)
{
  if(isRangeEmpty()) { //has no range
    imin = imax = item;
    return;
  }
  if(hasContainmentCache) {
    if(item < imin) {
      FatalError("RangeSet::expandRange(): error, item lower than the range bound");
    }
    if(item > imax) {
      //issue a warning, perhaps?
      contains.resize(item-imin+1,false);
      imax = item;
    }
  }
  else {
    if(item < imin) imin=item;
    else if(item > imax) imax=item;
  }
}

void RangeSet::insert(int item)
{
  if(hasContainmentCache) {
    if(item < imin) {
      FatalError("RangeSet::insert(): error, item lower than the range bound");
    }
    if(item > imax) {
      //issue a warning, perhaps?
      contains.resize(item-imin+1,false);
      imax = item;
    }
    if(!cacheGet(item)) {
      items.insert(item);
      cacheSet(item,true);
    }
  }
  else {
    items.insert(item);
    if(item < imin) imin=item;
    if(item > imax) imax=item;
  }
}

void RangeSet::erase(int item)
{
  if(!inRange(item)) return;
  if(hasContainmentCache) {
    if(cacheGet(item)) {
      items.erase(items.find(item));
      cacheSet(item,false);
    }
  }
  else {
    items.erase(items.find(item));
  }
}

void RangeSet::erase(iterator it)
{
  if(it == end()) return;
  if(hasContainmentCache) cacheSet(*it,false);
  items.erase(it);
}

int RangeSet::count(int item)
{
  if(!inRange(item)) return 0;
  if(hasContainmentCache) {
    if(cacheGet(item)) return 1;
    return 0;
  }
  return items.count(item);
}

RangeSet::iterator RangeSet::find(int item)
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    if(!cacheGet(item)) return items.end();
  return items.find(item);
}

RangeSet::const_iterator RangeSet::find(int item) const
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    if(!cacheGet(item)) return items.end();
  return items.find(item);
}

void RangeSet::BuildCache()
{
  if(imax < imin) return;  //no items
  if(hasContainmentCache) return;
  Assert(imax >= imin);
  contains.resize(imax-imin+1);
  fill(contains.begin(),contains.end(),false);
  for(set<int>::const_iterator i=items.begin();i!=items.end();i++)
    cacheSet(*i,true);
  hasContainmentCache = true;
}

void RangeSet::ClearCache()
{
  hasContainmentCache=false;
  contains.clear();
}

