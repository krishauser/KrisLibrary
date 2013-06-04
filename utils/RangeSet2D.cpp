#include "RangeSet2D.h"

RangeSet2D::RangeSet2D()
  :imin(1,1),imax(0,0),hasContainmentCache(false)
{}

void RangeSet2D::clear()
{
  imin.set(1,1);
  imax.set(0,0);
  hasContainmentCache=false;
  items.clear();
  contains.clear();
}

void RangeSet2D::setRange(const IntPair& _imin,const IntPair& _imax)
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

void RangeSet2D::expandRange(const IntPair& item)
{
  if(isRangeEmpty()) { //has no range
    imin = imax = item;
    return;
  }
  if(hasContainmentCache) {
    if(!inRange(item))
      FatalError("RangeSet2D::expandRange(): error, item out of bounds");
  }
  else {
    if(item.a < imin.a) imin.a=item.a;
    else if(item.a > imax.a) imax.a=item.a;
    if(item.b < imin.b) imin.b=item.b;
    else if(item.b > imax.b) imax.b=item.b;
  }
}

void RangeSet2D::insert(const IntPair& item)
{
  if(hasContainmentCache) {
    if(!inRange(item)) 
      FatalError("RangeSet2D::insert(): error, item out of bounds");
    if(!cacheGet(item)) {
      items.insert(item);
      cacheSet(item,true);
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

void RangeSet2D::erase(const IntPair& item)
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

int RangeSet2D::count(const IntPair& item)
{
  if(!inRange(item)) return 0;
  if(hasContainmentCache) {
    if(cacheGet(item)) return 1;
    return 0;
  }
  return items.count(item);
}

RangeSet2D::iterator RangeSet2D::find(const IntPair& item)
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    if(!cacheGet(item)) return items.end();
  return items.find(item);
}

RangeSet2D::const_iterator RangeSet2D::find(const IntPair& item) const
{
  if(!inRange(item)) return items.end();
  if(hasContainmentCache) 
    if(!cacheGet(item)) return items.end();
  return items.find(item);
}

void RangeSet2D::BuildCache()
{
  if(imax < imin) return;  //no items
  if(hasContainmentCache) return;
  contains.resize(imax.a-imin.a+1,imax.b-imin.b+1);
  contains.set(false);
  for(set<IntPair>::const_iterator i=items.begin();i!=items.end();i++)
    cacheSet(*i,true);
  hasContainmentCache = true;
}

void RangeSet2D::ClearCache()
{
  hasContainmentCache=false;
  contains.clear();
}
