#include "Arrangement1D.h"
#include <errors.h>
#include <iostream>
#include <algorithm>
using namespace std;
using namespace Geometry;

Arrangement1D::Arrangement1D()
{
  LeftInterval i;
  i.interval.first = -Inf;
  i.interval.second = Inf;
  intervals[-Inf]=i;
}

void Arrangement1D::Insert(Real imin,Real imax,int id)
{
  Assert(imin <= imax);
  SortedIntervals::iterator low = LocateInterval(imin);
  Assert(low != intervals.end());
  if(IsInf(imin)==-1) {
    Assert(IsInf(imax)!=-1);
    //don't split the segment or add id to the point
  }
  else {
    if(imin != low->second.interval.first) { //doesn't hit the point
      //split low interval
      Split(low,imin);
      ++low;
    }
    low->second.pointIDs.push_back(id);
  }
  SortedIntervals::iterator high = LocateInterval(imax);
  Assert(high != intervals.end());
  if(IsInf(imax)==1) {
    Assert(IsInf(imin)!=1);
    while(low != high) {
      low->second.intervalIDs.push_back(id);
      ++low;
      low->second.pointIDs.push_back(id);
    }
    high->second.intervalIDs.push_back(id);
  }
  else {
    if (imax != high->second.interval.first) {  //doesn't hit the point
      //split high interval
      Split(high,imax);
      ++high;
    }
    while(low != high) {
      low->second.intervalIDs.push_back(id);
      ++low;
      low->second.pointIDs.push_back(id);
    }
  }
  /*
    NOTE: this method must be slightly more complex than this to deal with infinity cases
  //insert ids into all of the segments
  low->second.pointIDs.push_back(id);
  while(low != high) {
    low->second.intervalIDs.push_back(id);
    ++low;
    low->second.pointIDs.push_back(id);
  }
  */
}

void Arrangement1D::InsertUnique(Real imin,Real imax,int id)
{
  Assert(imin <= imax);
  SortedIntervals::iterator low = LocateInterval(imin);
  Assert(low != intervals.end());
  if(IsInf(imin)==-1) {
    Assert(IsInf(imax)!=-1);
    //don't split the segment or add id to the point
  }
  else {
    if(imin != low->second.interval.first) { //doesn't hit the point
      //split low interval
      Split(low,imin);
      ++low;
    }
    if(count(low->second.pointIDs.begin(),low->second.pointIDs.end(),id)==0)
      low->second.pointIDs.push_back(id);
  }
  SortedIntervals::iterator high = LocateInterval(imax);
  Assert(high != intervals.end());
  if(IsInf(imax)==1) {
    Assert(IsInf(imin)!=1);
    while(low != high) {
      if(count(low->second.intervalIDs.begin(),low->second.intervalIDs.end(),id)==0)
	low->second.intervalIDs.push_back(id);
      ++low;
      if(count(low->second.pointIDs.begin(),low->second.pointIDs.end(),id)==0)
	low->second.pointIDs.push_back(id);
    }
    if(count(high->second.intervalIDs.begin(),high->second.intervalIDs.end(),id)==0)
      high->second.intervalIDs.push_back(id);
  }
  else {
    if (imax != high->second.interval.first) {  //doesn't hit the point
      //split high interval
      Split(high,imax);
      ++high;
    }
    while(low != high) {
      if(count(low->second.intervalIDs.begin(),low->second.intervalIDs.end(),id)==0)
	low->second.intervalIDs.push_back(id);
      ++low;
      if(count(low->second.pointIDs.begin(),low->second.pointIDs.end(),id)==0)
	low->second.pointIDs.push_back(id);
    }
  }
}

void Arrangement1D::GetIntervals(vector<Interval>& segs,vector<const IDList*>& ids) const
{
  size_t n=intervals.size();
  segs.reserve(n);
  ids.reserve(n);
  segs.resize(0);
  ids.resize(0);
  for(SortedIntervals::const_iterator i=intervals.begin();i!=intervals.end();i++) {
    if(i->second.pointIDs.size() != i->second.intervalIDs.size()) {
      if(i!=intervals.begin()) {
	SortedIntervals::const_iterator p=i; --p;
	if(i->second.pointIDs.size() != p->second.intervalIDs.size()) {
	  segs.push_back(pair<Real,Real>(i->first,i->first));
	  ids.push_back(&i->second.pointIDs);
	}
      }
    }
    if(!i->second.intervalIDs.empty()) {
      segs.push_back(i->second.interval);
      ids.push_back(&i->second.intervalIDs);
    }
  }
}

void Arrangement1D::GetAllIntervals(vector<Interval>& segs,vector<const IDList*>& ids) const
{
  size_t n=intervals.size();
  segs.reserve(n);
  ids.reserve(n);
  segs.resize(0);
  ids.resize(0);
  for(SortedIntervals::const_iterator i=intervals.begin();i!=intervals.end();i++) {
    if(i->second.pointIDs.size() != i->second.intervalIDs.size()) {
      if(i!=intervals.begin()) {
	SortedIntervals::const_iterator p=i; --p;
	if(i->second.pointIDs.size() != p->second.intervalIDs.size()) {
	  segs.push_back(pair<Real,Real>(i->first,i->first));
	  ids.push_back(&i->second.pointIDs);
	}
      }
    }
    segs.push_back(i->second.interval);
    ids.push_back(&i->second.intervalIDs);
  }
}

void Arrangement1D::GetOverlapIntervals(Real imin,Real imax,vector<Interval>& segs,vector<const IDList*>& ids) const
{
  Assert(imin <= imax);
  segs.resize(0);

  SortedIntervals::const_iterator low = LocateInterval(imin);
  Assert(low != intervals.end());
  SortedIntervals::const_iterator high = LocateInterval(imax);
  Assert(high != intervals.end());

  bool checkFirstPoint = false;
  bool checkLastInterval = true;
  if(IsInf(imin)==0 && imin == low->first) checkFirstPoint = true;
  if(IsInf(imax)==0 && imax == high->first) checkLastInterval = false;
  if(checkFirstPoint) {
    if(low->second.pointIDs.size() != low->second.intervalIDs.size() || imin==imax) {
      segs.push_back(pair<Real,Real>(low->first,low->first));
      ids.push_back(&low->second.pointIDs);
    }
  }
  //printf("Overlap %g %g, intervals %g %g, check %d %d\n",imin,imax,low->first,high->first,(int)checkFirstPoint,(int)checkLastInterval);
  while(low != high) {
    segs.push_back(low->second.interval);
    ids.push_back(&low->second.intervalIDs);
    ++low;
    if(low->second.pointIDs.size() != low->second.intervalIDs.size()) {
      if(low!=intervals.begin()) {
	SortedIntervals::const_iterator p=low; --p;
	if(low->second.pointIDs.size() != p->second.intervalIDs.size()) {
	  segs.push_back(pair<Real,Real>(low->first,low->first));
	  ids.push_back(&low->second.pointIDs);
	}
      }
    }
  }
  if(checkLastInterval) {
    segs.push_back(pair<Real,Real>(low->second.interval.first,imax));
    ids.push_back(&low->second.intervalIDs);    
  }
  //if it's only 1 interval need to do this:
  segs.front().first = imin;
}

Arrangement1D::SortedIntervals::iterator Arrangement1D::LocateInterval(Real x)
{
  int res=IsInf(x);
  if(res == 1) return --intervals.end();
  else if(res == -1) return intervals.begin();
  SortedIntervals::iterator i=intervals.upper_bound(x);
  --i;
  if(i->second.interval.first > x || x >= i->second.interval.second) {
    cerr<<"Arrangement1D: LocateInterval failed"<<endl;
    cerr<<"x = "<<x<<", interval = ["<<i->second.interval.first<<", "<<i->second.interval.second<<")"<<endl;
    i=intervals.upper_bound(x);
    cerr<<"upper bounded interval = ["<<i->second.interval.first<<", "<<i->second.interval.second<<")"<<endl;
  }
  Assert(i->second.interval.first <= x && x < i->second.interval.second);
  return i;
}

Arrangement1D::SortedIntervals::const_iterator Arrangement1D::LocateInterval(Real x) const
{
  int res=IsInf(x);
  if(res == 1) return --intervals.end();
  else if(res == -1) return intervals.begin();
  SortedIntervals::const_iterator i=intervals.upper_bound(x);
  --i;
  if(i->second.interval.first > x || x >= i->second.interval.second) {
    cerr<<"Arrangement1D: LocateInterval failed"<<endl;
    cerr<<"x = "<<x<<", interval = ["<<i->second.interval.first<<", "<<i->second.interval.second<<")"<<endl;
    i=intervals.upper_bound(x);
    cerr<<"upper bounded interval = ["<<i->second.interval.first<<", "<<i->second.interval.second<<")"<<endl;
  }
  Assert(i->second.interval.first <= x && x < i->second.interval.second);
  return i;
}

void Arrangement1D::Split(SortedIntervals::iterator interval,Real x)
{
  if(x <= interval->second.interval.first || x >= interval->second.interval.second) {
    cerr<<"Arrangement1D::Split(): interval doesn't contain x"<<endl;
    cerr<<"x = "<<x<<", interval = ["<<interval->second.interval.first<<", "<<interval->second.interval.second<<")"<<endl;
  }
  Assert(x > interval->second.interval.first && x < interval->second.interval.second);
  Real right = interval->second.interval.second;
  interval->second.interval.second = x;
  LeftInterval newInterval;
  newInterval.interval.first = x;
  newInterval.interval.second = right;
  newInterval.pointIDs = interval->second.intervalIDs;
  newInterval.intervalIDs = interval->second.intervalIDs;
  intervals[x] = newInterval;
}
