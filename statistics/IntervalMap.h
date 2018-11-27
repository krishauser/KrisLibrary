#ifndef STATS_INTERVAL_MAP_H
#define STATS_INTERVAL_MAP_H

#include <KrisLibrary/Logger.h>
#include "statistics.h"
#include <vector>
#include <KrisLibrary/structs/array2d.h>

namespace Statistics {

/** @ingroup Statistics
 * @brief Division of the real numbers into interval sets.  Each interval
 * is assigned data of type Data.
 */
template <class Data>
class IntervalMap
{
 public:
  IntervalMap();
  /// Sets the interval to be (-inf,inf)
  void Clear(); 
  /// Creates the intervals (-inf,split],(split,inf)
  void InitSplit(Real split);
  /// Creates n uniformly spaced intervals betwen a,b 
  void InitPartition(size_t n,Real a,Real b);
  /// Fills all intervals with the given value
  void Fill(const Data& val) 
  { std::fill(intervals.begin(),intervals.end(),val); }

  /// Returns the range of the interval indexed by index
  void IntervalRange(int index,Real& min,Real& max) const;
  /// Returns the index of the interval containing val
  int GetIntervalIndex(Real val) const;
  /// Returns the data of the interval containing val
  const Data& GetIntervalData(Real val) const 
  { return intervals[GetIntervalIndex(val)]; }
  Data& GetIntervalData(Real val)
  { return intervals[GetIntervalIndex(val)]; }

  std::vector<Real> divs;
  /// divs.size()+1 intervals (-inf,x0],(x1,x2],...,(xn-1,xn],(xn,inf)
  std::vector<Data> intervals;
};

template <class Data>
class IntervalMap2D
{
 public:
  typedef Real Point[2];
  typedef int Index[2];
  typedef size_t Size[2];
  
  IntervalMap2D();
  /// Sets the interval to be (-inf,inf)x(-inf,inf)
  void Clear(); 
  /// Creates mxn uniformly spaced intervals betwen min,max
  void InitPartition(const Size dims,const Point min, const Point max);
  /// Fills all intervals with the given value
  void Fill(const Data& val) 
  { intervals.set(val); }

  /// Returns the range of the interval indexed by index
  void IntervalRange(const Index index,Point min,Point max) const;
  /// Returns the index of the interval containing val
  void GetIntervalIndex(const Point val,Index index) const;
  /// Returns the data of the interval containing val
  const Data& GetIntervalData(const Point val) const 
  { Index i; GetIntervalIndex(val,i);
    return intervals(i[0],i[1]); }
  Data& GetIntervalData(const Point val)
  { Index i; GetIntervalIndex(val,i);
    return intervals(i[0],i[1]); }

  std::vector<Real> div1,div2;
  Array2D<Data> intervals;
};

template <class Data>
IntervalMap<Data>::IntervalMap()
{
  Clear();
}

template <class Data>
void IntervalMap<Data>::Clear()
{
  divs.clear();
  intervals.resize(1); //infinite
}

template <class Data>
void IntervalMap<Data>::InitSplit(Real splitVal)
{
  divs.resize(1);
  intervals.resize(2);
  divs[0] = splitVal;
}

template <class Data>
void IntervalMap<Data>::InitPartition(size_t n,Real a,Real b)
{
  if(n == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, resizing interval map to have 0 divisions -- was this intended?"<<"\n");
    Clear();
  }
  else {
    divs.resize(n+1);
    intervals.resize(n+2);
    Real x=a, h=(b-a)/n;
    for(size_t i=0;i<=n;i++) {
      divs[i] = x;
      x+=h;
    }
  }
}

template <class Data>
void IntervalMap<Data>::IntervalRange(int index,Real& min,Real& max) const
{
  assert(divs.size()+1 == intervals.size());
  assert(index >= 0 && index < (int)intervals.size());
  if(index == 0) min=-Inf; 
  else min = divs[index-1];
  if(index+1 == (int)intervals.size()) max=Inf;
  else max = divs[index];
}

template <class Data>
int IntervalMap<Data>::GetIntervalIndex(Real val) const
{
  if(intervals.empty()) return -1;
  if(val <= divs.front()) return 0;
  else if(val > divs.back()) return intervals.size()-1;
  else {
    std::vector<Real>::const_iterator it = --std::lower_bound(divs.begin(),divs.end(),val);
    int index=(it-divs.begin())+1;
    //Next 3 lines are for debugging only
    Real a,b;
    IntervalRange(index,a,b);
    assert(a < val && val <= b);
    return index;
  }
}


template <class Data>
IntervalMap2D<Data>::IntervalMap2D()
{
  Clear();
}

template <class Data>
void IntervalMap2D<Data>::Clear()
{
  div1.clear();
  div2.clear();
  intervals.resize(1,1); //infinite
}

template <class Data>
void IntervalMap2D<Data>::InitPartition(const Size dims,const Point min,const Point max)
{
  if(dims[0] == 0 || dims[1] == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, resizing interval map to have 0 divisions -- was this intended?"<<"\n");
    Clear();
  }
  else {
    size_t m=dims[0],n=dims[1];
    div1.resize(m+1);
    div2.resize(n+1);
    intervals.resize(m+2,n+2);
    Real x=min[0], dx=(max[0]-min[0])/m;
    for(size_t i=0;i<=m;i++) {
      div1[i] = x;
      x+=dx;
    }
    Real y=min[1], dy=(max[1]-min[1])/n;
    for(size_t i=0;i<=n;i++) {
      div2[i] = y;
      y+=dy;
    }
  }
}

template <class Data>
void IntervalMap2D<Data>::IntervalRange(const Index index,Point min,Point max) const
{
  assert(div1.size()+1 == intervals.m);
  assert(div2.size()+1 == intervals.n);
  assert(index[0] >= 0 && index[0] < (int)intervals.m);
  assert(index[1] >= 0 && index[1] < (int)intervals.n);
  min[0] = (index[0] == 0? -Inf: div1[index[0]-1]);
  max[0] = (index[0]+1 == intervals.m? Inf: div1[index[0]]);
  min[1] = (index[1] == 0? -Inf: div2[index[1]-1]);
  max[1] = (index[1]+1 == intervals.n? Inf: div2[index[1]]);
}

template <class Data>
void IntervalMap2D<Data>::GetIntervalIndex(const Point val,Index index) const
{
  if(intervals.empty()) { index[0]=index[1]=-1; return; }
  if(val[0] <= div1.front()) index[0]=0;
  else if(val[0] > div1.back()) index[0]=intervals.m-1;
  else {
    std::vector<Real>::const_iterator it = --std::lower_bound(div1.begin(),div1.end(),val[0]);
    index[0]=(it-div1.begin())+1;
  }
  if(val[1] <= div2.front()) index[1]=0;
  else if(val[1] > div2.back()) index[1]=intervals.n-1;
  else {
    std::vector<Real>::const_iterator it = --std::lower_bound(div2.begin(),div2.end(),val[1]);
    index[1]=(it-div2.begin())+1;
  }
}


} //namespace Statistics

#endif
