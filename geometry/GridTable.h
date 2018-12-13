#ifndef GRID_TABLE_H
#define GRID_TABLE_H

#include "Grid.h"

namespace Geometry {

/** @ingroup Geometry
 * @brief A table with entry T on a gridding of n-dimensional space.
 *
 * Indices run from imin to imax, inclusive.
 */
template <class T>
class GridTable
{
public:
  typedef Grid::Index Index;
  //typedef bool (*QueryCallback)(T&);

  GridTable(int numDims,Real h=1);
  GridTable(const Vector& h);

  void Clear();
  void Init(const Index& imin,const Index& imax);
  void Set(const T& val);

  inline T& operator[](const Index& i) { return values[ElementIndex(i)]; }
  inline const T& operator[](const Index& i) const { return values[ElementIndex(i)]; }

  int ElementIndex(const Index& i) const;

  Grid grid;
  Index imin,imax;
  vector<T> values;
};


template <class T>
GridTable<T>::GridTable(int numDims,Real h)
  :grid(numDims,h),imin(numDims,0),imax(numDims,0),values(1)
{}

template <class T>
GridTable<T>::GridTable(const Vector& h)
  :grid(h),imin(h.n,0),imax(h.n,0),values(1)
{}

template <class T>
void GridTable<T>::Clear()
{
  std::fill(imin.begin(),imin.end(),0);
  std::fill(imax.begin(),imax.end(),0);
  values.resize(1);
}

template <class T>
void GridTable<T>::Init(const Index& _imin,const Index& _imax)
{
  Assert(imin.size()==_imin.size());
  Assert(imax.size()==_imax.size());
  imin = _imin;
  imax = _imax;
  int size=1;
  for(size_t i=0;i<imin.size();i++) {
    Assert(imax[i] >= imin[i]);
    size *= (imax[i]-imin[i]+1);
  }
  values.resize(size);
}

template <class T>
void GridTable<T>::Set(const T& val)
{
  fill(values.begin(),values.end(),val);
}

template <class T>
int GridTable<T>::ElementIndex(const Index& i) const
{
  Assert(i.size()==imin.size());
  int index=0;
  for(size_t k=0;k<imin.size();k++) {
    index *= (imax[k]-imin[k]+1);
    Assert(i[k] >= imin[k] && i[k] <= imax[k]);
    index += i[k]-imin[k];
  }
  Assert(index >= 0 && index < (int)values.size());
  return index;
}

} //namespace Geometry

#endif
