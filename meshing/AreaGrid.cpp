#include <KrisLibrary/Logger.h>
#include "AreaGrid.h"
#include <iostream>
using namespace std;

namespace Meshing {

template <class T>
void AreaGridTemplate<T>::ResizeByResolution(const Vector2& res)
{
  Assert(res.x > 0 && res.y > 0);
  int m=(int)Ceil((bb.bmax.x-bb.bmin.x)/res.x);
  int n=(int)Ceil((bb.bmax.y-bb.bmin.y)/res.y);
  value.resize(m,n);
}

template <class T>
Vector2 AreaGridTemplate<T>::GetCellSize() const
{
  Vector2 size=bb.bmax-bb.bmin;
  size.x /= Real(value.m);
  size.y /= Real(value.n);
  return size;
}

template <class T>
void AreaGridTemplate<T>::GetCell(int i,int j,AABB2D& cell) const
{
  Real u=Real(i)/Real(value.m);
  Real v=Real(j)/Real(value.n);
  cell.bmin.x = bb.bmin.x + u*(bb.bmax.x-bb.bmin.x);
  cell.bmin.y = bb.bmin.y + v*(bb.bmax.y-bb.bmin.y);
  Real u2=Real(i+1)/Real(value.m);
  Real v2=Real(j+1)/Real(value.n);
  cell.bmax.x = bb.bmin.x + u2*(bb.bmax.x-bb.bmin.x);
  cell.bmax.y = bb.bmin.y + v2*(bb.bmax.y-bb.bmin.y);
}

template <class T>
void AreaGridTemplate<T>::GetCellCenter(int i,int j,Vector2& center) const
{
  Real u=(Real(i)+0.5)/Real(value.m);
  Real v=(Real(j)+0.5)/Real(value.n);
  center.x = bb.bmin.x + u*(bb.bmax.x-bb.bmin.x);
  center.y = bb.bmin.y + v*(bb.bmax.y-bb.bmin.y);
}

template <class T>
void AreaGridTemplate<T>::GetIndex(const Vector2& pt,int& i,int& j) const
{
  Real u=(pt.x - bb.bmin.x)/(bb.bmax.x-bb.bmin.x);
  Real v=(pt.y - bb.bmin.y)/(bb.bmax.y-bb.bmin.y);

  i = (int)Floor(u*value.m);
  j = (int)Floor(v*value.n);
}

template <class T>
void AreaGridTemplate<T>::GetIndexAndParams(const Vector2& pt,IntPair& index,Vector2& params) const
{
  Real u=(pt.x - bb.bmin.x)/(bb.bmax.x-bb.bmin.x)*value.m;
  Real v=(pt.y - bb.bmin.y)/(bb.bmax.y-bb.bmin.y)*value.n;

  Real ri = Floor(u);
  Real rj = Floor(v);
  
  //set params u,v,w to their fractional component
  params.x = u - ri;
  params.y = v - rj;
  //set the index to the integer component
  index.a = (int)ri;
  index.b = (int)rj;
}

template <class T>
void AreaGridTemplate<T>::GetIndexRange(const AABB2D& range,IntPair& imin,IntPair& imax) const
{
  GetIndex(range.bmin,imin);
  GetIndex(range.bmax,imax);
}

template <class T>
T AreaGridTemplate<T>::BilinearInterpolate(const Vector2& pt) const
{
  Real u=(pt.x - bb.bmin.x)/(bb.bmax.x-bb.bmin.x)*value.m;
  Real v=(pt.y - bb.bmin.y)/(bb.bmax.y-bb.bmin.y)*value.n;
 
  Real ri = Floor(u);
  Real rj = Floor(v);
 
  //set u,v,w to their fractional component
  u = u - ri;
  v = v - rj;
 
  //get the base cell index
  int i1=(int)ri;
  int j1=(int)rj;
 
  //get the alternate cell indices, interpolation parameters
  //(u interpolates between i1,i2, etc)
  int i2,j2;
  if(u > 0.5) { i2=i1+1; u = u-0.5; }
  else { i2=i1; i1--; u = 0.5+u; }
  if(v > 0.5) { j2=j1+1; v = v-0.5; }
  else { j2=j1; j1--; v = 0.5+v; }
 
  if(i1 < 0) i1=0; if(i1 >= value.m) i1=value.m-1;
  if(i2 < 0) i2=0; if(i2 >= value.m) i2=value.m-1;
  if(j1 < 0) j1=0; if(j1 >= value.n) j1=value.n-1;
  if(j2 < 0) j2=0; if(j2 >= value.n) j2=value.n-1;
  T w1 = (1-v)*value(i1,j1)+v*value(i1,j2);
  T w2 = (1-v)*value(i2,j1)+v*value(i2,j2);
  return (1-u)*w1 + u*w2;
}


template <class T>
T AreaGridTemplate<T>::Average(const AABB2D& range) const
{
  IntPair imin,imax;
  GetIndexRange(range,imin,imax);
  //check range
  if(imax.a < 0 || imax.b < 0) {
    return 0;
  }
  if(imin.a >= value.m || imin.b >= value.n) {
    return 0;
  }
  //limit range
  if(imin.a < 0) imin.a=0;
  if(imin.b < 0) imin.b=0;
  if(imax.a >= value.m) imax.a=value.m-1;
  if(imax.b >= value.n) imax.b=value.n-1;

  bool ignoreX=(range.bmin.x==range.bmax.x),ignoreY=(range.bmin.y==range.bmax.y);

  Vector2 cellcorner;
  Vector2 cellsize;
  cellsize.x = (bb.bmax.x-bb.bmin.x)/Real(value.m);
  cellsize.y = (bb.bmax.y-bb.bmin.y)/Real(value.n);
  Real sumValue=0;
  Real sumArea=0;
  cellcorner.x = bb.bmin.x + imin.a*cellsize.x;
  for(int i=imin.a;i<=imax.a;i++,cellcorner.x += cellsize.x) {
    cellcorner.y = bb.bmin.y + imin.b*cellsize.y;
    for(int j=imin.b;j<=imax.b;j++,cellcorner.y += cellsize.y) {
    AABB2D intersect;
    intersect.bmin=cellcorner;
    intersect.bmax=cellcorner+cellsize;
    intersect.setIntersection(range);
    Vector2 isectsize=intersect.bmax-intersect.bmin;
    //due to rounding errors, may have negative sizes
    if(isectsize.x < 0 || isectsize.y < 0) continue;
    Real volume=1;
    if(!ignoreX) volume*=isectsize.x;
    if(!ignoreY) volume*=isectsize.y;
    sumValue += volume*value(i,j);
    sumArea += volume;
    }
  }
  Vector2 rangesize=range.bmax-range.bmin;
  Real rangeArea = 1;
  if(!ignoreX) rangeArea*=rangesize.x;
  if(!ignoreY) rangeArea*=rangesize.y;
  Assert(sumArea < rangeArea + Epsilon);
  return sumValue / rangeArea;
}

template <class T>
void AreaGridTemplate<T>::Gradient_ForwardDifference(const IntPair& _index,Vector2& grad) const
{
  IntPair index = _index;
  if(index.a < 0) index.a = 0;  if(index.a >= value.m) index.a = value.m-1;
  if(index.b < 0) index.b = 0;  if(index.b >= value.n) index.b = value.n-1;
  Real cv = value(index);
  Vector2 h = GetCellSize();
  if(index.a+1 < value.m) grad.x = (value(index.a+1,index.b) - cv)/h.x;
  else grad.x = (cv - value(index.a-1,index.b))/h.x;
  if(index.b+1 < value.n) grad.y = (value(index.a,index.b+1) - cv)/h.y;
  else grad.y = (cv - value(index.a,index.b-1))/h.y;
}

template <class T>
void AreaGridTemplate<T>::Gradient_CenteredDifference(const IntPair& _index,Vector2& grad) const
{
  IntPair index = _index;
  if(index.a < 0) index.a = 0;  if(index.a >= value.m) index.a = value.m-1;
  if(index.b < 0) index.b = 0;  if(index.b >= value.n) index.b = value.n-1;
  Vector2 h = GetCellSize();
  Real n,p;
  int shift;
  shift=0;
  if(index.a+1 < value.m) { n = value(index.a+1,index.b); shift++; }
  else n = value(index);
  if(index.a > 0) { p = value(index.a-1,index.b); shift++; }
  else p = value(index);
  grad.x = (n - p)/(shift*h.x);

  shift=0;
  if(index.b+1 < value.n) { n = value(index.a,index.b+1); shift++; }
  else n = value(index);
  if(index.b > 0) { p = value(index.a,index.b-1); shift++; }
  else p = value(index);
  grad.y = (n - p)/(shift*h.y);
}

template <class T>
void AreaGridTemplate<T>::Gradient(const Vector2& pt,Vector2& grad) const
{
  IntPair ind;
  Vector2 params;
  GetIndexAndParams(pt,ind,params);
  Real u,v;
  params.get(u,v);
  int i1,j1;
  i1 = ind.a;
  j1 = ind.b;

  ///NOTE: THIS CALCULATION IS SUSPECT

  //get the alternate cell indices, interpolation parameters
  //(u interpolates between i1,i2, etc)
  int i2,j2;
  if(u > 0.5) { i2=i1+1; u = u-0.5; }
  else { i2=i1; i1--; u = 0.5+u; }
  if(v > 0.5) { j2=j1+1; v = v-0.5; }
  else { j2=j1; j1--; v = 0.5+v; }

  if(i1 < 0) i1=0; if(i1 >= value.m) i1=value.m-1;
  if(i2 < 0) i2=0; if(i2 >= value.m) i2=value.m-1;
  if(j1 < 0) j1=0; if(j1 >= value.n) j1=value.n-1;
  if(j2 < 0) j2=0; if(j2 >= value.n) j2=value.n-1;
  Real v1 = (1-u)*value(i1,j1)+u*value(i2,j1);
  Real v2 = (1-u)*value(i1,j2)+u*value(i2,j2);
  Real w1 = (1-v)*value(i1,j1)+v*value(i1,j2);
  Real w2 = (1-v)*value(i2,j1)+v*value(i2,j2);
  //Real res = (1-u)*w1 + u*w2;
  Vector2 h = GetCellSize();
  //u = x/h.x+bx, v = y/h.y+by
  //res = (1-u)*w1(v,w) + u*w2(v,w)
  if(u==0.5 || v==0.5 || i1==i2 || j1==j2) {
    Gradient_CenteredDifference(ind,grad);
  }
  if(u != 0.5 && i1 != i2) 
    grad.x = (w2 - w1)/h.x;
  if(v != 0.5 && j1 != j2) 
    grad.y = (v2 - v1)/h.y;
}

template <class T>
void AreaGridTemplate<T>::ResampleBilinear(const MyT& grid)
{
  if(IsSimilar(grid)) {
    value = grid.value;
    return;
  }
  Vector2 c;
  for(iterator it=getIterator();!it.isDone();++it) {
    it.getCellCenter(c);
    *it = grid.BilinearInterpolate(c);
  }
}

template <class T>
void AreaGridTemplate<T>::ResampleAverage(const MyT& grid)
{
  if(IsSimilar(grid)) {
    value = grid.value;
    return;
  }


  AABB2D cell;
  for(iterator it=getIterator();!it.isDone();++it) {
    it.getCell(cell);
    *it = grid.Average(cell);
  }
}

template <class T>
void AreaGridTemplate<T>::Add(const MyT& grid)
{
  if(!IsSimilar(grid)) {
    MyT resample;
    resample.value.resize(value.m,value.n);
    resample.bb = bb;
    resample.ResampleAverage(grid);
    Add(resample);
    return;
  }

  for(typename Array2D<T>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i += *j;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j) += grid.value(i,j);
  */
}

template <class T>
void AreaGridTemplate<T>::Subtract(const MyT& grid)
{
  if(!IsSimilar(grid)) {
    MyT resample;
    resample.value.resize(value.m,value.n);
    resample.bb = bb;
    resample.ResampleAverage(grid);
    Subtract(resample);
    return;
  }

  for(typename Array2D<T>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i -= *j;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j) -= grid.value(i,j);
  */
}

template <class T>
void AreaGridTemplate<T>::Multiply(const MyT& grid)
{
  if(!IsSimilar(grid)) {
    MyT resample;
    resample.value.resize(value.m,value.n);
    resample.bb = bb;
    resample.ResampleAverage(grid);
    Multiply(resample);
    return;
  }

  for(typename Array2D<T>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i *= *j;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
    value(i,j,k) *= grid.value(i,j,k);
  */
}

template <class T>
void AreaGridTemplate<T>::Max(const MyT& grid)
{
  if(!IsSimilar(grid)) {
    MyT resample;
    resample.value.resize(value.m,value.n);
    resample.bb = bb;
    resample.ResampleAverage(grid);
    Max(resample);
    return;
  }

  for(typename Array2D<T>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i = ::Max(*i,*j);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j,k) = ::Max(value(i,j),grid.value(i,j));
  */
}

template <class T>
void AreaGridTemplate<T>::Min(const MyT& grid)
{
  if(!IsSimilar(grid)) {
    MyT resample;
    resample.value.resize(value.m,value.n);
    resample.bb = bb;
    resample.ResampleAverage(grid);
    Min(resample);
    return;
  }

  for(typename Array2D<T>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i = ::Min(*i,*j);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j,k) = ::Min(value(i,j),grid.value(i,j));
  */
}

template <class T>
void AreaGridTemplate<T>::Add(T val)
{
  for(typename Array2D<T>::iterator i=value.begin();i!=value.end();++i) {
    *i += val;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
    value(i,j,k) += val;
  */
}

template <class T>
void AreaGridTemplate<T>::Multiply(T val)
{
  for(typename Array2D<T>::iterator i=value.begin();i!=value.end();++i) {
    *i *= val;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j) *= val;
  */
}

template <class T>
void AreaGridTemplate<T>::Max(T val)
{
  for(typename Array2D<T>::iterator i=value.begin();i!=value.end();++i) {
    *i = ::Max(*i,val);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j) = ::Max(value(i,j),val);
  */
}

template <class T>
void AreaGridTemplate<T>::Min(T val)
{
  for(typename Array2D<T>::iterator i=value.begin();i!=value.end();++i) {
    *i = ::Min(*i,val);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
    value(i,j) = ::Min(value(i,j),val);
  */
}


///forward declarations
template class AreaGridTemplate<float>;
template class AreaGridTemplate<double>;
template class AreaGridTemplate<int>;
template class AreaGridTemplate<char>;


} //namespace Meshing
