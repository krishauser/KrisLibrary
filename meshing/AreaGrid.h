#ifndef AREA_GRID_H
#define AREA_GRID_H

#include <KrisLibrary/structs/array2d.h>
#include <KrisLibrary/math3d/AABB2D.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/indexing.h>

namespace Meshing {

  using namespace Math3D;

/** @ingroup Meshing
 * @brief Iterator over a 2D area grid.
 *
 * Allows quick iteration and access to the 2D cell.
 * Use the ++ operator until isDone() return true.
 */
template <class T>
struct AreaGridIterator
{
  AreaGridIterator(const Array2D<T>& cells,const AABB2D& bb);
  void setRange(const IntPair& bmin,const IntPair& bmax);
  inline const T& operator *() const { return *it; }
  inline T& operator *() { return *it; }
  inline const IntPair& getIndex() const { return index; }
  void operator ++();
  inline bool isDone() const { return index.a > hi.a; }
  inline void getCell(AABB2D& cell) const
  { cell.bmin=cellCorner; cell.bmax=cellCorner+cellSize; }
  inline void getCellCenter(Vector2& c) const { c=cellCorner+cellSize*Half; }

  const Array2D<T>& cells;
  const AABB2D& bb;
  typename Array2D<T>::iterator it;
  IntPair lo,hi;
  IntPair index;
  Vector2 cellCorner,cellSize;
  Vector2 bbMin;
};

/** @ingroup Meshing
 * @brief A 2D array over an axis-aligned 2D box, containing Real values.
 *
 * GetIndex returns the cell containing a 2D point.
 * GetIndexAndParams returns the cell containing a 2D point, and its
 * coordinates within that cell.
 *
 * Cells can be quickly iterated over using the iterator class.
 * 
 * Values are interpreted so they are defined over an entire cell,
 * *not* the vertices.
 *
 * Most methods just clamp their arguments to the domain rather than
 * doing bounds checks.
 */
template <class T>
class AreaGridTemplate
{
 public:
  typedef AreaGridTemplate<T> MyT;

  bool IsEmpty() const { return value.empty(); }
  void Resize(int m,int n) { value.resize(m,n); }
  void ResizeByResolution(const Vector2& res);
  template <class T2>
  void MakeSimilar(const AreaGridTemplate<T2>& grid);
  template <class T2>
  bool IsSimilar(const AreaGridTemplate<T2>& grid) const;
  void GetCell(int i,int j,AABB2D& cell) const;
  void GetCellCenter(int i,int j,Vector2& center) const;
  Vector2 GetCellSize() const;
  void GetIndex(const Vector2& pt,int& i,int& j) const;
  void GetIndexAndParams(const Vector2& pt,IntPair& index,Vector2& params) const;
  void GetIndexRange(const AABB2D& range,IntPair& imin,IntPair& imax) const;
  inline void GetCell(const IntPair& index,AABB2D& cell) const { GetCell(index.a,index.b,cell); }
  inline void GetCenter(const IntPair& index,Vector2& center) const { GetCellCenter(index.a,index.b,center); }
  inline void GetIndex(const Vector2& pt,IntPair& index) const { GetIndex(pt,index.a,index.b); }
  inline void SetValue(int i,int j,const T& v) { value(i,j)=v; }
  inline void SetValue(const Vector2& pt,const T& v) { int i,j; GetIndex(pt,i,j); value(i,j)=v; }
  inline T GetValue(int i,int j) const { return value(i,j); }
  inline T GetValue(const Vector2& pt) const { int i,j; GetIndex(pt,i,j); return value(i,j); }

  ///Computes the trilinear interpolation of the field at pt, assuming values are sampled exactly at cell centers
  T BilinearInterpolate(const Vector2& pt) const;
  ///Average value of the range.  Each cell's value is weighted by the volume overlap with range
  T Average(const AABB2D& range) const;
  ///Resamples the given volume grid onto the current grid, taking trilinear interpolation at cell centers
  void ResampleBilinear(const MyT& grid);
  ///Resamples the given volume grid onto the current grid, taking averages over grid cells
  void ResampleAverage(const MyT& grid);
  ///Returns the gradient estimated using forward differencing
  void Gradient_ForwardDifference(const IntPair& index,Vector2& grad) const;
  ///Returns the gradient estimated using centered differencing
  void Gradient_CenteredDifference(const IntPair& index,Vector2& grad) const;
  ///Returns the trilinear interpolated gradient.  If any element of pt is on a cell boundary the gradient estimated
  ///using centered differencing
  void Gradient(const Vector2& pt,Vector2& grad) const;
  void Add(const MyT& grid);
  void Subtract(const MyT& grid);
  void Multiply(const MyT& grid);
  void Max(const MyT& grid);
  void Min(const MyT& grid);
  void Add(T val);
  void Multiply(T val);
  void Max(T val);
  void Min(T val);

  typedef AreaGridIterator<T> iterator;
  iterator getIterator() const { return iterator(value,bb); }

  Array2D<T> value;
  AABB2D bb;
};

typedef AreaGridTemplate<Real> AreaGrid;

template <class T>
std::istream& operator >> (std::istream& in,AreaGridTemplate<T>& grid)
{
  in>>grid.bb.bmin>>grid.bb.bmax;
  in>>grid.value;
  return in;
}

template <class T>
std::ostream& operator << (std::ostream& out,const AreaGridTemplate<T>& grid)
{
  out<<grid.bb.bmin<<"    "<<grid.bb.bmax<<std::endl;
  out<<grid.value<<std::endl;
  return out;
}


template <class T>
template <class T2>
void AreaGridTemplate<T>::MakeSimilar(const AreaGridTemplate<T2>& grid)
{
  bb=grid.bb;
  value.resize(grid.value.m,grid.value.n);
}

template <class T>
template <class T2>
bool AreaGridTemplate<T>::IsSimilar(const AreaGridTemplate<T2>& grid) const
{
  return (grid.value.m == value.m && grid.value.n == value.n) && (bb.bmin == grid.bb.bmin && bb.bmax == grid.bb.bmax);
}

template <class T>
AreaGridIterator<T>::AreaGridIterator(const Array2D<T>& _cells,const AABB2D& _bb)
  :cells(_cells),bb(_bb),it(&cells)
{
  cellSize.x = (bb.bmax.x-bb.bmin.x)/Real(cells.m);
  cellSize.y = (bb.bmax.y-bb.bmin.y)/Real(cells.n);
  lo.set(0,0);
  hi.set(_cells.m-1,_cells.n-1);
  index=lo;
  cellCorner = bbMin = bb.bmin;
}

template <class T>
void AreaGridIterator<T>::setRange(const IntPair& bmin,const IntPair& bmax)
{
  lo=bmin;
  hi=bmax;
  bbMin = bb.bmin;
  bbMin.x += Real(lo.a)*cellSize.x;
  bbMin.y += Real(lo.b)*cellSize.y;
  cellCorner = bbMin;
  it = cells.begin(Range2Indices(lo.a,hi.a+1,
                 lo.b,hi.b+1));
  index=lo;
}

template <class T>
void AreaGridIterator<T>::operator ++()
{
  ++it;
  index.b++;
  cellCorner.y+=cellSize.y;
  if(index.b > hi.b) {
    index.b=lo.b;
    cellCorner.y=bbMin.y;
    index.a++;
    cellCorner.x+=cellSize.x;
  }
  Assert(index == it.getElement());
}


} //namespace Meshing

#endif

