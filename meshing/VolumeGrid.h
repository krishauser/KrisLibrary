#ifndef VOLUME_GRID_H
#define VOLUME_GRID_H

#include <KrisLibrary/structs/array3d.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/indexing.h>

namespace Meshing {

  using namespace Math3D;

/** @ingroup Meshing
 * @brief Iterator over a 3D volume grid.
 *
 * Allows quick iteration and access to the 3D cell.
 * Use the ++ operator until isDone() return true.
 */
template <class T>
struct VolumeGridIterator
{
  VolumeGridIterator(const Array3D<T>& cells,const AABB3D& bb);
  void setRange(const IntTriple& bmin,const IntTriple& bmax);
  inline const T& operator *() const { return *it; }
  inline T& operator *() { return *it; }
  inline const IntTriple& getIndex() const { return index; }
  void operator ++();
  inline bool isDone() const { return index.a > hi.a; }
  inline void getCell(AABB3D& cell) const
  { cell.bmin=cellCorner; cell.bmax=cellCorner+cellSize; }
  inline void getCellCenter(Vector3& c) const { c=cellCorner+cellSize*Half; }

  const Array3D<T>& cells;
  const AABB3D& bb;
  typename Array3D<T>::iterator it;
  IntTriple lo,hi;
  IntTriple index;
  Vector3 cellCorner,cellSize;
  Vector3 bbMin;
};

/** @ingroup Meshing
 * @brief A 3D array over an axis-aligned 3D volume, containing Real values.
 *
 * GetIndex returns the cell containing a 3D point.
 * GetIndexAndParams returns the cell containing a 3D point, and its
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
class VolumeGridTemplate
{
 public:
  typedef VolumeGridTemplate<T> MyT;

  bool IsEmpty() const { return value.empty(); }
  void Resize(int m,int n,int p) { value.resize(m,n,p); }
  void ResizeByResolution(const Vector3& res);
  template <class T2>
  void MakeSimilar(const VolumeGridTemplate<T2>& grid);
  template <class T2>
  bool IsSimilar(const VolumeGridTemplate<T2>& grid) const;
  void GetCell(int i,int j,int k,AABB3D& cell) const;
  void GetCellCenter(int i,int j,int k,Vector3& center) const;
  Vector3 GetCellSize() const;
  void GetIndex(const Vector3& pt,int& i,int& j,int& k) const;
  void GetIndexAndParams(const Vector3& pt,IntTriple& index,Vector3& params) const;
  void GetIndexRange(const AABB3D& range,IntTriple& imin,IntTriple& imax) const;
  inline void GetCell(const IntTriple& index,AABB3D& cell) const { GetCell(index.a,index.b,index.c,cell); }
  inline void GetCenter(const IntTriple& index,Vector3& center) const { GetCellCenter(index.a,index.b,index.c,center); }
  inline void GetIndex(const Vector3& pt,IntTriple& index) const { GetIndex(pt,index.a,index.b,index.c); }
  inline void SetValue(int i,int j,int k,const T& v) { value(i,j,k)=v; }
  inline void SetValue(const Vector3& pt,const T& v) { int i,j,k; GetIndex(pt,i,j,k); value(i,j,k)=v; }
  inline T GetValue(int i,int j,int k) const { return value(i,j,k); }
  inline T GetValue(const Vector3& pt) const { int i,j,k; GetIndex(pt,i,j,k); return value(i,j,k); }

  ///Computes the trilinear interpolation of the field at pt, assuming values are sampled exactly at cell centers
  T TrilinearInterpolate(const Vector3& pt) const;
  ///Used only for fast marching method, really.
  T MinimumFreeInterpolate(const Vector3& pt) const;
  ///Average value of the range.  Each cell's value is weighted by the volume overlap with range
  T Average(const AABB3D& range) const;
  ///Resamples the given volume grid onto the current grid, taking trilinear interpolation at cell centers
  void ResampleTrilinear(const MyT& grid);
  ///Resamples the given volume grid onto the current grid, taking averages over grid cells
  void ResampleAverage(const MyT& grid);
  ///Returns the gradient estimated using forward differencing
  void Gradient_ForwardDifference(const IntTriple& index,Vector3& grad) const;
  ///Returns the gradient estimated using centered differencing
  void Gradient_CenteredDifference(const IntTriple& index,Vector3& grad) const;
  ///Returns the trilinear interpolated gradient.  If any element of pt is on a cell boundary the gradient estimated
  ///using centered differencing
  void Gradient(const Vector3& pt,Vector3& grad) const;
  void Add(const MyT& grid);
  void Subtract(const MyT& grid);
  void Multiply(const MyT& grid);
  void Max(const MyT& grid);
  void Min(const MyT& grid);
  void Add(T val);
  void Multiply(T val);
  void Max(T val);
  void Min(T val);

  typedef VolumeGridIterator<T> iterator;
  iterator getIterator() const { return iterator(value,bb); }

  Array3D<T> value;
  AABB3D bb;
};

typedef VolumeGridTemplate<Real> VolumeGrid;

template <class T>
std::istream& operator >> (std::istream& in,VolumeGridTemplate<T>& grid)
{
  in>>grid.bb.bmin>>grid.bb.bmax;
  in>>grid.value;
  return in;
}

template <class T>
std::ostream& operator << (std::ostream& out,const VolumeGridTemplate<T>& grid)
{
  out<<grid.bb.bmin<<"    "<<grid.bb.bmax<<std::endl;
  out<<grid.value<<std::endl;
  return out;
}


template <class T>
template <class T2>
void VolumeGridTemplate<T>::MakeSimilar(const VolumeGridTemplate<T2>& grid)
{
  bb=grid.bb;
  value.resize(grid.value.m,grid.value.n,grid.value.p);
}

template <class T>
template <class T2>
bool VolumeGridTemplate<T>::IsSimilar(const VolumeGridTemplate<T2>& grid) const
{
  return (grid.value.m == value.m && grid.value.n == value.n && grid.value.p == value.p) && (bb.bmin == grid.bb.bmin && bb.bmax == grid.bb.bmax);
}

template <class T>
VolumeGridIterator<T>::VolumeGridIterator(const Array3D<T>& _cells,const AABB3D& _bb)
  :cells(_cells),bb(_bb),it(&cells)
{
  cellSize.x = (bb.bmax.x-bb.bmin.x)/Real(cells.m);
  cellSize.y = (bb.bmax.y-bb.bmin.y)/Real(cells.n);
  cellSize.z = (bb.bmax.z-bb.bmin.z)/Real(cells.p);
  lo.set(0,0,0);
  hi.set(_cells.m-1,_cells.n-1,_cells.p-1);
  index=lo;
  cellCorner = bbMin = bb.bmin;
}

template <class T>
void VolumeGridIterator<T>::setRange(const IntTriple& bmin,const IntTriple& bmax)
{
  lo=bmin;
  hi=bmax;
  bbMin = bb.bmin;
  bbMin.x += Real(lo.a)*cellSize.x;
  bbMin.y += Real(lo.b)*cellSize.y;
  bbMin.z += Real(lo.c)*cellSize.z;
  cellCorner = bbMin;
  it = cells.begin(Range3Indices(lo.a,hi.a+1,
				 lo.b,hi.b+1,
				 lo.c,hi.c+1));
  index=lo;
}

template <class T>
void VolumeGridIterator<T>::operator ++()
{
  ++it;
  index.c++;
  cellCorner.z+=cellSize.z;
  if(index.c > hi.c) {
    index.c=lo.c;
    cellCorner.z=bbMin.z;
    index.b++;
    cellCorner.y+=cellSize.y;
    if(index.b > hi.b) {
      index.b=lo.b;
      cellCorner.y=bbMin.y;
      index.a++;
      cellCorner.x+=cellSize.x;
    }
  }
  Assert(index == it.getElement());
}


} //namespace Meshing

#endif

