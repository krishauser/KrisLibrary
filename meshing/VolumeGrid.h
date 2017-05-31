#ifndef VOLUME_GRID_H
#define VOLUME_GRID_H

#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/structs/array3d.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/indexing.h>
#include <iosfwd>

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
 * @brief A 3D array over a 3D volume, containing Real values.
 *
 * GetIndex returns the cell containing a 3D point.
 * GetIndexAndParams returns the cell containing a 3D point, and its
 * coordinates within that cell.
 *
 * Cells can be quickly iterated over using the iterator class.
 * 
 * Values are interpreted so they are defined over an entire cell,
 * *not* the vertices.
 */
class VolumeGrid
{
 public:
  bool IsEmpty() const { return value.empty(); }
  void Resize(int m,int n,int p) { value.resize(m,n,p); }
  void ResizeByResolution(const Vector3& res);
  void MakeSimilar(const VolumeGrid& grid);
  bool IsSimilar(const VolumeGrid& grid) const;
  void GetCell(int i,int j,int k,AABB3D& cell) const;
  void GetCellCenter(int i,int j,int k,Vector3& center) const;
  Vector3 GetCellSize() const;
  void GetIndex(const Vector3& pt,int& i,int& j,int& k) const;
  void GetIndexAndParams(const Vector3& pt,IntTriple& index,Vector3& params) const;
  void GetIndexRange(const AABB3D& range,IntTriple& imin,IntTriple& imax) const;
  inline void GetCell(const IntTriple& index,AABB3D& cell) const { GetCell(index.a,index.b,index.c,cell); }
  inline void GetCenter(const IntTriple& index,Vector3& center) const { GetCellCenter(index.a,index.b,index.c,center); }
  inline void GetIndex(const Vector3& pt,IntTriple& index) const { GetIndex(pt,index.a,index.b,index.c); }

  ///Computes the trilinear interpolation of the field at pt, assuming values are sampled exactly at cell centers
  Real TrilinearInterpolate(const Vector3& pt) const;
  ///Used only for fast marching method, really.
  Real MinimumFreeInterpolate(const Vector3& pt) const;
  ///Average value of the range.  Each cell's value is weighted by the volume overlap with range
  Real Average(const AABB3D& range) const;
  ///Resamples the given volume grid onto the current grid, taking trilinear interpolation at cell centers
  void ResampleTrilinear(const VolumeGrid& grid);
  ///Resamples the given volume grid onto the current grid, taking averages over grid cells
  void ResampleAverage(const VolumeGrid& grid);
  void Add(const VolumeGrid& grid);
  void Subtract(const VolumeGrid& grid);
  void Multiply(const VolumeGrid& grid);
  void Max(const VolumeGrid& grid);
  void Min(const VolumeGrid& grid);
  void Add(Real val);
  void Multiply(Real val);
  void Max(Real val);
  void Min(Real val);

  typedef VolumeGridIterator<Real> iterator;
  iterator getIterator() const { return iterator(value,bb); }

  Array3D<Real> value;
  AABB3D bb;
};

std::istream& operator >> (std::istream& in,VolumeGrid& grid);
std::ostream& operator << (std::ostream& out,const VolumeGrid& grid);


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
  /*
  if(index != it.getElement()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"VolumeGridIterator: Internal error!"<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Index "<<index<<", iterator element "<<it.getElement()<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Low = "<<lo<<", high = "<<hi<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Array3d iterator element "<<it.it.getElement()<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Array3D size "<<cells.size()<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Array3d range base "<<it.range.base<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"  size "<<it.range.isize<<" "<<it.range.jsize<<" "<<it.range.ksize<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"  stride "<<it.range.istride<<" "<<it.range.jstride<<" "<<it.range.kstride<<"\n");
  }
  if(index.c + cells.p*(index.b + cells.n*index.a) != *it.it) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"VolumeGridIterator: Internal error!"<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Offset into array: "<<index.c + cells.p*(index.b + cells.n*index.a)<<", stripe iterator "<<*it.it<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"iterator element "<<it.it.getElement()<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Array3D size "<<cells.size()<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Array3d range base "<<it.range.base<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"  size "<<it.range.isize<<" "<<it.range.jsize<<" "<<it.range.ksize<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"  stride "<<it.range.istride<<" "<<it.range.jstride<<" "<<it.range.kstride<<"\n");
  }
  */
  Assert(index == it.getElement());
}


} //namespace Meshing

#endif

