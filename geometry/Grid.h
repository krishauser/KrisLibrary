#ifndef GEOMETRY_GRID_H
#define GEOMETRY_GRID_H

#include <KrisLibrary/utils/IntTuple.h>
#include <KrisLibrary/math/vector.h>

namespace Geometry {

using namespace Math;

/** @ingroup Geometry
 * @brief A gridding of n-dimensional space.
 *
 * Each dimension k is divided into uniform divisions of size h(k).
 * A cell of the grid is indexed by a Grid::Index, a vector of ints.
 * The grid cell at 0 is (0,0,...,0), and each increment in a dimension k
 * shifts the cell by h(k) units.
 */
class Grid
{
public:
  typedef IntTuple Index;
  /// Called for each cell in the query, return false to stop enumerating
  typedef bool (*QueryCallback)(const Index& index);

  explicit Grid(int numDims,Real h=1);
  explicit Grid(const Vector& h);

  //returns the index of the point
  void PointToIndex(const Vector& p,Index& i) const;
  //same, but with the local coordinates in the cell [0,1]^n
  void PointToIndex(const Vector& p,Index& i,Vector& u) const;
  //returns the lower/upper corner of the cell
  void CellBounds(const Index& i,Vector& bmin,Vector& bmax) const;
  //returns the lower corner of the cell
  void CellCorner(const Index& index,Vector& bmin) const;
  //returns the center of the cell
  void CellCenter(const Index& index,Vector& c) const;

  //range imin to imax
  bool IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const;
  //bounding box from bmin to bmax
  bool BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const;
  //ball with center c, radius r
  bool BallQuery(const Vector& c,Real r,QueryCallback f) const;
  //segment from a to b
  bool SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;

  Vector h;
};

} //namespace Geometry

#endif
