#ifndef MESHING_RASTERIZE_H
#define MESHING_RASTERIZE_H

#include <KrisLibrary/structs/array2d.h>
#include <KrisLibrary/math3d/Triangle2D.h>
#include <KrisLibrary/math3d/AABB2D.h>
#include <KrisLibrary/math3d/Polygon2D.h>
#include <KrisLibrary/utils/IntPair.h>
#include <vector>

/** @file meshing/Rasterize.h
 * @ingroup Meshing
 * @brief 2D rasterization routines.
 */

namespace Meshing {

  using namespace Math3D;

  /** @addtogroup Meshing */
  /*@{*/

/// @brief Returns a list of cells that the segment overlaps, given
/// an infinite unit grid.
void GetSegmentCells(const Segment2D& tri,std::vector<IntPair>& cells);

/// @brief Returns a list of cells that the triangle overlaps, given
/// an infinite unit grid.
void GetTriangleCells(const Triangle2D& tri,std::vector<IntPair>& cells);

/// @brief Returns a list of cells that the triangle overlaps, in
/// a unit grid from [imin,imax)x[jmin,jmax)
void GetTriangleCells_Clipped(const Triangle2D& tri,std::vector<IntPair>& cells,int imin,int jmin,int imax,int jmax);


/** @brief A base class that allows rasterizing of 2D triangles into a grid.
 *
 * The triangle is assumed to be already in grid coordinates.  Each
 * cell (i,j) is filled if the point (i,j) is contained within the triangle.
 * More precisely, it is filled if (i+eps,j+eps) is contained within the
 * triangle for an infinitesmal perturbation eps.
 *
 * The triangle is drawn no matter what its orientation (ccw or cw).
 *
 * VisitCell() is an abstract method that should be overloaded by subclassing.
 */
struct Rasterizer2D
{
  virtual ~Rasterizer2D() {}
  void Rasterize(const Triangle2D& t);
  void ClippedRasterize(const Triangle2D& t,const AABB2D& aabb);
  void Rasterize(const AABB2D& b);
  void ClippedRasterize(const AABB2D& t,const AABB2D& aabb);

  //helpers
  void Rasterize(const Triangle2D& t,const Vector3& baryA,const Vector3& baryB,const Vector3& baryC);
  //Rasterizes the segment with x coordinate i, y coordinates [y1,y2]
  void RasterizeVerticalSegment(int i,Real y1,Real y2,const Vector3& baryA,const Vector3& baryB);

  //fill a point
  //params = barycentric coords of triangle, or (u,v) parameters of rect
  virtual void VisitCell(const Vector3& params,int i,int j)=0;
};

/// A rasterizer that flat-fills elements of a grid
template <class T>
struct FillRasterizer2D : public Rasterizer2D
{
  FillRasterizer2D() : grid(NULL) {}
  FillRasterizer2D(Array2D<T>& _grid) : grid(&_grid) {}
  virtual ~FillRasterizer2D() {}

  inline void Rasterize(const Triangle2D& tri) { Rasterizer2D::Rasterize(tri); }

  inline void Rasterize(const Triangle2D& tri,const T& val) {
    fillVal = val;
    Rasterizer2D::Rasterize(tri);
  }

  inline void Rasterize(const AABB2D& b,const T& val) {
    fillVal = val;
    Rasterizer2D::Rasterize(b);
  }

  virtual void VisitCell(const Vector3& bary,int i,int j)
  {
    if(grid&& i>=0 && j>=0 && i<grid->m && j<grid->n) {
      Fill(bary,(*grid)(i,j)); 
    }
  }

  virtual void Fill(const Vector3& params,T&cell) { cell = fillVal; }

  //the destination grid
  Array2D<T>* grid;
  //set this to the desired fill value
  T fillVal;
};

/// A smooth-fill rasterizer
template <class T>
struct SmoothFillRasterizer2D : public FillRasterizer2D<T>
{
  inline void Rasterize(const Triangle2D& tri) { FillRasterizer2D<T>::Rasterize(tri); }

  inline void Rasterize(const Triangle2D& tri,const T& flatColor) {
    fillA = fillB = fillC = flatColor;
    Rasterizer2D::Rasterize(tri);
  }

  inline void Rasterize(const Triangle2D& tri,const T& a,const T& b,const T& c) {
    fillA = a;
    fillB = b; 
    fillC = c;
    Rasterizer2D::Rasterize(tri);
  }

  virtual void Fill(const Vector3& bary,T& cell) { cell = bary.x*fillA+bary.y*fillB+bary.z*fillC; }

  //set these to the desired values at the corners of the triangle
  T fillA,fillB,fillC;
};


/*@}*/

} //namespace Meshing

#endif
