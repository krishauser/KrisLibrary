#ifndef GRID_2D_CSPACE_H
#define GRID_2D_CSPACE_H

#include "CSpaceHelpers.h"
#include <KrisLibrary/math3d/Circle2D.h>
#include <KrisLibrary/math3d/Triangle2D.h>
#include <KrisLibrary/math3d/AABB2D.h>
#include "Tabular.h"
using namespace Math3D;

/** @brief A simple CSpace that lets you add geometric obstacles as
 * blocked off grid cells.
 *
 * Can use euclidean or Linf metric, configured by euclideanSpace.
 */
class Grid2DCSpace : public BoxCSpace
{
public:
  Grid2DCSpace(int m,int n,const AABB2D& domain);
  void WorldToGrid(const Vector2& world,Vector2& grid);
  void GridToWorld(const Vector2& grid,Vector2& world);
  void Add(const Triangle2D& tri,bool obstacle=true);
  void Add(const AABB2D& bbox,bool obstacle=true);
  //void Add(const Polygon2D& poly,bool obstacle=true);
  void Add(const Circle2D& sphere,bool obstacle=true);
  void Rasterize(CSpace* space);
  void DrawGL();

  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual EdgePlannerPtr PathChecker(const InterpolatorPtr& b);
  virtual Real Distance(const Config& x, const Config& y);
  
  bool euclideanSpace;
  std::shared_ptr<Tabular2DSet> occupied;
};

#endif
