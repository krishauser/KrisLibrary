#ifndef GRID_2D_CSPACE_H
#define GRID_2D_CSPACE_H

#include "CSpace.h"
#include <KrisLibrary/math3d/Circle2D.h>
#include <KrisLibrary/math3d/Triangle2D.h>
#include <KrisLibrary/math3d/AABB2D.h>
#include <KrisLibrary/structs/array2d.h>
using namespace Math3D;

class Grid2DCSpace : public CSpace
{
public:
  Grid2DCSpace(int m,int n);
  void WorldToGrid(const Vector2& world,Vector2& grid);
  void GridToWorld(const Vector2& grid,Vector2& world);
  void Add(const Triangle2D& tri,bool obstacle=true);
  void Add(const AABB2D& bbox,bool obstacle=true);
  //void Add(const Polygon2D& poly,bool obstacle=true);
  void Add(const Circle2D& sphere,bool obstacle=true);
  void Rasterize(CSpace* space);
  void DrawGL();

  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlanner* PathChecker(const SmartPointer<Interpolator>& b);
  virtual Real Distance(const Config& x, const Config& y);

  bool euclideanSpace;
  AABB2D domain;
  Array2D<bool> occupied;
};

#endif
