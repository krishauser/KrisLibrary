#include "Grid2DCSpace.h"
#include <math/random.h>
#include <math/sample.h>
#include <meshing/Rasterize.h>
#include <GLdraw/GL.h>
#include "EdgePlanner.h"
using namespace std;

Grid2DCSpace::Grid2DCSpace(int m,int n,const AABB2D& _domain)
:BoxCSpace(Vector(2,_domain.bmin),Vector(2,_domain.bmax))
{
  euclideanSpace=true;
  occupied = make_shared<Tabular2DSet>(m,n,_domain.bmin,_domain.bmax);
  AddConstraint("occupied",occupied.get());
}

void Grid2DCSpace::WorldToGrid(const Vector2& world,Vector2& grid)
{
  grid.x = (world.x-occupied->bb.bmin.x)/(occupied->bb.bmax.x-occupied->bb.bmin.x);
  grid.y = (world.y-occupied->bb.bmin.y)/(occupied->bb.bmax.y-occupied->bb.bmin.y);
  grid.x *= (occupied->value.m);
  grid.y *= (occupied->value.n);
}

void Grid2DCSpace::GridToWorld(const Vector2& grid,Vector2& world)
{
  world.x = occupied->bb.bmin.x + (grid.x/Real(occupied->value.m))*(occupied->bb.bmax.x-occupied->bb.bmin.x);
  world.y = occupied->bb.bmin.y + (grid.y/Real(occupied->value.n))*(occupied->bb.bmax.y-occupied->bb.bmin.y);
}

void Grid2DCSpace::Add(const Triangle2D& tri,bool obstacle)
{
  Triangle2D locTri;
  WorldToGrid(tri.a,locTri.a);
  WorldToGrid(tri.b,locTri.b);
  WorldToGrid(tri.c,locTri.c);
  
  Meshing::FillRasterizer2D<bool> filler(occupied->value);
  filler.Rasterize(locTri,obstacle);
}

void Grid2DCSpace::Add(const AABB2D& bbox,bool obstacle)
{
  AABB2D locBB;
  WorldToGrid(bbox.bmin,locBB.bmin);
  WorldToGrid(bbox.bmax,locBB.bmax);
  
  Meshing::FillRasterizer2D<bool> filler(occupied->value);
  filler.Rasterize(locBB,obstacle);
}

//void Grid2DCSpace::Add(const Polygon2D& poly,bool obstacle);

void Grid2DCSpace::Add(const Circle2D& sphere,bool obstacle)
{
  Real xscale = (occupied->bb.bmax.x-occupied->bb.bmin.x)/Real(occupied->value.m);
  Real yscale = (occupied->bb.bmax.y-occupied->bb.bmin.y)/Real(occupied->value.n);
  Real x=occupied->bb.bmin.x + 0.5*xscale;
  for(int i=0;i<occupied->value.m;i++) {
    Real y = occupied->bb.bmin.y + 0.5*yscale;
    for(int j=0;j<occupied->value.n;j++) {
      if(sphere.contains(Vector2(x,y))) occupied->value(i,j)=obstacle;
      y += yscale;
    }
    x += xscale;
  }
}

void Grid2DCSpace::Rasterize(CSpace* space)
{
  Vector q(2);
  Real xscale = (occupied->bb.bmax.x-occupied->bb.bmin.x)/Real(occupied->value.m);
  Real yscale = (occupied->bb.bmax.y-occupied->bb.bmin.y)/Real(occupied->value.n);
  Real x=occupied->bb.bmin.x + 0.5*xscale;
  for(int i=0;i<occupied->value.m;i++) {
    Real y = occupied->bb.bmin.y + 0.5*yscale;
    for(int j=0;j<occupied->value.n;j++) {
      q(0)=x;
      q(1)=y;
      occupied->value(i,j) = !space->IsFeasible(q);
      y += yscale;
    }
    x += xscale;
  }
}

void Grid2DCSpace::DrawGL()
{
  Real xscale = (occupied->bb.bmax.x-occupied->bb.bmin.x)/Real(occupied->value.m);
  Real yscale = (occupied->bb.bmax.y-occupied->bb.bmin.y)/Real(occupied->value.n);
  //blank out background (light yellow)
  glColor3f(1.f,1.f,0.5f);
  glBegin(GL_QUADS);
  glVertex2d(occupied->bb.bmin.x,occupied->bb.bmin.y);
  glVertex2d(occupied->bb.bmax.x,occupied->bb.bmin.y);
  glVertex2d(occupied->bb.bmax.x,occupied->bb.bmax.y);
  glVertex2d(occupied->bb.bmin.x,occupied->bb.bmax.y);
  glEnd();
  //draw obstacles (dark grey)
  glColor3f(0.2f,0.2f,0.2f);
  glBegin(GL_QUADS);
  Real x=occupied->bb.bmin.x;
  for(int i=0;i<occupied->value.m;i++) {
    Real y = occupied->bb.bmin.y;
    for(int j=0;j<occupied->value.n;j++) {
      if(occupied->value(i,j)) {
	//draw cell
	glVertex2d(x,y);
	glVertex2d(x+xscale,y);
	glVertex2d(x+xscale,y+yscale);
	glVertex2d(x,y+yscale);
      }
      y += yscale;
    }
    x += xscale;
  }
  glEnd();
}


void Grid2DCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(2);
  if(euclideanSpace) SampleDisk(r,x(0),x(1));
  else { x(0)=Rand(-r,r); x(1)=Rand(-r,r); }
  x += c;
}

EdgePlannerPtr Grid2DCSpace::PathChecker(const InterpolatorPtr& path)
{
  Real res = Min((occupied->bb.bmax.x-occupied->bb.bmin.x)/Real(occupied->value.m),
		 (occupied->bb.bmax.y-occupied->bb.bmin.y)/Real(occupied->value.n));
  return std::make_shared<EpsilonEdgeChecker>(this,path,res);
}

Real Grid2DCSpace::Distance(const Config& x, const Config& y)
{ 
  if(euclideanSpace) return Distance_L2(x,y);
  else return Distance_LInf(x,y);
}
