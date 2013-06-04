#include "Grid2DCSpace.h"
#include <math/random.h>
#include <math/sample.h>
#include <meshing/Rasterize.h>
#include <GLdraw/GL.h>
#include "EdgePlanner.h"

Grid2DCSpace::Grid2DCSpace(int m,int n)
{
  euclideanSpace=true;
  domain.bmin.set(0,0);
  domain.bmax.set(1,1);
  occupied.resize(m,n,false);
}

void Grid2DCSpace::WorldToGrid(const Vector2& world,Vector2& grid)
{
  grid.x = (world.x-domain.bmin.x)/(domain.bmax.x-domain.bmin.x);
  grid.y = (world.y-domain.bmin.y)/(domain.bmax.y-domain.bmin.y);
  grid.x *= (occupied.m);
  grid.y *= (occupied.n);
}

void Grid2DCSpace::GridToWorld(const Vector2& grid,Vector2& world)
{
  world.x = domain.bmin.x + (grid.x/Real(occupied.m))*(domain.bmax.x-domain.bmin.x);
  world.y = domain.bmin.y + (grid.y/Real(occupied.n))*(domain.bmax.y-domain.bmin.y);
}

void Grid2DCSpace::Add(const Triangle2D& tri,bool obstacle)
{
  Triangle2D locTri;
  WorldToGrid(tri.a,locTri.a);
  WorldToGrid(tri.b,locTri.b);
  WorldToGrid(tri.c,locTri.c);
  
  Meshing::FillRasterizer2D<bool> filler(occupied);
  filler.Rasterize(locTri,obstacle);
}

void Grid2DCSpace::Add(const AABB2D& bbox,bool obstacle)
{
  AABB2D locBB;
  WorldToGrid(bbox.bmin,locBB.bmin);
  WorldToGrid(bbox.bmax,locBB.bmax);
  
  Meshing::FillRasterizer2D<bool> filler(occupied);
  filler.Rasterize(locBB,obstacle);
}

//void Grid2DCSpace::Add(const Polygon2D& poly,bool obstacle);

void Grid2DCSpace::Add(const Circle2D& sphere,bool obstacle)
{
  Real xscale = (domain.bmax.x-domain.bmin.x)/Real(occupied.m);
  Real yscale = (domain.bmax.y-domain.bmin.y)/Real(occupied.n);
  Real x=domain.bmin.x + 0.5*xscale;
  for(int i=0;i<occupied.m;i++) {
    Real y = domain.bmin.y + 0.5*yscale;
    for(int j=0;j<occupied.n;j++) {
      if(sphere.contains(Vector2(x,y))) occupied(i,j)=obstacle;
      y += yscale;
    }
    x += xscale;
  }
}

void Grid2DCSpace::Rasterize(CSpace* space)
{
  Vector q(2);
  Real xscale = (domain.bmax.x-domain.bmin.x)/Real(occupied.m);
  Real yscale = (domain.bmax.y-domain.bmin.y)/Real(occupied.n);
  Real x=domain.bmin.x + 0.5*xscale;
  for(int i=0;i<occupied.m;i++) {
    Real y = domain.bmin.y + 0.5*yscale;
    for(int j=0;j<occupied.n;j++) {
      q(0)=x;
      q(1)=y;
      occupied(i,j) = !space->IsFeasible(q);
      y += yscale;
    }
    x += xscale;
  }
}

void Grid2DCSpace::DrawGL()
{
  Real xscale = (domain.bmax.x-domain.bmin.x)/Real(occupied.m);
  Real yscale = (domain.bmax.y-domain.bmin.y)/Real(occupied.n);
  //blank out background (light yellow)
  glColor3f(1,1,0.5);
  glBegin(GL_QUADS);
  glVertex2f(domain.bmin.x,domain.bmin.y);
  glVertex2f(domain.bmax.x,domain.bmin.y);
  glVertex2f(domain.bmax.x,domain.bmax.y);
  glVertex2f(domain.bmin.x,domain.bmax.y);
  glEnd();
  //draw obstacles (dark grey)
  glColor3f(0.2,0.2,0.2);
  glBegin(GL_QUADS);
  Real x=domain.bmin.x;
  for(int i=0;i<occupied.m;i++) {
    Real y = domain.bmin.y;
    for(int j=0;j<occupied.n;j++) {
      if(occupied(i,j)) {
	//draw cell
	glVertex2f(x,y);
	glVertex2f(x+xscale,y);
	glVertex2f(x+xscale,y+yscale);
	glVertex2f(x,y+yscale);
      }
      y += yscale;
    }
    x += xscale;
  }
  glEnd();
}

void Grid2DCSpace::Sample(Config& x)
{
  x.resize(2);
  x(0)=Rand(domain.bmin.x,domain.bmax.x);
  x(1)=Rand(domain.bmin.y,domain.bmax.y);
}

void Grid2DCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(2);
  if(euclideanSpace) SampleDisk(r,x(0),x(1));
  else { x(0)=Rand(-r,r); x(1)=Rand(-r,r); }
  x += c;
}

bool Grid2DCSpace::IsFeasible(const Config& x)
{
  Vector2 scr;
  WorldToGrid(Vector2(x(0),x(1)),scr);
  int i=(int)Floor(scr.x);
  int j=(int)Floor(scr.y);
  if(i<0 || i>=occupied.m || j<0 || j>=occupied.n) return false;
  return !occupied(i,j);
}

EdgePlanner* Grid2DCSpace::LocalPlanner(const Config& a,const Config& b)
{
  Real res = Min((domain.bmax.x-domain.bmin.x)/Real(occupied.m),
		 (domain.bmax.y-domain.bmin.y)/Real(occupied.n));
  return new BisectionEpsilonEdgePlanner(this,a,b,res);
}

Real Grid2DCSpace::Distance(const Config& x, const Config& y)
{ 
  if(euclideanSpace) return Distance_L2(x,y);
  else return Distance_LInf(x,y);
}
