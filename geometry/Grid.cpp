#include "Grid.h"
#include <math/cast.h>
#include <utils/indexing.h>
#include <errors.h>
using namespace Geometry;
using namespace std;

typedef Grid::Index Index;
typedef Grid::QueryCallback QueryCallback;

Grid::Grid(int numDims,Real _h)
  :h(numDims,_h)
{}

Grid::Grid(const Vector& _h)
  :h(_h)
{}

void Grid::PointToIndex(const Vector& p,Index& i) const
{
  Assert(p.n == h.n);
  i.resize(p.n);
  for(int k=0;k<p.n;k++) {
    Assert(h(k) > 0);
    i[k] = iFloor(p(k)/h(k));
  }
}

void Grid::PointToIndex(const Vector& p,Index& i,Vector& u) const
{
  Assert(p.n == h.n);
  i.resize(p.n);
  u.resize(p.n);
  for(int k=0;k<p.n;k++) {
    Assert(h(k) > 0);
    Real f=Floor(p(k)/h(k));
    u(k) = p(k)-f;
    i[k] = (int)f;
  }
}

void Grid::CellBounds(const Index& i,Vector& bmin,Vector& bmax) const
{
  Assert((int)i.size() == h.n);
  bmin.resize(h.n);
  bmax.resize(h.n);
  for(int k=0;k<h.n;k++) {
    bmin(k) = h(k)*(Real)i[k];
    bmax(k) = bmin(k) + h(k);
  }
}

void Grid::CellCorner(const Index& i,Vector& bmin) const
{
  Assert((int)i.size() == h.n);
  bmin.resize(h.n);
  for(int k=0;k<h.n;k++) {
    bmin(k) = h(k)*(Real)i[k];
  }
}

void Grid::CellCenter(const Index& i,Vector& c) const
{
  Assert((int)i.size() == h.n);
  c.resize(h.n);
  for(int k=0;k<h.n;k++) {
    c(k) = h(k)*((Real)i[k]+0.5);
  }
}

bool Grid::IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const
{
  Assert(h.n == (int)imin.size());
  Assert(h.n == (int)imax.size());
  for(size_t k=0;k<imin.size();k++)
    Assert(imin[k] <= imax[k]);

  Index i=imin;
  for(;;) {
    if(!f(i)) return false;
    if(IncrementIndex(i,imin,imax)!=0) return true;
  }
  return true;
}

bool Grid::BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const
{
  Index imin,imax;
  PointToIndex(bmin,imin);
  PointToIndex(bmax,imax);
  return IndexQuery(imin,imax,f);
}

bool Grid::BallQuery(const Vector& c,Real r,QueryCallback f) const
{
  //TODO: crop out boxes not intersected by sphere?
  Index imin,imax;
  Vector u;
  PointToIndex(c,imin,u);
  for(int k=0;k<c.n;k++) {
    int ik=imin[k];
    Real r_over_h = r/h(k);
    imin[k] = ik - iFloor(u(k)-r_over_h);
    imax[k] = ik + iFloor(u(k)+r_over_h);
  }
  return IndexQuery(imin,imax,f);
}

//TODO: do this with a DDA algorithm
//bool Grid::SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;

