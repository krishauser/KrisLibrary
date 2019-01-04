#include <KrisLibrary/Logger.h>
#include "CSetHelpers.h"
#include "CSet.h"
#include "EdgePlanner.h"
#include <math/AABB.h>
#include <math/random.h>
using namespace std;


BoxSet::BoxSet(Real xmin,Real xmax,int d)
:bmin(d,xmin),bmax(d,xmax)
{}

BoxSet::BoxSet(const Vector& _bmin,const Vector& _bmax)
:bmin(_bmin),bmax(_bmax)
{
}

int BoxSet::NumDimensions() const
{
  return bmin.n;
}

void BoxSet::Sample(Config& x) 
{
  x.resize(bmin.n);
  for(int i=0;i<bmin.n;i++)
    x(i) = Rand(bmin[i],bmax[i]);
}

bool BoxSet::Contains(const Config& x)
{
  return AABBContains(x,bmin,bmax);
}

bool BoxSet::Project(Config& x)
{
  AABBClamp(x,bmin,bmax);
  return true;
}

Real BoxSet::ObstacleDistance(const Vector& x)
{
  Real dmin = Inf;
  for(int i=0;i<x.n;i++) {
    dmin = Min(dmin,x(i)-bmin(i));
    dmin = Min(dmin,bmax(i)-x(i));
  }
  return dmin;
}

Optimization::NonlinearProgram* BoxSet::Numeric()
{
  return NULL;
}

AxisRangeSet::AxisRangeSet(int _i,Real xmin,Real xmax)
:i(_i),low(xmin),high(xmax)
{}
int AxisRangeSet::NumDimensions() const { return -1; }
bool AxisRangeSet::Contains(const Config& x) { return x(i) >= low && x(i) <= high; }
bool AxisRangeSet::Project(Config& x) { 
  x[i] = Clamp(x[i],low,high);
  return true;
}
Real AxisRangeSet::ObstacleDistance(const Config& x)
{
  return Max(x[i]-low,high-x[i]);
}
Optimization::NonlinearProgram* AxisRangeSet::Numeric()
{
  return NULL;
}


NeighborhoodSet::NeighborhoodSet(CSpace* _space,const Config& _center,Real _radius)
  :space(_space),center(_center),radius(_radius)
{}

int NeighborhoodSet::NumDimensions() const { return center.n; }

void NeighborhoodSet::Sample(Config& x)
{
  space->SampleNeighborhood(center,radius,x);
}
bool NeighborhoodSet::Contains(const Config& x)
{
  return (space->Distance(x,center) <= radius); 
}

bool NeighborhoodSet::Project(Config& x)
{
  Real d = space->Distance(x,center);
  if(d < radius) return true;
  Vector temp;
  space->Interpolate(center,x,radius/d,temp);
  x = temp;
  return true;
}

Real NeighborhoodSet::ObstacleDistance(const Config& x)
{
  Real d = space->Distance(x,center);
  return radius-d;
}

Optimization::NonlinearProgram* NeighborhoodSet::Numeric()
{
  return NULL;
}


SubspaceSet::SubspaceSet(const shared_ptr<CSet>& _base,int _imin,int _imax)
:base(_base),imin(_imin),imax(_imax)
{}

bool SubspaceSet::Contains(const Config& x)
{
  Assert(imin >= 0 && imax >= 0);
  Assert(imin < x.n && imax <= x.n);
  Vector xsub;
  xsub.setRef(x,imin,1,imax-imin);
  return base->Contains(xsub);
}

bool SubspaceSet::Project(Config& x)
{
  Assert(imin >= 0 && imax >= 0);
  Assert(imin < x.n && imax <= x.n);
  Vector xsub;
  xsub.setRef(x,imin,1,imax-imin);
  return base->Project(xsub);
}

Real SubspaceSet::ObstacleDistance(const Config& x)
{
  Assert(imin >= 0 && imax >= 0);
  Assert(imin < x.n && imax <= x.n);
  Vector xsub;
  xsub.setRef(x,imin,1,imax-imin);
  return base->ObstacleDistance(xsub);
}

Optimization::NonlinearProgram* SubspaceSet::Numeric()
{
  return NULL;
}

VisibilitySet::VisibilitySet(CSpace* _space,const Config& _center)
  :space(_space),center(_center)
{}


int VisibilitySet::NumDimensions() const
{
  return space->NumDimensions();
}

bool VisibilitySet::Contains(const Config& x)
{ 
  EdgePlannerPtr e=space->LocalPlanner(x,center);
  return e->IsVisible();
}



FiniteSet::FiniteSet() {}
FiniteSet::FiniteSet(const Vector& item):items(1,item) {}
FiniteSet::FiniteSet(const Vector& item1,const Vector& item2) { items.resize(2); items[0] = item1; items[1] = item2; }
FiniteSet::FiniteSet(const Vector& item1,const Vector& item2,const Vector& item3) { items.resize(3); items[0] = item1; items[1] = item2; items[2] = item3; }
FiniteSet::FiniteSet(const std::vector<Vector>& _items) :items(_items) {}
void FiniteSet::Sample(Config& q)
{
  if(items.empty()) return;
  q = items[RandInt(items.size())];
}
bool FiniteSet::Contains(const Config& x) {
  for(size_t i=0;i<items.size();i++)
    if(x == items[i]) return true;
  return false;
}
bool FiniteSet::Project(Config& x)
{
  Real dmin = Inf;
  int index = -1;
  for(size_t i=0;i<items.size();i++) {
    Real d = x.distance(items[i]);
    if(d < dmin) { dmin=d; index=(int)i; }
  }
  if(index < 0) return false;
  x = items[index];
  return true;
}

int FiniteSet::NumDimensions() const
{
  if(items.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"FiniteSet::NumDimensions(): no items, returning -1\n");
    return -1;
  }
  return items[0].n;
}
Real FiniteSet::ObstacleDistance(const Config& x)
{
  if(Contains(x)) return 0;
  return -Inf;
}

Optimization::NonlinearProgram* FiniteSet::Numeric()
{
  return NULL;
}


UnionSet::UnionSet(const shared_ptr<CSet>& a,const shared_ptr<CSet>& b)
{
  items.resize(2);
  items[0] = a;
  items[1] = b;
}

UnionSet::UnionSet(const std::vector<shared_ptr<CSet> >& _items)
:items(_items)
{}

int UnionSet::NumDimensions() const
{
  return items[0]->NumDimensions();
}

bool UnionSet::Contains(const Config& x)
{
  for(size_t i=0;i<items.size();i++)
    if(items[i]->Contains(x)) return true;
  return false;
}

bool UnionSet::Project(Config& x)
{
  for(size_t i=0;i<items.size();i++)
    if(items[i]->Project(x)) return true;
  return false;
}

bool UnionSet::IsSampleable() const
{
  for(size_t i=0;i<items.size();i++)
    if(items[i]->IsSampleable()) return true;
  return false;
}

void UnionSet::Sample(Config& x)
{
  vector<int> sampleable;
  for(size_t i=0;i<items.size();i++)
    if(items[i]->IsSampleable()) sampleable.push_back(i);
  if(sampleable.empty()) return;
  int k = RandInt(sampleable.size());
  items[k]->Sample(x);
}


IntersectionSet::IntersectionSet(const shared_ptr<CSet>& a,const shared_ptr<CSet>& b)
{
  items.resize(2);
  items[0] = a;
  items[1] = b;
}

IntersectionSet::IntersectionSet(const std::vector<shared_ptr<CSet> >& _items)
:items(_items)
{}

int IntersectionSet::NumDimensions() const
{
  return items[0]->NumDimensions();
}

bool IntersectionSet::Contains(const Config& x)
{
  for(size_t i=0;i<items.size();i++)
    if(!items[i]->Contains(x)) return false;
  return true;
}

bool IntersectionSet::IsConvex() const
{
  for(size_t i=0;i<items.size();i++)
    if(!items[i]->IsConvex()) return false;
  return true;
}

Real IntersectionSet::ObstacleDistance(const Config& x)
{
  Real d = Inf;
  for(size_t i=0;i<items.size();i++)
    d = Min(d,items[i]->ObstacleDistance(x));
  return d;
}

Optimization::NonlinearProgram* IntersectionSet::Numeric()
{
  return NULL;
}

bool IntersectionSet::IsSampleable() const
{
  for(size_t i=0;i<items.size();i++)
    if(items[i]->IsSampleable()) return true;
  return false;
}

void IntersectionSet::Sample(Config& x)
{
  vector<int> sampleable;
  for(size_t i=0;i<items.size();i++)
    if(items[i]->IsSampleable()) sampleable.push_back(i);
  if(sampleable.empty()) return;
  int k = RandInt(sampleable.size());
  items[k]->Sample(x);
  for(size_t i=0;i<items.size();i++) if(!items[i]->Contains(x)) { x.resize(0); return; }
}
