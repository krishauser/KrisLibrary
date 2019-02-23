#include "Tabular.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/random.h>
#include <sstream>
using namespace std;

Tabular1DSet::Tabular1DSet(const std::vector<bool>& valid,Real a,Real b)
:BoxSet(a,b,1),Table1D<bool>(valid,a,b),calculatedValidIndices(false)
{}

Tabular1DSet::Tabular1DSet(int n,Real a,Real b)
:BoxSet(a,b,1),Table1D<bool>(n,a,b),calculatedValidIndices(false)
{}

bool Tabular1DSet::Contains(const Config& x) { return Lookup(x); }

bool Tabular1DSet::Project(Config& x)
{
  if(Contains(x)) return true;
  int i = Index(x);
  for(int d=1;d<(int)values.size();d++) {
    if(i-d >= 0 && values[i-d]) {
      x[0] = Cell(i-d).second;
      return true;
    }
    if(i+d < (int)values.size() && values[i+d]) {
      x[0] = Cell(i+d).first;
      return true;
    }
  }
  return false;
}

void Tabular1DSet::Sample(Config& x)
{
  if(!calculatedValidIndices) {
    for(size_t i=0;i<values.size();i++)
      if(values[i])
        validIndices.push_back(int(i));
    calculatedValidIndices = true;
  }
  if(validIndices.empty()) {
    x.resize(1,0.0);
    return;
  }
  int i = validIndices[RandInt(validIndices.size())];
  pair<Real,Real> c = Cell(i);
  x.resize(1);
  x[0] = Rand(c.first,c.second);
}

Tabular2DSet::Tabular2DSet(const Array2D<bool>& valid,const Vector2& bmin,const Vector2& bmax)
:BoxSet(Vector(2,bmin),Vector(2,bmax)),calculatedValidIndices(false)
{
  value = valid;
  bb.bmin = bmin;
  bb.bmax = bmax;
}

Tabular2DSet::Tabular2DSet(int m,int n,const Vector2& bmin,const Vector2& bmax)
:BoxSet(Vector(2,bmin),Vector(2,bmax)),calculatedValidIndices(false)
{
  value.resize(m,n,false);
  bb.bmin = bmin;
  bb.bmax = bmax;
}


bool Tabular2DSet::Contains(const Config& x)
{
  return GetValue(Vector2(x[0],x[1]));
}

bool Tabular2DSet::Project(Config& x)
{
  return BoxSet::Project(x) && Contains(x);
}

void Tabular2DSet::Sample(Config& x)
{
  if(!calculatedValidIndices) {
    for(int i=0;i<value.m;i++)
      for(int j=0;j<value.n;j++)
        if(value(i,j)) validIndices.push_back(IntPair(i,j));
    calculatedValidIndices = true;
  }
  if(validIndices.empty()) {
    x.resize(2,0.0);
    return;
  }
  IntPair i = validIndices[RandInt(validIndices.size())];
  AABB2D bb;
  GetCell(i,bb);
  Vector2 temp;
  SampleAABB(bb.bmin,bb.bmax,temp);
  x.resize(2);
  temp.get(x);
}

Tabular3DSet::Tabular3DSet(const Array3D<bool>& valid,const Vector3& bmin,const Vector3& bmax)
:BoxSet(Vector(3,bmin),Vector(3,bmax)),calculatedValidIndices(false)
{
  value = valid;
  bb.bmin = bmin;
  bb.bmax = bmax;
}


Tabular3DSet::Tabular3DSet(int m,int n,int p,const Vector3& bmin,const Vector3& bmax)
:BoxSet(Vector(3,bmin),Vector(3,bmax)),calculatedValidIndices(false)
{
  value.resize(m,n,p,false);
  bb.bmin = bmin;
  bb.bmax = bmax;
}
bool Tabular3DSet::Contains(const Config& x)
{
  return GetValue(Vector3(x[0],x[1],x[2]));
}

bool Tabular3DSet::Project(Config& x)
{
  return BoxSet::Project(x);
}

void Tabular3DSet::Sample(Config& x)
{
  if(!calculatedValidIndices) {
    for(int i=0;i<value.m;i++)
      for(int j=0;j<value.n;j++)
        for(int k=0;k<value.p;k++)
          if(value(i,j,k)) validIndices.push_back(IntTriple(i,j,k));
    calculatedValidIndices = true;
  }
  if(validIndices.empty()) {
    x.resize(3,0.0);
    return;
  }
  IntTriple i = validIndices[RandInt(validIndices.size())];
  AABB3D bb;
  GetCell(i,bb);
  Vector3 temp;
  SampleAABB(bb.bmin,bb.bmax,temp);
  x.resize(3);
  temp.get(x);
}

/** @ingroup Planning
 * @brief A 1D level set f(x) <= threshold.
 *
 * Values of f are linearly interpolated on the grid [a,b].  These are
 * interpreted as being located at the grid cell *centers*, not 
 * the vertices.
 */
Tabular1DLevelSet::Tabular1DLevelSet(const std::vector<Real>& f,Real a,Real b,Real _threshold)
:BoxSet(a,b,1),Table1D<Real>(f,a,b),threshold(_threshold)
{}

bool Tabular1DLevelSet::Contains(const Config& x)
{
  return Lookup(x) <= threshold;
}

bool Tabular1DLevelSet::Project(Config& x)
{
  return BoxSet::Project(x) && Contains(x);
}

Tabular2DLevelSet::Tabular2DLevelSet(const Array2D<Real>& f,const Vector2& bmin,const Vector2& bmax,Real _threshold)
:BoxSet(Vector(2,bmin),Vector(2,bmax))
{
  agrid.value = f;
  agrid.bb.bmin = bmin;
  agrid.bb.bmax = bmax;
}

bool Tabular2DLevelSet::Contains(const Config& x)
{
  return agrid.BilinearInterpolate(Vector2(x)) <= threshold;
}

bool Tabular2DLevelSet::Project(Config& x)
{
  return BoxSet::Project(x) && Contains(x);
}


Tabular3DLevelSet::Tabular3DLevelSet(const Array3D<Real>& f,const Vector3& bmin,const Vector3& bmax,Real _threshold)
:BoxSet(Vector(3,bmin),Vector(3,bmax)),threshold(_threshold)
{
  vgrid.value = f;
  vgrid.bb.bmin = bmin;
  vgrid.bb.bmax = bmax;
}

bool Tabular3DLevelSet::Contains(const Config& x)
{
  return vgrid.TrilinearInterpolate(Vector3(x)) <= threshold;
}

bool Tabular3DLevelSet::Project(Config& x)
{
  return BoxSet::Project(x) && Contains(x);
}


Tabular1DObjective::Tabular1DObjective(const std::vector<Real>& _differentialCosts,const std::vector<Real>& _terminalCosts,
    Real a,Real b,
    int _dim)
:differentialCosts(_differentialCosts,a,b),terminalCosts(_terminalCosts,a,b),dim(_dim)
{}

std::string Tabular1DObjective::Description()
{
  stringstream ss;
  ss<<"tabular1d ["<<differentialCosts.a<<","<<differentialCosts.b<<"] on dimension "<<dim;
  return ss.str();
}

Real Tabular1DObjective::IncrementalCost(const Interpolator* path)
{
  if(differentialCosts.values.empty()) return 0;
  Config x;
  path->Eval(0.5,x);
  return differentialCosts.Lookup(x[dim])*path->Length();
}

Real Tabular1DObjective::TerminalCost(const Vector& qend)
{
  if(terminalCosts.values.empty()) return 0;
  return terminalCosts.Lookup(qend[dim]);
}


Tabular2DObjective::Tabular2DObjective(const Array2D<Real>& _differentialCosts,const Array2D<Real>& _terminalCosts,
    const Vector2& bmin,const Vector2& bmax,
    int _dim0,int _dim1)
:dim0(_dim0),dim1(_dim1)
{
  differentialCosts.value = _differentialCosts;
  differentialCosts.bb.bmin = bmin;
  differentialCosts.bb.bmax = bmax;
  terminalCosts.value = _terminalCosts;
  terminalCosts.bb.bmin = bmin;
  terminalCosts.bb.bmax = bmax;
}

std::string Tabular2DObjective::Description()
{
  stringstream ss;
  ss<<"tabular2d ["<<differentialCosts.bb.bmin<<","<<differentialCosts.bb.bmin<<"] on dimensions ("<<dim0<<","<<dim1<<")";
  return ss.str();
}

Real Tabular2DObjective::IncrementalCost(const Interpolator* path)
{
  if(differentialCosts.value.empty()) return 0;
  Config x;
  path->Eval(0.5,x);
  return differentialCosts.BilinearInterpolate(Vector2(x[dim0],x[dim1]))*path->Length();
}

Real Tabular2DObjective::TerminalCost(const Vector& qend)
{
  if(terminalCosts.value.empty()) return 0;
  return terminalCosts.BilinearInterpolate(Vector2(qend[dim0],qend[dim1]));
}


Tabular3DObjective::Tabular3DObjective(const Array3D<Real>& _differentialCosts,const Array3D<Real>& _terminalCosts,
    const Vector3& bmin,const Vector3& bmax,
    int _dim0,int _dim1,int _dim2)
:dim0(_dim0),dim1(_dim1),dim2(_dim2)
{
  differentialCosts.value = _differentialCosts;
  differentialCosts.bb.bmin = bmin;
  differentialCosts.bb.bmax = bmax;
  terminalCosts.value = _terminalCosts;
  terminalCosts.bb.bmin = bmin;
  terminalCosts.bb.bmax = bmax;
}

std::string Tabular3DObjective::Description()
{
  stringstream ss;
  ss<<"tabular3d ["<<differentialCosts.bb.bmin<<","<<differentialCosts.bb.bmax<<"] on dimensions ("<<dim0<<","<<dim1<<","<<dim2<<")";
  return ss.str();
}

Real Tabular3DObjective::IncrementalCost(const Interpolator* path)
{
  if(differentialCosts.value.empty()) return 0;
  Config x;
  path->Eval(0.5,x);
  return differentialCosts.TrilinearInterpolate(Vector3(x[dim0],x[dim1],x[dim2]))*path->Length();
}

Real Tabular3DObjective::TerminalCost(const Vector& qend)
{
  if(terminalCosts.value.empty()) return 0;
  return terminalCosts.TrilinearInterpolate(Vector3(qend[dim0],qend[dim1],qend[dim2]));
}
