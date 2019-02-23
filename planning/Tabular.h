#ifndef TABULAR_CSPACE_H
#define TABULAR_CSPACE_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/AreaGrid.h>
#include "CSpace.h"
#include "CSetHelpers.h"
#include "Objective.h"
using namespace Math3D;

/** @ingroup Planning
 * @brief A 1D tabular function interpolated over a grid.
 *
 * The domain is [a,b]. If values contains n items v0,...,vn-1,
 * then each item vi is reached exactly at x = a+(i+0.5)/n*(b-a)
 * (interpolation through cell centers).
 *
 * Each cell is "active" over the x range [a+i/n*(b-a),a+(i+1)/n*(b-a)].
 * 
 * Lookup gives the index i and the interpolation value u in [-0.5,0.5]
 * that tells how much to blend an adjacent value at i-1 or i+1.
 */
template <class T>
class Table1D
{
public:
  Table1D(int n,Real a,Real b,
    int dim=0);
  Table1D(const std::vector<T>& values,Real a,Real b,
    int dim=0);
  T Lookup(Real x) const;
  int Index(Real x,Real* u=NULL) const;
  std::pair<Real,Real> Cell(int index) const;
  inline T Lookup(const Config& x) const { return Lookup(x[dim]); }
  inline int Index(const Config& x,Real* u=NULL) const { return Index(x[dim],u); }

  std::vector<T> values;
  Real a,b;
  int dim;
};

/** @ingroup Planning
 * @brief A 1D set that includes some marked regions on a grid.
 */
class Tabular1DSet : public BoxSet, public Table1D<bool>
{
public:
  Tabular1DSet(const std::vector<bool>& valid,Real a,Real b);
  Tabular1DSet(int n,Real a,Real b);
  virtual ~Tabular1DSet() {}
  virtual int NumDimensions() const { return 1; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsSampleable() const { return true; }
  virtual void Sample(Config& x);

  bool calculatedValidIndices;
  std::vector<int> validIndices;
};

/** @ingroup Planning
 * @brief A 2D set that includes some marked regions on a grid.
 *
 * Basically an adaptor from AreaGridTemplate to the CSpace methods.
 */
class Tabular2DSet : public BoxSet, public Meshing::AreaGridTemplate<bool>
{
public:
  Tabular2DSet(const Array2D<bool>& valid,const Vector2& bmin,const Vector2& bmax);
  Tabular2DSet(int m,int n,const Vector2& bmin,const Vector2& bmax);
  virtual ~Tabular2DSet() {}
  virtual int NumDimensions() const { return 2; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsSampleable() const { return true; }
  virtual void Sample(Config& x);

  bool calculatedValidIndices;
  std::vector<IntPair> validIndices;
};

/** @ingroup Planning
 * @brief A 3D set that includes some marked regions on a grid.
 *
 * Basically an adaptor from VolumeGridTemplate to the CSpace methods.
 */
class Tabular3DSet : public BoxSet, public Meshing::VolumeGridTemplate<bool>
{
public:
  Tabular3DSet(const Array3D<bool>& valid,const Vector3& bmin,const Vector3& bmax);
  Tabular3DSet(int m,int n, int p,const Vector3& bmin,const Vector3& bmax);
  virtual ~Tabular3DSet() {}
  virtual int NumDimensions() const { return 2; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsSampleable() const { return true; }
  virtual void Sample(Config& x);

  bool calculatedValidIndices;
  std::vector<IntTriple> validIndices;
};

/** @ingroup Planning
 * @brief A 1D level set f(x) <= threshold.
 *
 * Values of f are linearly interpolated on the grid [a,b].  These are
 * interpreted as being located at the grid cell *centers*, not 
 * the vertices.
 */
class Tabular1DLevelSet : public BoxSet, Table1D<Real>
{
public:
  Tabular1DLevelSet(const std::vector<Real>& f,Real a,Real b,Real threshold=0);
  virtual ~Tabular1DLevelSet() {}
  virtual int NumDimensions() const { return 1; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);

  Real threshold;
};

/** @ingroup Planning
 * @brief A 2D level set f(x) <= threshold. 
 *
 * Values of f are bilinearly interpolated on the grid [bmin,bmax]. 
 * These are interpreted as being located at the grid cell *centers*, not 
 * the vertices.
 *
 * Basically an adaptor from AreaGridTemplate to the CSpace methods.
 */
class Tabular2DLevelSet : public BoxSet
{
public:
  Tabular2DLevelSet(const Array2D<Real>& f,const Vector2& bmin,const Vector2& bmax,Real threshold=0);
  virtual ~Tabular2DLevelSet() {}
  virtual int NumDimensions() const { return 2; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);

  Meshing::AreaGridTemplate<Real> agrid;
  Real threshold;
};

/** @ingroup Planning
 * @brief A 3D level set f(x) <= threshold
 *
 * Values of f are trilinearly interpolated on the grid [bmin,bmax]. 
 * These are interpreted as being located at the grid cell *centers*, not 
 * the vertices.
 *
 * Basically an adaptor from VolumeGridTemplate to the CSpace methods.
 */
class Tabular3DLevelSet : public BoxSet
{
public:
  Tabular3DLevelSet(const Array3D<Real>& f,const Vector3& bmin,const Vector3& bmax,Real threshold=0);
  virtual ~Tabular3DLevelSet() {}
  virtual int NumDimensions() const { return 3; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);

  Meshing::VolumeGridTemplate<Real> vgrid;
  Real threshold;
};



/** @ingroup Planning
 * @brief A cost that penalizes via a 1D table lookup of a single index.
 *
 * The cost tables are assumed to be on a grid over the interval [a,b].
 * Values past the interval are clamped to the interval.  These are
 * interpreted so the value cost[i] is reached exactly at a+i/n*(b-a),
 * where n = length(cost).
 *
 * For the incremental cost, the state x(dim) is looked up in
 * differentialCosts.  This is integrated over the length of the
 * interpolator.
 *
 * For the incremental cost, the state x(dim) is looked up in
 * terminalCosts.
 *
 * Either table can be empty, in which case no cost is assessed.
 */
class Tabular1DObjective : public ObjectiveFunctionalBase
{
public:
  Tabular1DObjective(const std::vector<Real>& differentialCosts,const std::vector<Real>& terminalCosts,
    Real a,Real b,
    int dim);
  virtual ~Tabular1DObjective() {}
  virtual const char* TypeString() { return "tabular1d"; }
  virtual std::string Description();
  virtual bool PathInvariant() const { return differentialCosts.values.empty(); }
  virtual Real IncrementalCost(const Interpolator* path);
  virtual Real TerminalCost(const Vector& qend);

  Table1D<Real> differentialCosts,terminalCosts;
  int dim;
};

/** @ingroup Planning
 * @brief A cost that penalizes via a 2D table lookup of a single index.
 *
 * The cost tables are assumed to be on a grid over the domain [bmin,bmax].
 * Values past the interval are clamped to the range.
 *
 * For the incremental cost, the state [x(dim0),x(dim1)] is looked up in
 * differentialCosts.  This is integrated over the length of the interpolator.
 *
 * For the incremental cost, the state [x(dim0,x(dim1)] is looked up in
 * terminalCosts.
 *
 * Either table can be empty, in which case no cost is assessed.
 */
class Tabular2DObjective : public ObjectiveFunctionalBase
{
public:
  Tabular2DObjective(const Array2D<Real>& differentialCosts,const Array2D<Real>& terminalCosts,
    const Math3D::Vector2& bmin,const Math3D::Vector2& bmax,
    int dim0=0,int dim1=1);
  virtual ~Tabular2DObjective() {}
  virtual const char* TypeString() { return "tabular2d"; }
  virtual std::string Description();
  virtual bool PathInvariant() const { return differentialCosts.value.empty(); }
  virtual Real IncrementalCost(const Interpolator* path);
  virtual Real TerminalCost(const Vector& qend);

  Meshing::AreaGridTemplate<Real> differentialCosts,terminalCosts;
  int dim0,dim1;
};

/** @ingroup Planning
 * @brief A cost that penalizes via a 3D table lookup of a single index.
 *
 * The cost tables are assumed to be on a grid over the domain [bmin,bmax].
 * Values past the interval are clamped to the range.
 *
 * For the incremental cost, the state x(index0) is looked up in
 * differentialCosts, using linear interpolation.  This is integrated
 * over the length of the interpolator.
 *
 * For the incremental cost, the state x(index) is looked up in
 * terminalCosts, using linear interpolation.
 *
 * Either table can be empty, in which case no cost is assessed.
 */
class Tabular3DObjective : public ObjectiveFunctionalBase
{
public:
  Tabular3DObjective(const Array3D<Real>& differentialCosts,const Array3D<Real>& terminalCosts,
    const Math3D::Vector3& bmin,const Math3D::Vector3& bmax,
    int dim0=0,int dim1=1,int dim2=2);
  virtual ~Tabular3DObjective() {}
  virtual const char* TypeString() { return "tabular23"; }
  virtual std::string Description();
  virtual bool PathInvariant() const { return differentialCosts.value.empty(); }
  virtual Real IncrementalCost(const Interpolator* path);
  virtual Real TerminalCost(const Vector& qend);

  Meshing::VolumeGridTemplate<Real> differentialCosts,terminalCosts;
  int dim0,dim1,dim2;
};







template <class T>
Table1D<T>::Table1D(int n,Real _a,Real _b,int _dim)
:values(n,T(0)),a(_a),b(_b),dim(_dim)
{}

template <class T>
Table1D<T>::Table1D(const std::vector<T>& _values,Real _a,Real _b,int _dim)
:values(_values),a(_a),b(_b),dim(_dim)
{}

template <class T>
T Table1D<T>::Lookup(Real x) const
{
  Real u;
  int i=Index(x,&u);
  //linearly interpolate by default
  if(u == 0)
    return values[i];
  else if(u > 0)
    return values[i] + u*(values[i+1]-values[i]);
  else
    return values[i] + (-u)*(values[i-1]+values[i]);
}

template <class T>
std::pair<Real,Real> Table1D<T>::Cell(int index) const
{
  return std::make_pair(a + index*(b-a)/values.size(),a + (index+1)*(b-a)/values.size());
}

template <class T>
int Table1D<T>::Index(Real x,Real* u) const
{
  if(values.size() <= 1) {
    if(u) u[0]=0;
    return 0;
  }
  if(x <= a) {
    if(u) u[0]=0;
    return 0;
  }
  if(x >= b) {
    if(u) u[0]=0;
    return (int)values.size()-1;
  }
  Real xscale = (x-a)/(b-a)*(values.size()-1);
  Real xbase = Floor(xscale);
  if(u) u[0] = xscale-xbase-0.5;
  return (int)xbase;
}

//template specialization
template <>
bool Table1D<bool>::Lookup(Real x) const 
{
  int i=Index(x);
  return values[i];
}

#endif