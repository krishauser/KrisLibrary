#ifndef PLANNING_GEODESIC_SPACE_H
#define PLANNING_GEODESIC_SPACE_H

#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/metric.h>
#include <KrisLibrary/math/interpolate.h>
using namespace Math;
typedef Vector Config;

/** @brief A space with geodesics and (optionally) geodesic derivatives. 
 *
 * The Interpolate function is a function x=f(a,b,u) and Distance(a,b)
 * gives a metric on arc length of the geodesic.
 *
 * NumDimensions() gives the number of Config elements.  NumIntrinsicDimensions
 * returns then number of intrinsic dimensions, which may be different from
 * NumDimensions() if the space is a sub-manifold of the ambient space.
 *
 * Optionally, this class provides information about df/du (InterpolateDeriv),
 * df/da*a' (InterpolateDerivA), df/db*b' (InterpolateDerivB), and ddf/du^2
 * (InterpolateDeriv2). 
 *
 * It also allows you to integrate a tangent vector to the manifold da
 * starting from configuration a (Integrate).
 *
 * An example of the use of this would be geodesic interpolator on a sphere
 * that is parameterized via theta/phi angles.
 *
 * The default implementation assumes a linear interpolation / Cartesian space.
 */
class GeodesicSpace
{
 public:
  GeodesicSpace() {}
  virtual ~GeodesicSpace() {}
  virtual int NumDimensions() { FatalError("NumDimensions method not implemented"); return -1; }
  virtual int NumIntrinsicDimensions() { return NumDimensions(); }
  virtual Real Distance(const Config& x, const Config& y) { return Distance_L2(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { interpolate(x,y,u,out); }

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) { dx.sub(b,a); }
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) { dx.mul(da,1-u); }
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) { dx.mul(db,u); }
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) { ddx.resize(a.n,Zero); }
  virtual void Integrate(const Config& a,const Vector& da,Config& b) { b.add(a,da); }
};

class CartesianSpace : public GeodesicSpace
{
public:
  CartesianSpace(int d);
  virtual ~CartesianSpace() {}
  virtual int NumDimensions() { return d; }

  int d;
};

/*
class MultiGeodesicSpace : public GeodesicSpace
{
public:
  MultiGeodesicSpace();
  MultiGeodesicSpace(const std::vector<std::shared_ptr<GeodesicSpace> >& components);
  void Add(const std::shared_ptr<GeodesicSpace>& space,Real weight=1.0);

  void Split(const Vector& x,std::vector<Vector>& items) const;
  void Join(const std::vector<Vector>& items,Vector& x) const;

  virtual int NumDimensions();
  virtual int NumIntrinsicDimensions();
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx);
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx);
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  std::vector<std::shared_ptr<GeodesicSpace> > components;
  std::vector<Real> weights;
};
*/

#endif
