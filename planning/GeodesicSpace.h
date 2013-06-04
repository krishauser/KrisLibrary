#ifndef PLANNING_GEODESIC_SPACE_H
#define PLANNING_GEODESIC_SPACE_H

#include "CSpace.h"

/** @brief Extra information for a CSpace about manifold derivatives. 
 *
 * The CSpace's Interpolate function is a function x=f(a,b,u).  This
 * class provides information about df/du (InterpolateDeriv),
 * df/da*a' (InterpolateDerivA), df/db*b' (InterpolateDerivB), and ddf/du^2
 * (InterpolateDeriv2). 
 *
 * It also allows you to integrate a tangent vector to the manifold da
 * starting from configuration a (Integrate).
 *
 * An example of the use of this would be geodesic interpolator on a sphere
 * that is parameterized via theta/phi angles.
 *
 * The default implementation assumes a linear interpolation.
 */
class GeodesicManifold
{
 public:
  virtual ~GeodesicManifold() {}
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) { dx.sub(b,a); }
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) { dx.mul(da,1-u); }
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) { dx.mul(db,u); }
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) { ddx.resize(a.n,Zero); }
  virtual void Integrate(const Config& a,const Vector& da,Config& b) { b.add(a,da); }
};


#endif
