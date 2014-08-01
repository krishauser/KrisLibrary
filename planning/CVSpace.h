#ifndef PLANNING_CVSPACE_H
#define PLANNING_CVSPACE_H

#include "CSpace.h"
#include "GeodesicSpace.h"

/** @brief A C-space on configurations and velocities (q,v) that linear uses linear
 * interpolation on velocities
 */
class CVSpace : public CSpace
{
 public:
  CVSpace(CSpace* baseSpace,CSpace* velSpace);

  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);
  virtual void Properties(PropertyMap& props) const;

  static void GetState(const Config& x,Config& q,Vector& v);
  static void GetConfig(const Config& x,Config& q);
  static void GetVelocity(const Config& x,Vector& v);
  static void GetStateRef(const Config& x,Config& q,Vector& v);
  static void GetConfigRef(const Config& x,Config& q);
  static void GetVelocityRef(const Config& x,Vector& v);
  static void SetState(const Config& q,const Vector& v,Config& x);
  static void SetConfig(const Config& q,Config& x);
  static void SetVelocity(const Vector& v,Config& x);

  CSpace* baseSpace;
  CSpace* velSpace;
};

/** @brief A C-space on configurations and velocities (q,v) that uses Hermite
 * interpolation to construct continuously differentiable curves.
 */
class HermiteCSpace : public CVSpace
{
 public:
  HermiteCSpace(CSpace* baseSpace,CSpace* velSpace,GeodesicManifold* manifold=NULL);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);
  virtual void Properties(PropertyMap& props) const;

  GeodesicManifold* manifold;
};

class CVGeodesicManifold : public GeodesicManifold
{
 public:
  CVGeodesicManifold(GeodesicManifold* baseManifold=NULL,GeodesicManifold* velManifold=NULL);
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  GeodesicManifold *baseManifold, *velManifold;
};

class HermiteGeodesicManifold : public GeodesicManifold
{
 public:
  HermiteGeodesicManifold(HermiteCSpace* space);
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  HermiteCSpace* space;
};


#endif
