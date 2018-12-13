#ifndef PLANNING_CVSPACE_H
#define PLANNING_CVSPACE_H

#include "CSpace.h"
#include "CSpaceHelpers.h"

/** @brief A C-space on configurations and velocities (q,v) that linear uses linear
 * interpolation on velocities
 */
class CVSpace : public MultiCSpace
{
 public:
  CVSpace(const std::shared_ptr<CSpace>& baseSpace,const std::shared_ptr<CSpace>& velSpace);

  static void GetState(const Config& x,Config& q,Vector& v);
  static void GetConfig(const Config& x,Config& q);
  static void GetVelocity(const Config& x,Vector& v);
  static void GetStateRef(const Config& x,Config& q,Vector& v);
  static void GetConfigRef(const Config& x,Config& q);
  static void GetVelocityRef(const Config& x,Vector& v);
  static void SetState(const Config& q,const Vector& v,Config& x);
  static void SetConfig(const Config& q,Config& x);
  static void SetVelocity(const Vector& v,Config& x);
};

/** @brief A C-space on configurations and velocities (q,v) that uses Hermite
 * interpolation to construct continuously differentiable curves.
 */
class HermiteCSpace : public CVSpace
{
 public:
  HermiteCSpace(const std::shared_ptr<CSpace>& baseSpace,const std::shared_ptr<CSpace>& velSpace);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Properties(PropertyMap& props);

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);
};


#endif
