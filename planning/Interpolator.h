#ifndef PLANNING_INTERPOLATOR_H
#define PLANNING_INTERPOLATOR_H

#include <KrisLibrary/planning/CSpace.h>
#include <memory>

/** @ingroup MotionPlanning
 * @brief A base class for all 1D interpolators.
 */
class Interpolator
{
public:
  virtual ~Interpolator() {}
  virtual void Eval(Real u,Config& x) const=0;
  virtual Real Length() const=0;
  virtual const Config& Start() const=0;
  virtual const Config& End() const=0;	
  virtual Real ParamStart() const { return 0; }
  virtual Real ParamEnd() const { return 1; }
};

/** @ingroup MotionPlanning
 * @brief An interpolator that reverses another one.
 */
class ReverseInterpolator : public Interpolator
{
public:
  ReverseInterpolator(const std::shared_ptr<Interpolator>& base);
  virtual ~ReverseInterpolator() {}
  virtual void Eval(Real u,Config& x) const { base->Eval(1.0-u,x); }
  virtual Real Length() const { return base->Length(); }
  virtual const Config& Start() const { return base->End(); }
  virtual const Config& End() const { return base->Start(); }

  std::shared_ptr<Interpolator> base;
};

typedef std::shared_ptr<Interpolator> InterpolatorPtr;

#endif