#ifndef PLANNING_INTERPOLATOR_HELPERS_H
#define PLANNING_INTERPOLATOR_HELPERS_H

#include "Interpolator.h"
#include "GeodesicSpace.h"
#include <memory>
#include <KrisLibrary/math/interpolate.h>
#include <KrisLibrary/spline/Hermite.h>
#include <KrisLibrary/spline/PiecewisePolynomial.h>


/** @ingroup MotionPlanning
 * @brief An interpolator that goes between two Configs.
 */
class LinearInterpolator : public Interpolator
{
public:
  LinearInterpolator(Real ax,Real bx,Real ta=0,Real tb=1);
  LinearInterpolator(const Config& a,const Config& b,Real ta=0,Real tb=1);
  virtual ~LinearInterpolator() {}
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const { return a.distance(b); }
  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; }
  virtual Real ParamStart() const { return ta; }
  virtual Real ParamEnd() const { return tb; }

  Config a, b;
  Real ta, tb;
};

/** @ingroup MotionPlanning
 * @brief An interpolator that uses a GeodesicSpace's Interpolate function 
 */
class GeodesicInterpolator : public Interpolator
{
public:
  GeodesicInterpolator(GeodesicSpace* space,const Config& a,const Config& b);
  virtual ~GeodesicInterpolator() {}
  virtual void Eval(Real u,Config& x) const { space->Interpolate(a,b,u,x); }
  virtual Real Length() const { return space->Distance(a,b); }
  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; }

  GeodesicSpace* space;
  Config a, b;
};

/** @ingroup MotionPlanning
 * @brief An interpolator that uses a CSpace's Interpolate function 
 */
class CSpaceInterpolator : public Interpolator
{
public:
  CSpaceInterpolator(CSpace* space,const Config& a,const Config& b);
  virtual ~CSpaceInterpolator() {}
  virtual void Eval(Real u,Config& x) const { space->Interpolate(a,b,u,x); }
  virtual Real Length() const { return space->Distance(a,b); }
  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; }

  CSpace* space;
  Config a, b;
};

/** @ingroup MotionPlanning
 * @brief An interpolator that remaps a time interval of another interpolator.
 * Maps the parameter range [pstart,pend] (by default [0,1]) to the range [a,b]
 * on the base interpolator.
 */
class TimeRemappedInterpolator : public Interpolator
{
public:
  TimeRemappedInterpolator(const InterpolatorPtr& base,Real a,Real b,Real pstart=0,Real pend=1);
  virtual ~TimeRemappedInterpolator() {}
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const { return base->End(); }
  virtual const Config& End() const { return base->Start(); }
  virtual Real ParamStart() const { return pstart; }
  virtual Real ParamEnd() const { return pend; }

  InterpolatorPtr base;
  Real a,b;
  Real pstart,pend;
};

/** @ingroup MotionPlanning
 * @brief An interpolator that concatenates a list of other interpolators.
 * The interpolators may have nonuniform duration.
 */
class PathInterpolator : public Interpolator
{
public:
  PathInterpolator();
  PathInterpolator(const InterpolatorPtr& interp);
  PathInterpolator(const std::vector<InterpolatorPtr > & segments,Real totalTime=1.0);
  virtual ~PathInterpolator() {}
  ///Adds a new segment.  If duration = 0, all durations / times are scaled to keep the current total time
  void Append(const InterpolatorPtr& interp,Real duration=0);
  void Concat(const PathInterpolator& path);
  void ScaleDuration(Real scale);
  void SetTotalTime(Real ttotal);
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const { return segments.front()->Start(); }
  virtual const Config& End() const { return segments.back()->End(); }
  virtual Real ParamStart() const { return times.empty() ? 0 : times.front(); }
  virtual Real ParamEnd() const { return times.empty() ? 1 : times.back(); }

  std::vector<InterpolatorPtr > segments;
  std::vector<Real> durations;
  std::vector<Real> times;
};

class PiecewiseLinearInterpolator : public Interpolator
{
public:
  PiecewiseLinearInterpolator(const std::vector<Config>& configs);
  PiecewiseLinearInterpolator(const std::vector<Config>& configs,const std::vector<Real>& times);
  virtual ~PiecewiseLinearInterpolator() {}
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const { return configs.front(); }
  virtual const Config& End() const { return configs.back(); }
  virtual Real ParamStart() const { return times.empty() ? 0 : times.front(); }
  virtual Real ParamEnd() const { return times.empty() ? 1 : times.back(); }

  std::vector<Config> configs;
  std::vector<Real> times;
};

class PiecewiseLinearCSpaceInterpolator : public PiecewiseLinearInterpolator
{
public:
  PiecewiseLinearCSpaceInterpolator(CSpace* space,const std::vector<Config>& configs);
  PiecewiseLinearCSpaceInterpolator(CSpace* space,const std::vector<Config>& configs,const std::vector<Real>& times);
  virtual ~PiecewiseLinearCSpaceInterpolator() {}
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  
  CSpace* space;
};

class PiecewisePolynomialInterpolator : public Interpolator
{
public:
  PiecewisePolynomialInterpolator(const Spline::PiecewisePolynomialND& path);
  virtual ~PiecewisePolynomialInterpolator() {}
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const { return start; }
  virtual const Config& End() const { return end; }
  virtual Real ParamStart() const { return path.StartTime(); }
  virtual Real ParamEnd() const { return path.EndTime(); }

  Spline::PiecewisePolynomialND path;
  Vector start,end;
};


/** @ingroup MotionPlanning
 * @brief A cartesian product of interpolators.
 */
class MultiInterpolator : public Interpolator
{
public:
  MultiInterpolator(const InterpolatorPtr& component1,const InterpolatorPtr& component2);
  MultiInterpolator(const std::vector<InterpolatorPtr > & components);
  virtual ~MultiInterpolator() {}
  void Split(const Vector& x,std::vector<Vector>& items) const;
  void Join(const std::vector<Vector>& items,Vector& x) const;
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; }
  virtual Real ParamStart() const { return components[0]->ParamStart(); }
  virtual Real ParamEnd() const { return components[0]->ParamEnd(); }

  std::vector<InterpolatorPtr > components;
  Config a,b;
};

class SubsetInterpolator : public Interpolator
{
public:
  SubsetInterpolator(const InterpolatorPtr& base,int start,int end);
  virtual ~SubsetInterpolator() {}
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const { return base->Length(); }
  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; }
  virtual Real ParamStart() const { return base->ParamStart(); }
  virtual Real ParamEnd() const { return base->ParamEnd(); }

  InterpolatorPtr base;
  int start,end;
  Config a,b;
};

#endif