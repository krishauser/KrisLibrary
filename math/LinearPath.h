#ifndef MATH_LINEAR_PATH_H
#define MATH_LINEAR_PATH_H

#include "function.h"
#include "vector.h"
#include "Interval.h"
#include <vector>

namespace Math {

class PiecewiseLinearFunction : public VectorFunction
{
public:
  //if t is -Inf or +Inf, y specifies slope before/after function
  struct ControlPoint {
    Real t;
    Real y;
  };

  virtual void PreEval(Real t);
  virtual Real Eval(Real t);
  virtual Real Deriv(Real t);

  inline Real operator[](int i) const { return points[i].y; }
  inline Real& operator[](int i) { return points[i].y; }
  inline size_t size() const { return points.size(); }
  std::vector<ControlPoint>::iterator GetSegment(Real t);
  ClosedInterval Domain() const;
  ClosedInterval Range() const;
  bool IsValid() const;
  void ScaleTime(Real s);
  void OffsetTime(Real off);
  void Concat(const PiecewiseLinearFunction& p);  //adds p onto the end of this
  void Append(const Real y,Real dt=Zero);

  bool Read(File& f);
  bool Write(File& f) const;

  std::vector<ControlPoint> points;
};

class PiecewiseLinearPath : public VectorFunction
{
public:
  struct ControlPoint {
    Real t;
    Vector x;
  };

  virtual void PreEval(Real t);
  virtual void Eval(Real t,Vector& x);
  virtual void Deriv(Real t,Vector& dx);

  //overridable functions
  //Interpolate x from a->b
  //Set to be the difference x=a-b
  virtual void Interpolate(const Vector& a,const Vector& b,Real u,Vector& x) const;
  virtual void Difference(const Vector& a,const Vector& b,Vector& x) const;
  virtual Real Distance(const Vector& a,const Vector& b) const;

  inline const Vector& operator[](int i) const { return points[i].x; }
  inline Vector& operator[](int i) { return points[i].x; }
  inline size_t size() const { return points.size(); }
  std::vector<ControlPoint>::iterator GetSegment(Real t);
  inline Real BeginTime() const { return points.front().t; }
  inline Real EndTime() const { return points.back().t; }
  bool IsValid() const;
  Real Length() const;
  void ArcLengthParameterize();
  void ScaleTime(Real s);
  void OffsetTime(Real off);
  void Concat(const PiecewiseLinearPath& p);  //adds p onto the end of this
  void Append(const Vector& x,Real dt=Zero);

  bool Read(File& f);
  bool Write(File& f) const;

  std::vector<ControlPoint> points;
};

} //namespace Math

#endif
