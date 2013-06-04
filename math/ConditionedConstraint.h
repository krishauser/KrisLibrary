#ifndef MATH_CONDITIONED_CONSTRAINT_H
#define MATH_CONDITIONED_CONSTRAINT_H

#include "InequalityConstraint.h"

namespace Math {

//requires that scale*f(x)+offset >= 0
class ScaledOffsetConstraint : public InequalityConstraintAdaptor
{
public:
  ScaledOffsetConstraint(InequalityConstraint& f);
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const { return f.NumDimensions(); }
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  virtual Real Margin(const Vector& x,int& minConstraint) { return InequalityConstraint::Margin(x,minConstraint); }
  virtual bool Satisfies(const Vector& x,Real d=Zero) { return InequalityConstraint::Satisfies(x,d); }
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);
  virtual void LineSearch(const Vector& x0,const Vector& dx,Real& u);
  virtual bool Push(Vector& x,Real d=Zero) { return InequalityConstraint::Push(x,d); }
  virtual bool Push_i(Vector& x,int i,Real d);

  std::vector<Real> scale;
  std::vector<Real> offset;
};

class UniformScaledOffsetConstraint : public InequalityConstraintAdaptor
{
public:
  UniformScaledOffsetConstraint(InequalityConstraint& f);
  virtual std::string Label() const;
  virtual int NumDimensions() const { return f.NumDimensions(); }
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  virtual Real Margin(const Vector& x,int& minConstraint) { return InequalityConstraint::Margin(x,minConstraint); }
  virtual bool Satisfies(const Vector& x,Real d=Zero) { return InequalityConstraint::Satisfies(x,d); }
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);
  virtual void LineSearch(const Vector& x0,const Vector& dx,Real& u);
  virtual bool Push(Vector& x,Real d=Zero) { return InequalityConstraint::Push(x,d); }
  virtual bool Push_i(Vector& x,int i,Real d);

  Real scale,offset;
};

} //namespace Math

#endif
