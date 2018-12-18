#ifndef MATH_INEQUALITY_CONSTRAINT_H
#define MATH_INEQUALITY_CONSTRAINT_H

#include "function.h"
#include "matrix.h"
#include <KrisLibrary/utils/DirtyData.h>
#include <vector>

namespace Math {
  /** @addtogroup Math */
  /*@{*/

/** @brief A vector field with constraints s.t. ci(x) >= 0
 *
 * The value ci(x) represents the "margin" of the constraint.
 * There's an issue of scale, and in general it is nice to have
 * |grad(ci(x))| ~= 1 near the boundaries, so the margin is
 * roughly the distance to the boundary.
 */
class InequalityConstraint : public VectorFieldFunction
{
public:
  virtual ~InequalityConstraint() {}
  virtual int NumDimensions() const;

  //default implementation assumes constraints are independent
  virtual Real Margin(const Vector& x,int& minConstraint);
  virtual bool Satisfies(const Vector& x,Real d=Zero);
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);

  /// Given x0 feasible, finds a maximal value
  /// u<=u0 s.t. x=x0+u*dx satisfies the constraint.
  /// i version: only satisfy the i'th constraint.
  /// j version: move in the direction ej.
  /// (default uses secant method)
  virtual void LineSearch(const Vector& x0,const Vector& dx,Real& u);
  virtual void LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i);
  virtual void LineSearch_j(const Vector& x0,int j,Real& u); 

  /// Sets x to satisfy the constraint by a margin of d. 
  /// Returns true on success.
  virtual bool Push(Vector& x,Real d);
  virtual bool Push_i(Vector&,int i,Real d);
};

class CompositeInequalityConstraint : public InequalityConstraint
{
public:
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);
  virtual Real Margin(const Vector& x,int& minConstraint);
  virtual bool Satisfies(const Vector& x,Real d=Zero);
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);
  virtual void LineSearch(const Vector& x0,const Vector& dx,Real& u);
  virtual void LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i);
  virtual bool Push(Vector& x,Real d);
  virtual bool Push_i(Vector& x,int i,Real d);

  int GetConstraint(int &i) const;

  std::vector<InequalityConstraint*> constraints;
};

class LimitConstraint : public InequalityConstraint
{
public:
  LimitConstraint(const Vector& bmin,const Vector& bmax);
  virtual std::string Label() const { return "<x within limits>"; }
  virtual int NumDimensions() const;
  virtual void Eval(const Vector& x,Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual Real Margin(const Vector& x,int& minConstraint);
  virtual bool Satisfies(const Vector& x,Real d=Zero);
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);
  virtual void LineSearch(const Vector& x0,const Vector& dx,Real& u);
  virtual void LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i);
  virtual void LineSearch_j(const Vector& x0,int j,Real& u);
  virtual bool Push(Vector& x,Real d);
  virtual bool Push_i(Vector& x,int i,Real d);

  const Vector &bmin,&bmax;
};

/// @brief constraint represented as A*x >= b
/// i.e. constraint value is A*x-b
class LinearConstraint : public InequalityConstraint
{
public:
  LinearConstraint(const Matrix& A,const Vector& b);

  virtual std::string Label() const { return "<Ax>=b>"; }
  virtual int NumDimensions() const { return A.m; }
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual Real Jacobian_ij(const Vector& x,int i,int j);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);
  //can perform the line search analytically
  virtual void LineSearch (const Vector& x0,const Vector& dx,Real& u);
  virtual void LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i);
  //can perform the push operation nearly analytically
  virtual bool Push(Vector& x,Real d=Zero);

  const Matrix& A;
  const Vector& b;
};

/// @brief A class that makes it easier to construct "plugin" classes 
/// to alter constraint behavior
struct InequalityConstraintAdaptor : public InequalityConstraint
{
  InequalityConstraintAdaptor(InequalityConstraint& _f) :f(_f)
  {}
  virtual ~InequalityConstraintAdaptor() {}
  virtual std::string Label() const { return f.Label(); }
  virtual std::string Label(int i) const { return f.Label(i); }
  virtual int NumDimensions() const { return f.NumDimensions(); }
  virtual void PreEval(const Vector& x) { f.PreEval(x); }
  virtual void Eval(const Vector& x, Vector& v) { f.Eval(x,v); }
  virtual Real Eval_i(const Vector& x,int i) { return f.Eval_i(x,i); }
  virtual void Jacobian(const Vector& x,Matrix& J) { f.Jacobian(x,J); }
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji) { f.Jacobian_i(x,i,Ji); }
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi) { f.Hessian_i(x,i,Hi); }
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v) {f.DirectionalDeriv(x,h,v); }

  virtual Real Margin(const Vector& x,int& minConstraint) { return f.Margin(x,minConstraint); }
  virtual bool Satisfies(const Vector& x,Real d=Zero) { return f.Satisfies(x,d); }
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero) { return f.Satisfies_i(x,i,d); }
  virtual void LineSearch(const Vector& x0,const Vector& dx,Real& u) { f.LineSearch(x0,dx,u); }
  virtual void LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i) { f.LineSearch_i(x0,dx,u,i); }
  virtual void LineSearch_j(const Vector& x0,int j,Real& u) { f.LineSearch_j(x0,j,u); }

  virtual bool Push(Vector& x,Real d) { return f.Push(x,d); }
  virtual bool Push_i(Vector& x,int i,Real d) { return f.Push_i(x,i,d); }

  InequalityConstraint& f;
};

/*@}*/
} //namespace Math

#endif
