#ifndef OPTIMIZATION_NONLINEAR_PROGRAM_H
#define OPTIMIZATION_NONLINEAR_PROGRAM_H

#include <KrisLibrary/math/function.h>

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief A structure defining a nonlinear program.
 *
 * This has the structure <br>
 * min/max	f(x)      <br>
 *  s.t.	c(x)=0    <br>
 *              d(x)<?>0  <br>
 * where <?> can be either >= or <=
 */
struct NonlinearProgram
{
  NonlinearProgram(ScalarFieldFunction* f,VectorFieldFunction* c=NULL,VectorFieldFunction* d=NULL);
  void PreEval(const Vector& x) const;
  bool SatisfiesEquality(const Vector& x,Real tol=Epsilon) const;
  bool SatisfiesInequality(const Vector& x) const;
  Real Lagrangian(const Vector& x,const Vector& lambda,const Vector& mu) const;

  ///use these if it's faster to evaluate all constraints together
  Real LagrangianEval(const Vector& x,const Vector& lambda,const Vector& mu) const;
  void LagrangianGradient(const Vector& x,const Vector& lambda,const Vector& mu, Vector& grad) const;
  void LagrangianHessian(const Vector& x,const Vector& lambda,const Vector& mu, Matrix& H) const;

  ///use these if it's faster to evaluate constraints separately
  Real LagrangianEval_Sparse(const Vector& x,const Vector& lambda,const Vector& mu) const;
  void LagrangianGradient_Sparse(const Vector& x,const Vector& lambda,const Vector& mu, Vector& grad) const;
  void LagrangianHessian_Sparse(const Vector& x,const Vector& lambda,const Vector& mu, Matrix& H) const;

  ScalarFieldFunction* f;
  VectorFieldFunction* c;
  VectorFieldFunction* d;
  bool minimize;        ///true if f is to be minimized, false if maximized
  bool inequalityLess;  ///inequality constraints are d<=0, else d>=0
};

} //namespace Optimization

#endif
