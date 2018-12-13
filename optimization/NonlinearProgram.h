#ifndef OPTIMIZATION_NONLINEAR_PROGRAM_H
#define OPTIMIZATION_NONLINEAR_PROGRAM_H

#include <KrisLibrary/math/function.h>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/matrix.h>
#include <memory>

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief A structure defining a nonlinear program.
 *
 * This has the structure <br>
 * min/max	f(x)      <br>
 *  s.t.	c(x)=0    <br>
 *              d(x)<?>0  <br>
 * where <?> can be either >= or <= depending on the
 * value of inequalityLess.  By default, <= is used.
 */
class NonlinearProgram
{
public:
  NonlinearProgram(const std::shared_ptr<ScalarFieldFunction>& f,const std::shared_ptr<VectorFieldFunction>& c=NULL,const std::shared_ptr<VectorFieldFunction>& d=NULL);
  void PreEval(const Vector& x);
  bool SatisfiesEquality(const Vector& x,Real tol=Epsilon);
  bool SatisfiesInequality(const Vector& x);
  Real Lagrangian(const Vector& x,const Vector& lambda,const Vector& mu);

  ///use these if it's faster to evaluate all constraints together
  Real LagrangianEval(const Vector& x,const Vector& lambda,const Vector& mu);
  void LagrangianGradient(const Vector& x,const Vector& lambda,const Vector& mu, Vector& grad);
  void LagrangianHessian(const Vector& x,const Vector& lambda,const Vector& mu, Matrix& H);

  ///use these if it's faster to evaluate constraints separately
  Real LagrangianEval_Sparse(const Vector& x,const Vector& lambda,const Vector& mu);
  void LagrangianGradient_Sparse(const Vector& x,const Vector& lambda,const Vector& mu, Vector& grad);
  void LagrangianHessian_Sparse(const Vector& x,const Vector& lambda,const Vector& mu, Matrix& H);

  std::shared_ptr<ScalarFieldFunction> f;
  std::shared_ptr<VectorFieldFunction> c;
  std::shared_ptr<VectorFieldFunction> d;
  bool minimize;        ///true if f is to be minimized, false if maximized
  bool inequalityLess;  ///inequality constraints are d<=0, else d>=0
};

} //namespace Optimization

#endif

