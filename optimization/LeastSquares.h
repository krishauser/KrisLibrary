#ifndef OPTIMIZATION_LEAST_SQUARES_H
#define OPTIMIZATION_LEAST_SQUARES_H

#include <KrisLibrary/math/matrix.h>

namespace Optimization
{
using namespace Math;

/** @ingroup Optimization
 * @brief A least-squares problem definition.
 *
 * Defines the least-squares problem
 * min |A*x-b|^2 subject to <br>
 *   C*x = d   (equality constraints) <br>
 *   E*x <= f  (inequality constrants)
 */
struct LeastSquares
{
  LeastSquares();
  bool Solve(Vector& x) const;
  bool SatisfiesInequalities(const Vector& x) const;
  bool SatisfiesEqualities(const Vector& x,Real tol=Epsilon) const;
  Real Objective(const Vector& x) const;
  void Print(std::ostream& out) const;
  void PrintStats(const Vector& x,std::ostream& out) const;

  Matrix A; Vector b;
  Matrix Aeq; Vector beq;
  Matrix Aineq; Vector bineq;
  const Vector* initialPoint;  //optional initial feasible point
  int verbose;
};

} //namespace Optimization

#endif
