#ifndef STATISTICS_ONLINE_LASSO_H
#define STATISTICS_ONLINE_LASSO_H

#include "statistics.h"
#include <KrisLibrary/math/LDL.h>

namespace Statistics {

/** @brief Stochastic estimation of solution to y = A x via a LASSO-like
 * procedure
 * min_x ||b-A x||^2 + alpha*||x||_1
 *
 * The A matrix is the "data" vector, y is the "outcome" vector.
 * Each update step takes O(n) time, where n is the number of
 * dimensions in x.
 *
 * The change in coeffs is estimated each step via a LASSO-like step.  It is
 * then added to the current estimate via the rule
 *    x(m) = x(m-1) + w(m)*deltax(m)
 * where w(m) is a weight equal to 1/P(m), with P(m) being a polynomial.
 */
struct StochasticPseudoLASSO
{
  StochasticPseudoLASSO(Real alpha=0.01);
  void SetPrior(const Vector& coeffs,int strength);
  void AddPoint(const Vector& data,Real outcome);

  Real alpha;
  int numObservations;
  std::vector<Real> weightPolynomial;
  Vector coeffs;  //stores estimate of coeffs
};

} //namespace Statistics

#endif
