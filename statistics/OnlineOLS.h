#ifndef STATISTICS_ONLINE_OLS_H
#define STATISTICS_ONLINE_OLS_H

#include "statistics.h"
#include <KrisLibrary/math/LDL.h>

namespace Statistics {

/** @brief Online estimation of least-squares solution to y = A x
 *
 * The A matrix is the "data" vector, y is the "outcome" vector.
 * Each update step takes O(n^2) time, where n is the number of
 * dimensions in x.
 */
struct OnlineLeastSquares
{
  OnlineLeastSquares();
  void SetPrior(const Vector& coeffs,Real strength);
  void SetPrior(const Vector& coeffs,const Vector& strength);
  bool AddPoint(const Vector& data,Real outcome);

  int numObservations;
  bool store;
  std::vector<Vector> data;
  std::vector<Real> outcome;
  LDLDecomposition<Real> ldl;  //stores the LDL decomposition of AtA
  Vector Aty;  //stores At y
  Vector coeffs;  //stores estimate of coeffs
};

/** @brief Stochastic gradient descent estimation of least-squares solution
 * to y = A x
 *
 * The A matrix is the "data" vector, y is the "outcome" vector.
 * Each update step takes O(n) time, where n is the number of
 * dimensions in x.
 *
 * Single step error-minimizing least-squares solution gives
 *   min ||x-coeffs||^2 s.t. am^T x = bm
 * => 2(x-coeffs) + lambda * am = 0, am^T x = bm
 *   x = coeffs + lambda*am
 * am^T x = bm = am^T coeffs + lambda* am^T am
 * lambda = (bm - am^T coeffs) / am^T am
 * => x - coeffs = am * (bm - am^T coeffs) / am^T am
 * Uses the stochastic gradient descent step
 *   coeffs(m) = coeffs(m-1) + w(m)*(x - coeffs).
 * where w(m) is a weight term.
 * 
 * The weight is 1 / P(m) where m is the number of observations, and P(m)
 * is a polynomial.  If P(m) is constant, then the estimate "forgets" distant
 * history.  If P(m) is linear, then it converges to the true estimate.
 */
struct StochasticLeastSquares
{
  StochasticLeastSquares();
  void SetPrior(const Vector& coeffs,int strength=1);
  void AddPoint(const Vector& data,Real outcome);

  int numObservations;
  bool store;
  std::vector<Vector> data;
  std::vector<Real> outcome;

  std::vector<Real> weightPolynomial;
  Vector coeffs;  //stores estimate of coeffs
};


} //namespace Statistics

#endif
