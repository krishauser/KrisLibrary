#ifndef STATISTICS_ONLINE_OLS_H
#define STATISTICS_ONLINE_OLS_H

#include "statistics.h"
#include <math/LDL.h>

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

} //namespace Statistics

#endif
