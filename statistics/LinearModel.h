#ifndef STATISTICS_LINEAR_MODEL_H
#define STATISTICS_LINEAR_MODEL_H

#include "statistics.h"

namespace Statistics { 

/** @brief A linear model of n-dimensional data.
 *
 * If constantOffset = true, there are n+1 coefficients, and the
 * n+1'th element of coeffs is a constant offset.
 * That is, the model is x^T coeffs(0:n-1) + coeffs(n).
 * Otherwise, the model is x^T coeffs.
 *
 * LeastSquares assumes normally distributed data with independent errors.
 */
class LinearModel
{
 public:
  enum SolveMethod { Cholesky, SVD, QR };
  
  LinearModel();
  Real Evaluate(const Vector& x) const;
  Real Variance(const Vector& x) const;
  void CoeffVariance(Vector& coeffVariance) const;
  bool LeastSquares(const Matrix& x,const Vector& outcome);
  bool LeastSquares(const std::vector<Vector>& x,const std::vector<Real>& outcome);
  bool LeastSquares(const std::vector<Vector>& x,int outcomeIndex);

 private:
  bool LeastSquares_Internal(const Matrix& x,const Vector& outcome);
  bool LeastSquares_Cholesky(const Matrix& x,const Vector& outcome);
  bool LeastSquares_SVD(const Matrix& x,const Vector& outcome);
  bool LeastSquares_QR(const Matrix& x,const Vector& outcome);
 public:

  SolveMethod solveMethod;
  bool constantOffset;

  Vector coeffs;
  Matrix covariance;
};

std::istream& operator >> (std::istream& in,LinearModel& model);
std::ostream& operator << (std::ostream& out,const LinearModel& model);

} //namespace Statistics

#endif
