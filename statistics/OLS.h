#ifndef STATISTICS_OLS_H
#define STATISTICS_OLS_H

#include "statistics.h"

namespace Statistics {

/** @brief 1-D least squares y = ax+b
 *
 * Can solve normal, weighted, or perpendicular least-squares
 */
class LeastSquares1D
{
 public:
  LeastSquares1D(const Vector& x,const Vector& y);
  LeastSquares1D(const Vector& x,const Vector& y,const Vector& weights);
  bool Solve();               ///< standard least-squares, y as fn of x
  bool SolveUnweighted();     ///< ignores weights w, if defined
  bool SolveWeighted();       ///< weighted least-squares
  bool SolvePerpendicular();  ///< perpendicular least-squares

  const Vector &x, &y;
  const Vector* w;
  Real a,b;        ///< coefficients a,b
  Real stda,stdb;  ///< standard errors for a,b
  Real corrCoeff;  ///< correlation coefficient
};


/** @brief Calculate a least squares fit of the outcome given data.
 *
 * The rows of data give the individual observations.
 * Upon success, coeffs contains the coefficients of the independent
 * variables, and offset contains a constant offset.
 */
bool LeastSquares(const Matrix& data,const Vector& outcome,
		  Vector& coeffs,Real& offset);

/** @brief Calculate a least squares fit of some dependent variable.
 *
 * Upon success, coeffs contains the coefficients of the independent
 * variables, as well as the constant offset in coeffs(d) where d is
 * the index of the dependent variable.
 */
bool LeastSquares(const std::vector<Vector>& data,int dependentVariable,
		  Vector& coeffs);

/** @brief Calculate a least squares fit and pick some dependent variable
 *
 * Upon success, dependentVariable contains the index of the dependent
 * variable, coeffs contains the coefficients of the independent
 * variables, as well as the constant offset in coeffs(d) where d is
 * the index of the dependent variable.
 */
bool LeastSquaresPickDependent(const std::vector<Vector>& data,
			       int& dependentVariable,Vector& coeffs);

} //namespace Statistics

#endif

