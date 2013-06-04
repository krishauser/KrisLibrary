#ifndef MATH_CONDITIONER_H
#define MATH_CONDITIONER_H

#include "matrix.h"
#include "DiagonalMatrix.h"
#include "linalgebra.h"

/** @file math/Conditioner.h
 * @ingroup Math
 * @brief Numerical conditioners for a matrix equation A*x=b
 *
 * A conditioner solves a transformed equation A'*x'=b' with better 
 * properties (and such that x is readily available from x').
 *
 * One example is CG preconditioners (these are in conjgrad.h),
 * Uniform scaling: A' = cA, b' = c*b.
 * Matrix premultiply: A' = C*A, b' = C*b.
 *   or postmultiply: A' = A*C, x' = C^-1*x.
 * 
 * Would like to retain symmetricity for symmetric matrices,
 * so can use A' = C*A*C^t, x' = C^-t*x, b' = C*b.
 *
 * The implementation is such that on initialization, 
 * the matrices A and b are immediately altered to obtain A',b'.
 * x' is solved for with any method in in MatrixEquation,
 * and x is obtained from x' using the Post() method.
 */

namespace Math {
/** @addtogroup Math */
/*@{*/

/// A matrix conditioner that does a pre/postmultiply with a diagonal S
struct Conditioner_SymmDiag : public MatrixEquation
{
  /// An enum for how to calculate the scaling. 
  enum Method { None, NormalizeDiagonal };
  Conditioner_SymmDiag(Matrix& A,Vector& b,Method method);

  void Post(Vector& x) const;
  void CalculateS(Method m);
  void CalculateS_NormalizeDiagonal();

  DiagonalMatrix S;
};

/*@}*/
} //namespace Math

#endif
