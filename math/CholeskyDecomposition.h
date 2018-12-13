#ifndef MATH_CHOLESKY_DECOMPOSITION_H
#define MATH_CHOLESKY_DECOMPOSITION_H

#include "matrix.h"

namespace Math {

/** @ingroup Math
 * @brief Performs the Cholesky decomposition.
 *
 * Decomposes the positive semi-definite matrix A to LL^t.
 */
template <class T>
class CholeskyDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;

  CholeskyDecomposition();
  CholeskyDecomposition(const MatrixT& A);
  CholeskyDecomposition(const MatrixT& A,MatrixT& L);

  void setDestination(MatrixT& L);
  bool set(const MatrixT& A);
  bool setPerturbed(const MatrixT& A,T& lambda);
  void backSub(const VectorT& b, VectorT& x) const;
  void LBackSub(const VectorT& b, VectorT& x) const;
  void LTBackSub(const VectorT& b, VectorT& x) const;
  void backSub(const MatrixT& B, MatrixT& X) const;
  void getInverse(MatrixT& Ainv) const;

  /// Update the cholesky decomposition for A + xx^t
  void update(const VectorT& x);
  /// "Downdate" the cholesky decomposition for A - xx^t (on failure, L is undefined)
  bool downdate(const VectorT& x);

  MatrixT L;
  T zeroEpsilon;
};

}
#endif
