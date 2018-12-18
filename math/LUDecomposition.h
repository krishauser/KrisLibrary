#ifndef MATH_LU_DECOMPOSITION_H
#define MATH_LU_DECOMPOSITION_H

#include "MatrixTemplate.h"
#include <vector>

namespace Math {

/** @ingroup Math
 * @brief Forms the LU decomposition A=PLU
 *
 * L is stored in the lower sub-diagonal triangle of LU,
 * and formed to have ones on the diagonal.
 * U is stored in the upper triangle of LU.
 * P^-1 is stored as a permutation vector
 */
template <class T>
class LUDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;

  LUDecomposition();
  LUDecomposition(const MatrixT& A);
  
  bool set(const MatrixT& A);
  void backSub(const VectorT& b, VectorT& x) const;
  void PBackSub(const VectorT& b, VectorT& x) const;
  void LBackSub(const VectorT& b, VectorT& x) const;
  void UBackSub(const VectorT& b, VectorT& x) const;
  void getInverse(MatrixT& Ainv) const;
  
  void getL(MatrixT& L) const;
  void getU(MatrixT& U) const;

  MatrixT LU;
  std::vector<int> P;
  T zeroTolerance;
};

}
#endif
