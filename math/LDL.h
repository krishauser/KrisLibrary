#ifndef MATH_LDL_H
#define MATH_LDL_H

#include "matrix.h"

namespace Math {

/** @ingroup Math
 * @brief Performs the LDL^t decompositoin of a symmetric matrix A.
 *
 * L is stored in the lower diagonal of LDL, D is stored in the diagonal.
 * If any of the elements of D's diagonal have abs value less than 
 * zeroTolerance (i.e. A is singular), then they are ignored.
 */
template <class T>
struct LDLDecomposition
{
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;

  LDLDecomposition();
  LDLDecomposition(const MatrixT& A);

  void set(const MatrixT& A);
  bool backSub(const VectorT& b, VectorT& x) const;
  void LBackSub(const VectorT& b, VectorT& x) const;
  void LTBackSub(const VectorT& b, VectorT& x) const;
  bool DBackSub(const VectorT& b, VectorT& x) const;
  bool backSub(const MatrixT& B, MatrixT& X) const;
  bool getInverse(MatrixT& Ainv) const;
  void getPseudoInverse(MatrixT& Ainv) const;
  void getL(MatrixT& L) const;
  void getD(VectorT& d) const;
  void getA(MatrixT& A) const;
  void mulL(const Vector& x,Vector& y) const;
  void mulLT(const Vector& x,Vector& y) const;
  void mulD(const Vector& x,Vector& y) const;

  /// Update the LDL decomposition for A + xx^t
  void update(const VectorT& x);
  /// "Downdate" the LDL decomposition for A - xx^t (on failure, LDL is undefined)
  bool downdate(const VectorT& x);

  MatrixT LDL;
  T zeroTolerance;
  int verbose;
};

} // namespace Math

#endif
