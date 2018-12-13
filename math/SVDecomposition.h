#ifndef MATH_SV_DECOMPOSITION_H
#define MATH_SV_DECOMPOSITION_H

#include "MatrixTemplate.h"
#include "DiagonalMatrix.h"

namespace Math { 

/** @ingroup Math
 * @brief Performs the singular value decomposition.
 *
 * Decomposes the matrix A to UWV^t.  The values in the diagonal matrix W
 * are the singular values.  The backsubstitution functions can either
 * ignore zero singular values (within tolerance epsilon) or used a
 * "damped" backsubstitution where W is replaced with W+lambda*I.
 *
 * The nullspace matrix is the matrix N such that for each column n,
 * A*n = 0.
 */
template <class T>
class SVDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  typedef DiagonalMatrixTemplate<T> DiagonalMatrixT;

  SVDecomposition();
  SVDecomposition(const MatrixT& A);
  
  bool set(const MatrixT& A);
  void setIdentity(int m,int n);
  void setZero(int m,int n);
  void resize(int m,int n);
  void clear();
  void backSub(const VectorT& b, VectorT& x) const;
  void dampedBackSub(const VectorT& b,T lambda, VectorT& x) const;
  void nullspaceComponent(const VectorT& x,VectorT& xNull) const;
  int getRank() const;
  int getNull() const;
  void getInverse(MatrixT& Ainv) const;
  void getDampedPseudoInverse(MatrixT& Aplus,T lambda) const;
  void getNullspace(MatrixT& N) const;

  ///sorts the singular values from highest to lowest
  void sortSVs();
  
  MatrixT U;
  DiagonalMatrixT W;
  MatrixT V;

  //settings
  int maxIters;
  T epsilon;
};

/** @ingroup Math
 * @brief Performs a pre/postconditioned singular value decomposition.
 *
 * Performs the SVD on the matrix A' = Pre^-1*A*Post^-1 such that
 * A = Pre*U*W*Vt*Post.
 *
 * Also zero's out small elements of A' with tolerance zeroElementEpsilon.
 *
 * @see SVDecomposition
 */
template <class T>
class RobustSVD
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  typedef DiagonalMatrixTemplate<T> DiagonalMatrixT;

  RobustSVD();
  RobustSVD(const MatrixT& A);
  
  bool set(const MatrixT& A);
  bool setConditioned(const MatrixT& A);
  void setIdentity(int m,int n);
  void setZero(int m,int n);
  void resize(int m,int n);
  void clear();
  void backSub(const VectorT& b, VectorT& x) const;
  void dampedBackSub(const VectorT& b,T lambda, VectorT& x) const;
  void nullspaceComponent(const VectorT& x,VectorT& xNull) const;
  int getRank() const;
  int getNull() const;
  void getInverse(MatrixT& Ainv) const;
  void getDampedPseudoInverse(MatrixT& Aplus,T lambda) const;
  void getNullspace(MatrixT& N) const;
  void calcConditioning(const MatrixT& A);
  
  DiagonalMatrixT Pre;
  SVDecomposition<T> svd;
  DiagonalMatrixT Post;

  //settings
  T zeroElementEpsilon;
  bool preMultiply,postMultiply;
};

} //namespace Math

#endif
