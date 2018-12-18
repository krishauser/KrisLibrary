#ifndef MATH_QR_DECOMPOSITION_H
#define MATH_QR_DECOMPOSITION_H

#include "MatrixTemplate.h"

namespace Math {

/** @ingroup Math
 * @brief Calculates the QR decomposition.
 *
 * Factorise an M x N matrix A into A = Q R, where Q is orthogonal
 * (M x M) and R is upper triangular (M x N).
 *
 * Q is stored as a packed set of Householder transformations in the
 * strict lower triangular part of the input matrix.  R is stored in
 * the diagonal and upper triangle of the input matrix.
 *
 * The full matrix for Q can be obtained as the product
 *
 *       Q = Q_k .. Q_2 Q_1
 *
 * where k = MIN(M,N) and
 *
 *       Q_i = (I - tau_i * v_i * v_i')
 *
 * and where v_i is a Householder vector
 *
 *       v_i = [1, m(i+1,i), m(i+2,i), ... , m(M,i)]
 *
 * This storage scheme is the same as in LAPACK.  
 */
template <class T>
class QRDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  bool set(const MatrixT& A);
  void backSub(const VectorT& x,VectorT& b) const;
  void leastSquares(const VectorT& x,VectorT& b,VectorT& residual) const;
  void QMul(const VectorT& b,VectorT& x) const;
  void QtMul(const VectorT& b,VectorT& x) const;
  void RBackSub(const VectorT& b,VectorT& x) const;
  void getQ(MatrixT& Q) const;
  void getR(MatrixT& R) const;
  
  MatrixT QR;
  VectorT tau;
};

/** @ingroup Math
 * @brief The QR decomposition as computed by the algorithm
 * in Numerical Recipes in C.
 */
template <class T>
class NRQRDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  bool set(const MatrixT& A);
  void backSub(const VectorT& x,VectorT& b) const;
  void QBackSub(const VectorT& x,VectorT& b) const;
  void getQ(MatrixT& Q) const;
  
  MatrixT QR;
  VectorT c;
  bool singular;
};

} //namespace Math

#endif
