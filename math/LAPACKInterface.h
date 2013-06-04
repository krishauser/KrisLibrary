#ifndef MATH_LAPACK_INTERFACE_H
#define MATH_LAPACK_INTERFACE_H

#include "MatrixTemplate.h"

namespace Math {

/** @ingroup Math
 * @brief An interface to LAPACK.  Activated with the preprocessor flag
 * HAVE_CLAPACK=1.
 *
 * Not yet complete... who wants to fill this in?
 */
struct LAPACKInterface
{
  ///the following return true if the object can be used directly with fortran
  static bool IsCompliant(const fVector& x);
  static bool IsCompliant(const dVector& x);
  static bool IsCompliant(const cVector& x);
  static bool IsCompliant(const fMatrix& A);
  static bool IsCompliant(const dMatrix& A);
  static bool IsCompliant(const cMatrix& A);
  ///makes a fortran-compliant copy of the object
  static void MakeCompliant(const fVector& A,fVector& out);
  static void MakeCompliant(const dVector& A,dVector& out);
  static void MakeCompliant(const cVector& A,cVector& out);
  static void MakeCompliant(const fMatrix& A,fMatrix& out);
  static void MakeCompliant(const dMatrix& A,dMatrix& out);
  static void MakeCompliant(const cMatrix& A,cMatrix& out);

  ///solves the linear equation A*x = b
  static bool Solve(const fMatrix& A,const fVector& b,fVector& x);
  static bool Solve(const dMatrix& A,const dVector& b,dVector& x);
  ///solve a least-squares problem (if overdetermined) or minimum-norm
  ///problem (if underdetermined)
  static bool LeastSquares(const fMatrix& A,const fVector& b,fVector& x);
  static bool LeastSquares(const dMatrix& A,const dVector& b,dVector& x);

  ///get the eigenvalues of a symmetric matrix
  static bool Eigenvalues_Symmetric(const fMatrix& A,fVector& lambda);
  static bool Eigenvalues_Symmetric(const dMatrix& A,dVector& lambda);
  ///get the eigenvalues+eigenvectors of a symmetric matrix A.
  ///eigenvectors stored in columns of Q.
  static bool Eigenvectors_Symmetric(const fMatrix& A,fVector& lambda,fMatrix& Q);
  static bool Eigenvectors_Symmetric(const dMatrix& A,dVector& lambda,dMatrix& Q);

  static bool SVD(const fMatrix& A,fMatrix& U,fVector& W,fMatrix& Vt);
  static bool SVD(const dMatrix& A,dMatrix& U,dVector& W,dMatrix& Vt);
};

} //namespace Math

#endif
