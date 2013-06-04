#ifndef MATH_LAPACK_INTERFACE_H
#define MATH_LAPACK_INTERFACE_H

#include "MatrixTemplate.h"

namespace Math {

/** @ingroup Math
 * @brief An interface to BLAS.  Activated with the preprocessor flag
 * HAVE_BLAS=1.
 */
struct BLASInterface
{
  /// Level 1 BLAS
  static float Dot(const fVector& x,const fVector& y);
  static double Dot(const dVector& x,const dVector& y);
  static float Norm_L2(const fVector& x);
  static double Norm_L2(const dVector& x);
  static float Norm_L1(const fVector& x);
  static double Norm_L1(const dVector& x);
  static int MaxAbsIndex(const fVector& x);
  static int MaxAbsIndex(const dVector& x);
  static void Madd(fVector& v,const fVector& x,float a);
  static void Madd(dVector& v,const dVector& x,double a);
  static void InplaceMul(fVector& v,float a);
  static void InplaceMul(dVector& v,double a);

  /// Level 2 BLAS
  static void Mul(const fMatrix& A,const fVector& x,fVector& out);
  static void Mul(const dMatrix& A,const dVector& x,dVector& out);
  static void MulTranspose(const fMatrix& A,const fVector& x,fVector& out);
  static void MulTranspose(const dMatrix& A,const dVector& x,dVector& out);
  //y = alpha*A*x + beta*y
  static void Madd(const fMatrix& A,const fVector& x,fVector& y,float alpha=1.0,float beta=1.0);
  static void Madd(const dMatrix& A,const dVector& x,dVector& y,double alpha=1.0,double beta=1.0);
  //y = alpha*A^T*x + beta*y
  static void MaddTranspose(const fMatrix& A,const fVector& x,fVector& y,float alpha=1.0,float beta=1.0);
  static void MaddTranspose(const dMatrix& A,const dVector& x,dVector& y,double alpha=1.0,double beta=1.0);

  /// Level 3 BLAS
  static void Mul(const fMatrix& A,const fMatrix& B,fMatrix& X);
  static void Mul(const dMatrix& A,const dMatrix& B,dMatrix& X);
  //X = alpha*op(A)*op(B) + beta*X
  static void Madd(const fMatrix& A,const fMatrix& B,fMatrix& X,bool Atranspose=false,bool Btranspose=false,float alpha=1.0,float beta=1.0);
  static void Madd(const dMatrix& A,const dMatrix& B,dMatrix& X,bool Atranspose=false,bool Btranspose=false,double alpha=1.0,double beta=1.0);
};

} //namespace Math

#endif
