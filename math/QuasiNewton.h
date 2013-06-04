#ifndef QUASI_NEWTON_H
#define QUASI_NEWTON_H

#include "LDL.h"

namespace Math {

/** @ingroup Math
 * @brief Maintains the Cholesky decomposition of the hessian H over
 * Quasi-Newton steps.
 *
 * BFGS sets H' = H + q*qt / qt*s - Ht*s*st*H/st*H*s
 * DFS sets H' = H + s*st / st*q - Ht*q*qt*H/qt*H*q
 * where s = x - x0
 * q = grad - grad0
 *
 * BFGS is usually considered more stable than the latter.
 */
struct QNHessianUpdater
{
  ///set the initial hessian matrix
  void SetHessian(const Matrix& H) { ldl.set(H); }
  ///form the hessian matrix
  void GetHessian(Matrix& H) const { ldl.getA(H); }
  ///solve x=H^-1*b
  void BackSub(const Vector& b,Vector& x) const { ldl.backSub(b,x); }
  bool IsValidUpdate(const Vector& s,const Vector& q) { return (s.dot(q)>0); }
  bool UpdateBFGS(const Vector& s,const Vector& q);
  bool UpdateDFS(const Vector& s,const Vector& q);

  LDLDecomposition<Real> ldl;
  int verbose;

  //temporary
  Vector temp,Hs,upd;
};


} //namespace Math

#endif
