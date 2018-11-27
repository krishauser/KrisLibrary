#ifndef OPTIMIZATION_LSQR_INTERFACE_H
#define OPTIMIZATION_LSQR_INTERFACE_H

#include <KrisLibrary/Logger.h>
#include <KrisLibrary/math/sparsematrix.h>

namespace Optimization
{
using namespace Math;

/** @ingroup Optimization
 * @brief An interface to a sparse least-squares solver (lsqr).
 */
struct LSQRInterface
{
  LSQRInterface();
  bool Solve(const SparseMatrix& A,const Vector& b);

  //input quantities
  Vector x0;       ///<initial guess for x -- default set to 0's
  Real dampValue;  ///<damping term, min ||Ax-b||^2 + dampValue||x||^2
  Real relError;   ///<relative error in defining A,b
  Real condLimit;  ///<stop if the estimated condition number of A exceeds condLim
  int maxIters;    ///<maximum number of iterations, set to 0 to use default value
  int verbose;     ///<0 - no output printed, 1 - output to stdout, 2 - output to stderr

  //output quantities
  Vector x;        ///<the solution vector
  Vector stdErr;  ///<the standard error estimates
  int numIters;    ///<the number of iterations used
  Real condEstA;   ///<condition number estimate for A
  Real residualNorm; //<the estimate of the final residual's norm
};

} //namespace Optimization

#endif
