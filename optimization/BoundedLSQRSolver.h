#ifndef BOUNDED_LSQR_SOLVER_H
#define BOUNDED_LSQR_SOLVER_H

#include "LinearProgram.h"

namespace Optimization {

/**@brief Solve the bounded least-squares problem
 * min||Ax-b||^2 s.t. l <= x <= b
 * accepts infinite constraints too.
 */
class BoundedLSQRSolver
{
 public:
  BoundedLSQRSolver(const Matrix& A,const Vector& b,const Vector& l,const Vector& u);
  LinearProgram::Result Solve(Vector& x);

  Matrix A;
  Vector b;
  Vector l,u;

  int verbose;
  Real fTol,gradTol,xTol;
  int maxIters;
};

} //namespace Optimization

#endif
