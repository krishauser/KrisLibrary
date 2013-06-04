#ifndef OPTIMIZATION_LP_SOLVE_INTERFACE_H
#define OPTIMIZATION_LP_SOLVE_INTERFACE_H

#include "LinearProgram.h"

struct _lprec;

namespace Optimization {

/** @ingroup Optimization
 * @brief An interface to the LPSolve linear program solver.  Activated with
 * the HAVE_LPSOLVE preprocessor define.
 */
struct LPSolveInterface
{
  LPSolveInterface();
  ~LPSolveInterface();
  void Set(const LinearProgram& LP);
  void SetMatrix(const Matrix& A);
  void SetObjective(const Vector& c);
  void SetObjectiveBreak(Real val);
  LinearProgram::Result Solve(Vector& xopt);

  static bool Enabled();
  static void SelfTest();

  _lprec* lp;
};

} //namespace Optimization

#endif
