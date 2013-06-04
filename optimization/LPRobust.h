#ifndef OPTIMIZATION_LP_ROBUST_H
#define OPTIMIZATION_LP_ROBUST_H

#include "GLPKInterface.h"

namespace Optimization {

/** @ingroup Optimization
 * @brief A class that tries out as many available routines as possible 
 * to solve an LP.
 */
struct RobustLPSolver
{
  RobustLPSolver();
  void Clear();

  LinearProgram::Result Solve(const LinearProgram& lp);
  LinearProgram::Result Solve(const LinearProgram_Sparse& lp);
  LinearProgram::Result Solve_NewObjective(const LinearProgram& lp);
  LinearProgram::Result Solve_NewObjective(const LinearProgram_Sparse& lp);
  void UpdateGLPK(const LinearProgram& lp);
  LinearProgram::Result SolveGLPK();

  GLPKInterface glpk;
  bool initialized;
  int verbose;

  ///temporary variable, stores the output of the solver
  Vector xopt;
};

} //namespace Optimization

#endif
