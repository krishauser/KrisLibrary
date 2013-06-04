#ifndef OPTIMIZATION_LP_ROBUST_H
#define OPTIMIZATION_LP_ROBUST_H

#include "GLPKInterface.h"
#include "LPSolveInterface.h"
#include "LP_InteriorPoint.h"

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
  void UpdateLPSolve(const LinearProgram& lp);
  void UpdateGLPK(const LinearProgram& lp);
  void UpdateInteriorPoint(const LinearProgram& lp);
  LinearProgram::Result SolveGLPK();
  LinearProgram::Result SolveLPSolve();
  LinearProgram::Result SolveInteriorPoint();

  enum { SOLVE_GLPK, SOLVE_LPSOLVE, SOLVE_IP };
  int solveOrder;
  LPSolveInterface lp_solve;
  GLPKInterface glpk;
  LP_InteriorPoint lpi;
  bool initialized[3];
  int verbose;

  ///temporary variable, stores the output of the solver
  Vector xopt;
};

} //namespace Optimization

#endif
