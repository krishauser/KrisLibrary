#ifndef OPTIMIZATION_QP_ROBUST_H
#define OPTIMIZATION_QP_ROBUST_H

#include "QuadraticProgram.h"

namespace Optimization {

/** @ingroup Optimization
 * @brief A class that tries out as many available routines as possible 
 * to solve a QP.
 */
struct RobustQPSolver
{
  RobustQPSolver();
  void Clear();

  LinearProgram::Result Solve(const QuadraticProgram& qp);

  int verbose;

  ///temporary variable, stores the output of the solver
  Vector xopt;
};

} //namespace Optimization

#endif
