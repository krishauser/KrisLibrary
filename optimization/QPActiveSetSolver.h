#ifndef OPTIMIZATION_QP_ACTIVE_SET_H
#define OPTIMIZATION_QP_ACTIVE_SET_H

#include "QuadraticProgram.h"
#include <math/CholeskyDecomposition.h>
#include <math/LDL.h>
#include <math/root.h>
#include <vector>

namespace Optimization {

/** @ingroup Optimization
 * @brief Active-set solution to a dense positive definite quadratic program
 *
 * I haven't implemented any anti-cycling procedure, and don't allow
 * linearly dependent constraints, or a semidefinite objective.
 *
 * This works ok for small problems, but is not recommended.  QPOPT is much
 * more reliable.
 */
class QPActiveSetSolver
{
public:
  QPActiveSetSolver(const QuadraticProgram& qp);
  ConvergenceResult Solve();
  ConvergenceResult Step();
  bool FindFeasiblePoint();
  bool SolveCurOptimum(Vector& x,Vector& u);
  void AddActiveSet(int i,bool lowerBound);   //i = row of Aineq to add
  void RemoveActiveSet(int index);  //index = remove activeSet[index] from active set
  bool ComputeHinv();

  const QuadraticProgram& qp;
  Real tol_zero;
  int maxIters;
  bool stopWhenObjectiveIsNegative;
  int verbose;

  Vector x;
  std::vector<int> activeSet;
  std::vector<bool> isActive;

  //temporary
  //Ax=b is the current active set equalities
  //Hinv is the cholesky decomposition of
  //[Pinv At]
  //[A    0 ]
  Matrix A,mtemp,mtemp2;
  Vector b,vtemp,vtemp2;
  LDLDecomposition<Real> Hinv;
};

} //namespace Optimization

#endif
