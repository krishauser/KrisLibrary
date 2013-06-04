#ifndef QP_INTERIOR_POINT_H
#define QP_INTERIOR_POINT_H

#include "QuadraticProgram.h"

namespace Optimization {

/** @ingroup Optimization
 * @brief Interior-point solution to a quadratic program
 */
class QPInteriorPoint
{
public:
  QPInteriorPoint(const QuadraticProgram& qp);
  void SetInitialPoint(const Vector& x0);
  bool Solve();
  bool FindFeasiblePoint(Vector& x0,int verbose=0) const;
  //returns objective function including the inequality log-barrier
  Real Objective_Ineq(const Vector& x, Real t) const;
  const Vector& GetOptimum() const { return x; }
  void SetStopCriterion();

  const QuadraticProgram& qp;
  Real tol_zero, tol_inner, tol_outer;
  int maxOuterIters,maxInnerIters;
  bool stopWhenObjectiveIsNegative;
  int verbose;

  Vector x0;
  Vector x,lambda;
  Matrix Aineq,Aeq;
  Vector bineq,beq;
};

} //namespace Optimization

#endif
