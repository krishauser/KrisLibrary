
#ifndef OPTIMIZATION_LP_INTERIORPOINT_H
#define OPTIMIZATION_LP_INTERIORPOINT_H

#include "LinearProgram.h"

namespace Optimization {

/** @ingroup Optimization
 * @brief An interior-point method for solving a LP.
 *
 * Only solves problems in the form
 * min		f'*x               <br>
 *  s.t.	A x <= p
 *
 * Can be used when no initial feasible point
 * has been specified.
 *
 * NOTE: variable bounds, lower bound constraints,
 * and maximization option are ignored.  To handle these, use
 * LP_InteriorPoint.
 *
 * NOTE: Not recently tested.  Use at your own risk.
 *
 * @sa LP_InteriorPoint
 */
class LP_InteriorPointSolver : public LinearProgram
{
public:
  enum Result { Optimal, SubOptimal, OptimalNoBreak, Infeasible, MaxItersReached, Error };

  LP_InteriorPointSolver();
  void SetInitialPoint(const Vector& x0);
  Result Solve();
  const Vector& GetX0() const { return x0; }
  const Vector& GetOptimum() const { return xopt; }
  void SetObjectiveBreak(double val) { objectiveBreak=val; }

  double tol_zero, tol_inner, tol_outer;
  double objectiveBreak;   //stop when f'*x < objectiveBreak
  int verbose;

private:
  double Objective_Ineq(const Vector& x, double t);
  bool FindFeasiblePoint();

  Vector x0;
  Vector xopt;
};

/** @ingroup Optimization
 * @brief Solves a general LP using LP_InteriorPointSolver.
 *
 * Transforms the original LP into one in the reduced form.
 *
 * -# Decompose equality constraints Aeq*x = beq into x = x0 + N*y.
 * -# Transform the remaining LP to optimize y using eq 1).
 * -# If maximize, set the negative objective.
 */
struct LP_InteriorPoint
{
  LP_InteriorPoint();
  bool Set(const LinearProgram& lp);
  LP_InteriorPointSolver::Result Solve();

  void SetObjective(const Vector& c);
  void SetInitialPoint(const Vector& xinit);
  void GetInitialPoint(Vector& xinit) const;
  void GetOptimum(Vector& xopt) const;
  void SetObjectiveBreak(double val);

  Vector x0;
  Matrix N;
  Real foffset;  //f'*x0
  LP_InteriorPointSolver solver;
};

} //namespace Optimization

#endif
