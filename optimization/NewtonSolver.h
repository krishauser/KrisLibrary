#ifndef OPTIMIZATION_NEWTON_SOLVER_H
#define OPTIMIZATION_NEWTON_SOLVER_H

#include "NonlinearProgram.h"
#include <math/root.h>

namespace Optimization {

struct InequalityBarrierNLP;

/** @ingroup Optimization
 * @brief A newton's-method style solver for a NLP.
 *
 * NOTE: Not tested very thoroughly.
 *
 * TODO: bound constraints bmin,bmax
 * TODO: make this work for inequality constraints d(x) <= 0
 */
struct NewtonSolver
{
  NewtonSolver(NonlinearProgram& nlp);
  ~NewtonSolver();
  ///Call to initialize the solver after setting the initial point x
  void Init();
  ///Use newton's method to solve the NLP using the given number of iters
  ConvergenceResult Solve(int& maxIters);
  ///Performs a line search in direction (dx,da).
  ///returns MaxItersReached on normal exit
  ConvergenceResult LineSearch(Real& alpha);
  Real Merit(const Vector& x,const Vector& a) const;

  ///state - x is the variable, a is the lagrange multipliers
  NonlinearProgram& origProblem;
  InequalityBarrierNLP* barrierProblem;
  NonlinearProgram* problem;
  Vector x,a;
  Vector dx,da;

  ///user settings
  Vector bmin,bmax;
  Real tolf,tolx;
  int verbose;
  std::vector<Vector>* S;

  ///temp
  Matrix H,Hinv;
  Matrix A,AHA,mtemp;
  Vector res1,res2,vtemp;
};

} //namespace Optimization

#endif
