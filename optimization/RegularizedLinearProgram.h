#ifndef OPTIMIZATION_MIN_NORM_PROBLEM_H
#define OPTIMIZATION_MIN_NORM_PROBLEM_H

#include "LinearProgram.h"
#include "QuadraticProgram.h"

namespace Optimization {

/** @ingroup Optimization
 * @brief Solves a linear program with regularization
 *
 * Problem is of the form
 * min c^t x + ||C x - d||
 * s.t. q <= A x <= p
 *      l <= x <= u
 * where ||.|| is either a 1 or infinity norm
 */
struct RegularizedLinearProgram : public LinearConstraints
{
  RegularizedLinearProgram();
  void ResizeObjective(int m,int n) { C.resize(m,n); d.resize(n); }
  void ResizeConstraints(int m,int n) { LinearConstraints::Resize(m,n); }
  bool IsValid() const;
  void Print(std::ostream& out) const;
  void Assemble();  ///<converts these matrices into lp form
  LinearProgram::Result Solve(Vector& x);   ///<solves the assembled matrices
  Real Objective(const Vector& x) const;
  Real Norm(const Vector& x) const;
  Real ObjectiveNormSum(const Vector& x) const;

  Vector c;   //objective vector
  Real norm;  //either 1 or inf
  Matrix C;
  Vector d;
  int verbose;

  //temporary
  LinearProgram lp;
};

} //namespace Optimization

#endif
