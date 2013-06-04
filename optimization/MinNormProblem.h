#ifndef OPTIMIZATION_MIN_NORM_PROBLEM_H
#define OPTIMIZATION_MIN_NORM_PROBLEM_H

#include "LinearProgram.h"
#include "QuadraticProgram.h"

namespace Optimization {

/** @ingroup Optimization
 * @brief Solves a linearly constrained minimum-norm problem
 *
 * Problem is of the form
 * min ||C x - d||
 * s.t. q <= A x <= p
 *      l <= x <= u
 * where ||.|| is either a 1,2, or infinity norm
 */
struct MinNormProblem : public LinearConstraints
{
  MinNormProblem();
  void ResizeObjective(int m,int n) { C.resize(m,n); d.resize(m); }
  void ResizeConstraints(int m,int n) { LinearConstraints::Resize(m,n); }
  void AddVariables(int num);
  void AddVariable(Real lj=-Inf,Real uj=Inf);
  bool IsValid() const;
  void Print(std::ostream& out) const;
  void Assemble();  ///<converts these matrices into lp/qp form
  LinearProgram::Result Solve(Vector& x);   ///<solves the assembled matrices
  Real Norm(const Vector& x) const;
  
  Real norm;  //either 1,2 or inf
  Matrix C;
  Vector d;
  int verbose;

  //temporary
  LinearProgram lp;
  QuadraticProgram qp;
};

/** @ingroup Optimization
 * @brief Solves a sparse, linearly constrained minimum-norm problem
 *
 * Problem is of the form
 * min ||C x - d||
 * s.t. q <= A x <= p
 *      l <= x <= u
 * where ||.|| is either a 1,2, or infinity norm
 *
 * NOTE: constrained L2-norm not done yet
 */
struct MinNormProblem_Sparse : public LinearConstraints_Sparse
{
  MinNormProblem_Sparse();
  void ResizeObjective(int m,int n) { C.resize(m,n); d.resize(m); }
  void ResizeConstraints(int m,int n) { LinearConstraints_Sparse::Resize(m,n); }
  void AddVariables(int num);
  void AddVariable(Real lj=-Inf,Real uj=Inf);
  bool IsValid() const;
  void Print(std::ostream& out) const;
  void Assemble();  ///<converts these matrices into lp/qp form
  LinearProgram::Result Solve(Vector& x);   ///<solves the assembled matrices
  Real Norm(const Vector& x) const;
  
  Real norm;  //either 1,2 or inf
  SparseMatrix C;
  Vector d;
  int verbose;

  //temporary
  LinearProgram_Sparse lp;
  //QuadraticProgram qp;
};


} //namespace Optimization

#endif
