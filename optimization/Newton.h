#ifndef OPTIMIZATION_NEWTON_H
#define OPTIMIZATION_NEWTON_H

#include <KrisLibrary/math/function.h>
#include <KrisLibrary/math/root.h>
#include <KrisLibrary/math/SVDecomposition.h>
#include <vector>

namespace Math {
  template <class T> class SparseMatrixTemplate_RM;
  typedef SparseMatrixTemplate_RM<Real> SparseMatrix;
}

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief A globally convergent Newton's method for multidimensional root
 * solving.  Solves for func(x)=0.
 *
 * Can use sparse methods for solving for a SparseVectorFieldFunction if
 * sparse = true.  However, cannot use the bias-vector option.
 */
class NewtonRoot
{
public:
  NewtonRoot(VectorFieldFunction* func);
  virtual ~NewtonRoot();
  bool GlobalSolve(int& iters,ConvergenceResult* res=NULL);
  ConvergenceResult Solve(int& iters);
  ConvergenceResult Solve_Sparse(int& iters);
  bool LineMinimization(const Vector& g, const Vector& p, Real *f);
  Real MaxDistance(const Vector& x);
  virtual bool SolveUnderconstrainedLS(const Matrix& A,const Vector& b,Vector& x);
  virtual bool SolveUnderconstrainedLS(const SparseMatrix& A,const Vector& b,Vector& x);
  virtual Real Merit();  //evaluates the merit function at x

  Vector x;
  VectorFieldFunction* func;
  Real tolf,tolmin,tolx;
  Real stepMax;  ///< maximum distance to step
  Real lambda;   ///< damped-least-squares constant 
  Vector bmin,bmax; ///< optional bound constraints
  bool sparse;   ///< set to true if should use a sparse least-squares solver. 
  Vector bias;   ///< set this to a bias vector to solve for min ||x-bias|| s.t. func(x)=0
  int verbose;
  int debug;

  //temporary
  RobustSVD<Real> svd;
  Vector fx, g, p, xold;
  Matrix fJx;
};

/** @ingroup Optimization
 * @brief A globally convergent Newton's method with inequality constraints
 * c(x) >= 0.
 */
class ConstrainedNewtonRoot : public NewtonRoot
{
public:
  ConstrainedNewtonRoot(VectorFieldFunction* func,VectorFieldFunction* c);
  bool GlobalSolve(int& iters,ConvergenceResult* res=NULL);
  ConvergenceResult SolveConstrained(int& iters);
  ConvergenceResult SolveConstrained2(int& iters);
  ConvergenceResult SolveConstrained2_Sparse(int& iters);
  ConvergenceResult SolveConstrained_SLP(int& iters);
  virtual Real Merit();

  VectorFieldFunction* c;
  Real tolc;

  //temporary
  Vector cx;
  std::vector<int> activeSetC,activeSetBound;
  Matrix A;  //solve for A*dx = rhs
  Vector rhs;
};

///Returns C(x).maxElement()
Real SurfaceDistance(VectorFieldFunction*C,const Vector& x);
///Returns true if C(x)<=tol, pointwise
bool SatisfiesEquality(VectorFieldFunction*C,const Vector& x,Real tol=Epsilon);
///Returns true if C(x) >= margin
bool SatisfiesInequality(VectorFieldFunction*C,const Vector& x,Real margin=Zero);
///Returns C(x).minElement() (and the index if non=NULL)
Real InequalityMargin(VectorFieldFunction* c,const Vector& x,int* index=NULL);

} //namespace Optimization

#endif
