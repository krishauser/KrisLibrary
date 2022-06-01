#ifndef OPTIMIZATION_MINIMIZATION_H
#define OPTIMIZATION_MINIMIZATION_H

#include <KrisLibrary/math/matrix.h>
#include <KrisLibrary/math/function.h>
#include <KrisLibrary/math/root.h>
//#include "Manifold.h"
#include "Newton.h"

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief An iterative solution to minimize f over multidimensional x.
 * May only find a local minimum, depending on the initial point.
 *
 * Optionally, can include bound constraints
 *
 * Methods: <br>
 *   Gradient descent (GD):  move by alpha*gradient. <br>
 *   Steepest descent (SD):  alpha*gradient. <br>
 *   Conjugate gradient (CG): use the CG method 
 *      (Fletcher-Reeves-Polak-Ribiere) <br>
 *   Newton iteraton (Newton): use Newton's method (requires Hessians) 
 *   Quasi-Newton iteraton (QuasiNewton): use a quasi-Newton method
 *      (requires Gradient)
 *   Levenberg-marquardt (LM): use Levenberg-marquardt method with starting 
 *      lambda and lambda update scale factor (requires Hessian)
 *   Levenberg-marquardt (LMTrust): same as above, but uses a trust region
 *      search
 *
 * Each method starts at the initial point x.  It then determines a direction
 * d and a step size alpha that f(x+alpha*d)<f(x).  It then moves x to this 
 * position and iterates until maxIters is reached, or one of the 3
 * termination criteria is met: <br>
 * change in x < tolx <br>
 * change in f < tolf <br>
 * ||gradient|| < tolgrad.<br>
 * The appropriate ConvergenceResult is returned (tolf and tolgrad both 
 * return ConvergenceF).  tolx is by default 1e-6, tolf is 1e-5, and
 * tolgrad is 1e-8
 *
 * maxIters is returned as the number of iterations taken.
 * 
 * If S is not NULL it logs the sequence of iterates x0...xn, starting
 * from x1.
 */
struct MinimizationProblem
{
  MinimizationProblem(ScalarFieldFunction* f);

  ///Gradient descent
  ConvergenceResult SolveGD(Real alpha,int& maxIters);
  void StepGD(Real alpha);

  ///Steepest descent
  ConvergenceResult SolveSD(int& maxIters);

  ///Conjugate gradient
  ConvergenceResult SolveCG(int& maxIters);

  ///Newton iteration
  ConvergenceResult SolveNewton(int& maxIters);

  ///Quasi-newton iteration, with differing initial hessian methods.
  ///Identity hessian
  ConvergenceResult SolveQuasiNewton_Ident(int& maxIters);
  ///Finite-difference hessian with difference dx
  ConvergenceResult SolveQuasiNewton_Diff(Real dx,int& maxIters);
  ///Hessian given in H
  ConvergenceResult SolveQuasiNewton(int& maxIters);

  ///Levenberg-Marquardt iteration
  ConvergenceResult SolveLM(int& maxIters,Real lambda0=1,Real lambdaGrow=2,Real lambdaShrink=3);

  ///Performs a line search to minimize f(x) in the direction dx.
  ///fx must be set to the initial f(x).  f is updated to the final x
  ///fx is set to the final f(x).  Alpha0 is given as the initial step size
  ///and returned as the final step size.
  ///
  ///Returns the convergence result for the given tolerances or
  ///MaxItersReached to continue.
  ConvergenceResult LineMinimizationStep(const Vector& dx,Real& alpha0);
  //TODO: different types of line minimization
  //ConvergenceResult LineMinimization_Bisection(const Vector& dx,Real& alpha0);
  //ConvergenceResult LineMinimization_Parabolic(const Vector& dx,Real& alpha0);

  ScalarFieldFunction* f;
  Vector x;
  Real tolx,tolf,tolgrad;
  Real fbreak; ///< stop when the objective function goes below this value

  //output options
  int verbose;
  std::vector<Vector>* S;

  //temporary
  Real fx;
  Vector grad,dx;
  Matrix H;
};

/** @ingroup Optimization
 * @brief A bound-constrained MinimizationProblem.  Uses an active-set
 * method to determine the components of the free search direction at x.
 */
struct BCMinimizationProblem 
{
  BCMinimizationProblem(ScalarFieldFunction* f);

  ///Steepest descent
  ConvergenceResult SolveSD(int& maxIters);

  ///Newton iteration
  ConvergenceResult SolveNewton(int& maxIters);

  ///Quasi-newton iteration, with differing initial hessian methods.
  ///Identity hessian
  ConvergenceResult SolveQuasiNewton_Ident(int& maxIters);
  ///Finite-difference hessian with difference dx
  ConvergenceResult SolveQuasiNewton_Diff(Real dx,int& maxIters);
  ///Hessian given in H
  ConvergenceResult SolveQuasiNewton(int& maxIters);

  ///Performs Levenberg-Marquardt on a least squares problem min ||f(x)||^2.  Converges
  ///better than newton or quasi-newton as df/dx approaches singular.
  ConvergenceResult SolveLM(VectorFieldFunction* vf,int& maxIters,Real lambda0=1,Real lambdaGrow=2,Real lambdaShrink=3);

  ///Performs a line search to minimize f(x) in the direction dx.
  ConvergenceResult LineMinimizationStep(Vector& dx,Real& alpha0);

  ScalarFieldFunction* f;
  Vector bmin,bmax;
  Vector x;
  Real tolx,tolf,tolgrad;
  Real fbreak; ///< stop when the objective function goes below this value (and x is feasible)

  //output options
  int verbose;
  std::vector<Vector>* S;

  //temporary
  Real fx;
  Vector grad;
  Matrix H;
  std::vector<bool> activeSet; 
};

/** @ingroup Optimization
 * @brief Same as MinimizationProblem but with nonlinear constraints
 * C(x)=0 and D(x)<=0, bound constraints bmin/bmax
 *
 * Both inequalities and inequalities are not fully-implemented.
 */
struct ConstrainedMinimizationProblem
{
  ConstrainedMinimizationProblem(ScalarFieldFunction* f,VectorFieldFunction*C,VectorFieldFunction*D);

  ///Augmented Lagrangian newton iteration  
  ConvergenceResult SolveAugmentedLangrangian(int& maxIters);
  ///Returns MaxItersReached on normal exit
  ConvergenceResult StepAugmentedLangrangian(Real mu,const Vector& lambda_c,const Vector& lambda_d);

  ///Sequential quadratic programming. Currently only implemented for equality constraints.
  ConvergenceResult SolveSQP(int& maxIters);
  ///Returns MaxItersReached on normal exit
  ConvergenceResult StepSQP(Real &alpha);

  ///Returns true if x is a feasible point. Checks ||C(X)||_inf<=tol_c and D(X)<=-tol_d. 
  ///If tol_c<=0, uses this->tolc as the equality testing tolerance. 
  bool Feasible(const Vector& x,Real tol_c=0,Real tol_d=0);
  ///Same as feasible but uses current values of cx, dx
  bool CurrentFeasible(Real tol_c=0,Real tol_d=0);

  ScalarFieldFunction* f;
  VectorFieldFunction* C,*D;
  Vector bmin,bmax;
  Vector x;
  Real tolx,tolf,tolgrad,tolc;
  int innerIters;
  Real fbreak; ///< stop when the constraints are satisfied and the objective function goes below this value (and x is feasible)

  BCMinimizationProblem augmentedLagrangianProblem;

  //output options
  int verbose;
  std::vector<Vector>* S;

  //temporary
  Real fx;
  Vector cx,dx;
};

} //namespace Optimization

#endif
