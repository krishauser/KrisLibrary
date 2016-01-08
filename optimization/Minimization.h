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
  ConvergenceResult SolveLM(int& maxIters,Real lambda0=1,Real lambdaScale=10);

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

  ///Performs a line search to minimize f(x) in the direction dx.
  ConvergenceResult LineMinimizationStep(Vector& dx,Real& alpha0);

  ScalarFieldFunction* f;
  Vector bmin,bmax;
  Vector x;
  Real tolx,tolf,tolgrad;

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
 * Performs the line search while maintaining feasibility on the constraint 
 * manifold.  The initial point must lie in the feasibility set.
 * This isn't really a well established minimization
 * technique and hasn't been tested thoroughly.
 *
 * StepTR performs a sequential linear programming step with a given
 * trust region.  We could implement SQP, but other libraries would probably
 * be better.
 *
 * If sparse is set to true, C and D are assumed to be subclasses of
 * SparseVectorFunction.  Undefined behavior if this does not hold!
 */
struct ConstrainedMinimizationProblem
{
  //ConstrainedMinimizationProblem(ScalarFieldFunction* f,ImplicitManifold*M);
  ConstrainedMinimizationProblem(ScalarFieldFunction* f,VectorFieldFunction*C,VectorFieldFunction*D);

  ///Gradient descent
  ConvergenceResult SolveGD(int& maxIters);
  ///Returns MaxItersReached on normal exit
  ConvergenceResult StepGD();

  ///Newton iteration  
  ConvergenceResult SolveNewton(int& maxIters);
  ///Returns MaxItersReached on normal exit
  ConvergenceResult StepNewton();

  ///Gradient-based trust-region method
  ///Returns MaxItersReached on normal exit, LocalMinimum if R should be increased, Divergence if it should be reduced
  ConvergenceResult StepTR(Real R);

  ///Helper to project grad onto the constraint nullspace 
  void NullspaceProjection(const Vector& x,Vector& dx);
  ///Helper for line minimzation -- like above, but on the manifold surface
  ConvergenceResult LineMinimizationStep(const Vector& dx,Real& alpha0);
  ///Helper to check point feasibility
  bool CheckPoint(const Vector& x) const;
  ///Helper to solve for a feasible point
  bool SolveFeasiblePoint(Vector& x,int maxIters,ConvergenceResult* res=NULL);

  ScalarFieldFunction* f;
  VectorFieldFunction* C,*D;
  Vector bmin,bmax;
  Vector x;
  Real tolx,tolf,tolgrad,tolc;
  bool sparse;

  //output options
  int verbose;
  std::vector<Vector>* S;

  //temporary
  Real fx;
  Vector grad;
  Matrix H;
  ConstrainedNewtonRoot rootSolver;
};

} //namespace Optimization

#endif
