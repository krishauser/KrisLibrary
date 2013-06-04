#ifndef MATH_ROOT_H
#define MATH_ROOT_H

#include "function.h"

/** @file root.h
 * @ingroup Math
 * @brief Numerical root-solving routines.
 */

namespace Math {
  /** @addtogroup Math */
  /*@{*/

enum ConvergenceResult { ConvergenceX, ConvergenceF, Divergence, LocalMinimum, MaxItersReached, ConvergenceError };

std::ostream& operator<<(std::ostream&, ConvergenceResult r);

/** @brief Uses bisection to bracket f's root.
 *
 * [a,b] must be a range such that f(a) and f(b) have opposing signs.
 * [a,b] is returned as a refined bracket.
 * Linear convergence.
 * The number of iterations used is ceil(log2(b-a)-log2(eps)).
 */
ConvergenceResult Root_Bisection(RealFunction& f,Real& a, Real& b, Real& x, Real tol);

/** @brief Uses the secant method to search for f's root.
 *
 * Order of convergence ~1.6. The # of iterations is returned in maxIters.
 */
ConvergenceResult Root_Secant(RealFunction& f,Real x0, Real& x1, int& maxIters, Real tolx, Real tolf);

///Same as above, but the interval [a,b] must be known to contain a root. 
///[a,b] is returned as a refined bracket.
ConvergenceResult Root_SecantBracket(RealFunction& f,Real& a, Real& b, Real& x, int& maxIters, Real tolx, Real tolf);

/** @brief Performs Newton's method, which uses the derivative of f to 
 * search for it's root.
 *
 * f.Deriv() must be defined.  Quadratic convergence.  The # of iterations 
 * is returned in maxIters.
 */
ConvergenceResult Root_Newton(RealFunction& f,Real& x, int& maxIters, Real tolx, Real tolf);

/// Same as above, but the interval [a,b] must be known to contain a root.
ConvergenceResult Root_NewtonBracket(RealFunction& f,Real a, Real b, Real& x, int& maxIters, Real tolx, Real tolf);

/// Same as Root_Newton(), but takes a scalar field function as input
ConvergenceResult Root_Newton(ScalarFieldFunction& f,const Vector& x0, Vector& x, int& maxIters, Real tolx, Real tolf);


/// Same as Root_Newton(), but takes a vector field function as input
ConvergenceResult Root_Newton(VectorFieldFunction& f,const Vector& x0, Vector& x, int& maxIters, Real tolx, Real tolf);

  /*@}*/
} // namespace Math

#endif
