#ifndef MATH_BRENT_H
#define MATH_BRENT_H

#include "function.h"
#include "root.h"
#include <memory>

/** @file math/brent.h
 * @ingroup Math
 * @brief Brent's methods for parabolic minimization of a 1D function.
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

/** @brief Returns a bracketing triplet given some a,b.
 *
 * A bracketing triplet [a,b,c] is such that b is between a and c,
 * and f(b) is less than both f(a) and f(c).
 * The function values are returned as well.
 */
void BracketMin(Real &a, Real &b, Real &c, Real& fa, Real& fb, Real& fc, RealFunction& func);

/** @brief Brent's algorithm for parabolic minimization.
 *
 * Brackets the minimum to a fractional precision of 
 * about tol.  [ax,bx,cx] are a bracketing triplet. 
 * The abscissa of the minimum is returned
 * as xmin, and the result of convergence is returned
 * (either ConvergenceX or MaxItersReached).
 * The minimum function value is not returned...
 * The number of iterations taken is returned in maxIters.
 *
 * The second version only takes a starting point, and
 * finds a minimum from there.
 */
ConvergenceResult ParabolicMinimization(Real ax,Real bx,Real cx,RealFunction& f,int& maxIters,Real tol,Real& xmin);
ConvergenceResult ParabolicMinimization(Real x,RealFunction& f,int& maxIters,Real tol,Real& xmin);

///Finds a minimum t* of f(x + t*n), using parabolic minimization
///Other parameters are the same as ParabolicMinimization.
Real ParabolicLineMinimization(ScalarFieldFunction& f,const Vector& x,const Vector& n,int maxIters,Real tol);

///Same as above, but minimizes f(x + t*ei).
Real ParabolicLineMinimization_i(ScalarFieldFunction& f,const Vector& x,int i,int maxIters,Real tol);

/*@}*/

} //namespace Math

#endif
