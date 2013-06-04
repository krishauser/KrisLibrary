#ifndef MATH_DIFFEQ_H
#define MATH_DIFFEQ_H

#include "function.h"

/** @file math/diffeq.h
 * @ingroup Math
 * @brief Various methods for solving ordinary differential equations.
 */

namespace Math {
/** @addtogroup Math */
/*@{*/

/** @brief Runge-Kutta 4 step for 1D ODEs
 *
 * Steps forward the initial value problem
 *
 * y' = f(t,y) <br>
 * y(t0) = w
 *
 * by the time step h.
 */
Real RungeKutta4_step(RealFunction2* f, Real t0, Real h, Real w);

/** @brief Runge-Kutta 4 method for 1D ODEs
 *
 * Solves the initial value problem
 *
 * y' = f(t,y) <br>
 * y(a) = alpha
 *
 * using n iterations of Runge-kutta order 4 to reach time b.
 */
Real RungeKutta4(RealFunction2* f, Real a, Real b, Real alpha, int n);

/** @brief Runge-Kutta-Fehlberg method for 1D ODEs
 *
 * For the IVP
 *
 * y' = f(t,y) <br>
 * y(a) = alpha
 *
 * does the Runge-Kutta-Fehlberg method with tolerance tol, max 
 * step size hmax, min step size hmin to reach time b.
 */
Real RKF(RealFunction2* f, Real a, Real b, Real alpha, Real tol, Real hmax, Real hmin);

/** @brief Adams-Moulton 2 step implicit method for 1D ODEs
 *
 * Implicit, so it returns w2 (given w2!)
 */
Real AM2I(RealFunction2* f, Real h, Real t0, Real t1, Real t2, Real w0, Real w1, Real w2);

/** @brief Adams-Bashforth 2 step explicit method for 1D ODEs
 *
 * Returns w2 given previous 2 time steps
 */
Real AM2E(RealFunction2* f, Real h, Real t0, Real t1, Real w0, Real w1);

/*
function w2 = AM2_implicit_step(f, df, h, t0, t1, t2, w0, w1)
	//define a function j with a root at w2
	f1 = f(t1,w1);
	f0 = f(t0,w0);
	function y = j(w)
		y = w1 + h/12*(5*f(t2,w) + 8*f1 - f0) - w;
	endfunction
	//define dj = j'
	function y = dj(w)
		y = h/12*(5*df(t2,w)) - 1;
	endfunction
	function y = g(w)
		y = w - j(w)/dj(w);
	endfunction

	//perform newton iteration, tol 10^-4, max 10 steps
	//w2 = newton(j,dj,w1, 1e-4,10);
	w2 = fixed_point(g,w1,1e-5,10);
endfunction
*/

/** @brief Adams-Moulton predictor-corrector step of order 2 for 1D ODEs */
Real AM2_predictor_corrector_step(RealFunction2* f, Real h, Real t0, Real t1, Real t2, Real w0, Real w1);

/*
W = AM2_implicit(RealFunction* f, RealFunction* df, Real a, Real b, Real alpha0, Real alpha1, int n)
{
	h = (b-a)/n;
	W(1) = alpha0;
	W(2) = alpha1;
	t = a + 2*h;
	for i=2:n
		W(i+1) = AM2_implicit_step(f,df, h, t-h,t,t+h, W(i-1), W(i));
	end
}
*/

/** @brief Adams-Moulton predictor-corrector of order 2 for 1D ODEs */
Real AM2_predictor_corrector(RealFunction2* f, Real a, Real b, Real alpha0, Real alpha1, int n);



/////////////////////////////Systems of differential equations///////////////////////////


/** @brief Euler step for an ODE system.
 *
 * Steps forward the ODE system
 *
 * y' = f(t,y) <br>
 * y(t0) = w0
 *
 * by the time step h, placing the result in w1.
 */
void Euler_step(DiffEqFunction* f, Real t0, Real h, const Vector& w0, Vector& w1);

/** @brief Runge-Kutta 4 step for an ODE system.
 *
 * Steps forward the ODE system
 *
 * y' = f(t,y) <br>
 * y(t0) = w0
 *
 * by the time step h, placing the result in w1.
 */
void RungeKutta4_step(DiffEqFunction* f, Real t0, Real h, const Vector& w0, Vector& w1);

/** @brief Solve an ODE system using Euler's method
 *
 * Steps forward the ODE system
 *
 * y' = f(t,y) <br>
 * y(a) = alpha
 *
 * using n iterations of Euler's method to reach time b.
 */
void Euler(DiffEqFunction* f, Real a, Real b, const Vector& alpha, int n, Vector& wn);

/** @brief Solve an ODE system using the Runge Kutta 4 method
 *
 * Steps forward the ODE system
 *
 * y' = f(t,y) <br>
 * y(a) = alpha
 *
 * using n iterations of Runge Kutta 4 to reach time b.
 */
void RungeKutta4(DiffEqFunction* f, Real a, Real b, const Vector& alpha, int n, Vector& wn);

/*@}*/
}//namespace Math
#endif
