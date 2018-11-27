#include <KrisLibrary/Logger.h>
#include "diffeq.h"
#include "vector.h"
using namespace std;

namespace Math {

//steps forward the initial value problem
//y' = f(t,y)
//y(t0) = w
//by the time step h
Real RungeKutta4_step(RealFunction2* f, Real t0, Real h, Real w)
{
	Real k1,k2,k3,k4;
	k1 = h*(*f)(t0,w);
	k2 = h*(*f)(t0+h*Half, w+k1*Half);
	k3 = h*(*f)(t0+h*Half, w+k2*Half);
	k4 = h*(*f)(t0+h, w+k3);
	return w + (k1 + Two*k2 + Two*k3 + k4)/6.0;
}

//solves the initial value problem
//y' = f(t,y)
//y(a) = alpha
//using n iterations of Runge-kutta order 4
Real RungeKutta4(RealFunction2* f, Real a, Real b, Real alpha, int n)
{
	Real w = alpha;
	Real t = a;
	Real h = (b-a)/n;
	for(int i=1; i<n; i++)
	{
		w = RungeKutta4_step(f,t,h,w);
		t+=h;
	}
	return w;
}



//given IVP
//   y' = f(t,y)
//   y(a) = alpha
//does the Runge-Kutta-Fehlberg method with tolerance tol, max step size hmax, min step size hmin
//outputs a matrix W of triplets (ti,wi,hi) (+1 because matrix is 1-based)
Real RKF(RealFunction2* f, Real a, Real b, Real alpha, Real tol, Real hmax, Real hmin)
{
	Real t = a;
	Real w = alpha;
	Real h = hmax;
	int FLAG = 1;
	/*W(1,1) = t;
	W(1,2) = w;
	W(1,3) = h;*/
	int i = 1;

	Real k1,k2,k3,k4,k5,k6,R,delta;
	//define constants for the Runge-Kutta-Fehlberg weights
	const static Real k2_t = 1.0/4.0, k3_t = 3.0/8.0, k4_t = 12.0/13.0, k6_t = 1.0/2.0;
	const static Real k2_k1 = 1.0/4.0;
	const static Real k3_k1 = 3.0/32.0, k3_k2 = 29.0/32.0;
	const static Real k4_k1 = 1932.0/2197.0, k4_k2 = -7200.0/2197.0, k4_k3 = 7296.0/2197.0;
	const static Real k5_k1 = 439.0/216.0, k5_k2 = -8.0, k5_k3 = 3680.0/513.0, k5_k4 = -845.0/4104.0;
	const static Real k6_k1 = -8.0/27.0, k6_k2 = 2.0, k6_k3 = -3544.0/2565.0, k6_k4 = 1859.0/4104.0, k6_k5 = - 11.0/40.0;
	const static Real R_k1 = 1.0/360, R_k3 = - 128.0/4275.0, R_k4 = - 2197.0/75240.0, R_k5 = 1.0/50.0, R_k6 = 2.0/55.0;
	const static Real w_k1 = 25.0/216.0, w_k3 = 1408.0/2565.0, w_k4 = 2197.0/4104.0, w_k5 = - 1.0/5.0;
	while (FLAG == 1)
	{
		k1 = h*(*f)(t,w);
		k2 = h*(*f)(t + k2_t*h, w + k2_k1*k1);
		k3 = h*(*f)(t + k3_t*h, w + k3_k1*k1 + k3_k2*k2);
		k4 = h*(*f)(t + k4_t*h, w + k4_k1*k1 + k4_k2*k2 + k4_k3*k3);
		k5 = h*(*f)(t + h,      w + k5_k1*k1 + k5_k2*k2 + k5_k3*k3 + k5_k4*k4);
		k6 = h*(*f)(t + k6_t*h, w + k6_k1*k1 + k6_k2*k2 + k6_k3*k3 + k6_k4*k4 + k6_k5*k5);

		//compute R = 1/h|Wi+1 - wi+1| where W is the RK5 approximation, w is RK4
		R = Abs(R_k1*k1 + R_k3*k3 + R_k4*k4 + R_k5*k5 + R_k6*k6)/h;
		if (R < tol) {		//accept new approximation, advance
			t = t + h;
			i = i + 1;
			w = w + w_k1*k1 + w_k3*k3 + w_k4*k4 + w_k5*k5;
			/*
			W(i,1) = t;
			W(i,2) = w;
			W(i,3) = h;*/
		}
		
		//calculate new h
		delta = 0.84 * Pow(tol/R, (Real)0.25);		//raise to 1/n where n is the order of the method
		if (delta <= 0.1)			//arbitrary clamping of delta
			h = 0.1*h;
		else if (delta >= 4)
			h = 4*h;
		else
			h = delta*h;

		if (h > hmax)
			h = hmax;

		if (t >= b)
			FLAG = 0;
		else if (t + h > b)
			h = b - t;
		else if (h < hmin) {
			FLAG = 0;
						LOG4CXX_ERROR(KrisLibrary::logger(),"RKF: minimum h exceeded\n");
			return w;
		}
	}

	//success!
	return w;
}


//Adams-Moulton 2 step implicit
//implicit, so it returns w2 (given w2!)
Real AM2I(RealFunction2* f, Real h, Real t0, Real t1, Real t2, Real w0, Real w1, Real w2)
{
	return w1 + h/12*(5*(*f)(t2,w2) + 8*(*f)(t1,w1) - (*f)(t0,w0));
}

//Adams-Bashforth 2 step explicit
//returns w2 given previous 2 time steps
Real AM2E(RealFunction2* f, Real h, Real t0, Real t1, Real w0, Real w1)
{
	return w1 + h/2*(3*(*f)(t1,w1) - (*f)(t0,w0));
}

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

//Adams-Moulton predictor-corrector of order 2
Real AM2_predictor_corrector_step(RealFunction2* f, Real h, Real t0, Real t1, Real t2, Real w0, Real w1)
{
	Real w2;
	w2 = AM2E(f,h,t0,t1,w0,w1);
	w2 = AM2I(f,h,t0,t1,t2,w0,w1,w2);
	return w2;
}

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

Real AM2_predictor_corrector(RealFunction2* f, Real a, Real b, Real alpha0, Real alpha1, int n)
{
	Real h = (b-a)/n;
	Real t = a + h;
	Real w0,w1,w2;
	w0 = alpha0;
	w2 = w1 = alpha1;
	for (int i=2; i<n; i++)
	{
		//make new prediction
		w2 = AM2_predictor_corrector_step(f, h, t-h,t,t+h, w0, w1);

		//step forward
		t = t+h;
		w0 = w1;
		w1 = w2;
	}
	return w2;
}





/////////////////////////////Systems of differential equations///////////////////////////


//steps forward the initial value problem
//y' = f(t,y)
//y(t0) = w0
//by the time step h, placing the result in w1
void Euler_step(DiffEqFunction* f, Real t0, Real h, const Vector& w0, Vector& w1)
{
	Vector k;
	(*f)(t0,w0,k);
	w1=w0;
	w1.madd(k,h);
}

void RungeKutta4_step(DiffEqFunction* f, Real t0, Real h, const Vector& w0, Vector& w1)
{
  Assert(&w0 != &w1);
	Vector k1,k2,k3,k4, tmp;

	(*f)(t0,w0,k1);
	k1 *= h;

	tmp = w0;
	tmp.madd(k1,Half);
	(*f)(t0+h*Half, tmp, k2);
	k2 *= h;

	tmp = w0;
	tmp.madd(k2,Half);
	(*f)(t0+h*Half, tmp, k3);
	k3 *= h;

	tmp.add(w0,k3);
	(*f)(t0+h, tmp, k4);
	k4 *= h;

	tmp.add(k2,k3);
	tmp.add(tmp,tmp);
	w1.add(k1,tmp);
	w1 += k4;
	w1.inplaceMul(1.0/6.0);
	w1 += w0;
}

//solves the initial value problem
//y' = f(t,y)
//y(a) = alpha
//using n iterations of the Euler/Runge-Kutta 4 scheme
void Euler(DiffEqFunction* f, Real a, Real b, const Vector& alpha, int n, Vector& wn)
{
	Vector k;
	wn=alpha;

	Real t = a;
	Real h = (b-a)/n;
	for(int i=0; i<n; i++)
	{
		(*f)(t,wn,k);
		wn.madd(k,h);
		t+=h;
	}
}

void RungeKutta4(DiffEqFunction* f, Real a, Real b, const Vector& alpha, int n, Vector& wn)
{
	//we use a storage space of 2 vectors
	Vector tmp [2];

	Vector* w [2];
	w[0] = &tmp[0];
	w[1] = &tmp[1];

	*w[0] = alpha;
	Real t = a;
	Real h = (b-a)/n;
	for(int i=0; i<n; i++)
	{
		RungeKutta4_step(f,t,h,*w[0],*w[1]);
		//the new value is in w1

		t+=h;

		//swap w0 and w1
		Vector* swp = w[1];
		w[1] = w[0];
		w[0] = swp;
	}
	wn = *w[0];
}

}//namespace Math
