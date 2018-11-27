#include <KrisLibrary/Logger.h>
#include "root.h"
#include <iostream>
#include "SVDecomposition.h"
#include <errors.h>

namespace Math {

std::ostream& operator<<(std::ostream& out, ConvergenceResult r)
{
  switch(r) {
  case ConvergenceX: out<<"converged on X"; break;
  case ConvergenceF: out<<"converged on F"; break;
  case Divergence: out<<"divergence"; break;
  case LocalMinimum: out<<"local minimum"; break;
  case MaxItersReached: out<<"max iterations reached"; break;
  case ConvergenceError: out<<"convergence error"; break;
  default: out<<"undefined convergence"; break;
  }
  return out;
}

ConvergenceResult Root_Bisection(RealFunction& f,Real& a, Real& b, Real& x, Real tol)
{
  Real fa,fb,fx,tol2;
  fa = f(a);
  fb = f(b);
  if(fa == 0) { x=b=a; return ConvergenceX; }
  if(fb == 0) { x=a=b; return ConvergenceX; }
  if(Sign(fa) == Sign(fb)) return ConvergenceError;  //must have opposing signs
  tol2 = tol*Two;
  int maxIters = (int)Ceil((Log(b-a)-Log(tol))/Ln2);
  x = Half*(a+b);
  for(int i=0;i<maxIters;i++) {
    fx = f(x);
    if(Sign(fx) == Sign(fa)) {  //shift a to x
      a = x; fa = fx;
    }
    else if(Sign(fx) == Sign(fb)) { //shift b to x
      b = x; fb = fx;
    }
    else {
      if(fx == 0) { return ConvergenceX;  }
      else {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Weird error in Root_Bisection(), perhaps function not stateless?\n");
	return ConvergenceError;
      }
    }
    x = Half*(a+b);
  }
  return MaxItersReached;
}

ConvergenceResult Root_Secant(RealFunction& f,Real x0, Real& x1, int& iters, Real tolx, Real tolf)
{
  Real x,fx0,fx1;
  fx0 = f(x0);
  fx1 = f(x1);
  if(Abs(fx0) <= tolf) { x1=x0; return ConvergenceF; }
  if(Abs(fx1) <= tolf) { return ConvergenceF; }

  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    if(fx0-fx1 == 0) return ConvergenceError;
    x = (x1*fx0-x0*fx1)/(fx0-fx1);
    if(Abs(x-x1) <= tolx) {
      x1=x;
      return ConvergenceX; 
    }

    x0 = x1; fx0 = fx1;
    x1 = x; fx1 = f(x1);
    if(Abs(fx1) <= tolf) { return ConvergenceF; }
  }
  return MaxItersReached;
}

ConvergenceResult Root_SecantBracket(RealFunction& f,Real& a, Real& b, Real& x, int& iters, Real tolx, Real tolf)
{
  Real fx,fa,fb;
  fa = f(a);
  fb = f(b);
  if(Abs(fa) <= tolf) { x=b=a; return ConvergenceF; }
  if(Abs(fb) <= tolf) { x=a=b; return ConvergenceF; }
  if(Sign(fa) == Sign(fb)) return ConvergenceError;  //must have opposing signs

  //keep x in the [a,b] bracket
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    if(fa-fb == 0) return ConvergenceError;
    x = (b*fa-a*fb)/(fa-fb);
    Assert(x >= a && x <= b);  //can't leave bracket
    if(x-a <= tolx || b-x <= tolx)
      return ConvergenceX; 

    fx = f(x);
    if(Abs(fx) <= tolf) return ConvergenceF;
    if((fx<Zero) == (fa<Zero)) {
      a = x;
      fa = fx;
    }
    else {
      Assert((fx<Zero) == (fb<Zero));
      b = x;
      fb = fx;
    }
  }
  return MaxItersReached;
}

ConvergenceResult Root_Newton(RealFunction& f,Real& x, int& iters, Real tolx, Real tolf)
{
  Real fx,dfx,dx;
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    f.PreEval(x);
    fx = f.Eval(x);
    if(Abs(fx) < tolf) 
      return ConvergenceF;
    dfx = f.Deriv(x);
    if(dfx == 0)   //derivative is 0
      return ConvergenceError;
    dx = fx/dfx;
    x -= dx;
    if(Abs(dx) <= tolx)   //reached tolerance
      return ConvergenceX;
  }
  return MaxItersReached;
}

ConvergenceResult Root_NewtonBracket(RealFunction& f,Real a, Real b, Real& x,int& iters, Real tolx, Real tolf)
{
  Real fx,dfx,dx;
  x = Half*(a+b);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    f.PreEval(x);
    fx = f.Eval(x);
    if(Abs(fx) < tolf) 
      return ConvergenceF;
    dfx = f.Deriv(x);
    if(dfx == 0)   //derivative is 0
      return ConvergenceError;
    dx = fx/dfx;
    x -= dx;
    if((x-a)*(b-x) < Zero) //leaves bracket
      return Divergence;
    if(Abs(dx) <= tolx)   //reached tolerance
      return ConvergenceX;
  }
  return MaxItersReached;
}

ConvergenceResult Root_Newton(ScalarFieldFunction& f,const Vector& x0, Vector& x, int& iters, Real tolx, Real tolf)
{
  //move in gradient direction to set f to 0
  //f(x) ~= f(x0) + df/dx(x0).(x-x0) + O(h^2)
  //=> 0 = fx + grad.(x-x0) = fx + grad.grad*h => h = -fx/grad.grad
  Real fx;
  Vector grad(x.n);
  if(&x != &x0) x = x0;
  int maxIters=iters;
  for (iters=0;iters<maxIters;iters++) { 
    f.PreEval(x);
    fx = f.Eval(x);
    f.Gradient(x,grad);
    //Check function convergence.
    if (Abs(fx) <= tolf) return ConvergenceF;
    Real gSquared = grad.normSquared();
    if(FuzzyZero(gSquared)) return LocalMinimum;
    x.madd(grad,-fx/gSquared);
    if (Abs(grad.maxAbsElement()*fx/gSquared) <= tolx) return ConvergenceX;
  } 
  return MaxItersReached;
}

ConvergenceResult Root_Newton(VectorFieldFunction& f,const Vector& x0, Vector& x, int& iters, Real tolx, Real tolf)
{
  //move in gradient direction to set f to 0
  //f(x) ~= f(x0) + df/dx(x0)*(x-x0) + O(h^2)
  //=> 0 = fx + J*(x-x0) => dx = -J^-1*fx
  SVDecomposition<Real> svd;
  Vector fx,p;
  Matrix fJx;
  if(&x != &x0) x = x0;
  int maxIters=iters;
  for (iters=0;iters<maxIters;iters++) {
    f.PreEval(x);
    f.Eval(x,fx);
    f.Jacobian(x,fJx);
    if(fx.maxAbsElement() <= tolf) return ConvergenceF;
    if(!svd.set(fJx)) {
      return ConvergenceError;
    }
    svd.backSub(fx,p);
    x -= p;
    if(p.maxAbsElement() <= tolx) return ConvergenceX;
  }
  return MaxItersReached;
}

}//namespace Math
