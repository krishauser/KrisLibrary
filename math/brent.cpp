#include "brent.h"
#include "vectorfunction.h"

namespace Math {

Real ParabolicExtremum(Real a, Real b, Real c, Real fa, Real fb, Real fc)
{
  Real bc = b-c;
  Real ba = b-a;
  Real fbc = fb-fc;
  Real fba = fb-fa;
  Real num = ba*ba*fbc-bc*bc*fba;
  Real den = ba*fbc-bc*fba;
  if(den==Zero) return (den*num>Zero ? -Inf : Inf);
  return b - Half*num/den;
}

#define GOLD (Real)1.618034
#define GLIMIT (Real)100.0
#define CGOLD (Real)0.3819660
#define SHFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b) (b>=Zero?Abs(a):-Abs(a))

/*
Given a function func, and given distinct initial points ax and bx, this routine searches in
the downhill direction (defined by the function as evaluated at the initial points) and returns
new points ax, bx, cx that bracket a minimum of the function. Also returned are the function
values at the three points, fa, fb, and fc.
*/
void BracketMin(Real &ax, Real &bx, Real &cx, Real& fa, Real& fb, Real& fc, RealFunction& func)
{
  Real ulim,u,fu;
  fa=func(ax);
  fb=func(bx);
  if (fb > fa) { //Switch roles of a and b so that we can go downhill in the direction from a to b.
    std::swap(ax,bx);
    std::swap(fa,fb);
  }
  cx=bx+GOLD*(bx-ax); //First guess for c.
  fc=func(cx);
  while (fb > fc) { //Keep returning here until we bracket.
    u=ParabolicExtremum(ax,bx,cx,fa,fb,fc);
    ulim=bx+GLIMIT*(cx-bx);  //We won't go farther than this. Test various possibilities:
    if ((bx-u)*(u-cx) > 0.0) { //Parabolic u is between b and c: try it.
      fu=func(u);
      if (fu < fc) { //Got a minimum between b and c.
        ax=bx;  bx=u;
        fa=fb;  fb=fu;
        return;
      }
      else if (fu > fb) { //Got a minimum between between a and u.
        cx=u;
        fc=fu;
        return;
      }
      u=cx+GOLD*(cx-bx); //Parabolic fit was no use. Use default magnification.
      fu=func(u);
    }
    else if ((cx-u)*(u-ulim) > 0.0) { //Parabolic fit is between c and its allowed limit.
      fu=func(u);
      if (fu < fc) {
        SHFT(bx,cx,u,cx+GOLD*(cx-bx))
        SHFT(fb,fc,fu,func(u))
      }
    }
    else if ((u-ulim)*(ulim-cx) >= 0.0) { //Limit parabolic u to maximum allowed value.
      u=ulim;
      fu=func(u);
    }
    else { //Reject parabolic u, use default magnification.
      u=cx+GOLD*(cx-bx);
      fu=func(u);
    }
    SHFT(ax,bx,cx,u) //Eliminate oldest point and continue.
    SHFT(fa,fb,fc,fu)
  }
}


ConvergenceResult ParabolicMinimization(Real ax,Real bx,Real cx,RealFunction& f,int& iters,Real tol,Real& xmin)
{
  Real a,b,d=0,etemp,fu,fv,fw,fx,p,q,r,tol1,tol2,u,v,w,x,xm;
  Real e=0.0; //This will be the distance moved on the step before last.
  a=(ax < cx ? ax : cx); //a and b must be in ascending order, but input abscissas need not be.
  b=(ax > cx ? ax : cx);
  x=w=v=bx; 
  fw=fv=fx=f(x);
  int maxIters=iters;
  for (iters=1;iters<=maxIters;iters++) { //Main program loop.
    xm=Half*(a+b);
    tol1=tol*Abs(x)+Epsilon;
    tol2=Two*tol1;
    if (Abs(x-xm) <= (tol2-Half*(b-a))) { //Test for done here.
      xmin = x;
      return ConvergenceX;
      //return fx;
    }
    if (Abs(e) > tol1) { //Construct a trial parabolic fit.
      r=(x-w)*(fx-fv);
      q=(x-v)*(fx-fw);
      p=(x-v)*q-(x-w)*r;
      q=Two*(q-r);
      if (q > 0.0) p = -p;
      q=Abs(q);
      etemp=e;
      e=d;
      if (Abs(p) >= Abs(Half*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
        d=CGOLD*(e=(x >= xm ? a-x : b-x));
      //The above conditions determine the acceptability of the parabolic fit. Here we
      //take the golden section step into the larger of the two segments.
      else {
        d=p/q; //Take the parabolic step.
        u=x+d;
        if (u-a < tol2 || b-u < tol2)
          d=SIGN(tol1,xm-x);
      }
    } else {
      d=CGOLD*(e=(x >= xm ? a-x : b-x));
    }
    u=(Abs(d) >= tol1 ? x+d : x+SIGN(tol1,d));
    fu=f(u);
    if (fu <= fx) { //Now decide what to do with our function evaluation. 
      if (u >= x) a=x; else b=x;
      SHFT(v,w,x,u)
      SHFT(fv,fw,fx,fu)
    }
    else { //Housekeeping follows:
      if (u < x) a=u; else b=u;
      if (fu <= fw || w == x) {
        v=w; w=u;
        fv=fw; fw=fu;
      }
      else if (fu <= fv || v == x || v == w) {
        v=u;
        fv=fu;
      }
    } //Done with housekeeping. 
    //Back for another iteration.
  }
  xmin = x;
  return MaxItersReached;
  //return fx;
}

ConvergenceResult ParabolicMinimization(Real x,RealFunction& f,int& maxIters,Real tol,Real& xmin)
{
  Real a,b,c,fa,fb,fc;
  a=x;
  b=x+One;
  BracketMin(a,b,c,fa,fb,fc,f);
  return ParabolicMinimization(a,b,c,f,maxIters,tol,xmin);
}

Real ParabolicLineMinimization(ScalarFieldFunction& f,const Vector& x,const Vector& n,int maxIters,Real tol)
{
  ScalarFieldDirectionalFunction pf(f,x,n);
  Real dx;
  //Real f=
  ParabolicMinimization(0,pf,maxIters,tol,dx);
  return dx;
}

Real ParabolicLineMinimization_i(ScalarFieldFunction& f,const Vector& x,int i,int maxIters,Real tol)
{
  ScalarFieldProjectionFunction pf(f,x,i);
  Real dx;
  //Real f=
  ParabolicMinimization(Zero,pf,maxIters,tol,dx);
  return dx;
}

} //namespace Math
