#include <KrisLibrary/Logger.h>
#include "Newton.h"
#include <utils.h>
#include <math/infnan.h>
#include <math/linalgebra.h>
#include <math/AABB.h>
#include <math/indexing.h>
#include <math/sparsefunction.h>
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
#include "MinNormProblem.h"
#include "LSQRInterface.h"
#include <iostream>
using namespace std;
using namespace Optimization;

#define USE_ACTIVE_SET 1
#define USE_SLP 1
const static Real kInequalityAdjustment=1e-4;

namespace Optimization {


Real SurfaceDistance(VectorFieldFunction*C,const Vector& x)
{
  Vector temp(C->NumDimensions());
  (*C)(x,temp);
  return temp.maxAbsElement();
}

bool SatisfiesEquality(VectorFieldFunction*C,const Vector& x,Real tol)
{
  Vector temp(C->NumDimensions());
  (*C)(x,temp);
  return temp.maxAbsElement()<=tol;
}

//returns true if C(x) >= margin
bool SatisfiesInequality(VectorFieldFunction*C,const Vector& x,Real margin)
{
  Vector temp(C->NumDimensions());
  (*C)(x,temp);
  return temp.minElement()>=margin;
}

Real InequalityMargin(VectorFieldFunction* c,const Vector& x,int* index)
{
  c->PreEval(x);
  Vector temp(c->NumDimensions());
  c->Eval(x,temp);
  return temp.minElement(index);
}

} //namespace Optimization


NewtonRoot::NewtonRoot(VectorFieldFunction* _func)
  :func(_func),tolf(1e-4),tolmin(1e-6),tolx(1e-7),stepMax(10),lambda(0.01),
   sparse(false),
   verbose(0),debug(0)
{
}

NewtonRoot::~NewtonRoot()
{}

Real NewtonRoot::Merit()
{
  (*func)(x,fx);
  return Half*fx.normSquared(); 
}

Real NewtonRoot::MaxDistance(const Vector& x)
{
  fx.resize(func->NumDimensions());
  (*func)(x,fx);
  return fx.maxAbsElement();
}

bool NewtonRoot::GlobalSolve(int& iters,ConvergenceResult* r)
{
  if(verbose) { LOG4CXX_INFO(KrisLibrary::logger(),"NewtonRoot::GlobalSolve(): "); }  //Vector xinit;
  //xinit.copy(x);
  Real initDist = MaxDistance(x);
  if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"initial distance "<<initDist<<". ");
  ConvergenceResult res;
  if(sparse)
    res=Solve_Sparse(iters);
  else
    res=Solve(iters);
  if(r) *r=res;
  Real endDist = MaxDistance(x);

  switch(res) {
  case ConvergenceX:
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached convergence on x in "<<iters<<" iters... ");
    if(endDist <= tolf) {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"satisfies constraint.");
      return true;
    }
    else {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"does not satisfy tolerance, distance "<<endDist<<".");
      return false;
    }
    break;

  case LocalMinimum:
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached local minimum in "<<iters<<" iters... ");
    if(endDist <= tolf) {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"satisfies constraint.");
      return true;
    }
    else {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"stuck at distance "<<endDist<<".");
      return false;
    }

  case ConvergenceF:
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached convergence on f in "<<iters<<" iters, new distance "<<endDist);
    Assert(endDist <= tolf);
    return true;

  case MaxItersReached:
    if(endDist < initDist) {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Max iters reached, distance was decreased to "<<endDist);
    }
    else {
      //if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Max iters reached, looks like divergence.  Reverting to initial.");
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Max iters reached, looks like divergence.");
      //x.copy(xinit);
    }
    return false;
  default:
    if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"Error");
    return false;
  }
}

bool NewtonRoot::SolveUnderconstrainedLS(const Matrix& A,const Vector& b,Vector& x)
{
  if(sparse) {
    SparseMatrix sA;
    //Real zeroTol=1e-6*A.maxAbsElement();  //tolerance for zero-sized entries in A
    Real zeroTol=Max(1e-6,1e-7*A.maxAbsElement());  //tolerance for zero-sized entries in A
    sA.set(A,zeroTol);
    return SolveUnderconstrainedLS(sA,b,x);
  }
  else {
    svd.resize(A.m,A.n);
    if(verbose>=1 && A.m*A.n>10000) LOG4CXX_INFO(KrisLibrary::logger(),"Calculating SVD...");
    if(svd.set(A)) {
      if(verbose>=1 && A.m*A.n>10000) LOG4CXX_INFO(KrisLibrary::logger(),"done");
      svd.dampedBackSub(b,lambda,x);
      //svd.epsilon = lambda;
      //svd.backSub(vtemp,p);
      return true;
    }
    return false;
  }
}

bool NewtonRoot::SolveUnderconstrainedLS(const SparseMatrix& A,const Vector& b,Vector& x)
{
  Optimization::LSQRInterface lsqr;
  //A.mulTranspose(b,lsqr.x);
  lsqr.dampValue = lambda;
  lsqr.relError = tolx;
  //lsqr.dampValue=0;
  lsqr.verbose=0;
  if(lsqr.Solve(A,b)) {
    if(!IsFinite(lsqr.x)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonRoot::SolveUnderconstrainedLS: Warning, LSQR returned a non-finite solution");
      LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(lsqr.x,VectorPrinter::AsciiShade));
      KrisLibrary::loggerWait();
      return false;
    }
    //LOG4CXX_INFO(KrisLibrary::logger(),"NewtonRoot::SolveUnderconstrainedLS: LSQR residual is "<<lsqr.residualNorm);
    x=lsqr.x;
    return true;
  }
  //Hmm.. should we try a non-converged x?
  x=lsqr.x;
  return true;
  return false;
}

/*
min_x (x - d)^T W (x - d)
  s.t. g(x) = 0

Newton approach to solving g(x)=0 starts with point x[0]
takes step:
  x[n+1] = x[n] + alpha[n+1]*p[n]
where the descent direction p[n] = -dg/dx(x[n])^+*g(x[n])
is based on the Taylor expansion about x[n]:
   g(x) ~= g(x[n]) + dg/dx(x[n])*(x-x[n])
and using a line search to determine alpha[n+1].  Denote
Jn = dg/dx(x[n]) and gn = g(x[n]).

There are many directions that set the Taylor expansion
to 0, and we can use the null space to bias toward the
center d.  I.e., we may solve:

  min_p (p+x[n]-d)^T W (p+x[n]-d)
    s.t. Jn p = -gn

Generically this is a problem 
  min_x (x-y)^T W (x-y)
    s.t. Ax = b

With W symmetric we can compute a cholesky decomposition W = U^T U
with U invertible
  min_x (Ux-Uy)^T  (Ux-Uy)
    s.t. Ax = b
Let z = Ux - Uy, then x = U^-1 z + y
We want to solve
  min_z ||z||^2
    s.t. AU^-1 z = b - Ay
The solution is given by the pseudoinverse z = (AU^-1)^+ (b-Ay)
and thus x = U^-1 (AU^-1)^+ (b-Ay) + y

In the special case where W = I => U = I, and hence
x = A^+ (b-Ay) + y = A^+ b + (1-A^+ A) y  (this latter part is the projection operator)

Going back to newton solving, we have
p = -Jn^+ gn + (1-Jn^+ Jn) (d-x[n])

Another way to look at this is a weak least-squares problem:
min ||g(x)||^2 + epsilon*(x - d)^T W (x - d)

Problem is now to do the line search. alpha[n] is determined through
the merit function ||g(x)||^2 in normal Newton solving.
*/

ConvergenceResult NewtonRoot::Solve(int& iters)
{
  int m=func->NumDimensions();
  fx.resize(m);
  fJx.resize(m,x.n);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);

  bool check;
  Real f=Merit(); //fx is also computed by this call. 
  if (fx.maxAbsElement() < tolf) { 
    iters=0;
    Real fxmax=fx.maxAbsElement();
    Assert(MaxDistance(x) == fxmax);
    return ConvergenceF;
  } 
  Real stpmax= stepMax*Max(x.norm(),(Real)x.n);
  int maxIters = iters;
  for (iters=0;iters<maxIters;iters++) { 
    if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"Iteration "<<iters<<", x = "<<x);
    if(verbose >= 2) LOG4CXX_ERROR(KrisLibrary::logger(),"   errors"<<fx);
    func->Jacobian(x,fJx);
    fJx.mulTranspose(fx,g);
    xold.copy(x);
    //if x at the bounds, examine descent direction, set those directions
    //to be constrained
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(x(i) == bmin(i) && g(i) > 0) { fJx.setCol(i,0.0); g(i) = 0; }
	if(x(i) == bmax(i) && g(i) < 0) { fJx.setCol(i,0.0); g(i) = 0; }
      }
    }

    //solve newton step
    if(!SolveUnderconstrainedLS(fJx,fx,p)) {
      //least squares
      MatrixEquation eq(fJx,fx);
      if(!eq.LeastSquares_Cholesky(p)) {
        if(verbose >= 2) {
          LOG4CXX_INFO(KrisLibrary::logger(),"NewtonRoot::Solve: Unable to compute either pseudoinverse or Cholesky least-squares\n");
        }
        return ConvergenceError;
      }
    }
    p.inplaceNegative();
    //TEST: make length of step decrease with larger error?
    //p *= 1.0 / (1.0 + fx.norm()/fx.n);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Step size: "<<1.0 / (1.0 + fx.norm());
    if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"  Descent direction "<<p);
    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    Real gnorm2 = g.dot(g);
    Real gp = g.dot(p);
    if(gp > 0) {
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"  Error in slope and descent directions? Check jacobian");
	LOG4CXX_INFO(KrisLibrary::logger(),"  g: "<<g);
	LOG4CXX_INFO(KrisLibrary::logger(),"  p: "<<p);
	LOG4CXX_ERROR(KrisLibrary::logger(),"  Error: "<<fx);
	MatrixPrinter printer(fJx);
	LOG4CXX_INFO(KrisLibrary::logger(),"  Jacobian: ");
	printer.Print(cout,4);
        cout<<endl;
      }
      //p.setNegative(g);
      //blend p and -g so that a sufficient descent direction is obtained
      //p' = u*p - (1-u)g
      //g^T p' = -mu*g^T g
      //u*g^T p  = (1-u-mu)*g^T g
      //u  = (1-u-mu) * g^T g / g^T p
      //u  = (1-mu) * g^T g / g^T p - u g^T g / g^T p
      //(1+g^T g / g^T p) u = (1-mu) * g^T g / g^T p
      //u = (1-mu) * g^T g / (g^T p+g^T g)
      Real mu = 0.1;
      Real u = (1-mu)*gnorm2 / (gp+gnorm2);
      p *= u;
      p.madd(g,u-1);
      Assert(g.dot(p) < 0);
    }
    check = LineMinimization(g,p,&f); //lnsrch returns new x and f. It also calculates fx at the new x when it calls Merit()
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of f after lnsrch: "<<f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of fx after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(fx));
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of x after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(x));
    //if(c) { if(!c->Satisfies(x)) LOG4CXX_WARN(KrisLibrary::logger(),"Warning: line searched x doesnt satisfy contact constraints"); }
    if (fx.maxAbsElement() < tolf) {
      Real fxmax=fx.maxAbsElement();
      Assert(MaxDistance(x) == fxmax);
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	      Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	      if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        //LOG4CXX_INFO(KrisLibrary::logger(),"Hmm.... check is returned on iter "<<iters<<", but test is not < tolmin"); 
	/*
	LOG4CXX_INFO(KrisLibrary::logger(),"Converging on x!");
	OutputASCIIShade(cout,g); cout<<endl;
	OutputASCIIShade(cout,p); cout<<endl;
        return ConvergenceX;
	*/
	if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"  Check is returned but test "<<test<<" is not < tolmin");
	return ConvergenceX;
      }
    }
    Real test=0.0; //Test for convergence on dx.
    for (int j=0;j<x.n;j++) {
      Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
      if (temp > test) test=temp; 
    }
    if (test < tolx) {
      if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"  Convergence on X, difference "<<test);
      return ConvergenceX;
    }
  } 
  return MaxItersReached;
}

ConvergenceResult NewtonRoot::Solve_Sparse(int& iters)
{
  int m=func->NumDimensions();
  fx.resize(m);
  SparseVectorFunction* sf;
  try {
    sf=dynamic_cast<SparseVectorFunction*>(func);
  }
  catch(exception& e) {
    FatalError("Could not cast VectorFieldFunctions to sparse, exception %s",e.what());    
  }
  SparseMatrix A(m,x.n);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);

  bool check;
  Real f=Merit(); //fx is also computed by this call. 
  if (fx.maxAbsElement() < tolf) { 
    iters=0;
    Real fxmax=fx.maxAbsElement();
    Assert(MaxDistance(x) == fxmax);
    return ConvergenceF;
  } 
  Real stpmax= stepMax*Max(x.norm(),(Real)x.n);
  int maxIters = iters;
  for (iters=0;iters<maxIters;iters++) { 
    sf->Jacobian_Sparse(x,A);
    A.mulTranspose(fx,g);
    xold.copy(x);

    //if x at the bounds, examine descent direction, set those directions
    //to be constrained
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(x(i) == bmin(i) && g(i) > 0) {
	  for(int j=0;j<A.m;j++)
	    A.rows[j].erase(i);
	}
	if(x(i) == bmax(i) && g(i) < 0) {
	  for(int j=0;j<A.m;j++)
	    A.rows[j].erase(i);
	}
      }
    }

    if(!SolveUnderconstrainedLS(A,fx,p)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"NewtonRoot::Solve: Unable to compute pseudoinverse of sparse matrix\n");
      return ConvergenceError;
    }
    p.inplaceNegative();
    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    check = LineMinimization(g,p,&f); //lnsrch returns new x and f. It also calculates fx at the new x when it calls Merit()
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of f after lnsrch: "<<f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of fx after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(fx));
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of x after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(x));
    //if(c) { if(!c->Satisfies(x)) LOG4CXX_WARN(KrisLibrary::logger(),"Warning: line searched x doesnt satisfy contact constraints"); }
    if (fx.maxAbsElement() < tolf) {
      Real fxmax=fx.maxAbsElement();
      Assert(MaxDistance(x) == fxmax);
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	      Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	      if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        //LOG4CXX_INFO(KrisLibrary::logger(),"Hmm.... check is returned on iter "<<iters<<", but test is not < tolmin"); 
	/*
	LOG4CXX_INFO(KrisLibrary::logger(),"Converging on x!");
	OutputASCIIShade(cout,g); cout<<endl;
	OutputASCIIShade(cout,p); cout<<endl;
        return ConvergenceX;
	*/
	return ConvergenceX;
      }
    }
    Real test=0.0; //Test for convergence on dx.
    for (int j=0;j<x.n;j++) {
      Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
      if (temp > test) test=temp; 
    }
    if (test < tolx) {
      return ConvergenceX;
    }
  } 
  return MaxItersReached;
}



#define ALF 1.0e-4 //Ensures sufficient decrease in function value. 

/*Given an n-dimensional point x0=x, the value of the
  function and gradient there, f and g, and a direction 
  p, finds a new point x along the direction p from
  x0 where the Merit() function has decreased  sufficiently.  The
  new function value is returned in f. stpmax is an input
  quantity that limits the length of the steps so that you do
  not try to evaluate the function in regions where it is
  undetermined or subject to overflow. p is usually the Newton
  direction. The output return value is false (0) on a
  normal exit. It is true (1) when x is too close to x0.
  In a minimization algorithm, this usually signals
  convergence and can be ignored. However, in a zero-finding
  algorithm the calling program should check whether the
  convergence is spurious. Some  difficult problems may require
  double precision in this routine.
*/
bool NewtonRoot::LineMinimization(const Vector& g, const Vector& p, Real *f) 

{ 
  if(debug && !IsFinite(p)) {
    if(verbose) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonRoot::LineMinimization: Error, p is not finite!");
      LOG4CXX_ERROR(KrisLibrary::logger(),"p="<<VectorPrinter(p,VectorPrinter::AsciiShade));
      KrisLibrary::loggerWait();
    }
    return false;
  }
  if(debug && !IsFinite(g)) {
    if(verbose) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonRoot::LineMinimization: Error, g is not finite!");
      LOG4CXX_ERROR(KrisLibrary::logger(),"g="<<VectorPrinter(g,VectorPrinter::AsciiShade));
      KrisLibrary::loggerWait();
    }
    return false;
  }
  Real fold = *f;
  xold.copy(x);
  Real f2,slope,tmplam=1.0;
  slope = g.dot(p);
  if (slope >= 0.0) {
    /*if(slope > 0.001) */{
      if(verbose)
	LOG4CXX_INFO(KrisLibrary::logger(),"NewtonRoot::LineMinimization: Opposing slope and descent directions\n");
      return false;
    }
    //  else slope = Abs(slope);
  }
  Vector biasDir;
  if(!bias.empty() && !sparse) {
    svd.nullspaceComponent(bias-x,biasDir);
  }
  Real test=Zero; //Compute lambdamin.
  for (int i=0;i<x.n;i++) {
    Real temp=Abs(p[i])/Max(Abs(xold[i]),One);
    if (temp > test) test=temp; 
  }
  Real alamin=tolx/test;
  Real alam=1.0,alam2;
  for (;;) { //Start of iteration loop.
    x.copy(xold); x.madd(p,alam);
    if(!biasDir.empty()) x.madd(biasDir,alam);
    if(bmin.n!=0) {
      AABBClamp(x,bmin,bmax);
    }
    *f=Merit();
    if (alam < alamin) { //Convergence on  x. For zero finding, the calling program should verify the convergence. 
      //x.copy(xold);
      //LOG4CXX_INFO(KrisLibrary::logger(),"Resulting step size "<<alam);
      return true;
    }
    else if (*f <= fold+ALF*alam*slope) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Resulting step size "<<alam);
      return false; //Sufficient function decrease. 
    }
    else if(!IsFinite(*f)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonRoot::LineMinimization: f(x) is infinite or NaN... backtracking");
      /*
      LOG4CXX_ERROR(KrisLibrary::logger(),"x0="<<VectorPrinter(xold,VectorPrinter::AsciiShade));
      LOG4CXX_ERROR(KrisLibrary::logger(),"p="<<VectorPrinter(p,VectorPrinter::AsciiShade));
      LOG4CXX_ERROR(KrisLibrary::logger(),"g="<<VectorPrinter(g,VectorPrinter::AsciiShade));
      LOG4CXX_ERROR(KrisLibrary::logger(),"lambda="<<alam);
      KrisLibrary::loggerWait();
      */
      *f = fold;
      tmplam = 0.5*alam;
    }
    else { //Backtrack. 
      if (alam == 1.0) 
	tmplam = -slope/(2.0*(*f-fold-slope)); //First time. 
      else { //Subsequent backtracks. 
	Real rhs1 = *f-fold-alam*slope; 
	Real rhs2=f2-fold-alam2*slope; 
	Real a=(rhs1/(alam*alam)-rhs2/(alam2*alam2))/(alam-alam2); 
	Real b=(-alam2*rhs1/(alam*alam)+alam*rhs2/(alam2*alam2))/(alam-alam2); 
	if (a == 0.0) tmplam = -slope/(2.0*b);
	else { 
	  Real disc=b*b-3.0*a*slope; 
	  if (disc < 0.0) tmplam=0.5*alam;
	  else if (b <= 0.0) tmplam=(-b+Sqrt(disc))/(3.0*a);
	  else tmplam=-slope/(b+Sqrt(disc));
	}
	if(IsNaN(tmplam)) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonRoot::LineMinimization: templam is NaN??");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"f="<<*f);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"fold="<<fold);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"a="<<a);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"b="<<b);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"rhs1="<<rhs1);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"hrs2="<<rhs2);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"slope="<<slope);
	  KrisLibrary::loggerWait();
	  tmplam = 0.5*alam;
	}
	if (tmplam > 0.5*alam) 
	  tmplam=0.5*alam; 
      } 
    }
    alam2=alam;
    f2 = *f; 
    alam=Max(tmplam,(Real)0.1*alam); 
  } //Try again.
}

ConstrainedNewtonRoot::ConstrainedNewtonRoot(VectorFieldFunction* _func,VectorFieldFunction* _c)
  :NewtonRoot(_func),c(_c),tolc(0)
{
}

bool ConstrainedNewtonRoot::GlobalSolve(int& iters,ConvergenceResult* r)
{
  if(c==NULL) {
    return NewtonRoot::GlobalSolve(iters,r);
  }

  if(verbose) { LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedNewtonRoot::GlobalSolve(): ");   }
  Vector xinit;
  xinit.copy(x);
  ConvergenceResult res;
#if USE_ACTIVE_SET
  if(sparse)
#if USE_SLP
    res=SolveConstrained_SLP(iters);
#else
    res=SolveConstrained2_Sparse(iters);
#endif //USE_SLP
  else
    res=SolveConstrained2(iters);
#else
  res=SolveConstrained(iters);
#endif //USE_ACTIVE_SET
  if(r) *r=res;
  Real initDist,endDist = MaxDistance(x);
  int index;
  Real endMargin = InequalityMargin(c,x,&index);

  initDist = MaxDistance(xinit);
  switch(res) {
  case ConvergenceX:
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached convergence on x... ");
    if(endDist <= tolf) {
      if(endMargin < tolc) {
	if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"satisfies f but not c: "<<endMargin<<" < "<<tolc);
	return false;
      }
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"satisfies all constraints.");
      return true;
    }
    else {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"does not satisfy tolerance, distance "<<endDist<<", margin "<<endMargin<<".");
      return false;
    }
    break;

  case LocalMinimum:
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached local minimum... ");
    if(endDist <= tolf) {
      if(endMargin < tolc) {
	if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"satisfies f but not c: "<<endMargin<<" < "<<tolc);
	return false;
      }
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"satisfies all constraints.");
      return true;
    }
    else {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"stuck at distance "<<endDist<<".");
      return false;
    }
  case ConvergenceF:
    if(endMargin < tolc) {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached convergence on f, but not margin "<<endMargin);
      return false;
    }
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Reached convergence on f, new distance "<<endDist<<" margin "<<endMargin);
    Assert(endDist <= tolf);
    return true;

  case MaxItersReached:
    initDist = MaxDistance(xinit);
    if(endDist < initDist) {
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Max iters reached, distance was decreased to "<<endDist);
    }
    else {
      //if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Max iters reached, looks like divergence.  Reverting to initial.");
      if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Max iters reached, distance increased (may have reduced inequality margin).");
      //x.copy(xinit);
    }
    return false;
  default:
    if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"Error");
    return false;
  }
  return false;
}

//this one steps across the constraint manifold first, then solves
//inequality constraints while maintaining equality
ConvergenceResult ConstrainedNewtonRoot::SolveConstrained(int& iters)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedNewtonRoot::SolveConstrained() is deprecated, use SolveConstrained2()");
  KrisLibrary::loggerWait();
  Assert(c!=NULL);

  g.resize(x.n);
  int maxIters=iters;
  ConvergenceResult res;
  if(!NewtonRoot::GlobalSolve(iters,&res)) {
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"SolveConstrained(): Failed on initial equality solve");
    return res;
  }
  Vector xold2;

  Real margin;
  int index;
  for(;iters<maxIters;iters++) {
    c->PreEval(x);
    margin = InequalityMargin(c,x,&index);
    if(margin >= tolc) {
      return res;

      //LOG4CXX_INFO(KrisLibrary::logger(),"Maximizing margin "<<margin<<" at "<<c->Label(index));
      //take step
      c->Jacobian_i(x,index,g);
      //project onto tangent space of func
      func->PreEval(x);
      func->Jacobian(x,fJx);
      if(!svd.set(fJx)) {
	if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"SolveConstrained(): error setting SVD of equality jacobian");
	return ConvergenceError;
      }
      svd.nullspaceComponent(g,p);
      g -= p;

      //newton step is -g*margin/|g|^2
      Real gnorm = g.norm();
      if(gnorm < tolf) {
	return LocalMinimum;
      }
      Real steplen = (tolc+tolf-margin)/gnorm;
      steplen = Min(steplen,stepMax);

      //search for a valid t
      const Real stepScale = 0.5;
      Real t = steplen;
      Real tmargin;
      xold2.copy(x);
      for(int lsiters=0;;lsiters++) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"line search iteration "<<lsiters<<", step length "<<t);
	if(t < tolx) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"No progress in SolveConstrained() after "<<iters<<" iters, looks like a local minimum");
	  return LocalMinimum;
	}
	x.copy(xold2); 
	x.madd(g,t/gnorm);
	if(bmin.n!=0) AABBClamp(x,bmin,bmax);

	//LOG4CXX_INFO(KrisLibrary::logger(),"Step has distance "<<MaxDistance(x)<<" from eq");
	int solveIters=maxIters-iters;
	if(NewtonRoot::GlobalSolve(solveIters,&res) || res==ConvergenceX) {
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Used "<<solveIters<<" iters of "<<maxIters-iters);
	  iters += solveIters;
	  c->PreEval(x);
	  tmargin = c->Eval_i(x,index);
	  if(tmargin > margin) {
	    break;
	  }
	}
	else {
	  if(verbose) {
	    switch(res) {
	    case LocalMinimum: LOG4CXX_INFO(KrisLibrary::logger(),"LocalMinimum "); break;
	    case ConvergenceError: LOG4CXX_ERROR(KrisLibrary::logger(),"Error "); break;
	    default: break;
	    }
	    LOG4CXX_INFO(KrisLibrary::logger(),"Equality solve failed, using "<<solveIters<<" iters of "<<maxIters-iters<<" to solve distance "<<MaxDistance(x));
	  }
	  iters += solveIters/4+1;
	  if(maxIters-iters <= 0) return MaxItersReached;
	}
	t *= stepScale;
      }
    }
  }
  return MaxItersReached;
}




Real ConstrainedNewtonRoot::Merit()
{
#if USE_ACTIVE_SET
  (*func)(x,fx);
  Real sum = fx.normSquared();
  if(!activeSetC.empty()) {
    c->PreEval(x);
    for(size_t i=0;i<activeSetC.size();i++) {
      Real ci = c->Eval_i(x,activeSetC[i])-kInequalityAdjustment;
      if(ci < tolc)
	sum += Sqr(tolc-ci);
    }
  }
  return Half*sum;
#else
  return NewtonRoot::Merit();
#endif
}


//An active-set method.
//At any given point in the iteration to solve
//f(x) = 0, c(x) >= 0, bmin <= x <= bmax
//there's a set of active constraints.
//We solve a newton step for f(x) = 0 as well as ci(x) = 0
//for all active constraints i.
//
//Bound constraints are handled implicitly.
//
//The merit function is 1/2(|f(x)|^2 + |ci(x)|^2).
ConvergenceResult ConstrainedNewtonRoot::SolveConstrained2(int& iters)
{
  if(debug && !IsFinite(x)) { 
    if(verbose) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR! Initial x is not finite!");
      LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
      KrisLibrary::loggerWait(); 
    }
    return ConvergenceError; 
  }

  Assert(func!=NULL);
  Assert(c!=NULL);
  bool check;
  int fn=func->NumDimensions(),cn=c->NumDimensions();
  cx.resize(cn);
  fx.resize(fn);
  activeSetC.reserve(cn);
  activeSetC.resize(0);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);
  Real f;
  (*func)(x,fx);
  (*c)(x,cx);
  if (fx.maxAbsElement() < tolf && cx.minElement() >= tolc) { 
    return ConvergenceF;
  }

  A.resize(fn+cn/2,x.n);  //allocate more space than necessary
  rhs.resize(fn+cn/2,x.n);

  Real stpmax= stepMax*Max(x.norm(),(Real)x.n);
  int maxIters = iters;
  //at the beginning of the loop, fx and cx must be evaluated
  for (iters=0;iters<maxIters;iters++) {
    //debug
    if(debug && !IsFinite(fx)) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"f(x) is not finite!");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(fx));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }
    if(debug && !IsFinite(cx)) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"c(x) is not finite!");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(cx));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }

    //pick initial active set
    activeSetC.resize(0);
    for(int i=0;i<cn;i++)
      if(cx(i) <= tolc) activeSetC.push_back(i);
    /*
      if(bmin.n != 0) {
      activeSetBound.reserve(x.n);
      for(int i=0;i<x.n;i++)
      if(bmin(i) == x(i) || bmax(i) == x(i))
      activeSetBound.push_back(i);
      }
    */
    if(verbose >= 1) {
      int index;
      Real d=cx.minElement(&index);
      LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index));
    }
    if(verbose >= 3 && !activeSetC.empty()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Active set: {");
      for(size_t i=0;i<activeSetC.size();i++)
	LOG4CXX_INFO(KrisLibrary::logger(),c->Label(activeSetC[i])<<",");
      LOG4CXX_INFO(KrisLibrary::logger(),"}");
    }

    //TODO: use a simplex-like method to find a search direction that optionally uses some constraints?
    //TODO: use lagrange multipliers to help choose changes in active set
    //newton step
    //first, calculate the jacobian of active constraints
    A.resize(fn+(int)activeSetC.size(),x.n);
    rhs.resize(A.m);
    rhs.copySubVector(0,fx);
    for(size_t i=0;i<activeSetC.size();i++)
      rhs(fn+(int)i) = cx(activeSetC[i])-kInequalityAdjustment-tolf;
    fJx.setRef(A,0,0,1,1,fn,x.n);
    func->Jacobian(x,fJx);
    if(debug && !IsFinite(fJx)) {
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Jacobian of equality is not finite!");
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }
    for(size_t i=0;i<activeSetC.size();i++) {
      Vector Ai; A.getRowRef(fn+(int)i,Ai);
      c->Jacobian_i(x,activeSetC[i],Ai);
      if(debug && !IsFinite(Ai)) {
	if(verbose) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Jacobian "<<i<<" of inequality is not finite!");
	  LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(Ai));
	  LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
	  KrisLibrary::loggerWait();
	}
	return ConvergenceError;
      }
    }

    //solve
    bool solvedProperly=false;
    A.mulTranspose(rhs,g);
    if(SolveUnderconstrainedLS(A,rhs,p)) {
      solvedProperly=true;
      if(bmin.n != 0) {
	//revise A,p,g based on active set of bound constraints
	activeSetBound.resize(0);
	for(int i=0;i<x.n;i++) {
	  if((x(i) == bmin(i) && p(i) < Zero) ||
	     (x(i) == bmax(i) && p(i) > Zero))
	    activeSetBound.push_back(i);
	}
	if(!activeSetBound.empty()) {
	  if(verbose >= 2) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Active bounds: {");
	    for(size_t i=0;i<activeSetBound.size();i++)
	      LOG4CXX_INFO(KrisLibrary::logger(),activeSetBound[i]<<",");
	    LOG4CXX_INFO(KrisLibrary::logger(),"}");
	  }
	  //remove columns from A matrix
	  RemoveColumns(A,activeSetBound);
	  //solve for A*p = rhs
	  p.n = p.n - (int)activeSetBound.size();
	  if(SolveUnderconstrainedLS(A,rhs,p)) {
	    //SetElements(g,activeSetBound,Zero);
	    AddElements(p,activeSetBound,Zero);
	  }
	  else {
	    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to solve pseudoinverse of bound-constrained problem, just setting entries of dirs to 0");
	    solvedProperly=false;
	    p.n = p.n + (int)activeSetBound.size();
	    //SetElements(g,activeSetBound,Zero);
	    SetElements(p,activeSetBound,Zero);
	  }
	  A.n = A.n + (int)activeSetBound.size();
	}
      }
      /*
      Vector xtemp,vtemp;
      xtemp.add(x,p);
      vtemp.resize(fn);
      (*func)(xtemp,vtemp);
      LOG4CXX_INFO(KrisLibrary::logger(),"desired f: "<<VectorPrinter(vtemp,VectorPrinter::AsciiShade));
      vtemp.resize(cn);
      (*c)(xtemp,vtemp);
      LOG4CXX_INFO(KrisLibrary::logger(),"desired c min: "<<vtemp.minElement());

      //A*p = J(x)*p = rhs = f(x)
      vtemp.resize(A.m);
      A.mul(p,vtemp);
      vtemp -= rhs;
      vtemp.inplaceNegative();
      Vector temp; temp.setRef(vtemp,fn);
      LOG4CXX_INFO(KrisLibrary::logger(),"linear c min: "<<temp.minElement());
      */
    }
    else if(fx.maxAbsElement() > -cx.minElement()) {  //seems more important to solve equality constraints
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to solve pseudoinverse of equalities+inequalities, trying unconstrained");
      }
      if(verbose >= 2) {
	MatrixPrinter mp(A);
	if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	LOG4CXX_ERROR(KrisLibrary::logger(),mp);
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
	//LOG4CXX_ERROR(KrisLibrary::logger(),"New dims: "<<fn<<" x "<<A.n);
	KrisLibrary::loggerWait();
      }
      A.m = fn;
      rhs.n = fn;
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"New matrix: ");
	MatrixPrinter mp(A);
	if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	LOG4CXX_ERROR(KrisLibrary::logger(),mp);
	KrisLibrary::loggerWait();
      }
      A.mulTranspose(rhs,g);
      if(SolveUnderconstrainedLS(A,rhs,p)) {
      }
      else {
	if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to solve pseudoinverse!");
	if(verbose >= 2) {
	  MatrixPrinter mp(A);
	  if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	  LOG4CXX_ERROR(KrisLibrary::logger(),mp);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Press a key to continue");
	  KrisLibrary::loggerWait();
	}
	p=g;
	//return ConvergenceError;
      }
    }
    else {  //seems more important to solve inequality constraints
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to solve pseudoinverse of equalities, trying inequality");
      }
      for(size_t i=0;i<activeSetC.size();i++) {
	for(int j=0;j<A.n;j++)
	  A(i,j) = A(fn+i,j);
	rhs(i) = rhs(fn+i);
      }
      A.m=(int)activeSetC.size();
      rhs.n=(int)activeSetC.size();
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"New matrix: ");
	MatrixPrinter mp(A);
	if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	LOG4CXX_ERROR(KrisLibrary::logger(),mp);
	KrisLibrary::loggerWait();
      }
      A.mulTranspose(rhs,g);
      if(SolveUnderconstrainedLS(A,rhs,p)) {
      }
      else {
	if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to solve pseudoinverse!");
	if(verbose >= 2) {
	  MatrixPrinter mp(A);
	  if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	  LOG4CXX_ERROR(KrisLibrary::logger(),mp);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Press a key to continue");
	  KrisLibrary::loggerWait();
	}
	p=g;
	//return ConvergenceError;
      }
    }
    if(g.dot(p) > Zero) {
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error, gradient and search direction are opposing...");
	LOG4CXX_ERROR(KrisLibrary::logger(),"g: "<<VectorPrinter(g));
	LOG4CXX_ERROR(KrisLibrary::logger(),"p: "<<VectorPrinter(p));
	LOG4CXX_ERROR(KrisLibrary::logger(),"g dot p: "<<g.dot(p));
	LOG4CXX_ERROR(KrisLibrary::logger(),"Using gradient direction...");
      }
      p=g;
      //return ConvergenceError;
    }
    p.inplaceNegative();
    if(verbose>=2) LOG4CXX_INFO(KrisLibrary::logger(),"Line search direction: "<<p);

    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    f = Merit();
    check = LineMinimization(g,p,&f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of f after lnsrch: "<<f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of fx after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(fx));
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of x after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(x));

    //Note: fx updated by last call to merit function
    if(debug && !IsFinite(x)) { 
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR! after LineMinimization, x is not finite!");
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x,VectorPrinter::AsciiShade));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }

    (*c)(x,cx);
    if (fx.maxAbsElement() < tolf && cx.minElement() > tolc) {
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	      Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	      if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        if(verbose) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedNewtonRoot(): Hmm.... check is returned, but test is not < tolmin"); 
	  LOG4CXX_INFO(KrisLibrary::logger(),"Gradient: "<<g);
	}
        return ConvergenceX;
      }
    }
    if(solvedProperly) {
      Real test=0.0; //Test for convergence on dx.
      for (int j=0;j<x.n;j++) {
	Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
	if (temp > test) test=temp; 
      }
      if (test < tolx) 
	return ConvergenceX;
    }
  } 
  return MaxItersReached;
}


ConvergenceResult ConstrainedNewtonRoot::SolveConstrained2_Sparse(int& iters)
{
  if(debug && !IsFinite(x)) { 
    if(verbose) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR! Initial x is not finite!");
      LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
      KrisLibrary::loggerWait(); 
    }
    return ConvergenceError; 
  }

  Assert(c != NULL);
  SparseVectorFunction *seq,*sineq;
  try {
    seq=dynamic_cast<SparseVectorFunction*>(func);
    sineq=dynamic_cast<SparseVectorFunction*>(c);
  }
  catch(exception& e) {
    FatalError("Could not cast VectorFieldFunctions to sparse, exception %s",e.what());
  }

  Assert(func!=NULL);
  Assert(c!=NULL);
  bool check;
  int fn=func->NumDimensions(),cn=c->NumDimensions();
  cx.resize(cn);
  fx.resize(fn);
  activeSetC.reserve(cn);
  activeSetC.resize(0);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);
  Real f;
  (*func)(x,fx);
  (*c)(x,cx);
  if (fx.maxAbsElement() < tolf && cx.minElement() >= tolc) { 
    return ConvergenceF;
  }

  SparseMatrix A;
  SparseMatrix Jeq(fn,x.n);
  A.resize(fn+cn/2,x.n);  //allocate more space than necessary
  rhs.resize(fn+cn/2,x.n);
  p.resize(x.n);

  Real stpmax= stepMax*Max(x.norm(),(Real)x.n);
  int maxIters = iters;
  //at the beginning of the loop, fx and cx must be evaluated
  for (iters=0;iters<maxIters;iters++) {
    //debug
    if(debug && !IsFinite(fx)) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"f(x) is not finite!");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(fx));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }
    if(debug && !IsFinite(cx)) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"c(x) is not finite!");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(cx));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }

    //pick initial active set
    activeSetC.resize(0);
    activeSetBound.resize(0);
    for(int i=0;i<cn;i++)
      if(cx(i) <= tolc) activeSetC.push_back(i);
    if(bmin.n != 0) {
      activeSetBound.reserve(x.n);
      for(int i=0;i<x.n;i++)
	if(x(i) <= bmin(i) || x(i) >= bmax(i))
	  activeSetBound.push_back(i);
    }
    if(verbose) {
      int index;
      Real d=cx.minElement(&index);
      LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index));
    }
    if(verbose >= 3 && !activeSetC.empty()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Active set: {");
      for(size_t i=0;i<activeSetC.size();i++)
	LOG4CXX_INFO(KrisLibrary::logger(),c->Label(activeSetC[i])<<",");
      LOG4CXX_INFO(KrisLibrary::logger(),"}");
    }

    //TODO: use a simplex-like method to find a search direction that optionally uses some constraints?
    //TODO: use lagrange multipliers to help choose changes in active set
    //newton step
    //first, calculate the jacobian of active constraints
    A.resize(fn+(int)activeSetC.size(),x.n);
    A.setZero();
    rhs.resize(A.m);
    rhs.copySubVector(0,fx);
    for(size_t i=0;i<activeSetC.size();i++)
      rhs(fn+(int)i) = cx(activeSetC[i])-kInequalityAdjustment-tolf;
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Evaluating jacobian...");     seq->Jacobian_Sparse(x,Jeq);
    A.copySubMatrix(0,0,Jeq);
    if(debug && !IsFinite(Jeq)) {
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Jacobian of equality is not finite!");
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }
    for(size_t i=0;i<activeSetC.size();i++) {
      SparseVector Ai(x.n);
      sineq->Jacobian_i_Sparse(x,activeSetC[i],Ai);
      A.copyRow(fn+i,Ai);
      if(debug && !IsFinite(Ai)) {
	if(verbose) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Jacobian "<<i<<" of inequality is not finite!");
	  Vector temp;
	  Ai.get(temp);
	  LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(temp));
	  LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
	  KrisLibrary::loggerWait();
	}
	return ConvergenceError;
      }
    }
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Done.");

    if(!activeSetBound.empty()) {
      if(verbose >= 2) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Active bounds: {");
	for(size_t i=0;i<activeSetBound.size();i++)
	  LOG4CXX_INFO(KrisLibrary::logger(),activeSetBound[i]<<",");
	LOG4CXX_INFO(KrisLibrary::logger(),"}");
      }
    }
    //for bound constraints D(x), A is +/- 1, but since D(x)=0 it doesn't add into g
    A.mulTranspose(rhs,g);

    //remove columns from A matrix
    RemoveColumns(A,activeSetBound);

    //solve
    bool solvedProperly=false;
    p.resize(A.n);
    Assert(p.n + (int)activeSetBound.size() == x.n);
    Assert(p.getCapacity() >= x.n);
    if(SolveUnderconstrainedLS(A,rhs,p)) {
      solvedProperly=true;
      AddElements(p,activeSetBound,Zero);
    }
    else {
      if(verbose)
	LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedNewtonRoot: Unable to solve pseudoinverse of equalities+inequalities, returning error");
      p.n = p.n + (int)activeSetBound.size();
      SetElements(p,activeSetBound,Zero);
      return ConvergenceError;
    }
    if(g.dot(p) <= Zero) {
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error, gradient and search direction are opposing...");
	LOG4CXX_ERROR(KrisLibrary::logger(),"g: "<<VectorPrinter(g));
	LOG4CXX_ERROR(KrisLibrary::logger(),"p: "<<VectorPrinter(p));
	LOG4CXX_ERROR(KrisLibrary::logger(),"g dot p: "<<g.dot(p));
	LOG4CXX_ERROR(KrisLibrary::logger(),"Using gradient direction...");
      }
      p=g;
      //return ConvergenceError;
    }
    p.inplaceNegative();

    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    f = Merit();
    check = LineMinimization(g,p,&f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of f after lnsrch: "<<f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of fx after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(fx));
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of x after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(x));

    //Note: fx updated by last call to merit function
    if(debug && !IsFinite(x)) { 
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR! after LineMinimization, x is not finite!");
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x,VectorPrinter::AsciiShade));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }

    (*c)(x,cx);
    if (fx.maxAbsElement() < tolf && cx.minElement() > tolc) {
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedNewtonRoot(): Hmm.... check is returned, but test is not < tolmin"); 
        return ConvergenceX;
      }
    }
    if(solvedProperly) {
      Real test=0.0; //Test for convergence on dx.
      for (int j=0;j<x.n;j++) {
	Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
	if (temp > test) test=temp; 
      }
      if (test < tolx) 
	return ConvergenceX;
    }
  } 
  return MaxItersReached;
}


ConvergenceResult ConstrainedNewtonRoot::SolveConstrained_SLP(int& iters)
{
  if(debug && !IsFinite(x)) { 
    if(verbose) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR! Initial x is not finite!");
      LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x));
      KrisLibrary::loggerWait(); 
    }
    return ConvergenceError; 
  }

  Assert(c != NULL);
  SparseVectorFunction *seq,*sineq;
  try {
    seq=dynamic_cast<SparseVectorFunction*>(func);
    sineq=dynamic_cast<SparseVectorFunction*>(c);
  }
  catch(exception& e) {
    FatalError("Could not cast VectorFieldFunctions to sparse, exception %s",e.what());
  }

  Assert(func!=NULL);
  Assert(c!=NULL);
  bool check;
  int fn=func->NumDimensions(),cn=c->NumDimensions();
  cx.resize(cn);
  fx.resize(fn);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);
  Real f;
  (*func)(x,fx);
  (*c)(x,cx);
  if (fx.maxAbsElement() < tolf && cx.minElement() >= tolc) { 
    return ConvergenceF;
  }

  MinNormProblem_Sparse lp;
  MinNormProblem_Sparse lp2;
  SparseMatrix Aeq,Aineq;
  Vector beq,bineq;
  lp.norm = Inf;
  lp.ResizeObjective(x.n,x.n);
  lp.ResizeConstraints(fn+cn,x.n);
  lp.C.setIdentity();
  lp.d.setZero();
  Aeq.resize(fn,x.n);
  Aineq.resize(cn,x.n);
  Assert(lp.IsValid());

  //setup activeSetC
  activeSetC.resize(cn);
  for(int i=0;i<cn;i++) activeSetC[i]=i;

  int maxIters = iters;
  //at the beginning of the loop, fx and cx must be evaluated
  for (iters=0;iters<maxIters;iters++) {
    //debug
    if(debug && !IsFinite(fx)) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"f(x) is not finite!");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(fx));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }
    if(debug && !IsFinite(cx)) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"c(x) is not finite!");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(cx));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }

    if(verbose) {
      int index;
      Real d=cx.minElement(&index);
      LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index));
    }

    //setup the problem
    //min||x-x0||
    //C(x) = 0  => dC/dx(x0)*(x-x0) + C(x0) = 0 => JC*dx = -C(x0)
    //D(x) > 0  => dD/dx(x0)*(x-x0) + D(x0) > 0 => -JD*dx < D(x0)
    //bmin <= x0+dx <= bmax => bmin-x0 <= dx <= bmax-x0
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Evaluating jacobians...");     seq->Jacobian_Sparse(x,Aeq);
    beq.setNegative(fx);
    sineq->Jacobian_Sparse(x,Aineq);
    Aineq.inplaceNegative();
    bineq.copy(cx);
    for(int i=0;i<x.n;i++) //what step length?
      Assert(lp.l(i) <= 0 && lp.u(i) >= 0);
    for(int i=0;i<bineq.n;i++) bineq(i) -= kInequalityAdjustment;
    lp.SetSimpleForm(Aeq,beq,Aineq,bineq);
    lp.l.sub(bmin,x);
    lp.u.sub(bmax,x);
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Done.");
    Assert(lp.IsValid());

    p.resize(x.n); p.setZero();
    /*
    LOG4CXX_INFO(KrisLibrary::logger(),"********** Zero Vector: ***********");
    LOG4CXX_INFO(KrisLibrary::logger(),"L-inf norm: "<<lp.Norm(p));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error: "<<lp.EqualityError(p));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Inequality error: "<<lp.InequalityMargin(p));
    LOG4CXX_INFO(KrisLibrary::logger(),"************************************");
    */

    lp.Assemble();
    LinearProgram::Result res=lp.Solve(p);
    if(res == LinearProgram::Infeasible || res == LinearProgram::Error) {
      if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"LP failed, solving minimum error problem...");
      //solve minimum error problem
      //min{p,z} |Jc*p+c0|+|z|+|p| (|.| denotes the 1-norm)
      //s.t. -Jd*p - z < d0
      /*
      lp2.norm = 1;
      lp2.A.resize(fn+cn,x.n+cn);
      lp2.A.setZero();
      lp2.A.copySubMatrix(0,0,lp.Aeq);
      for(int i=0;i<cn;i++) lp2.A(fn+i,x.n+i)=1;
      lp2.b.resize(fn+cn);
      lp2.b.copySubVector(0,lp.b);
      lp2.Aineq.resize(cn,x.n+cn);
      lp2.Aineq.copySubMatrix(0,0,lp.Aineq);
      for(int i=0;i<cn;i++) lp2.Aineq(i,i+x.n) = -1;
      lp2.bineq.resize(cn);
      lp2.bineq = lp.bineq;
      //TODO: allow errors in bound constraints?
      lp2.l.resize(x.n+cn,-Inf);
      lp2.u.resize(x.n+cn,Inf);
      Vector temp;
      temp.setRef(lp2.l,0,1,x.n); temp.sub(bmin,x);
      temp.setRef(lp2.u,0,1,x.n); temp.sub(bmax,x);
      Real stepmax = 0.1;
      for(int i=0;i<x.n;i++) { //what step length?
	Assert(lp2.l(i) <= 0 && lp2.u(i) >= 0);
	if(lp2.l(i) < -stepmax) lp2.l(i)=-stepmax;
	if(lp2.u(i) > stepmax) lp2.u(i)=stepmax;
      }
      Assert(lp2.IsValid());

      temp.clear();
      temp.resize(x.n+cn);
      */
      /*
      temp.setZero();
      for(int i=0;i<cn;i++)
	if(cx(i) < kInequalityAdjustment) temp(x.n+i) = kInequalityAdjustment-cx(i);
      LOG4CXX_INFO(KrisLibrary::logger(),"********** Zero Vector: ***********");
      LOG4CXX_INFO(KrisLibrary::logger(),"L1 norm: "<<lp2.Norm(temp));
      LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error: "<<lp2.EqualityError(temp));
      LOG4CXX_ERROR(KrisLibrary::logger(),"Inequality error: "<<lp2.InequalityMargin(temp));
      LOG4CXX_INFO(KrisLibrary::logger(),"************************************");
      */
      /*
      lp2.Assemble();
      res=lp2.Solve(temp);
    */
      res = LinearProgram::Infeasible;
      if(res == LinearProgram::Infeasible || res == LinearProgram::Error) {
	if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"Minimum error problem failed, solving LS...");
	//try doing a LS solve step
	SparseMatrix mat;
	Vector rhs;
	int numInequalities=0;
	for(int i=0;i<cn;i++) if(cx(i) <= 0) numInequalities++;
	mat.resize(fn+numInequalities,x.n);
	rhs.resize(fn+numInequalities);
	mat.copySubMatrix(0,0,Aeq);
	rhs.copySubVector(0,beq);
	numInequalities=0;
	for(int i=0;i<cn;i++) {
	  if(cx(i) <= 0) {
	    mat.rows[numInequalities+fn] = Aineq.rows[i];
	    rhs(numInequalities+fn) = bineq(i);
	    numInequalities++;
	  }
	}
	if(SolveUnderconstrainedLS(mat,rhs,p)) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Solved using least-squares solution...");
	}
	else {
	  if(verbose) LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to solve for a feasible step!");
	  return ConvergenceError;
	}
      }
      /*
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"********* Min-norm solution 2: **********");
	LOG4CXX_INFO(KrisLibrary::logger(),"L1 norm of shift: "<<lp2.Norm(temp));
	LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error: "<<lp2.EqualityError(temp));
	LOG4CXX_ERROR(KrisLibrary::logger(),"Inequality error: "<<lp2.InequalityMargin(temp));
	LOG4CXX_INFO(KrisLibrary::logger(),"************************************");
	//get just the top n elements
	temp.getSubVectorCopy(0,p);
      }
      */
    }
    Assert(res != LinearProgram::Unbounded);

    /*
    LOG4CXX_INFO(KrisLibrary::logger(),"********* Min-norm solution: ************");
    LOG4CXX_INFO(KrisLibrary::logger(),"L-inf norm of shift: "<<lp.Norm(p));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Equality error: "<<lp.EqualityError(p));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Inequality error: "<<lp.InequalityMargin(p));
    LOG4CXX_INFO(KrisLibrary::logger(),"************************************");
    */

    LOG4CXX_INFO(KrisLibrary::logger(),"Length of shift: "<<p.norm());
    xold = x;
    x += p;
    (*func)(x,fx);
    (*c)(x,cx);
    int index;
    Real d=cx.minElement(&index);
    LOG4CXX_ERROR(KrisLibrary::logger(),"Shifted equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index));
    x = xold;
    (*func)(x,fx);
    (*c)(x,cx);

    //fake a gradient direction
    g.setNegative(p);
    f = Merit();
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Merit value before lnsrch: "<<f);
    check = LineMinimization(g,p,&f);
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Merit value after lnsrch: "<<f);
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of fx after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(fx));
    //LOG4CXX_INFO(KrisLibrary::logger(),"New value of x after lnsrch: "); LOG4CXX_INFO(KrisLibrary::logger(),VectorPrinter(x));

    //Note: fx updated by last call to merit function
    if(debug && !IsFinite(x)) { 
      if(verbose) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR! after LineMinimization, x is not finite!");
	LOG4CXX_ERROR(KrisLibrary::logger(),VectorPrinter(x,VectorPrinter::AsciiShade));
	KrisLibrary::loggerWait();
      }
      return ConvergenceError;
    }

    (*c)(x,cx);
    if (fx.maxAbsElement() < tolf && cx.minElement() > tolc) {
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedNewtonRoot(): Hmm.... check is returned, but test is not < tolmin"); 
        return ConvergenceX;
      }
    }
    Real test=0.0; //Test for convergence on dx.
    for (int j=0;j<x.n;j++) {
      Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
      if (temp > test) test=temp; 
    }
    if (test < tolx) 
      return ConvergenceX;
  } 
  return MaxItersReached;
}
