#include "Newton.h"
#include <utils.h>
#include <math/infnan.h>
#include <math/linalgebra.h>
#include <math/AABB.h>
#include <math/indexing.h>
#include <math/sparsefunction.h>
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
  if(verbose) { cout<<"NewtonRoot::GlobalSolve(): "; cout.flush(); }
  //Vector xinit;
  //xinit.copy(x);
  Real initDist = MaxDistance(x);
  if(verbose) cout<<"initial distance "<<initDist<<". ";
  ConvergenceResult res;
  if(sparse)
    res=Solve_Sparse(iters);
  else
    res=Solve(iters);
  if(r) *r=res;
  Real endDist = MaxDistance(x);

  switch(res) {
  case ConvergenceX:
    if(verbose) cout<<"Reached convergence on x in "<<iters<<" iters... ";
    if(endDist <= tolf) {
      if(verbose) cout<<"satisfies constraint."<<endl;
      return true;
    }
    else {
      if(verbose) cout<<"does not satisfy tolerance, distance "<<endDist<<"."<<endl;
      return false;
    }
    break;

  case LocalMinimum:
    if(verbose) cout<<"Reached local minimum in "<<iters<<" iters... ";
    if(endDist <= tolf) {
      if(verbose) cout<<"satisfies constraint."<<endl;
      return true;
    }
    else {
      if(verbose) cout<<"stuck at distance "<<endDist<<"."<<endl;
      return false;
    }

  case ConvergenceF:
    if(verbose) cout<<"Reached convergence on f in "<<iters<<" iters, new distance "<<endDist<<endl;
    Assert(endDist <= tolf);
    return true;

  case MaxItersReached:
    if(endDist < initDist) {
      if(verbose) cout<<"Max iters reached, distance was decreased to "<<endDist<<endl;
    }
    else {
      //if(verbose) cout<<"Max iters reached, looks like divergence.  Reverting to initial."<<endl;
      if(verbose) cout<<"Max iters reached, looks like divergence."<<endl;
      //x.copy(xinit);
    }
    return false;
  default:
    if(verbose) cout<<"Error"<<endl;
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
    if(verbose>=1 && A.m*A.n>10000) cout<<"Calculating SVD..."<<endl;
    if(svd.set(A)) {
      if(verbose>=1 && A.m*A.n>10000) cout<<"done"<<endl;
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
      cerr<<"NewtonRoot::SolveUnderconstrainedLS: Warning, LSQR returned a non-finite solution"<<endl;
      cerr<<VectorPrinter(lsqr.x,VectorPrinter::AsciiShade)<<endl;
      getchar();
      return false;
    }
    //cout<<"NewtonRoot::SolveUnderconstrainedLS: LSQR residual is "<<lsqr.residualNorm<<endl;
    x=lsqr.x;
    return true;
  }
  //Hmm.. should we try a non-converged x?
  x=lsqr.x;
  return true;
  return false;
}

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
    if(verbose >= 2) cout<<"Iteration "<<iters<<", x = "<<x<<endl;
    func->Jacobian(x,fJx);
    fJx.mulTranspose(fx,g);
    xold.copy(x);
    //if x at the bounds, examine descent direction, set those directions
    //to be constrained
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(x(i) == bmin(i) && g(i) > 0) fJx.setCol(i,0.0);
	if(x(i) == bmax(i) && g(i) < 0) fJx.setCol(i,0.0);
      }
    }

    //solve newton step
    if(!SolveUnderconstrainedLS(fJx,fx,p)) {
      //least squares
      MatrixEquation eq(fJx,fx);
      if(!eq.LeastSquares_Cholesky(p)) {
	printf("NewtonRoot::Solve: Unable to compute either pseudoinverse or Cholesky least-squares\n");
	return ConvergenceError;
      }
    }
    p.inplaceNegative();
    if(verbose >= 2) cout<<"  Descent direction "<<p<<endl;
    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    check = LineMinimization(g,p,&f); //lnsrch returns new x and f. It also calculates fx at the new x when it calls Merit()
    //printf("New value of f after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;
    //if(c) { if(!c->Satisfies(x)) cout<<"Warning: line searched x doesnt satisfy contact constraints"<<endl; }
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
        //cout<<"Hmm.... check is returned on iter "<<iters<<", but test is not < tolmin"<<endl; 
	/*
	cout<<"Converging on x!"<<endl;
	OutputASCIIShade(cout,g); cout<<endl;
	OutputASCIIShade(cout,p); cout<<endl;
        return ConvergenceX;
	*/
	if(verbose >= 1) cout<<"  Check is returned but test "<<test<<" is not < tolmin"<<endl;
	return ConvergenceX;
      }
    }
    Real test=0.0; //Test for convergence on dx.
    for (int j=0;j<x.n;j++) {
      Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
      if (temp > test) test=temp; 
    }
    if (test < tolx) {
      if(verbose >= 1) cout<<"  Convergence on X, difference "<<test<<endl;
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
      printf("NewtonRoot::Solve: Unable to compute pseudoinverse of sparse matrix\n");
      return ConvergenceError;
    }
    p.inplaceNegative();
    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    check = LineMinimization(g,p,&f); //lnsrch returns new x and f. It also calculates fx at the new x when it calls Merit()
    //printf("New value of f after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;
    //if(c) { if(!c->Satisfies(x)) cout<<"Warning: line searched x doesnt satisfy contact constraints"<<endl; }
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
        //cout<<"Hmm.... check is returned on iter "<<iters<<", but test is not < tolmin"<<endl; 
	/*
	cout<<"Converging on x!"<<endl;
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
      cerr<<"NewtonRoot::LineMinimization: Error, p is not finite!"<<endl;
      cerr<<"p="<<VectorPrinter(p,VectorPrinter::AsciiShade)<<endl;
      getchar();
    }
    return false;
  }
  if(debug && !IsFinite(g)) {
    if(verbose) {
      cerr<<"NewtonRoot::LineMinimization: Error, g is not finite!"<<endl;
      cerr<<"g="<<VectorPrinter(g,VectorPrinter::AsciiShade)<<endl;
      getchar();
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
	printf("NewtonRoot::LineMinimization: Opposing slope and descent directions\n");
      return false;
    }
    //  else slope = Abs(slope);
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
    if(bmin.n!=0) {
      AABBClamp(x,bmin,bmax);
    }
    *f=Merit();
    if (alam < alamin) { //Convergence on  x. For zero finding, the calling program should verify the convergence. 
      //x.copy(xold);
      return true;
    }
    else if (*f <= fold+ALF*alam*slope) {
      return false; //Sufficient function decrease. 
    }
    else if(!IsFinite(*f)) {
      cerr<<"NewtonRoot::LineMinimization: f(x) is infinite or NaN... backtracking"<<endl;
      /*
      cerr<<"x0="<<VectorPrinter(xold,VectorPrinter::AsciiShade)<<endl;
      cerr<<"p="<<VectorPrinter(p,VectorPrinter::AsciiShade)<<endl;
      cerr<<"g="<<VectorPrinter(g,VectorPrinter::AsciiShade)<<endl;
      cerr<<"lambda="<<alam<<endl;
      getchar();
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
	  cerr<<"NewtonRoot::LineMinimization: templam is NaN??"<<endl;
	  cerr<<"f="<<*f<<endl;
	  cerr<<"fold="<<fold<<endl;
	  cerr<<"a="<<a<<endl;
	  cerr<<"b="<<b<<endl;
	  cerr<<"rhs1="<<rhs1<<endl;
	  cerr<<"hrs2="<<rhs2<<endl;
	  cerr<<"slope="<<slope<<endl;
	  getchar();
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

  if(verbose) { cout<<"ConstrainedNewtonRoot::GlobalSolve(): "; cout.flush(); }
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
    if(verbose) cout<<"Reached convergence on x... ";
    if(endDist <= tolf) {
      if(endMargin < tolc) {
	if(verbose) cout<<"satisfies f but not c: "<<endMargin<<" < "<<tolc<<endl;
	return false;
      }
      if(verbose) cout<<"satisfies all constraints."<<endl;
      return true;
    }
    else {
      if(verbose) cout<<"does not satisfy tolerance, distance "<<endDist<<", margin "<<endMargin<<"."<<endl;
      return false;
    }
    break;

  case LocalMinimum:
    if(verbose) cout<<"Reached local minimum... ";
    if(endDist <= tolf) {
      if(endMargin < tolc) {
	if(verbose) cout<<"satisfies f but not c: "<<endMargin<<" < "<<tolc<<endl;
	return false;
      }
      if(verbose) cout<<"satisfies all constraints."<<endl;
      return true;
    }
    else {
      if(verbose) cout<<"stuck at distance "<<endDist<<"."<<endl;
      return false;
    }
  case ConvergenceF:
    if(endMargin < tolc) {
      if(verbose) cout<<"Reached convergence on f, but not margin "<<endMargin<<endl;
      return false;
    }
    if(verbose) cout<<"Reached convergence on f, new distance "<<endDist<<" margin "<<endMargin<<endl;
    Assert(endDist <= tolf);
    return true;

  case MaxItersReached:
    initDist = MaxDistance(xinit);
    if(endDist < initDist) {
      if(verbose) cout<<"Max iters reached, distance was decreased to "<<endDist<<endl;
    }
    else {
      //if(verbose) cout<<"Max iters reached, looks like divergence.  Reverting to initial."<<endl;
      if(verbose) cout<<"Max iters reached, distance increased (may have reduced inequality margin)."<<endl;
      //x.copy(xinit);
    }
    return false;
  default:
    if(verbose) cout<<"Error"<<endl;
    return false;
  }
  return false;
}

//this one steps across the constraint manifold first, then solves
//inequality constraints while maintaining equality
ConvergenceResult ConstrainedNewtonRoot::SolveConstrained(int& iters)
{
  cerr<<"ConstrainedNewtonRoot::SolveConstrained() is deprecated, use SolveConstrained2()"<<endl;
  getchar();
  Assert(c!=NULL);

  g.resize(x.n);
  int maxIters=iters;
  ConvergenceResult res;
  if(!NewtonRoot::GlobalSolve(iters,&res)) {
    if(verbose) cout<<"SolveConstrained(): Failed on initial equality solve"<<endl;
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

      //cout<<"Maximizing margin "<<margin<<" at "<<c->Label(index)<<endl;
      //take step
      c->Jacobian_i(x,index,g);
      //project onto tangent space of func
      func->PreEval(x);
      func->Jacobian(x,fJx);
      if(!svd.set(fJx)) {
	if(verbose) cout<<"SolveConstrained(): error setting SVD of equality jacobian"<<endl;
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
	//cout<<"line search iteration "<<lsiters<<", step length "<<t<<endl;
	if(t < tolx) {
	  //cout<<"No progress in SolveConstrained() after "<<iters<<" iters, looks like a local minimum"<<endl;
	  return LocalMinimum;
	}
	x.copy(xold2); 
	x.madd(g,t/gnorm);
	if(bmin.n!=0) AABBClamp(x,bmin,bmax);

	//cout<<"Step has distance "<<MaxDistance(x)<<" from eq"<<endl;
	int solveIters=maxIters-iters;
	if(NewtonRoot::GlobalSolve(solveIters,&res) || res==ConvergenceX) {
	  //cout<<"Used "<<solveIters<<" iters of "<<maxIters-iters<<endl;
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
	    case LocalMinimum: cout<<"LocalMinimum "; break;
	    case ConvergenceError: cout<<"Error "; break;
	    default: break;
	    }
	    cout<<"Equality solve failed, using "<<solveIters<<" iters of "<<maxIters-iters<<" to solve distance "<<MaxDistance(x)<<endl;
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
      cerr<<"ERROR! Initial x is not finite!"<<endl;
      cerr<<VectorPrinter(x)<<endl;
      getchar(); 
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
      if(verbose) cerr<<"f(x) is not finite!"<<endl;
      if(verbose >= 2) {
	cerr<<VectorPrinter(fx)<<endl;
	getchar();
      }
      return ConvergenceError;
    }
    if(debug && !IsFinite(cx)) {
      if(verbose) cerr<<"c(x) is not finite!"<<endl;
      if(verbose >= 2) {
	cerr<<VectorPrinter(cx)<<endl;
	getchar();
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
      cout<<"Equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index)<<endl;
    }
    if(verbose >= 3 && !activeSetC.empty()) {
      cout<<"Active set: {";
      for(size_t i=0;i<activeSetC.size();i++)
	cout<<c->Label(activeSetC[i])<<",";
      cout<<"}"<<endl;
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
	cerr<<"Jacobian of equality is not finite!"<<endl;
	cerr<<VectorPrinter(x)<<endl;
	getchar();
      }
      return ConvergenceError;
    }
    for(size_t i=0;i<activeSetC.size();i++) {
      Vector Ai; A.getRowRef(fn+(int)i,Ai);
      c->Jacobian_i(x,activeSetC[i],Ai);
      if(debug && !IsFinite(Ai)) {
	if(verbose) {
	  cerr<<"Jacobian "<<i<<" of inequality is not finite!"<<endl;
	  cerr<<VectorPrinter(Ai)<<endl;
	  cerr<<VectorPrinter(x)<<endl;
	  getchar();
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
	    cout<<"Active bounds: {";
	    for(size_t i=0;i<activeSetBound.size();i++)
	      cout<<activeSetBound[i]<<",";
	    cout<<"}"<<endl;
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
	    if(verbose) cout<<"Unable to solve pseudoinverse of bound-constrained problem, just setting entries of dirs to 0"<<endl;
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
      cout<<"desired f: "<<VectorPrinter(vtemp,VectorPrinter::AsciiShade)<<endl;
      vtemp.resize(cn);
      (*c)(xtemp,vtemp);
      cout<<"desired c min: "<<vtemp.minElement()<<endl;

      //A*p = J(x)*p = rhs = f(x)
      vtemp.resize(A.m);
      A.mul(p,vtemp);
      vtemp -= rhs;
      vtemp.inplaceNegative();
      Vector temp; temp.setRef(vtemp,fn);
      cout<<"linear c min: "<<temp.minElement()<<endl;
      */
    }
    else if(fx.maxAbsElement() > -cx.minElement()) {  //seems more important to solve equality constraints
      if(verbose) {
	cerr<<"Unable to solve pseudoinverse of equalities+inequalities, trying unconstrained"<<endl;
      }
      if(verbose >= 2) {
	MatrixPrinter mp(A);
	if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	cerr<<mp<<endl;
	cerr<<VectorPrinter(x)<<endl;
	//cerr<<"New dims: "<<fn<<" x "<<A.n<<endl;
	getchar();
      }
      A.m = fn;
      rhs.n = fn;
      if(verbose >= 2) {
	cerr<<"New matrix: "<<endl;
	MatrixPrinter mp(A);
	if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	cerr<<mp<<endl;
	getchar();
      }
      A.mulTranspose(rhs,g);
      if(SolveUnderconstrainedLS(A,rhs,p)) {
      }
      else {
	if(verbose) cerr<<"Unable to solve pseudoinverse!"<<endl;
	if(verbose >= 2) {
	  MatrixPrinter mp(A);
	  if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	  cerr<<mp<<endl;
	  cerr<<"Press a key to continue"<<endl;
	  getchar();
	}
	p=g;
	//return ConvergenceError;
      }
    }
    else {  //seems more important to solve inequality constraints
      if(verbose) {
	cerr<<"Unable to solve pseudoinverse of equalities, trying inequality"<<endl;
      }
      for(size_t i=0;i<activeSetC.size();i++) {
	for(int j=0;j<A.n;j++)
	  A(i,j) = A(fn+i,j);
	rhs(i) = rhs(fn+i);
      }
      A.m=(int)activeSetC.size();
      rhs.n=(int)activeSetC.size();
      if(verbose >= 2) {
	cerr<<"New matrix: "<<endl;
	MatrixPrinter mp(A);
	if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	cerr<<mp<<endl;
	getchar();
      }
      A.mulTranspose(rhs,g);
      if(SolveUnderconstrainedLS(A,rhs,p)) {
      }
      else {
	if(verbose) cerr<<"Unable to solve pseudoinverse!"<<endl;
	if(verbose >= 2) {
	  MatrixPrinter mp(A);
	  if(A.n > 10) mp.mode = MatrixPrinter::AsciiShade;
	  cerr<<mp<<endl;
	  cerr<<"Press a key to continue"<<endl;
	  getchar();
	}
	p=g;
	//return ConvergenceError;
      }
    }
    if(g.dot(p) <= Zero) {
      if(verbose) {
	cerr<<"Error, gradient and search direction are opposing..."<<endl;
	cerr<<"g: "<<VectorPrinter(g)<<endl;
	cerr<<"p: "<<VectorPrinter(p)<<endl;
	cerr<<"g dot p: "<<g.dot(p)<<endl;
	cerr<<"Using gradient direction..."<<endl;
      }
      p=g;
      //return ConvergenceError;
    }
    p.inplaceNegative();
    if(verbose>=2) cout<<"Line search direction: "<<p<<endl;

    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    f = Merit();
    check = LineMinimization(g,p,&f);
    //printf("New value of f after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;

    //Note: fx updated by last call to merit function
    if(debug && !IsFinite(x)) { 
      if(verbose) {
	cerr<<"ERROR! after LineMinimization, x is not finite!"<<endl;
	cerr<<VectorPrinter(x,VectorPrinter::AsciiShade)<<endl;
	getchar();
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
	  cout<<"ConstrainedNewtonRoot(): Hmm.... check is returned, but test is not < tolmin"<<endl; 
	  cout<<"Gradient: "<<g<<endl;
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
      cerr<<"ERROR! Initial x is not finite!"<<endl;
      cerr<<VectorPrinter(x)<<endl;
      getchar(); 
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
      if(verbose) cerr<<"f(x) is not finite!"<<endl;
      if(verbose >= 2) {
	cerr<<VectorPrinter(fx)<<endl;
	getchar();
      }
      return ConvergenceError;
    }
    if(debug && !IsFinite(cx)) {
      if(verbose) cerr<<"c(x) is not finite!"<<endl;
      if(verbose >= 2) {
	cerr<<VectorPrinter(cx)<<endl;
	getchar();
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
      cout<<"Equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index)<<endl;
    }
    if(verbose >= 3 && !activeSetC.empty()) {
      cout<<"Active set: {";
      for(size_t i=0;i<activeSetC.size();i++)
	cout<<c->Label(activeSetC[i])<<",";
      cout<<"}"<<endl;
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
    if(verbose) cout<<"Evaluating jacobian..."; cout.flush();
    seq->Jacobian_Sparse(x,Jeq);
    A.copySubMatrix(0,0,Jeq);
    if(debug && !IsFinite(Jeq)) {
      if(verbose) {
	cerr<<"Jacobian of equality is not finite!"<<endl;
	cerr<<VectorPrinter(x)<<endl;
	getchar();
      }
      return ConvergenceError;
    }
    for(size_t i=0;i<activeSetC.size();i++) {
      SparseVector Ai(x.n);
      sineq->Jacobian_i_Sparse(x,activeSetC[i],Ai);
      A.copyRow(fn+i,Ai);
      if(debug && !IsFinite(Ai)) {
	if(verbose) {
	  cerr<<"Jacobian "<<i<<" of inequality is not finite!"<<endl;
	  Vector temp;
	  Ai.get(temp);
	  cerr<<VectorPrinter(temp)<<endl;
	  cerr<<VectorPrinter(x)<<endl;
	  getchar();
	}
	return ConvergenceError;
      }
    }
    if(verbose) cout<<"Done."<<endl;

    if(!activeSetBound.empty()) {
      if(verbose >= 2) {
	cout<<"Active bounds: {";
	for(size_t i=0;i<activeSetBound.size();i++)
	  cout<<activeSetBound[i]<<",";
	cout<<"}"<<endl;
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
	cerr<<"ConstrainedNewtonRoot: Unable to solve pseudoinverse of equalities+inequalities, returning error"<<endl;
      p.n = p.n + (int)activeSetBound.size();
      SetElements(p,activeSetBound,Zero);
      return ConvergenceError;
    }
    if(g.dot(p) <= Zero) {
      if(verbose) {
	cerr<<"Error, gradient and search direction are opposing..."<<endl;
	cerr<<"g: "<<VectorPrinter(g)<<endl;
	cerr<<"p: "<<VectorPrinter(p)<<endl;
	cerr<<"g dot p: "<<g.dot(p)<<endl;
	cerr<<"Using gradient direction..."<<endl;
      }
      p=g;
      //return ConvergenceError;
    }
    p.inplaceNegative();

    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    f = Merit();
    check = LineMinimization(g,p,&f);
    //printf("New value of f after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;

    //Note: fx updated by last call to merit function
    if(debug && !IsFinite(x)) { 
      if(verbose) {
	cerr<<"ERROR! after LineMinimization, x is not finite!"<<endl;
	cerr<<VectorPrinter(x,VectorPrinter::AsciiShade)<<endl;
	getchar();
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
        if(verbose) cout<<"ConstrainedNewtonRoot(): Hmm.... check is returned, but test is not < tolmin"<<endl; 
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
      cerr<<"ERROR! Initial x is not finite!"<<endl;
      cerr<<VectorPrinter(x)<<endl;
      getchar(); 
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
      if(verbose) cerr<<"f(x) is not finite!"<<endl;
      if(verbose >= 2) {
	cerr<<VectorPrinter(fx)<<endl;
	getchar();
      }
      return ConvergenceError;
    }
    if(debug && !IsFinite(cx)) {
      if(verbose) cerr<<"c(x) is not finite!"<<endl;
      if(verbose >= 2) {
	cerr<<VectorPrinter(cx)<<endl;
	getchar();
      }
      return ConvergenceError;
    }

    if(verbose) {
      int index;
      Real d=cx.minElement(&index);
      cout<<"Equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index)<<endl;
    }

    //setup the problem
    //min||x-x0||
    //C(x) = 0  => dC/dx(x0)*(x-x0) + C(x0) = 0 => JC*dx = -C(x0)
    //D(x) > 0  => dD/dx(x0)*(x-x0) + D(x0) > 0 => -JD*dx < D(x0)
    //bmin <= x0+dx <= bmax => bmin-x0 <= dx <= bmax-x0
    if(verbose) cout<<"Evaluating jacobians..."; cout.flush();
    seq->Jacobian_Sparse(x,Aeq);
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
    if(verbose) cout<<"Done."<<endl;
    Assert(lp.IsValid());

    p.resize(x.n); p.setZero();
    /*
    cout<<"********** Zero Vector: ***********"<<endl;
    cout<<"L-inf norm: "<<lp.Norm(p)<<endl;
    cout<<"Equality error: "<<lp.EqualityError(p)<<endl;
    cout<<"Inequality error: "<<lp.InequalityMargin(p)<<endl;
    cout<<"************************************"<<endl;
    */

    lp.Assemble();
    LinearProgram::Result res=lp.Solve(p);
    if(res == LinearProgram::Infeasible || res == LinearProgram::Error) {
      if(verbose) cout<<"LP failed, solving minimum error problem..."<<endl;
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
      cout<<"********** Zero Vector: ***********"<<endl;
      cout<<"L1 norm: "<<lp2.Norm(temp)<<endl;
      cout<<"Equality error: "<<lp2.EqualityError(temp)<<endl;
      cout<<"Inequality error: "<<lp2.InequalityMargin(temp)<<endl;
      cout<<"************************************"<<endl;
      */
      /*
      lp2.Assemble();
      res=lp2.Solve(temp);
    */
      res = LinearProgram::Infeasible;
      if(res == LinearProgram::Infeasible || res == LinearProgram::Error) {
	if(verbose) cout<<"Minimum error problem failed, solving LS..."<<endl;
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
	  cout<<"Solved using least-squares solution..."<<endl;
	}
	else {
	  if(verbose) cerr<<"Unable to solve for a feasible step!"<<endl;
	  return ConvergenceError;
	}
      }
      /*
      else {
	cout<<"********* Min-norm solution 2: **********"<<endl;
	cout<<"L1 norm of shift: "<<lp2.Norm(temp)<<endl;
	cout<<"Equality error: "<<lp2.EqualityError(temp)<<endl;
	cout<<"Inequality error: "<<lp2.InequalityMargin(temp)<<endl;
	cout<<"************************************"<<endl;
	//get just the top n elements
	temp.getSubVectorCopy(0,p);
      }
      */
    }
    Assert(res != LinearProgram::Unbounded);

    /*
    cout<<"********* Min-norm solution: ************"<<endl;
    cout<<"L-inf norm of shift: "<<lp.Norm(p)<<endl;
    cout<<"Equality error: "<<lp.EqualityError(p)<<endl;
    cout<<"Inequality error: "<<lp.InequalityMargin(p)<<endl;
    cout<<"************************************"<<endl;
    */

    cout<<"Length of shift: "<<p.norm()<<endl;
    xold = x;
    x += p;
    (*func)(x,fx);
    (*c)(x,cx);
    int index;
    Real d=cx.minElement(&index);
    cout<<"Shifted equality error "<<fx.maxAbsElement()<<", Inequality margin: "<<d<<" at "<<c->Label(index)<<endl;
    x = xold;
    (*func)(x,fx);
    (*c)(x,cx);

    //fake a gradient direction
    g.setNegative(p);
    f = Merit();
    if(verbose) printf("Merit value before lnsrch: %f\n",f);
    check = LineMinimization(g,p,&f);
    if(verbose) printf("Merit value after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;

    //Note: fx updated by last call to merit function
    if(debug && !IsFinite(x)) { 
      if(verbose) {
	cerr<<"ERROR! after LineMinimization, x is not finite!"<<endl;
	cerr<<VectorPrinter(x,VectorPrinter::AsciiShade)<<endl;
	getchar();
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
        if(verbose) cout<<"ConstrainedNewtonRoot(): Hmm.... check is returned, but test is not < tolmin"<<endl; 
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
