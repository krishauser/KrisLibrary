#include <KrisLibrary/Logger.h>
#include "Minimization.h"
#include <math/LDL.h>
#include <math/AABB.h>
#include <math/differentiation.h>
#include <math/brent.h>
#include <math/metric.h>
#include <math/QuasiNewton.h>
#include <math/indexing.h>
#include <math/sparsefunction.h>
#include <math/MatrixPrinter.h>
#include <iostream>
#include "LPRobust.h"
//#include "LSQRInterface.h"
using namespace std;
using namespace Optimization;

#define kLineMinMaxIters 20

#define boundEpsilon tolx

enum { DFS, BFGS };
const static int updateType = BFGS;

MinimizationProblem::MinimizationProblem(ScalarFieldFunction* _f)
  :f(_f),tolx(1e-7),tolf(1e-5),tolgrad(1e-8),verbose(1),S(NULL)
{}



ConvergenceResult MinimizationProblem::SolveGD(Real alpha,int& iters)
{
  grad.resize(x.n);
  Real fx0;
  fx = (*f)(x);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    //Step()
    fx0 = fx;
    f->Gradient(x,grad);
    x.madd(grad,-alpha);
    fx=(*f)(x);
    if(S) S->push_back(x);

    //termination criteria
    if(grad.maxAbsElement()*alpha <= tolx)
      return ConvergenceX;
    if(Abs(fx0-fx) < tolf*alpha)
      return ConvergenceF;
    if(grad.maxAbsElement()*alpha > 1e8)
      return Divergence;
  }
  return MaxItersReached;
}

void MinimizationProblem::StepGD(Real alpha)
{
  grad.resize(x.n);
  f->PreEval(x);
  f->Gradient(x,grad);
  Real dx=-alpha;
  x.madd(grad,dx);
}

ConvergenceResult MinimizationProblem::SolveSD(int& iters)
{
  grad.resize(x.n);
  int maxIters=iters;
  fx=(*f)(x); 
  for(iters=0;iters<maxIters;iters++) {
    f->Gradient(x,grad);
    grad.inplaceNegative();
    Real alpha0=1;
    ConvergenceResult res=LineMinimizationStep(grad,alpha0);
    if(res != MaxItersReached) return res;
  }
  return MaxItersReached;
}

/*
Fletcher-Reeves-Polak-Ribiere minimization
gradient of f must be defined!
*/
ConvergenceResult MinimizationProblem::SolveCG(int& iters)
{
  Real gg,gam,fx0,dgg;
  Vector g,h;
  grad.resize(x.n);
  fx=(*f)(x);
  f->Gradient(x,grad);
  g.setNegative(grad);
  grad=h=g;
  int maxIters=iters;
  for (iters=0;iters<maxIters;iters++) {
    fx0 = fx;
    Real dx = ParabolicLineMinimization(*f,x,grad,kLineMinMaxIters,tolx);
    x.madd(grad,dx);
    fx=(*f)(x);
    if(S) S->push_back(x);

    if (2.0*Abs(fx0-fx) <= tolf*(Abs(fx0)+Abs(fx)+Epsilon)) {
      return ConvergenceF;
    }
    f->Gradient(x,grad);
    dgg=gg=0.0;
    gg = g.normSquared();
    //NOTE: Polak-Ribiere, comment out grad.dot(g) for Fletcher-Reeves
    dgg = grad.normSquared() + grad.dot(g); 
    if (Sqrt(gg) < tolgrad ) {  //gradient exactly zero, return
      return ConvergenceF;
    }
    gam=Max(dgg/gg,0.0);
    g.setNegative(grad);
    grad = g; grad.madd(h,gam);
    h=grad;
  }
  return MaxItersReached;
}

ConvergenceResult MinimizationProblem::SolveNewton(int& iters)
{
  grad.resize(x.n);
  H.resize(x.n,x.n);
  dx.resize(x.n);
  LDLDecomposition<Real> ldl;
  fx = (*f)(x);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    f->Gradient(x,grad);
    f->Hessian(x,H);

    //f(dx + x0) ~= f(x0) + grad_f*dx + 1/2 dx'*H_f*dx 
    //grad(f)(x+dx) ~= grad_f + 1/2*H_f*dx + O(dx^2)
    //setting = grad_f(x+dx) = 0, we see dx ~= -H_f^-1 grad_f
    //use that as the backtracking search direction (if it's in same direction as grad)
    ldl.set(H);
    Vector d;
    ldl.LDL.getDiagRef(0,d);
    if(d.minElement() < 0) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"Warning, hessian is not positive definite");
      for(int i=0;i<d.n;i++)
	if(d(i) < 1e-4) d(i) = 1e-4;
      //return ConvergenceError;
    }
    ldl.backSub(grad,dx);
    if(dot(dx,grad) < Zero) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"MinimizationProblem::SolveNewton(): Warning, direction opposes gradient on step "<<iters);
      //dx.setNegative(grad);
    }
    dx.inplaceNegative();

    Real alpha0 = 1;
    ConvergenceResult res = LineMinimizationStep(dx,alpha0);
    if(res != MaxItersReached) return res;
  }
  return MaxItersReached;
}

ConvergenceResult MinimizationProblem::SolveQuasiNewton_Ident(int& maxIters)
{
  H.resize(x.n,x.n);
  H.setIdentity();
  return SolveQuasiNewton(maxIters);
}

ConvergenceResult MinimizationProblem::SolveQuasiNewton_Diff(Real dx,int& maxIters)
{
  H.resize(x.n,x.n);
  HessianCenteredDifference_Grad(*f,x,dx,H);
  return SolveQuasiNewton(maxIters);
}

ConvergenceResult MinimizationProblem::SolveQuasiNewton(int& iters)
{
  Assert(H.m == x.n && H.n == x.n);
  grad.resize(x.n);
  Vector x0,grad0,s,q,upd;
  QNHessianUpdater qn;
  qn.verbose = verbose;
  dx.resize(x.n);
  qn.SetHessian(H);
  bool posDef = qn.IsPositiveDefinite(1e-5);
  if(!posDef) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"MinimizationProblem::SolveQuasiNewton(): initial hessian is not positive definite with tolerance 1e-5");
    H.setIdentity();
    qn.SetHessian(H);
  }
  fx = (*f)(x);
  f->Gradient(x,grad);

  Matrix trueH;
  if(verbose >= 3) {
    trueH.resize(x.n,x.n);
    f->Hessian(x,trueH);
    LOG4CXX_ERROR(KrisLibrary::logger(),"Iter "<<0<<", Norm_f of error "<<Distance_Frobenius(H,trueH));
  }

  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    qn.BackSub(grad,dx);
    if(dot(dx,grad) < Zero) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"MinimizationProblem::SolveQuasiNewton(): Warning, direction opposes gradient on step "<<iters);
      //dx.setNegative(grad);
    }
    dx.inplaceNegative();

    x0 = x;
    grad0 = grad;

    Real alpha0=1;
    ConvergenceResult res = LineMinimizationStep(dx,alpha0);
    if(res != MaxItersReached) return res;

    //update the gradient
    f->Gradient(x,grad);

    s.sub(x,x0);
    q.sub(grad,grad0);

    if(updateType == BFGS) {
      bool res=qn.UpdateBFGS(s,q);
      if(!res) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to update the hessian approximation, ignoring update");
	    }
    }
    else if(updateType == DFS) {
      bool res=qn.UpdateDFS(s,q);
      if(!res) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to update the hessian approximation, ignoring update");
	    }
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"Unknown type of quasi-Newton update");
      Abort();
    }

    if(verbose >= 3) {
      //test
      qn.GetHessian(H);
      f->Hessian(x,trueH);
      LOG4CXX_ERROR(KrisLibrary::logger(),"Iter "<<iters+1<<", Norm_f of error "<<Distance_Frobenius(H,trueH));
      LOG4CXX_INFO(KrisLibrary::logger(),"H':"<<MatrixPrinter(H)<<"\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"H:"<<MatrixPrinter(trueH)<<"\n");
      KrisLibrary::loggerWait();
    }
  }
  return MaxItersReached;
}

ConvergenceResult MinimizationProblem::SolveLM(int& iters,Real lambda0,Real lambdaScale)
{
  Real lambda=lambda0;
  grad.resize(x.n);
  H.resize(x.n,x.n);
  dx.resize(x.n);
  Vector x0,diagH;
  LDLDecomposition<Real> ldl;
  Real fx0;
  fx = (*f)(x);
  bool tookStep=true;
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    if(tookStep) {
      f->Gradient(x,grad);
      f->Hessian(x,H);
      H.getDiagCopy(0,diagH);
    }
    for(int i=0;i<x.n;i++) H(i,i) += Max((Real)1e-4,diagH(i))*lambda;

    ldl.set(H);
    Vector d; ldl.LDL.getDiagRef(0,d);
    for(int i=0;i<x.n;i++) {
      if(d(i) <= 1e-3) { //non-PD or nearly non-PD (adjust lambda?)
	d(i) = 1e-3;
      }
    }
    fx0 = fx;
    x0 = x;
    ldl.backSub(grad,dx);
    dx.inplaceNegative();
    x += dx;

    //check if the step works or not
    fx = (*f)(x);
    if(Abs(fx0 - fx) < tolf) return ConvergenceF;
    if(grad.maxAbsElement() < tolgrad) return ConvergenceF;
    if(dx.maxAbsElement() < tolx) return ConvergenceX;
    if(fx < fx0) {
      tookStep=true;
      lambda /= lambdaScale;
    }
    else {
      tookStep=false;
      x = x0;
      fx = fx0;
      H.copyDiag(0,diagH); 
      lambda *= lambdaScale;
    }
  }
  return MaxItersReached;
}

#define ALF 1e-4

ConvergenceResult MinimizationProblem::LineMinimizationStep(const Vector& dx,Real& alpha0)
{
  Real fx0=fx;
  Vector x0=x;
  Real normdx = dx.maxAbsElement();
  Real slope = Abs(dx.dot(grad));

  Real t = alpha0;
  for(int lineSearchIters=1;;lineSearchIters++) {
    if(Abs(t) * normdx < tolx) {
      if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"MinimizationProblem::LineMinimizationStep(): Quitting on line search iter "<<lineSearchIters);
      x = x0; alpha0 = 0;
      return ConvergenceX;
    }

    x = x0;   x.madd(dx,t);
    fx = (*f)(x);
    if(fx < fx0-ALF*Abs(t)*slope) break; //ensure sufficient function decrease
    t *= 0.5;
  }

  /*
  if(alpha0 != 1) {
    if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"MinimizationProblem::LineMinimizationStep(): Warning, alpha0 is set to something other than 1");
  }
  Real t=f->LineMinimization(x,dx);
  x.madd(dx,t);
  fx = (*f)(x);
  */

  alpha0 = t;
  if(S) S->push_back(x);

  //termination criteria
  if(normdx <= tolgrad)
    return ConvergenceF;
  if(normdx*Abs(t) <= tolx)
    return ConvergenceX;
  if(Abs(fx0-fx) < tolf*Abs(t)) 
    return ConvergenceF;
  if(normdx*Abs(t) > 1e8)
    return Divergence;
  return MaxItersReached;
}


BCMinimizationProblem::BCMinimizationProblem(ScalarFieldFunction* _f)
  :f(_f),tolx(1e-7),tolf(1e-5),tolgrad(1e-8),verbose(1),S(NULL)
{}

ConvergenceResult BCMinimizationProblem::SolveSD(int& iters)
{
  int maxIters=iters;
  fx=(*f)(x); 
  for(iters=0;iters<maxIters;iters++) {
    f->Gradient(x,grad);
    grad.inplaceNegative();
    Real alpha0=1;
    ConvergenceResult res=LineMinimizationStep(grad,alpha0);
    if(res != MaxItersReached) return res;
  }
  return MaxItersReached;
}

ConvergenceResult BCMinimizationProblem::SolveNewton(int& iters)
{
  grad.resize(x.n);
  H.resize(x.n,x.n);
  Vector dx(x.n);
  LDLDecomposition<Real> ldl;
  fx = (*f)(x);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    f->Gradient(x,grad);
    f->Hessian(x,H);

    //f(dx + x0) ~= f(x0) + grad_f*dx + 1/2 dx'*H_f*dx 
    //grad(f)(x+dx) ~= grad_f + 1/2*H_f*dx + O(dx^2)
    //setting = grad_f(x+dx) = 0, we see dx ~= -H_f^-1 grad_f
    //use that as the backtracking search direction (if it's in same direction as grad)
    ldl.set(H);
    Vector d;
    ldl.LDL.getDiagRef(0,d);
    if(d.minElement() < 0) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"Warning, hessian is not positive definite");
      for(int i=0;i<d.n;i++)
	if(d(i) < 1e-4) d(i) = 1e-4;
      //return ConvergenceError;
    }
    ldl.backSub(grad,dx);
    if(dot(dx,grad) < Zero) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"BCMinimizationProblem::SolveNewton(): Warning, hessian direction opposes gradient on step "<<iters);
      //dx.setNegative(grad);
    }
    dx.inplaceNegative();

    Real alpha0 = 1;
    ConvergenceResult res = LineMinimizationStep(dx,alpha0);
    if(res != MaxItersReached) return res;
  }
  return MaxItersReached;
}

ConvergenceResult BCMinimizationProblem::SolveQuasiNewton_Ident(int& maxIters)
{
  H.resize(x.n,x.n);
  H.setIdentity();
  return SolveQuasiNewton(maxIters);
}

ConvergenceResult BCMinimizationProblem::SolveQuasiNewton_Diff(Real dx,int& maxIters)
{
  H.resize(x.n,x.n);
  HessianCenteredDifference_Grad(*f,x,dx,H);
  return SolveQuasiNewton(maxIters);
}

ConvergenceResult BCMinimizationProblem::SolveQuasiNewton(int& iters)
{
  Assert(H.m == x.n && H.n == x.n);
  grad.resize(x.n);
  Vector x0,grad0,s,q,upd,temp,dx(x.n);
  LDLDecomposition<Real> ldl;
  ldl.set(H);
  fx = (*f)(x);
  f->Gradient(x,grad);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    ldl.backSub(grad,dx);
    if(dot(dx,grad) < Zero) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"BCMinimizationProblem::SolveQuasiNewton(): Warning, hessian direction opposes gradient on step "<<iters);
      //dx.setNegative(grad);
    }
    grad.inplaceNegative();

    x0 = x;
    grad0 = grad;

    Real alpha0=1.0;
    ConvergenceResult res = LineMinimizationStep(dx,alpha0);
    if(res != MaxItersReached) return res;

    //update the gradient
    f->Gradient(x,grad);
    //Hessian update step
    //BFGS has H' = H + q*qt / qt*s - Ht*s*st*H/st*H*s
    //where s = x - x0
    //q = grad - grad0
    s.sub(x,x0);
    q.sub(grad,grad0);
    //first part
    Real qdots = q.dot(s);
    if(qdots <= 0) {
      if(verbose >= 2) LOG4CXX_WARN(KrisLibrary::logger(),"BCMinimizationProblem: Warning, gradient update is in wrong direction\n");
      //revert to identity
      ldl.LDL.setIdentity();
      continue;
    }
    else {
      upd.div(q,Sqrt(qdots));
      ldl.update(upd);
    }

    //second part
    //multiply s by H => upd
    ldl.mulLT(s,temp);
    ldl.mulD(temp,temp);
    ldl.mulL(temp,upd);
    Real sHs = upd.dot(s);
    Assert(sHs > 0);
    upd /= Sqrt(sHs);
    if(!ldl.downdate(upd)) {
      if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to maintain strict positive definiteness of hessian!");
      //revert to identity
      ldl.LDL.setIdentity();
      continue;
      //TODO: fix the downdate
      //return ConvergenceError;
    }
    Vector d;
    ldl.LDL.getDiagRef(0,d);
    if(d.minElement() <= 0) {
      if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to maintain positive definiteness of hessian!");
      return ConvergenceError;
    }
  }
  return MaxItersReached;
}

ConvergenceResult BCMinimizationProblem::LineMinimizationStep(Vector& dx,Real& alpha0)
{
  if(alpha0 != 1) {
    if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"BCMinimizationProblem::LineMinimizationStep(): Warning, alpha0 is set to something other than 1");
  }
  Assert(AABBContains(x,bmin,bmax));
  activeSet.resize(x.n);
  for(int i=0;i<dx.n;i++) {
    if(FuzzyEquals(x(i),bmin(i),boundEpsilon) ||
       FuzzyEquals(x(i),bmax(i),boundEpsilon)) 
      activeSet[i] = false;
    else
      activeSet[i] = true;
  }
  for(int i=0;i<dx.n;i++) {
    //free an index if the direction opposes the constraint direction
    if(!activeSet[i]) {
      if(FuzzyEquals(x(i),bmin(i),boundEpsilon)) {
	if(dx(i) > 0) activeSet[i] = false;
      }
      else {
	Assert(FuzzyEquals(x(i),bmax(i),boundEpsilon));
	if(dx(i) < 0) activeSet[i] = false;
      }
    }
    if(!activeSet[i])
      dx(i) = 0;
  }
  Real fx0=fx;
  Real t=ParabolicLineMinimization(*f,x,dx,kLineMinMaxIters,tolx);
  AABBLineSearch(x,dx,bmin,bmax,t);
  alpha0 = t;
  x.madd(dx,t);
  if(S) S->push_back(x);
  fx = (*f)(x);

  //termination criteria
  Real normdx = dx.maxAbsElement();
  if(normdx <= tolgrad) 
    return ConvergenceF;
  if(normdx*Abs(t) <= tolx)
    return ConvergenceX;
  if(Abs(fx0-fx) < tolf)
    return ConvergenceF;
  if(normdx*Abs(t) > 1e8)
    return Divergence;
  return MaxItersReached;
}



#if 0


/*
ConstrainedMinimizationProblem::ConstrainedMinimizationProblem(ScalarFieldFunction* _f,ImplicitManifold* M)
  :f(_f),C(M->C),D(M->D),
   tolx(1e-7),tolf(1e-5),tolgrad(1e-8),tolc(M->ctol),
   verbose(1),S(NULL),rootSolver(M->C,M->D)
{
  if(!M->bmin.isEmpty()) {
    bmin.setRef(M->bmin);
    bmax.setRef(M->bmax);
  }
}
*/

ConstrainedMinimizationProblem::ConstrainedMinimizationProblem(ScalarFieldFunction* _f,VectorFieldFunction*_c,VectorFieldFunction*_d)
  :f(_f),C(_c),D(_d),
   tolx(1e-7),tolf(1e-5),tolgrad(1e-8),tolc(1e-6),sparse(false),
   verbose(1),S(NULL),rootSolver(_c,_d)
{
}

bool ConstrainedMinimizationProblem::CheckPoint(const Vector& x) const
{
  if(C&&!SatisfiesEquality(C,x,tolc)) {
    if(verbose>=1) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimizationProblem::Solve(): Error, initial point\n    doesn't satisfy equality constraints");
      Vector v(C->NumDimensions());
      (*C)(x,v);
      int index;
      Real max=v.maxAbsElement(&index);
      LOG4CXX_ERROR(KrisLibrary::logger(),"    Error "<<max<<" at "<<C->Label(index)<<" exceeds threshold "<<tolc);
    }
    return false;
  }
  if(D&&!SatisfiesInequality(D,x,Zero)) {
    if(verbose>=1) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimizationProblem::Solve(): Error, initial point\n    doesn't satisfy inequality constraints");
      Vector v(D->NumDimensions());
      (*D)(x,v);
      int index;
      Real max=v.minElement(&index);
      LOG4CXX_ERROR(KrisLibrary::logger(),"    Error "<<max<<" at "<<D->Label(index)<<" is less than zero");
    }
    return false;
  }
  if(!bmin.isEmpty()&&!AABBContains(x,bmin,bmax)) {
    if(verbose>=1) LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimizationProblem::Solve(): Error, initial point doesn't satisfy bound constraints");
    return false;
  }
  return true;
}


ConvergenceResult ConstrainedMinimizationProblem::SolveGD(int& iters)
{
  if(!CheckPoint(x)) return ConvergenceError;

  fx=(*f)(x);
  grad.resize(x.n);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    f->Gradient(x,grad);
    grad.inplaceNegative();

    Real dxnorm0 = grad.maxAbsElement();
    NullspaceProjection(x,grad);
    Real dxnorm = grad.maxAbsElement();
    if(dxnorm < tolgrad) {
      if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Projected gradient is 0!");
      return LocalMinimum;
    }
    Real alpha0 = Min(dxnorm0/dxnorm,(Real)10);
    ConvergenceResult res=LineMinimizationStep(grad,alpha0);
    if(res != MaxItersReached) return res;
  }
  if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimzationProblem::Solve(): Max iters reached "<<iters<<".");
  return MaxItersReached;
}


ConvergenceResult ConstrainedMinimizationProblem::StepGD()
{
  if(!CheckPoint(x)) return ConvergenceError;

  fx=(*f)(x);
  grad.resize(x.n);
  f->Gradient(x,grad);
  grad.inplaceNegative();

  //HACK!
  Real dxnorm0 = grad.maxAbsElement();
  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization: Projecting gradient to nullspace...");
  NullspaceProjection(x,grad);
  LOG4CXX_INFO(KrisLibrary::logger(),"Done");
  Real dxnorm = grad.maxAbsElement();
  if(dxnorm < tolgrad) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Projected gradient is 0!");
    return LocalMinimum;
  }
  Real alpha0 = Min(dxnorm0/dxnorm,(Real)1);
  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization: Line minimization...");
  ConvergenceResult res = LineMinimizationStep(grad,alpha0);
  LOG4CXX_INFO(KrisLibrary::logger(),"Done");
  return res;
}

ConvergenceResult ConstrainedMinimizationProblem::SolveNewton(int& iters)
{
  if(!CheckPoint(x)) return ConvergenceError;

  Vector dx;
  fx=(*f)(x);
  grad.resize(x.n);
  H.resize(x.n,x.n);
  LDLDecomposition<Real> ldl;
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    f->Gradient(x,grad);
    f->Hessian(x,H);
    ldl.set(H);
    Vector d;
    ldl.LDL.getDiagRef(0,d);
    if(d.minElement() < 0) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"Warning, hessian is not positive definite");
      for(int i=0;i<d.n;i++)
	if(d(i) < 1e-4) d(i) = 1e-4;
      //return ConvergenceError;
    }
    ldl.backSub(grad,dx);
    if(dot(dx,grad) < Zero) {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveNewton(): Warning, hessian direction opposes gradient on step "<<iters);
      //dx.setNegative(grad);
    }
    dx.inplaceNegative();

    Real dxnorm0 = dx.maxAbsElement();
    NullspaceProjection(x,dx);
    Real dxnorm = dx.maxAbsElement();
    Real alpha0 = Min(dxnorm0/dxnorm,(Real)10);
    ConvergenceResult res=LineMinimizationStep(dx,alpha0);
    if(res != MaxItersReached) return res;
  }
  if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimzationProblem::Solve(): Max iters reached "<<iters<<".");
  return MaxItersReached;
}

Real Merit(Real f,const Vector& c,const Vector& d,Real fScale=One)
{
  Real m=fScale*f + c.normSquared();
  for(int i=0;i<d.n;i++)
    if(d(i) < Zero) m += Sqr(d(i));
  return m;
}


ConvergenceResult ConstrainedMinimizationProblem::StepTR(Real R)
{
  grad.resize(x.n);
  fx=(*f)(x);
  f->Gradient(x,grad);

  int cn = C->NumDimensions();
  int dn = D->NumDimensions();
  Vector cx(cn),dx(dn);
  (*C)(x,cx);
  (*D)(x,dx);
  LOG4CXX_INFO(KrisLibrary::logger(),"******************************************************");
  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): objective "<<fx<<", initial distance "<<cx.maxAbsElement()<<", margin "<<dx.minElement());

  //solving the linearized version of the problem for a displacement
  //y such that C(x+y) ~= C(x)+J(x)*y = 0
  //etc
  if(!sparse) {
    LinearProgram lp;
    lp.minimize=true;
    lp.Resize(cn+dn,x.n);  //reserve enough space in A
    Assert(lp.c.n == grad.n);
    lp.c = grad;
    Matrix Ac,Ad;
    Vector bc,bd;
    Ac.setRef(lp.A,0,0,1,1,cn,x.n);
    Ad.setRef(lp.A,cn,0,1,1,dn,x.n);
    C->Jacobian(x,Ac);
    D->Jacobian(x,Ad);
    bc.setRef(lp.p,0,1,cn);
    bc.setNegative(cx);
    bc.setRef(lp.q,0,1,cn);
    bc.setNegative(cx);
    bd.setRef(lp.q,cn,1,dn);
    bd.setNegative(dx);
    lp.l.set(-R);
    lp.u.set(R);
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(lp.l(i)+x(i) < bmin(i)) lp.l(i) = bmin(i)-x(i);
	if(lp.u(i)+x(i) > bmax(i)) lp.u(i) = bmax(i)-x(i);
      }
    }

    RobustLPSolver lps;
    LinearProgram::Result res=lps.Solve(lp);

    if(res == LinearProgram::Infeasible) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): trust region radius "<<R<<" too small, doing a feasibility solve step");
      //NOTE: if MaxItersReached is returned, this will return ConvergenceError
      ConvergenceResult r;
      SolveFeasiblePoint(x,1,&r);
      if(r == MaxItersReached || r == ConvergenceF) 
	//if(!SolveFeasiblePoint(x,1)) return ConvergenceError;
	return LocalMinimum;
      else if(r == ConvergenceX)
	return ConvergenceX;
      else
	return ConvergenceError;
    }
    else if(res == LinearProgram::Unbounded) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): uh... can't be unbounded!");
      Abort();
    }
    else if(res == LinearProgram::Feasible) {
      //arbitrary merit function...
      Real fScale=tolf/tolc;
      Real origMerit = Merit(fx,cx,dx,fScale);
      
      Real alpha = One;
      Real stepNorm = lps.xopt.norm();
      Vector xold=x;
      int numSteps=0;
      while(alpha > 1e-3) {
	x=xold; x.madd(lps.xopt,alpha);
	(*C)(x,cx);
	(*D)(x,dx);
	fx = (*f)(x);
	Real newMerit = Merit(fx,cx,dx,fScale);
	LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): step length "<<stepNorm*alpha);
	LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): objective "<<fx<<", distance "<<cx.maxAbsElement()<<", margin "<<dx.minElement());
      
	if(newMerit < origMerit) {
	  if(numSteps == 0) {
	    if(FuzzyEquals(newMerit,origMerit,tolf)) return ConvergenceF;
	    if(stepNorm < tolx) return ConvergenceX;
	    return MaxItersReached;
	  }
	  else {
	    LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): trust region radius "<<R<<" too large");
	    LOG4CXX_INFO(KrisLibrary::logger(),"Suggest radius "<<R*alpha);
	    return Divergence;
	  }
	}
	numSteps++;
	alpha *= 0.5;
      }
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): trust region radius "<<R<<" too large");
      x = xold;
      return Divergence;
    }
    else { //error
      LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): error solving linear program!");
      return ConvergenceError;
    }
    return MaxItersReached;
  }
  else {  //identical, but sparse
    LinearProgram_Sparse lp;
    lp.minimize=true;
    lp.Resize(cn+dn,x.n);  //reserve enough space in A
    Assert(lp.c.n == grad.n);
    lp.c = grad;
    SparseMatrix Ac(cn,x.n),Ad(dn,x.n);
    Vector bc,bd;
    SparseVectorFunction* sC,*sD;
    //Note: dangerous cast here!
    try {
      sC=dynamic_cast<SparseVectorFunction*>(C);
      sD=dynamic_cast<SparseVectorFunction*>(D);
    }
    catch(exception& e) {
      FatalError("Could not cast C or D to sparse functions! exception: %s",e.what());
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"Evaluating equality jacobian...");     sC->Jacobian_Sparse(x,Ac);
    LOG4CXX_INFO(KrisLibrary::logger(),Ac.numNonZeros()<<" nonzeros");
    LOG4CXX_INFO(KrisLibrary::logger(),"Evaluating inequality jacobian...");     sD->Jacobian_Sparse(x,Ad);
    LOG4CXX_INFO(KrisLibrary::logger(),Ad.numNonZeros()<<" nonzeros");
    lp.A.copySubMatrix(0,0,Ac);
    lp.A.copySubMatrix(cn,0,Ad);
    bc.setRef(lp.p,0,1,cn);
    bc.setNegative(cx);
    bc.setRef(lp.q,0,1,cn);
    bc.setNegative(cx);
    bd.setRef(lp.q,cn,1,dn);
    bd.setNegative(dx);
    lp.l.set(-R);
    lp.u.set(R);
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(lp.l(i)+x(i) < bmin(i)) lp.l(i) = bmin(i)-x(i);
	if(lp.u(i)+x(i) > bmax(i)) lp.u(i) = bmax(i)-x(i);
      }
    }
    RobustLPSolver lps;
    LinearProgram::Result res=lps.Solve(lp);

    if(res == LinearProgram::Infeasible) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Linear program is infeasible!");

      /*
      lp.l.set(-Inf);
      lp.u.set(Inf);
      LinearProgram::Result res2=lps.Solve(lp);
      if(res2 == LinearProgram::Infeasible) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Seems to be infeasible over entire space!");
	KrisLibrary::loggerWait();

	//deactivate inequality constraints
	for(int i=0;i<dn;i++) {
	  lp.q(cn+i)=-Inf;
	  lp.p(cn+i)=Inf;
	}
	res2 = lps.Solve(lp);
	if(res2 == LinearProgram::Infeasible) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"equalities are infeasible!");
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Equalities: "<<cn<<", variables "<<x.n);
	  //lp.Print(cout);
	  KrisLibrary::loggerWait();

	  LOG4CXX_INFO(KrisLibrary::logger(),"Trying LSQR of equalities...");
	  LSQRInterface lsqr;
	  lsqr.verbose = 0;
	  if(lsqr.Solve(Ac,cx)) {
	    lsqr.x.inplaceNegative();
	    LOG4CXX_INFO(KrisLibrary::logger(),"LSQR solved the problem with residual "<<lsqr.residualNorm);
	  }
	  else
	    LOG4CXX_INFO(KrisLibrary::logger(),"LSQR failed with residual "<<lsqr.residualNorm);
	  return ConvergenceError;
	}
	else {
	}
      }
      */
      /*
      lp.A.eraseZeros();
      LinearProgram_Sparse lptemp;
      lptemp = lp;
      lptemp.A.clear();
      lptemp.p.clear();
      lptemp.q.clear();
      for(int i=1;i<lp.A.m;i++) {
	lptemp.A.resize(i,lp.A.n);
	lptemp.p.resize(i);
	lptemp.q.resize(i);
	for(int j=0;j<i;j++) {
	  lptemp.A.rows[j] = lp.A.rows[j];
	  lptemp.p(j) = lp.p(j);
	  lptemp.q(j) = lp.q(j);
	}
	if(lps.Solve(lptemp) == LinearProgram::Infeasible) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Constraint "<<i<<" caused the LP to become infeasible!");
	  SparseVector tmp;
	  tmp.set(lptemp.A.rows[i-1]);
	  LOG4CXX_INFO(KrisLibrary::logger()(),lptemp.q(i-1)<<" < "); tmp.print(	  if(KrisLibrary::logger()()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
	  break;
	}
      }
      */

      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): trust region radius "<<R<<" too small, doing a feasibility solve step");
      rootSolver.tolf = tolc;
      rootSolver.tolmin = tolx;
      rootSolver.tolx = tolx;
      rootSolver.bmin.setRef(bmin);
      rootSolver.bmax.setRef(bmax);
      rootSolver.verbose = verbose;
      rootSolver.x.setRef(x);
      rootSolver.sparse = sparse;
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveFeasiblePoint...");
      int iters=1;
      ConvergenceResult r=rootSolver.SolveConstrained_SLP(iters);
      if(r == ConvergenceError) {
	LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): feasibility could not be solved!");
	return ConvergenceError;
      }
      else if(r == ConvergenceX) {
	LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): feasibility could not be solved, x tolerance has been reached");
	return ConvergenceX;
      }

      /*
      if(!SolveFeasiblePoint(x,1)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): feasibility could not be solved!");
	return ConvergenceError;
      }
      */
      (*C)(x,cx);
      (*D)(x,dx);
      fx = (*f)(x);
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): final objective "<<fx<<", distance "<<cx.maxAbsElement()<<", margin "<<dx.minElement());
      /*
      //shortcut to SolveFeasiblePoint() -- just solve the least squares problem
      for(int i=0;i<dn;i++)
	if(dx(i) > Zero) { //remove from lp.A, lp.b
	  lp.A.rows[i+cn].entries.clear();
	  lp.q(i+cn)=Zero;
	}
      bool res=rootSolver.SolveUnderconstrainedLS(lp.A,lp.q,lps.xopt);
      if(res) {
	Real merit = Merit(Zero,cx,dx);
	Real alpha = One;
	Vector x0=x;
	while(alpha > 1e-4) {
	  x = x0; x.madd(lps.xopt,alpha);
	  (*C)(x,cx);
	  (*D)(x,dx);
	  fx = (*f)(x);
	  Real newMerit = Merit(Zero,cx,dx);
	  if(newMerit < merit-1e-4) //sufficient decrease
	    break;
	  alpha *= 0.5;
	}
	if(alpha <= 1e-4) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): backtracking failed!");
	  x=x0;
	}
	else {
	  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): final objective "<<fx<<", distance "<<cx.maxAbsElement()<<", margin "<<dx.minElement());
	}
      }
      */
      return LocalMinimum;
    }
    else if(res == LinearProgram::Unbounded) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): uh... can't be unbounded!");
      Abort();
    }
    else if(res == LinearProgram::Feasible) {
      //arbitrary merit function...
      Real fScale=tolf/tolc;
      Real origMerit = Merit(fx,cx,dx,fScale);
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR():");
      LOG4CXX_INFO(KrisLibrary::logger(),"   Original merit "<<origMerit);

      Real alpha = One;
      Real stepNorm = lps.xopt.norm();
      Vector xold=x;
      int numSteps=0;
      while(alpha > 1e-3) {
	x=xold; x.madd(lps.xopt,alpha);
	(*C)(x,cx);
	(*D)(x,dx);
	fx = (*f)(x);
	Real newMerit = Merit(fx,cx,dx,fScale);
	LOG4CXX_INFO(KrisLibrary::logger(),"   Step length "<<stepNorm*alpha<<", merit "<<newMerit);
	LOG4CXX_INFO(KrisLibrary::logger(),"   Objective "<<fx<<", distance "<<cx.maxAbsElement()<<", margin "<<dx.minElement());
      
	if(newMerit < origMerit) {
	  if(numSteps == 0) {
	    if(FuzzyEquals(newMerit,origMerit,tolf)) return ConvergenceF;
	    if(stepNorm < tolx) return ConvergenceX;
	    return MaxItersReached;
	  }
	  else {
	    LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): trust region radius "<<R<<" too large");
	    LOG4CXX_INFO(KrisLibrary::logger(),"Suggest radius "<<R*alpha);
	    return Divergence;
	  }
	}
	numSteps++;
	alpha *= 0.5;
      }
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): trust region radius "<<R<<" too large");
      x = xold;
      return Divergence;
    }
    else { //error
      LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimization::StepTR(): error solving linear program!");
      return ConvergenceError;
    }
    return MaxItersReached;
  }
}

void ConstrainedMinimizationProblem::NullspaceProjection(const Vector& x,Vector& dx)
{
  if(C) {
    //C(x+dx) = C(x)
    //C(x+dx) ~= C(x) + J*dx
    //=> J*dx = 0
    //let p be an offset to dx
    //J*p = -J*dx
    Matrix J;
    int m=C->NumDimensions();
    int n=x.n;
    J.resize(m,n);
    C->PreEval(x);
    C->Jacobian(x,J);
    Vector p,Jdx;
    J.mul(dx,Jdx);
    rootSolver.SolveUnderconstrainedLS(J,Jdx,p);
    dx -= p;
    /*
    RobustSVD<Real> svd;
    Matrix J;
    int m=C->NumDimensions();
    int n=x.n;
    J.resize(m,n);
    C->PreEval(x);
    C->Jacobian(x,J);
    svd.resize(m,n);
    if(!svd.set(J)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimizationProblem::NullspaceProjection: Error setting SVD of equality Jacobian!!!");
      //LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(J));
      //KrisLibrary::loggerWait();
      J.setZero();
      svd.setZero(J.m,J.n);
    }
    Vector proj;
    svd.nullspaceComponent(dx,proj);
    dx -= proj;
    */
  }

  //TODO: active set of D constraints, simultaneous svd solution
  if(bmin.n != 0) {
    for(int i=0;i<bmin.n;i++) {
      Assert(x(i) >= bmin(i));
      Assert(x(i) <= bmax(i));
      if(x(i) == bmin(i)) {
	if(dx(i) < 0) dx(i) = 0;
      }
      else if(x(i) == bmax(i)) {
	if(dx(i) > 0) dx(i) = 0;
      }
    }
  }
}

ConvergenceResult ConstrainedMinimizationProblem::LineMinimizationStep(const Vector& dx,Real& alpha0)
{
  Vector x0 = x;
  Real fx0 = fx;
  const Real lineSearchShrink = 0.5;
  Real dxnorm = dx.norm();
  Real t = alpha0;
  Real slope = Abs(dx.dot(grad));
  if(dxnorm < tolgrad) {
    return ConvergenceF;
  }

  //if(verbose>=1 && t != alpha0) LOG4CXX_INFO(KrisLibrary::logger(),"Starting t value: "<<t);
  //find a step that descends along grad
  for(int lineSearchIters=0;;lineSearchIters++) {
    if(t*dxnorm < tolx) {
      if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimzationProblem::LineSearch(): Converged on t on line search iteration "<<lineSearchIters<<", |dx|="<<dxnorm);
      //x must remain on surface... just set x to x0
      x = x0; alpha0 = 0;
      return ConvergenceX;
    }
    x = x0; x.madd(dx,t);
    for(int i=0;i<x.n;i++)
      if(IsNaN(x(i))) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimizationProblem: x is NaN!");
	LOG4CXX_ERROR(KrisLibrary::logger(),"t is "<<t);
	LOG4CXX_ERROR(KrisLibrary::logger(),"x0 is "<<x0);
	KrisLibrary::loggerWait();
	return ConvergenceError;
      }
    if(SolveFeasiblePoint(x,20)) {
      Assert(CheckPoint(x));
      fx = (*f)(x);
      if(fx < fx0 - ALF*slope*t) {
	break;
      }
    }
    else {
      if(verbose>=1) LOG4CXX_WARN(KrisLibrary::logger(),"ConstrainedMinimzationProblem::StepGD(): Warning, MoveToSurface_Bounded failed on line search iteration "<<lineSearchIters);
    }
    //reduce t
    t *= lineSearchShrink;
  }
  alpha0 = t;

  if(Abs(fx-fx0) <= tolf) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimzationProblem::StepGD(): Success, change in f is "<<fx0-fx);
    return ConvergenceF;
  }

  if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"StepGD(): Target "<<fx<<", length of gradient "<<dxnorm<<", step size "<<t*dxnorm);

  if(bmin.n != 0)
    Assert(AABBContains(x0,bmin,bmax));

  return MaxItersReached;
}

bool ConstrainedMinimizationProblem::SolveFeasiblePoint(Vector& x,int maxIters,ConvergenceResult* res)
{
  rootSolver.tolf = tolc;
  rootSolver.tolmin = tolx;
  rootSolver.tolx = tolx;
  rootSolver.bmin.setRef(bmin);
  rootSolver.bmax.setRef(bmax);
  rootSolver.verbose = verbose;
  rootSolver.x.setRef(x);
  rootSolver.sparse = sparse;
  LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveFeasiblePoint...");
  return rootSolver.GlobalSolve(maxIters,res);
}



#endif //commented out ConstrainedMinimizationProblem
