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

ConvergenceResult MinimizationProblem::SolveLM(int& iters,Real lambda0,Real lambdaGrow,Real lambdaShrink)
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
      lambda /= lambdaShrink;
    }
    else {
      tookStep=false;
      x = x0;
      fx = fx0;
      H.copyDiag(0,diagH); 
      lambda *= lambdaGrow;
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
    dx.inplaceNegative();

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
  if(!AABBContains(x,bmin,bmax)) 
    LOG4CXX_ERROR(KrisLibrary::logger(),"BCMinimizationProblem::LineMinimizationStep(): Initial state not in bounds?")
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
        if(dx(i) > 0) {
          //printf("Freeing x[%d], dx=%f > 0\n",i,dx(i));
          activeSet[i] = true;
        }
      }
      else {
        Assert(FuzzyEquals(x(i),bmax(i),boundEpsilon));
        if(dx(i) < 0) {
          //printf("Freeing x[%d], dx=%f < 0\n",i,dx(i));
          activeSet[i] = true;
        }
      }
    }
    if(!activeSet[i])
      dx(i) = 0;
  }
  //termination criteria
  Real normdx = Norm_LInf(dx);
  if(normdx <= tolgrad) {
    //printf("Converged on f: |dx|=%f\n",normdx);
    return ConvergenceF;
  }
  
  //start line search
  Real fx0=fx;
  Vector x0 = x;
  Real t=alpha0;
  AABBLineSearch(x,dx,bmin,bmax,t);
  //printf("Fx0 = %f\n",fx0);

  if(normdx*Abs(t) > 1e8)
    return Divergence;
  for(int lineSearchIters=0;;lineSearchIters++) {
    if(normdx*Abs(t) <= tolx)
      return ConvergenceX;
    x = x0;
    x.madd(dx,t);
    fx = (*f)(x);
    //printf("Exploring line search alpha %f, step size %f, fx %f\n",t,normdx*t,fx);
    if(fx < fx0) break; //accept step
    t *= 0.5;
  }
  //decide how to modify alpha
  if(alpha0 == t) alpha0 = t*2.5;
  else alpha0 = t;

  if(S) S->push_back(x);
  if(Abs(fx0-fx) < tolf)
      return ConvergenceF;

  return MaxItersReached;
}




ConstrainedMinimizationProblem::ConstrainedMinimizationProblem(ScalarFieldFunction* _f,VectorFieldFunction*_c,VectorFieldFunction*_d)
  :f(_f),C(_c),D(_d),
   tolx(1e-7),tolf(1e-5),tolgrad(1e-8),tolc(1e-6),
   innerIters(2),augmentedLagrangianProblem(_f),
   verbose(1),S(NULL)
{
}


ConvergenceResult ConstrainedMinimizationProblem::SolveAugmentedLangrangian(int& iters)
{
  augmentedLagrangianProblem.bmin = bmin;
  augmentedLagrangianProblem.bmax = bmax;
  augmentedLagrangianProblem.x = x;
  augmentedLagrangianProblem.tolf = tolf;
  augmentedLagrangianProblem.tolx = tolx;
  augmentedLagrangianProblem.tolgrad = tolgrad;
  augmentedLagrangianProblem.verbose = verbose;
  augmentedLagrangianProblem.H.resize(x.n,x.n);
  augmentedLagrangianProblem.H.setIdentity();

  if(f) fx = (*f)(x);
  else fx = 0;
  if(C) {
    cx.resize(C->NumDimensions());
    (*C)(x,cx);
  }
  if(D) {
    dx.resize(D->NumDimensions());
    (*D)(x,dx);
  }
  

  Vector lambda_c,lambda_d;
  Real mu = 100.0;
  if(C) lambda_c.resize(C->NumDimensions(),0.0);
  if(D) lambda_d.resize(D->NumDimensions(),0.0);
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    if(verbose>=1) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveAugmentedLagrangian(): Iter "<<iters<<" fx "<<fx<<" cx "<<(cx.n ? cx.maxAbsElement() : 0)<<" dx "<<(dx.n ? dx.maxAbsElement() : 0));
      LOG4CXX_INFO(KrisLibrary::logger(),"    mu "<<mu<<" lambdas "<<lambda_c<<", "<<lambda_d);
    }
    ConvergenceResult res = StepAugmentedLangrangian(mu,lambda_c,lambda_d);
    if(res != MaxItersReached) {
      if(cx.n == 0 || cx.maxAbsElement() < tolc) {
        if(dx.n == 0 || dx.maxElement() < tolc) {
          if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveAugmentedLagrangian(): Converged with feasible solution");
          return res;
        }
      }
    }
    if(S) S->push_back(x);

    //modify augmented langrangian multipliers
    for(int i=0;i<lambda_c.n;i++)
      lambda_c[i] += mu*cx[i];
    for(int i=0;i<lambda_d.n;i++) {
      if(dx[i] < 0 && lambda_d[i] > 0) lambda_d[i] = 0;
      else lambda_d[i] += mu*dx[i];
    }
    mu *= 2.0;
    mu = Min(mu,1000000.0);
  }
  if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveAugmentedLagrangian(): Max iters reached "<<iters<<".");
  return MaxItersReached;
}

class AugmentedLagrangianFunction : public ScalarFieldFunction
{
  public:
  AugmentedLagrangianFunction(ScalarFieldFunction* _f,VectorFieldFunction* _C,VectorFieldFunction* _D,
    Real _mu,const Vector& _lambda_c,const Vector& _lambda_d)
    :f(_f),C(_C),D(_D),mu(_mu),lambda_c(_lambda_c),lambda_d(_lambda_d)
  {}
  virtual void PreEval(const Vector& x) override {
    if(f) f->PreEval(x);
    if(C) C->PreEval(x);
    if(D) D->PreEval(x);
    if(f) fx = f->Eval(x);
    else fx = 0.0;
    if(C) {
      cx.resize(C->NumDimensions());
      C->Eval(x,cx);
    }
    if(D) {
      dx.resize(D->NumDimensions());
      D->Eval(x,dx);
    }
  }
  virtual Real Eval(const Vector& x) override {
    Assert(cx.n == lambda_c.n);
    Assert(dx.n == lambda_d.n);
    Real dx2=0,clam=0,dlam=0;
    for(int i=0;i<dx.n;i++) if(dx[i] > 0) dx2 += Sqr(dx[i]);
    for(int i=0;i<cx.n;i++) clam += lambda_c[i]*cx[i];
    for(int i=0;i<dx.n;i++) dlam += lambda_d[i]*dx[i];
    return fx + mu*(cx.normSquared() + dx2) + clam + dlam;
  }
  virtual void Gradient(const Vector& x,Vector& g) override
  {
    g.resize(x.n);
    Matrix dC,dD;
    if(f) {
      f->Gradient(x,g);
    }
    else {
      g.set(0.0);
    }
    if(C) {
      dC.resize(C->NumDimensions(),x.n);
      C->Jacobian(x,dC);
      Vector temp;
      dC.mulTranspose(cx,temp);
      g.madd(temp,2*mu);
      dC.mulTranspose(lambda_c,temp);
      g += temp;
    }
    if(D) {
      dD.resize(D->NumDimensions(),x.n);
      D->Jacobian(x,dD);
      Vector temp;
      Vector dxpos(dx.n,0.0);
      for(int i=0;i<dx.n;i++)
        if(dx[i] > 0) dxpos[i] = dx[i];
      dD.mulTranspose(dx,temp);
      g.madd(temp,2*mu);
      dD.mulTranspose(lambda_d,temp);
      g += temp;
    }
  }

  ScalarFieldFunction* f;
  VectorFieldFunction *C,*D;
  Real mu;
  Vector lambda_c,lambda_d;

  Real fx;
  Vector cx,dx;
};


ConvergenceResult ConstrainedMinimizationProblem::StepAugmentedLangrangian(Real mu,const Vector& lambda_c,const Vector& lambda_d)
{
  AugmentedLagrangianFunction faug(f,C,D,mu,lambda_c,lambda_d);
  augmentedLagrangianProblem.f = &faug;
  int tempIters = innerIters;
  ConvergenceResult res = augmentedLagrangianProblem.SolveQuasiNewton(tempIters);
  cout<<"SolveQuasiNewton got "<<res<<" in "<<tempIters<<" iterations"<<endl;
  fx = faug.fx;
  cx = faug.cx;
  dx = faug.dx;
  x = augmentedLagrangianProblem.x;
  return res;
}

ConvergenceResult ConstrainedMinimizationProblem::SolveSQP(int& iters)
{
  if(f) fx = (*f)(x);
  else fx = 0;
  if(C) {
    cx.resize(C->NumDimensions());
    (*C)(x,cx);
  }
  if(D) {
    dx.resize(D->NumDimensions());
    (*D)(x,dx);
  }
  
  Real alpha = 1.0;
  int maxIters=iters;
  for(iters=0;iters<maxIters;iters++) {
    if(verbose>=1) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveSQP(): Iter "<<iters<<" fx "<<fx<<" cx "<<(cx.n ? cx.maxAbsElement() : 0)<<" dx "<<(dx.n ? dx.maxAbsElement() : 0));
    }
    ConvergenceResult res = StepSQP(alpha);
    if(res != MaxItersReached) {
      return res;
    }
    if(S) S->push_back(x);
  }
  if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::SolveSQP(): Max iters reached "<<iters<<".");
  return MaxItersReached;
}

ConvergenceResult ConstrainedMinimizationProblem::StepSQP(Real &alpha)
{
  //given x, f(x+dx) ~= f(x) + f'(x)dx + 1/2 dx^T f''(x) dx
  //C(x+dx) ~= C(x) + C'(x) dx = 0
  //To minimize f(x+dx) s.t. C(x+dx) = 0, solve lagrange multipliers
  //f' + f''*dx + C'^T lambda = 0
  //C + C'*dx = 0
  //[f'' C'^T ] [dx    ] = [-f']
  //[C'  0    ] [lambda]   [-C ]
  //dx = -f''^-1*f' + f''^-1 C'^T lambda
  //-C' f''^-1*f' + C' f''^-1 C'^T lambda = -C
  //lambda = (C' f''^-1 C'^T)^-1 (C' f''^-1*f' - C)
  //dx = -f''^-1*f' + f''^-1 C'^T (C' f''^-1 C'^T)^-1 (C' f''^-1*f' - C)
  if(D) FatalError("TODO: SQP with inequalities");
  if(!f) FatalError("TODO: SQP without objective");
  Vector gf(x.n);
  Matrix Hf,JC;
  Matrix lagrangian(x.n+cx.n,x.n+cx.n,0.0);
  if(f) {
    f->Gradient(x,gf);
    Hf.setRef(lagrangian,0,0,1,1,x.n,x.n);
    f->Hessian(x,Hf);
  }
  else {
    gf.setZero();
  }
  if(C) {
    JC.setRef(lagrangian,x.n,0,1,1,cx.n,x.n);
    C->Jacobian(x,JC);
    Matrix JCT;
    JCT.setRefTranspose(JC);
    lagrangian.copySubMatrix(0,x.n,JCT);
  }
  Vector fc(x.n+cx.n);
  fc.copySubVector(0,gf);
  fc.copySubVector(x.n,cx);
  fc.inplaceNegative();
  Vector dxlambda,deltax;
  LDLDecomposition<Real> ldl(lagrangian);
  Vector ldlD;
  ldl.LDL.getDiagRef(0,ldlD);
  if(ldlD.minAbsElement() <= ldl.zeroTolerance) { //not stable, need pseudoinverse
    SVDecomposition<Real> svd(lagrangian);
    svd.backSub(fc,dxlambda);
  }
  else {
    ldl.backSub(fc,dxlambda);
  }
  deltax.setRef(dxlambda,0,1,x.n);
  if(!AABBContains(x,bmin,bmax)) 
    LOG4CXX_ERROR(KrisLibrary::logger(),"ConstrainedMinimizationProblem::StepSQP: Initial state not in bounds?")
  for(int i=0;i<x.n;i++) {
    if(FuzzyEquals(x[i],bmin[i],boundEpsilon)) {
      if(deltax[i] < 0) deltax[i] = 0;
    }
    else if(FuzzyEquals(x[i],bmax[i],boundEpsilon)) {
      if(deltax[i] > 0) deltax[i] = 0;
    }
  }

  //now do line search on merit function f(x) + mu||c(x)||_1
  Real normdx = deltax.norm();
  if(normdx < tolgrad) 
    return ConvergenceF;
  Real muscale = 2.0;
  Real cnorm = Norm_L1(cx);
  //f(x+dx) + mu||c(x+dx)|| < f + mu||c|| 
  //f'*dx + mu (||c(x+dx)|| - ||c(x)||_1) < 0
  //mu > f'*dx / (||c(x)|| - ||c(x+dx)||)  but ||c(x+dx)|| = 0
  Real mu;
  if(cnorm!=0) 
    mu = muscale * Max(deltax.dot(gf) / cnorm,1.0);
  else
    mu = 1.0;
  Real merit0 = fx + mu*cnorm;
  if(verbose >= 2) {
    printf("Expected full step decrease in f = %f, in c = %f\n",deltax.dot(gf), -cnorm);
    printf("Mu %f, ",mu);
    printf("%.3f ",merit0);
  }
  Vector x0=x;
  AABBLineSearch(x,deltax,bmin,bmax,alpha);
  for(int lineSearchIters=1;;lineSearchIters++) {
    if(alpha * normdx < tolx) {
      if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"ConstrainedMinimizationProblem::LineMinimizationStep(): Quitting on line search iter "<<lineSearchIters);
      if(verbose >= 2) printf("\n");
      return ConvergenceF;
    }

    x = x0;
    x.madd(deltax,alpha);
    fx = (*f)(x);
    if(C) (*C)(x,cx);
    if(D) (*D)(x,dx);
    Real merit = fx + mu*Norm_L1(cx);
    if(verbose >= 2) printf("%.3f ",merit);
    if(merit < merit0) {
      if(verbose >= 2) printf("\n");
      if(lineSearchIters==1) alpha *= 2.5;
      return MaxItersReached;
    }
    if(!IsFinite(merit)) {
      x = x0;
      return ConvergenceError;
    }
    alpha *= 0.5;
  } 
}