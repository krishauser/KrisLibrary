#include <KrisLibrary/Logger.h>
#include "NewtonSolver.h"
#include <math/linalgebra.h>
#include <math/LDL.h>
#include <math/MatrixEquationPrinter.h>
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
#include <math/SelfTest.h>
#include <iostream>
#include <errors.h>
using namespace Optimization;
using namespace std; 

namespace Optimization {

//subtracts the logarithmic barrier function alpha*log(di(x)) 
//from the objective function f
struct LogarithmicBarrierFunction : public ScalarFieldFunction
{
  LogarithmicBarrierFunction(ScalarFieldFunction* _f,VectorFieldFunction* _D,Real _alpha)
    :f(_f),D(_D),alpha(_alpha)
  {}
  virtual void PreEval(const Vector& x)
  {
    f->PreEval(x);
    if(D) {
      D->PreEval(x);
      Dx.resize(D->NumDimensions());
      D->Eval(x,Dx);
    }
  }
  virtual Real Eval(const Vector& x)
  {
    Real fx=f->Eval(x);
    Real barrier=0;
    if(alpha == 0) return fx;
    for(int i=0;i<Dx.n;i++) {
      if(Dx(i) <= 0) return Inf;
      barrier += Log(Dx(i));
    }
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(x(i) <= bmin(i) || x(i) >= bmax(i)) return Inf;
	barrier += Log(x(i)-bmin(i));
	barrier += Log(bmax(i)-x(i));
      }
    }
    return fx-alpha*barrier;
  }
  virtual void Gradient(const Vector& x,Vector& grad)
  {
    f->Gradient(x,grad);
    if(alpha == 0) return;
    temp.resize(grad.n);
    for(int i=0;i<Dx.n;i++) {
      if(Dx(i) <= 0) {
	grad.set(Inf);
	return;
      }
      D->Jacobian_i(x,i,temp);
      grad.madd(temp,-alpha/Dx(i));
    }
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(x(i) <= bmin(i) || x(i) >= bmax(i)) {
	  grad.set(Inf);
	  return;
	}
	grad(i) -= alpha/(x(i)-bmin(i)) - alpha/(bmax(i)-x(i));
      }
    }
  }
  virtual void Hessian(const Vector& x,Matrix& H)
  {
    f->Hessian(x,H);
    mtemp.resize(x.n,x.n);
    temp.resize(x.n);
    for(int i=0;i<Dx.n;i++) {
      if(Dx(i) <= 0 && alpha!=0) {
	H.set(Inf);
	return;
      }
      D->Jacobian_i(x,i,temp);
      D->Hessian_i(x,i,mtemp);
      H.madd(mtemp,-alpha/Dx(i));
      //add outer product/Dx(i)^2
      Real scale = One/Dx(i);
      for(int j=0;j<x.n;j++) {
	for(int k=0;k<x.n;k++) {
	  H(j,k) += alpha*(temp(j)*scale)*(temp(k)*scale);
	}
      }
    }
    if(bmin.n != 0) {
      for(int i=0;i<x.n;i++) {
	if(x(i) <= bmin(i) || x(i) >= bmax(i)) {
	  H.set(Inf);
	  return;
	}
	H(i,i) -= -alpha/Sqr(x(i)-bmin(i)) + alpha/Sqr(bmax(i)-x(i));
      }
    }
  }
   
  ScalarFieldFunction* f;
  VectorFieldFunction* D;
  Real alpha;
  Vector bmin,bmax;

  //temp
  Vector Dx,temp;
  Matrix mtemp;
};

//A nonlinear program adaptor that removes inequality constraints,
//but adds the logarithmic barrier function alpha*log(di(x)) 
//to the objective function
struct InequalityBarrierNLP : public NonlinearProgram
{
  InequalityBarrierNLP(NonlinearProgram& nlp,Real alpha=One)
    :NonlinearProgram(NULL,nlp.c),f_alpha(new LogarithmicBarrierFunction(nlp.f.get(),nlp.d.get(),alpha))
  {
    this->f = f_alpha;
    Assert(nlp.minimize==true);
    Assert(nlp.inequalityLess==false);
  }

  void SetBarrierScale(Real alpha) { f_alpha->alpha=alpha; }

  std::shared_ptr<LogarithmicBarrierFunction> f_alpha;
};

} //namespace Optimization


NewtonSolver::NewtonSolver(NonlinearProgram& nlp)
  :origProblem(nlp),barrierProblem(NULL),problem(NULL),
   tolf(1e-6),tolx(1e-9),
   verbose(1),S(NULL)
{}

NewtonSolver::~NewtonSolver()
{
  SafeDelete(barrierProblem);
}

void NewtonSolver::Init()
{
  SafeDelete(barrierProblem);

  Assert(origProblem.f != NULL);
  Assert(origProblem.c != NULL);
  if(origProblem.d != NULL) {
    Assert(origProblem.inequalityLess==false);
    vtemp.resize(origProblem.d->NumDimensions());
    origProblem.d->PreEval(x);
    origProblem.d->Eval(x,vtemp);
    if(vtemp.minAbsElement() <= 0) {
      FatalError("Initial point to Newton solver not strictly feasible");
      //TODO: infeasible interior point solution
    }
    barrierProblem = new InequalityBarrierNLP(origProblem,100);
    problem = barrierProblem;

    if(!bmin.empty()) {
      Assert(bmin.n == x.n);
      Assert(bmax.n == x.n);
      barrierProblem->f_alpha->bmin.setRef(bmin);
      barrierProblem->f_alpha->bmin.setRef(bmax);
    }

    /*
    if(!TestGradient(barrierProblem->f,x,0.01,1e-3)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Barrier gradient is incorrect");
      KrisLibrary::loggerWait();
    }
    if(!TestHessian(barrierProblem->f,x,0.01,1e-3)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Barrier hessian is incorrect");
      KrisLibrary::loggerWait();
    }
    */
  }
  else if(!bmin.empty()) {
    barrierProblem = new InequalityBarrierNLP(origProblem,100);
    problem = barrierProblem;
    Assert(bmin.n == x.n);
    Assert(bmax.n == x.n);
    barrierProblem->f_alpha->bmin.setRef(bmin);
    barrierProblem->f_alpha->bmin.setRef(bmax);    
  }
  else
    problem = &origProblem;

  Assert(problem->d == NULL);
  Assert(problem->f != NULL);
  Assert(problem->c != NULL);
  a.resize(problem->c->NumDimensions());
  da.resize(a.n);

  //solve for good lagrange multipliers
  //i.e. get as close as possible to grad(f) + dC^t*lambda = 0
  vtemp.resize(x.n);
  A.resize(problem->c->NumDimensions(),x.n);
  problem->f->PreEval(x);
  problem->f->Gradient(x,vtemp);
  problem->c->PreEval(x);
  problem->c->Jacobian(x,A);
  mtemp.setRefTranspose(A);
  
  vtemp.inplaceNegative();
  /*
  MatrixEquation solvelambda(mtemp,vtemp);
  bool res=solvelambda.LeastSquares_SVD(a);
  if(!res) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, couldn't compute the SVD of the constraint Jacobian");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Using zero for the lagrange multipliers...");
    a.setZero();
    KrisLibrary::loggerWait();
  }
  if(verbose >= 2) {
    Vector temp;
    solvelambda.Residual(a,temp);
    LOG4CXX_INFO(KrisLibrary::logger(),"Constraint Jacobian is"<<MatrixPrinter(A)<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Gradient of objective is "<<VectorPrinter(vtemp));
    LOG4CXX_INFO(KrisLibrary::logger(),"Lambda is "<<VectorPrinter(a));
    LOG4CXX_INFO(KrisLibrary::logger(),"Residual with lambda at start: "<<VectorPrinter(temp));
  }
  */
  a.set(One);
  H.resize(x.n,x.n);
  Hinv.resize(x.n,x.n);
  res1.resize(x.n);
  res2.resize(problem->c->NumDimensions());
}

ConvergenceResult NewtonSolver::Solve(int& iters)
{
  LDLDecomposition<Real> ldl,ldl2;
  mtemp.clear();
  mtemp.resize(problem->c->NumDimensions(),x.n);
  Real alphaScale = 0.5;
  int maxIters = iters;
  for(iters=0;iters<maxIters;iters++) {
    problem->PreEval(x);
    //vtemp is ignored
    problem->LagrangianGradient_Sparse(x,a,vtemp,res1);
    problem->LagrangianHessian_Sparse(x,a,vtemp,H);
    problem->c->Eval(x,res2);
    problem->c->Jacobian(x,A);
    res1.inplaceNegative();
    res2.inplaceNegative();
    if(verbose >= 1)
      LOG4CXX_INFO(KrisLibrary::logger(),"Step "<<iters<<", function value "<<problem->f->Eval(x)<<", constraint dist "<<res2.maxAbsElement());
    if(verbose >= 2) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Initial: "<<VectorPrinter(x)<<", "<<VectorPrinter(a));
      MatrixEquationPrinter meq;
      meq.PushMatrix(H);
      meq.PushText("dx");
      meq.PushAdd();
      meq.PushMatrixTranspose(A);
      meq.PushText("da");
      meq.PushEquals();
      meq.PushVector(res1);
      meq.Print(cout);
      meq.Clear();
      meq.PushMatrix(A);
      meq.PushText("dx");
      meq.PushAdd();
      Matrix zero(a.n,a.n); zero.setZero();
      meq.PushMatrix(zero);
      meq.PushText("  ");
      meq.PushEquals();
      meq.PushVector(res2);
      meq.Print(cout);
    }

    //da = (A H^-1 A^t)^-1*(A H^-1 res1 - res2)
    ldl.set(H);
    Vector d;
    ldl.LDL.getDiagRef(0,d);
    if(d.minAbsElement() <= 1e-5) {
      if(verbose >= 1) LOG4CXX_ERROR(KrisLibrary::logger(),"Hessian uninvertible... LDL diagonal gives "<<d.minAbsElement());
      for(int i=0;i<d.n;i++) {
	if(d(i) < 0) d(i) -= 1e-3;
	else d(i) += 1e-3;
      }
    }
    ldl.getInverse(Hinv);

    mtemp.mul(A,Hinv);
    vtemp.resize(mtemp.m);
    mtemp.mul(res1,vtemp);    vtemp -= res2;
    AHA.mulTransposeB(mtemp,A);
    ldl2.set(AHA);
    ldl2.LDL.getDiagRef(0,d);
    if(d.minAbsElement() <= 1e-5) {
      if(verbose >= 1) LOG4CXX_ERROR(KrisLibrary::logger(),"AHA matrix uninvertible... LDL diagonal gives "<<d.minAbsElement());
      for(int i=0;i<d.n;i++) {
	if(d(i) < 0) d(i) -= 1e-3;
	else d(i) += 1e-3;
      }
    }
    ldl2.backSub(vtemp,da);
    
    //dx = H^-1*(res1 - A^t da)
    vtemp.resize(x.n);
    A.mulTranspose(da,vtemp);   vtemp -= res1;  vtemp.inplaceNegative();
    Hinv.mul(vtemp,dx);
    if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"Direction: "<<VectorPrinter(dx)<<", "<<VectorPrinter(da));

    Real alpha = 1;
    ConvergenceResult res=LineSearch(alpha);
    if(res != MaxItersReached) {
      return res;
    }

    if(problem == barrierProblem) {
      barrierProblem->f_alpha->alpha *= alphaScale;
    }
  }
  return MaxItersReached;
}

ConvergenceResult NewtonSolver::LineSearch(Real& alpha)
{
  //res1.resize(x.n);
  //res2.resize(a.n);
  Vector& x0 = res1;
  Vector& a0 = res2;
  x0 = x;
  a0 = a;
  Real fx,fx0 = Merit(x,a);
  if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"Initial merit="<<fx0);
  Real t = alpha;
  Real gnorm = Max(dx.maxAbsElement(),da.maxAbsElement());
  for(int iters=0;;iters++) {
    if(Abs(t) * gnorm < tolx) {
      if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"NewtonSolver::LineSearch(): Quitting on line search iter "<<iters);
      x = x0;
      a = a0;
      alpha = 0;
      return ConvergenceX;
    }

    x = x0; x.madd(dx,t);
    a = a0; a.madd(da,t);

    problem->PreEval(x);
    fx = Merit(x,a);
    if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"Step "<<t<<" merit="<<fx);
    if(fx < fx0-1e-4*gnorm*Abs(t)) break;

    t *= 0.5;
  }
  if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"Decreased merit function from "<<fx0<<" to "<<fx<<" with step "<<t); 
  alpha = t;
  if(S) S->push_back(x);

  //termination criteria
  //if(gnorm <= tolgrad) 
  //  return ConvergenceF;
  if(gnorm*Abs(t) <= tolx)
    return ConvergenceX;
  if(Abs(fx0-fx) < tolf*Abs(t))
    return ConvergenceF;
  if(gnorm*Abs(t) > 1e8)
    return Divergence;
  return MaxItersReached;
}

Real NewtonSolver::Merit(const Vector& x,const Vector& a) const
{
  problem->PreEval(x);

  Vector temp(x.n);
  problem->LagrangianGradient(x,a,vtemp,temp);
  Real merit = temp.normSquared();

  /*
  Real merit = (problem->f?problem->f->Eval(x):Zero);
  if(!problem->minimize) merit = -merit;
  */

  //eval |c(x)|^2
  Real dist2=Zero;
  int j=0;
  if(problem->c) {
    int neq=problem->c->NumDimensions();
    Vector ctemp(neq);
    problem->c->Eval(x,ctemp);
    for(int i=0;i<neq;i++)
      dist2 += Sqr(ctemp(i));
    j += neq;
  }
  Assert(problem->d == NULL);
  merit += dist2;
  return merit;
}
