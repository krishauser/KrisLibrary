#include "NonlinearProgram.h"
#include <errors.h>
using namespace Optimization;


NonlinearProgram::NonlinearProgram(const std::shared_ptr<ScalarFieldFunction>& _f,const std::shared_ptr<VectorFieldFunction>& _c,const std::shared_ptr<VectorFieldFunction>& _d)
  :f(_f),c(_c),d(_d),minimize(true),inequalityLess(true)
{
}

void NonlinearProgram::PreEval(const Vector& x) 
{
  if(f) f->PreEval(x);
  if(c) c->PreEval(x);
  if(d) d->PreEval(x);
}


bool NonlinearProgram::SatisfiesEquality(const Vector& x,Real tol) 
{
  Vector temp(c->NumDimensions());
  c->Eval(x,temp);
  for(int i=0;i<temp.n;i++) if(!FuzzyZero(temp(i),tol)) return false;
  return true;
}

bool NonlinearProgram::SatisfiesInequality(const Vector& x) 
{
  Vector temp(d->NumDimensions());
  d->Eval(x,temp);
  if(inequalityLess) {
    for(int i=0;i<temp.n;i++) if(temp(i) > Zero) return false;
  }
  else {
    for(int i=0;i<temp.n;i++) if(temp(i) < Zero) return false;
  }
  return true;
}

Real NonlinearProgram::Lagrangian(const Vector& x,const Vector& lambda,const Vector& mu) 
{
  PreEval(x);
  return LagrangianEval(x,lambda,mu);
}

Real NonlinearProgram::LagrangianEval(const Vector& x,const Vector& lambda,const Vector& mu) 
{
  Real v;
  Vector tmp;
  if(f) {
    v = f->Eval(x);
    if(!minimize) v = -v;
  }
  else v=Zero;
  if(c) {
    tmp.resize(c->NumDimensions());
    c->Eval(x,tmp);
    v += dot(lambda,tmp);
  }
  if(d) {
    tmp.resize(d->NumDimensions());
    d->Eval(x,tmp);
    if(inequalityLess)  v += dot(mu,tmp);
    else  v -= dot(mu,tmp);
  }
  return v;
}

void NonlinearProgram::LagrangianGradient(const Vector& x,const Vector& lambda,const Vector& mu, Vector& grad) 
{
  grad.resize(x.n);
  if(f) {
    f->Gradient(x,grad);
    if(!minimize) grad.inplaceNegative();
  }
  else grad.setZero();
  Matrix J;
  if(c) {
    J.resize(c->NumDimensions(),x.n);
    c->Jacobian(x,J);
    J.maddTranspose(lambda,grad);
  }
  if(d) {
    J.resize(d->NumDimensions(),x.n);
    d->Jacobian(x,J);
    if(inequalityLess) 
      J.maddTranspose(mu,grad);
    else {
      Vector temp;
      J.mulTranspose(mu,temp);
      grad -= temp;
    }
  }
}

void NonlinearProgram::LagrangianHessian(const Vector& x,const Vector& lambda,const Vector& mu, Matrix& H) 
{
  LagrangianHessian_Sparse(x,lambda,mu,H);
}



Real NonlinearProgram::LagrangianEval_Sparse(const Vector& x,const Vector& lambda,const Vector& mu) 
{
  Real v;
  if(f) {
    v = f->Eval(x);
    if(!minimize) v = -v;
  }
  else v=Zero;
  if(c) {
    Assert(lambda.n == c->NumDimensions());
    Vector tmp(c->NumDimensions());
    c->Eval(x,tmp);
    v += dot(lambda,tmp);
  }
  if(d) {  //each component done separately because mu(i) may be zero
    Assert(mu.n == d->NumDimensions());
    for(int i=0;i<mu.n;i++) {
      if(mu(i) != Zero) {
	if(inequalityLess) v += d->Eval_i(x,i)*mu(i);
	else v -= d->Eval_i(x,i)*mu(i);
      }
    }
  }
  return v;
}

void NonlinearProgram::LagrangianGradient_Sparse(const Vector& x,const Vector& lambda,const Vector& mu, Vector& grad) 
{
  grad.resize(x.n);
  if(f) {
    f->Gradient(x,grad);
    if(!minimize) grad.inplaceNegative();
  }
  else grad.setZero();
  if(c) {
    Matrix J;
    J.resize(c->NumDimensions(),x.n);
    c->Jacobian(x,J);
    J.maddTranspose(lambda,grad);
  }
  if(d) {
    Assert(mu.n == d->NumDimensions());
    Vector tmp(x.n);
    for(int i=0;i<mu.n;i++) {
      if(mu(i) != Zero) {
	d->Jacobian_i(x,i,tmp);
	if(inequalityLess) grad.madd(tmp,mu(i));
	else grad.madd(tmp,-mu(i));
      }
    }
  }
}

void NonlinearProgram::LagrangianHessian_Sparse(const Vector& x,const Vector& lambda,const Vector& mu, Matrix& H) 
{
  H.resize(x.n,x.n);
  if(f) {
    f->Hessian(x,H);
    if(!minimize) H.inplaceNegative();
  }
  else H.setZero();
  Matrix temp(x.n,x.n);
  if(c) {
    Assert(lambda.n == c->NumDimensions());
    for(int i=0;i<lambda.n;i++) {
      c->Hessian_i(x,i,temp);
      H.madd(temp,lambda(i)); 
    }
  }
  if(d) {
    Assert(mu.n == d->NumDimensions());
    for(int i=0;i<mu.n;i++) {
      if(mu(i) != Zero) {
	d->Hessian_i(x,i,temp);
	if(inequalityLess) H.madd(temp,mu(i));
	else H.madd(temp,-mu(i));
      }
    }
  }
}
