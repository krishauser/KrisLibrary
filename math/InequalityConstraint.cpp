#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "InequalityConstraint.h"
#include "realfunction.h"
#include "vectorfunction.h"
#include "root.h"
#include "VectorPrinter.h"
#include <errors.h>
#include <iostream>

using namespace Math;
using namespace std;

const static int kLineSearchMaxIters = 50;
const static Real kLineSearchToleranceX = (Real)1e-7;
const static Real kLineSearchToleranceF = (Real)1e-3;
const static int kPushMaxIters = 20;
const static Real kPushXTolerance = (Real)1e-3;
const static Real kPushDTolerance = (Real)1e-3;

int InequalityConstraint::NumDimensions() const
{
  LOG4CXX_INFO(logger,"InequalityConstraint::NumDimensions(): Shouldn't get here"<<"\n");
  AssertNotReached();
  return 1;
}

Real InequalityConstraint::Margin(const Vector& x,int& minConstraint) 
{
  minConstraint=-1;
  Real margin=Inf;
  Real temp;
  int nd=NumDimensions();
  for(int i=0;i<nd;i++) {
    temp = Eval_i(x,i);
    if(temp < margin) {
      margin=temp;
      minConstraint=i;
    }
  }
  return margin;
}

bool InequalityConstraint::Satisfies(const Vector& x,Real d)
{
  int nd=NumDimensions();
  for(int i=0;i<nd;i++)
    if(!Satisfies_i(x,i,d)) return false;
  return true;
}

bool InequalityConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  return (Eval_i(x,i) >= d);
}

void InequalityConstraint::LineSearch(const Vector& x0,const Vector& dx,Real& u)
{
  Vector x;
  int minConstraint=-1;
  Real margin=-Inf;

  PreEval(x0);
  Real margin0 = Margin(x0,minConstraint);
  if(margin0 < 0) {
    LOG4CXX_INFO(logger,"LineSearch: Initial point is not feasible! Margin "<<margin0<<" at "<<Label(minConstraint)<<"\n");
  }
  Assert(margin0 >= Zero);

  for(int i=0;i<1000;i++) {
    x=x0; x.madd(dx,u);
    Real oldMargin = margin;
    int oldMinConstraint = minConstraint;
    PreEval(x);
    margin = Margin(x,minConstraint);
    if(minConstraint == oldMinConstraint) {
      if(margin < oldMargin) {
	LOG4CXX_INFO(logger,"At constraint "<<Label(minConstraint)<<", ");
	LOG4CXX_INFO(logger,"Line search at u gave a lower margin than previous value "<<margin<<" <= "<<oldMargin<<"\n");
	LOG4CXX_INFO(logger,VectorPrinter(x)<<"\n");
	LOG4CXX_INFO(logger,VectorPrinter(dx)<<"\n");
	LOG4CXX_INFO(logger,"u is "<<u<<"\n");
	if(logger->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
      }
      if(margin == oldMargin) {
	LOG4CXX_INFO(logger,"At constraint "<<Label(minConstraint)<<", ");
	LOG4CXX_WARN(logger,"Warning: margin didn't change"<<"\n");
      }
      Assert(margin >= oldMargin);
    }
    else {
      if(margin < oldMargin) {
	LOG4CXX_INFO(logger,"At constraint "<<Label(minConstraint)<<", ");
	LOG4CXX_WARN(logger,"Warning, constraint changed from "<<oldMinConstraint<<" to "<<minConstraint<<" and decreased margin from "<<oldMargin<<" to "<<margin<<"\n");
      }
    }

    if(margin < Zero) {
      LineSearch_i(x0,dx,u,minConstraint);
    }
    else 
      return;
  }
  LOG4CXX_INFO(logger,"LineSearch didn't converge within 1000 iters"<<"\n");
  LOG4CXX_INFO(logger,"Resulting margin is "<<margin<<"\n");
}

void InequalityConstraint::LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i)
{
  VectorFieldProjectionFunction p(this,i);
  ScalarFieldDirectionalFunction g(&p,x0,dx);
  LinearFunction ofs(Zero,-kLineSearchToleranceF*Half-1e-5);
  AddFunction f(&g,&ofs);
  Real umin=Zero;
  Real uroot=u;
  int iters=kLineSearchMaxIters;
  ConvergenceResult res=Root_SecantBracket(f,umin,u,uroot,iters,kLineSearchToleranceX,kLineSearchToleranceF*Half);
  switch(res) {
  case ConvergenceF:
    break;
  case ConvergenceX:
    if(g(uroot) < Zero) {
      if(FuzzyZero(uroot)) { u=0; return; }
      LOG4CXX_INFO(logger,"At constraint "<<Label(i)<<", ");
      LOG4CXX_ERROR(logger,"Error -- secant method didn't return valid u?"<<"\n");
      LOG4CXX_INFO(logger,"u_root = "<<uroot<<", margin "<<g(uroot)<<"\n");
      LOG4CXX_INFO(logger,"u = "<<u<<", margin "<<g(u)<<"\n");
      if(logger->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
      Abort();
    }
    break;
  case MaxItersReached:
    LOG4CXX_WARN(logger,"InequalityConstraint::LineSearch_i(): Warning, max iters reached... should we try some more???"<<"\n");
    break;
  case LocalMinimum:
  case Math::Divergence:
  case ConvergenceError:
    LOG4CXX_ERROR(logger,"Error in secant method: start value is "<<f(umin)<<" end is "<<f(u)<<" for constraint "<<Label(i)<<"\n");
    Abort();
  }
  u=uroot;
}

void InequalityConstraint::LineSearch_j(const Vector& x0,int j,Real& u)
{
  Vector ej(x0.n,Zero);
  ej(j) = One;
  LineSearch(x0,ej,u);
}

bool InequalityConstraint::Push(Vector& x,Real d)
{
  int minConstraint;
  Real margin=-Inf;
  for(int i=0;i<1000;i++) {
    Real oldMargin = margin;
    PreEval(x);
    margin = Margin(x,minConstraint);
    if(margin < oldMargin)
      return false;
    if(margin < d) {
      if(!Push_i(x,minConstraint,d)) return false;
    }
    else {
      Assert(margin+Epsilon >= d);
      return true;
    }
  }
  LOG4CXX_INFO(logger,"Push didn't converge within 1000 iters"<<"\n");
  LOG4CXX_INFO(logger,"Resulting margin is "<<margin<<"\n");
  return false;
}

bool InequalityConstraint::Push_i(Vector& x,int i,Real d)
{
  VectorFieldProjectionFunction p(this,i);
  LinearFunction add_d(One,-d-kPushDTolerance*Half);
  ComposeScalarFieldFunction fd(&add_d,&p);
  int iters=kPushMaxIters;
  ConvergenceResult res=Root_Newton(fd,x,x,iters,kPushXTolerance,kPushDTolerance*Half);
  Real dtrue = p(x);
  if(res == ConvergenceF) {
    if(dtrue < d) {
      LOG4CXX_INFO(logger,"Hmm... Root_Newton worked, but didn't give a valid result!"<<"\n");
      LOG4CXX_INFO(logger,"Desired d="<<d<<", true d="<<dtrue<<"\n");
      LOG4CXX_INFO(logger,"Tolerance setting "<<kPushDTolerance<<"\n");
    }
    Assert(dtrue >= d);
  }
  else if(res == ConvergenceError) {
    return false;
  }
  else {
    if(dtrue > d) {
      LOG4CXX_WARN(logger,"Warning: overshot the desired distance "<<d<<", got "<<dtrue<<"\n");
      return true;
    }
    LOG4CXX_INFO(logger,"At constraint "<<Label(i)<<", ");
    LOG4CXX_INFO(logger,"Unable to solve for f(x) == "<<d<<"\n");
    LOG4CXX_INFO(logger,"Current f(x) == "<<dtrue<<"\n");
    return false;
  }
  return true;
}



std::string CompositeInequalityConstraint::Label() const
{
  std::string str="Composite(";
  for(size_t i=0;i<constraints.size();i++) {
    str+=constraints[i]->Label();
    if(i+1 < constraints.size())
      str+=",";
  }
  str+=")";
  return str;
}

std::string CompositeInequalityConstraint::Label(int i) const
{
  int c=GetConstraint(i);
  return constraints[c]->Label(i);
}

int CompositeInequalityConstraint::NumDimensions() const
{
  int nd=0;
  for(size_t i=0;i<constraints.size();i++) {
    nd+=constraints[i]->NumDimensions();
  }
  return nd;
}

void CompositeInequalityConstraint::PreEval(const Vector& x)
{
  for(size_t i=0;i<constraints.size();i++) {
    constraints[i]->PreEval(x);
  }
}

void CompositeInequalityConstraint::Eval(const Vector& x, Vector& v)
{
  Vector vtemp;
  int nd=0;
  for(size_t i=0;i<constraints.size();i++) {
    int idims=constraints[i]->NumDimensions();
    Assert(nd + idims <= v.n);
    vtemp.setRef(v,nd,1,idims);
    constraints[i]->Eval(x,vtemp);
    Assert(vtemp.n == idims);
    nd += vtemp.n;
  }
  Assert(nd == v.n);
}

Real CompositeInequalityConstraint::Eval_i(const Vector& x,int i)
{
  int c=GetConstraint(i);
  return constraints[c]->Eval_i(x,i);
}

void CompositeInequalityConstraint::Jacobian(const Vector& x,Matrix& J)
{
  J.resize(NumDimensions(),x.n);
  Matrix mtemp;
  int offset=0;
  for(size_t i=0;i<constraints.size();i++) {
    mtemp.setRef(J,offset,0,1,1,constraints[i]->NumDimensions(),x.n);
    constraints[i]->Jacobian(x,mtemp);
    offset += mtemp.m;
  }
}

void CompositeInequalityConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  int c=GetConstraint(i);
  constraints[c]->Jacobian_i(x,i,Ji);
}

void CompositeInequalityConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  int c=GetConstraint(i);
  constraints[c]->Hessian_i(x,i,Hi);
}

void CompositeInequalityConstraint::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  Vector vtemp;
  int nd=0;
  for(size_t i=0;i<constraints.size();i++) {
    vtemp.setRef(v,nd,1,constraints[i]->NumDimensions());
    constraints[i]->DirectionalDeriv(x,h,vtemp);
    nd+=vtemp.n;
  }
  Assert(v.n == nd);
}

Real CompositeInequalityConstraint::Margin(const Vector& x,int& minConstraint)
{
  Real minMargin = Inf, temp;
  minConstraint=-1;
  int itemp;
  int nd=0;
  for(size_t i=0;i<constraints.size();i++) {
    temp = constraints[i]->Margin(x,itemp);
    if(temp < minMargin) {
      minConstraint = itemp+nd;
      minMargin = temp;
    }
    nd += constraints[i]->NumDimensions();
  }
  return minMargin;
}

bool CompositeInequalityConstraint::Satisfies(const Vector& x,Real d)
{
  for(size_t i=0;i<constraints.size();i++)
    if(!constraints[i]->Satisfies(x,d)) return false;
  return true;
}

bool CompositeInequalityConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  int c=GetConstraint(i);
  return constraints[c]->Satisfies_i(x,i,d);
}

void CompositeInequalityConstraint::LineSearch(const Vector& x0,const Vector& dx,Real& u) 
{
  for(size_t i=0;i<constraints.size();i++)
    constraints[i]->LineSearch(x0,dx,u);
}

void CompositeInequalityConstraint::LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i)
{
  int c=GetConstraint(i);
  constraints[c]->LineSearch_i(x0,dx,u,i);
}

bool CompositeInequalityConstraint::Push(Vector& x,Real d)
{
  Real margin=-Inf;
  int temp;
  for(int iters=0;iters<1000;iters++) {
    PreEval(x);

    margin = Inf;
    InequalityConstraint* marginC=NULL;
    int marginK=0;
    int minConstraint=0;
    for(size_t i=0;i<constraints.size();i++) {
      Real iMargin = constraints[i]->Margin(x,temp);
      if(iMargin < margin) {
	margin=iMargin;
	marginC=constraints[i];
	marginK=i;
	minConstraint=temp;
      }
    }
    //Assert(margin == Margin(x,temp));
    /*
    if(margin < oldMargin) {
      LOG4CXX_INFO(logger,"Margin was decreased by last push!"<<"\n");
      LOG4CXX_INFO(logger,"From "<<oldMargin<<" to "<<margin<<"\n");
      return false;
    }
    */
    if(margin < d-Epsilon) {
      if(!marginC->Push_i(x,minConstraint,d)) return false;
      marginC->PreEval(x);
      Real iMargin = marginC->Eval_i(x,minConstraint);
      if(iMargin < d-Epsilon) {
	LOG4CXX_INFO(logger,"Group "<<marginK<<", constraint "<<Label(minConstraint)<<"\n");
	LOG4CXX_INFO(logger,"Push didn't move constraint to distance "<<d<<"\n");
	LOG4CXX_INFO(logger,"Value went from "<<margin<<" to "<<iMargin<<"\n");
      }
      Assert(iMargin >= d-Epsilon);
    }
    else {
      return true;
    }
  }
  LOG4CXX_INFO(logger,"Push didn't converge within 1000 iters"<<"\n");
  LOG4CXX_INFO(logger,"Resulting margin is "<<margin<<"\n");
  return false;
}

bool CompositeInequalityConstraint::Push_i(Vector& x,int i,Real d)
{
  int c = GetConstraint(i);
  return constraints[c]->Push_i(x,i,d);
}

int CompositeInequalityConstraint::GetConstraint(int& i) const
{
  int iorig = i;
  for(size_t k=0;k<constraints.size();k++) {
    int nd=constraints[k]->NumDimensions();
    if(i < nd) {
      return (int)k;
    }
    else {
      i -= nd;
    }
  }
  LOG4CXX_INFO(logger,"Shouldn't ever get here!  i="<<iorig<<" must be out of range 0->"<<NumDimensions()<<"\n");
  AssertNotReached();
  return -1;
}







LimitConstraint::LimitConstraint(const Vector& _bmin,const Vector& _bmax)
  :bmin(_bmin),bmax(_bmax)
{}

int LimitConstraint::NumDimensions() const { return bmin.n+bmax.n; }

Real LimitConstraint::Margin(const Vector& x,int& minConstraint)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);

  Real minMargin=Inf;
  minConstraint = -1;
  for(int i=0;i<bmin.n;i++) {
    if(x(i)-bmin(i) < minMargin) {
      minConstraint = i;
      minMargin = x(i)-bmin(i);
    }
    if(bmax(i)-x(i) < minMargin) {
      minConstraint = i+bmin.n;
      minMargin = bmax(i)-x(i);
    }
  }
  return minMargin;
}

bool LimitConstraint::Satisfies(const Vector& x,Real d)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);

  for(int i=0;i<bmin.n;i++) {
    if(x(i)-d < bmin(i)) return false;
    if(x(i)+d > bmax(i)) return false;
  }
  return true;
}

bool LimitConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  return Eval_i(x,i) >= d;
}

void LimitConstraint::Eval(const Vector& x,Vector& v)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);
  Assert(v.n == bmin.n+bmax.n);
  for(int i=0;i<bmin.n;i++)
    v(i) = x(i) - bmin(i);
  for(int i=0;i<bmax.n;i++)
    v(i+bmin.n) = bmax(i) - x(i);
}

Real LimitConstraint::Eval_i(const Vector& x,int i)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);
  if(i < bmin.n) 
    return x(i) - bmin(i);
  else {
    i-=bmin.n;
    return bmax(i) - x(i);
  }
}

void LimitConstraint::Jacobian(const Vector& x,Matrix& J)
{
  Assert(x.n == bmin.n);
  Assert(J.m == bmin.n+bmax.n);
  Assert(J.n == x.n);
  J.setZero();
  for(int i=0;i<bmin.n;i++) {
    J(i,i) = One;
    J(i+bmin.n,i) = -One;
  }
}

void LimitConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);
  Assert(Ji.n == x.n);
  Ji.setZero();
  if(i < bmin.n) 
    Ji(i) = One;
  else
    Ji(i-bmin.n) = -One;
}


void LimitConstraint::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);
  Assert(h.n == x.n);
  Assert(v.n == bmin.n+bmax.n);
  for(int i=0;i<bmin.n;i++) {
    v(i) = h(i);
    v(i+bmin.n) = -h(i);
  }
}

void LimitConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  Hi.setZero();
}

void LimitConstraint::LineSearch (const Vector& x0,const Vector& dx,Real& u)
{
  Assert(x0.n == dx.n);
  Assert(x0.n == bmin.n);
  Assert(x0.n == bmax.n);
  for(int i=0;i<bmax.n;i++) {
    Assert(x0(i) <= bmax(i));
    Assert(x0(i) >= bmin(i));
    if(x0(i) + u*dx(i) > bmax(i)) {
      Assert(dx(i) > Zero);
      u = (bmax(i)-x0(i))/dx(i);
    }
    if(x0(i) + u*dx(i) < bmin(i)) {
      Assert(dx(i) < Zero);
      u = (bmin(i)-x0(i))/dx(i);
    }
  }
}

void LimitConstraint::LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i)
{
  Assert(x0.n == bmin.n);
  if(i < bmin.n) {
    if(x0(i) + u*dx(i) < bmin(i)) {
      Assert(dx(i) < Zero);
      u = (bmin(i)-x0(i))/dx(i);
    }
  }
  else {
    i -= bmin.n;
    if(x0(i) + u*dx(i) > bmax(i)) {
      Assert(dx(i) > Zero);
      u = (bmax(i)-x0(i))/dx(i);
    }
  }
}

void LimitConstraint::LineSearch_j(const Vector& x0,int j,Real& u)
{
  Assert(x0.n == bmin.n);
  Assert(x0.n == bmax.n);
  Assert(0<=j && j<x0.n);
  Assert(x0(j) <= bmax(j));
  Assert(x0(j) >= bmin(j));
  if(x0(j) + u > bmax(j))
    u = bmax(j)-x0(j);
  if(x0(j) + u < bmin(j))
    u = bmin(j)-x0(j);
}

bool LimitConstraint::Push(Vector& x, Real d)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);
  for(int i=0;i<x.n;i++) {
    if(d+d > bmax(i)-bmin(i)) return false;
    if(x(i)+d > bmax(i))
      x(i) = bmax(i)-d;
    if(x(i)-d < bmin(i))
      x(i) = bmin(i)+d;
  }
  int temp;
  Assert(Margin(x,temp)+Epsilon >= d);
  return true;
}

bool LimitConstraint::Push_i(Vector& x,int i,Real d)
{
  Assert(x.n == bmin.n);
  Assert(x.n == bmax.n);
  if(i >= bmin.n) {
    i -= bmin.n;
    if(x(i)+d > bmax(i))
      x(i) = bmax(i)-d;
  }
  else {
    if(x(i)-d < bmin(i))
      x(i) = bmin(i)+d;
  }
  return true;
}

LinearConstraint::LinearConstraint(const Matrix& _A,const Vector& _b)
  :A(_A),b(_b)
{}

void LinearConstraint::Eval(const Vector& x, Vector& v)
{
  A.mul(x,v); v-=b;
}

Real LinearConstraint::Eval_i(const Vector& x, int i)
{
  return A.dotRow(i,x)-b(i);
}

void LinearConstraint::Jacobian(const Vector& x,Matrix& J)
{
  J = A; 
}

void LinearConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji) 
{
  A.getRowCopy(i,Ji); 
}

Real LinearConstraint::Jacobian_ij(const Vector& x,int i,int j)
{
  return A(i,j);
}

void LinearConstraint::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  A.mul(h,v);
}

void LinearConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  Hi.setZero();
}

bool LinearConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  Assert(x.n == A.n);
  Assert(A.m == b.n);

  return (A.dotRow(i,x)-b(i) >= d);
}

void LinearConstraint::LineSearch (const Vector& x0,const Vector& dx,Real& u)
{
  Assert(x0.n == A.n);
  Assert(A.m == b.n);
  Assert(x0.n == dx.n);
  for(int i=0;i<A.m;i++) {
    Real s=A.dotRow(i,dx);
    Real t=A.dotRow(i,x0);
    Assert(t >= b(i));
    if(t+u*s < b(i)) {
      Assert(s > Zero);
      u = (b(i)-t)/s;
    }
  }
}

void LinearConstraint::LineSearch_i(const Vector& x0,const Vector& dx,Real& u,int i)
{
  Assert(x0.n == A.n);
  Assert(A.m == b.n);
  Assert(x0.n == dx.n);

  Real s=A.dotRow(i,dx);
  Real t=A.dotRow(i,x0);
  Assert(t >= b(i));
  if(t+u*s < b(i)) {
    Assert(s > Zero);
    u = (b(i)-t)/s;
  }
}

bool LinearConstraint::Push(Vector& x, Real d)
{
  Assert(x.n == A.n);
  Assert(A.m == b.n);

  //for each plane, Ai.x >= b must be satisfied by distance d
  //i.e. Ai.x >= b+d, if not, let x=x0+c*Ai
  // => Ai.x0 + c*|Ai|^2 >= b+d
  // => c = (b+d-Ai.x0)/(Ai.Ai)
  //Repeat this process for all planes, reducing the maximum distance pushed
  // = max(c*|Ai|) for each i

  Vector Ai;
  Real pushLimit = Inf;    //convergence tolerance
  Real pushMultiple = One;  //heuristic to improve convergence
  int temp;
  for(int iters=0;iters<1000;iters++) {
    Real maxPush = 0;
    for(int i=0;i<A.m;i++) {
      //gradient of Ai is Ai itself...
      Real g = A.dotRow(i,x);
      if(g < b(i)+d) {
	      Real c = b(i)+d-g;
	      A.getRowRef(i,Ai);
	      Real h = Ai.norm();
	      x.madd(Ai,c/h/h);
	      maxPush = Max(maxPush,c/h);
      }
    }
    if(FuzzyZero(maxPush)) {
      Assert(Margin(x,temp)+Epsilon >= d);
      return true;
    }
    if(maxPush >= pushLimit) {
      LOG4CXX_INFO(logger,"LinearConstraint::Push(): No convergence"<<"\n");
      return false;
    }
    pushMultiple *= 1.01;
    pushLimit = maxPush*pushMultiple;
  }
  LOG4CXX_INFO(logger,"LinearConstraint::PushX(): No convergence in 1000 iters"<<"\n");
  return false;
}

