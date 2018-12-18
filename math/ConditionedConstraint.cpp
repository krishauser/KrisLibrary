#include "ConditionedConstraint.h"
#include <errors.h>
#include <sstream>
using namespace Math;
using namespace std;

ScaledOffsetConstraint::ScaledOffsetConstraint(InequalityConstraint& _f)
  :InequalityConstraintAdaptor(_f)
{}

string ScaledOffsetConstraint::Label() const
{
  string sf=f.Label();
  stringstream ss;
  ss<<"<scale*"<<sf<<"+offset>";
  return ss.str();
}

string ScaledOffsetConstraint::Label(int i) const
{
  string sf=f.Label(i);
  stringstream ss;
  if(IsInf(scale[i])==1 || IsInf(offset[i])==1) {
    ss<<"<(off)"<<sf<<">";
  }
  else if(scale[i] == One) {
    if(offset[i] == Zero)
      ss<<sf;
    else 
      ss<<"<"<<sf<<"+"<<offset[i]<<">";
  }
  else {
    ss<<"<"<<scale[i]<<"*"<<sf;
    if(offset[i] != Zero) ss<<"+"<<offset[i];
    ss<<">";
  }
  return ss.str();
}

void ScaledOffsetConstraint::PreEval(const Vector& x)
{
  f.PreEval(x);
  Assert((int)scale.size() == NumDimensions());
}

void ScaledOffsetConstraint::Eval(const Vector& x, Vector& v)
{
  f.Eval(x,v);
  for(int i=0;i<v.n;i++) {
    if(IsInf(scale[i])==1 || IsInf(offset[i])==1)
      v(i) = Inf;
    else
      v(i) = v(i)*scale[i]+offset[i];
  }
}

Real ScaledOffsetConstraint::Eval_i(const Vector& x,int i) 
{
  if(IsInf(scale[i])==1 || IsInf(offset[i])==1) return Inf;
  return f.Eval_i(x,i)*scale[i]+offset[i];
}

void ScaledOffsetConstraint::Jacobian(const Vector& x,Matrix& J) 
{
  f.Jacobian(x,J);
  Vector Ji;
  for(int i=0;i<J.m;i++) {
    J.getRowRef(i,Ji);
    if(IsInf(scale[i])==1 || IsInf(offset[i])==1) Ji.setZero();
    else Ji *= scale[i];
  }
}

void ScaledOffsetConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  if(IsInf(scale[i])==1||IsInf(offset[i])==1) Ji.setZero();
  else {
    f.Jacobian_i(x,i,Ji);
    Ji *= scale[i];
  }
}

void ScaledOffsetConstraint::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  f.DirectionalDeriv(x,h,v);
  for(int i=0;i<v.n;i++) {
    if(IsInf(scale[i])==1||IsInf(offset[i])==1) v(i) = Zero;
    else
      v(i) *= scale[i];
  }
}

void ScaledOffsetConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  if(IsInf(scale[i])==1||IsInf(offset[i])==1) 
    Hi.setZero();
  else {
    f.Hessian_i(x,i,Hi);
    Hi.inplaceMul(scale[i]);
  }
}

bool ScaledOffsetConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  Assert(scale[i] != Zero);
  if(IsInf(scale[i])==1||IsInf(offset[i])==1) return true;
  return f.Satisfies_i(x,i,(d-offset[i])/scale[i]);
}

bool ScaledOffsetConstraint::Push_i(Vector& x,int i,Real d) 
{
  Assert(scale[i] != Zero);
  if(IsInf(scale[i])==1||IsInf(offset[i])==1) return true;
  return f.Push_i(x,i,(d-offset[i])/scale[i]);
}

void ScaledOffsetConstraint::LineSearch(const Vector& x0,const Vector& dx,Real& u)
{
  InequalityConstraint::LineSearch(x0,dx,u);
}





UniformScaledOffsetConstraint::UniformScaledOffsetConstraint(InequalityConstraint& _f)
  :InequalityConstraintAdaptor(_f),scale(One),offset(Zero)
{}

string UniformScaledOffsetConstraint::Label() const
{
  string sf=f.Label();
  stringstream ss;
  if(scale==One && offset==Zero)
    return sf;

  ss<<"<";
  if(scale != One) ss<<scale<<"*";
  ss<<sf;
  if(offset != Zero) ss<<"+"<<offset;
  ss<<">";
  return ss.str();
}

void UniformScaledOffsetConstraint::PreEval(const Vector& x)
{
  f.PreEval(x);
  Assert(scale != Zero);
}

void UniformScaledOffsetConstraint::Eval(const Vector& x, Vector& v)
{
  f.Eval(x,v);
  v.inplaceMul(scale);
  for(int i=0;i<v.n;i++) v(i) += offset;
}

Real UniformScaledOffsetConstraint::Eval_i(const Vector& x,int i) 
{
  return f.Eval_i(x,i)*scale+offset;
}

void UniformScaledOffsetConstraint::Jacobian(const Vector& x,Matrix& J) 
{
  f.Jacobian(x,J);
  J.inplaceMul(scale);
}

void UniformScaledOffsetConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  f.Jacobian_i(x,i,Ji);
  Ji *= scale;
}

void UniformScaledOffsetConstraint::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  f.DirectionalDeriv(x,h,v);
  v *= scale;
}

void UniformScaledOffsetConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  f.Hessian_i(x,i,Hi);
  Hi.inplaceMul(scale);
}

bool UniformScaledOffsetConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  return f.Satisfies_i(x,i,(d-offset)/scale);
}

bool UniformScaledOffsetConstraint::Push_i(Vector& x,int i,Real d) 
{
  return f.Push_i(x,i,(d-offset)/scale);
}

void UniformScaledOffsetConstraint::LineSearch(const Vector& x0,const Vector& dx,Real& u)
{
  f.LineSearch(x0,dx,u);
}
