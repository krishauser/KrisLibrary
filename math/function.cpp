#include <KrisLibrary/Logger.h>
#include "function.h"
#include "vectorfunction.h"
#include "vector.h"
#include "matrix.h"
#include <errors.h>
using namespace Math;
using namespace std;


Real ScalarFieldFunction::DirectionalDeriv(const Vector& x,const Vector& h)
{
  Vector grad;
  Gradient(x,grad);
  return grad.dot(h);
}

Real ScalarFieldFunction::DirectionalDeriv2(const Vector& x,const Vector& h)
{
    LOG4CXX_ERROR(KrisLibrary::logger(),"ScalarFieldFunction::DirectionalDeriv2: Warning, possibly inefficient evaluation\n");
  Matrix H(x.n,x.n);
  Hessian(x,H);
  //calc h^t H h
  Real d=Zero;
  for(int i=0;i<x.n;i++) {
    d += h(i)*H.dotRow(i,h);
  }
  return d;
}


std::string VectorFunction::Label(int i) const
{
  std::string str = Label();
  char buf[32];
  snprintf(buf,32,"[%d]",i);
  str += buf;
  return str;
}

void VectorFunction::operator()(Real t,Vector& x) { PreEval(t); Eval(t,x); }

std::string ScalarFieldFunction::VariableLabel(int i) const
{
  char buf[32];
  snprintf(buf,32,"x[%d]",i);
  return buf;
}

std::string VectorFieldFunction::Label() const
{
  return "<unknown Rm->Rn>";
}

std::string VectorFieldFunction::Label(int i) const
{
  std::string str = Label();
  char buf[32];
  snprintf(buf,32,"[%d]",i);
  str += buf;
  return str;
}

std::string VectorFieldFunction::VariableLabel(int i) const
{
  char buf[32];
  snprintf(buf,32,"x[%d]",i);
  return buf;
}

Real VectorFieldFunction::Eval_i(const Vector& x,int i)
{
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: really inefficient call VectorFieldFunction::Eval_i\n");
  Vector v(NumDimensions());
  Eval(x,v);
  return v(i);
}

void VectorFieldFunction::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  Ji.resize(x.n);
  for(int j=0;j<Ji.n;j++)
    Ji(j) = Jacobian_ij(x,i,j);
}

void VectorFieldFunction::Jacobian_j(const Vector& x,int j,Vector& Jj)
{
  Jj.resize(NumDimensions());
  for(int i=0;i<Jj.n;i++)
    Jj(i) = Jacobian_ij(x,i,j);
}

void VectorFieldFunction::Jacobian(const Vector& x,Matrix& J)
{
  J.resize(NumDimensions(),x.n);
  for(int i=0;i<J.m;i++)
    for(int j=0;j<J.n;j++)
      J(i,j) = Jacobian_ij(x,i,j);
}

void VectorFieldFunction::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  Matrix J;
  Jacobian(x,J);
  J.mul(h,v);
}

Real VectorFieldFunction::Divergence(const Vector& x)
{
  Assert(x.n==NumDimensions());
  Real sum=Zero;
  for(int i=0;i<x.n;i++) sum+=Jacobian_ij(x,i,i);
  return sum;
}





