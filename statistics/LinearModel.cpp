#include <KrisLibrary/Logger.h>
#include "LinearModel.h"
#include "OLS.h"
#include "errors.h"
#include <math/LDL.h>
#include <math/SVDecomposition.h>
#include <utils/StatCollector.h>
#include <Timer.h>
using namespace Statistics;
using namespace std;

//evaluates x^T*A*y
inline Real EvalQuadraticForm(const Vector& x,const Matrix& A,const Vector& y)
{
  Real sum=0;
  for(int j=0;j<A.n;j++)
    sum += y(j) * A.dotCol(j,x);
  return sum;
}


LinearModel::LinearModel()
  :solveMethod(Cholesky),constantOffset(true)
{}

Real LinearModel::Evaluate(const Vector& x) const
{
  if(constantOffset) {
    Assert(x.n+1 == coeffs.n);
    Vector c1;
    c1.setRef(coeffs);
    c1.n--;
    return dot(c1,x)+coeffs(x.n);
  }
  else {
    Assert(x.n == coeffs.n);
    return dot(coeffs,x);
  }
}

Real LinearModel::Variance(const Vector& x) const
{
  return EvalQuadraticForm(x,covariance,x);
}

void LinearModel::CoeffVariance(Vector& coeffVariance) const
{
  covariance.getDiagRef(0,coeffVariance);
}

bool LinearModel::LeastSquares(const Matrix& data,const Vector& outcome)
{
  if(constantOffset) {
    //check if you need to add a constant offset to the data
    bool lastColConstant = true;
    for(int i=0;i<data.m;i++)
      if(data(i,data.n-1) != 1.0) {
	lastColConstant = false;
	break;
      }
    if(!lastColConstant) {  //append a column of one's
      Matrix newData(data.m,data.n+1);
      newData.copySubMatrix(0,0,data);
      for(int i=0;i<data.m;i++)
	newData(i,data.n) = 1;
      return LeastSquares_Internal(newData,outcome);
    }
  }
  return LeastSquares_Internal(data,outcome);
}

bool LinearModel::LeastSquares_Internal(const Matrix& data,const Vector& outcome)
{
  switch(solveMethod) { 
  case Cholesky: return LeastSquares_Cholesky(data,outcome);
  case SVD: return LeastSquares_SVD(data,outcome);
  case QR: return LeastSquares_QR(data,outcome);
  default:
    FatalError("Invalid solve method");
    return false;
  }
}

bool LinearModel::LeastSquares(const vector<Vector>& x,const vector<Real>& outcome)
{
  Assert(!x.empty());
  Assert(x.size()==outcome.size());
  int n=x[0].n;
  for(size_t i=0;i<x.size();i++)
    Assert(x[i].n == n);

  Vector b(x.size());
  for(size_t i=0;i<x.size();i++)
    b(i) = outcome[i];

  Matrix X;
  if(constantOffset) {
    Vector xi;
    X.resize(x.size(),n+1);
    for(size_t i=0;i<x.size();i++) {
      X.getRowRef(i,xi);
      xi.copySubVector(0,x[i]);
      xi(n) = 1;
    }
    for(size_t i=0;i<x.size();i++)
      b(i) = outcome[i];
  }
  else {
    for(size_t i=0;i<x.size();i++)
      X.copyRow(i,x[i]);
  }
  return LeastSquares_Internal(X,b);
}

bool LinearModel::LeastSquares(const vector<Vector>& x,int outcomeIndex)
{
  FatalError("Not done yet");
  return false;
  /*
  Vector c;
  bool res=::LeastSquares(x,outcomeIndex,c);
  Assert(res);

  coeffs.resize(c.n-1);
  for(int i=0;i<outcomeIndex;i++)
    coeffs(i)=c(i);
  offset = c(outcomeIndex);
  for(int i=outcomeIndex+1;i<c.n;i++)
    coeffs(i-1)=c(i);
  */
}

bool LinearModel::LeastSquares_Cholesky(const Matrix& data,const Vector& outcome)
{
  Assert(outcome.n == data.m);
  //solve for vector (coeffs,offset) using normal equations
  Matrix A(data.n,data.n);
  Vector b(data.n),c(data.n);
  Vector xi,xj;
  for(int i=0;i<data.n;i++) {
    data.getColRef(i,xi);
    for(int j=i;j<data.n;j++) {
      data.getColRef(j,xj);
      A(j,i) = A(i,j) = xi.dot(xj);
    }
    b(i) = xi.dot(outcome);
  }

  //Solve Ac=b for coefficients c;
  LDLDecomposition<Real> ldl;
  ldl.set(A);
  ldl.backSub(b,coeffs);
  /*  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Cholesky didn't succeed!");
    LOG4CXX_INFO(KrisLibrary::logger(),"Matrix: "<<MatrixPrinter(A)<<"\n");
    return false;
  }
  */

  //get the covariance matrix (inverse of normal equations * s.o.s. errors)
  ldl.getInverse(covariance);

  //scale by residual error squared
  Vector resid;
  data.mul(coeffs,resid);
  resid -= outcome;

  Real residSqr = resid.normSquared();
  covariance *= residSqr;

  return true;
}



bool LinearModel::LeastSquares_SVD(const Matrix& data,const Vector& outcome)
{
  Assert(outcome.n == data.m);
  SVDecomposition<Real> svd;
  if(!svd.set(data)) {
    return false;
  }
  svd.backSub(outcome,coeffs);

  //get the covariance matrix (inverse of normal equations * s.o.s. errors)
  //get the matrix (AtA)^-1
  //  = (VWU'TUWV')^-1
  //  = (VWWV')^-1
  //  = VW^-2V'

  DiagonalMatrix sv_inv_sqr(svd.W.n);
  for(int i=0;i<svd.W.n;i++) {
    if(Abs(svd.W(i)) > svd.epsilon)
      sv_inv_sqr(i) = 1.0/Sqr(svd.W(i));
    else
      sv_inv_sqr(i) = 0;
  }
  Matrix VW;
  sv_inv_sqr.postMultiply(svd.V,VW);
  covariance.mulTransposeB(VW,svd.V);

  //scale by residual error squared
  Vector resid;
  data.mul(coeffs,resid);
  resid -= outcome;

  Real residSqr = resid.normSquared();
  covariance *= residSqr;

  return true;
}

bool LinearModel::LeastSquares_QR(const Matrix& data,const Vector& outcome)
{
  FatalError("Not done yet!\n");
  return false;
}

namespace Statistics {

std::istream& operator >> (std::istream& in,LinearModel& model)
{
  in >> model.coeffs;
  return in;
}

std::ostream& operator << (std::ostream& out,const LinearModel& model)
{
  out << model.coeffs;
  return out;
}

} //namespace Statistics
