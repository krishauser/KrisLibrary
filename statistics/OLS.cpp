#include <KrisLibrary/Logger.h>
#include "OLS.h"
#include <math/linalgebra.h>
#include <math/LDL.h>
#include <iostream>
using namespace std;

namespace Statistics {

LeastSquares1D::LeastSquares1D(const Vector& _x,const Vector& _y)
  :x(_x),y(_y),w(NULL),a(0),b(0),stda(0),stdb(0),corrCoeff(0)
{}

LeastSquares1D::LeastSquares1D(const Vector& _x,const Vector& _y,const Vector& weights)
  :x(_x),y(_y),w(&weights),a(0),b(0),stda(0),stdb(0),corrCoeff(0)
{}

bool LeastSquares1D::Solve()
{
  if(w) return SolveWeighted();
  else return SolveUnweighted();
}

bool LeastSquares1D::SolveUnweighted()
{
  assert(x.n == y.n);
  assert(x.n >= 2);
  Real sxx,syy,sxy;
  Real xmean = Mean(x);
  Real ymean = Mean(y);
  sxx = syy = sxy = 0;
  for(int i=0;i<x.n;i++) {
    sxx += Sqr(x(i) - xmean);
    syy += Sqr(y(i) - ymean);
    sxy += (x(i) - xmean)*(y(i) - ymean);
  }
  if(sxx == 0) {  //vertical line!
    b = xmean;
    a = Inf;
    stda = stdb = 0;
    corrCoeff = 1;
    return true;
  }
  a = sxy / sxx;
  b = ymean - b*xmean;
  if(x.n == 2) {
    stda = stdb = 0;
    corrCoeff = 1;
    return true;
  }
  Real s = Sqrt((syy - b*sxy)/(x.n-2));
  stda = s / Sqrt(sxx);
  stdb = s*Sqrt(One/(Real)x.n + Sqr(xmean)/sxx);
  return true;
}

bool LeastSquares1D::SolveWeighted()
{
  assert(w != NULL);
  assert(x.n == y.n);
  assert(x.n >= 2);
  Real sxx,syy,sxy;
  Real xmean = WeightedMean(x,*w);
  Real ymean = WeightedMean(y,*w);
  sxx = syy = sxy = 0;
  for(int i=0;i<x.n;i++) {
    Real w2 = Sqr((*w)(i));
    sxx += w2*Sqr(x(i) - xmean);
    syy += w2*Sqr(y(i) - ymean);
    sxy += w2*(x(i) - xmean)*(y(i) - ymean);
  }
  if(sxx == 0) {  //vertical line!
    b = xmean;
    a = Inf;
    stda = stdb = 0;
    corrCoeff = 1;
    return true;
  }
  a = sxy / sxx;
  b = ymean - b*xmean;
  if(x.n == 2) {
    stda = stdb = 0;
    corrCoeff = 1;
    return true;
  }
  Real wtotal = Sum(*w);
  Real s = Sqrt((syy - b*sxy)/(wtotal-2));
  stda = s / Sqrt(sxx);
  stdb = s*Sqrt(One/wtotal + Sqr(xmean)/sxx);
  return true;
}


bool LeastSquares1D::SolvePerpendicular()
{
  //minimize sum_i { (yi - a*xi - b)^2 / (1+b^2) }
  assert(x.n == y.n);
  assert(x.n >= 2);
  Real sxx,syy,sxy;
  Real xmean = Mean(x);
  Real ymean = Mean(y);
  sxx = syy = sxy = 0;
  for(int i=0;i<x.n;i++) {
    sxx += Sqr(x(i) - xmean);
    syy += Sqr(y(i) - ymean);
    sxy += (x(i) - xmean)*(y(i) - ymean);
  }
  Real den = (x.n*xmean*ymean - dot(x,y));
  if(FuzzyZero(den)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, denominator is close to zero");
  }
  Real B = (syy - sxx) / den;
  a = -B + Sqrt(Sqr(B)+1);
  b = ymean - xmean*b;
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, don't know how to calculate std errors for perpendicular least-squares");
  stda = stdb = 0;
  corrCoeff = 1;
  return true;
}

bool LeastSquares(const Matrix& data,const Vector& outcome,
		  Vector& coeffs,Real& offset)
{
  assert(outcome.n == data.m);
  //solve for vector (coeffs,offset) using normal equations
  Matrix A(data.n+1,data.n+1);
  Vector b(data.n+1),c(data.n+1);
  Vector xi,xj;
  for(int i=0;i<data.n;i++) {
    data.getColRef(i,xi);
    for(int j=i;j<data.n;j++) {
      data.getColRef(j,xj);
      A(j,i) = A(i,j) = xi.dot(xj);
    }
    b(i) = xi.dot(outcome);
  }
  for(int i=0;i<data.n;i++) {
    data.getColRef(i,xi);
    A(data.n,i) = A(i,data.n) = Sum(xi);
  }
  A(data.n,data.n) = (Real)data.m;
  b(data.n) = Sum(outcome);

  //Solve Ac=b for coefficients c;
  LDLDecomposition<Real> ldl;
  ldl.set(A);
  ldl.backSub(b,c);
  /*  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Cholesky didn't succeed!");
    LOG4CXX_INFO(KrisLibrary::logger(),"Matrix: "<<MatrixPrinter(A)<<"\n");
    return false;
  }
  */
  coeffs.resize(data.n);
  c.getSubVectorCopy(0,coeffs);
  offset = c(data.n);

  return true;
}

bool LeastSquares(const vector<Vector>& data,int dependentVariable,
		  Vector& coeffs)
{
  assert(!data.empty());
  int n=data[0].n;
  assert((int)data.size() >= n);
  assert(dependentVariable >= 0 && dependentVariable < n);
  Matrix mdata((int)data.size(),n-1);
  Vector vdep((int)data.size());
  for(size_t i=0;i<data.size();i++) {
    assert(data[i].n == n);
    for(int j=0;j<n;j++) {
      if(j < dependentVariable)
	mdata(i,j) = data[i](j);
      else if(j > dependentVariable)
	mdata(i,j-1) = data[i](j);
      else
	vdep(i) = data[i](j);
    }
  }
  Vector tempcoeffs; Real offset;
  if(!LeastSquares(mdata,vdep,tempcoeffs,offset)) return false;
  coeffs.resize(n);
  for(int j=0;j<n;j++) {
    if(j < dependentVariable)
      coeffs(j) = tempcoeffs(j);
    else if(j > dependentVariable)
      coeffs(j) = tempcoeffs(j-1);
    else
      coeffs(j) = offset;
  }
  return true;
}

bool LeastSquaresPickDependent(const vector<Vector>& data,
			       int& dependentVariable,Vector& coeffs)
{
  //pick the dimension with the smallest standard deviation
  Vector stddev;
  StdDev_Robust(data,stddev);
  Real minstddev = stddev.minElement(&dependentVariable);
  return LeastSquares(data,dependentVariable,coeffs);
}

} //namespace Statistics
