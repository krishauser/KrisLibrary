#include "OnlineMoments.h"
using namespace Statistics; 


OnlineMoments::OnlineMoments()
  :sumWeight(0.0)
{}

void AddOuterProduct(Matrix& A,const Vector& u,const Vector& v,Real w)
{
  assert(A.m==u.n);
  assert(A.n==v.n);
  for(int i=0;i<u.n;i++)
    for(int j=0;j<v.n;j++)
      A(i,j) += u(i)*v(j)*w;
}

void OnlineMoments::AddPoint(const Vector& x,Real weight)
{
  assert(weight >= 0);
  if(weight == 0) return;
  if(sumWeight == 0.0) {
    xmin = x;
    xmax = x;
    mean = x;
    cov.resize(x.n,x.n);
    cov.setZero();
    sumWeight = weight;
    return;
  }
  assert(x.n == xmin.n);
  for(int i=0;i<x.n;i++) {
    xmin(i) = Min(xmin(i),x(i));
    xmax(i) = Max(xmin(i),x(i));
  }
  Real newWeight = sumWeight+weight;
  Vector newmean;
  newmean.mul(mean,sumWeight/newWeight);
  newmean.madd(x,weight/newWeight);
  //cov *= sumWeight/newWeight;
  //AddOuterProduct(cov,x-mean,x-mean,weight*sumWeight/Sqr(newWeight));
  AddOuterProduct(cov,mean,mean,1.0);
  //now it's the old second moment
  cov *= sumWeight/newWeight;
  AddOuterProduct(cov,x,x,weight/newWeight);  
  AddOuterProduct(cov,newmean,newmean,-1.0);  
  sumWeight = newWeight;
  mean = newmean;
}

void OnlineMoments::Clear()
{
  sumWeight = 0.0;
  xmin.clear();
  xmax.clear();
  mean.clear();
  cov.clear();
}
