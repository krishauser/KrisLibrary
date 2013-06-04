#include "DistributionCollector.h"
#include <limits>
#include "errors.h"
using namespace Statistics;
using namespace std;

DistributionCollector::DistributionCollector()
  :n(0),xmin(numeric_limits<Real>::infinity()),xmax(-numeric_limits<Real>::infinity()),sum(0),sumsquared(0),sumcubed(0)
{}

void DistributionCollector::clear()
{
  n=0;
  xmin=numeric_limits<Real>::infinity();
  xmax=-numeric_limits<Real>::infinity();
  sum=0;
  sumsquared=0;
  sumcubed=0;
}
  
void DistributionCollector::collect(Real x)
{
  if(x<xmin) xmin=x;
  if(x>xmax) xmax=x;
  sum += x;
  sumsquared += x*x;
  sumcubed += x*x*x;
  n+=1;
}

void DistributionCollector::weightedCollect(Real x,Real weight)
{
  if(x<xmin) xmin=x;
  if(x>xmax) xmax=x;
  sum += weight*x;
  sumsquared += weight*x*x;
  sumcubed += weight*x*x*x;
  n+=weight;
}

Real DistributionCollector::skewness() const
{
  Real std = stddev();
  return moment(3,mean())/(std*std*std);
}

Real DistributionCollector::moment(int i,Real center) const
{
  if(i==1) {
    return mean() - center;
  }
  else if(i==2) {
    //sum (x[k] - center)^2 / n =
    //sum (x[k]^2 - 2x[k]*center + center^2) / n
    //= sumsquared/n - 2*sum*center/n + center^2
    return (sumsquared - Two*sum*center)/n + Sqr(center);
  }
  else if(i==3) {
    //sum (x[k] - center)^3 / n =
    //sum (x[k]^3 - 3x[k]^2*center + 3x[k]*center^2 + center^3) / n
    //= sumcubed/n - 3*sumsquared*center/n + 3*sum*center/n - center^3
    return (sumcubed - 3.0*sumsquared*center + 3.0*sum*Sqr(center))/n - center*center*center;
  }
  else FatalError("Unable to do moments higher than 3");
  return 0;
}



void AddOuterProduct(Matrix& a,const Vector& x,Real weight=1.0)
{
  for(int i=0;i<a.m;i++)
    for(int j=0;j<a.n;j++)
      a(i,j) += weight*x(i)*x(j);
}

DistributionCollectorND::DistributionCollectorND()
  :n(0)
{}

void DistributionCollectorND::collect (const Vector& x)
{
  if(sum.empty()) {
    sum.resize(x.n,Zero);
    sumouterproduct.resize(x.n,x.n,Zero);
    xmin.resize(x.n,numeric_limits<Real>::infinity());
    xmax.resize(x.n,-numeric_limits<Real>::infinity());
  }
  Assert(sum.n == x.n);

  n += 1;
  sum += x;
  AddOuterProduct(sumouterproduct,x);
  for(int i=0;i<x.n;i++) {
    if(x(i) < xmin(i)) xmin(i) = x(i);
    if(x(i) > xmax(i)) xmax(i) = x(i);
  }
}

void DistributionCollectorND::weightedCollect (const Vector& x,Real weight)
{
  if(sum.empty()) {
    sum.resize(x.n,Zero);
    sumouterproduct.resize(x.n,x.n,Zero);
    xmin.resize(x.n,numeric_limits<Real>::infinity());
    xmax.resize(x.n,-numeric_limits<Real>::infinity());
  }
  Assert(sum.n == x.n);

  n += weight;
  sum.madd(x,weight);
  AddOuterProduct(sumouterproduct,x,weight);
  for(int i=0;i<x.n;i++) {
    if(x(i) < xmin(i)) xmin(i) = x(i);
    if(x(i) > xmax(i)) xmax(i) = x(i);
  }
}

void DistributionCollectorND::collect (Real x1)
{
  Vector x(1);
  x(0) = x1;
  collect(x);
}

void DistributionCollectorND::collect (Real x1,Real x2)
{
  Vector x(2);
  x(0) = x1;
  x(1) = x2;
  collect(x);
}

void DistributionCollectorND::collect (Real x1,Real x2,Real x3)
{
  Vector x(3);
  x(0) = x1;
  x(1) = x2;
  x(2) = x3;
  collect(x);
}


void DistributionCollectorND::collect (Real x1,Real x2,Real x3,Real x4)
{
  Vector x(4);
  x(0) = x1;
  x(1) = x2;
  x(2) = x3;
  x(3) = x4;
  collect(x);
}


void DistributionCollectorND::getMean(Vector& mean) const
{
  if(!sum.empty()) mean.mul(sum,1.0/n);
}

void DistributionCollectorND::getVariance(Vector& var) const
{
  getMean(var);
  for(int i=0;i<var.n;i++) {
    var(i) = sumouterproduct(i,i)/n - var(i)*var(i);
  }
}

void DistributionCollectorND::getCovariance(Matrix& covar) const
{
  Vector mean;
  getMean(mean);
  covar.mul(sumouterproduct,1.0/n);
  AddOuterProduct(covar,mean,-1.0);
}

void DistributionCollectorND::getStddev(Vector& stddev) const
{
  getVariance(stddev);
  for(int i=0;i<stddev.n;i++) stddev(i) = Sqrt(stddev(i));
}

void DistributionCollectorND::clear()
{
  n=0;
  xmin.clear();
  xmax.clear();
  sum.clear();
  sumouterproduct.clear();
}
