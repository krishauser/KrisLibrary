#include "GaussianDistribution.h"
#include <math/misc.h>
using namespace Statistics;

GaussianDistribution::GaussianDistribution(Real _mean,Real _var)
  :mean(_mean),var(_var)
{}

void GaussianDistribution::GetParameters(Vector& parameters)
{
  parameters.resize(2);
  parameters(0)=mean;
  parameters(1)=var;
}

void GaussianDistribution::SetParameters(const Vector& parameters) 
{
  Assert(parameters.n==2);
  mean=parameters(0);
  var=parameters(1);
}

Real GaussianDistribution::PDF(Real value) 
{
  return Exp(-Half*Sqr(value-mean)/var)/Sqrt(var*TwoPi);
}

Real GaussianDistribution::CDF(Real value) 
{
  return Half*(1.0+Erf((value-mean)/Sqrt(var*2.0)));
}


GaussianMultivariateDistribution::GaussianMultivariateDistribution()
{}

GaussianMultivariateDistribution::GaussianMultivariateDistribution(const Vector& mean,const Matrix& cov)
  :gaussian(cov,mean)
{}

void GaussianMultivariateDistribution::GetParameters(Vector& parameters)
{
  int n=gaussian.numDims();
  parameters.resize(n + n*(n+1)/2);
  for(int i=0;i<n;i++)
    parameters(i) = gaussian.mu(i);
  int k=n;
  for(int i=0;i<n;i++)
    for(int j=0;j<=i;j++)
      parameters(k) = gaussian.L(i,j);
}

void GaussianMultivariateDistribution::SetParameters(const Vector& parameters)
{
  int n=gaussian.numDims();
  Assert(parameters.n == n+n*(n+1)/2);
  for(int i=0;i<n;i++)
    gaussian.mu(i) = parameters(i);
  int k=n;
  for(int i=0;i<n;i++)
    for(int j=0;j<=i;j++)
      gaussian.L(i,j) = parameters(k);
}

