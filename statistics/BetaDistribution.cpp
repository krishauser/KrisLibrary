#include "BetaDistribution.h"
#include <math/misc.h>
using namespace Statistics;


BetaDistribution::BetaDistribution(Real _alpha,Real _beta)
  :alpha(_alpha),beta(_beta)
{}

void BetaDistribution::GetParameters(Vector& parameters)
{
  parameters.resize(2);
  parameters[0]=alpha;
  parameters[1]=beta;
}

void BetaDistribution::SetParameters(const Vector& parameters)
{
  Assert(parameters.n == 2);
  alpha = parameters[0];
  beta = parameters[1];
}

Real BetaDistribution::PDF(Real value)
{
  if(value <= 0 || value >= 1.0) return 0;
  return Pow(value,alpha-One)*Pow(One-value,beta-One)/Beta(alpha,beta);
}

Real BetaDistribution::CDF(Real value)
{
  return NormalizedIncompleteBeta(alpha,beta,value);
}

Real BetaDistribution::Mean()
{
  return alpha/(alpha+beta);
}

Real BetaDistribution::Variance()
{
  return alpha*beta/(Sqr(alpha+beta)*(alpha+beta+1.0));
}

Real BetaDistribution::Skewness()
{
  return Two*(beta-alpha)*Sqrt(alpha+beta+1)/((alpha+beta+2.0)*Sqrt(alpha*beta));
}

Real BetaDistribution::Kurtosis()
{
  Real a3=alpha*alpha*alpha;
  Real a2=alpha*alpha;
  Real a=alpha;
  Real b2=beta*beta;
  Real b=beta;
  return 6.0*(a3-a2*(2.0*b-1)+b2*(b+1)-2.0*a*b*(b+2.0))/(a*b*(a+b+2.0)*(a+b+3.0));
}
