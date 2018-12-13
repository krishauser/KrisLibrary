#ifndef BETA_DISTRIBUTION_H
#define BETA_DISTRIBUTION_H

#include "ProbabilityDistribution.h"
#include <KrisLibrary/math/random.h>

namespace Statistics {

struct BetaDistribution : public UnivariateProbabilityDistribution
{
  BetaDistribution(Real alpha=1,Real beta=1);
  virtual ~BetaDistribution() {}
  virtual void GetParameters(Vector& parameters);
  virtual void SetParameters(const Vector& parameters);
  virtual Real PDF(Real value);
  virtual Real CDF(Real value);
  virtual Real Mean();
  virtual Real Variance();
  virtual Real Skewness();
  virtual Real Kurtosis();

  Real alpha,beta;
};

} //namespace Statistics

#endif
