#ifndef GAUSSIAN_DISTRIBUTION_H
#define GAUSSIAN_DISTRIBUTION_H

#include "ProbabilityDistribution.h"
#include <KrisLibrary/math/gaussian.h>
#include <KrisLibrary/math/random.h>

namespace Statistics {

struct GaussianDistribution : public UnivariateProbabilityDistribution
{
  GaussianDistribution(Real mean=0,Real var=1);
  virtual ~GaussianDistribution() {}
  virtual void GetParameters(Vector& parameters);
  virtual void SetParameters(const Vector& parameters);
  virtual Real PDF(Real value);
  virtual Real CDF(Real value);
  virtual Real Mean() { return mean; }
  virtual Real Variance() { return var; }
  virtual Real Skewness() { return 0; }
  virtual Real Kurtosis() { return 0; }
  virtual Real Sample() { return RandGaussian(mean,Sqrt(var)); }
  virtual bool CanSample() const { return true; }

  Real mean,var;
};

struct GaussianMultivariateDistribution : public MultivariateProbabilityDistribution
{
  GaussianMultivariateDistribution();
  GaussianMultivariateDistribution(const Vector& mean,const Matrix& cov);
  virtual ~GaussianMultivariateDistribution() {}
  virtual void GetParameters(Vector& parameters);
  virtual void SetParameters(const Vector& parameters);
  virtual int NumDimensions() { return gaussian.numDims(); }
  virtual Real PDF(const Vector& value) { return gaussian.probability(value); }
  virtual Real CDF(const Vector& value) { FatalError("Erf not done yet"); return 0; }
  virtual void Mean(Vector& mean) { mean = gaussian.mu; }
  virtual void Covariance(Matrix& var) { gaussian.getCovariance(var); }
  virtual void Sample(Vector& x) { gaussian.generate(x); }
  virtual bool CanSample() const { return true; }

  Gaussian<Real> gaussian;
};

} //namespace Statistics

#endif
