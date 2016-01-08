#ifndef UNIFORM_PROBABILITY_DISTRIBUTION_H
#define UNIFORM_PROBABILITY_DISTRIBUTION_H

#include "ProbabilityDistribution.h"
#include <KrisLibrary/math/random.h>

namespace Statistics {

struct UniformProbabilityDistribution : public UnivariateProbabilityDistribution
{
  UniformProbabilityDistribution(Real a=0,Real b=1);
  virtual ~UniformProbabilityDistribution() {}
  virtual void GetParameters(Vector& parameters);
  virtual void SetParameters(const Vector& parameters);
  virtual Real PDF(Real value);
  virtual Real CDF(Real value);
  virtual Real Minimum() { return a; }
  virtual Real Maximum() { return b; }
  virtual Real Mean() { return Half*(a+b); }
  virtual Real Variance() { return Sqr(b-a)/12.0; }
  virtual Real Skewness() { return 0; }
  virtual Real Kurtosis() { return -6.0/5.0; }
  virtual Real Sample() { return Rand(a,b); }
  virtual bool CanSample() const { return true; }

  Real a,b;
};

struct BoxProbabilityDistribution : public MultivariateProbabilityDistribution
{
  BoxProbabilityDistribution();
  BoxProbabilityDistribution(const Vector& a,const Vector& b);
  virtual ~BoxProbabilityDistribution() {}
  virtual void GetParameters(Vector& parameters);
  virtual void SetParameters(const Vector& parameters);
  virtual int NumDimensions() { return bmin.n; }
  virtual Real PDF(const Vector& value);
  virtual Real CDF(const Vector& value);
  virtual void Minimum(Vector& _bmin) { _bmin=bmin; }
  virtual void Maximum(Vector& _bmax) { _bmax=bmax; }
  virtual void Mean(Vector& mean) { mean.add(bmin,bmax); mean.inplaceMul(Half); }
  virtual void Covariance(Matrix& var);
  virtual void Sample(Vector& x);
  virtual bool CanSample() const { return true; }
  virtual UnivariateProbabilityDistribution* Conditional(int index,const Vector& x);
  virtual MultivariateProbabilityDistribution* Conditional(const std::vector<int> indices,const Vector& x);

  Vector bmin,bmax;
};

} //namespace Statistics

#endif
