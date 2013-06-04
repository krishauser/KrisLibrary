#ifndef BERNOULLI_DISTRIBUTION_H
#define BERNOULLI_DISTRIBUTION_H

#include "ProbabilityDistribution.h"

namespace Statistics {

struct BernoulliDistribution : public UnivariateProbabilityDistribution
{
  BernoulliDistribution(Real _p):p(_p) {}
  virtual void GetParameters(Vector& parameters) {
    parameters.resize(1); 
    parameters(0)=p;
  }
  virtual void SetParameters(const Vector& parameters) {
    Assert(parameters.n==1);
    p=parameters(0); 
  }
  virtual Real PDF(Real value) {
    if(value==0.0) return 1-p;
    else if(value==1.0) return p; 
    else return 0; 
  }
  virtual Real CDF(Real value) {
    if(value==1.0) return 1;
    else if(value<0.0) return 0;
    else return 1-p; 
  }
  virtual bool IsDiscrete() { return true; }
  virtual Real Minimum() { return 0; }
  virtual Real Maximum() { return 1; }
  virtual Real Mean() { return p; }
  virtual Real Variance() { return p*(1-p); }
  virtual Real Skewness() { return (1.0-2.0*p)/Sqrt(p*(1.0-p)); }
  virtual Real Kurtosis() { return (1.0-6.0*p+6.0*p*p)/p*(1.0-p)); }
  virtual Real Sample() { return RandBool(p); }
  virtual bool CanSample() const { return true; }

  Real p;
};

struct BernoulliMAPModel
{
  BernoulliMAPModel(Real prior0,Real prior1);
  BernoulliDistribution* Estimate() { return new BernoulliDistribution(n1/(n0+n1))); }
  virtual void MaximumLikelihood(const vector<Real>& data);
  void AddSample(bool success) { if(success) n1 += 1.0; else n0 += 1.0; }

  Real n0,n1;
};

} //namespace Statistics

#endif
