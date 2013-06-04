#ifndef DIRAC_PROBABILITY_DISTRIBUTION_H
#define DIRAC_PROBABILITY_DISTRIBUTION_H

#include "ProbabilityDistribution.h"

namespace Statistics {

struct DiracDistribution : public UnivariateProbabilityDistribution
{
  DiracDistribution() : value(0) {}
  DiracDistribution(Real _value) : value(_value) {}
  virtual void GetParameters(Vector& parameters) {
    parameters.resize(1);
    parameters(0) = value;
  }
  virtual void SetParameters(const Vector& parameters) {
    Assert(parameters.n == 1);
    value = parameters(0);
  }
  virtual Real PDF(Real _value) { if(value==_value) return 1; else return 0; }
  virtual Real CDF(Real value)  { if(value>=_value) return 1; else return 0; }
  virtual bool IsDiscrete() { return true; }
  virtual Real Minimum() { return value; }
  virtual Real Maximum() { return value; }
  virtual Real Mean() { return value; }
  virtual Real Variance() { return 0; }
  virtual Real Skewness() { return 0; }
  virtual Real Kurtosis() { return 0; }
  virtual Real Moment(int moment) {
    if(moment == 0) return value;
    else return 0;
  }
  virtual Real Sample() { return value; }
  virtual bool CanSample() const { return true; }

  Real value;
};

struct DiracMultivariateDistribution : public MultivariateProbabilityDistribution
{
  DiracMultivariateDistribution() {}
  DiracMultivarateDistribution(const Vector& _value) : value(_value) {}
  virtual int NumDimensions() { return value.n; }
  virtual void GetParameters(Vector& parameters) { parameters = value; }
  virtual void SetParameters(const Vector& parameters) { value = parameters; }
  virtual Real PDF(const Vector& x) { if(value == x) return 1; }
  virtual Real CDF(const Vector& x) {
    Assert(x.n == value.n);
    for(int i=0;i<value.n;i++)
      if(x(i) < value(i)) return 0;
    return 1;
  }
  virtual bool IsDiscrete() { return true; }
  virtual void Minimum(Vector& bmin) { bmin=value; }
  virtual void Maximum(Vector& bmax) { bmax=value; }
  virtual void Mean(Vector& mean) { mean=value; }
  virtual void Covariance(Matrix& m) { m.resize(value.n,value.n,Zero); }
  virtual void Sample(Vector& x) { x=value; }
  virtual bool CanSample() const { return true; }
  virtual UnivariateProbabilityDistribution* Conditional(int var,const Vector& x) {
    for(int i=0;i<x.n;i++) {
      if(i!=var) Assert(x[i] == value[i]);
    }
    return new DiracDistribution(x[var]);
  }
  virtual MultivariateProbabilityDistribution* Conditional(const std::vector<int> indices,const Vector& x) {
    vector<bool> returned(x.n,false);
    for(size_t i=0;i<indices.size();i++)
      returned[indices[i]]=true;
    Vector newvalue(indices.size());
    int k=0;
    for(int i=0;i<x.n;i++) {
      if(!returned[i]) Assert(x[i] == value[i]);
      else newvalue[k++] = x[i];
    }
    return new DiracMultivariateDistribution(newvalue);
  }

  Vector value;
};

} //namespace Statistics

#endif
