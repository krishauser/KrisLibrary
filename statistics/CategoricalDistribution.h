#ifndef CATEGORICAL_DISTRIBUTION_H
#define CATEGORICAL_DISTRIBUTION_H

#include "ProbabilityDistribution.h"
#include <map>
#include <KrisLibrary/math/cast.h>

namespace Statistics {

struct CategoricalDistribution : public UnivariateProbabilityDistribution
{
  CategoricalDistribution();
  CategoricalDistribution(const std::vector<Real>& w);
  virtual ~CategoricalDistribution() {}
  void SetWeights(const std::vector<Real>& w);
  void SetValues(const std::vector<Real>& values);
  virtual void GetParameters(Vector& parameters);
  virtual void SetParameters(const Vector& parameters);
  virtual Real PDF(Real value);
  virtual Real CDF(Real value);
  virtual bool IsDiscrete() { return true; }
  virtual Real Minimum();
  virtual Real Maximum();
  virtual Real Mean();
  virtual Real Variance();
  virtual Real Skewness();
  virtual Real Kurtosis();
  virtual Real Sample();
  virtual bool CanSample() { return true; }

  std::vector<Real> p;       //the probability of items 0 through n
  std::vector<Real> cp;      //the cumulative probability of items 0 through n
  std::vector<Real> values;  //an optional sorted vector of size n giving the values
};

struct CategoricalMAPModel
{
  CategoricalMAPModel(const std::vector<Real>& priors);
  CategoricalMAPModel(const std::vector<Real>& values,const std::vector<Real>& priors);
  CategoricalDistribution* Estimate();
  void MaximumLikelihood(const std::vector<Real>& data);
  void AddSample(Real value,Real weight=1.0) {
    if(counts.count(value)==0) counts[value]=weight;
    else counts[value]+=weight;
  }
 
  std::map<Real,Real> counts;
};

} //namespace Statistics

#endif
