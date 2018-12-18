#ifndef PROBABILITY_DISTRIBUTION_H
#define PROBABILITY_DISTRIBUTION_H

#include <KrisLibrary/math/math.h>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/matrix.h>
#include <errors.h>
#include <vector>

namespace Statistics {

  using namespace Math;

struct UnivariateProbabilityDistribution
{
  virtual ~UnivariateProbabilityDistribution() {}
  virtual void GetParameters(Vector& parameters)=0;
  virtual void SetParameters(const Vector& parameters)=0;
  virtual Real PDF(Real value)=0;
  virtual Real CDF(Real value)=0;
  virtual bool IsDiscrete() { return false; }
  virtual Real Minimum() { return -Inf; }
  virtual Real Maximum() { return Inf; }
  virtual Real Mean()=0;
  virtual Real StandardDeviation() { return Sqrt(Variance()); }
  virtual Real Variance() { FatalError("Variance not defined"); return 0; }
  virtual Real Skewness() { FatalError("Skewness not defined"); return 0; }
  virtual Real Kurtosis() { FatalError("Kurtosis not defined"); return 0; }
  virtual Real Moment(int moment) {
    if(moment == 0) return Mean();
    else if(moment == 1) return Variance();
    else if(moment == 2) return Skewness();
    else if(moment == 3) return Kurtosis();
    else FatalError("Moment %d not defined",moment);
    return 0;
  }
  virtual Real Sample() { return 0; }
  virtual bool CanSample() const { return false; }
};

struct MultivariateProbabilityDistribution
{
  virtual ~MultivariateProbabilityDistribution() {}
  virtual void GetParameters(Vector& parameters)=0;
  virtual void SetParameters(const Vector& parameters)=0;
  virtual int NumDimensions()=0;
  virtual Real PDF(const Vector& value)=0;
  virtual Real CDF(const Vector& value)=0;
  virtual bool IsDiscrete() { return false; }
  virtual void Minimum(Vector& bmin) { bmin.resize(NumDimensions(),-Inf); }
  virtual void Maximum(Vector& bmax) { bmax.resize(NumDimensions(),Inf); }
  virtual void Mean(Vector& mean)=0;
  virtual void Covariance(Matrix& var) { FatalError("Covariance not defined");}
  virtual void Sample(Vector& x) { }
  virtual bool CanSample() const { return false; }
  //returns the probability distribution on x[index] conditional on
  //the other x[i] as a dirac distribution
  virtual UnivariateProbabilityDistribution* Conditional(int index,const Vector& x) { return NULL; }
  //returns the probability distribution on x[indices] conditional on
  //the other x[i] as a dirac distribution
  virtual MultivariateProbabilityDistribution* Conditional(const std::vector<int> indices,const Vector& x)  { return NULL; }
};


} //namespace Statistics

#endif
