#ifndef STATISTICS_LOGISTIC_MODEL_H
#define STATISTICS_LOGISTIC_MODEL_H

#include "statistics.h"

namespace Statistics { 

/** @brief Models the probability of a boolean outcome using a logit.
 *
 * The function gives p(x)=e^v/(1+e^v), where v = coeffs^t*x
 * The data must contain a constant term in position n.  This requirement
 * may be relaxed in future versions
 */
class LogisticModel
{
 public:
  LogisticModel() {}
  Real Evaluate(const Vector& x) const;
  Real EvaluateExpectation(const Vector& x,int numSamples=20) const;
  Real EvaluateVariance(const Vector& x,int numSamples=20) const;
  //calculate the maximum likelihood estimate for v[i] the boolean result
  //for datapoint x[i]
  void MaximumLikelihood(const std::vector<Vector>& x,const std::vector<bool>& v,int numIters,Real tol);

  Vector coeffs;
  Matrix covariance;
};

std::ostream& operator << (std::ostream& out,const LogisticModel& model);
std::istream& operator >> (std::istream& in,LogisticModel& model);

} //namespace Statistics

#endif
