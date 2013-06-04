#ifndef STATISTICS_DISTRIBUTION_COLLECTOR_H
#define STATISTICS_DISTRIBUTION_COLLECTOR_H

#include "statistics.h"

namespace Statistics {

/** @ingroup Statistics
 * @brief Incrementally collects samples from a univariate distribution.
 *
 * Allows calculating the moments 1-3 (mean,variance,skewness) and the
 * minimum and maximum values.
 */
struct DistributionCollector
{
  DistributionCollector();  
  void operator << (Real x) { collect(x); }
  void collect (Real x);
  void weightedCollect (Real x,Real weight);

  Real number() const { return n; }
  Real minimum() const { return xmin; }
  Real maximum() const { return xmax; }
  Real average() const { return sum/n; }
  Real mean() const { return sum/n; }
  Real variance() const {
    Real avg=mean();
    return Max(sumsquared/n - avg*avg,Zero);  //may have small numerical errors
  }
  Real stddev() const { return Sqrt(variance()); }
  Real skewness() const;
  Real moment(int i,Real center) const;
  void clear();
  
  Real n;
  Real xmin,xmax;
  Real sum,sumsquared,sumcubed;
};

/** @ingroup Statistics
 * @brief Incrementally collects samples from a multivariate distribution.
 *
 * Allows calculating the moments 1-2 (mean,variance) and the
 * minimum and maximum values.
 */
class DistributionCollectorND
{
 public:
  DistributionCollectorND();  
  void operator << (const Vector& x) { collect(x); }
  void collect (const Vector& x);
  void weightedCollect (const Vector& x,Real weight);

  //convenience functions
  void collect (Real x1);
  void collect (Real x1,Real x2);
  void collect (Real x1,Real x2,Real x3);
  void collect (Real x1,Real x2,Real x3,Real x4);

  Real number() const { return n; }
  const Vector& minimum() const { return xmin; }
  const Vector& maximum() const { return xmax; }
  void getMean(Vector& mean) const;
  void getVariance(Vector& var) const;
  void getCovariance(Matrix& covar) const;
  void getStddev(Vector& stddev) const;
  void clear();

  Real n;
  Vector xmin,xmax;
  Vector sum;
  Matrix sumouterproduct;
};

} //namespace Statistics

#endif
