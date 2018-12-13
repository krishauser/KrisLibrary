#ifndef STATISTICS_HISTOGRAM_H
#define STATISTICS_HISTOGRAM_H

#include "statistics.h"

namespace Statistics {

/** @ingroup Statistics
 * @brief 1-D histogram class
 */
class Histogram
{
 public:
  Histogram();
  /// Creates one big bucket
  void Clear(); 
  /// Creates buckets (-inf,split],(split,inf)
  void Resize(Real split);
  /// Creates n uniformly spaced buckets betwen a,b 
  void Resize(size_t n,Real a,Real b);
  /// Creates n uniformly spaced buckets between the min and max of the data
  void ResizeToFit(const std::vector<Real>& data,size_t n);
  /// Fills all buckets with the given value
  void Fill(Real val=0);
  /// Calculates the histogram of the data
  void Calculate(const std::vector<Real>& data);

  void GetRange(int bucket,Real& min,Real& max) const;
  int GetBucket(Real val) const;
  void AddBucket(Real val,Real num=1);
  Real GetBucketCount(int bucket) const { return buckets[bucket]; }
  Real NumObservations() const;

  std::vector<Real> divs;
  ///divs.size()+1 buckets (-inf,x0),[x1,x2),...,[xn-1,xn),[xn,inf)
  std::vector<Real> buckets;
};

} //namespace Statistics

#endif
