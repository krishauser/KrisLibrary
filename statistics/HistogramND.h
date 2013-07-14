#ifndef STATISTICS_HISTOGRAMND_H
#define STATISTICS_HISTOGRAMND_H

#include "statistics.h"
#ifdef _MSC_VER
//MSVC doesn't put STL TR1 items in a tr1 folder
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif //_MSC_VER

namespace Statistics {

struct IndexHash
{
  IndexHash(size_t pow=257);
  size_t operator () (const std::vector<int>& x) const;
  size_t pow;
};

/** @ingroup Statistics
 * @brief N-D histogram class
 *
 * Uses unordered_map so the histogram can be sparse and of manageable size even when N is large
 */
class HistogramND
{
 public:
  typedef Vector Point;
  typedef std::vector<int> Index;
  typedef std::vector<size_t> Size;

  HistogramND(int numDims=0);
  /// resizes to the given number of dimensions, sets one big bucket
  void Resize(int numDims);
  /// Creates one big bucket
  void Clear(); 
  /// Creates mxnxp uniformly spaced buckets between min,max
  void Resize(const Size& dims,const Point& min,const Point& max);
  void Resize(const Index& dims,const Point& min,const Point& max);
  /// Creates mxnxp uniformly spaced buckets between the min/max of data
  void ResizeToFit(const std::vector<Point>& data,const Size& dims);
  /// Fills all buckets with the given value
  void Fill(Real val=0);
  /// Calculates the histogram of the data
  void Calculate(const std::vector<Point>& data);

  /// Gets the range of the given bucket
  void GetRange(const Index& bucket,Point& min,Point& max) const;
  void GetBucket(const Point& val,Index& bucket) const;
  void AddBucket(const Point& val,Real num=1);
  Real GetBucketCount(const Index& bucket) const;
  Real NumObservations() const;

  std::vector<std::vector<Real> > divs;
  typedef std::tr1::unordered_map<Index,Real,IndexHash> BucketHash;
  BucketHash buckets;
};

} //namespace Statistics

#endif
