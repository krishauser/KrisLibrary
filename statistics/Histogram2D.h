#ifndef STATISTICS_HISTOGRAM2D_H
#define STATISTICS_HISTOGRAM2D_H

#include "statistics.h"
#include <KrisLibrary/structs/array2d.h>

namespace Statistics {

/** @ingroup Statistics
 * @brief 2-D histogram class
 */
class Histogram2D
{
 public:
  typedef Real Point[2];        //Point[0] = x, Point[1] = y
  typedef int Index[2];         //Index[0] = i, Index[1] = j
  typedef size_t Size[2];       //Size[0] = m, Size[1] = n

  Histogram2D();
  /// Creates one big bucket
  void Clear(); 
  /// Creates mxn uniformly spaced buckets betwen [a1,b1]x[a2,b2]
  void Resize(size_t m,size_t n,Real a1,Real b1,Real a2,Real b2);
  void Resize(const Size dims,const Point min,const Point max);
  /// Creates mxn uniformly spaced buckets between the min/max of data1,data2
  void ResizeToFit(const std::vector<Real>& data1,const std::vector<Real>& data2,size_t m,size_t n);
  void ResizeToFit(const std::vector<Point>& data,const Size dims);
  /// Fills all buckets with the given value
  void Fill(Real val=0);
  /// Calculates the histogram of the data
  void Calculate(const std::vector<Point>& data);
  /// Calculates the histogram of the data, given as (data1[i],data2[i])
  void Calculate(const std::vector<Real>& data1,const std::vector<Real>& data2);

  void GetRange(const Index bucket,Point min,Point max) const;
  void GetBucket(const Point val,Index bucket) const;
  void GetBucket(Real v1,Real v2,Index bucket) const;
  void AddBucket(const Point val,Real num=1);
  void AddBucket(Real v1,Real v2,Real num=1);
  Real GetBucketCount(Index bucket) const { return buckets(bucket[0],bucket[1]); }
  Real NumObservations() const;

  std::vector<Real> div1, div2;
  ///(div1.size()+1) x (div2.size()+1) buckets
  Array2D<Real> buckets;
};

} //namespace Statistics

#endif
