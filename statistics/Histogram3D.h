#ifndef STATISTICS_HISTOGRAM3D_H
#define STATISTICS_HISTOGRAM3D_H

#include "statistics.h"
#include <KrisLibrary/structs/array3d.h>

namespace Statistics {

/** @ingroup Statistics
 * @brief 3-D histogram class
 */
class Histogram3D
{
 public:
  typedef Real Point[3];        //Point[0] = x, Point[1] = y, Point[2] = z
  typedef int Index[3];         //Index[0] = i, Index[1] = j, Index[2] = k
  typedef size_t Size[3];       //Size[0] = m, Size[1] = n, Size[2] = p

  Histogram3D();
  /// Creates one big bucket
  void Clear(); 
  /// Creates mxnxp uniformly spaced buckets between min,max
  void Resize(const Size dims,const Point min,const Point max);
  /// Creates mxnxp uniformly spaced buckets between the min/max of data
  void ResizeToFit(const std::vector<Point>& data,const Size dims);
  /// Fills all buckets with the given value
  void Fill(Real val=0);
  /// Calculates the histogram of the data
  void Calculate(const std::vector<Point>& data);
  /// Calculates the histogram of the data, given as (data1[i],data2[i],data3[i])
  void Calculate(const std::vector<Real>& data1,const std::vector<Real>& data2,const std::vector<Real>& data3);

  /// Gets the range of the given bucket
  void GetRange(const Index bucket,Point min,Point max) const;
  void GetBucket(const Point val,Index bucket) const;
  void AddBucket(const Point val,Real num=1);
  void AddBucket(Real v1,Real v2,Real v3,Real num=1);
  Real GetBucketCount(Index bucket) const { return buckets(bucket[0],bucket[1],bucket[2]); }
  Real NumObservations() const;

  std::vector<Real> div1, div2, div3;
  ///(div1.size()+1) x (div2.size()+1) x (div3.size()+1 buckets
  Array3D<Real> buckets;
};

} //namespace Statistics

#endif
