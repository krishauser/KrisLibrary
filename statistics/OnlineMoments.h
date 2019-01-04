#ifndef STATISTICS_ONLINE_MOMENTS_H
#define STATISTICS_ONLINE_MOMENTS_H

#include "statistics.h"

namespace Statistics {

/** @brief Online calculation of mean, covariance, min/max of vectors
 *
 * For scalars, use utils/StatCollector
 */
struct OnlineMoments
{
  OnlineMoments();
  void AddPoint(const Vector& x,Real weight=1.0);
  void Clear();

  Real sumWeight;
  Vector xmin,xmax;
  Vector mean;
  Matrix cov;
};

} //namespace Statistics

#endif
