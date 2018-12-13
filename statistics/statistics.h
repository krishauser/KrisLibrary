#ifndef STAT_STATISTICS_H
#define STAT_STATISTICS_H

#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/matrix.h>
#include <vector>
#include <numeric>

/** @defgroup Statistics
 * @brief Various statistical algorithms.
 */

/** @file statistics/statistics.h
 * @ingroup Statistics
 * @brief Basic statistical utilities.
 *
 * Real-valued data is given either by an STL vector or a Math Vector.
 * Vector-valued data is given either as an STL vector of Math Vector's or
 * a Matrix whose rows are the observations.
 */

/** @ingroup Statistics
 * @brief Contains all definitions in the statistics directory.
 */
namespace Statistics {
  using namespace Math;

/**@addtogroup Statistics*/
/*@{*/

inline Real Sum(const std::vector<Real>& data)
{ return std::accumulate(data.begin(),data.end(),Zero); }
inline Real Sum(const Vector& data)
{ return std::accumulate(data.begin(),data.end(),Zero); }
void Sum(const std::vector<Vector>& data,Vector& sum);
void Sum(const Matrix& data,Vector& sum);
Real Mean(const std::vector<Real>& data);
void Mean(const std::vector<Vector>& data,Vector& mean);
Real Mean(const Vector& data);
void Mean(const Matrix& data,Vector& mean);
Real Variance(const std::vector<Real>& data);
void Variance(const std::vector<Vector>& data,Vector& var);
Real Variance(const Vector& data);
void Variance(const Matrix& data,Vector& var);
Real StdDev(const std::vector<Real>& data);
void StdDev(const std::vector<Vector>& data,Vector& stddev);
Real StdDev(const Vector& data);
void StdDev(const Matrix& data,Vector& stddev);
Real StdDev_Robust(const std::vector<Real>& data);
void StdDev_Robust(const std::vector<Vector>& data,Vector& stddev);
Real StdDev_Robust(const Vector& data);
void StdDev_Robust(const Matrix& data,Vector& stddev);

Real WeightedSum(const Vector& data,const Vector& w);
void WeightedSum(const Matrix& data,const Vector& w,Vector& sum);
Real WeightedMean(const Vector& data,const Vector& w);
void WeightedMean(const Matrix& data,const Vector& w,Vector& mean);
Real WeightedVariance(const Vector& data,const Vector& w);
void WeightedVariance(const Matrix& data,const Vector& w,Vector& var);
Real WeightedStdDev(const Vector& data,const Vector& w);
void WeightedStdDev(const Matrix& data,const Vector& w,Vector& stddev);

/*@}*/

} //namespace Statistics

#endif
