#ifndef MATH_SAMPLE_H
#define MATH_SAMPLE_H

#include <vector>

/** @file math/sample.h
 * @brief Functions for random sampling of various sets.
 */

namespace Math {
/** @addtogroup Math */
/*@{*/
  struct Interval;
  struct ClosedIntervalSet;

/** @brief Samples an integer with weighted probability.
 *
 * The probability of sampling integer i in [0,n) is wi / W
 * where W = sum of all wi.
 */
int WeightedSample(const std::vector<Real>& weights);
/// Same as above, but the sum of all weights, W, is provided.
int WeightedSample(const std::vector<Real>& weights,Real totWeight);
/// Same as above, but the array stores cumulative weights (i.e. partial sums)
/// O(log n) algorithm
int CumulativeWeightedSample(const std::vector<Real>& partialSumWeights);

/** @brief Allocates total samples within the buckets in num, roughly
 * evenly.  An O(n) algorithm where n is the number of buckets.
 *
 * Specifically, this allocates at least floor(total/n) samples to each bucket,
 * then distributes the remaining total-n*floor(total/n) such that no bucket
 * receives more than 1 extra sample.
 */
void RandomAllocate(std::vector<int>& num,size_t total);
/// Same as above, but with weights.
void RandomAllocate(std::vector<int>& num,size_t total,const std::vector<Real>& weights);

/// Uniformly samples the given intervals
Real Sample(const Interval& s);
/// Uniformly samples the interval set
Real Sample(const ClosedIntervalSet& s);

/// Uniform distribution on boundary of a circle with radius r
void SampleCircle(Real r, Real& x, Real& y);
/// Uniform distribution inside a circle with radius r
void SampleDisk(Real r, Real& x, Real& y);

/// Uniform distribution in the triangle whose vertices are (0,0),(1,0),(0,1)
void SampleTriangle(Real& x, Real& y);

/// Uniform distribution on boundary of a sphere with radius r
void SampleBall(Real r, Real& x, Real& y, Real& z);
/// Uniform distribution inside a sphere with radius r
void SampleSphere(Real r, Real& x, Real& y, Real& z);

/// Uniformly samples from a hyper-ball of dimension v.size()
void SampleHyperBall(Real r,std::vector<Real>& v);
/// Uniformly samples from a hyper-sphere of dimension v.size()
void SampleHyperSphere(Real r,std::vector<Real>& v);

} //namespace Math
/*@}*/

#endif
