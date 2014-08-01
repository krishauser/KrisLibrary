#ifndef ROBOTICS_CSPACE_H
#define ROBOTICS_CSPACE_H

#include <math/vector.h>
#include <math/metric.h>
#include <utils/PropertyMap.h>
using namespace Math;
typedef Vector Config;

class EdgePlanner;

/** @ingroup MotionPlanning
 * @brief Motion planning configuration space base class.
 */
class CSpace
{
public:
  virtual ~CSpace() {}
  virtual void Sample(Config& x)=0;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&)=0;
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) =0;

  ///optionally overrideable (default uses euclidean space)
  virtual Real Distance(const Config& x, const Config& y) { return Distance_L2(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);

  ///for local planners using obstacle distance
  virtual Real ObstacleDistance(const Config& a) { return Inf; }

  /** @brief Returns properties of the space that might be useful for planners.
   *
   * Typical properties may include
   * - name (string): an identifier for the space
   * - cartesian (0 or 1): whether its a euclidean space
   * - submanifold (0 or 1): whether its a submanifold space
   * - volume (real): volume of the space
   * - diameter (real): maximum distance between any two points in the space
   * - minimum (real array): minimum element of the space
   * - maximum (real array): maximum element of the space
   * - intrinsicDimension (real): for submanifolds, intrinsic dimension of
   *   space
   * - geodesic (0 or 1): whether interpolation is along geodesics
   * - metric (string): the type of metric ("euclidean", "weighted euclidean",
   *   "mahalanobis", "manhattan", "weighted manhattan", "Linf",
   *   "weighted Linf")
   * - metricWeights (real array): the metric weight vector
   * Empty values indicate that the property is unknown.
   *
   * Default implementation returns the properties of a Euclidean space.
   */
  virtual void Properties(PropertyMap&) const;
};

#endif
