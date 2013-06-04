#ifndef ROBOTICS_CSPACE_H
#define ROBOTICS_CSPACE_H

#include <math/vector.h>
#include <math/metric.h>
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
};

#endif
