#ifndef PLANNING_DENSITY_ESTIMATOR_H
#define PLANNING_DENSITY_ESTIMATOR_H

#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/geometry/GridSubdivision.h>
#include "PointLocation.h"

/** @ingroup MotionPlanning
 * @brief A base class for density estimators.
 * Each point in an N dimensional space is attached to some data
 * pointer.  Density in the space is measured by the Density()
 * function, and only matters in a relative sense.
 *
 * Subclasses may also overload Random() and RandomNear() to sample
 * an existing point
 */
class DensityEstimatorBase 
{
public:
  virtual ~DensityEstimatorBase () {}
  virtual void Clear()=0;
  virtual void Add(const Math::Vector& x,void* data)=0;
  virtual void Remove(const Math::Vector& x,void* data)=0;
  virtual double Density(const Config& x)=0;
  virtual void* RandomNear(const Math::Vector& x) { return NULL; }
  virtual void* Random() { return NULL; }
  virtual void Random(Math::Vector& x) { }
};

/** @ingroup MotionPlanning
 * @brief A grid-based (n-d projected histogram) density estimator.
 *
 * The grid operates on certain dimensions of the configuration.
 * Specifically, it picks mappedDims dimensions at random from
 * the full configuration space, and divides the space in those
 * dimensions into cells of width h. 
 *
 * Due to data fragmentation, the number of mapped dimensions
 * should be relatively small. <= 3 is usually a good choice.
 *
 * Note: Randomize will destroy the contents of the estimator.
 */
class GridDensityEstimator : public DensityEstimatorBase
{
public:
  GridDensityEstimator();
  GridDensityEstimator(const std::vector<int>& mappedDims,Math::Real h);
  GridDensityEstimator(const std::vector<int>& mappedDims,const Math::Vector& h);
  void Randomize(int numSourceDims,int numMapped,const Vector& hsource);
  void Randomize(int numSourceDims,int numMapped,Real h);
  virtual void Clear();
  virtual void Add(const Math::Vector& x,void* data);
  virtual void Remove(const Math::Vector& x,void* data);
  virtual double Density(const Config& x);
  virtual void* RandomNear(const Math::Vector& x);
  virtual void* Random();
  
  std::vector<int> mappedDims;
  Math::Vector h;
  Geometry::GridSubdivision subdiv;

  //temporary
  Math::Vector temp;
  std::vector<Geometry::GridSubdivision::ObjectSet*> flattenedBuckets;
};

/** @ingroup MotionPlanning
 * @brief A multiple-grid based density estimator that
 * attempts to avoid data fragmentation by collecting histograms
 * on multiple axes and fusing them together.
 *
 * Note: Randomize will destroy the contents of the estimator.
 */
class MultiGridDensityEstimator : public DensityEstimatorBase
{
public:
  MultiGridDensityEstimator(int numDims,int numMappedDims,Real h);
  MultiGridDensityEstimator(int numDims,int numMappedDims,const Math::Vector& h);
  void Randomize();
  virtual void Clear();
  virtual void Add(const Math::Vector& x,void* data);
  virtual void Remove(const Math::Vector& x,void* data);
  virtual double Density(const Config& x);
  virtual void* RandomNear(const Math::Vector& x);
  virtual void* Random();

  int numDims,numMappedDims;
  Math::Vector h;
  std::vector<GridDensityEstimator> grids;
};

/** @ingroup MotionPlanning
 * @brief a Kernel Density Estimation density estimator.
 */
class KernelDensityEstimator : public DensityEstimatorBase
{
public:
  enum KernelType { KernelUniform, KernelGaussian, KernelTriangular};
  KernelDensityEstimator(Math::Real kernelRadius,Math::Real kernelTruncationFactor=3.0);
  virtual void Clear();
  virtual void Add(const Math::Vector& x,void* data);
  virtual void Remove(const Math::Vector& x,void* data);
  virtual double Density(const Config& x);
  virtual void* RandomNear(const Math::Vector& x);
  virtual void* Random();
  virtual void Random(Math::Vector& x);
  
  KernelType kernelType;
  Math::Real kernelRadius,kernelTruncationFactor;
  std::vector<Math::Vector> pointList;
  std::vector<void*> dataList;
  std::shared_ptr<PointLocationBase> pointLocation;
};


#endif