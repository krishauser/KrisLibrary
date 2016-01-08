#ifndef STAT_KMEANS_H
#define STAT_KMEANS_H

#include <KrisLibrary/math/vector.h>
#include <vector>

namespace Statistics {
  using namespace Math;

/** @ingroup Statistics
 * @brief A simple clustering method to choose k clusters from a set of data.
 *
 * The cluster centers are 
 */
class KMeans
{
 public:
  KMeans(const std::vector<Vector>& data);
  KMeans(const std::vector<Vector>& data,int k);
  virtual ~KMeans() {}
  int GetK() const { return (int)centers.size(); }
  void SetK(int k);
  ///Initialization 
  void RandomInitialCenters();
  void ClearLabels();
  ///Returns in maxIters the number of used iterations before convergence
  void Iterate(int& maxIters);

  //Individual steps of the iteration
  ///Returns true if any label has changed
  bool CalcLabelsFromCenters();
  ///Sets the centers from the data points in a center's group
  void CalcCentersFromLabels();

  ///Returns the average distance of points for the given cluster
  Real AverageDistance(int c);
  ///Same as above, but for all clusters
  void AverageDistance(std::vector<Real>& dist);

  ///Overrideable: distance metric
  virtual Real Distance(const Vector& a,const Vector& b) 
  { return a.distance(b); }

  const std::vector<Vector>& data;
  const std::vector<Real>* weights;
  std::vector<int> labels;
  std::vector<Vector> centers;
};

}

#endif
