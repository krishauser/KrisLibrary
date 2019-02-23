#ifndef POINT_LOCATION_H
#define POINT_LOCATION_H

#include "CSpace.h"
//#include <KrisLibrary/geometry/Grid.h>
#include <KrisLibrary/geometry/KDTree.h>
#include <KrisLibrary/geometry/BallTree.h>
#include <memory>

/** @brief A uniform abstract interface to point location data structures.
 * The point locator operators in-place on a vector of Vectors and does not
 * allocate any extra memory unless this is desired.
 */
class PointLocationBase
{
 public:
  PointLocationBase(std::vector<Vector>& _points);
  virtual ~PointLocationBase() {}
  ///Call this when the point list is completely changed
  virtual void OnBuild() =0;
  ///Call this when something is appended to the point list
  virtual void OnAppend() =0;
  ///Call this when an index is deleted from the point list. Subclasses
  ///should return false if deletion is not supported
  virtual bool OnDelete(int id) { return false; }
  ///Call this when the point list is cleared
  virtual bool OnClear() { return false; }
  ///Subclass returns true if this is exact nearest neighbors
  virtual bool Exact() { return true; }
  ///Call this to retrieve the index of the nearest neighbor and its
  ///distance.  Subclasses should return false if NN queries are not
  ///supported.
  virtual bool NN(const Vector& p,int& nn,Real& distance) { return false; }
  ///Call this to retrieve the indices of the k-nearest neighbors and their
  ///distances.  Subclasses should return false if KNN queries are not
  ///supported.
  virtual bool KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances) { return false; }
  ///Call this to retrieve the index of the points within distance r, and
  ///their distances.  Subclasses should return false if close-neighbor 
  ///queries are not supported.
  virtual bool Close(const Vector& p,Real r,std::vector<int>& neighbors,std::vector<Real>& distances) { return false; }
  ///Same as NN, but with a filter
  virtual bool FilteredNN(const Vector& p,bool (*filter)(int),int& nn,Real& distance) { return false; }
  ///Same as KNN, but with a filter
  virtual bool FilteredKNN(const Vector& p,int k,bool (*filter)(int),std::vector<int>& nn,std::vector<Real>& distances) { return false; }
  ///Same as close, but with a filter
  virtual bool FilteredClose(const Vector& p,Real r,bool (*filter)(int),std::vector<int>& neighbors,std::vector<Real>& distances) { return false; }
  ///Returns any potentially interesting statistics
  virtual void GetStats(PropertyMap& stats) {}

  std::vector<Vector>& points;
};

/** @brief The Naive O(n) point location algorithm. */
class NaivePointLocation : public PointLocationBase
{
 public:
  NaivePointLocation(std::vector<Vector>& points,CSpace* _space);
  virtual void OnBuild() {}
  virtual void OnAppend() {}
  virtual bool OnDelete(int id) { return true; }
  virtual bool OnClear() { return true; }
  virtual bool NN(const Vector& p,int& nn,Real& distance);
  virtual bool KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool Close(const Vector& p,Real r,std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool FilteredNN(const Vector& p,bool (*filter)(int),int& nn,Real& distance);
  virtual bool FilteredKNN(const Vector& p,int k,bool (*filter)(int),std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool FilteredClose(const Vector& p,Real r,bool (*filter)(int),std::vector<int>& neighbors,std::vector<Real>& distances);


  CSpace* space;
};

/** @brief The approximate O(1) point location algorithm that simply
 * returns a random point.
 */
class RandomPointLocation : public PointLocationBase
{
 public:
  RandomPointLocation(std::vector<Vector>& points);
  virtual void OnBuild() {}
  virtual void OnAppend() {}
  virtual bool OnDelete(int id) { return true; }
  virtual bool OnClear() { return true; }
  virtual bool Exact() { return false; }
  virtual bool NN(const Vector& p,int& nn,Real& distance);
  virtual bool KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool FilteredNN(const Vector& p,bool (*filter)(int),int& nn,Real& distance);
  virtual bool FilteredKNN(const Vector& p,int k,bool (*filter)(int),std::vector<int>& nn,std::vector<Real>& distances);
};

/** @brief The approximate O(k) point location algorithm that returns the closest
 * of k randomly chosen points.
 */
class RandomBestPointLocation : public PointLocationBase
{
 public:
  RandomBestPointLocation(std::vector<Vector>& points,CSpace* _space,int k=1);
  virtual void OnBuild() {}
  virtual void OnAppend() {}
  virtual bool OnDelete(int id) { return true; }
  virtual bool OnClear() { return true; }
  virtual bool Exact() { return false; }
  virtual bool NN(const Vector& p,int& nn,Real& distance);
  virtual bool KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool FilteredNN(const Vector& p,bool (*filter)(int),int& nn,Real& distance);
  virtual bool FilteredKNN(const Vector& p,int k,bool (*filter)(int),std::vector<int>& nn,std::vector<Real>& distances);

  CSpace* space;
  int k;
};


/** @brief An accelerated point location algorithm that uses a K-D tree.
 *
 * Uses an L-n norm, optionally with weights.
 *
 * Does not support deletion.
 */
class KDTreePointLocation : public PointLocationBase
{
 public:
  KDTreePointLocation(std::vector<Vector>& points);
  KDTreePointLocation(std::vector<Vector>& points,Real norm,const Vector& weights);
  virtual void OnBuild();
  virtual void OnAppend();
  virtual bool OnClear();
  virtual bool NN(const Vector& p,int& nn,Real& distance);
  virtual bool KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool Close(const Vector& p,Real r,std::vector<int>& nn,std::vector<Real>& distances);
  virtual void GetStats(PropertyMap& stats);

  Real norm;
  Vector weights;
  std::unique_ptr<Geometry::KDTree> tree;
};

/** @brief An accelerated point location algorithm that uses a ball tree.
 *
 * Uses a geodessic norm (Distance(a,b) in the given cspace).
 *
 * Does not support deletion.
 */
class BallTreePointLocation : public PointLocationBase
{
 public:
  BallTreePointLocation(CSpace* cspace,std::vector<Vector>& points);
  virtual void OnBuild();
  virtual void OnAppend();
  virtual bool OnClear();
  virtual bool NN(const Vector& p,int& nn,Real& distance);
  virtual bool KNN(const Vector& p,int k,std::vector<int>& nn,std::vector<Real>& distances);
  virtual bool Close(const Vector& p,Real r,std::vector<int>& nn,std::vector<Real>& distances);
  virtual void GetStats(PropertyMap& stats);

  CSpace* cspace;
  std::unique_ptr<Geometry::BallTree> tree;
};

#endif
