#ifndef GEOMETRY_KDTREE_H
#define GEOMETRY_KDTREE_H

#include <math/vector.h>
#include <vector>
using namespace Math;

namespace Geometry {

/** @ingroup Geometry
 * @brief A node of a kd-tree.
 *
 * A whole kd-tree is created from the return value of KDTree::Create().
 * At the end of its life, delete it.
 *
 * Members:
 * - dim: the split dimension
 * - val: the split value
 * - pos: the node on the positive side of val (that is, the node storing
 *   all points x with x(d)>val)
 * -neg: the node on the negative side of val
 * -pts: if this is a leaf, the set of points contained within
 */
class KDTree
{
 public:
  struct Point
  {
    inline operator const Vector& () const { return *pt; }
    const Vector* pt;
    int index;
  };

  ///Creates a kdtree with the given points, dimension k, and max depth
  static KDTree* Create(const std::vector<Vector>& p, int k, int depth);

  KDTree(std::vector<Point>& p, int k, int depth, int _d=0);

  inline bool IsLeaf() const { return dim==-1; }
  int MaxDepth() const;
  int MinDepth() const;

  ///max/min number of points in leaves of tree
  int MaxLeafSize() const;
  int MinLeafSize() const;

  ///finds the kdtree leaf in which this point is located
  KDTree* Locate(const Vector& p);
  ///The node must be a leaf
  bool Remove(int i);

  ///returns the index of the closest point to pt, and its distance in dist
  int ClosestPoint(const Vector& pt,Real& dist) const;
  ///same, but uses the L-n norm with optional weights w
  int ClosestPoint(const Vector& pt,Real n,const Vector& w,Real& dist) const;

  ///returns the index of the closest point within distance dist of pt.
  ///dist is set to the distance of the new closest point.
  ///returns -1 if there is no point within dist.
  int PointWithin(const Vector& pt,Real& dist) const;

  ///returns the indices and distances of the k closest points to pt.
  ///dist and idx are assumed to point to arrays of length k.
  void KClosestPoints(const Vector& pt,int k,Real* dist,int* idx) const;
  ///same, but uses the L-n norm with weights w
  void KClosestPoints(const Vector& pt,int k,Real n,const Vector& w,Real* dist,int* idx) const;

  ///gives a split value (in dim d) that gives the k'th value in the list
  static Real Select(const std::vector<Point>& S, int d, int k);

private:
  void _ClosestPoint(const Vector& pt,Real& dist,int& idx) const;
  void _KClosestPoints(const Vector& pt,int k,Real* dist,int* idx,int& maxdist) const;
  void _ClosestPoint2(const Vector& pt,Real& dist,int& idx,Real norm,const Vector& weights) const;
  void _KClosestPoints2(const Vector& pt,int k,Real* dist,int* idx,int& maxdist,Real norm,const Vector& weights) const;

  ///can't use copy constructor or assignment operation
  KDTree(const KDTree&) {}
  const KDTree& operator=(const KDTree&) const { return *this; }

  int dim;
  Real val;
  KDTree *pos,*neg;
  std::vector<Point> pts;
};

} //namespace Geometry

#endif
