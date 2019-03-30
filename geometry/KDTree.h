#ifndef GEOMETRY_KDTREE_H
#define GEOMETRY_KDTREE_H

#include <KrisLibrary/math/vector.h>
#include <vector>
using namespace Math;

namespace Geometry {

/** @ingroup Geometry
 * @brief A kd-tree or a node of one.
 *
 * A whole kd-tree is created from the return value of KDTree::Create().
 * At the end of its life, delete it.
 *
 * Members:
 * - splitDim: the split dimension
 * - splitVal: the split value
 * - pos: the node on the positive side of val (that is, the node storing
 *   all points x with x(d)>val)
 * - neg: the node on the negative side of val
 * - pts: if this is a leaf, the set of points contained within
 *
 * XXXXX this note doesnt apply anymore... referencing was problematic XXXXX
 * NOTE: To avoid unnecessary copying, kd-trees only store REFERENCES
 * to the Vectors that are given as input.
 * As a result you must store them in some auxiliary data structure.
 * XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 */
class KDTree
{
 public:
  struct Point {
    Point();
    Point(const Point& pt);
    const Point& operator = (const Point&);
    Vector pt;
    int id;
  };

  ///Creates a kdtree with the given points, dimension k, and max depth
  static KDTree* Create(const std::vector<Vector>& p, int k, int maxDepth);

  KDTree();
  KDTree(const std::vector<Point>& pts, int k, int depth=0, int maxDepth=100);
  ~KDTree();

  ///can't use copy constructor or assignment operation
  KDTree(const KDTree&) = delete;
  const KDTree& operator=(const KDTree&) const = delete;

  inline bool IsLeaf() const { return splitDim==-1; }
  int MaxDepth() const;
  int MinDepth() const;

  ///max/min number of points in leaves of tree
  int MaxLeafSize() const;
  int MinLeafSize() const;

  ///Number of nodes in tree
  int TreeSize() const;

  ///inserts a point, splitting leaf nodes with the indicated number of points
  KDTree* Insert(const Vector& p,int id,int maxLeafPoints=2);
  ///finds the kdtree leaf in which this point is located
  KDTree* Locate(const Vector& p);

  ///Splits the points along the given dimension, returns true if there's 
  ///actually a split.  If dimension is left default, one is picked at random
  bool Split(int dimension=-1);
  ///Joints the point lists in two subtrees so that this node becomes a leaf
  void Join();
  ///Clears the tree
  void Clear();

  ///returns the index of the closest point to pt, and its distance in dist
  int ClosestPoint(const Vector& pt,Real& dist) const;
  ///same, but uses the L-n norm with optional weights w
  int ClosestPoint(const Vector& pt,Real n,const Vector& w,Real& dist) const;

  ///returns the index of the closest point within distance dist of pt.
  ///dist is set to the distance of the new closest point.
  ///returns -1 if there is no point within dist.
  int PointWithin(const Vector& pt,Real& dist) const;
  ///computes the set of points within the given radius
  void ClosePoints(const Vector& pt,Real radius,std::vector<Real>& distances,std::vector<int>& ids) const;
  ///same, but uses the L-n norm with weights w
  void ClosePoints(const Vector& pt,Real radius,Real n,const Vector& w,std::vector<Real>& distances,std::vector<int>& ids) const;

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

  int depth;
  int splitDim;
  Real splitVal;
  KDTree *pos,*neg;
  std::vector<Point> pts;

  //temporary -- used for performance testing
  int visits;
};

} //namespace Geometry

#endif
