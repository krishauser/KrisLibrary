#ifndef GEOMETRY_BallTREE_H
#define GEOMETRY_BallTREE_H

#include "KDTree.h"
#include <KrisLibrary/math/vector.h>
#include <vector>
#include <functional>
#include <memory>
using namespace Math;

namespace Geometry {

/** @ingroup Geometry
 * @brief A node of a ball-tree point location data structure
 *
 * Members:
 * - center: the center of the ball
 * - radius: the radius of the ball
 * - pts: the set of points contained within (not just for children)
 * - children: the list of child nodes
 *
 * XXXXX this note doesnt apply anymore... referencing was problematic XXXXX
 * NOTE: To avoid unnecessary copying, BallTreeNodes only store REFERENCES
 * to the Vectors that are given as input.
 * As a result you must store them in some auxiliary data structure.
 * XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 */
class BallTreeNode
{
public:
  typedef KDTree::Point Point;

  BallTreeNode();

  inline bool IsLeaf() const { return children.empty(); }
  int MaxDepth() const;
  int MinDepth() const;

  ///max/min number of points in leaves of tree
  int MaxLeafSize() const;
  int MinLeafSize() const;

  ///Number of nodes in tree
  int TreeSize() const;

  Vector center;
  Real radius;

  std::vector<Point> pts;

  BallTreeNode* parent;
  std::vector<std::unique_ptr<BallTreeNode> > children;
};

class BallTree
{
public:
  typedef std::function<double(const Vector&,const Vector&)> Metric;
  typedef std::function<void(const Vector&,const Vector&,Real,Vector&)> Interpolator;

  Metric metric;
  BallTreeNode root;
  Interpolator interpolator;
  bool cartesian;
  int splitsPerNode;

  BallTree(Metric metric,int numSplitsPerNode=2);
  BallTree(Metric metric,Interpolator interpolator,int numSplitsPerNode=2);

  ///depth statistics
  inline int MaxDepth() const { return root.MaxDepth(); }
  inline int MinDepth() const { return root.MinDepth(); }
  ///max/min number of points in leaves of tree
  inline int MaxLeafSize() const { return root.MaxLeafSize(); }
  inline int MinLeafSize() const { return root.MinLeafSize(); }

  ///Number of nodes in tree
  inline int TreeSize() const { return root.TreeSize(); }

  ///Creates the data structure with the given points and max depth
  void Build(const std::vector<Vector>& p,int maxDepth);

  ///inserts a point, splitting leaf nodes with the indicated number of points.
  ///If maxLeafPoints = -1, then this is split when 2*splitsPerNode points are in
  ///each node.
  BallTreeNode* Insert(const Vector& p,int id,int maxLeafPoints=-1);

  ///Splits the points in a leaf node.
  bool Split(BallTreeNode* node);
  ///Joins the point lists in subtrees of node so that this node becomes a leaf
  void Join(BallTreeNode* node);
  ///Re-fits the node to the points therein
  void Fit(BallTreeNode* node,bool tight);
  ///Clears the tree
  void Clear();

  ///returns the index of the closest point to pt, and its distance in dist
  int ClosestPoint(const Vector& pt,Real& dist) const;

  ///returns the index of the closest point within distance dist of pt.
  ///dist is set to the distance of the new closest point.
  ///returns -1 if there is no point within dist.
  int PointWithin(const Vector& pt,Real& dist) const;

  ///computes the set of points within the given radius
  void ClosePoints(const Vector& pt,Real radius,std::vector<Real>& distances,std::vector<int>& ids) const;

  ///returns the indices and distances of the k closest points to pt.
  ///dist and idx are assumed to point to arrays of length k.
  void KClosestPoints(const Vector& pt,int k,Real* dist,int* idx) const;

private:
  void _ClosestPoint(const BallTreeNode* node,const Vector& pt,Real& dist,int& idx) const;
  void _ClosePoints(const BallTreeNode* node,const Vector& pt,Real radius,std::vector<Real>& distances,std::vector<int>& ids) const;
  void _KClosestPoints(const BallTreeNode* node,const Vector& pt,int k,Real* dist,int* idx,int& maxdist) const;
  BallTreeNode* _LookupClosestLeaf(BallTreeNode* node,const Vector& pt,Real& dist);
};


} //namespace Geometry

#endif
