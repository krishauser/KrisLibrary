#ifndef GEOMETRY_OCTREE_H
#define GEOMETRY_OCTREE_H

#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/Box3D.h>
#include <KrisLibrary/math3d/Ray3D.h>
#include <vector>
#include <list>

namespace Geometry {
  using namespace std;
  using namespace Math3D;


struct OctreeNode
{
  //child indices
  enum {LLL,LLU,LUL,LUU,ULL,ULU,UUL,UUU};

  AABB3D bb;
  int parentIndex;
  int childIndices[8];
};

class Octree
{
 public:
  ///Creates a single-node octree with the given bounding box
   explicit Octree(const AABB3D& bb);
  virtual ~Octree() {}
  ///Returns the number of nodes in the octree
  int NumNodes() const { return (int)nodes.size()-(int)freeNodes.size(); }
  ///Returns the number of nodes in the octree
  int Size() const { return NumNodes(); }
  ///Tests whether a node is a leaf
  bool IsLeaf(const OctreeNode& n) const { return n.childIndices[0] < 0; }
  ///Returns the depth of the node in the tree (root is depth 0)
  int Depth(const OctreeNode& n) const { if(n.parentIndex<0) return 0; return 1+Depth(nodes[n.parentIndex]); }
  ///Returns the index of the node 
  int Index(const OctreeNode& n) const { return int(&n - &nodes[0]); }
  ///Returns the node for a given index
  const OctreeNode& Node(int index) const { return nodes[index]; }
  ///Returns the node for a given index
  OctreeNode& Node(int index) { return nodes[index]; }
  ///Returns the maximum depth of the tree
  int MaxDepth() const;
  ///splits the Octree uniformly to the given depth d
  void SplitToDepth(int d);
  ///Splits the given node to the given depth d.  Note: n is invalidated
  ///after this call!
  void SplitToDepth(OctreeNode& n,int d);
  ///Splits the octree until the point is contained within a cell of
  ///the given depth d.  Returns the leaf node containing point. Note:
  ///n is invalidated after this call!
  OctreeNode* SplitToDepth(OctreeNode& n,const Vector3& point,int d);
  ///splits the Octree uniformly to the given resolution res
  void SplitToResolution(const Vector3& res);
  ///Splits the given node to the given resolution res. Note: n is invalidated
  ///after this call!
  void SplitToResolution(OctreeNode& n,const Vector3& res);
  ///Splits the octree until the point is contained within a cell of
  ///the given resolution res.  Returns the leaf node containing point.
  ///Note: n is invalidated after this call!
  OctreeNode* SplitToResolution(OctreeNode& n,const Vector3& point,const Vector3& res);
  ///Returns the child index in which pt is located (0-7, not the node index)
  int Child(const OctreeNode& n,const Vector3& pt) const;
  ///Returns the bounding box of a hypothetical child of n
  void Range(const OctreeNode& n,int child,AABB3D& bbchild) const;
  ///Finds the leaf node containing point or NULL if no such node exists
  OctreeNode* Lookup(const Vector3& point);
  ///Same as above, but given a root node for searching
  OctreeNode* Lookup(OctreeNode& root,const Vector3& point);
  ///Same as above, but given a root node for searching and a maximum recursion
  ///depth
  OctreeNode* Lookup(OctreeNode& root,const Vector3& point,int depthMax);
  ///Returns the indices of all leaf nodes overlapping the axis-aligned box
  ///[bmax,bmin]
  void BoxLookup(const Vector3& bmin,const Vector3& bmax,vector<int>& nodeIndices) const;
  ///Returns the indices of all leaf nodes overlapping the 3D box b
  void BoxLookup(const Box3D& b,vector<int>& nodeIndices) const;
  ///Returns the indices of all leaf nodes overlapping the sphere of radius r
  ///centered at c
  void BallLookup(const Vector3& c,Real r,vector<int>& nodeIndices) const;
  ///Returns the sorted list of leaf nodes that intersect the ray
  void RayLookup(const Ray3D& ray,vector<int>& nodeindices) const;
  ///Returns the sorted list of leaf nodes that intersect the ray fattened
  ///by the given radius 
  void FattenedRayLookup(const Ray3D& ray,Real radius,vector<int>& nodeindices) const;
  ///Splits the given node (once).
  virtual void Split(int nodeindex);
  ///Joins the given node and all descendants
  virtual void Join(int nodeindex);

 protected:
  virtual int AddNode(int parent=-1);
  virtual void DeleteNode(int);
  void _RayLookup(int nodeindex,const Ray3D& ray,vector<int>& nodeindices) const;
  void _FattenedRayLookup(int nodeindex,const Ray3D& ray,Real radius,vector<int>& nodeindices) const;

  vector<OctreeNode> nodes;
  list<int> freeNodes;
};

/** @brief Stores a point set P on an octree grid.  Allows for O(d) adding,
 * O(d h) range queries and pseudo-O(d) nearest neighbor queries.
 */
class OctreePointSet : public Octree
{
 public:
  OctreePointSet(const AABB3D& bbox,int maxPointsPerCell=1,Real minCellSize=0);
  virtual ~OctreePointSet() {}
  size_t NumPoints(int node) const;
  size_t NumPoints(const OctreeNode& node) const { return NumPoints(Index(node)); }
  void GetPoints(int node,vector<Vector3>& pts) const;
  void GetPoints(const OctreeNode& node,vector<Vector3>& pts) const { GetPoints(Index(node),pts); }
  void GetPointIDs(int node,vector<int>& ids) const;
  void GetPointIDs(const OctreeNode& node,vector<int>& ids) const { GetPointIDs(Index(node),ids); }
  void Add(const Vector3& pt,int id=-1);
  void BoxQuery(const Vector3& bmin,const Vector3& bmax,vector<Vector3>& points,vector<int>& ids) const;
  void BoxQuery(const Box3D& b,vector<Vector3>& points,vector<int>& ids) const;
  void BallQuery(const Vector3& c,Real r,vector<Vector3>& points,vector<int>& ids) const;
  ///Returns the points p for which r intersects a ball of radius radius
  ///around p
  void RayQuery(const Ray3D& r,Real radius,vector<Vector3>& points,vector<int>& ids) const;
  ///Returns the ID of the closest point for which r intersects a ball of
  ///radius radius around it
  int RayCast(const Ray3D& r,Real radius) const;
  bool NearestNeighbor(const Vector3& c,Vector3& closest,int& id) const;
  void KNearestNeighbors(const Vector3& c,int k,vector<Vector3>& closest,vector<int>& ids) const;
  ///Collapses all non-leaf nodes if the collapsed number of points per cell
  ///is less than or equal to maxSize
  void Collapse(int maxSize=0);
  ///Fits AABBs and balls to point sets.  May speed up query times.  IMPORTANT: can no
  ///longer use Lookup, Child, or Add after this is called because the octree
  ///subdivision property will no longer hold.
  void FitToPoints();
  const Sphere3D& Ball(int index) const { return balls[index]; }

 protected:
  Real _NearestNeighbor(const OctreeNode& n,const Vector3& c,Vector3& closest,int& id,Real minDist) const;
  int _KNearestNeighbors(const OctreeNode& n,const Vector3& c,vector<Vector3>& closest,vector<int>& ids,vector<Real>& distances,int kmin) const;
  virtual int AddNode(int parent=-1);
  virtual void DeleteNode(int id);
  virtual void Split(int nodeindex);
  virtual void Join(int nodeindex);

  int maxPointsPerCell;
  Real minCellSize;
  vector<vector<int> > indexLists;
  vector<Vector3> points;
  vector<int> ids;
  vector<Sphere3D> balls;
  bool fit;
};

/** @brief Stores a function f(x) on an octree grid.  Allows for O(d) setting,
 * sub O(d) testing of f(x) in range [a,b], O(d h) selection of nodes with
 * values in range [a,b]
 */
class OctreeScalarField : public Octree
{
 public:
   explicit OctreeScalarField(const AABB3D& bb,Real defaultValue = -Inf);
  virtual ~OctreeScalarField() {}
  ///Sets the value at the leaf node associated with the point pt.
  ///To set the field to a given resolution, first call SplitToDepth(pt,d) or
  ///SplitToResolution(pt,res).
  void Set(const Vector3& pt,Real value,int id=-1);
  ///Returns the value at the point
  Real Value(const Vector3& pt) const;
  ///Returns true if the value at point pt is in the range [valueMin,valueMax]
  ///(equivalent to valueMin <= Value(pt) <= valueMax but potentially faster)
  bool ValueIn(const Vector3& pt,Real valueMin,Real valueMax) const;
  ///Returns true if the value at point pt is greater than bound (equivalent to
  ///Value(pt) > bound but potentially faster)
  bool ValueGreater(const Vector3& pt,Real bound) const;
  ///Returns true if the value at point pt is less than bound
  bool ValueLess(const Vector3& pt,Real bound) const;
  ///Returns all node indices that are contained within the bbox [bmin,bmax]
  ///AND whose values are within [valueMin,valueMax] (if inclusive=true)
  ///or (valueMin,valueMax) (if inclusive=false).
  void BoxLookupRange(const Vector3& bmin,const Vector3& bmax,Real valueMin,Real valueMax,vector<int>& nodeIndices,bool inclusive=true) const;
  ///Returns all node indices that are contained within the bbox [bmin,bmax]
  ///AND whose values are > bound
  void BoxLookupGreater(const Vector3& bmin,const Vector3& bmax,Real bound,vector<int>& nodeIndices) const;
  ///Returns all node indices that are contained within the bbox [bmin,bmax]
  ///AND whose values are < bound
  void BoxLookupLess(const Vector3& bmin,const Vector3& bmax,Real bound,vector<int>& nodeIndices) const;
  ///Collapses all nodes whose difference between min/max is less than
  ///tolerance
  void Collapse(Real tolerance=0);

 protected:
  struct Data {
    ///The value, at a leaf, or the average value of leaves, if it's non-leaf
    Real value;
    ///The min / max value
    Real valueMin,valueMax;
    ///An ID
    int id;
  };

  virtual int AddNode(int parent=-1);
  virtual void DeleteNode(int id);
  virtual void Split(int nodeindex);
  virtual void Join(int nodeindex);

  Real defaultValue;
  vector<Data> data;
};

} //namespcae Geometry

#endif
