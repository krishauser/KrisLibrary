#ifndef ROBOTICS_SBL_TREE_H
#define ROBOTICS_SBL_TREE_H

#include <KrisLibrary/graph/Tree.h>
#include <KrisLibrary/geometry/GridSubdivision.h>
#include <KrisLibrary/utils/ArrayMapping.h>
#include <list>
#include "CSpace.h"
#include "EdgePlanner.h"
#include "DensityEstimator.h"
#include "Path.h"

/** @ingroup MotionPlanning
 * @brief A tree of configurations to be used in the SBL motion planner.
 */
class SBLTree
{
public:
  typedef Graph::TreeNode<Config,EdgePlannerPtr > Node;
  struct EdgeInfo
  {
    Node *s,*t;
    EdgePlannerPtr e;
    bool reversed;
  };

  SBLTree(CSpace*);
  virtual ~SBLTree();
  virtual void Cleanup();
  virtual void Init(const Config& qStart);
  virtual Node* Extend(Real maxDistance,int maxIters);

  virtual void AddMilestone(Node* n) {}
  virtual void RemoveMilestone(Node* n) {}
  virtual Node* PickExpand();

  //helpers
  Node* AddMilestone(const Config& q) { Node* n=new Node(q); AddMilestone(n); return n; }
  bool HasNode(Node* n) const;
  Node* AddChild(Node* n,const Config& x);
  Node* FindClosest(const Config& x);
  void AdjustMilestone(Node* n,const Config& newConfig);
  void DeleteSubtree(Node* n);

  //collision testing along path from ts->ns->ng->tg
  static bool CheckPath(SBLTree* ts, Node* ns,SBLTree* tg,Node* ng,std::list<EdgeInfo>& outputPath);
  //collision testing along path from ns -> ng in a single tree
  static bool CheckPath(SBLTree* t,Node* ns,Node* ng,MilestonePath& outputPath);

  CSpace* space;
  Node *root;
};

/** @ingroup MotionPlanning
 * @brief An SBLTree with a node index
 */
class SBLTreeWithIndex : public SBLTree
{
 public:
  SBLTreeWithIndex(CSpace*);
  virtual void Cleanup();  
  virtual void AddMilestone(Node* n);
  virtual void RemoveMilestone(Node* n);
  virtual Node* PickExpand() { return PickRandom(); }
  Node* PickRandom() const;

  std::vector<Node*> index;
};

/** @ingroup MotionPlanning
 * @brief An SBL motion planner that uses a SBLSubdivision to pick the
 * next node to expand, and nodes to connect.
 */
class SBLTreeWithGrid : public SBLTree
{
public:
  SBLTreeWithGrid(CSpace*);
  virtual void Init(const Config& qStart);
  virtual void Cleanup();
  ///Initializes the grids Astart,Agoal to a configuration space
  ///of numDims dimensions, uniform cell width of h
  void InitDefaultGrid(int numDims,Real h);
  ///Randomizes the dimensions of the grid divisions Astart,Agoal
  void RandomizeSubset();

  virtual void AddMilestone(Node* n);
  virtual void RemoveMilestone(Node* n);
  virtual Node* PickExpand();
  
  Node* FindNearby(const Config& x);

  Real gridDivision;
  GridDensityEstimator A;
};


#endif

