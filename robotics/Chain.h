#ifndef ROBOTICS_CHAIN_H
#define ROBOTICS_CHAIN_H

#include <vector>

/** @defgroup Kinematics
 * @brief Classes to define robot kinematics.
 */

/** @ingroup Kinematics
 * @brief A tree-based chain structure.
 *
 * Represents the tree as a topologically sorted list of parents. 
 * -1 represents no parent.
 */
class Chain
{
public:
  enum { NoParent = -1 };
  
  bool HasValidOrdering() const;
  /// Returns true if p is an ancestor of n
  bool IsAncestor(int n, int p) const;
  /// Returns true if c is a descendant of n
  inline bool IsDescendent(int n, int c) const { return IsAncestor(c,n);}
  /// Least common ancestor
  int LCA(int a,int b) const;
  /// Returns a vector where element i is a vectors of the children of link i
  void GetChildList(std::vector<std::vector<int> >& children) const;
  /// Returns a vector where element i is true if it is an ancestor of n
  void GetAncestors(int k,std::vector<bool>& ancestors) const;
  /// Returns a vector where element i is true if it is an descendant of n
  void GetDescendants(int k,std::vector<bool>& descendants) const;

  /// Topologically sorted list of parents
  std::vector<int> parents;
};

#endif
