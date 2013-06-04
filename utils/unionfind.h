#ifndef UNION_FIND_H
#define UNION_FIND_H

#include <vector>
#include <stdlib.h>

/** @ingroup Utils
 * @brief From an indexed set of elments, allows fast unioning 
 * of the elements into sets.
 *
 * From a set X={x1,x2,...} (such that |X| = entries), the sets
 * are initialized to Si = {xi}.  Larger sets are created by
 * unioning the set of xi with that of xj using the Union() method.
 *
 * The sets are referenced using unique integer identifiers
 * from 0 to entries-1.
 */
class UnionFind
{
public:
  UnionFind(int entries=0);
  ///Resize X to size entries, sets all sets Si={xi}
  void Initialize(const int entries);
  ///Resize X to size entries, sets all sets Si={xi}
  void Resize(const int entries);
  ///Increment the size of X by 1, sets Sn={xn}
  int AddEntry();
  ///Returns the id of the set to which xi belongs 
  int FindSet(const int i);  
  ///Unions the sets to which xi and xj belong
  int Union(const int i,const int j);
  ///Returns the sets to which the items belong, such that sets[i] = set(xi)
  void GetSets(std::vector<int>& sets);
  ///Counts the number of sets
  size_t CountSets() const;
  ///Returns the roots of all sets
  void GetRoots(std::vector<int>& roots) const;
  ///Returns the sets, by listing all items for each set
  void EnumerateSets(std::vector<std::vector<int> >& sets) const;
  ///Returns the entire set for item i
  void EnumerateSet(int i,std::vector<int>& s) const;

  //low level routines
  inline bool IsRoot(const int i) const { return parents[i]==-1; }
  int FindRoot(const int i) const;

private:
  std::vector<int> parents;
  void PathCompress(const int i,const int root);
  void CompressAll();
};
#endif

