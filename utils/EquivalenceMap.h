#ifndef UTILS_EQUIVALENCE_MAP_H
#define UTILS_EQUIVALENCE_MAP_H

#include "unionfind.h"
#include <KrisLibrary/errors.h>
#include <map>

/** @ingroup Utils
 * @brief For a mapping f:i->fwd[i], this gets the inverse multi-mapping
 * g:j->{i|fwd[i]=j}
 */
inline void InverseMapping(const std::vector<int>& fwd,std::vector<vector<int> >& bwd) {
  bwd.clear();
  bwd.reserve(fwd.size());
  if(fwd.empty()) return;
  int N=*max_element(fwd.begin(),fwd.end());
  bwd.resize(N+1);
  for(size_t i=0;i<fwd.size();i++) {
    Assert(0<=fwd[i]&&fwd[i]<=N);
    bwd[fwd[i]].push_back(i);
  }
  size_t n=0;
  for(size_t i=0;i<bwd.size();i++) {
    n+=bwd[i].size();
  }
  if(n != fwd.size()) {
    FatalError("InverseMapping error! inserted only %d of %d items",n,fwd.size());
  }
}

/** @ingroup Utils
 * @brief Forms the equivalence map on x, given an equivalence function.
 *
 * Forms a graph where an edge (i,j) exists if x[i]=x[j] for some i,j.
 * Upon exit, each entry eq[k] contains a connected component of the graph.
 *
 * The algorithm makes at most n(n+1)/2 comparisons.
 */
template <class T,class EqFn>
void EquivalenceMap(const std::vector<T>& x,std::vector<vector<int> >& eq,EqFn& Eq)
{
  int n=(int)x.size();
  UnionFind uf(n);
  for(int i=0;i<n;i++) {
    int seti=i;
    for(int j=0;j<i;j++) {
      int setj = uf.FindSet(j);
      if(seti != setj) {
	if(Eq(x[i],x[j])) {
	  //merge i->j
	  uf.Union(j,i);
	  Assert(uf.FindSet(j) == setj);
	  Assert(uf.FindSet(i) == setj);
	  seti = setj;
	}
      }
    }
  }
  vector<int> sets;
  uf.GetSets(sets);
  InverseMapping(sets,eq);
  //remove empty sets
  vector<vector<int> >::iterator it,prev;
  int i=0;
  int num=0;
  for(it=eq.begin();it!=eq.end();it++,i++) {
    num += (int)it->size();
    if(it->empty()) {
      prev = it; prev--;
      eq.erase(it);
      it = prev;
    }
  }
  Assert(num==n);
}


#endif
