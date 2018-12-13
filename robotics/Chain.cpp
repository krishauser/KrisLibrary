#include "Chain.h"
#include <set>
using namespace std;

bool Chain::HasValidOrdering() const
{
  int size = parents.size();
  for(int i=0;i<size;i++)
    if(parents[i]>=i) return false;
  return true;
}

bool Chain::IsAncestor(int n, int p) const 
{
	while(n != -1) {
		if(n==p) return true;
		n = parents[n];
	}
	return false;
}

int Chain::LCA(int a,int b) const
{
  set<int> pa;
  while(a != -1) {
    pa.insert(a);
    a = parents[a];
  }
  while(b != -1) {
    if(pa.count(b)!=0) return b;
    b = parents[b];
  }
  return -1;
}

void Chain::GetChildList(std::vector<std::vector<int> >& children) const
{
  int size = (int)parents.size();
  children.resize(parents.size());
  for(int i=0;i<size;i++) children[i].clear();
  for(int i=0;i<size;i++) {
    int p=parents[i];
    if(p>=0)
      children[p].push_back(i);
  }
}

void Chain::GetAncestors(int z,std::vector<bool>& ancestors) const
{
  ancestors.resize(parents.size());
  std::fill(ancestors.begin(),ancestors.end(),false);
  while(z>=0) {
    ancestors[z]=true;
    z = parents[z];
  }
}

void Chain::GetDescendants(int z,std::vector<bool>& descendants) const
{
  descendants.resize(parents.size());
  std::fill(descendants.begin(),descendants.end(),false);
  descendants[z] = true;
  //since topologically sorted, no descendants of z are less than z
  for(size_t k=size_t(z);k<parents.size();k++) 
    if(parents[k]>=0 && descendants[parents[k]])
      descendants[k]=true;
}
