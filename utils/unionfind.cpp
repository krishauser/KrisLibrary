#include "unionfind.h"
#include <map>

UnionFind::UnionFind(int entries)
{
    parents.resize(entries,-1);
}

void UnionFind::Initialize(const int entries)
{
	parents.clear();
	parents.resize(entries,-1);
}

void UnionFind::Resize(const int entries)
{
	parents.resize(entries,-1);
}

int UnionFind::AddEntry()
{
	parents.push_back(-1);
	return parents.size()-1;
}

int UnionFind::FindSet(const int i)
{
	int root=FindRoot(i);
	PathCompress(i,root);
	return root;
}

//unions the sets to which i and j belong
int UnionFind::Union(const int i,const int j)
{
	int root_i=FindSet(i),root_j=FindRoot(j);
	PathCompress(j,root_i);
	if(root_i!=root_j)
		parents[root_j]=root_i;
	return root_i;
}

//since on a merge, we don't go through all the children of a set
//to change their roots, we have to do some path compression
void UnionFind::CompressAll()
{
  for(int i=0;i<(int)parents.size();i++) PathCompress(i,FindRoot(i));
}

//gets the sets to which the items belong
void UnionFind::GetSets(std::vector<int>& sets)
{
  CompressAll();
  sets.resize(parents.size());
  for(int i=0;i<(int)parents.size();i++) sets[i]=FindRoot(i);
}

size_t UnionFind::CountSets() const
{
  size_t n=0;
  for(size_t i=0;i<parents.size();i++)
    if(IsRoot((int)i)) n++;
  return n;
}

void UnionFind::GetRoots(std::vector<int>& roots) const
{
  roots.resize(0);
  for(size_t i=0;i<parents.size();i++)
    if(IsRoot((int)i))
       roots.push_back((int)i);
}

void UnionFind::EnumerateSets(std::vector<std::vector<int> >& sets) const
{
  std::vector<int> roots;
  GetRoots(roots);
  std::map<int,size_t> rootMap;
  for(size_t i=0;i<roots.size();i++)
    rootMap[roots[i]] = i;
  sets.resize(roots.size());
  for(size_t i=0;i<parents.size();i++)
    sets[rootMap[FindRoot((int)i)]].push_back((int)i);
}

void UnionFind::EnumerateSet(int i,std::vector<int>& s) const
{
  int root = FindRoot(i);
  s.resize(0);
  for(size_t i=0;i<parents.size();i++)
    if(root == FindRoot(i)) s.push_back(i);
}

int UnionFind::FindRoot(const int i) const
{
  int j=i;
  while(!IsRoot(j)) { j=parents[j]; }
  return j;
}

void UnionFind::PathCompress(const int i,const int root)
{
  int j=i,k;
  while(!IsRoot(j)) { k=parents[j]; parents[j]=root; j=k;}
}
