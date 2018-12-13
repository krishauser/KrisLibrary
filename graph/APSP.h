#ifndef GRAPH_ALL_PAIRS_SHORTEST_PATHS_H
#define GRAPH_ALL_PAIRS_SHORTEST_PATHS_H

#include "ShortestPaths.h"
#include "Path.h"
#include "Operations.h"

namespace Graph {

const static Weight inf = Math::dInf;

template <class Node,Edge>
class SetUpdatingShortestPathProblem<Node,Edge>;

/** @ingroup Graph
 * @brief All-pairs shortest paths.

 * There are two solution methods: Floyd Warshall, or n invokations of
 * Dijkstra.  Both use O(N^2) storage and have O(k) query time where k is
 * the number of nodes in the shortest path.
 *
 * With the Floyd Warshall method, the member d contains the distance matrix
 * where d[i][j] is the distance of the shortest path from i to j
 * and next[i][j] is the index of the highest intermediate vertex on the path
 * from i to j.  Running time is O(N^3).
 *
 * With the Dijkstra method, the SPP's from each source vertex are listed
 * in the spp vector.  Running time is O(N^3 log N).
 *
 * If Dijkstra's has been used, dynamic updates to the data structures
 * can be performed.  First call InitDynamicUpdates_X(), and then call the 
 * IncreaseUpdate_X and DecreaseUpdate_X methods as needed.  For a sequence of
 * small graph changes these can save a lot of time over running the
 * shortest paths algorithm from scratch.
 */
template <class Node,class Edge>
class APSP
{
 public:
  APSP(const Graph<Node,Edge>& g);
  virtual ~APSP() {}
  void SolveFloydWarshall_Directed(WeightFunc w);
  void SolveFloydWarshall_Undirected(WeightFunc w);
  void SolveDijkstras_Directed(WeightFunc w);
  void SolveDijkstras_Undirected(WeightFunc w);
  Real GetDistance(int i,int j) const;
  bool GetPath(int i,int j,std::vector<int>& path) const;

  void InitDynamicUpdates_Directed();
  void InitDynamicUpdates_Undirected();
  void IncreaseUpdate_Directed(int u,int v,WeightFunc w);
  void DecreaseUpdate_Directed(int u,int v,WeightFunc w);
  void IncreaseUpdate_Undirected(int u,int v,WeightFunc w);
  void DecreaseUpdate_Undirected(int u,int v,WeightFunc w);

  const Graph<Node,Edge>& g;

  //Floyd Warshall
  std::vector<std::vector<Weight> > d;
  std::vector<std::vector<int> > next;

  //Dijkstra
  std::vector<std::shared_ptr<SetUpdatingShortestPathProblem<Node,Edge> > > spps;
  Graph<int,std::set<int> > edgeSets;
};

template <class Node,Edge>
class SetUpdatingShortestPathProblem<Node,Edge> : public ShortestPathProblem<Node,Edge>
{
 public:
  virtual void SetDistance(int n,Real dn,int pn)
  {
    if(!edgeSets) {
      ShortestPathProblem<Node,Edge>::SetDistance(n,dn,pn);
      return;
    }
    Real dold = d[n];
    int pold = p[n];
    ShortestPathProblem<Node,Edge>::SetDistance(n,dn,pn);
    std::set<int>* s = *edgeSets->FindEdge(pold,n);
    s->remove(index);
    s = *edgeSets->FindEdge(pn,n);
    s->insert(index);
  }

  int index;
  Graph<int,std::set<int> >* edgeSets;
};

template <class Node,Edge>
APSP<Node,Edge>::APSP(const Graph<Node,Edge>& _g)
  :g(_g)
  {}

template <class Node,Edge>
void APSP<Node,Edge>::SolveFloydWarshall_Directed(WeightFunc w)
{
  EdgeIterator<Edge> e;
  int n=ng.NumNodes();
  d.resize();
  next.resize(n);
  for(int i=0;i<n;i++) {
    d[i].resize(n);
    next[i].resize(n);
    fill(d[i].begin(),d[i].end(),inf);
    fill(next[i].begin(),next[i].end(),-1);
    for(g.Begin(i,e);!e.end();e++) {
      d[i][e.target()] = w(*e,i,e.target());
    }
    d[i][i] = 0.0;
  }
  for(int k=0;k<n;k++)
    for(int i=0;i<n;i++)
      for(int j=0;j<n;j++) {
	if(d[i][k]+d[k][j] < d[i][j]) {
	  d[i][j] = d[i][k]+d[k][j];
	  next[i][j] = k;
	}
	  }
}


template <class Node,Edge>
void APSP<Node,Edge>::SolveFloydWarshall_Undirected(WeightFunc w)
{
  UndirectedEdgeIterator<Edge> e;
  int n=ng.NumNodes();
  d.resize();
  next.resize(n);
  for(int i=0;i<n;i++) {
    d[i].resize(n);
    next[i].resize(n);
    fill(d[i].begin(),d[i].end(),inf);
    fill(next[i].begin(),next[i].end(),-1);
    for(g.Begin(i,e);!e.end();e++) {
      d[i][e.target()] = w(*e,i,e.target());
    }
    d[i][i] = 0.0;
  }
  for(int k=0;k<n;k++)
    for(int i=0;i<n;i++)
      for(int j=0;j<n;j++) {
	if(d[i][k]+d[k][j] < d[i][j]) {
	  d[i][j] = d[i][k]+d[k][j];
	  next[i][j] = k;
	}
}

template <class Node,Edge>
void APSP<Node,Edge>::SolveDijkstras_Directed(WeightFunc w)
{
  spp.resize(g.nodes.size());
  for(size_t i=0;i<g.nodes.size();i++) {
    spp[i] = new SetUpdatingShortestPathsProblem<Node,Edge>(g);
    spp[i]->index = i;
    spp[i]->edgeSets = NULL;
    spp[i]->InitializeSource(i);
    spp[i]->FindAllPaths_Directed(w);
  }
}

template <class Node,Edge>
void APSP<Node,Edge>::SolveDijkstras_Undirected(WeightFunc w)
{
  spp.resize(g.nodes.size());
  for(size_t i=0;i<g.nodes.size();i++) {
    spp[i] = new SetUpdatingShortestPathsProblem<Node,Edge>(g);
    spp[i]->index = i;
    spp[i]->edgeSets = NULL;
    spp[i]->InitializeSource(i);
    spp[i]->FindAllPaths_Undirected(w);
  }
}

template <class Node,Edge>
Real APSP<Node,Edge>::GetDistance(int i,int j) const
{
  if(!d.empty()) return d[i][j];
  else {
    Assert(!spp.empty());
    return spp[i].d[j];
  }
}

template <class Node,Edge>
bool APSP<Node,Edge>::GetPath(int i,int j,std::vector<int>& path) const
{
  if(!d.empty()) {
    path.resize(0);
    if(next[i][j] < 0) return false;
    if(i == j) {
      path.push_back(i);
      return true;
    }
    std::vector<int> p1,p2;
    GetPath(i,next[i][j],p1);
    GetPath(next[i][j],j,p2);
    path = p1;
    path.insert(path.end(),++p2.begin(),p2.end());
    return true;
  }
  else {
    Assert(!spp.empty());
    return GetAncestorPath(spp[i]->p,j,-1,path);
  }
}

template <class Node,Edge>
void APSP<Node,Edge>::InitDynamicUpdates_Directed()
{
  Assert(!spp.empty());
  //initialize edgeSets structure
  CopyStructure(g,edgeSets);
  //mark which edges are in which SPP problems
  for(size_t i=0;i<spp.size();i++) {
    spp[i]->edgeSets = &edgeSets;
    for(size_t j=0;j<spp[i]->p.size();j++)
      if(spp[i]->p[j] >= 0)
	edgeSets.FindEdge(j,spp[i]->p[j])->insert(i);
  }
}

template <class Node,Edge>
void APSP<Node,Edge>::InitDynamicUpdates_Undirected()
{
  Assert(!spp.empty());
  //add forward and reverse edges
  edgeSets.Cleanup();
  edgeSets.Resize(g.nodes.size());
  for(size_t i=0;i<g.nodes.size();i++) {
    EdgeIterator<E1> e;
    for(g.Begin(i,e);!e.end();e++) {
      edgeSets.AddEdge(i,e.target());
      edgeSets.AddEdge(e.target(),i);
    }
  }
  //mark which edges are in which SPP problems
  for(size_t i=0;i<spp.size();i++) {
    spp[i]->edgeSets = &edgeSets;
    for(size_t j=0;j<spp[i]->p.size();j++)
      if(spp[i]->p[j] >= 0)
	edgeSets.FindEdge(j,spp[i]->p[j])->insert(i);
  }
}


template <class Node,Edge>
void APSP<Node,Edge>::IncreaseUpdate_Directed(int u,int v,WeightFunc w)
{
  EdgeIterator<std::set<int> >* e=edgeSets.FindEdge(u,v);
  Assert(e!=NULL);
  for(std::set<int>::iterator i=(*e)->begin();i!=(*e)->end();i++) {
    spp[*i]->IncreaseUpdate_Directed(u,v,w);
  }
}

template <class Node,Edge>
void APSP<Node,Edge>::DecreaseUpdate_Directed(int u,int v,WeightFunc w)
{
  EdgeIterator<std::set<int> >* e=edgeSets.FindEdge(u,v);
  Assert(e!=NULL);
  for(std::set<int>::iterator i=(*e)->begin();i!=(*e)->end();i++) {
    spp[*i]->DecreaseUpdate_Directed(u,v,w);
  }
}

template <class Node,Edge>
void APSP<Node,Edge>::IncreaseUpdate_Undirected(int u,int v,WeightFunc w)
{
  EdgeIterator<std::set<int> >* e=edgeSets.FindEdge(u,v);
  Assert(e!=NULL);
  for(std::set<int>::iterator i=(*e)->begin();i!=(*e)->end();i++) {
    spp[*i]->IncreaseUpdate_Undirected(u,v,w);
  }
  e=edgeSets.FindEdge(v,u);
  Assert(e!=NULL);
  for(std::set<int>::iterator i=(*e)->begin();i!=(*e)->end();i++) {
    spp[*i]->IncreaseUpdate_Undirected(v,u,w);
  }
}

template <class Node,Edge>
void APSP<Node,Edge>::DecreaseUpdate_Undirected(int u,int v,WeightFunc w)
{
  EdgeIterator<std::set<int> >* e=edgeSets.FindEdge(u,v);
  Assert(e!=NULL);
  for(std::set<int>::iterator i=(*e)->begin();i!=(*e)->end();i++) {
    spp[*i]->IncreaseUpdate_Undirected(u,v,w);
  }
  e=edgeSets.FindEdge(v,u);
  Assert(e!=NULL);
  for(std::set<int>::iterator i=(*e)->begin();i!=(*e)->end();i++) {
    spp[*i]->IncreaseUpdate_Undirected(v,u,w);
  }
}


} //namespace Graph

#endif 
