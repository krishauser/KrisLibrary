#ifndef GRAPH_CONNECTED_COMPONENTS_H
#define GRAPH_CONNECTED_COMPONENTS_H

#include "UndirectedGraph.h"
#include <KrisLibrary/utils/unionfind.h>

namespace Graph {

class ConnectedComponents
{
 public:
  ConnectedComponents() {}
  template <class Node,class Edge>
  void Compute(const UndirectedGraph<Node,Edge>& G) {
    sets.Initialize(G.nodes.size());
    for(size_t i=0;i<G.nodes.size();i++) {
      for(const auto& e:G.edges[i]) {
	sets.Union(i,e.first);
      }
    }
  }
  void Resize(int numNodes) { sets.Initialize(numNodes); }
  void Clear() { Resize(0); }
  void AddEdge(int i,int j) { sets.Union(i,j); }
  void AddNode() { sets.AddEntry(); }
  int GetComponent(int i) { return sets.FindSet(i); }
  int GetComponent(int i) const { return sets.FindRoot(i); }
  bool SameComponent(int i,int j) { return sets.FindSet(i)==sets.FindSet(j); }
  bool SameComponent(int i,int j) const { return sets.FindRoot(i)==sets.FindRoot(j); }
  void GetRepresentatives(std::vector<int>& reps) const { sets.GetRoots(reps); }
  size_t NumComponents() const { return sets.CountSets(); }
  void EnumerateComponent(int node,std::vector<int>& items) const { return sets.EnumerateSet(node,items); }

  UnionFind sets;
};

} //namespace Graph

#endif
