#ifndef GRAPH_DIRECTED_GRAPH_H
#define GRAPH_DIRECTED_GRAPH_H

#include "Graph.h"

namespace Graph {

/** @ingroup Graph
 * @brief A specialization of a Graph to be directed graph.
 *
 * This has some nice examples of how to use callbacks to perform
 * graph computations.
 */
class DirectedGraph : public Graph<Node,Edge>
{
public:
  typedef typename Graph<Node,Edge> P;
  typedef EdgeIterator<Edge> ForwardIterator;
  typedef ReverseEdgeIterator<Edge> ReverseIterator;
  void DFS(Callback& f) { P::DFS<ForwardIterator>(f); }
  void DFSReverse(Callback& f) { P::DFS<ReverseIterator>(f); }
  void SimpleDFS(Callback& f) { P::SimpleDFS<ForwardIterator>(f); }
  void SimpleBFS(Callback& f) { P::SimpleBFS<ForwardIterator>(f); }
  void GuidedDFS(Callback& f) { P::GuidedDFS<ForwardIterator>(f); }
  void GuidedBFS(Callback& f) { P::GuidedBFS<ForwardIterator>(f); }
    
  bool HasDescendent(int n,int d);
  bool HasAncestor(int n,int a);
  std::list<int> TopologicalSort();
  bool HasCycle();
};

template <class Node,class Edge>
bool DirectedGraph<Node,Edge>::HasDescendent(const int n,const int d)
{
  NewTraversal();
  FindCallback<int> findNode;
  findNode.node = d;
  ForwardIterator it;
  return _SimpleDFS(n,findNode,it);
}

template <class Node,class Edge>
bool DirectedGraph<Node,Edge>::HasAncestor(const int n,const int a)
{
  return HasDescendent(a,n);
}

template <class Node,class Edge>
std::list<int> DirectedGraph<Node,Edge>::TopologicalSort()
{
  TopologicalSortCallback<int> f;
  DFS(f);
  return f.list;
}

template <class Node,class Edge>
bool DirectedGraph<Node,Edge>::HasCycle()
{
  CycleCallback<int> f;
  DFS(f);
  return f.hasCycle;
}


} //namespace Graph

#endif
