#ifndef GRAPH_DIRECTED_GRAPH_H
#define GRAPH_DIRECTED_GRAPH_H

#include "Graph.h"
#include "Callback.h"

namespace Graph {

/** @ingroup Graph
 * @brief A specialization of a Graph to be directed graph.
 *
 * This has some nice examples of how to use callbacks to perform
 * graph computations.
 */
template <class Node,class Edge>
class DirectedGraph : public Graph<Node,Edge>
{
public:
  typedef Graph<Node,Edge> P;
  typedef EdgeIterator<Edge> ForwardIterator;
  typedef EdgeIterator<Edge> ReverseIterator;
  void DFS(CallbackBase<Node>& f) { P::DFS(f,ForwardIterator()); }
  void DFSReverse(CallbackBase<Node>& f) { P::DFS(f,ReverseIterator()); }
  void SimpleDFS(CallbackBase<Node>& f) { P::SimpleDFS(f,ForwardIterator()); }
  void SimpleBFS(CallbackBase<Node>& f) { P::SimpleBFS(f,ForwardIterator()); }
  void GuidedDFS(CallbackBase<Node>& f) { P::GuidedDFS(f,ForwardIterator()); }
  void GuidedBFS(CallbackBase<Node>& f) { P::GuidedBFS(f,ForwardIterator()); }
    
  bool HasDescendent(int n,int d);
  bool HasAncestor(int n,int a);
  std::list<int> TopologicalSort();
  bool HasCycle();
};

template <class Node,class Edge>
bool DirectedGraph<Node,Edge>::HasDescendent(const int n,const int d)
{
  P::NewTraversal();
  FindCallback<int> findNode(d);
  ForwardIterator it;
  return P::_SimpleDFS(n,findNode,it);
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
