#ifndef GRAPH_UNDIRECTED_GRAPH_H
#define GRAPH_UNDIRECTED_GRAPH_H

#include "Graph.h"
#include "Callback.h"

namespace Graph {

/** @ingroup Graph
 * @brief A specialization of a Graph to be an undirected graph.
 *
 * All edges (i,j) have i<j.
 * All co-edges (i,j) have j>i.
 */
template <class Node,class Edge>
class UndirectedGraph : public Graph<Node,Edge>
{
public:
  typedef Graph<Node,Edge> P;
  typedef typename Graph<Node,Edge>::Callback Callback;
  typedef UndirectedEdgeIterator<Edge> Iterator;

  inline void Order(int& a, int& b) const { if(a>b) std::swap(a,b); }

  inline Edge& AddEdge(int i,int j) {
    Order(i,j);
    return P::AddEdge(i,j);
  }

  inline Edge& AddEdge(int i,int j,const Edge& e) {
    Order(i,j);
    return P::AddEdge(i,j,e);
  }

  inline bool HasEdge(int i,int j) const {
    Order(i,j);
    return P::HasEdge(i,j);
  }

  inline Edge* FindEdge(int i,int j) const {
    Order(i,j);
    return P::FindEdge(i,j);
  }

  inline void DeleteEdge(int i,int j) {
    Order(i,j);
    P::DeleteEdge(i,j);
  }

  inline void DeleteIncomingEdges(int i) {
    P::DeleteIncomingEdges(i);
    P::DeleteOutgoingEdges(i);
  }

  inline void DeleteNode(int n) {
    P::DeleteNode(n);
    //since the last node gets moved into n, the order of the edges
    //has changed.  need to order the edges outgoing/incoming to n
    if(n < (int)P::nodes.size()) NormalizeEdges(n);
  }

  inline void DeleteNodes(std::vector<int>& delnodes) {
    P::DeleteNodes(delnodes);
    //normalize the edges
    for(size_t i=0;i<delnodes.size();i++) 
      if(delnodes[i]>=0 && delnodes[i] < (int)P::nodes.size() && delnodes[i]!=(int)i)
	NormalizeEdges(delnodes[i]);
  }


  inline void DFS(Callback& f) { P::DFS(f,Iterator()); }
  inline void BFS(Callback& f) { P::BFS(f,Iterator()); }
  inline void SimpleDFS(Callback& f) { P::SimpleDFS(f,Iterator()); }
  inline void SimpleBFS(Callback& f) { P::SimpleBFS(f,Iterator()); }
  inline void GuidedDFS(Callback& f) { P::GuidedDFS(f,Iterator()); }
  inline void GuidedBFS(Callback& f) { P::GuidedBFS(f,Iterator()); }
    
  ///NOTE: these do not call NewTraversal()
  inline void _DFS(int node,Callback& f)
  { P::_DFS(node,f,Iterator()); }
  inline void _BFS(int node,Callback& f)
  { P::_BFS(node,f,Iterator()); }
  inline void _SimpleDFS(int node,Callback& f)
  { P::_SimpleDFS(node,f,Iterator()); }
  inline void _SimpleBFS(int node,Callback& f)
  { P::_SimpleBFS(node,f,Iterator()); }
  inline void _GuidedDFS(int node,Callback& f)
  { P::_GuidedDFS(node,f,Iterator()); }
  inline void _GuidedBFS(int node,Callback& f)
  { P::_GuidedBFS(node,f,Iterator()); }

  inline size_t Degree(int n) const { return P::edges[n].size()+P::co_edges[n].size(); }

  ///Returns true if the two nodes are connected by a path. O(n log n) time
  ///in the worst case. For better running time for connected components,
  ///especially for many queries, use ConnectedComponents in
  ///ConnectedComponents.h
  bool IsConnected(int n1,int n2);
  ///Returns the topological sort of the graph, O(n) time.
  std::list<int> TopologicalSort();
  ///Returns true if there is any cycle, O(n) time.
  bool HasCycle();
  bool IsValid() const;

  //helper that orders the edges (n,m) out of n such that n<m,
  //and incoming edges (m,n) into n such that n>m
  void NormalizeEdges(int n);
};

template <class Node,class Edge>
bool UndirectedGraph<Node,Edge>::IsConnected(const int n1,const int n2)
{
  Order(n1,n2);
  FindCallback<int> findNode(n2);
  return _SimpleDFS(n1,findNode);
}

template <class Node,class Edge>
std::list<int> UndirectedGraph<Node,Edge>::TopologicalSort()
{
  TopologicalSortCallback<int> f;
  DFS(f);
  return f.list;
}

template <class Node,class Edge>
bool UndirectedGraph<Node,Edge>::HasCycle()
{
  //TODO: this doesn't quite work because every edge has a cycle
  CycleCallback<int> f;
  DFS(f);
  return f.hasCycle;
}

template <class Node,class Edge>
bool UndirectedGraph<Node,Edge>::IsValid() const
{
  if(!P::IsValid()) return false;
  bool res=true;
  typedef typename P::ConstEdgeIterator ConstEdgeIterator;
  for(size_t i=0;i<P::edges.size();i++) {
    ConstEdgeIterator ebegin=P::edges[i].begin(),eend=P::edges[i].end();
    for(ConstEdgeIterator e=ebegin;e!=eend;e++) {
      if(e->first < (int)i) {
	fprintf(stderr,"Undirected edge (%d,%d) has i > j\n",i,e->first);
	res=false;
      }
    }
  }
  for(size_t i=0;i<P::co_edges.size();i++) {
    ConstEdgeIterator ebegin=P::co_edges[i].begin(),eend=P::co_edges[i].end();
    for(ConstEdgeIterator e=ebegin;e!=eend;e++) {
      if(e->first > (int)i) {
	fprintf(stderr,"Undirected co-edge (%d,%d) has j > i\n",e->first,i);
	res=false;
      }
    }
  }
  return res;
}

template <class Node,class Edge>
void UndirectedGraph<Node,Edge>::NormalizeEdges(int n)
{
  Assert(0 <= n && n < P::NumNodes());
  typedef typename P::EdgeIterator EdgeIterator;
  typedef typename P::CoEdgeIterator CoEdgeIterator;
  typedef typename P::EdgeDataPtr EdgeDataPtr;
  std::map<int,EdgeDataPtr> delEdges,delCoEdges;
  EdgeIterator ebegin=P::edges[n].begin(),eend=P::edges[n].end();
  for(EdgeIterator e=ebegin;e!=eend;e++) {
    int tgt=e->first;
    Assert(0 <= tgt && tgt < P::NumNodes());
    if(tgt < n) //swap the edge
      delEdges[tgt] = e->second;
  }
  CoEdgeIterator cbegin=P::co_edges[n].begin(),cend=P::co_edges[n].end();  
  for(CoEdgeIterator c=cbegin;c!=cend;c++) {
    int src=c->first;
    Assert(0 <= src && src < P::NumNodes());
    if(src > n)  //swap the edge
      delCoEdges[src] = c->second;
  }

  for(EdgeIterator e=delEdges.begin();e!=delEdges.end();e++) {
    int tgt=e->first;
    size_t num=P::edges[n].erase(tgt);  Assert(num==1);
    num=P::co_edges[tgt].erase(n);  Assert(num==1);

    P::edges[tgt][n] = e->second;
    P::co_edges[n][tgt] = e->second;
  }
  for(CoEdgeIterator c=delCoEdges.begin();c!=delCoEdges.end();c++) {
    int src=c->first;
    size_t num=P::co_edges[n].erase(src);  Assert(num==1);
    num=P::edges[src].erase(n);  Assert(num==1);

    P::co_edges[src][n] = c->second;
    P::edges[n][src] = c->second;
  }
}


} //namespace Graph

#endif
