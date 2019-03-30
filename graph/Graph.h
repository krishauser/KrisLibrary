#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <KrisLibrary/Logger.h>
#include "Callback.h"
#include "Node.h"
#include "Edge.h"
#include <KrisLibrary/errors.h>
#include <vector>
#include <queue>
#include <iostream>

/** @defgroup Graph
 * @brief Template classes for general graph computations.
 */

/** @addtogroup Graph */
/*@{*/

///Namespace for all classes and functions in the Graph subdirectory
namespace Graph {

/** @brief Basic template graph structure.
 *
 * The graph is represented with nodes indexed by 0,...,NumNodes()-1, and
 * for each node, an adjacency list of outgoing edges 'edges' and incoming
 * edges 'co_edges'.
 *
 * A NodeData is stored for each node in a vector 'nodes'.
 * A EdgeData is stored for each edge in a list 'edgeData'.
 * An adjacency list in 'edges' or 'co_edges' is represented as 
 * a map from adjacent node indices to EdgeData pointers.
 *
 * Edges are most easily traversed using the EdgeListIterator classes in
 * Edge.h.  They allow forwards, backwards, or bidirectional traversals.
 * For more strict definition of directed vs. undirected graphs, consider
 * DirectedGraph or UndirectedGraph.
 *
 * NodeData must have valid constructors and assignment operators.
 * EdgeData must have a default constructor.
 *
 * Basic traversal functions (DFS,BFS) are provided.  More sophisticated
 * computations should be written in as algorithm classes 
 * (see ShortestPathProblem).
 */
template <class NodeData, class EdgeData>
class Graph
{
public:
  typedef CallbackBase<int> Callback;

  Graph() {}
  ~Graph() { Cleanup(); }

  void Resize(int n);
  void Cleanup();
  bool IsValid() const;
  void Copy(const Graph<NodeData,EdgeData>& g);
  void SetTranspose(const Graph<NodeData,EdgeData>& g);

  int NumNodes() const { return (int)nodes.size(); }

  /// Adds a new node to the node list and returns its index
  int AddNode(const NodeData&);

  /// @brief Deletes node n.
  ///
  /// Does so by moving the last node to position n.   This
  /// modifies the node index of the last node from NumNodes()-1 to n.
  void DeleteNode(int n);

  /// @brief Deletes several nodes at once.
  ///
  /// Takes a vector of nodes to delete, deletes them, and int
  /// the same vector returns the mapping of altered nodes.
  /// set to -1 if deleted, or the new index otherwise.
  void DeleteNodes(std::vector<int>& delnodes);

  int NumEdges() const { return (int)edgeData.size(); }
  EdgeData& AddEdge(int i,int j);
  EdgeData& AddEdge(int i,int j,const EdgeData&);
  bool HasEdge(int i,int j) const;
  EdgeData* FindEdge(int i,int j) const;
  void DeleteEdge(int i,int j);
  void DeleteOutgoingEdges(int i);
  void DeleteIncomingEdges(int i);
  inline size_t OutDegree(int n) const { return edges[n].size(); }
  inline size_t InDegree(int n) const { return co_edges[n].size(); }

  template <class Iterator>
  void Begin(int n,Iterator&) const;

  /// Call NewTraversal to erase history of previous searches by 
  /// setting the node colors to white
  ///
  /// Full-blown D[B]FS calls all callback functions. <br>
  /// SimpleD[B]fs ignores the descend/edge functions. <br>
  /// GuidedD[B]fs uses the Descend function rather than the color to
  ///   determine if a node should be visited.  Be careful with this
  ///   to avoid infinite loops.
  ///
  /// NOTE: only outgoing, reverse, or undirected iteration methods
  /// should be used.
  void NewTraversal();
  template <class Iterator> void DFS(Callback&,Iterator);
  template <class Iterator> void BFS(Callback&,Iterator);
  template <class Iterator> void SimpleDFS(Callback&,Iterator);
  template <class Iterator> void SimpleBFS(Callback&,Iterator);  
  template <class Iterator> void GuidedDFS(Callback&,Iterator);
  template <class Iterator> void GuidedBFS(Callback&,Iterator);  

  /// The following perform a traversal from a given node.
  /// They do not call NewTraversal().
  template <class Iterator> void _DFS(int node,Callback& f,Iterator);
  template <class Iterator> void _BFS(int node,Callback& f,Iterator);
  template <class Iterator> void _SimpleDFS(int node,Callback& f,Iterator);
  template <class Iterator> void _SimpleBFS(int node,Callback& f,Iterator);
  template <class Iterator> void _GuidedDFS(int node,Callback& f,Iterator);
  template <class Iterator> void _GuidedBFS(int node,Callback& f,Iterator);
  
  void WriteDOT(std::ostream& out);

  typedef typename std::list<EdgeData>::iterator EdgeDataPtr;
  typedef std::map<int,EdgeDataPtr> EdgeList;
  typedef std::map<int,EdgeDataPtr> CoEdgeList;
  typedef typename EdgeList::iterator EdgeListIterator;
  typedef typename CoEdgeList::iterator CoEdgeListIterator;
  typedef typename EdgeList::const_iterator ConstEdgeListIterator;
  std::vector<Color> nodeColor;
  std::vector<NodeData> nodes;
  std::vector<EdgeList> edges;
  std::vector<CoEdgeList> co_edges;
  std::list<EdgeData> edgeData;
};

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::Resize(int n)
{
  nodeColor.resize(n,White);
  nodes.resize(n);
  edges.resize(n);
  co_edges.resize(n);
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::Copy(const Graph<NodeData,EdgeData>& g)
{
  nodeColor = g.nodeColor;
  nodes = g.nodes;
  edgeData.clear();
  edges.resize(g.edges.size());
  co_edges.resize(g.co_edges.size());
  for(size_t i=0;i<edges.size();i++) edges[i].clear();
  for(size_t i=0;i<co_edges.size();i++) co_edges[i].clear();
  for(size_t i=0;i<edges.size();i++) {
    for(ConstEdgeListIterator e=g.edges[i].begin();e!=g.edges[i].end();e++)
      AddEdge(i,e->first,e->second);
  }
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::SetTranspose(const Graph<NodeData,EdgeData>& g)
{
  nodeColor = g.nodeColor;
  nodes = g.nodes;
  edgeData.clear();
  edges.resize(g.edges.size());
  co_edges.resize(g.co_edges.size());
  for(size_t i=0;i<edges.size();i++) edges[i].clear();
  for(size_t i=0;i<co_edges.size();i++) co_edges[i].clear();
  for(size_t i=0;i<edges.size();i++) {
    for(ConstEdgeListIterator e=g.edges[i].begin();e!=g.edges[i].end();e++)
      AddEdge(e->first,i,e->second);
  }
}


template <class NodeData,class EdgeData>
int Graph<NodeData,EdgeData>::AddNode(const NodeData& n)
{
  nodeColor.push_back(White);
  nodes.push_back(n);
  edges.push_back(EdgeList());
  co_edges.push_back(CoEdgeList());
  return int(nodes.size())-1;
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::DeleteNode(int n)
{
  Assert(n >= 0 && n < (int)nodes.size());
  DeleteOutgoingEdges(n);
  DeleteIncomingEdges(n);

  int rep = (int)nodes.size()-1;
  //move all references from rep to n
  nodeColor[n] = nodeColor[rep];  nodeColor.resize(nodeColor.size()-1);
  nodes[n] = nodes[rep];  nodes.resize(nodes.size()-1);
  //here we must move outgoing edges of rep to point to n
  EdgeListIterator ebegin=edges[rep].begin(),eend=edges[rep].end();
  for(EdgeListIterator e=ebegin;e!=eend;e++) {
    int t = e->first;
    CoEdgeListIterator crep=co_edges[t].find(rep);
    Assert(crep->second == e->second);
    co_edges[t].erase(crep);
    co_edges[t][n] = e->second;
  }
  edges[n] = edges[rep];  edges.erase(--edges.end());

  //here we must move incoming edges of rep to point to n
  CoEdgeListIterator cbegin=co_edges[rep].begin(),cend=co_edges[rep].end();
  for(CoEdgeListIterator e=cbegin;e!=cend;e++) {
    int s = e->first;
    EdgeListIterator erep=edges[s].find(rep);
    Assert(erep->second == e->second);
    edges[s].erase(erep);
    edges[s][n] = e->second;
  }
  co_edges[n] = co_edges[rep]; co_edges.erase(--co_edges.end());
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::DeleteNodes(std::vector<int>& delnodes)
{
  //get the mapping from old to new nodes
  std::vector<int> nodeMap(nodes.size(),-2);
  std::vector<int> srcMap(nodes.size(),-1);

  int last=(int)nodes.size()-1;
  for(std::vector<int>::const_iterator i=delnodes.begin();i!=delnodes.end();i++) {
    int n=*i;
    int n_orig=n;
    Assert(nodeMap[n] != -1);
    //n may have been moved around, follow it until it stops
    while(nodeMap[n]>=0)
      n=nodeMap[n];

    int src = last;
    while(srcMap[src]>=0)
      src=srcMap[src];
    //move src to position n
    nodeMap[src] = n;
    nodeMap[n_orig] = -1;
    srcMap[n] = src;
    srcMap[src] = -1;
    last--;
    DeleteNode(n);
  }
  Assert(last == (int)nodes.size()-1);

  //-2 marks unchanged nodes
  delnodes.resize(nodeMap.size());
  for(size_t i=0;i<nodeMap.size();i++) {
    delnodes[i] = (nodeMap[i]==-2?i:nodeMap[i]);
    Assert(delnodes[i] < (int)nodes.size());
  }
}

template <class NodeData,class EdgeData>
EdgeData& Graph<NodeData,EdgeData>::AddEdge(int i,int j)
{
  return AddEdge(i,j,EdgeData());
}

template <class NodeData,class EdgeData>
EdgeData& Graph<NodeData,EdgeData>::AddEdge(int i,int j,const EdgeData& d)
{
  Assert(i!=j);
  Assert(!HasEdge(i,j));
  edgeData.push_back(d);
  EdgeDataPtr ptr = --edgeData.end();
  edges[i][j]=ptr;
  co_edges[j][i]=ptr;
  return *ptr;
}

template <class NodeData,class EdgeData>
bool Graph<NodeData,EdgeData>::HasEdge(int i,int j) const
{
  return (edges[i].count(j) != 0);
}

template <class NodeData,class EdgeData>
EdgeData* Graph<NodeData,EdgeData>::FindEdge(int i,int j) const
{
  ConstEdgeListIterator e=edges[i].find(j);
  if(e == edges[i].end()) return NULL;
  return &(*e->second);
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::DeleteEdge(int i,int j)
{
  EdgeListIterator k=edges[i].find(j);
  if(k == edges[i].end()) {
    FatalError("Graph::DeleteEdge(): Edge doesn't exist");
  }
  EdgeDataPtr ptr = k->second;
  edges[i].erase(k);

  CoEdgeListIterator k_co = co_edges[j].find(i);
  if(k_co == co_edges[j].end()) {
    FatalError("Graph::DeleteEdge(): Co-edge doesn't exist");
  }
  Assert(ptr == k_co->second);
  co_edges[j].erase(k_co);
  edgeData.erase(ptr);
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::DeleteOutgoingEdges(int i)
{
  //delete co-edges
  EdgeListIterator k;

  EdgeListIterator ebegin=edges[i].begin(),eend=edges[i].end();
  for(k=ebegin;k!=eend;++k) {
    EdgeListIterator it = co_edges[k->first].find(i);
    Assert(it != co_edges[k->first].end());
    Assert(it->second == k->second);
    edgeData.erase(it->second);
    co_edges[k->first].erase(it);
  }
    
  //delete edges
  edges[i].clear();
}

template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::DeleteIncomingEdges(int i)
{
  //delete edges
  CoEdgeListIterator k;
  CoEdgeListIterator cbegin=co_edges[i].begin(),cend=co_edges[i].end();
  for(k=cbegin;k!=cend;++k) {
    EdgeListIterator it = edges[k->first].find(i);
    edgeData.erase(it->second);
    edges[k->first].erase(it);
  }

  //delete co-edges
  co_edges[i].clear();
}

template<class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::Cleanup()
{
  nodeColor.clear();
  nodes.clear();
  edges.clear();
  co_edges.clear();
  edgeData.clear();
}

template <class NodeData,class EdgeData>
bool Graph<NodeData,EdgeData>::IsValid() const
{
  bool res=true;
  if(nodeColor.size() != nodes.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"nodeColor.size() doesn't match nodes.size(): "<<nodeColor.size()<<" vs "<<nodes.size());
    res=false;
  }
  if(edges.size() != nodes.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"edges.size() doesn't match nodes.size(): "<<edges.size()<<" vs "<<nodes.size());
    res=false;
  }
  if(co_edges.size() != nodes.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"co_edges.size() doesn't match nodes.size(): "<<co_edges.size()<<" vs "<<nodes.size());
    res=false;
  }
  int numEdges=0;
  for(size_t i=0;i<edges.size();i++) {
    ConstEdgeListIterator ebegin=edges[i].begin(),eend=edges[i].end();
    for(ConstEdgeListIterator e=ebegin;e!=eend;e++) {
      numEdges++;
      if(e->first < 0 || e->first >= (int)nodes.size()) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Edge ("<<i<<","<<e->first);
	res=false;
      }
      else if(e->first == (int)i) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Edge ("<<i<<","<<e->first);
	res=false;
      }
      else if(edges.size() == co_edges.size()) {
	ConstEdgeListIterator f=co_edges[e->first].find((int)i);
	if(f == co_edges[e->first].end()) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"Edge ("<<i<<","<<e->first);
	  res=false;
	}
	else {
	  Assert(f->first == (int)i);  //STL guarantees this...
	  if(e->second != f->second) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Edge ("<<i<<","<<e->first);
	    res=false;
	  }
	  else if(e->second == edgeData.end()) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Edge ("<<i<<","<<e->first);
	    res=false;
	  }
	}
      }
    }
  }
  if(numEdges != (int)edgeData.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Different number of edges vs edge data: "<<numEdges<<" vs "<<edgeData.size());
    res=false;
  }
  int numCoEdges=0;
  for(size_t i=0;i<co_edges.size();i++) {
	ConstEdgeListIterator  ebegin=co_edges[i].begin(),eend=co_edges[i].end();
    for(ConstEdgeListIterator e=ebegin;e!=eend;e++) {
      numCoEdges++;
      if(e->first < 0 || e->first >= (int)nodes.size()) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Co-edge ("<<i<<","<<e->first);
	res=false;
      }
      else if(edges.size() == co_edges.size()) {
	ConstEdgeListIterator f=edges[e->first].find((int)i);
	if(f == edges[e->first].end()) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"Co-edge ("<<i<<","<<e->first);
	  res=false;
	}
	else {
	  Assert(f->first == (int)i);  //STL guarantees this...
	  if(e->second != f->second) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Co-edge ("<<i<<","<<e->first);
	    res=false;
	  }
	  else if(e->second == edgeData.end()) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Co-edge ("<<i<<","<<e->first);
	    res=false;
	  }
	}
      }
    }
  }
  if(numCoEdges != (int)edgeData.size()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Different number of coedges vs edge data: "<<numCoEdges<<" vs "<<edgeData.size());
    res=false;
  }
  return res;
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::Begin(int n,Iterator& it) const {
  it.n=n;
  it.init(edges[n],co_edges[n]);
}


template <class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::NewTraversal()
{
  std::fill(nodeColor.begin(),nodeColor.end(),White);
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::DFS(Callback& f,Iterator)
{
  NewTraversal();
  int n = (int)nodeColor.size();
  for(int i=0;i<n;i++)
    if(nodeColor[i] == White) {
      f.NewComponent(i);
      _DFS(i,f,Iterator());
      if(f.Stop()) return;
    }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::BFS(Callback& f,Iterator it)
{
  NewTraversal();
  int n = (int)nodeColor.size();
  for(int i=0;i<n;i++)
    if(nodeColor[i] == White) {
      f.NewComponent(i);
      _BFS(i,f,Iterator());
      if(f.Stop()) return;
    }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::SimpleDFS(Callback& f,Iterator it)
{
  NewTraversal();
  int n = NumNodes();
  for(int i=0;i<n;i++)
    if(nodeColor[i] == White) {
      f.NewComponent(i);
      _SimpleDFS(i,f,it);
    }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::SimpleBFS(Callback& f,Iterator it)
{
  NewTraversal();
  int n = NumNodes();
  for(int i=0;i<n;i++)
    if(nodeColor[i] == White) {
      f.NewComponent(i);
      _SimpleBFS(i,f,it);
    }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::GuidedDFS(Callback& f,Iterator it)
{
  int n = NumNodes();
  for(int i=0;i<n;i++)
    if(f.Descend(i)) {
      f.NewComponent(i);
      _GuidedDFS(i,f,it);
    }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::GuidedBFS(Callback& f,Iterator it)
{
  int n = (int)nodeColor.size();
  for(int i=0;i<n;i++)
    if(f.Descend(i)) {
      f.NewComponent(i);
      _GuidedDFS(i,f,it);
    }
}



template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::_DFS(int n,Callback& f,Iterator it)
{
  Assert(nodeColor[n] == White);
  nodeColor[n] = Grey;
  f.Visit(n);
  if(f.Stop()) return;
  if(f.Descend(n)) {
    for(Begin(n,it);!it.end();it++) {
      int a=it.target();
      switch(nodeColor[a]) {
      case White:
	if(f.ForwardEdge(n,a))
	  _DFS(a,f,it);
	break;
      case Grey:
	f.BackEdge(n,a);
	break;
      case Black:
	f.CrossEdge(n,a);
	break;
      }
      if(f.Stop()) return;
    }
  }
  f.PostVisit(n);
  nodeColor[n] = Black;
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::_BFS(int s,Callback& f,Iterator it)
{
  std::queue<int> queue;
  int n=s;
  queue.push(n); nodeColor[n] = Grey;
  while(!queue.empty()) {
    n=queue.front(); queue.pop();
    f.Visit(n); if(f.Stop()) return;
    if(f.Descend(n)) {
      for(Begin(n,it);!it.end();it++) {
	int a=it.target();
	switch(nodeColor[a]){
	case White:
	  if(f.ForwardEdge(n,a)) {
	    queue.push(a); nodeColor[a] = Grey;
	  }
	  break;
	case Grey:
	  f.CrossEdge(n,a);
	  break;
	case Black:
	  f.BackEdge(n,a);
	  break;
	}
	if(f.Stop()) return;
      }
    }
    f.PostVisit(n);
    nodeColor[n] = Black;
    if(f.Stop()) return;
  }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::_SimpleDFS(int n,Callback& f,Iterator it)
{
  Assert(nodeColor[n] == White);
  nodeColor[n] = Grey;
  f.Visit(n);
  if(f.Stop()) return;
  for(Begin(n,it);!it.end();it++) {
    int a=it.target();
    if(nodeColor[a] == White) {
      _SimpleDFS(a,f,it);
      if(f.Stop()) return;
    }
  }
  f.PostVisit(n);
  nodeColor[n] = Black;
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::_SimpleBFS(int s,Callback& f,Iterator it)
{
  std::queue<int> queue;
  int n=s;
  queue.push(n); nodeColor[n] = Grey;
  if(f.Stop()) return;
  while(!queue.empty()) {
    n=queue.front(); queue.pop();
    f.Visit(n); if(f.Stop()) return;
    for(Begin(n,it);!it.end();it++) {
      int a=it.target();
      if(nodeColor[a] == White) {
	queue.push(a); nodeColor[a] = Grey;
      }
    }
    f.PostVisit(n);
    nodeColor[n] = Black;
    if(f.Stop()) return;
  }
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::_GuidedDFS(int n,Callback& f,Iterator it)
{
  Assert(f.Descend(n));
  f.Visit(n);
  for(Begin(n,it);!it.end();it++) {
    int a=it.target(); 
    if(f.Descend(a))
      _GuidedDFS(a,f,it);
  }
  f.PostVisit(n);
}

template <class NodeData,class EdgeData>
template <class Iterator>
void Graph<NodeData,EdgeData>::_GuidedBFS(int s,Callback& f,Iterator it)
{
  std::queue<int> queue;
  int n=s;
  Assert(f.Descend(n));
  queue.push(n);
  while(!queue.empty()) {
    n=queue.front(); queue.pop();
    f.Visit(n);
    for(Begin(n,it);!it.end();it++) {
      int a=it.target(); 
      if(f.Descend(a)) {
	queue.push(a);
      }
    }
    f.PostVisit(n);
  }
}


template<class NodeData,class EdgeData>
void Graph<NodeData,EdgeData>::WriteDOT(std::ostream& out)
{
  out<<"digraph {"<<std::endl;
  for(size_t i=0;i<nodeColor.size();i++) {
    for(EdgeListIterator e=edges[i].begin();e!=edges[i].end();e++)
      out<<"  "<<i<<" -> "<<e->target<<";"<<std::endl;
  }
  out<<"}"<<std::endl;
}



} //namespace Graph

#endif
