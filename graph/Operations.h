#ifndef GRAPH_OPERATIONS_H
#define GRAPH_OPERATIONS_H

#include "Graph.h"
#include <KrisLibrary/errors.h>
#include <vector>
#include <set>
#include <map>

namespace Graph {

/// @brief Copies the edge structure of G1 to G2 without setting its data.
template <class N1,class E1,class N2,class E2>
void CopyStructure(const Graph<N1,E1>& G1, Graph<N2,E2>& G2)
{
  G2.Cleanup();
  G2.Resize(G1.nodes.size());
  for(size_t i=0;i<G1.nodes.size();i++) {
    EdgeIterator<E1> e;
    for(G1.Begin(i,e);!e.end();e++) {
      G2.AddEdge(i,e.target());
    }
  }
}

///@brief Transforms a graph G1 to G2 using the function f to transform nodes
///from G1 to G2.
///
///The semantics of f are that it takes params (a,b) where a
///is of type N1, and b is of type N2&, and transforms a to b.
template <class N1,class E,class N2,class F>
void Transform(const Graph<N1,E>& G1,F f,Graph<N2,E>& G2)
{
  G2.Cleanup();
  G2.Resize(G1.nodes.size());
  for(size_t i=0;i<G1.nodes.size();i++)
    f(G1.nodes[i],G2.nodes[i]);
  for(size_t i=0;i<G1.nodes.size();i++) {
    EdgeIterator<E> e;
    for(G1.Begin(i,e);!e.end();e++) {
      G2.AddEdge(i,e.target(),*e);
    }
  }
}

///@brief Transforms a graph G1 to G2 using the function f to transform nodes
///from G1 to G2, and using g to transform edges from G1 to G2.
///
///The semantics of f are that it takes params (a,b) where a
///is of type N1, and b is of type N2&, and transforms a to b.
template <class N1,class E1,class N2,class E2,class F,class G>
void Transform(const Graph<N1,E1>& G1,F f,G g,Graph<N2,E2>& G2)
{
  G2.Cleanup();
  G2.Resize(G1.nodes.size());
  for(size_t i=0;i<G1.nodes.size();i++)
    f(G1.nodes[i],G2.nodes[i]);
  E2 temp;
  for(size_t i=0;i<G1.nodes.size();i++) {
    EdgeIterator<E1> e;
    for(G1.Begin(i,e);!e.end();e++) {
      g(*e,temp);
      G2.AddEdge(i,e.target(),temp);
    }
  }
}

/// @brief Gets a list of edges
template <class Node,class Edge>
void GetEdges(const Graph<Node,Edge>& G, std::vector<std::pair<int,int> >& edges)
{
  edges.resize(G.NumEdges());
  int k=0;
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(i,e);!e.end();e++) {    
      edges[k].first = i;
      edges[k].second = e.target();
      k++;
    }
  }
}

/** @brief Creates the subgraph S of G induced by picking the nodes in nodes
 *
 * Nodes is a <b>sorted</b> list of the node indices.
 */
template <class Node,class Edge>
void GetSubGraph(const Graph<Node,Edge>& G,
		 const std::vector<int>& nodes,
		 Graph<Node,Edge>& S)
{
  //sanity check
  for(size_t i=0;i<nodes.size();i++) {
    Assert(0 <= nodes[i] && nodes[i] < (int)G.nodes.size());
    if(i!=0)
      Assert(nodes[i-1] < nodes[i]);
  }

  //inverse node map
  std::vector<int> nodeMap(G.nodes.size(),-1);
  for(size_t i=0;i<nodes.size();i++)
    nodeMap[nodes[i]]=(int)i;

  //copy nodes
  S.Resize(nodes.size());
  for(size_t i=0;i<nodes.size();i++)
    S.nodes[i] = G.nodes[nodes[i]];

  //copy edges
  for(size_t i=0;i<nodes.size();i++) {
    S.edges[i].clear();
    S.co_edges[i].clear();
  }
  S.edgeData.clear();
  for(size_t i=0;i<nodes.size();i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(nodes[i],e);!e.end();e++) {
      if(nodeMap[e.target()] != -1) {
	S.AddEdge(i,nodeMap[e.target()],*e);
      }
    }
  }
}

/** @brief Creates the subgraph S of G induced by nodes [nmin,nmax)
 */
template <class Node,class Edge>
void GetSubGraph(const Graph<Node,Edge>& G,
		 int nmin,int nmax,
		 Graph<Node,Edge>& S)
{
  Assert(nmin >= 0 && nmax <= (int)G.nodes.size());
  Assert(nmax >= 0);

  //copy nodes
  S.Resize(nmax-nmin);
  for(int i=nmin;i<nmax;i++)
    S.nodes[i-nmin] = G.nodes[i];

  //copy edges
  for(int i=0;i<nmax-nmin;i++) {
    S.edges[i].clear();
    S.co_edges[i].clear();
  }
  S.edgeData.clear();
  for(int i=nmin;i<nmax;i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(i,e);!e.end();e++) {
      if(e.target() >= nmin && e.target() < nmax) {
	S.AddEdge(i-nmin,e.target()-nmin,*e);
      }
    }
  }
}

/** @brief Creates the dual graph of G.  The dual references into the graph.
 */
template <class Node,class Edge>
void GetDualGraph(const Graph<Node,Edge>& G,
		  Graph<std::pair<int,int>,int>& D)
{
  D.Resize(G.NumEdges());
  std::map<std::pair<int,int>,int> edgeMap;
  int k=0;
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(i,e);!e.end();e++) {
      D.nodes[k].first = i;
      D.nodes[k].second = e.target();
      edgeMap[D.nodes[k]] = k;
      k++;
    }
  }
      
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(i,e);!e.end();e++) {
      std::pair<int,int> source(i,e.target());
      int sourceIndex = edgeMap[source];
      int vertex=e.target();
      EdgeIterator<Edge> e2;
      for(G.Begin(vertex,e2);!e2.end();e2++) {
	std::pair<int,int> target(vertex,e2.target());
	Assert(edgeMap.count(target)!=0);
	int targetIndex = edgeMap[target];
	if(targetIndex != sourceIndex)
	  D.AddEdge(sourceIndex,targetIndex,vertex);
      }
    }
  }
}

/** @brief Creates the dual graph of G and copies the data.
 */
template <class Node,class Edge>
void GetDualGraphCopy(const Graph<Node,Edge>& G,
		      Graph<Edge,Node>& D)
{
  D.Resize(G.NumEdges());
  std::map<std::pair<int,int>,int> edgeMap;
  int k=0;
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(i,e);!e.end();e++) {
      D.nodes[k] = *e;
      edgeMap[std::pair<int,int>(i,e.target())] = k;
      k++;
    }
  }
      
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<Edge> e;
    for(G.Begin(i,e);!e.end();e++) {
      std::pair<int,int> source(i,e.target());
      int sourceIndex = edgeMap[source];
      int vertex=e.target();
      EdgeIterator<Edge> e2;
      for(G.Begin(vertex,e2);!e2.end();e2++) {
	std::pair<int,int> target(vertex,e2.target());
	Assert(edgeMap.count(target)!=0);
	int targetIndex = edgeMap[target];
	if(targetIndex != sourceIndex)
	  D.AddEdge(sourceIndex,targetIndex,G.nodes[vertex]);
      }
    }
  }
}

///Returns the undirected 1-ring of the given node
template <class Node,class Edge>
void Get1Ring(const Graph<Node,Edge>& G,int node,std::set<int>& ring)
{
  std::set<int> nodes;
  nodes.insert(node);
  Get1Ring(G,nodes,ring);
}

///Returns the undirected n-ring of the given node
template <class Node,class Edge>
void GetNRing(const Graph<Node,Edge>& G,int node,int n,std::set<int>& ring)
{
  std::set<int> nodes;
  nodes.insert(node);
  for(int i=0;i<n;i++) {
    Get1Ring(G,nodes,ring);
    swap(nodes,ring);
  }
}

///Returns the undirected 1-ring of the given set of nodes
template <class Node,class Edge>
void Get1Ring(const Graph<Node,Edge>& G,const std::set<int>& nodes,std::set<int>& ring)
{
  for(std::set<int>::const_iterator i=nodes.begin();i!=nodes.end();i++) {
    UndirectedEdgeIterator<Edge> e;
    for(G.Begin(*i,e);!e.end();++e) {
      if(nodes.count(e.target())==0)
	ring.insert(e.target());
    }
  }
}

} //namespace Graph

#endif
