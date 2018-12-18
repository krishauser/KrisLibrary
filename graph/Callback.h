#ifndef GRAPH_CALLBACK_H
#define GRAPH_CALLBACK_H

#include <list>
#include <map>
#include <vector>
#include <assert.h>
#include <limits.h>

namespace Graph {

  /** @addtogroup Graph */
  /*@{*/

/// @brief A template base class for a graph traversal.
///
/// The call order is different between DFS and BFS traversals.
///
/// For Graph traversal, Node is set to int.
template<class Node>
struct CallbackBase
{
  virtual ~CallbackBase() {}
  ///Return true to halt the traversal
  virtual bool Stop() { return false; }
  ///Called when a node is first visited
  virtual void Visit(Node) {}
  ///Return true to visit the node's adjacencies
  virtual bool Descend(Node) { return true; }
  ///Called after a node has been visited
  virtual void PostVisit(Node) {}
  ///Called on traversal of edges from i to unvisited j
  virtual bool ForwardEdge(Node i,Node j) { return true; }
  ///Called on traversal of edges from i to currently visiting j
  virtual void CrossEdge(Node i,Node j) {}
  ///Called on traversal of edges from i to previously visited j
  virtual void BackEdge(Node i,Node j) {}
  ///Called when a new component is visited
  virtual void NewComponent(Node) {}
};

/// Count the number of nodes traversed
template <class Node>
struct CountCallback : public CallbackBase<Node>
{
  CountCallback() :count(0) {}
  virtual void Visit(Node n) { count++; }
  int count;
};

/// Find a particular node
template <class Node>
struct FindCallback : public CallbackBase<Node>
{
  FindCallback(Node n) :node(n),found(false) {}
  virtual bool Stop() { return found; }
  virtual void Visit(Node n) { if(n == node) found = true; }
  Node node;
  bool found;
};

/// Get the depth of all nodes (in an indexed graph, such as Graph)
struct DepthIntCallback : public CallbackBase<int>
{
  DepthIntCallback(int numNodes):depth(numNodes,0) {}
  virtual bool ForwardEdge(int i,int j) {
    depth[j] = depth[i]+1;
    return true;
  }

  std::vector<int> depth;
};

/// Get the start/finish time of a traversal
template <class Node>
struct TimeCallback : public CallbackBase<Node>
{
  TimeCallback() :time(0) {}
  virtual void Visit(Node n) {
    time++;
    startTime[n] = time;
  }
  virtual void PostVisit(Node n) {
    time++;
    finishTime[n] = time;
  }
  int time;
  std::map<Node,int> startTime, finishTime;
};

/// Same as above, but for indexed graphs
struct TimeIntCallback : public CallbackBase<int>
{
  TimeIntCallback(int numNodes)
    :time(0),startTime(numNodes,-1),finishTime(numNodes,-1)
  {}
  virtual void Visit(int n) { startTime[n] = time++; }
  virtual void PostVisit(int n) { finishTime[n] = time++; }
  int time;
  std::vector<int> startTime, finishTime;
};

/// Compute if the graph has a cycle
template <class Node>
struct CycleCallback : public CallbackBase<Node>
{
  CycleCallback() : hasCycle(false) {}
  virtual bool Stop() { return hasCycle; }
  virtual void BackEdge(Node i,Node j) { hasCycle = true; }
  bool hasCycle;
};

/// Perform topological sort, when used with DFS
template <class Node>
struct TopologicalSortCallback : public CallbackBase<Node>
{
  TopologicalSortCallback() :hasCycle(false) {}
  virtual bool Stop() { return hasCycle; }
  virtual void PostVisit(Node n) { list.push_front(n); }
  virtual void BackEdge(Node i,Node j) { hasCycle = true; }
  std::list<Node> list;
  bool hasCycle;
};

/// Compute the traversal graph of a traversal
template <class Node>
struct TraversalGraphCallback : public CallbackBase<Node>
{
  TraversalGraphCallback() {}
  virtual bool ForwardEdge(Node i,Node j) { parents[j]=i; return true; }
  std::map<Node,Node> parents;
};

/// Same as above, but for indexed graphs
struct TraversalGraphIntCallback : public CallbackBase<int>
{
  TraversalGraphIntCallback(int numNodes):parents(numNodes,-1) {}
  virtual bool ForwardEdge(int i,int j) {  parents[j]=i; return true; }
  std::vector<int> parents;
};

/// Find the shortest path to the destination node when used with DFS.
/// The path is returned in a parent mapping.
template <class Node>
struct PathCallback : public CallbackBase<Node>
{
  PathCallback(Node n) :node(n),found(false) {}
  virtual bool Stop() { return found; }
  virtual void Visit(Node n) { if(n == node) found = true; }
  virtual bool ForwardEdge(Node i,Node j) { parents[j]=i; return true; }
  Node node;
  bool found;
  std::map<Node,Node> parents;
};

/// Same as above, but for indexed graphs
struct PathIntCallback : public CallbackBase<int>
{
  PathIntCallback(int numNodes,int n) :node(n),found(false),parents(numNodes,-1) {}
  virtual bool Stop() { return found; }
  virtual void Visit(int n) { if(n == node) found = true; }
  virtual bool ForwardEdge(int i,int j) { assert(parents[j]==-1); parents[j]=i;  return true; }
  int node;
  bool found;
  std::vector<int> parents;
};

/// Calculate the shortest paths to all nodes from a single start node
/// for an indexed graph.
struct ShortestPathsIntCallback : public CallbackBase<int>
{
  ShortestPathsIntCallback(int numNodes,int startNode)
    :parents(numNodes,-1),depth(numNodes,INT_MAX)
  {
    depth[startNode] = 0;
  }
  virtual bool ForwardEdge(int i,int j) 
  {
    assert(parents[j]==-1); 
    parents[j] = i;
    depth[j] = depth[i]+1;
    return true;
  }
  std::vector<int> parents;
  std::vector<int> depth;
};

/// Counts the connected components in a graph, storing the connected
/// component # for each node
struct ComponentIntCallback : public Graph::CallbackBase<int>
{
  ComponentIntCallback(int numNodes)
    :cComponents(numNodes), numComponents(0) {}
  virtual void NewComponent(int node) 
  { numComponents++; }
  virtual void Visit(int node) 
  { cComponents[node] = numComponents-1; }

  std::vector<int> cComponents;
  int numComponents;
};

  /*@}*/
} //namespace Graph

#endif
