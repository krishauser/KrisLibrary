#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

#include <map>
#include <list>

namespace Graph {

/****************************************************************
 * EdgeIterator() classes
 * ---------------------------------------
 * source() returns the source (tail) of the edge.
 * target() returns the target (head) of the edge.
 * *() returns a reference to the satellite data.
 * ->() returns a pointer to the satellite data.
 * ++ proceeds to the next item.
 * end() returns true if the iteration has ended.
 *
 * Iteration is done as follows:
 * EdgeIterator<Data> it;
 * for(graph.Begin(it,n);!it.end();it++) {
 *   //do something
 * }
 * 
 ****************************************************************/

template <class Data>
struct EdgeIterator
{
public:
  typedef typename std::list<Data>::iterator DataPtr;
  typedef const std::map<int,DataPtr> EdgeList;
  typedef typename EdgeList::const_iterator EdgeListIterator;

  EdgeIterator()
    : n(-1), edges(NULL)
  {}
  inline void init(EdgeList& _edges,EdgeList& _co_edges) {
    edges = &_edges;
    e = _edges.begin();
  }
  inline int source() const { return n; } 
  inline int target() const { return e->first; }
  inline bool end() const { return e==edges->end(); }
  inline void operator++() { ++e; }
  inline void operator++(int) { ++e; }
  inline void operator--() { --e; }
  inline void operator--(int) { --e; }
  inline Data* operator->() const { return &(*e->second); }
  inline Data& operator*() const { return *e->second; }

  int n;
  EdgeListIterator e;
  EdgeList *edges;
};

template <class Data>
struct CoEdgeIterator
{
  typedef typename std::list<Data>::iterator DataPtr;
  typedef const std::map<int,DataPtr> EdgeList;
  typedef typename EdgeList::const_iterator EdgeListIterator;

  CoEdgeIterator()
    : n(-1), co_edges(NULL)
  {}
  inline void init(EdgeList& _edges,EdgeList& _co_edges) {
    co_edges = &_co_edges;
    e = _co_edges.begin();
  }
  inline int source() const { return n; }
  inline int target() const { return e->first; } 
  inline bool end() const { return e==co_edges->end(); }
  inline void operator++() { ++e; }
  inline void operator++(int) { ++e; }
  inline void operator--() { --e; }
  inline void operator--(int) { --e; }
  inline Data* operator->() const { return &(*e->second); }
  inline Data& operator*() const { return *e->second; }
  inline Data& operator*() { return *e->second; }

  int n;
  EdgeListIterator e;
  EdgeList *co_edges;
};

template <class Data>
struct UndirectedEdgeIterator
{
public:
  typedef typename std::list<Data>::iterator DataPtr;
  typedef const std::map<int,DataPtr> EdgeList;
  typedef typename EdgeList::const_iterator EdgeListIterator;

  UndirectedEdgeIterator()
    : n(-1), edges(NULL), co_edges(NULL)
  {}
  inline void init(EdgeList& _edges,EdgeList& _co_edges) {
    edges = &_edges;
    co_edges = &_co_edges;
    e = (_edges.empty() ? _co_edges.begin() : _edges.begin());
  }
  inline int source() const { return n; } 
  inline int target() const { return e->first; }
  inline bool end() const { return e==co_edges->end(); }
  inline void operator++() {
    ++e;
    if(e == edges->end()) e=co_edges->begin();
  }
  inline void operator++(int) { operator++(); }
  inline void operator--() {
    if(e == co_edges->begin())
      e = --edges->end();
    else --e;
  }
  inline void operator--(int) { operator--(); }
  inline Data* operator->() const { return &(*e->second); }
  inline Data& operator*() const { return *e->second; }

  int n;
  EdgeListIterator e;
  const EdgeList *edges,*co_edges;
};


typedef double Weight;

template <class Edge>
inline Weight ConstantWeight(const Edge& e) { return 1.0; }
template <class Edge>
inline Weight StandardWeight(const Edge& e) { return e.weight; }

} //namespace Graph

#endif
