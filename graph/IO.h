#ifndef GRAPH_IO_H
#define GRAPH_IO_H

#include "Graph.h"
#include "Operations.h"
#include <string>
#include <sstream>
#include <map>
#include <stdlib.h>
#include <KrisLibrary/utils/stringutils.h>

namespace Graph {

/** @ingroup Graph
 * @brief Loads a graph from the Trivial Graph Format.
 */
bool Load_TGF(std::istream& in,Graph<std::string,std::string>& G);

/** @ingroup Graph
 * @brief Saves a graph to the Trivial Graph Format.
 */
void Save_TGF(std::ostream& o,const Graph<std::string,std::string>& G);

/** @ingroup Graph
 * @brief Serializes the graph's nodes using the ostream << operator.
 */
template <class N,class E>
void NodesToStrings(const Graph<N,E>& G,Graph<std::string,std::string>& Gs)
{
  CopyStructure(G,Gs);
  for(size_t i=0;i<G.nodes.size();i++) {
    std::stringstream ss;
    ss << G.nodes[i];
    Gs.nodes[i] = ss.str();
  }
}

/** @ingroup Graph
 * @brief De-serializes the graph's nodes using the istream >> operator.
 */
template <class N,class E>
bool NodesFromStrings(const Graph<std::string,std::string>& Gs,Graph<N,E>& G)
{
  CopyStructure(Gs,G);
  for(size_t i=0;i<G.nodes.size();i++) {
    std::stringstream ss(Gs.nodes[i]);
    ss >> G.nodes[i];
    if(ss.bad()) return false;
  }
  return true;
}

/** @ingroup Graph
 * @brief Serializes the graph's nodes and edges using the ostream << operator.
 */
template <class N,class E>
void NodesEdgesToStrings(const Graph<N,E>& G,Graph<std::string,std::string>& Gs)
{
  CopyStructure(G,Gs);
  for(size_t i=0;i<G.nodes.size();i++) {
    std::stringstream ss;
    ss << G.nodes[i];
    Gs.nodes[i] = ss.str();
  }
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<E> e;
    EdgeIterator<std::string> es;
    G.Begin(i,e);
    Gs.Begin(i,es);
    while(!e.done()) {
      std::stringstream ss;
      ss<<*e;
      *es = ss.str();
      ++e;
      ++es;
    }
  }
}

/** @ingroup Graph
 * @brief De-serializes the graph's nodes and edges using the istream >> operator.
 */
template <class N,class E>
bool NodesEdgesFromStrings(const Graph<std::string,std::string>& Gs,Graph<N,E>& G)
{
  CopyStructure(Gs,G);
  for(size_t i=0;i<G.nodes.size();i++) {
    std::stringstream ss(Gs.nodes[i]);
    ss >> G.nodes[i];
    if(ss.bad()) return false;
  }
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<E> e;
    EdgeIterator<std::string> es;
    G.Begin(i,e);
    Gs.Begin(i,es);
    while(!e.done()) {
      std::stringstream ss(*es);
      ss>>*e;
      if(ss.bad()) return false;
      ++e;
      ++es;
    }
  }
  return true;
}


} //namespace Graph

#endif
