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

bool Load_TGF(std::istream& in,Graph<std::string,std::string>& G)
{
  G.Cleanup();
  int numNodes=0;
  int line=0;
  std::map<int,int> idsToIndices;
  std::string s;
  while(in) {
    line++;
    getline(in,s);
    if(!in) break;

    std::stringstream ss(s);
    ss>>s;
    if(!ss) continue;  //nothing on line
    if(s == "#") //now read the edges
      break;
    if(!IsValidInteger(s.c_str()))
      return false;
    int id=atoi(s.c_str());
    idsToIndices[id] = numNodes;
    numNodes++;
    G.Resize(numNodes);
    //here's where you might load node data from ss
    std::getline(ss,G.nodes.back());
  }
  if(!in) return false;

  //load edges
  while(in) {
    line++;
    getline(in,s);
    if(!in) break;

    std::stringstream ss(s);
    int id1,id2;
    ss>>id1>>id2;
    if(ss.bad()) return false;
    if(idsToIndices.count(id1)==0 || idsToIndices.count(id2)==0)
      return false;
    
    //here's where you might load edge data from ss
    std::getline(ss,s);
    G.AddEdge(idsToIndices[id1],idsToIndices[id2],s);
  }
  if(in.bad()) return false;
  return true;
}

void Save_TGF(std::ostream& o,const Graph<std::string,std::string>& G)
{
  for(size_t i=0;i<G.nodes.size();i++) {
    o<<i+1<<" "<<G.nodes[i]<<std::endl;
  }
  o<<"#"<<std::endl;
  for(size_t i=0;i<G.nodes.size();i++) {
    EdgeIterator<std::string> e;
    G.Begin(i,e);
    while(!e.end()) {
      o<<e.source()+1<<" "<<e.target()+1<<" "<<*e<<std::endl;
      e++;
    }
  }
}


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
