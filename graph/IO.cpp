#include <stdlib.h>
#include "IO.h"

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


}