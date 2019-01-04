#ifndef GRAPH_PATH_H
#define GRAPH_PATH_H

#include <KrisLibrary/Logger.h>
#include <vector>
#include <list>
#include <map>

/** @file Path.h 
 * @ingroup Graph
 * @brief Functions for creating paths out of ancestor lists.
 */

namespace Graph {

/** @ingroup Graph
 * @brief Calculates a path of nodes from a list of parents.
 *
 * @param p The array of parents such that p[n] is the parent of node n
 * @param n The destination node
 * @param lastAncestor Either the start node or -1
 * @param path The output path from lastAncestor to n
 * @return true if the path reached lastAncestor
 */
inline bool GetAncestorPath(const std::vector<int>& p,
			    int n,int lastAncestor,
			    std::list<int>& path) {
  path.clear();
  path.push_front(n);
  if(n == lastAncestor) return true;
  int i=0;
  while(p[n] != -1) {
    n = p[n];
    path.push_front(n);
    if(n == lastAncestor) return true;
    if(i++ > (int)p.size()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"GetAncestorPath(): Iterated more than the number of nodes, aborting\n");
      i=0;
      for(std::list<int>::iterator it=path.begin();it!=path.end()&&i<20;it++,i++) 
	LOG4CXX_INFO(KrisLibrary::logger(),""<<*it);
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"...\n");
      std::list<int>::iterator it = path.end();
      for(int i=0;i<20;i++) it--;
      while(it != path.end()) {
	LOG4CXX_INFO(KrisLibrary::logger(),""<<*it);
	it++;
      }
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
      //abort();
      return false;
    }
  }
  return (lastAncestor == -1);
}

/** @ingroup Graph
 * @brief Vector version of GetAncestorPath for convenience
 */
inline bool GetAncestorPath(const std::vector<int>& p,
			    int n,int lastAncestor,
			    std::vector<int>& path) {
  std::list<int> lpath;
  if(!GetAncestorPath(p,n,lastAncestor,lpath)) return false;
  path.resize(0);
  path.insert(path.end(),lpath.begin(),lpath.end());
  return true;
}

/** @ingroup Graph
 * @brief Same as above, but with an arbitrary Node type
 */
template <class Node>
inline void GetAncestorPath(const std::map<Node,Node>& p,
			    Node n,std::list<Node>& path) 
{
  path.clear();
  path.push_front(n);
  int i=0;
  while(p.count(n) != 0) {
    n = p[n];
    path.push_front(n);
    if(i++ > p.size()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"GetAncestorPath(): Iterated more than the number of nodes, aborting\n");
      abort();
    }
  }
}


}  //namespace Graph;

#endif
