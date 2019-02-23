#ifndef GRAPH_APPROXIMATE_SHORTEST_PATHS_H
#define GRAPH_APPROXIMATE_SHORTEST_PATHS_H

#include <KrisLibrary/Logger.h>
#include "ShortestPaths.h"

namespace Graph {

/** @ingroup Graph
 * @brief Approximate single-source shortest paths, whose output is at most
 * (1+epsilon)^d times the true shortest path, where d is the depth of the node.
 * This can be somewhat faster than running true shortest paths
 * 
 * @sa ShortestPathProblem
 */
template <class Node,class Edge>
class ApproximateShortestPathProblem
{
 public:
  ApproximateShortestPathProblem(const Graph<Node,Edge>& g,Real epsilon=0);
  virtual ~ApproximateShortestPathProblem() {}
  
  void InitializeSource(int s);
  void InitializeSources(const vector<int>& s);

  template <typename WeightFunc,typename Iterator>
  void FindPath(int t,WeightFunc w,Iterator it);
  template <typename WeightFunc,typename Iterator>
  int FindAPath(const set<int>& t,WeightFunc w,Iterator it);
  template <typename WeightFunc,typename Iterator>
  inline void FindAllPaths(WeightFunc w,Iterator it) { FindPath(-1,w,it); }

  //after all paths are found, the shortest paths can be updated with the following
  //when an edge (u,v)'s weight is increased/decreased, call these methods
  template <typename WeightFunc,typename InIterator,typename OutIterator>
  void IncreaseUpdate(int u,int v,WeightFunc w,InIterator in,OutIterator out);
  template <typename WeightFunc,typename InIterator,typename OutIterator>
  void DecreaseUpdate(int u,int v,WeightFunc w,InIterator in,OutIterator out);
  template <typename WeightFunc,typename InIterator,typename OutIterator>
  void DeleteUpdate(int u,int v,WeightFunc w,InIterator in,OutIterator out);

  template <typename WeightFunc,typename Iterator>
  bool HasShortestPaths(int s,WeightFunc w,Iterator it,Real epsilon=-1);

  //template specializations (directed/undirected)

  template <typename WeightFunc>
  inline void FindPath_Directed(int t,WeightFunc w) {
    FindPath(t,w,EdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline int FindAPath_Directed(const set<int>& t,WeightFunc w) {
    return FindAPath(t,w,EdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline void FindAllPaths_Directed(WeightFunc w) {
    EdgeIterator<Edge> it;
    FindAllPaths(w,it);
  }
  template <typename WeightFunc>
  inline void IncreaseUpdate_Directed(int u,int v,WeightFunc w) {
    CoEdgeIterator<Edge> in; EdgeIterator<Edge> out;
    IncreaseUpdate(u,v,w,in,out);
  }
  template <typename WeightFunc>
  inline void DecreaseUpdate_Directed(int u,int v,WeightFunc w) {
    CoEdgeIterator<Edge> in; EdgeIterator<Edge> out;
    DecreaseUpdate(u,v,w,in,out);
  }
  template <typename WeightFunc>
  inline void DeleteUpdate_Directed(int u,int v,WeightFunc w) {
    CoEdgeIterator<Edge> in; EdgeIterator<Edge> out;
    DeleteUpdate(u,v,w,in,out);
  }
  template <typename WeightFunc>
  inline bool HasShortestPaths_Directed(int s,WeightFunc w,Real epsilon=-1) {
    CoEdgeIterator<Edge> it;
    return HasShortestPaths(s,w,it,epsilon);
  }

  template <typename WeightFunc>
  inline void FindPath_Undirected(int t,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    FindPath(t,w,it);
  }
  template <typename WeightFunc>
  inline int FindAPath_Undirected(const set<int>& t,WeightFunc w) {
    return FindAPath(t,w,UndirectedEdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline void FindAllPaths_Undirected(WeightFunc w) {
    FindAllPaths(w,UndirectedEdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline void IncreaseUpdate_Undirected(int u,int v,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    IncreaseUpdate(u,v,w,it,it);
    IncreaseUpdate(v,u,w,it,it);
  }
  template <typename WeightFunc>
  inline void DecreaseUpdate_Undirected(int u,int v,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    DecreaseUpdate(u,v,w,it,it);
    DecreaseUpdate(v,u,w,it,it);
  }
  template <typename WeightFunc>
  inline void DeleteUpdate_Undirected(int u,int v,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    DeleteUpdate(u,v,w,it,it);
    DeleteUpdate(v,u,w,it,it);
  }
  template <typename WeightFunc>
  inline bool HasShortestPaths_Undirected(int s,WeightFunc w,Real epsilon=-1) {
    UndirectedEdgeIterator<Edge> it;
    return HasShortestPaths(s,w,it,epsilon);
  }

  ///Optional: a callback that is called to change the distance/parent
  ///to the node n.  Subclasses can intercept this call if desired
  ///but it is the responsibility of the subclass to call 'd[n]=dn,p[n]=pn'.
  ///Otherwise undefined behavior will result!
  virtual void SetDistance(int n,Real dn,int pn) { d[n]=dn; p[n]=pn; }

  const Graph<Node,Edge>& g;
  Real epsilon;

  std::vector<int> p;
  std::vector<Weight> d;
};


//definition of ApproximateShortestPaths

template <class Node,class Edge>
ApproximateShortestPathProblem<Node,Edge>::
ApproximateShortestPathProblem(const Graph<Node,Edge>& _g,Real _epsilon)
  :g(_g),epsilon(_epsilon)
{}

template <class Node,class Edge>
void ApproximateShortestPathProblem<Node,Edge>::InitializeSource(int s)
{
  //initialize
  int nn = g.NumNodes();
  p.resize(nn);
  d.resize(nn);
  for(int i=0;i<nn;i++) {
    p[i] = -1;
    d[i] = inf;
  }
  d[s] = 0;
}

template <class Node,class Edge>
void ApproximateShortestPathProblem<Node,Edge>::InitializeSources(const vector<int>& s)
{
  //initialize
  int nn = g.NumNodes();
  p.resize(nn);
  d.resize(nn);
  for(int i=0;i<nn;i++) {
    p[i] = -1;
    d[i] = inf;
  }
  for(size_t i=0;i<s.size();i++)
    d[s[i]] = 0;
}

template <class Node,class Edge>
template <typename WeightFunc,typename Iterator>
void ApproximateShortestPathProblem<Node,Edge>::FindPath(int t,WeightFunc w,Iterator it)
{
  int nn=g.NumNodes();
  FixedSizeHeap<Weight> H(nn);  //O(n) init, worst case log n update
  for(int i=0;i<nn;i++) H.push(i,-d[i]);
  Real fudgeFactor = (1.0+epsilon);

  while(!H.empty()) {
    //pop the smallest distance element from q
    int u = H.top(); H.pop();
    if(u == t) return;
    
    for(g.Begin(u,it);!it.end();it++) {
      int v=it.target();
      //relax distances
      Weight dvu = d[u] + w(*it,it.source(),it.target());
      if(d[v] > dvu*fudgeFactor) {
	SetDistance(v,dvu,u);
	H.adjust(v,-d[v]);
      }
    }
  } 
}

template <class Node,class Edge>
template <typename WeightFunc,typename Iterator>
int ApproximateShortestPathProblem<Node,Edge>::FindAPath(const set<int>& targetSet,WeightFunc w,Iterator it)
{
  int nn=g.NumNodes();
  FixedSizeHeap<Weight> H(nn);  //O(n) init, worst case log n update
  for(int i=0;i<nn;i++) H.push(i,-d[i]);

  Real fudgeFactor = (1.0+epsilon);

  while(!H.empty()) {
    //pop the smallest distance element from q
    int u = H.top(); H.pop();
    if(targetSet.count(u)!=0) return u;
    
    for(g.Begin(u,it);!it.end();it++) {
      int v=it.target();
      //relax distances
      Weight dvu = d[u] + w(*it,it.source(),it.target());
      if(d[v] > dvu*fudgeFactor) {
	SetDistance(v,dvu,u);
	H.adjust(v,-d[v]);
      }
    }
  } 
  return -1;
}


template <class Node,class Edge>
template <typename WeightFunc,typename InIterator,typename OutIterator>
void ApproximateShortestPathProblem<Node,Edge>::IncreaseUpdate(int u,int v,WeightFunc w,
						    InIterator in,OutIterator out)
{
  //1) if not in the SP tree, return
  if(p[v] != u) return;

  //2) look for alternative SP, if found, return
  for(g.Begin(v,in);!in.end();in++) {
    int t=in.target();
    if(d[v] == d[t]+w(*in,in.source(),in.target())) {
      p[v]=t;
      return;
    }
  }

  //3) identify affected nodes
  vector<int> Q(1,v);
  for(size_t i=0;i<Q.size();i++) {
    int n=Q[i];
    SetDistance(n,inf,-1);
    for(g.Begin(n,out);!out.end();out++) {
      int t=out.target();
      if(p[t]==n) {
	//identify any alternative routes
	for(g.Begin(t,in);!in.end();in++) {
	  int s=in.target();
	  if(d[t]==d[s]+w(*in,in.source(),in.target())) {
	    p[t]=s;
	    break;
	  }
	}
	if(p[t]==n) Q.push_back(t);
      }
    }
  }
  //4) update affected nodes by finding paths from outside nodes
  Real fudgeFactor = (1.0+epsilon);
  Heap<int,Weight> H;  //O(1) init, O(n) adjust
  for(size_t i=0;i<Q.size();i++) {
    int n=Q[i];
    for(g.Begin(n,in);!in.end();in++) {
      int s=in.target();
      double W=w(*in,in.source(),in.target());
      if(d[n]>(d[s]+W)*fudgeFactor) {
	SetDistance(n,d[s]+W,s);
      }
    }
    if(!Math::IsInf(d[n])) H.push(n,-d[n]);
  }
  while(!H.empty()) {
    int n=H.top(); H.pop();
    for(g.Begin(n,out);!out.end();out++) {
      int t=out.target();
      double W=w(*out,out.source(),out.target());
      if(d[t]>(d[n]+W)*fudgeFactor) {
	SetDistance(t,d[n]+W,n);
	H.adjust(t,-d[t]);
      }
    }
  }
}

template <class Node,class Edge>
template <typename WeightFunc,typename InIterator,typename OutIterator>
void ApproximateShortestPathProblem<Node,Edge>::DecreaseUpdate(int u,int v,WeightFunc w,
						    InIterator in,OutIterator out)
{
  //NOTE: can't use find edge because it doesn't work for undirected graphs
  Real fudgeFactor = (1.0+epsilon);
  for(g.Begin(u,out);!out.end();out++)
    if(out.target()==v) {
      double W=w(*out,out.source(),out.target());
      if(d[v] <= d[u]+W) {
	return;  //no change
      }
      SetDistance(v,d[u]+W,u);
      break;
    }
  if(out.end()) {
    FatalError("ApproximateShortestPathProblem::DecreaseUpdate(): Warning, decreasing an edge that doesn't exist in the graph!");
  }
  Heap<int,Weight> H;   //O(1) init, O(n) adjust
  H.push(v,-d[v]);
  while(!H.empty()) {
    int n=H.top(); H.pop();
    for(g.Begin(n,out);!out.end();out++) {
      int t=out.target();
      double W=w(*out,out.source(),out.target());
      if(d[t]>(d[n]+W)*fudgeFactor) {
	SetDistance(t,d[n]+W,n);
	H.adjust(t,-d[t]);
      }
    }
  }
}

template <class Node,class Edge>
template <typename WeightFunc,typename InIterator,typename OutIterator>
void ApproximateShortestPathProblem<Node,Edge>::DeleteUpdate(int u,int v,WeightFunc w,
						  InIterator in,OutIterator out)
{
  Real fudgeFactor = 1.0+epsilon;
  if(p[v] == u) {
    SetDistance(v,inf,-1);
    //find the best alternate parent
    for(g.Begin(v,in);!in.end();++in) {
      int t=in.target();
      if(p[t] == v) continue;
      Assert(t != u);
      double W=w(*in,in.source(),in.target());
      if(d[v] > (d[t]+W)*fudgeFactor) {
	SetDistance(v,d[t]+W,t);
      }
    }
    if(p[v] != -1) {
      d[v] = inf;
      DecreaseUpdate(p[v],v,w,in,out);
      d[v] = 0;
      IncreaseUpdate(p[v],v,w,in,out);
    }
    else {
      for(g.Begin(v,out);!out.end();++out) {
	int t=out.target();
	IncreaseUpdate(v,t,w,in,out);
      }
    }
  }
}

template <class Node,class Edge>
template <typename WeightFunc,typename Iterator>
bool ApproximateShortestPathProblem<Node,Edge>::HasShortestPaths(int s,WeightFunc w,Iterator it,Real testEpsilon) {
  Real fudgeFactor;
  if(testEpsilon < 0) fudgeFactor = 1.0+epsilon;
  else fudgeFactor = 1.0+testEpsilon;
  int nn=g.NumNodes();
  FixedSizeHeap<Weight> H(nn);
  for(int i=0;i<nn;i++) H.push(i,-d[i]);
  while(!H.empty()) {
    int n=H.top(); H.pop();
    if(n==s) {
      if(d[n]!=0) {
	LOG4CXX_INFO(KrisLibrary::logger(),"The start doesn't have distance 0\n");
	return false;
      }
    }
    for(g.Begin(n,it);!it.end();++it) {
      int t=it.target();
      double W=w(*it,it.source(),it.target());
      if(p[n] == t) {
	if(fudgeFactor == 1.0) {
	  if(fabs(d[n]-d[t]-W) > 1e-10) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Inconsistency in node "<< n<<"'s weight through parent "<< t<<", "<< d[n]<<" vs "<<d[t]+W<<"="<<d[t]<<"+"<<W);
	    return false;
	  }
	}
      }
      if(d[n]-(d[t]+W)*fudgeFactor > 1e-10) {
	LOG4CXX_INFO(KrisLibrary::logger(),"There exists a shorter path ("<<t<<","<<n<<") not ("<<p[n]<<","<<n);
	LOG4CXX_INFO(KrisLibrary::logger(),"Weight 1 is "<< d[t]+W<<" compared to "<< d[n]);
	LOG4CXX_INFO(KrisLibrary::logger(),"Fudge factor "<< fudgeFactor<<", comparison value "<< d[n]-(d[t]-W)*fudgeFactor);
	return false;
      }
    }
  }
  return true;
}


} //namespace Graph

#endif
