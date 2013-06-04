#ifndef AI_GENERALIZED_ASTAR_H
#define AI_GENERALIZED_ASTAR_H

#include <vector>
#include <structs/IndexedPriorityQueue.h>

namespace AI {

using namespace std;

//can be set to 1 -- may return a suboptimal solution (but faster)
#define TEST_GOAL_ON_GENERATION 0

/** This class is a generalization of AStar that allows for non-scalar costs.
 * See AI/AStar.h.
 * 
 * The cost class is required to implement an ordering '<', equality testing
 * '=', unary negation '-', and accumulation '+'.  The default constructor is
 * required to return an identity element '0' such that x+'0' = x.
 */
template <class S,class C>
struct GeneralizedAStar
{
  //a node in the tree
  struct Node
  {
    Node():parent(NULL) {}
    ~Node() { for(size_t i=0;i<children.size();i++) delete children[i]; }

    C g;
    C f;
    S data;
    Node* parent;
    vector<Node*> children;
  };

  GeneralizedAStar();
  GeneralizedAStar(const S& start);
  virtual ~GeneralizedAStar() {}
  /// Resets the search from the given start state
  void SetStart(const S& start);
  /// Performs search until a goal is reached
  bool Search();
  /// Performs a single iteration of search
  bool SearchStep();
  /// Returns true if search failed
  bool SearchFailed();
  /// Returns the number of nodes in the tree
  int NumNodes() const;
  /// Returns the number of descendents of n
  int NumDescendents(const Node& n) const;

  ///The following must be overloaded by the subclass
  virtual bool IsGoal(const S& s) =0;
  virtual void Successors(const S& s,vector<S>& successors,vector<C>& cost) =0;
  virtual C Heuristic(const S& s) { return C(); }
  ///Optionally, overload these functions.  If not overloaded, does no
  ///visited test
  virtual void ClearVisited() {}
  virtual void Visit(const S& s,Node* n) {}
  virtual Node* VisitedStateNode(const S& s) { return NULL; }

  ///The A* search tree
  Node root;

  ///The A* search fringe.  Requires a pair for the key value,
  ///because if two items have the same f value, then the one with the
  ///greatest g is picked
  IndexedPriorityQueue<Node*,pair<C,C> > fringe;

  ///Upon successful termination, goal contains the goal node
  Node* goal;
  ///Upon successful termination, path contains the path from start to goal
  vector<S> path;
};


template <class S,class C>
GeneralizedAStar<S,C>::GeneralizedAStar()
{
}

template <class S,class C>
GeneralizedAStar<S,C>::GeneralizedAStar(const S& start)
{
  SetStart(start);
}

template <class S,class C>
void GeneralizedAStar<S,C>::SetStart(const S& start)
{  
  ClearVisited();
  fringe.clear();
  goal = NULL;

#if TEST_GOAL_ON_GENERATION
  if(IsGoal(start)) {
    path.push_back(start);
    return;
  }
#endif

  //initialize with the root node
  root.g=C();
  root.f=Heuristic(start);
  root.data = start;
  root.parent = NULL;
  for(size_t i=0;i<root.children.size();i++)
    delete root.children[i];
  root.children.clear();
  fringe.insert(&root,pair<C,C>(root.f,-root.g));
  Visit(start,&root);
}

template <class S,class C>
bool GeneralizedAStar<S,C>::Search()
{
  while(!fringe.empty()) {
    bool res=SearchStep();
    if(res) return true;
  }
  return false;
}

template <class S,class C>
bool GeneralizedAStar<S,C>::SearchStep()
{
  if(fringe.empty()) return false;

  Node* n = fringe.top().second;
  fringe.pop();

#if !TEST_GOAL_ON_GENERATION
  if(IsGoal(n->data)) {  //reached the goal!
    goal = n;
    path.resize(0);
    while(n != NULL) {
      path.push_back(n->data);
      n = n->parent;
    }
    //path is in backwards order, reverse it
    reverse(path.begin(),path.end());  
    return true;
  }
#endif

  vector<S> successors;
  vector<C> costs;
  Successors(n->data,successors,costs);

  n->children.resize(0);
  n->children.reserve(successors.size());
  for(size_t i=0;i<successors.size();i++) {
#if TEST_GOAL_ON_GENERATION
    if(IsGoal(successors[i])) {  //reached the goal!
      goal = NULL;
      path.resize(0);
      path.push_back(successors[i]);
      while(n != NULL) {
	path.push_back(n->data);
	n = n->parent;
      }
      //path is in backwards order, reverse it
      reverse(path.begin(),path.end());  
      return true;
    }
#endif
    Node* visited = VisitedStateNode(successors[i]);
    if(visited) {
      if(n->g + costs[i] < visited->g) {
	//cost is lower than previous, keep new state
	//reparent the node
	Node* p=visited->parent;
	Assert(p!=NULL);
	for(size_t c=0;c<p->children.size();c++)
	  if(visited == p->children[c]) {
	    p->children[c] = p->children.back();
	    p->children.resize(p->children.size()-1);
	    break;
	  }
	n->children.push_back(visited);
	visited->g = n->g + costs[i];
	visited->f = visited->g + Heuristic(successors[i]);
	visited->parent = n;
	fringe.refresh(visited,pair<C,C>(visited->f,-visited->g));
      }
    }
    else {
      //add successors[i] to the child list
      n->children.push_back(new Node);
      Node* child = n->children.back();
      child->data = successors[i];
      child->parent = n;
      child->g = n->g + costs[i];
      child->f = child->g + Heuristic(successors[i]);

      //add successors[i] to the fringe and mark as visited
      fringe.insert(child,pair<C,C>(child->f,-child->g));
      Visit(successors[i],child);
    }
  }
  return false;
}

template <class S,class C>
bool GeneralizedAStar<S,C>::SearchFailed()
{
  return fringe.empty();
}

template <class S,class C>
int GeneralizedAStar<S,C>::NumNodes() const
{
  return 1 + NumDescendents(root);
}

template <class S,class C>
int GeneralizedAStar<S,C>::NumDescendents(const Node& n) const
{
  int count=(int)n.children.size();
  for(size_t i=0;i<n.children.size();i++)
    count += NumDescendents(n.children[i]);
  return count;
}

} //namespace AI

#endif
