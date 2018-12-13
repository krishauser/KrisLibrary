#ifndef AI_GENERALIZED_ASTAR_H
#define AI_GENERALIZED_ASTAR_H

#include <vector>
#include <map>
#include <KrisLibrary/utils/stl_tr1.h>
#include <KrisLibrary/structs/IndexedPriorityQueue.h>

namespace AI {

/** This class is a generalization of AStar that allows for non-scalar costs.
 *
 * The user of this class should subclass it and overload, at the minimum,
 * IsGoal() and Successors().
 *
 * To perform visited state detection (usually recommended), the user should
 * implement ClearVisited(), Visit(), and VisitedStateNode().  The convenience
 * classes GeneralizedAStarWithMap and GeneralizedAStarWithHashMap provide
 * simple implementations of the visited state detection routines.
 *
 * To run the search, first call SetStart(), and then call Search().
 * Alternatively, to get more control, you can run SearchStep() until true
 * is returned (in which case a solution was found), or SearchFailed()
 * returns true (no solution exists).
 * 
 * The cost C class is required to implement an ordering '<', equality testing
 * '=', unary negation '-', and accumulation '+'.  The default constructor is
 * required to return an identity element '0' such that x+'0' = x.
 *
 * ints, floats, and doubles are fine for the cost class, but you can also
 * implement more sophisticated costs, such as multi-objective costs.
 */
template <class S,class C>
struct GeneralizedAStar
{
  //a node in the tree
  struct Node
  {
    Node():parent(NULL) {}
    ~Node() { for(size_t i=0;i<children.size();i++) delete children[i]; }

    ///cost from start
    C g;
    ///cost from start + heuristic
    C f;
    ///state data
    S data;
    ///pointer to parent
    Node* parent;
    ///list of pointers to children
    std::vector<Node*> children;
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
  /// Returns the number of previously expanded nodes
  int NumExpanded() const;
  /// Returns the number of descendents of n
  int NumDescendents(const Node& n) const;
  /// Returns the priority of the next node to be expanded
  C TopPriority() const;

  /// Returns true if the goal has been found
  inline bool GoalFound() const { return !path.empty(); }
  /// Returns cost from the start to the goal.  Note: only works with
  /// testGoalOnGeneration=false
  inline C GoalCost() const { if(goal != NULL) return goal->g; return zero; }
  /// Returns path of states to the goal
  inline const std::vector<S>& GoalPath() const { return path; }

  ///The following must be overloaded by the subclass
  virtual bool IsGoal(const S& s) =0;
  virtual void Successors(const S& s,std::vector<S>& successors,std::vector<C>& costs) =0;
  virtual C Heuristic(const S& s) { return zero; }
  ///Optionally, overload these functions.  If not overloaded, does no
  ///visited test
  virtual void ClearVisited() {}
  virtual void Visit(const S& s,Node* n) {}
  virtual Node* VisitedStateNode(const S& s) { return NULL; }
  ///Optionally, overload this callback to get feedback about how the
  ///search is proceeding.  If false is returned, search under this node
  ///is pruned.
  virtual bool OnExpand(Node* n) { return true; }

  ///By default false.  This can be set to true, in which case the search
  ///may return a suboptimal solution (but faster)
  bool testGoalOnGeneration;

  /// The zero element of type C.  By default this is uninitialized!  Be
  /// careful if you are using plain old data types (int, float, double)
  C zero;

  ///The A* search tree
  Node root;

  ///The A* search fringe.  Requires a pair for the key value,
  ///because if two items have the same f value, then the one with the
  ///greatest g is picked
  IndexedPriorityQueue<Node*,std::pair<C,C> > fringe;
  ///Temporary variables -- slightly reduces the number of memory allocations
  std::vector<S> successors;
  std::vector<C> costs;
  int numNodes;

  ///Upon successful termination, goal contains the goal node
  Node* goal;
  ///Upon successful termination, path contains the path from start to goal
  std::vector<S> path;
};

///Defines standard AStar as a GeneralizedAStar instance with double-valued
///costs.
template <class S>
struct AStar : public GeneralizedAStar<S,double>
{
  typedef struct GeneralizedAStar<S,double>::Node Node;
  AStar() { this->zero = 0.0; }
  virtual ~AStar() {}
};

///Defines IntegerAStar as a GeneralizedAStar instance with integer-valued
///costs.
template <class S>
struct IntegerAStar : public GeneralizedAStar<S,int>
{
  typedef struct GeneralizedAStar<S,int>::Node Node;
  IntegerAStar() { this->zero = 0; }
  virtual ~IntegerAStar() {}
};

///Convenience class: uses a std::map to store visited nodes.  Requires
///S to be a mappable type (e.g., implement ==, <, copy constructor).
template <class S,class C>
class GeneralizedAStarWithMap : public GeneralizedAStar<S,C>
{
 public:
  typedef struct GeneralizedAStar<S,C>::Node Node;
  std::map<S,Node*> visited;
  virtual ~GeneralizedAStarWithMap() {}
  virtual void ClearVisited() { visited.clear(); }
  virtual void Visit(const S& s,Node* n) { visited[s]=n;}
  virtual Node* VisitedStateNode(const S& s) {
    typename std::map<S,Node*>::const_iterator i=visited.find(s);
    if(i==visited.end()) return NULL;
    return i->second;
  }
};

///Convenience class: uses a std::unordered_map to store visited nodes. 
///Requires S to be a hashable type.
template <class S,class C>
class GeneralizedAStarWithHashMap : public GeneralizedAStar<S,C>
{
 public:
  typedef struct GeneralizedAStar<S,C>::Node Node;
  UNORDERED_MAP_TEMPLATE<S,Node*> visited;
  virtual ~GeneralizedAStarWithHashMap() {}
  virtual void ClearVisited() { visited.clear(); }
  virtual void Visit(const S& s,Node* n) { visited[s]=n;}
  virtual Node* VisitedStateNode(const S& s) {
    typename UNORDERED_MAP_TEMPLATE<S,Node*>::const_iterator i=visited.find(s);
    if(i==visited.end()) return NULL;
    return i->second;
  }
};


template <class S,class C>
GeneralizedAStar<S,C>::GeneralizedAStar()
  :testGoalOnGeneration(false),numNodes(0),goal(NULL)
{
}

template <class S,class C>
GeneralizedAStar<S,C>::GeneralizedAStar(const S& start)
  :testGoalOnGeneration(false),numNodes(0),goal(NULL)
{
  SetStart(start);
}

template <class S,class C>
void GeneralizedAStar<S,C>::SetStart(const S& start)
{  
  ClearVisited();
  fringe.clear();
  goal = NULL;
  path.resize(0);
  numNodes = 1;

  if(testGoalOnGeneration) {
    if(IsGoal(start)) {
      path.push_back(start);
      return;
    }
  }

  //initialize with the root node
  root.g=zero;
  root.f=Heuristic(start);
  root.data = start;
  root.parent = NULL;
  for(size_t i=0;i<root.children.size();i++)
    delete root.children[i];
  root.children.clear();
  fringe.insert(&root,std::pair<C,C>(root.f,-root.g));
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

  //give the subclass optional feedback
  if(!OnExpand(n)) return false;

  if(!testGoalOnGeneration) {
    if(IsGoal(n->data)) {  //reached the goal!
      goal = n;
      path.resize(0);
      while(n != NULL) {
	path.push_back(n->data);
	n = n->parent;
      }
      //path is in backwards order, reverse it
      std::reverse(path.begin(),path.end());  
      return true;
    }
  }

  successors.resize(0);
  costs.resize(0);
  Successors(n->data,successors,costs);
  Assert(successors.size()==costs.size());
  /*
  for(size_t i=0;i<costs.size();i++)
    Assert(costs[i] >= zero);
  */

  n->children.resize(0);
  n->children.reserve(successors.size());
  for(size_t i=0;i<successors.size();i++) {
    if(testGoalOnGeneration) {
      if(IsGoal(successors[i])) {  //reached the goal!
	goal = NULL;
	path.resize(0);
	path.push_back(successors[i]);
	while(n != NULL) {
	  path.push_back(n->data);
	  n = n->parent;
	}
	//path is in backwards order, reverse it
	std::reverse(path.begin(),path.end());  
	return true;
      }
    }
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
	fringe.refresh(visited,std::pair<C,C>(visited->f,-visited->g));
      }
    }
    else {
      //add successors[i] to the child list
      numNodes ++;
      n->children.push_back(new Node);
      Node* child = n->children.back();
      child->data = successors[i];
      child->parent = n;
      child->g = n->g + costs[i];
      child->f = child->g + Heuristic(successors[i]);

      //add successors[i] to the fringe and mark as visited
      fringe.insert(child,std::pair<C,C>(child->f,-child->g));
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
  return numNodes;
  //return 1 + NumDescendents(root);
}

template <class S,class C>
int GeneralizedAStar<S,C>::NumExpanded() const
{
  return NumNodes()-(int)fringe.size();
}

template <class S,class C>
int GeneralizedAStar<S,C>::NumDescendents(const Node& n) const
{
  int count=(int)n.children.size();
  for(size_t i=0;i<n.children.size();i++)
    count += NumDescendents(*n.children[i]);
  return count;
}

template <class S,class C>
C GeneralizedAStar<S,C>::TopPriority() const
{
  if(fringe.empty()) return zero;
  return fringe.top().first.first;
}

} //namespace AI

#endif
