#ifndef GRAPH_TREE_H
#define GRAPH_TREE_H

#include "Callback.h"
#include <KrisLibrary/errors.h>
#include <queue>
#include <set>

namespace Graph {

/** @ingroup Graph
 * @brief A tree graph structure, represented directly at the node level.
 *
 * Node data T is carried directly as a base class of the TreeNode.
 * Edge data E is carried at the child j of an edge (i,j).
 *
 * Ensuring the graph is acyclical must be done by the user.  If adding node 
 * B as a child of A, first check if A is a descendent of B by calling
 * B.hasDescendant(A) or A.hasAncestor(B).
 *
 * A callbacks for DFS or BFS must be a CallbackBase with a 
 * TreeNode<T,E>* as the template argument.
 */
template <class T,class E=int>
class TreeNode : public T
{
public:
  typedef TreeNode<T,E> MyType;
  typedef CallbackBase<MyType*> Callback;
  
  TreeNode();
  TreeNode(const T& t);
  TreeNode(const MyType& t);
  ~TreeNode();
  const MyType& operator =(const MyType&);
  
  inline MyType* getParent() const { return parent; }
  inline MyType* getFirstChild() const { return childFirst; }
  inline MyType* getLastChild() const { return childLast; }
  inline MyType* getNextSibling() const { return nextSibling; }
  inline E& edgeFromParent() { return parentEdge; }
  inline const E& edgeFromParent() const { return parentEdge; }

  MyType* addChild();
  MyType* addChild(const T&);
  MyType* addChild(MyType*);
  MyType* getChild(int index) const;
  int getChildIndex(MyType* c) const;
  void enumChildren(std::vector<MyType*>& children) const;
  void setChildEdge(MyType* c,const E&) const;
  void detachChild(MyType*);
  void eraseChild(MyType*);
  void clearChildren();
  
  bool hasDescendent(const MyType* d) const;
  bool hasAncestor(const MyType* a) const;
  
  void DFS(Callback&);
  void BFS(Callback&);
  void reRoot();
  MyType* LCA(const MyType* n);

private:
  MyType* parent;
  MyType* nextSibling;
  MyType *childFirst, *childLast;
  E parentEdge;  //edge from parent to this
};

template <class T,class E>
TreeNode<T,E>::TreeNode()
:parent(NULL),nextSibling(NULL),childFirst(NULL),childLast(NULL)
{}

template <class T,class E>
TreeNode<T,E>::TreeNode(const T& t)
:T(t),parent(NULL),nextSibling(NULL),childFirst(NULL),childLast(NULL)
{}

template <class T,class E>
TreeNode<T,E>::TreeNode(const MyType& t)
:parent(NULL),nextSibling(NULL),childFirst(NULL),childLast(NULL)
{
	operator = (t);
}

template <class T,class E>
TreeNode<T,E>::~TreeNode()
{
	clearChildren();
}

template <class T,class E>
const TreeNode<T,E>& TreeNode<T,E>::operator =(const MyType& t)
{
	T::operator = (t);
	parentEdge = t.parentEdge;
	clearChildren();
	MyType* tc = t.childFirst;
	while(tc) {
		MyType* c=addChild();
		*c = *tc;
		tc = tc->nextSibling;
	}
	return *this;
}

template <class T,class E>
TreeNode<T,E>* TreeNode<T,E>::addChild()
{
	MyType* c = new MyType();
	return addChild(c);
}

template <class T,class E>
TreeNode<T,E>* TreeNode<T,E>::addChild(const T& t)
{
	MyType* c = new MyType(t);
	return addChild(c);
}

template <class T,class E>
TreeNode<T,E>* TreeNode<T,E>::addChild(MyType* c)
{
	Assert(c->parent == NULL);
	Assert(c->nextSibling == NULL);
	c->parent=this;
	if(childLast) {
		childLast->nextSibling = c;
		childLast = c;
	}
	else {
		childLast = childFirst = c;
	}
	return c;
}

template <class T,class E>
TreeNode<T,E>* TreeNode<T,E>::getChild(int index) const
{
  MyType* c=getFirstChild();
  while(c && index > 0) {
    index--;
    c = c->getNextSibling();
  }
  Assert(c!=NULL && index==0);
  return c;
}

template <class T,class E>
int TreeNode<T,E>::getChildIndex(MyType* cquery) const
{
  int index=0;
  MyType* c=getFirstChild();
  while(c) {
    if(c==cquery) return index;
    index++;
    c = c->getNextSibling();
  }
  return -1;
}

template <class T,class E>
void TreeNode<T,E>::enumChildren(std::vector<MyType*>& children) const
{
  children.resize(0);
  MyType* c=getFirstChild();
  while(c) {
    children.push_back(c);
    c = c->getNextSibling();
  }
}

template <class T,class E>
void TreeNode<T,E>::setChildEdge(MyType* c,const E& e) const
{
  Assert(c->parent == this);
  c->parentEdge = e;
}

template <class T,class E>
void TreeNode<T,E>::clearChildren()
{
	MyType* c = childFirst;
	while(c) {
		MyType* next =c->nextSibling;
		delete c;
		c=next;
	}
	childFirst = childLast = NULL;
}

template <class T,class E>
void TreeNode<T,E>::detachChild(MyType* c)
{
	MyType* p = NULL;
	MyType* f = childFirst;
	while(f) {
		if(f == c) {
			if(p == NULL) childFirst = f->nextSibling;
			else p->nextSibling = f->nextSibling;
			if(f == childLast)
				childLast = p;
			c->nextSibling = NULL;
			c->parent = NULL;
			return;
		}
		p=f;
		f=f->nextSibling;
	}
	FatalError("TreeNode::detatchChild(): Error, child does not exist!");
}

template <class T,class E>
void TreeNode<T,E>::eraseChild(MyType* c)
{
	detachChild(c);
	delete c;
}

template <class T,class E>
bool TreeNode<T,E>::hasDescendent(const MyType* d) const
{
	return d->hasAncestor(this);
}

template <class T,class E>
bool TreeNode<T,E>::hasAncestor(const MyType* a) const
{
	if(a == parent) return true;
	if(!parent) return false;
	return parent->hasAncestor(a);
}

template <class T,class E>
void TreeNode<T,E>::DFS(Callback& f)
{
	f.Visit(this);
	if(f.Stop()) return;
	if(f.Descend(this)) {
		MyType* c=childFirst;
		while(c) {
		  if(f.ForwardEdge(this,c))
		    c->DFS(f);
		  if(f.Stop()) return;
		  c = c->nextSibling;
		}
	}
	f.PostVisit(this);
}

template <class T,class E>
void TreeNode<T,E>::BFS(Callback& f)
{
	std::queue<MyType*> queue;
	MyType* node;
	queue.push_back(this);
	while(!queue.empty()) {
		node=queue.front(); queue.pop();
		f.Visit(node);
		if(f.Stop()) return;
		if(f.Descend(this)) {
			MyType* c=node->childFirst;
			while(c) {
			  if(f.ForwardEdge(this,c))
			    queue.push_back(c);
			  c = c->nextSibling;
			}
		}
	}
	return;
}

template <class T,class E>
void TreeNode<T,E>::reRoot()
{
  if(parent) {
    MyType* p = parent;
    parent->detachChild(this);
    Assert(parent == NULL);
    p->reRoot();
    addChild(p);
    p->parentEdge = parentEdge;
  }
}

template <class T,class E>
TreeNode<T,E>* TreeNode<T,E>::LCA(const MyType* n)
{
  std::set<const MyType*> nAncestors;
  while(n) {
    nAncestors.insert(n);
    n = n->parent;
  }
  MyType* m = this;
  while(m) {
    if(nAncestors.count(m) != 0) return m;
    m = m->parent;
  }
  return NULL;
}

} // namespace Graph

#endif
