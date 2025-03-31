#ifndef REDBLACK_H
#define REDBLACK_H

#include <KrisLibrary/Logger.h>
#include <iostream>
#include <assert.h>
#include <KrisLibrary/utils/cmp_func.h>

namespace RedBlack {

enum Color { Black, Red };
//defines the match type for the lookup function
enum Lookup { None, Equal,GTEQ,LTEQ,Less,Greater,Next,Prev};

template <class T>
struct WalkCallback {
  enum Visit { PreOrder, PostOrder, EndOrder, Leaf };
  virtual ~WalkCallback() {}
  virtual void Visit(const T& key, Visit visit, int level) {}
};

template <class T>
class Node
{
public:
  typedef Node<T> MyT;

  Node();
  Node(const T&);
  ~Node();
  void detatch();
  bool isValid() const;
  int count_black();
  void dumptree(std::ostream& out,int n);
  MyT* successor() const;  //Return a pointer to the smallest key greater than x
  MyT* predecessor() const;  //Return a pointer to the largest key less than x
  void walk(WalkCallback<T>& callback,int level);

  MyT *left;		/* Left down */
  MyT *right;		/* Right down */
  MyT *up;		/* Up */
  T key;	

  Color color;
};

template <class T,class Cmp=std::cmp_func<T> >
class Tree
{
public:
  typedef Tree<T> MyT;
  typedef Node<T> NodeT;

  struct iterator
  { 
    inline iterator() : curp(NULL) {}
    inline iterator(NodeT* n) { curp=n; }
    inline iterator(const iterator& i) : curp(i.curp) {}
    inline operator const T* () const { assert(curp); return &curp->key; }
    inline const T* operator->() const  { assert(curp); return &curp->key; }
    inline void operator++() { if(curp) curp=curp->successor(); }
    inline void operator--() { if(curp) curp=curp->predecessor(); }
    inline void operator++(int) { if(curp) curp=curp->successor(); }
    inline void operator--(int) { if(curp) curp=curp->predecessor(); }
    inline bool operator == (const iterator& i) const { return i.curp==curp; }
    inline bool operator != (const iterator& i) const { return i.curp!=curp; }
    inline operator bool() const { return curp!=NULL; }
    inline bool operator !() const { return curp==NULL; }
    inline bool end() const { return curp==NULL; }
    
    NodeT* curp; 
  }; 

  Tree();
  ~Tree();
  bool checkValid() const;
  inline bool empty() const { return root==NULL; }
  void clear();
  bool erase(const T&);
  inline void erase(const iterator& x) { _erase(x.curp); }
  inline iterator find(const T& key) const { return iterator(_find(key)); }
  inline iterator insert(const T& key) { return iterator(_insert(key)); }
  iterator lookup(Lookup type, const T&);
  void walk(WalkCallback<T>& callback);
  iterator begin() const; 
  inline iterator end() const { return iterator(); }
  iterator front() const;
  iterator back() const;

  Cmp cmpFunc;

private:
  NodeT* _insert(const T&);
  NodeT* _find(const T&) const;
  NodeT* _lookup(Lookup, const T&) const;
  void _erase(NodeT* x);
  void _erase_fix(NodeT* x,NodeT* p);
  void _left_rotate(NodeT* x);
  void _right_rotate(NodeT* x);

  inline static void setColor(Node<T>* n,Color c) { if(n==NULL) assert(c==Black); else n->color=c; }
  inline static Color getColor(const Node<T>* n) { if(n==NULL) return Black; return n->color; }
  inline static bool hasColor(const Node<T>* n,Color c) { return c==getColor(n); }

  NodeT *root;
};






template <class T>
Node<T>::Node()
:left(NULL),right(NULL),up(NULL),color(Black)
{}

template <class T>
Node<T>::Node(const T& _key)
:left(NULL),right(NULL),up(NULL),key(_key),color(Black)
{}

template <class T>
Node<T>::~Node()
{
  if(left) delete left;
  if(right) delete right;
}

template <class T>
void Node<T>::detatch()
{
  left=right=up=NULL;
}

template <class T>
Node<T>* Node<T>::successor() const
{
	MyT *y;
  const MyT *x;

	if (right!=NULL)
	{
		/* If right is not NULL then go right one and
		** then keep going left until we find a node with
		** no left pointer.
		*/
		for (y=right; y->left!=NULL; y=y->left);
	}
	else
	{
		/* Go up the tree until we get to a node that is on the
		** left of its parent (or the root) and then return the
		** parent.
		*/
		y=up;
    x=this;
		while(y!=NULL && x==y->right)
		{
			x=y;
			y=y->up;
		}
	}
	return(y);
}

template <class T>
Node<T>* Node<T>::predecessor() const
{
	MyT *y;
  const MyT *x;

	if (left!=NULL)
	{
		/* If left is not NULL then go left one and
		** then keep going right until we find a node with
		** no right pointer.
		*/
		for (y=left; y->right!=NULL; y=y->right);
	}
	else
	{
		/* Go up the tree until we get to a node that is on the
		** right of its parent (or the root) and then return the
		** parent.
		*/
		y=up;
    x=this;
		while(y!=NULL && x==y->left)
		{
			x=y;
			y=y->up;
		}
	}
	return(y);
}

template <class T>
void Node<T>::walk(WalkCallback<T>& callback,int level)
{
	if (left==NULL && right==NULL)
	{
	  callback(key, WalkCallback<T>::Leaf, level);
	}
	else
	{
		callback(key, WalkCallback<T>::Preorder, level);
    if(left != NULL) left->walk(callback, level+1);
  	callback(key, WalkCallback<T>::Postorder, level);
    if(right != NULL) right->walk(callback, level+1);
		callback(key, WalkCallback<T>::Endorder, level);
	}
}

template <class T>
bool Node<T>::isValid() const
{
	if (color == Red)
	{
		if ((left && left->color != Black) && (right && right->color != Black))
		{
						//LOG4CXX_ERROR(KrisLibrary::logger(), "Children of red node not both black, x="<< (unsigned long)this);
			return false;
		}
	}

	if (left)
	{
		if (left->up != this)
		{
						//LOG4CXX_ERROR(KrisLibrary::logger(), "x->left->up != x, x="<< (unsigned long)this);
			return false;
		}

		if (!left->isValid())
			return false;
	}		

	if (right)
	{
		if (right->up != this)
		{
						//LOG4CXX_ERROR(KrisLibrary::logger(), "x->right->up != x, x="<< (unsigned long)this);
			return false;
		}

		if (!right->isValid())
			return false;
	}		
	return true;
}

template <class T>
int Node<T>::count_black()
{
 	int nleft= left ? left->count_black() : 1;
	int nright= right ? right->count_black() : 1;
	if (nleft==-1 || nright==-1)
		return(-1);

	if (nleft != nright)
	{
				//LOG4CXX_ERROR(KrisLibrary::logger(), "Black count not equal on left & right, x="<< (unsigned long)this);
		return(-1);
	}

	if (color == Black) nleft++;
	return(nleft);
}

template <class T>
void Node<T>::dumptree(std::ostream& out,int n)
{
	n++;
	out<<"Tree: "<<n<<" "<<(unsigned int)this<<
		": color="<<(color==Black? "Black" : "Red") <<", key="<<key<<
		", left="<<(unsigned int)left<<", right="<<(unsigned int)right<<std::endl;
	if(left) left->dumptree(out,n);
	if(right) right->dumptree(out,n);
}	


/*
** OK here we go, the balanced tree stuff. The algorithm is the
** fairly standard red/black taken from "Introduction to Algorithms"
** by Cormen, Leiserson & Rivest. 
**
** Basically a red/black balanced tree has the following properties:-
** 1) Every node is either red or black 
** 2) A leaf (NULL pointer) is considered black
** 3) If a node is red then its children are black
** 4) Every path from a node to a leaf contains the same no
**    of black nodes
**
** 3) & 4) above guarantee that the longest path (alternating
** red and black nodes) is only twice as long as the shortest
** path (all black nodes). Thus the tree remains fairly balanced.
*/

template <class T,class Cmp>
Tree<T,Cmp>::Tree()
:root(NULL)
{}

template <class T,class Cmp>
Tree<T,Cmp>::~Tree()
{
  clear();
}

template <class T,class Cmp>
void Tree<T,Cmp>::clear()
{
  if (root)	delete root;
  root=NULL;
}

template <class T,class Cmp>
bool Tree<T,Cmp>::erase(const T& key)
{
  NodeT *x=_find(key);
  if (x) {
    _erase(x);
    return true;
  }
  return false;
}

template <class T,class Cmp>
void Tree<T,Cmp>::walk(WalkCallback<T>& callback)
{
	_walk(root, callback);
}

template <class T,class Cmp>
typename Tree<T,Cmp>::iterator Tree<T,Cmp>::begin() const
{
  NodeT* start=root;
  //seek the minimum
  if(start) while(start->left!=NULL) start=start->left;
  return iterator(start);
}

template <class T,class Cmp>
typename Tree<T,Cmp>::iterator Tree<T,Cmp>::lookup(Lookup mode, const T& key)
{
  if (!root) return iterator(NULL);
  return iterator(_lookup(mode, key));
}

template <class T,class Cmp>
typename Tree<T,Cmp>::iterator Tree<T,Cmp>::front() const
{
  NodeT* start=root;
  //seek the minimum
  if(start) while(start->left!=NULL) start=start->left;
  return iterator(start);
}

template <class T,class Cmp>
typename Tree<T,Cmp>::iterator Tree<T,Cmp>::back() const
{
  NodeT* start=root;
  //seek the maximum
  if(start) while(start->right!=NULL) start=start->right;
  return iterator(start);
}

/* --------------------------------------------------------------------- */

/* Search for and if not found and insert is true, will add a new
** node in. Returns a pointer to the new node, or the node found
*/
template <class T,class Cmp>
Node<T>* Tree<T,Cmp>::_insert(const T& key)
{
  NodeT *x,*y,*z;
  int cmp;

  y=NULL; /* points to the parent of x */
  x=root;

  /* walk x down the tree */
  while(x!=NULL) {
      y=x;
      cmp=cmpFunc(key, x->key);

      if (cmp<0)
	x=x->left;
      else if (cmp>0)
	x=x->right;
      else //found the key
	return x;
  }

  z=new NodeT(key);
  if (z==NULL) {
    /* Whoops, no memory */
    return(NULL);
  }

	z->up=y;
	if (y==NULL)
	{
		root=z;
	}
	else
	{
		cmp=cmpFunc(z->key, y->key);
		if (cmp<0)
			y->left=z;
		else
			y->right=z;
	}

	z->left=NULL;
	z->right=NULL;

	/* color this new node red */
    setColor(z,Red);

	/* Having added a red node, we must now walk back up the tree balancing
	** it, by a series of rotations and changing of colors
	*/
	x=z;

	/* While we are not at the top and our parent node is red
	** N.B. Since the root node is garanteed black, then we
	** are also going to stop if we are the child of the root
	*/

	while(x != root && hasColor(x->up,Red))
	{
		/* if our parent is on the left side of our grandparent */
		if (x->up == x->up->up->left)
		{
			/* get the right side of our grandparent (uncle?) */
			y=x->up->up->right;
			if (hasColor(y,Red))
			{
				/* make our parent black */
				setColor(x->up,Black);
				/* make our uncle black */
				setColor(y,Black);
				/* make our grandparent red */
				setColor(x->up->up,Red);

				/* now consider our grandparent */
				x=x->up->up;
			}
			else
			{
				/* if we are on the right side of our parent */
				if (x == x->up->right)
				{
					/* Move up to our parent */
					x=x->up;
					_left_rotate(x);
				}

				/* make our parent black */
				setColor(x->up,Black);
				/* make our grandparent red */
				setColor(x->up->up,Red);
				/* right rotate our grandparent */
				_right_rotate(x->up->up);
			}
		}
		else
		{
			/* everything here is the same as above, but
			** exchanging left for right
			*/

			y=x->up->up->left;
			if (hasColor(y,Red))
			{
				setColor(x->up,Black);
				setColor(y,Black);
				setColor(x->up->up,Red);

				x=x->up->up;
			}
			else
			{
				if (x == x->up->left)
				{
					x=x->up;
					_right_rotate(x);
				}

				setColor(x->up,Black);
				setColor(x->up->up,Red);
				_left_rotate(x->up->up);
			}
		}
	}

	/* Set the root node black */
	setColor(root,Black);
	return z;
}

template <class T,class Cmp>
Node<T>* Tree<T,Cmp>::_find(const T& key) const
{
  NodeT* x=root;
  int cmp;

  /* walk x down the tree */
  while(x!=NULL) {
    cmp=cmpFunc(key, x->key);

    if (cmp<0)
      x=x->left;
    else if (cmp>0)
      x=x->right;
    else
      return x;
  }
  return NULL;
}

template <class T,class Cmp>
Node<T>* Tree<T,Cmp>::_lookup(Lookup mode, const T& key) const
{
	NodeT *x,*y;
	int cmp=0;
	int found=0;

	y=NULL; /* points to the parent of x */
	x=root;

	/* walk x down the tree */
	while(x!=NULL && found==0)
	{
		y=x;
		cmp=cmpFunc(key, x->key);

		if (cmp<0)
			x=x->left;
		else if (cmp>0)
			x=x->right;
		else
			found=1;
	}

	if (found && (mode==Equal || mode==GTEQ || mode==LTEQ))
		return(x);
	
	if (!found && (mode==Equal || mode==Next || mode==Prev))
		return(NULL);
	
	if (mode==GTEQ || (!found && mode==Greater))
	{
		if (cmp>0)
			return(y->successor());
		else
			return(y);
	}

	if (mode==LTEQ || (!found && mode==Less))
	{
		if (cmp<0)
			return(y->predecessor());
		else
			return(y);
	}

	if (mode==Next || (found && mode==Greater))
		return(x->successor());

	if (mode==Prev || (found && mode==Less))
		return(x->predecessor());
	
	/* Shouldn't get here unless mode=None */
	return(NULL);
}


/*
** Rotate our tree thus:-
**
**             X           left_rotate(X)--->            Y
**           /   \                                     /   \
**          A     Y     <---   right_rotate(Y)        X     C
**              /   \                               /   \
**             B     C                             A     B
**
** N.B. This does not change the ordering.
**
** We assume that neither X or Y is NULL
*/

template <class T,class Cmp>
void Tree<T,Cmp>::_left_rotate(NodeT *x)
{
	NodeT *y;

	assert(x!=NULL);
	assert(x->right!=NULL);

	y=x->right; /* set Y */

	/* Turn Y's left subtree into X's right subtree (move B)*/
	x->right = y->left;

	/* If B is not null, set it's parent to be X */
	if (y->left != NULL)
		y->left->up = x;

	/* Set Y's parent to be what X's parent was */
	y->up = x->up;

	/* if X was the root */
	if (x->up == NULL)
	{
		root=y;
	}
	else
	{
		/* Set X's parent's left or right pointer to be Y */
		if (x == x->up->left)
		{
			x->up->left=y;
		}
		else
		{
			x->up->right=y;
		}
	}

	/* Put X on Y's left */
	y->left=x;

	/* Set X's parent to be Y */
	x->up = y;
}

template <class T,class Cmp>
void Tree<T,Cmp>::_right_rotate(NodeT *y)
{
	NodeT *x;

	assert(y!=NULL);
	assert(y->left!=NULL);

	x=y->left; /* set X */

	/* Turn X's right subtree into Y's left subtree (move B) */
	y->left = x->right;

	/* If B is not null, set it's parent to be Y */
	if (x->right != NULL)
		x->right->up = y;

	/* Set X's parent to be what Y's parent was */
	x->up = y->up;

	/* if Y was the root */
	if (y->up == NULL)
	{
		root=x;
	}
	else
	{
		/* Set Y's parent's left or right pointer to be X */
		if (y == y->up->left)
		{
			y->up->left=x;
		}
		else
		{
			y->up->right=x;
		}
	}

	/* Put Y on X's right */
	x->right=y;

	/* Set Y's parent to be X */
	y->up = x;
}

template <class T,class Cmp>
void Tree<T,Cmp>::_erase(NodeT *z)
{
	NodeT *x, *y, *p;

	if (z->left == NULL || z->right == NULL)
		y=z;
	else
		y=z->successor();

	p = y->up;
	if (y->left != NULL)
		x=y->left;
	else
		x=y->right;
	if(x) x->up=p;
	if (p == NULL)
	{
		root=x;
	}
	else
	{
		if (y==p->left)
			p->left = x;
		else
			p->right = x;
	}
	y->detatch();

 	if (hasColor(y,Black))
		_erase_fix(x,p);

/* DELETED: don't want to be changing iterators around on erase!
  if(z!=y) {
    z->key = y->key;
  }
  delete y;
*/
  if(z!=y) {
    //sit y into z's place
    if(z->up) {
      if(z->up->left == z) z->up->left=y;
      else { assert(z->up->right == z); z->up->right=y; }
    }
    y->up=z->up;
    if(z->left) z->left->up = y;
    y->left=z->left;
    if(z->right) z->right->up = y;
    y->right=z->right;
    setColor(y,getColor(z));
    z->detatch();
  }

  delete z;
}

/* Restore the red-black properties after a delete */
template <class T,class Cmp>
void Tree<T,Cmp>::_erase_fix(NodeT *x,NodeT* p)
{
	NodeT *w;
	while (x!=root && hasColor(x,Black))
	{
		if (x==p->left)
		{
			w=p->right;
			if (hasColor(w,Red))
			{
				setColor(w,Black);
				setColor(p,Red);
				_left_rotate(p);
				w=p->right;
			}

			if (hasColor(w->left,Black) && hasColor(w->right,Black))
			{
				setColor(w,Red);
				x=p;
				p=x->up;
			}
			else
			{
				if (hasColor(w->right,Black))
				{
					setColor(w->left,Black);
					setColor(w,Red);
					_right_rotate(w);
					w=p->right;
				}


				setColor(w,getColor(p));
				setColor(p,Black);
				setColor(w->right,Black);
				_left_rotate(p);
				x=root;
			}
		}
		else
		{
			w=p->left;
			if (hasColor(w,Red))
			{
				setColor(w,Black);
				setColor(p,Red);
				_right_rotate(p);
				w=p->left;
			}

			if (hasColor(w->right,Black) && hasColor(w->left,Black))
			{
				setColor(w,Red);
				x=p;
				p=x->up;
			}
			else
			{
				if (hasColor(w->left,Black))
				{
					setColor(w->right,Black);
					setColor(w,Red);
					_left_rotate(w);
					w=p->left;
				}

				setColor(w,getColor(p));
				setColor(p,Black);
				setColor(w->left,Black);
				_right_rotate(p);
				x=root;
			}
		}
	}

	setColor(x,Black);
}


template <class T,class Cmp>
bool Tree<T,Cmp>::checkValid() const
{
	if (root->up!=NULL)
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(), "Root up pointer not NULL" <<"\n");
		root->dumptree(std::cerr,0);
		return false;
	}

	if (!root->isValid())
	{
	  root->dumptree(std::cerr,0);
	  return false;
	}

	if (root->count_black()==-1)
	{
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error in black count"<<"\n");
	  root->dumptree(std::cerr,0);
	  return false;
	}

	return true;
}

} //namespace RedBlack

#endif
