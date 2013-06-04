#ifndef BASIC_MYLIST_H
#define BASIC_MYLIST_H

#include <stdlib.h>
#include <assert.h>

/* mylist -- doubly linked list template class
class mylist
{
public:
	mylist();
	mylist(int);
	mylist(int, const type&);
	mylist(const mylist<type>&);
	~mylist();

	const mylist<type>& operator = (const mylist&);
	void append(mylist&);						//note: empties the second list
	bool empty() const;
	void clear();

	int size() const;
	void resize(int);
	void resize(int, const type&);

	type* push_back();
	type* push_back(const type&);
	type* push_front();
	type* push_front(const type&);

	void pop_front();
	void pop_front(type&);
	void pop_back();
	void pop_back(type&);

	type* erase(type*);							//note: returns the temp->prev (for easy list iteration)
	void insert(type*, const type&);			//note: inserts a new node after the specified node

	type* back();
	const type* back() const;
	type* front();
	const type* front() const;
	type* next(const type*) const;
	type* prev(const type*) const;

	type* get_nth(int n);
	type* find_node(const type& t) const;		//returns NULL if can't find it
	int find(const type& t) const;				//returns -1 if can't find it
	int find(const type* t) const;				//returns -1 if can't find it
	bool contains(const type& t) const;
	bool contains(const type* t) const;
*/

template <class type>
class mylist
{
public:
	mylist()
	:_front(NULL), _back(NULL), num_nodes(0) {
	}
	mylist(int n)
	:_front(NULL), _back(NULL), num_nodes(0)  {
		resize(n);
	}
	mylist(int n, const type& t)
	:_front(NULL), _back(NULL), num_nodes(0)  {
		resize(n,t);
	}
	mylist(const mylist<type>& l)
	:_front(NULL), _back(NULL), num_nodes(0)  {
		*this = l;
	}

	virtual ~mylist() {
		clear();
	}

	bool empty() const {
		return (num_nodes == 0);
	}
	void clear() {
		type_node* temp=_front, *next;
		while(temp)
		{
			next = temp->next;
			delete temp;
			temp = next;
		}
		_front = NULL;
		_back = NULL;
		num_nodes = 0;
	}


	int size() const { 
		return num_nodes;
	}
	void resize(int n) {
		int dest=n;
		n = n-num_nodes;
		if(n<0) {		//resize smaller
			while(n++)
				pop_back();
		}
		else {		//resize bigger
			while(n--)
				push_back();
		}
		assert(num_nodes == dest);
	}
	void resize(int n, const type& t) {
		int dest=n;
		n = n-num_nodes;
		if(n<0) {		//resize smaller
			while(n++)
				pop_back();
		}
		else {		//resize bigger
			while(n--)
				push_back(t);
		}
		assert(num_nodes == dest);
	}

	void append(mylist<type>& l) {
		_back->next = l._front;
		_back->next->prev = _back;
		_back = l._back;
		num_nodes = num_nodes + l.num_nodes;
		l._back = NULL;
		l._front = NULL;
		l.clear();
	}

	type* push_back() {
		type_node* temp=new type_node;
		temp->prev = _back;
		temp->next = NULL;
		if(!_back)
			_front = temp;
		else
			_back->next = temp;
		_back = temp;
		num_nodes++;
		return &_back->t;
	}
	type* push_back(const type& t) {
		type_node* temp=new type_node;
		temp->t = t;
		temp->prev = _back;
		temp->next = NULL;
		if(!_back)
			_front = temp;
		else
			_back->next = temp;
		_back = temp;
		num_nodes++;
		return &_back->t;
	}

	type* push_front() {
		type_node* temp=new type_node;
		temp->next = _front;
		temp->prev = NULL;
		if(!_front)
			_back = temp;
		else
			_front->prev = temp;
		_front = temp;
		num_nodes++;
		return &_front->t;
	}
	type* push_front(const type& t) {
		type_node* temp=new type_node;
		temp->t = t;
		temp->next = _front;
		temp->prev = NULL;
		if(!_front)
			_back = temp;
		else
			_front->prev = temp;
		_front = temp;
		num_nodes++;
		return &_front->t;
	}


	void pop_front() {
		assert(_front != NULL);
		num_nodes--;
		type_node* temp = _front;
		_front = temp->next;
		if(_front)
			_front->prev = NULL;
		else
			_back = NULL;
		delete temp;
	}
	void pop_front(type& t) {
		assert(_front != NULL);
		num_nodes--;
		type_node* temp = _front;
		_front = temp->next;
		if(_front)
			_front->prev = NULL;
		else
			_back = NULL;
		t = temp->t;
		delete temp;
	}
	void pop_back() {
		assert(_back != NULL);
		num_nodes--;
		type_node* temp = _back;
		_back = temp->prev;
		if(_back)
			_back->next = NULL;
		else
			_front = NULL;
		delete temp;
	}
	void pop_back(type& t) {
		assert(_back != NULL);
		num_nodes--;
		type_node* temp = _back;
		_back = temp->prev;
		if(_back)
			_back->next = NULL;
		else
			_front = NULL;
		t = temp->t;
		delete temp;
	}

	type* erase(type* t) {
		if(num_nodes == 0 || !t)
			abort();

		type_node* temp = (type_node*)t;
		if(temp == _front)
		{
			_front = temp->next;
		}
		if(temp == _back)
		{
			_back = temp->prev;
		}

		if(temp->prev)
			temp->prev->next = temp->next;
		if(temp->next)
			temp->next->prev = temp->prev;

		type_node* prev = temp->prev;

		num_nodes--;
		delete temp;
		return &prev->t;
	}

	void insert(type* nt, const type& t) {
		type_node* node = (type_node*)nt, *temp = new type_node;

		temp->t = t;
		temp->prev = node;

		if(node == _back)
			_back = temp;

		if(node)
		{
			temp->next = node->next;
			node->next = temp;
		}
		else 					//call to null lets you insert before the front
		{
			temp->next = _front;
			_front = temp;
		}

		if(temp->next)
			temp->next->prev = temp;

		num_nodes++;
	}

	type* back(){
		if(!_back)
			return NULL;
		return &_back->t;
	}
	const type* back() const {
		if(!_back)
			return NULL;
		return &_back->t;
	}
	type* front() {
		if(!_front)
			return NULL;
		return &_front->t;
	}
	const type* front() const {
		if(!_front)
			return NULL;
		return &_front->t;
	}
	type* next(const type* t) const {
		if(!t)
			return (_front ? &_front->t : NULL);
		type_node* node = (type_node*)t;
		if(node->next)
			return &node->next->t;
		else
			return NULL;
	}
	type* prev(const type* t) const {
		if(!t)
			return (_back ? &_back->t : NULL);
		type_node* node = (type_node*)t;
		if(node->prev)
			return &node->prev->t;
		else
			return NULL;
	}

	type* get_nth(int n) const {
		int i = 0;
		type_node* temp = _front;
		while(temp) {
			if(i == n)
				return &temp->t;
			temp = temp->next;
			i++;
		}
		return NULL;
	}

	type* find_node(const type& t) const {
		int i=0;
		type_node* temp = _front;
		while(temp)
		{
			if(temp->t == t)
				return &temp->t;
			temp = temp->next;
		}
		return NULL;
	}
	int find(const type& t) const {
		int i=0;
		type_node* temp = _front;
		while(temp)
		{
			if(temp->t == t)
				return i;
			i++;
			temp = temp->next;
		}
		return -1;
	}
	int find(const type* t) const {
		int i=0;
		type_node* temp = _front;
		while(temp)
		{
			if(&temp->t == t)
				return i;
			i++;
			temp = temp->next;
		}
		return -1;
	}
	bool contains(const type& t) const {
		return (find_node(t) != NULL);
	}
	bool contains(const type* t) const {
		return (find(t) != -1);
	}

	const mylist<type>& operator = (const mylist<type>&l) {
	resize(l.num_nodes);
	type_node* temp = _front;
	type_node* temp2 = l._front;
	while(temp)
	{
		temp->t = temp2->t;
		temp = temp->next;
		temp2 = temp2->next;
	}
	return *this;
}


protected:
	struct type_node
	{
		type t;
		type_node *next, *prev;
	};
	type_node *_front, *_back;
	int num_nodes;
};






/* myindexedlist -- doubly linked list template class with indexing and caching
NOTE: is occasionally incompatible with normal list operations.  Call reset() between an operation and [] access
class myindexedlist : public mylist
{
public:
	void reset();
	type& operator [] (int);
}
*/

template <class type>
class myindexedlist : public mylist<type>
{
public:
	typedef typename mylist<type>::type_node type_node;

	myindexedlist()
	:mylist<type>(), _last(NULL), last_index(-1) {
	}
	myindexedlist(int n)
	:mylist<type>(n), _last(NULL), last_index(-1) {
	}
	myindexedlist(int n, const type& t)
	:mylist<type>(n, t), _last(NULL), last_index(-1) {
	}
	myindexedlist(const mylist<type>& l)
	:mylist<type>(l), _last(NULL), last_index(-1) {
	}

	void reset() {
		_last = NULL;
		last_index = -1;
	}

	type& operator [] (int i) {
		if(i<0 || i>=num_nodes)
			abort();
		
		int front_dist=i, back_dist = num_nodes-1-i;
		int last_dist=(_last ? i-last_index : num_nodes);

		type_node* t;
		if(front_dist <= back_dist)
		{
			if(abs(last_dist) < front_dist)
				t = seek(_last, last_dist);
			else
				t = seek(_front, front_dist);
		}
		else
		{
			if(abs(last_dist) < back_dist)
				t = seek(_last, last_dist);
			else
				t = seek(_back, -back_dist);
		}
		last_index = i;
		_last = t;

		return t->t;
	}

private:
	type_node *_last;
	int last_index;

	type_node* seek_backward(type_node* t, int n) {
		while(n)
		{
			t = t->prev;
			n--;
		}
		return t;
	}
	type_node* seek_forward(type_node* t, int n) {
		while(n)
		{
			t = t->next;
			n--;
		}
		return t;
	}
	type_node* seek(type_node* t, int n) {
		if(n>0)
			return seek_forward(t,n);
		else if(n<0)
			return seek_backward(t,-n);
		return t;
	}
};

#endif
