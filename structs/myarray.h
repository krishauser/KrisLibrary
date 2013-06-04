#ifndef MY_ARRAY_H
#define MY_ARRAY_H

#include <stdlib.h>

template <class type>
class myarray
{
public:
	myarray();
	myarray(int);
	myarray(int, const type&);
	myarray(const myarray<type>&);
	~myarray();

	const myarray<type>& operator = (const myarray&);
	bool empty() const;
	void clear();

	type& operator [] (int);

	int size() const;
	void resize(int);
	void resize(int, const type&);

	type* back();
	const type* back() const;
	type* front();
	const type* front() const;
	type* next(type*);
	const type* next(const type*) const;
	type* prev(type*);
	const type* prev(const type*) const;

	int find(const type&);
	bool contains(const type& t) { return (find(t)!=-1);}

private:
	type *_data;
	int _size;
};

template <class type>
myarray<type>::myarray()
:_data(NULL), _size(0)
{
}

template <class type>
myarray<type>::myarray(int n)
:_data(NULL), _size(0)
{
	resize(n);
}

template <class type>
myarray<type>::myarray(int n, const type& t)
:_data(NULL), _size(0)
{
	resize(n,t);
}

template <class type>
myarray<type>::myarray(const myarray<type>& l)
:_data(NULL), _size(0)
{
	*this = l;
}

template <class type>
myarray<type>::~myarray()
{
	clear();
}

template <class type>
void myarray<type>::resize(int n)
{
	clear();
	_size = n;
	_data = new type [n];
}

template <class type>
void myarray<type>::resize(int n, const type& t)
{
	clear();
	_size = n;
	_data = new type [n];
	for(int i=0; i<n; i++)
		_data[i] = t;
}

template <class type>
int myarray<type>::size() const
{
	return _size;
}

template <class type>
bool myarray<type>::empty() const
{
	return (_size == 0);
}

template <class type>
void myarray<type>::clear()
{
	if(_data)
		delete [] _data;
	_size = NULL;
}


template <class type>
type* myarray<type>::back()
{
	if(_size == 0)
		return NULL;
	return &_data[_size-1];
}


template <class type>
const type* myarray<type>::back() const
{
	if(_size == 0)
		return NULL;
	return &_data[_size-1];
}

template <class type>
type* myarray<type>::front()
{
	if(_size == 0)
		return NULL;
	return &_data[0];
}


template <class type>
const type* myarray<type>::front() const
{
	if(_size == 0)
		return NULL;
	return &_data[0];
}

template <class type>
type* myarray<type>::next(type* t)
{
	if(!t)
		return _data;
	if(t == back())
		return NULL;
	return t+1;
}

template <class type>
const type* myarray<type>::next(const type* t) const
{
	if(!t)
		return _data;
	if(t == back())
		return NULL;
	return t+1;
}

template <class type>
type* myarray<type>::prev(type* t)
{
	if(!t)
		return back();
	if(t == _data)
		return NULL;
	return t-1;
}

template <class type>
const type* myarray<type>::prev(const type* t) const
{
	if(!t)
		return back();
	if(t == _data)
		return NULL;
	return t-1;
}


template <class type>
type& myarray<type>::operator [] (int i)
{
	if(i<0 || i>=_size)
		abort();
	
	return _data[i];
}

template <class type>
const myarray<type>& myarray<type>::operator = (const myarray<type>& l)
{
	resize(l._size);
	for(int i=0; i<_size; i++)
		_data[i] = l.data[i];
	return *this;
}

template <class type>
int myarray<type>::find(const type& t)
{
	for(int i=0; i<_size; i++)
		if(t == _data[i])
			return i;
	return -1;
}

#endif
