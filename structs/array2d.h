#ifndef ARRAY2D_H
#define ARRAY2D_H

#include <KrisLibrary/File.h>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/utils/IntPair.h>
#include <KrisLibrary/utils/indexing.h>
#include <iosfwd>

/** @brief A two-dimensional m x n array.
 *
 * Indices are in the range [0,m) x [0,n).  Elements can be accessed with the
 * (i,j) operator, or with the (IntPair) operator.  They are stored in memory
 * in row-major order.  
 *
 * The array is sized with initialize(m,n) or resize(m,n).
 * The difference is that initialize always deallocates previously allocated
 * memory, and resize only deallocates it if the new size is greater
 * than the established capacity (capacity >= m*n).  Capacity can be set
 * using the reserve() method, but only if the array is empty.  The existing
 * elements are not maintained in any particular order upon a resize.
 *
 * If the optional initVal parameter is passed to initialize or resize,
 * all elements are set to initVal after resizing.
 *
 * The iterator class allows fast iteration of all or some elements.  A
 * range of the array can be accessed using the Range2Indices class.
 */
template <class T>
class Array2D
{
public:
  Array2D();
  Array2D(int m,int n);
  Array2D(int m,int n,const T& initVal);
  Array2D(const Array2D<T>& rhs);
  Array2D(Array2D<T>&& rhs);
  ~Array2D();
  
  inline T& operator()(int i,int j) { Assert(i>=0&&i<m); Assert(j>=0&&j<n); return items[i*n+j]; }
  inline const T& operator()(int i,int j) const { Assert(i>=0&&i<m); Assert(j>=0&&j<n); return items[i*n+j]; }
  inline T& operator()(const IntPair& t) { return operator()(t.a,t.b); }
  inline const T& operator()(const IntPair& t) const { return operator()(t.a,t.b); }
  Array2D<T>& operator =(const Array2D<T>& rhs);
  Array2D<T>& operator =(Array2D<T>&& rhs);
  
  bool Read(File& f);
  bool Write(File& f) const;
  
  inline int numRows() const { return m; }
  inline int numCols() const { return n; }
  inline IntPair size() const { return IntPair(m,n); }
  inline bool empty() const { return m==0&&n==0; }
  void initialize(int m,int n);
  void initialize(int m,int n,const T& initVal);
  void resize(int m,int n);
  void resize(int m,int n,const T& initVal);
  void reserve(int numItems);
  void clear();
  
  bool find(const T& item,int& i,int& j) const;
  bool find(const T& item,IntPair& t) const { return find(item,t.a,t.b); }
  inline bool contains(const T& item) const { int i,j; return find(item,i,j); }
  void set(const T& item);
  void set(const Array2D<T>&);
  void swap(Array2D<T>&);
  inline T* getData() const { return items; }
  inline T* getRowData(int i) const { return &items[i*n]; }

  class iterator
  {
  public:
    explicit iterator(const Array2D<T>* array);
    explicit iterator(const Array2D<T>* array,int invalid);
    explicit iterator(const Array2D<T>* array,const Stripe2Indices& range);
    explicit iterator(const Array2D<T>* array,const Stripe2Indices& range,int invalid);
    iterator(const iterator& rhs);
    inline iterator& operator ++() { ++it; return *this; }
    inline iterator& operator --() { --it; return *this; }
    inline iterator& operator +=(int skip) { it+=skip; return *this; }
    inline iterator& operator -=(int skip) { it-=skip; return *this; }
    inline void incFirst(int skip=1) { it.incFirst(skip); }
    inline void incSecond(int skip=1) { it.incSecond(skip); }
    inline T& operator*() { return array->getData()[*it]; }
    iterator& operator = (const iterator& rhs);
    inline bool operator == (const iterator& rhs) const { return it == rhs.it && array==rhs.array; }
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    inline bool operator < (const iterator& rhs) const { return it<rhs.it; }
    IntPair getElement() const {
      //translate to original coordinates
      IntPair i=it.getElement();
      int ibase=range.base/range.istride;
      int jbase=range.base/range.jstride;
      int istride=range.istride/(range.jsize*range.jstride);
      int jstride=range.jstride;
      return IntPair(i.a*istride+ibase,i.b*jstride+jbase);
    }
    //private:
    const Array2D<T>* array;
    Stripe2Indices range;
    Stripe2Indices::iterator it;
  };
  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(this,-1); }
  iterator begin(const Range2Indices& range) const { return iterator(this,range); }
  iterator end(const Range2Indices& range) const { return iterator(this,range,-1); }
  
  //READ ONLY
  int m,n;
protected:
  T* items;
  int capacity;
};

    

template <class T>
Array2D<T>::Array2D()
:m(0),n(0),items(0),capacity(0)
{}

template <class T>
Array2D<T>::Array2D(int _m,int _n)
:m(0),n(0),items(0),capacity(0)
{
	initialize(_m,_n);
}

template <class T>
Array2D<T>::Array2D(int _m,int _n,const T& initVal)
:m(0),n(0),items(0),capacity(0)
{
	initialize(_m,_n,initVal);
}

template <class T>
Array2D<T>::Array2D(const Array2D<T>& rhs)
:m(0),n(0),items(0),capacity(0)
{
	set(rhs);
}

template <class T>
Array2D<T>::Array2D(Array2D<T>&& rhs)
:m(rhs.m),n(rhs.n),items(rhs.items),capacity(rhs.capacity)
{
  //prevent deletion
  rhs.items = 0;
}

template <class T>
Array2D<T>::~Array2D()
{
	clear();
}

template <class T>
Array2D<T>& Array2D<T>::operator =(const Array2D<T>& rhs)
{
	set(rhs);
	return *this;
}

template <class T>
Array2D<T>& Array2D<T>::operator =(Array2D<T>&& rhs)
{
  m = rhs.m;
  n = rhs.n;
  items = rhs.items;
  capacity = rhs.capacity;
  //prevent deletion
  rhs.items = 0;
  return *this;
}

template <class T>
bool Array2D<T>::Read(File& f)
{
	if(!ReadFile(f,m)) return false;
	if(!ReadFile(f,n)) return false;
	initialize(m,n);
	if(!ReadArrayFile(f,items,m*n)) return false;
	return true;
}

template <class T>
bool Array2D<T>::Write(File& f) const
{
	if(!WriteFile(f,m)) return false;
	if(!WriteFile(f,n)) return false;
	if(!WriteArrayFile(f,items,m*n)) return false;
	return true;
}

template <class T>
void Array2D<T>::initialize(int _m,int _n)
{
	clear();
	m=_m;
	n=_n;
	capacity=m*n;
	items=new T[capacity];
}

template <class T>
void Array2D<T>::initialize(int _m,int _n,const T& initVal)
{
	clear();
	m=_m;
	n=_n;
	capacity=m*n;
	items=new T[capacity];
	set(initVal);
}

template <class T>
void Array2D<T>::resize(int _m,int _n)
{
	if(_m*_n > capacity)
		initialize(_m,_n);
	m=_m;
	n=_n;
}

template <class T>
void Array2D<T>::resize(int _m,int _n,const T& initVal)
{
	if(_m*_n > capacity)
		initialize(_m,_n);
	m=_m;
	n=_n;
	set(initVal);
}

template <class T>
void Array2D<T>::reserve(int cap)
{
  if(cap > capacity) {
    if(m*n != 0)
      FatalError("TODO: copy elements in Array2D resize/reserve");
    if(items) delete [] items;
    capacity = cap;
    items = new T[capacity];
  }
}

template <class T>
void Array2D<T>::clear()
{
  if(items) {
    delete [] items;
    items = NULL;
  }
  m=n=0;
  capacity=0;
}

template <class T>
bool Array2D<T>::find(const T& item,int& i,int& j) const
{
	for(int p=0;p<m;p++)
		for(int q=0;q<n;q++)
			if(operator()(p,q)==item) {
				i=p; j=q;
				return true;
			}
	return false;
}

template <class T>
void Array2D<T>::set(const T& item)
{
	for(int i=0;i<m*n;i++) items[i]=item;
}

template <class T>
void Array2D<T>::set(const Array2D<T>& rhs)
{
	resize(rhs.m,rhs.n);
	for(int i=0;i<m*n;i++) items[i]=rhs.items[i];
}

template <class T>
void Array2D<T>::swap(Array2D<T>& b)
{
  int tempm=m,tempn=n;
  T* tempitems=items;
  int tempcap=capacity;
  m=b.m; n=b.n;
  items=b.items;
  capacity=b.capacity;
  b.m=tempm; b.n=tempn;
  b.items=tempitems;
  b.capacity=tempcap;
}


template <class T>
std::ostream& operator << (std::ostream& out,const Array2D<T>& a)
{
  out<<a.m<<" "<<a.n<<std::endl;
  for(int i=0;i<a.m;i++) {
    for(int j=0;j<a.n;j++) {
      out<<a(i,j);
      if(j+1 < a.n) out<<" ";
    }
    if(i+1 < a.m) out<<std::endl;
  }
  return out;
}

template <class T>
std::istream& operator >> (std::istream& in,Array2D<T>& a)
{
  int m,n;
  in>>m>>n;
  if(in.bad()) return in;
  Assert(m>=0 && n>=0);
  a.resize(m,n);
  for(int i=0;i<m;i++) {
    for(int j=0;j<n;j++) {
      in>>a(i,j);
    }
  }
  return in;
}

template <class T>
Array2D<T>::iterator::iterator(const Array2D<T>* _array)
  :array(_array),range(_array->m,_array->n),it(&range)
{}

template <class T>
Array2D<T>::iterator::iterator(const Array2D<T>* _array,int invalid)
  :array(_array),range(_array->m,_array->n),it(range.end())
{}

template <class T>
Array2D<T>::iterator::iterator(const Array2D<T>* _array,const Stripe2Indices& _range)
  :array(_array),range(_range),it(&range)
{}

template <class T>
Array2D<T>::iterator::iterator(const Array2D<T>* _array,const Stripe2Indices& _range,int invalid)
  :array(_array),range(_range),it(range.end())
{}

template <class T>
Array2D<T>::iterator::iterator(const iterator& rhs)
  :array(rhs.array),range(rhs.range),it(rhs.it)
{
  it.stripe = &range;   //maintain a valid pointer over the iterator's lifetime
}

template <class T>
typename Array2D<T>::iterator& Array2D<T>::iterator::operator = (const iterator& rhs)
{
  it = rhs.it;
  array = rhs.array;
  range = rhs.range;
  it.stripe = &range;   //maintain a valid pointer over the iterator's lifetime
  return *this;
}

namespace std {

  template <class T>
  void swap(Array2D<T>& a,Array2D<T>&b)
  {
    a.swap(b);
  }

} //namespace std

#endif
