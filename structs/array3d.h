#ifndef ARRAY3D_H
#define ARRAY3D_H

#include <KrisLibrary/File.h>
#include <KrisLibrary/utils.h>
#include <KrisLibrary/utils/IntTriple.h>
#include <KrisLibrary/utils/indexing.h>
#include <KrisLibrary/errors.h>
#include <iosfwd>

/** @brief A three-dimensional m x n x parray.
 *
 * Indices are in the range [0,m) x [0,n) x [0,p).  Elements can be accessed
 * with the (i,j,k) operator, or with the (IntTriple) operator.  Element
 * i,j,k is in memory at location i*n*p + j*p + k.
 *
 * The array is sized with initialize(m,n,p) or resize(m,n,p).
 * The difference is that initialize always deallocates previously allocated
 * memory, and resize only deallocates it if the new size is greater
 * than the established capacity (capacity >= m*n*p).  Capacity can be set
 * using the reserve() method, but only if the array is empty.  The existing
 * elements are not maintained in any particular order upon a resize.
 *
 * If the optional initVal parameter is passed to initialize or resize,
 * all elements are set to initVal after resizing.
 *
 * The iterator class allows fast iteration of all or some elements.  A
 * range of the array can be accessed using the Range3Indices class.
 */
template <class T>
class Array3D
{
 public:
  Array3D();
  Array3D(int m,int n,int p);
  Array3D(int m,int n,int p,const T& initVal);
  Array3D(const Array3D<T>& rhs);
  Array3D(Array3D<T>&& rhs);
  ~Array3D();
  
  inline T& operator()(int i,int j,int k) { Assert(i>=0&&i<m); return items[(i*n+j)*p+k]; }
  inline const T& operator()(int i,int j,int k) const { return items[(i*n+j)*p+k]; }
  inline T& operator()(const IntTriple& t) { return operator()(t.a,t.b,t.c); }
  inline const T& operator()(const IntTriple& t) const { return operator()(t.a,t.b,t.c); }
  Array3D<T>& operator =(const Array3D<T>& rhs);
  Array3D<T>& operator =(Array3D<T>&& rhs);
  
  bool Read(File& f);
  bool Write(File& f) const;
  
  inline int dim1() const { return m; }
  inline int dim2() const { return n; }
  inline int dim3() const { return p; }
  inline IntTriple size() const { return IntTriple(m,n,p); }
  inline bool empty() const { return m==0&&n==0&&p==0; }
  void initialize(int m,int n,int p);
  void initialize(int m,int n,int p,const T& initVal);
  void resize(int m,int n,int p);
  void resize(int m,int n,int p,const T& initVal);
  void reserve(int cap);
  void clear();
  
  bool find(const T& item,int& i,int& j,int& k) const;
  bool find(const T& item,IntTriple& t) const { return find(item,t.a,t.b,t.c); }
  inline bool contains(const T& item) const { int i,j,k; return find(item,i,j,k); }
  void set(const T& item);
  void set(const Array3D<T>&);
  void swap(Array3D<T>&);
  inline T* getData() const { return items; }
  
  class iterator
  {
  public:
    explicit iterator(const Array3D<T>* array);
    explicit iterator(const Array3D<T>* array,int invalid);
    explicit iterator(const Array3D<T>* array,const Stripe3Indices& range);
    explicit iterator(const Array3D<T>* array,const Stripe3Indices& range,int invalid);
    iterator(const iterator& rhs);
    inline iterator& operator ++() { ++it; return *this; }
    inline iterator& operator --() { --it; return *this; }
    inline iterator& operator +=(int skip) { it+=skip; return *this; }
    inline iterator& operator -=(int skip) { it-=skip; return *this; }
    inline void incFirst(int skip=1) { it.incFirst(skip); }
    inline void incSecond(int skip=1) { it.incSecond(skip); }
    inline void incThird(int skip=1) { it.incThird(skip); }
    inline T& operator*() { return array->getData()[*it]; }
    const iterator& operator = (const iterator& rhs);
    inline bool operator == (const iterator& rhs) const { return it == rhs.it && array==rhs.array; }
    inline bool operator != (const iterator& rhs) const { return !operator==(rhs); }
    inline bool operator < (const iterator& rhs) const { return it<rhs.it; }
    IntTriple getElement() const {
      //translate to original coordinates
      IntTriple i=it.getElement();
      //range.base = ibase*n*p + jbase*p + kbase;
      //range.istride = istride*n*p;
      //range.jstride = jstride*p;
      //range.kstride = kstride;
      int kstride = range.kstride;
      int jstride = range.jstride/array->p;
      int istride = range.istride/(array->p*array->n);
      div_t d=div(range.base,array->p);
      int kbase=d.rem;
      d=div(d.quot,array->n);
      int jbase=d.rem;
      int ibase=d.quot;
      return IntTriple(i.a*istride+ibase,i.b*jstride+jbase,i.c*kstride+kbase);
    }
    
    //private:
    const Array3D<T>* array;
    Stripe3Indices range;
    Stripe3Indices::iterator it;
  };
  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(this,-1); }
  iterator begin(const Range3Indices& range) const { return iterator(this,Stripe3Indices(m,n,p,range)); }
  iterator end(const Range3Indices& range) const{ return iterator(this,Stripe3Indices(m,n,p,range),-1); }

  //READ ONLY
  int m,n,p;
 protected:
  T* items;
  int capacity;
};

template <class T>
std::istream& operator >> (std::istream& in,Array3D<T>& a)
{
  int m,n,p;
  in>>m>>n>>p;
  if(!in) return in;
  Assert(m >= 0 && n >= 0 && p >= 0);
  a.resize(m,n,p);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      for(int k=0;k<p;k++)
        in>>a(i,j,k);
  return in;
}

template <class T>
std::ostream& operator << (std::ostream& out,const Array3D<T>& a)
{
  out<<a.m<<" "<<a.n<<" "<<" "<<a.p<<std::endl;
  for(int i=0;i<a.m;i++) {
    for(int j=0;j<a.n;j++) {
      for(int k=0;k<a.p;k++)
        out<<a(i,j,k)<<" ";
      out<<std::endl;
    }
  }
  return out;
}

template <class T>
Array3D<T>::Array3D()
  :m(0),n(0),p(0),items(0),capacity(0)
{}

template <class T>
Array3D<T>::Array3D(int _m,int _n,int _p)
:m(0),n(0),p(0),items(0),capacity(0)
{
	initialize(_m,_n,_p);
}

template <class T>
Array3D<T>::Array3D(int _m,int _n,int _p,const T& initVal)
:m(0),n(0),p(0),items(0),capacity(0)
{
	initialize(_m,_n,_p,initVal);
}

template <class T>
Array3D<T>::Array3D(const Array3D<T>& rhs)
:m(0),n(0),p(0),items(0),capacity(0)
{
	set(rhs);
}

template <class T>
Array3D<T>::Array3D(Array3D<T>&& rhs)
{
  m = rhs.m;
  n = rhs.n;
  p = rhs.p;
  items = rhs.items;
  capacity = rhs.capacity;
  //prevent deletion
  rhs.items = 0;
}

template <class T>
Array3D<T>::~Array3D()
{
	clear();
}

template <class T>
Array3D<T>& Array3D<T>::operator =(const Array3D<T>& rhs)
{
	set(rhs);
	return *this;
}

template <class T>
Array3D<T>& Array3D<T>::operator =(Array3D<T>&& rhs)
{
  m = rhs.m;
  n = rhs.n;
  p = rhs.p;
  items = rhs.items;
  capacity = rhs.capacity;
  //prevent deletion
  rhs.items = 0;
  return *this;
}

template <class T>
bool Array3D<T>::Read(File& f)
{
	if(!ReadFile(f,m)) return false;
	if(!ReadFile(f,n)) return false;
	if(!ReadFile(f,p)) return false;
	initialize(m,n,p);
	if(!ReadArrayFile(f,items,m*n*p)) return false;
	return true;
}

template <class T>
bool Array3D<T>::Write(File& f) const
{
	if(!WriteFile(f,m)) return false;
	if(!WriteFile(f,n)) return false;
	if(!WriteFile(f,p)) return false;
	if(!WriteArrayFile(f,items,m*n*p)) return false;
	return true;
}

template <class T>
void Array3D<T>::initialize(int _m,int _n,int _p)
{
	clear();
	m=_m;
	n=_n;
	p=_p;
	capacity = m*n*p;
	items=new T[capacity];
}

template <class T>
void Array3D<T>::initialize(int _m,int _n,int _p,const T& initVal)
{
	clear();
	m=_m;
	n=_n;
	p=_p;
	capacity = m*n*p;
	items=new T[capacity];
	set(initVal);
}

template <class T>
void Array3D<T>::resize(int _m,int _n,int _p)
{
	if(_m*_n*_p > capacity)
		initialize(_m,_n,_p);
	m=_m;
	n=_n;
	p=_p;
}

template <class T>
void Array3D<T>::resize(int _m,int _n,int _p,const T& initVal)
{
	if(_m*_n*_p > capacity)
		initialize(_m,_n,_p);
	m=_m;
	n=_n;
	p=_p;
	set(initVal);
}

template <class T>
void Array3D<T>::clear()
{
	SafeArrayDelete(items);
	m=n=p=0;
	capacity=0;
}

template <class T>
bool Array3D<T>::find(const T& item,int& i,int& j,int& k) const
{
  for(int s=0;s<m;s++)
    for(int t=0;t<n;t++)
      for(int u=0;u<p;u++)
	if(operator()(s,t,u)==item) {
	  i=s; j=t; k=u;
	  return true;
	}
  return false;
}

template <class T>
void Array3D<T>::set(const T& item)
{
	for(int i=0;i<m*n*p;i++) items[i]=item;
}

template <class T>
void Array3D<T>::set(const Array3D<T>& rhs)
{
	resize(rhs.m,rhs.n,rhs.p);
	for(int i=0;i<m*n*p;i++) items[i]=rhs.items[i];
}

template <class T>
void Array3D<T>::swap(Array3D<T>& b)
{
  int tempm=m,tempn=n,tempp=p;
  int tempcap=capacity;
  T* tempitems=items;
  m=b.m; n=b.n; p=b.p;
  items=b.items;
  capacity=b.capacity;
  b.m=tempm; b.n=tempn; b.p=tempp;
  b.items=tempitems;
  b.capacity=tempcap;
}

template <class T>
Array3D<T>::iterator::iterator(const Array3D<T>* _array)
  :array(_array),range(_array->m,_array->n,_array->p),it(&range)
{}

template <class T>
Array3D<T>::iterator::iterator(const Array3D<T>* _array,int invalid)
  :array(_array),range(_array->m,_array->n,_array->p),it(range.end())
{}

template <class T>
Array3D<T>::iterator::iterator(const Array3D<T>* _array,const Stripe3Indices& _range)
  :array(_array),range(_range),it(&range)
{}

template <class T>
Array3D<T>::iterator::iterator(const Array3D<T>* _array,const Stripe3Indices& _range,int invalid)
  :array(_array),range(_range),it(range.end())
{}

template <class T>
Array3D<T>::iterator::iterator(const iterator& rhs)
  :array(rhs.array),range(rhs.range),it(rhs.it)
{
  it.stripe = &range;   //maintain a valid pointer over the iterator's lifetime
}

template <class T>
const typename Array3D<T>::iterator& Array3D<T>::iterator::operator = (const iterator& rhs)
{
  array = rhs.array;
  range = rhs.range;
  it = rhs.it;
  it.stripe = &range; //maintain a valid pointer over the iterator's lifetime
  return *this;
}


namespace std {

  template <class T>
  void swap(Array3D<T>& a,Array3D<T>&b)
  {
    a.swap(b);
  }

} //namespace std

#endif
