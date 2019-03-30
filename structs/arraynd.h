#ifndef ARRAY_ND
#define ARRAY_ND

#include <KrisLibrary/Logger.h>
#include "array2d.h"
#include "array3d.h"
#include <KrisLibrary/utils/IntTuple.h>
#include <vector>
#include <assert.h>

template <class T> class ArrayND;
template <class T> class ArrayNDRef;

/** @brief An N-D array class.
 */
template <class T>
class ArrayND
{
 public:
  ArrayND();
  ArrayND(const ArrayND<T>& rhs);
  ArrayND(const Array2D<T>& rhs);
  ArrayND(const Array3D<T>& rhs);
  ArrayND(int dim1);
  ArrayND(int dim1,int dim2);
  ArrayND(int dim1,int dim2,int dim3);
  ArrayND(const std::vector<int>& dims);
  ArrayND(const IntTuple& dims);

  void clear();
  inline size_t numDims() const { return dims.size(); }
  inline size_t numValues() const { return values.size(); }
  inline const std::vector<int>& size() const { return dims; }

  ///note: resize is destructive 
  void resize(const std::vector<int>& newdims);
  inline void resize(const IntTuple& dims) { resize(dims.elements); }
  inline void resize(int dim1) { resize(IntTuple(dim1)); }
  inline void resize(int dim1,int dim2) { resize(IntTuple(dim1,dim2)); }
  inline void resize(int dim1,int dim2,int dim3) { resize(IntTuple(dim1,dim2,dim3)); }

  inline void set(T val) { std::fill(values.begin(),values.end(),val); }

  const ArrayND<T>& operator = (const ArrayND<T>& rhs);
  const ArrayND<T>& operator = (const std::vector<T>& rhs);
  const ArrayND<T>& operator = (const Array2D<T>& rhs);
  const ArrayND<T>& operator = (const Array3D<T>& rhs);

  bool operator == (const ArrayND<T>& rhs) const;
  ArrayNDRef<T> operator [] (int i);
  T& operator [] (const std::vector<int>& index);
  const T& operator [] (const std::vector<int>& index) const;
  inline T& operator [] (const IntTuple& index) { return operator [](index.elements); }
  inline const T& operator [] (const IntTuple& index) const { return operator [](index.elements); }

  void getSubArray(const std::vector<int>& imin,const std::vector<int>& imax,ArrayND<T>& subArray) const;
  void setSubArray(const std::vector<int>& imin,const std::vector<int>& imax,const ArrayND<T>& subArray);
  void getSlice(const std::vector<int>& dimIndices,ArrayND<T>& slice) const;
  void setSlice(const std::vector<int>& dimIndices,const ArrayND<T>& slice);

  int indexToOffset(const std::vector<int>& index) const;
  inline void indexToOffset(const IntTuple& index) const { return indexToOffset(index.elements); }
  inline int incOffset(int offset,int dim) const { return offset+strides[dim]; }
  inline int decOffset(int offset,int dim) const { return offset-strides[dim]; }
  std::vector<int> offsetToIndex(int offset) const;

  struct iterator
  {
    iterator& operator ++();
    iterator& operator --();
    iterator& operator +=(int skip);
    iterator& operator -=(int skip);
    void inc(int dim);
    void inc(int dim,int count);
    void dec(int dim);
    void dec(int dim,int count);
    inline bool operator == (const iterator& it) const  { return it.obj == obj && it.offset == offset; }
    inline bool operator != (const iterator& it) const  { return !operator == (it); }
    inline bool operator < (const iterator& it) const  { return it.obj == obj && it.offset > offset; }
    inline bool operator > (const iterator& it) const  { return it.obj == obj && it.offset < offset; }
    inline int operator - (const iterator& it) const  { assert(it.obj == obj); return offset-it.offset; }
    inline T& operator * () {
      assert(offset >= 0 && offset<obj->values.size());
      return obj->values[offset];
    }
    inline T* operator -> () { 
      assert(offset >= 0 && offset<obj->values.size());
      return &obj->values[offset];
    }

    ArrayND<T>* obj;
    std::vector<int> index;
    int offset;
  };

  iterator begin();
  iterator begin(const std::vector<int>& index);
  iterator end();

  std::vector<int> dims;
  std::vector<int> strides;
  std::vector<T> values;
};

template <class T>
class ArrayNDRef
{
 public:
  ArrayNDRef<T>();
  ArrayNDRef<T>(const ArrayNDRef<T>& rhs);
  ArrayNDRef<T>(ArrayND<T>* obj,int offset,int curDim);
  const ArrayNDRef<T>& operator = (T val);
  const ArrayNDRef<T>& operator = (const ArrayND<T>& val);
  operator T ();
  ArrayNDRef<T> operator [] (int i);

  ArrayND<T>* obj;
  int offset;
  int curDim;
};

template <class T>
std::ostream& operator <<(std::ostream& out,const ArrayND<T>& array)
{
  out<<array.dims.size()<<'\t';
  for(size_t i=0;i<array.dims.size();i++)
    out<<array.dims[i]<<" ";
  out<<std::endl;
  for(size_t i=0;i<array.values.size();i++)
    out<<array.values[i]<<std::endl;
  return out;
}

template <class T>
std::istream& operator >>(std::istream& in,ArrayND<T>& array)
{
  std::vector<int> dims;
  int n;
  in >> n;
  if(!in || n < 0) { in.setstate(std::ios::badbit); return in; }
  dims.resize(n);
  for(size_t i=0;i<dims.size();i++) {
    in >> dims[i];
    if(dims[i] < 0) { in.setstate(std::ios::badbit); return in; }
  }
  array.resize(dims);
  for(size_t i=0;i<array.values.size();i++)
    in>>array.values[i];
  return in;
}



template <class T>
ArrayND<T>::ArrayND()
{}

template <class T>
ArrayND<T>::ArrayND(const ArrayND<T>& rhs)
  :dims(rhs.dims),strides(rhs.strides),values(rhs.values)
{}


template <class T>
ArrayND<T>::ArrayND(const Array2D<T>& rhs)
  :dims(2),strides(2),values(rhs.getData(),rhs.getData()+rhs.m*rhs.n)
{
  dims[0] = rhs.m;
  dims[1] = rhs.n;
  strides[0] = rhs.n;
  strides[1] = 1;
}

template <class T>
ArrayND<T>::ArrayND(const Array3D<T>& rhs)
  :dims(3),strides(3),values(rhs.getData(),rhs.getData()+rhs.m*rhs.n*rhs.p)
{
  dims[0] = rhs.m;
  dims[1] = rhs.n;
  dims[2] = rhs.p;
  strides[0] = rhs.n*rhs.p;
  strides[1] = rhs.p;
  strides[2] = 1;
}

template <class T>
ArrayND<T>::ArrayND(int dim1)
{
  resize(dim1);
}

template <class T>
ArrayND<T>::ArrayND(int dim1,int dim2)
{
  resize(dim1,dim2);
}

template <class T>
ArrayND<T>::ArrayND(int dim1,int dim2,int dim3)
{
  resize(dim1,dim2,dim3);
}

template <class T>
ArrayND<T>::ArrayND(const std::vector<int>& dims)
{
  resize(dims);
}

template <class T>
ArrayND<T>::ArrayND(const IntTuple& dims)
{
  resize(dims);
}

template <class T>
void ArrayND<T>::clear()
{
  dims.clear();
  strides.clear();
  values.clear();
}

template <class T>
void ArrayND<T>::resize(const std::vector<int>& newdims)
{
  if(newdims.empty()) {
    clear();
    return;
  }
  dims = newdims;
  int nv = 1;
  for(size_t i=0;i<newdims.size();i++)
    nv *= newdims[i];
  values.resize(nv);
  strides.resize(dims.size());
  strides[dims.size()-1] = 1;
  for(size_t i=dims.size()-1;i>0;i--)
    strides[i-1] = dims[i]*strides[i];
}
 
template <class T>
const ArrayND<T>& ArrayND<T>::operator = (const ArrayND<T>& rhs)
{
  dims = rhs.dims;
  strides = rhs.strides;
  values = rhs.value;
  return *this;
}

template <class T>
const ArrayND<T>& ArrayND<T>::operator = (const std::vector<T>& rhs)
{
  resize(rhs.size());
  std::copy(rhs.begin(),rhs.end(),values.begin());
  return *this;
}

template <class T>
const ArrayND<T>& ArrayND<T>::operator = (const Array2D<T>& rhs)
{
  resize(rhs.m,rhs.n);
  std::copy(rhs.getData(),rhs.getData()+values.size(),values.begin());
  return *this;
}

template <class T>
const ArrayND<T>& ArrayND<T>::operator = (const Array3D<T>& rhs)
{
  resize(rhs.m,rhs.n,rhs.p);
  std::copy(rhs.getData(),rhs.getData()+values.size(),values.begin());
  return *this;
}

template <class T>
bool ArrayND<T>::operator == (const ArrayND<T>& rhs) const
{
  if(dims != rhs.dims) return false;
  if(values != rhs.values) return false;
  return true;
}

template <class T>
ArrayNDRef<T> ArrayND<T>::operator [] (int i)
{
  assert(!dims.empty());
  int offset = i*strides[0];
  return ArrayNDRef<T>(this,offset,0);
}


template <class T>
T& ArrayND<T>::operator [] (const std::vector<int>& index)
{
  int offset=indexToOffset(index);
  for(size_t i=0;i<index.size();i++)
    assert(index[i] >= 0 && index[i] < dims[i]);
  return values[offset];
}

template <class T>
const T& ArrayND<T>::operator [] (const std::vector<int>& index) const
{
  int offset=indexToOffset(index);
  for(size_t i=0;i<index.size();i++)
    assert(index[i] >= 0 && index[i] < dims[i]);
  return values[offset];
}

/*
template <class T>
void ArrayND<T>::getSubArray(const std::vector<int>& imin,const std::vector<int>& imax,ArrayND<T>& subArray);
template <class T>
void ArrayND<T>::setSubArray(const std::vector<int>& imin,const std::vector<int>& imax,const ArrayND<T>& subArray);
template <class T>
void ArrayND<T>::getSlice(const std::vector<int>& dimIndices,ArrayND<T>& slice);
template <class T>
void ArrayND<T>::setSlice(const std::vector<int>& dimIndices,ArrayND<T>& slice);
*/

template <class T>
int ArrayND<T>::indexToOffset(const std::vector<int>& index) const
{
  assert(index.size()==strides.size());
  int offset = 0;
  for(size_t i=0;i<index.size();i++)
    offset += index[i]*strides[i];
  return offset;
}

template <class T>
std::vector<int> ArrayND<T>::offsetToIndex(int offset) const
{
  std::vector<int> index(strides.size());
  for(size_t i=0;i<index.size();i++) {
    index[i] = offset/strides[i];
    offset = offset%strides[i];
  }
  return index;
}

template <class T>
typename ArrayND<T>::iterator ArrayND<T>::begin()
{
  iterator res;
  res.obj = this;
  res.offset = 0;
  res.index.resize(dims.size(),0);
  return res;
}

template <class T>
typename ArrayND<T>::iterator ArrayND<T>::begin(const std::vector<int>& index)
{
  iterator res;
  res.obj = this;
  res.offset = indexToOffset(index);
  res.index = index;
  return res;
}

template <class T>
typename ArrayND<T>::iterator ArrayND<T>::end()
{
  iterator res;
  res.obj = this;
  res.offset = int(values.size());
  res.index.resize(dims.size(),0);
  res.index[0] = dims[0];
  return res;
}




template <class T>
typename ArrayND<T>::iterator& ArrayND<T>::iterator::operator ++()
{
  offset++;
  for(int i=(int)index.size()-1;i>=0;i--) {
    index[i]++;
    if(index[i]==obj->dims[i])
      index[i] = 0;
    else {
      assert(index[i] < obj->dims[i]);
      break;
    }
  }
  return *this;
}

template <class T>
typename ArrayND<T>::iterator& ArrayND<T>::iterator::operator --()
{
  offset--;
  for(int i=(int)index.size()-1;i>=0;i--) {
    index[i]--;
    if(index[i] < 0)
      index[i] = obj->dims[i]-1;
    else {
      assert(index[i] >= 0);
      break;
    }
  }
  return *this;
}

template <class T>
typename ArrayND<T>::iterator& ArrayND<T>::iterator::operator +=(int skip)
{
  offset+=skip;
  for(int i=(int)index.size()-1;i>=0;i--) {
    index[i]+=skip;
    if(index[i] < obj->dims[i]) break;
    assert(obj->dims[i] > 0);
    skip = index[i]/obj->dims[i];
    index[i]=index[i]%obj->dims[i];
  }
  return *this;
}

template <class T>
typename ArrayND<T>::iterator& ArrayND<T>::iterator::operator -=(int skip)
{
  offset-=skip;
  for(int i=(int)index.size()-1;i>=0;i--) {
    index[i]-=skip;
    if(index[i] >=0) break;
    //TODO: faster version of this
    assert(obj->dims[i] > 0);
    skip=0;
    while(index[i] < 0) {
      skip++;
      index[i]+=obj->dims[i];
    }
  }
  return *this;
}

template <class T>
void ArrayND<T>::iterator::inc (int dim)
{
  assert(dim >= 0 && dim < (int)index.size());
  index[dim]++;
  offset += obj->strides[dim];
}

template <class T>
void ArrayND<T>::iterator::inc (int dim,int count)
{
  assert(dim >= 0 && dim < (int)index.size());
  index[dim]+=count;
  offset += count*obj->strides[dim];
}

template <class T>
void ArrayND<T>::iterator::dec (int dim)
{
  assert(dim >= 0 && dim < (int)index.size());
  index[dim]--;
  offset -= obj->strides[dim];
}

template <class T>
void ArrayND<T>::iterator::dec (int dim,int count)
{
  assert(dim >= 0 && dim < (int)index.size());
  index[dim]-=count;
  offset -= count*obj->strides[dim];
}


template <class T>
ArrayNDRef<T>::ArrayNDRef()
  :obj(NULL),offset(0),curDim(0)
{}

template <class T>
ArrayNDRef<T>::ArrayNDRef(const ArrayNDRef<T>& rhs)
  :obj(rhs.obj),offset(rhs.offset),curDim(rhs.curDim)
{}

template <class T>
ArrayNDRef<T>::ArrayNDRef(ArrayND<T>* _obj,int _offset,int _curDim)
  :obj(_obj),offset(_offset),curDim(_curDim)
{}

template <class T>
const ArrayNDRef<T>& ArrayNDRef<T>::operator = (T val)
{
  assert(curDim+1 == (int)obj->dims.size());
  obj->values[offset] = val;
  return *this;
}

template <class T>
const ArrayNDRef<T>& ArrayNDRef<T>::operator = (const ArrayND<T>& val)
{
  assert(curDim+val.dims.size() == (int)obj->dims.size());
    LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: copy slices\n");
  abort();
  return *this;
}

template <class T>
ArrayNDRef<T>::operator T ()
{
  assert(curDim+1 == (int)obj->dims.size());
  return obj->values[offset];
}

template <class T>
ArrayNDRef<T> ArrayNDRef<T>::operator [] (int i)
{
  assert(curDim+1 < (int)obj->dims.size());
  return ArrayNDRef<T>(obj,offset+obj->strides[curDim+1]*i,curDim+1);
}





#endif // ARRAY_ND
