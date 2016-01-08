#ifndef UTILS_UNIQUE_POINTER_H
#define UTILS_UNIQUE_POINTER_H

#include <iostream>
#include <stdlib.h>
#include "KrisLibrary/errors.h"

/** @ingroup Utils
 * @brief A unique pointer class that duplicates on non-const access.
 * It's used to implement non-conflicting writes to data stored in
 * multiple locations, for example in sparse state representations.
 */
template <class T>
class UniquePointer 
{
public:
  inline UniquePointer(T* ptr = 0);
  inline UniquePointer(const UniquePointer<T>& other);
  inline ~UniquePointer();
  const UniquePointer<T>& operator =(const UniquePointer<T>& rhs);
  inline bool isNull() const { return (ptr == NULL); }
  inline operator const T* () const { return ptr; }
  inline operator T* () { if(*refCount!=1) ptr=new T(*ptr); return ptr; }
  inline T* operator ->() { if(*refCount!=1) ptr=new T(*ptr); return ptr;  }
  inline const T* operator ->() const { return ptr; }
  inline T& operator *() { if(*refCount!=1) ptr=new T(*ptr); return ptr;  }
  inline const T& operator *() const { return *ptr; }
  inline bool isNonUnique() const;  
  inline int getRefCount() const;
  
protected:
  T* ptr;
  int* refCount;
};

template <class T> 
inline UniquePointer<T>::UniquePointer(T* _ptr)
  : ptr(_ptr), refCount(NULL)
{
  if (ptr) {
    refCount = new int;
    Assert(refCount != NULL);
    *refCount = 1;
  }
}

template <class T> 
inline UniquePointer<T>::UniquePointer(const UniquePointer<T>& other)
  : ptr(other.ptr), refCount(other.refCount)
{
  if (refCount != NULL)
    ++(*refCount);
}

template <class T> 
inline UniquePointer<T>::~UniquePointer() 
{
  if (refCount != NULL && --(*refCount) == 0) {
    delete ptr;
    ptr = NULL;
    delete refCount;
    refCount = NULL;
  }
}

template <class T>
inline bool UniquePointer<T>::isNonUnique() const {
  return refCount == NULL ? false : *refCount != 1;
}

template <class T> 
inline int UniquePointer<T>::getRefCount() const {
  return refCount == NULL ? 0 : *refCount;
}

template <class T> 
inline const UniquePointer<T>& UniquePointer<T>::operator =(const UniquePointer<T>& rhs) {
  if (ptr != rhs.ptr) {
    if (refCount != NULL && --(*refCount) == 0) {
      delete ptr;
      delete refCount;
    }
    ptr = rhs.ptr;
    refCount = rhs.refCount;
    if (refCount != NULL)
      ++(*refCount);
  }
  return *this;
}

#endif
