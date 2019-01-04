#ifndef UTILS_SMART_POINTER_H
#define UTILS_SMART_POINTER_H

#include <stdlib.h>
#include <KrisLibrary/errors.h>

/** @ingroup Utils
 * @brief A smart pointer class.  Performs automatic reference
 * counting.
 */
template <class T>
class SmartPointer 
{
public:
  inline SmartPointer(T* ptr = 0);
  inline SmartPointer(const SmartPointer<T>& other);
  inline ~SmartPointer();
  const SmartPointer<T>& operator =(const SmartPointer<T>& rhs);
  inline bool isNull() const { return (ptr == NULL); }
  inline operator const T* () const { return ptr; }
  inline operator T* () { return ptr; }
  inline T* operator ->() { return ptr; }
  inline const T* operator ->() const { return ptr; }
  inline T& operator *() { return *ptr; }
  inline const T& operator *() const { return *ptr; }
  inline bool isNonUnique() const;  
  inline int getRefCount() const;
  
protected:
  T* ptr;
  int* refCount;
};

template <class T> 
inline SmartPointer<T>::SmartPointer(T* _ptr)
  : ptr(_ptr), refCount(NULL)
{
  if (ptr) {
    refCount = new int;
    Assert(refCount != NULL);
    *refCount = 1;
  }
}

template <class T> 
inline SmartPointer<T>::SmartPointer(const SmartPointer<T>& other)
  : ptr(other.ptr), refCount(other.refCount)
{
  if (refCount != NULL)
    ++(*refCount);
}

template <class T> 
inline SmartPointer<T>::~SmartPointer() 
{
  if (refCount != NULL && --(*refCount) == 0) {
    delete ptr;
    ptr = NULL;
    delete refCount;
    refCount = NULL;
  }
}

template <class T>
inline bool SmartPointer<T>::isNonUnique() const {
  return refCount == NULL ? false : *refCount != 1;
}

template <class T> 
inline int SmartPointer<T>::getRefCount() const {
  return refCount == NULL ? 0 : *refCount;
}

template <class T> 
inline const SmartPointer<T>& SmartPointer<T>::operator =(const SmartPointer<T>& rhs) {
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

