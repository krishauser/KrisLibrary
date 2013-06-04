#ifndef POINTERS_H
#define POINTERS_H

// Overload create(),destroy(),copy() to change behavior
namespace Pointers {

template <class T>
T* create() { return new T(); }
template <class T>
T* copy(const T* p) { return new T(*p); }
template <class T>
void destroy(T* p) { delete p; }

} //namespace Pointers

#endif
