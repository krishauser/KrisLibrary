#ifndef ITERATOR_UTILS_H
#define ITERATOR_UTILS_H

/** @file utils/iteratorutils.h
 * @ingroup Utils
 * @brief Provides basic utilities to increment/decrement an 
 * iterator by a certain amount, 2D range iterators, etc.
 */

/** @addtogroup Utils */
/*@{*/

template <class T>
inline T increment(const T& a,int n=1)
{
  T x=a;
  for(int i=0;i<n;i++) ++x;
  return x;
}

template <class T>
inline T decrement(const T& a,int n=1)
{
  T x=a;
  for(int i=0;i<n;i++) --x;
  return x;
}

//returns a-b, the # of times you'd need to increment b to get to a
template <class T>
inline int iterator_diff(const T& a,const T& b)
{
  T x=b;
  int n=0;
  while(x!=a) { ++x; ++n; }
  return n;
}

/*@}*/

#endif
