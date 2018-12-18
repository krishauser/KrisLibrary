#ifndef UTILS_SHIFT_H
#define UTILS_SHIFT_H

#include <vector>

/** @file utils/shift.h
 * @ingroup Utils
 * @brief Cyclical shifts of array elements
 */

/** @addtogroup Utils */
/*@{*/

///Shift forward: set a'=c,b'=a,c'=b
template <class T>
void ShiftForward(T& a, T& b, T& c)
{
  T temp=c;
  c=b;
  b=a;
  a=temp;
}

///Shift backward: set a'=b,b'=c,c'=a
template <class T>
void ShiftBackward(T& a, T& b, T& c)
{
  T temp=a;
  a=b;
  b=c;
  c=temp;
}

///For all i, v'[i] = v[i-1], v'[0] = v[n-1]
template <class T>
void ShiftForward(std::vector<T>& v)
{
  if(v.empty()) return;
  T temp=v.back();
  std::vector<int>::iterator i,prev;
  for(i=--v.end();i!=v.begin();i--) {
    prev=i; prev--;
    *i = *prev;
  }
  v.front()=temp;
}

///For all i, v'[i] = v[i+1], v'[n-1] = v[0]
template <class T>
void ShiftBackward(std::vector<T>& v)
{
  if(v.empty()) return;
  T temp=v.front();
  std::vector<int>::iterator i,next;
  for(i=v.begin();next!=--v.end();i++) {
    next=i; next++;
    *i = *next;
  }
  v.back()=temp;
}

/*@}*/

#endif
