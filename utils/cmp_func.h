#ifndef UTILS_CMP_FUNC_H
#define UTILS_CMP_FUNC_H

#include <functional>

namespace std {

/** @ingroup Utils
 * @brief A templated function class like strcmp: 
 * returns -1 if a<b, 0 if a==b, 1 if a>b.
 */
template <class T,class Less=less<T> >
struct cmp_func
{
  int operator()(const T& a,const T& b) {
    if(lessFunc(a,b)) return -1;
    else if(lessFunc(b,a)) return 1;
    return 0;
  }
  Less lessFunc;
};

} //namespace std

#endif
