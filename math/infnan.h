#ifndef MATH_INFNAN_H
#define MATH_INFNAN_H

#include <math.h>

#ifndef INFINITY
  #include <limits>
#endif //INFINITY

/** @file math/infnan.h
 * @ingroup Math
 * @brief Cross-platform infinity and not-a-number routines.
 *
 * Warning: if INFINITY is not defined as a constant, this uses
 * the STL numeric_limits<T>::infinity() call.  Global static members
 * set to Inf will NOT necessarily be initialized properly on certain
 * platforms, e.g., Windows.  The solution is to avoid declaring static
 * variables equal to infinity.
 */

namespace Math { 

/** @addtogroup Math */
/*@{*/


#ifdef INFINITY
  const static double dInf = INFINITY;
  const static float fInf = INFINITY;
#else
  const static double dInf = std::numeric_limits<double>::infinity();
  const static float fInf = std::numeric_limits<float>::infinity();
#endif // INFINITY


///Returns nonzero if x is not-a-number (NaN)
int IsNaN(double x);
int IsNaN(float x);
///Returns nonzero unless x is infinite or a NaN
int IsFinite(double x);
int IsFinite(float x);
///Returns +1 if x is +inf, -1 if x is -inf, and 0 otherwise
int IsInf(double x);
int IsInf(float x);

/*@}*/
} //namespace Math

#endif
