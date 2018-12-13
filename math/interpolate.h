#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"

/** @file math/interpolate.h
 * @ingroup Math
 * @brief Convenience functions for linear interpolation of vectors and
 * matrices
 */

/** @addtogroup Math */
/*@{*/

namespace Math {

template <class type>
void interpolate(const type& a, const type& b, Real u, type& x)
{
  x=(One-u)*a + u*b;
}


template <class T>
void interpolate(const VectorTemplate<T>& a, const VectorTemplate<T>& b, Real u, VectorTemplate<T>& x)
{
  x.resize(a.n);
  x.mul(a,(One-u));
  x.madd(b,u);
}

template <class T>
void interpolate(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, Real u, MatrixTemplate<T>& x)
{
  x.resize(a.m,a.n);
  x.mul(a,(One-u));
  x.madd(b,u);
}

} //namespace Math

/*@}*/

#endif
