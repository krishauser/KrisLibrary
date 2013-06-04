#ifndef MATH_HOUSEHOLDER_H
#define MATH_HOUSEHOLDER_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"

/** @file Householder.h
 * @ingroup Math
 * @brief Functions for householder transformations.
 */

namespace Math {

  /** @addtogroup Math */
  /*@{*/

/** Replace v[0:n-1] with a householder vector (v[0:n-1]) and
   coefficient tau that annihilate v[1:n-1].  Tau is returned */
template <class T>
T HouseholderTransform(VectorTemplate<T>& v);

/** Applies a householder transformation v,tau to matrix A */
template <class T>
void HouseholderPreMultiply(T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A);

/** Applies a householder transformation v,tau to matrix m from the
   right hand side in order to zero out rows */
template <class T>
void HouseholderPostMultiply(T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A);

/** Applies a householder transformation tau,v to vector w */
template <class T>
void HouseholderApply(T tau, const VectorTemplate<T>& v, VectorTemplate<T>& w);

/** Applies a householder transformation v,tau to a matrix being
   built up from the identity matrix, using the first column of A as
   a householder vector */
template <class T>
void HouseholderHM1(T tau, MatrixTemplate<T>& A);

  /*@}*/
} //namespace Math

#endif
