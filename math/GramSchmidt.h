#ifndef MATH_GRAM_SCHMIDT_H
#define MATH_GRAM_SCHMIDT_H

#include "VectorTemplate.h"

namespace Math {

  /** @addtogroup Math */
  /*@{*/

///Performs the Gram-Schmidt process to get an orthonormal basis
///for the span of the input vectors X.
///Returns the number of nonzero vectors in the output basis
template <class T>
int OrthonormalBasis(const VectorTemplate<T>* x, VectorTemplate<T>* basis, int n);

///Same as above, but does not normalize
template <class T>
int OrthogonalBasis(const VectorTemplate<T>* x, VectorTemplate<T>* basis, int n);

///orthogonalizes a vector w.r.t the orthogonal basis of n vectors
template <class T>
void Orthogonalize(VectorTemplate<T>& x,const VectorTemplate<T>* basis, int n);

/*@}*/
} //namespace Math

#endif
