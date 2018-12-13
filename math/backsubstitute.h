#ifndef MATH_BACK_SUBSTITUTE_H
#define MATH_BACK_SUBSTITUTE_H

#include "MatrixTemplate.h"

namespace Math {

// If A is upper triangular nxn, solves Ax=b
template <class T>
bool UBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);

// If A is lower triangular nxn, solves Ax=b
template <class T>
bool LBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);

// If A is lower triangular nxn, solves A^t*x=b
template <class T>
bool LtBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);

// The following are identical to the above, but assume the diagonal of a is all ones
template <class T>
void U1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);
template <class T>
void L1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);
template <class T>
void Lt1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);


// The following perform the same operations as above but with x and b matrices
template <class T>
bool UBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
bool LBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
bool LtBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
void U1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
void L1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
void Lt1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);


} // namespace Math

#endif
