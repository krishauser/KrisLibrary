#ifndef MATH_LINEARLY_DEPENDENT_H
#define MATH_LINEARLY_DEPENDENT_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"

namespace Math {

/** @ingroup Math
 * @brief Robust determination of linear dependence.
 *
 * Calculates a constant c s.t. they're linearly dependent within rel error 
 * eps and |c| <= 1.
 *
 *  2 cases, (1) a*c = b with maxabs(a*c-b)/|a| <= eps.
 *  (2) a = c*b with maxabs(a-c*b)/|b| <= eps.
 *  in case 1, cb = false, in case 2, cb=true.
 *  c is chosen by pseudoinverse
 * @return True if the vectors are dependent.
 */
template <class T>
bool LinearlyDependent_Robust(const VectorTemplate<T>& a, const VectorTemplate<T>& b, T& c, bool& cb, T eps = Epsilon);

/** @ingroup Math
 * @brief A matrix version of the function above.
 *
 * @return A vector c s.t. A*c = 0.  All coefficients |ci| <= 1
 *  At least one ci = 1.
 */
template <class T>
bool LinearlyDependent_Robust(const MatrixTemplate<T>& A, VectorTemplate<T>& c, T eps = Epsilon);


} //namespace Math

#endif 
