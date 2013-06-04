#ifndef MATH3D_LINEARLY_DEPENDENT_H
#define MATH3D_LINEARLY_DEPENDENT_H

#include "primitives.h"

namespace Math3D {

//Robust determination of linear dependence.
//Return true if the vectors are dependent.
//Single-vector versions:
//  Returns a constant c s.t. they're linearly dependent within rel error eps
//    and |c| <= 1.
//  2 cases, (1) a*c = b with maxabs(a*c-b)/|a| <= eps.
//  (2) a = c*b with maxabs(a-c*b)/|b| <= eps.
//  in case 1, cb = false, in case 2, cb=true.
//  c is chosen by pseudoinverse
//Matrix versions:
//  Returns a vector c s.t. A*c = 0.  All coefficients |ci| <= 1
//  At least one ci = 1.

bool LinearlyDependent_Robust(const Vector2& a, const Vector2& b, Real& c, bool& cb, Real eps = Epsilon); 
bool LinearlyDependent_Robust(const Vector3& a, const Vector3& b, Real& c, bool& cb, Real eps = Epsilon); 
bool LinearlyDependent_Robust(const Vector4& a, const Vector4& b, Real& c, bool& cb, Real eps = Epsilon); 

} 

#endif 
