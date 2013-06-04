#ifndef MATH_QUADRATURE_H
#define MATH_QUADRATURE_H

#include "function.h"

/** @file quadrature.h
 * @ingroup Math
 * @brief Several functions for performing quadrature (function integration)
 */

namespace Math {
  /** @addtogroup Math */
  /*@{*/

typedef Real (*QuadratureFunction) (RealFunction&, Real, Real);

///Namespace for quadrature functions.
namespace Quadrature {

///General purpose summation routine, returns sum[i=0..k-1]ci*f(xi)
Real Quadrature(RealFunction& f,const Real* c,const Real* x,int k);
void Quadrature(VectorFunction& f,const Real* c,const Real* x,int k,Vector& res);


///trapezoidal rule
///integral[a,b] f dx ~= 1/2h(f(a) + f(b)) + O(h^3)
Real trapezoidal(RealFunction& f, Real a, Real b);
Real trapezoidal(Real f0, Real f1);

///simpsons rule
///integral[a,b] f dx ~= 1/3h(f(a) + 4f(a+h) + f(b)) + O(h^5)
Real simpsons(RealFunction& f, Real a, Real b);
Real simpsons(Real f0, Real f1, Real f2);

///simpsons 3/8 rule 
///integral[a,b] f dx ~= 3/8h(f(a) + 3f(a+h) + 3f(a+2h) + f(b)) + O(h^5)
Real simpsons_3_8(RealFunction& f, Real a, Real b);
Real simpsons_3_8(Real f0, Real f1, Real f2, Real f3);

///Newton-Cotes order 4
///integral[a,b] f dx ~= 2/45h(7f(a) + 32f(a+h) + 12f(a+2h) + 32f(a+3h) + 7f(b)) + O(h^7)
Real NC4(RealFunction& f, Real a, Real b);
Real NC4(Real f0, Real f1, Real f2, Real f3, Real f4);

///Gaussian quadrature:
///The coefficient versions Gaussiank give coefficients for the
///k-point gaussian quadrature rule in the arrays c and x
///The resulting rule is int[-1,1]f(x) = sum ci*f(xi)
///note: only the positive half is provided
extern const Real cGaussian2[1],xGaussian2[1];
extern const Real cGaussian3[2],xGaussian3[2];
extern const Real cGaussian4[2],xGaussian4[2];
extern const Real cGaussian5[3],xGaussian5[3];
extern const Real cGaussian6[3],xGaussian6[3];
extern const Real cGaussian7[4],xGaussian7[4];
extern const Real cGaussian8[4],xGaussian8[4];
extern const Real cGaussian10[5],xGaussian10[5];

///Warped gaussian quadrature:
///Same as above, but integral of a warped function
///f*x on [0,1]
extern const Real cGaussian2_x_unit[2],xGaussian2_x_unit[2];
extern const Real cGaussian3_x_unit[3],xGaussian3_x_unit[3];
///f*x^2 on [0,1]
extern const Real cGaussian2_x2_unit[2],xGaussian2_x2_unit[2];
extern const Real cGaussian3_x2_unit[3],xGaussian3_x2_unit[3];
///f*x(x-1) on [0,1]
extern const Real cGaussian2_x1x_unit[2],xGaussian2_x1x_unit[2];
extern const Real cGaussian3_x1x_unit[3],xGaussian3_x1x_unit[3];

///Gaussian quadrature rule with k points on [-1,1], [a,b] respectively
Real Gaussian(RealFunction& f,int k);
Real Gaussian(RealFunction& f,Real a,Real b,int k);

///composite trapezoidal rule on n segments
///integral[a,b] f dx ~= 1/2h(f(a) + 2*sum[i=1:n-1]f(a+ih) + f(b)) + (b-a)O(h^2)
Real compositeTrapezoidal(RealFunction& f, Real a, Real b, int n);

///composite simpsons rule on n segments
///(n assumed to be even)
///integral[a,b] f dx ~= 1/3h(f(a) + 4*sum[i=1:n-1, odd]f(a+ih) + 2*sum[i=1:n-1, even]f(a+ih) + f(b)) + (b-a)O(h^4)
Real compositeSimpsons(RealFunction& f, Real a, Real b, int n);

///composite integration for an arbitrary quadrature function
Real composite(QuadratureFunction q, RealFunction& f, Real a, Real b, int n);

///integral[a,b]x[c,d] f dA = 1/4(b-a)(c-d) (f(a,c) + f(b,c) + f(a,d) + f(b,d)) + O(h^2)
Real trapezoidal2D(RealFunction2& f, Real a, Real b, Real c, Real d);

///integral[a,b]x[c,d] f dA = 1/9(b-a)(c-d) (f(a,c) + f(a,d) + f(b,c) + f(b,d)
///	+ 4[f(a+h,c)+f(a+h,d)+f(a,c+h)+f(b,c+h)] + 16*f(a+h,c+h) ) + O(h^4)
Real simpsons2D(RealFunction2& f, Real a, Real b, Real c, Real d);

///integral[a,b]x[c(x),d(x)] f dA = 1/9 (f(a,c(a)) + f(a,d(a)) + f(b,c(b)) + f(b,d(b))
///	+ 4[f(a+h,c(a+h))+f(a+h,d(a+h))+f(a,c(a)+h(a))+f(b,c(b)+h(b))] + 16*f(a+h,c(a+h)+h(a+h)) ) + O(h^4)
Real simpsons2D(RealFunction2& f, Real a, Real b, RealFunction& c, RealFunction& d);

} //namespace Quadrature
  /*@}*/
} //namespace Math

#endif
