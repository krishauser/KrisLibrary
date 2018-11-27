#ifndef MATH_SELF_TEST_H
#define MATH_SELF_TEST_H

#include <KrisLibrary/Logger.h>
#include "math.h"
#include "vector.h"

namespace Math {

void SelfTest();
void BasicSelfTest();
void VectorSelfTest();
void MatrixSelfTest();
void DifferentiationSelfTest();
void QuadratureSelfTest();
void BlockVectorSelfTest();
void BlockMatrixSelfTest();
void BLASSelfTest();
void LAPACKSelfTest();

class RealFunction;
class VectorFunction;
class ScalarFieldFunction;
class VectorFieldFunction;

///the TestX functions test the methods of the function objects
///and prints errors if the error e=x1-x2 exceeds atol and rtol*max(|x1|,|x2|)
bool TestDeriv(RealFunction* f,Real t,Real h,Real atol,Real rtol);
bool TestDeriv(VectorFunction* f,Real t,Real h,Real atol,Real rtol);
bool TestGradient(ScalarFieldFunction* f,Vector& x,Real h,Real atol,Real rtol);
bool TestJacobian(VectorFieldFunction* f,Vector& x,Real h,Real atol,Real rtol);
bool TestDeriv2(RealFunction* f,Real t,Real h,Real atol,Real rtol);
bool TestDeriv2(VectorFunction* f,Real t,Real h,Real atol,Real rtol);
bool TestHessian(ScalarFieldFunction* f,Vector& x,Real h,Real atol,Real rtol);
///these versions test the methods Gradient_i, Jacobian_i, etc. 
bool TestGradients(ScalarFieldFunction* f,Vector& x,Real h,Real atol,Real rtol);
bool TestJacobians(VectorFieldFunction* f,Vector& x,Real h,Real atol,Real rtol);
bool TestHessians(VectorFieldFunction* f,Vector& x,Real h,Real atol,Real rtol);

} // namespace Math

#endif
