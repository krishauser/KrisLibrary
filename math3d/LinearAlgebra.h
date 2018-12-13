#ifndef MATH3D_LINEAR_ALGEBRA_H
#define MATH3D_LINEAR_ALGEBRA_H

#include "primitives.h"
#include <math/complex.h>
#include <math/vector.h>
#include <math/matrix.h>

namespace Math3D
{

//copy nD primitives to a general vector/matrix
void Copy(Real v,Vector& vec);
void Copy(const Vector2& v,Vector& vec);
void Copy(const Vector3& v,Vector& vec);
void Copy(const Vector4& v,Vector& vec);
void Copy(const Vector& vec,Real& v);
void Copy(const Vector& vec,Vector2& v);
void Copy(const Vector& vec,Vector3& v);
void Copy(const Vector& vec,Vector4& v);
void Copy(Real m,Matrix& mat);
void Copy(const Matrix2& m,Matrix& mat);
void Copy(const Matrix3& m,Matrix& mat);
void Copy(const Matrix4& m,Matrix& mat);
void Copy(const Matrix& mat,Real& m);
void Copy(const Matrix& mat,Matrix2& m);
void Copy(const Matrix& mat,Matrix3& m);
void Copy(const Matrix& mat,Matrix4& m);

//A = U diag(W) V^T  (SVs are sorted from highest to lowest)
bool SVD(const Matrix2& A,Matrix2& U,Vector2& W,Matrix2& V);
bool SVD(const Matrix3& A,Matrix3& U,Vector3& W,Matrix3& V);
bool SVD(const Matrix4& A,Matrix4& U,Vector4& W,Matrix4& V);

//returns eigenvalues of matrices
void Eigenvalues(const Matrix2& A,Complex& lambda1,Complex& lambda2);
bool Eigenvalues(const Matrix2& A,Real& lambda1,Real& lambda2);
void Eigenvalues(const Matrix3& A,Complex& lambda1,Complex& lambda2,Complex& lambda3);
bool Eigenvalues(const Matrix3& A,Real& lambda1,Real& lambda2,Real& lambda3);

//A = Q diag(lambda) Q^T for symmetric matrices A
bool Eigendecomposition(const Matrix2& A,Vector2& lambda,Matrix2& Q);
bool Eigendecomposition(const Matrix3& A,Vector3& lambda,Matrix3& Q);
bool Eigendecomposition(const Matrix4& A,Vector4& lambda,Matrix4& Q);

} //namespace Math3D


#endif
