#ifndef MATH_LINEAR_ALGEBRA_H
#define MATH_LINEAR_ALGEBRA_H

#include "matrix.h"

namespace Math {

struct IterativeMethod
{
  enum Type { Jacobi, GaussSeidel, SOR };
  IterativeMethod(const Matrix& A,const Vector& b,Real omega=One);

  void InitialRandom(Vector& x0,Real xmin=-One,Real xmax=One) const;
  void InitialOnes(Vector& x0) const;
  bool Solve(Type type,Vector& x0,int& maxIters,Real& tol) const;

  bool IsValid_Jacobi() const;  //must be diagonally dominant
  bool IsValid_GaussSeidel() const;  //must be diagonally dominant or pos. def
  bool IsValid_SOR() const;  //must be diagonally dominant or pos. def

  void Iterate_Jacobi(Vector& x) const;
  void Iterate_GaussSeidel(Vector& x) const;
  void Iterate_SOR(Vector& x) const;

  const Matrix& A;
  const Vector& b;
  Real omega; //for SOR
};

//Ax=b
struct MatrixEquation
{
  MatrixEquation(const Matrix& A, const Vector& b);
  
  // Exact solution of x (returns false if A is not invertible)
  bool Solve(Vector& x) const;
  bool Solve_Cholesky(Vector& x) const;
  bool Solve_LU(Vector& x) const;
  bool Solve_SVD(Vector& x) const;
  bool Solve_Jacobi(Vector& x,int maxIters=100,Real tol=Epsilon) const;
  bool Solve_GaussSeidel(Vector& x,int maxIters=100,Real tol=Epsilon) const;
  bool Solve_SOR(Vector& x,Real omega=1.25,int maxIters=100,Real tol=Epsilon) const;
  
  // Least-squares solution of x (returns false if method fails)
  bool LeastSquares(Vector& x) const;
  bool LeastSquares_Cholesky(Vector& x) const;
  bool LeastSquares_QR(Vector& x) const;
  bool LeastSquares_SVD(Vector& x) const;
  bool LeastSquares_GaussSeidel(Vector& x,int maxIters=100,Real tol=Epsilon) const;

  
  // Back substitution, supposing A is upper/lower triangular (U/L)
  bool LBackSubstitute(Vector& x) const;
  bool UBackSubstitute(Vector& x) const;
  bool LTBackSubstitute(Vector& x) const;

  // All solutions -- x=x0+N*y, where y is arbitrary
  bool AllSolutions(Vector& x0,Matrix& N) const;
  bool AllSolutions_RE(Vector& x0,Matrix& N) const;
  bool AllSolutions_SVD(Vector& x0,Matrix& N) const;
  
  inline bool IsValid() const { return A.m == b.n; }
  inline void Residual(const Vector& x, Vector& r) const { r.setNegative(b); A.madd(x,r); }

  const Matrix& A;
  const Vector& b;
};

}

#endif
