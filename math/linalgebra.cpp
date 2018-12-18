#include <KrisLibrary/Logger.h>
#include "linalgebra.h"
#include "RowEchelon.h"
#include "GramSchmidt.h"
#include "CholeskyDecomposition.h"
#include "LDL.h"
#include "LUDecomposition.h"
#include "SVDecomposition.h"
#include "QRDecomposition.h"
#include "backsubstitute.h"
#include "random.h"
#include <iostream>
#include <errors.h>
#include <utils.h>
using namespace std;

namespace Math {

IterativeMethod::IterativeMethod(const Matrix& _A,const Vector& _b,Real _omega)
  :A(_A),b(_b),omega(_omega)
{}

void IterativeMethod::InitialRandom(Vector& x0,Real xmin,Real xmax) const
{
  x0.resize(A.n);
  for(int i=0;i<A.n;i++) x0(i) = Rand(xmin,xmax);
}

void IterativeMethod::InitialOnes(Vector& x0) const
{
  x0.resize(A.n);
  x0.set(One);
}

bool IterativeMethod::IsValid_Jacobi() const
{
  return A.isDiagonallyDominant();
}

bool IterativeMethod::IsValid_GaussSeidel() const
{
  return A.isSymmetric() || A.isDiagonallyDominant();
}

bool IterativeMethod::IsValid_SOR() const
{
  if(Zero <= omega || omega >= Two) return false;
  return A.isSquare();
}

void IterativeMethod::Iterate_Jacobi(Vector& x) const
{
  int i,j;
  Vector x2(x.n);
  for(i=0;i<A.n;i++) {
    Real sum=Zero;
    for(j=0;j<i;j++) sum += A(i,j)*x(j);
    for(j=i+1;j<A.n;j++) sum += A(i,j)*x(j);
    if(A(i,i) == Zero) x2(i) = Zero;
    else x2(i) = (b(i)-sum)/A(i,i);
  }
  x=x2;
}

void IterativeMethod::Iterate_GaussSeidel(Vector& x) const
{
  int i,j;
  for(i=0;i<A.n;i++) {
    Real sum=Zero;
    for(j=0;j<i;j++) sum += A(i,j)*x(j);
    for(j=i+1;j<A.n;j++) sum += A(i,j)*x(j);
    if(A(i,i) == Zero) x(i) = Zero;
    else x(i) = (b(i)-sum)/A(i,i);
  }
}

void IterativeMethod::Iterate_SOR(Vector& x) const
{
  int i,j;
  for(i=0;i<A.n;i++) {
    Real sum=Zero;
    for(j=0;j<i;j++) sum += A(i,j)*x(j);
    for(j=i+1;j<A.n;j++) sum += A(i,j)*x(j);
    if(A(i,i) == Zero) sum = Zero;
    else sum = (b(i)-sum)/A(i,i);
    x(i) += omega*(sum-x(i));
  }
}

bool IterativeMethod::Solve(Type type,Vector& x0,int& maxIters,Real& tol) const
{
  bool valid=false;
  switch(type) {
  case Jacobi: valid=IsValid_Jacobi(); break;
  case GaussSeidel: valid=IsValid_GaussSeidel(); break;
  case SOR: valid=IsValid_SOR(); break;
  default: AssertNotReached();
  }
  if(!valid) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: matrix in IterativeMethod::Solve() won't guarantee convergence");
  }

  Vector r;
  for(int i=0;i<maxIters;i++) {
    switch(type) {
    case Jacobi: Iterate_Jacobi(x0); break;
    case GaussSeidel: Iterate_GaussSeidel(x0); break;
    case SOR: Iterate_SOR(x0); break;
    default: AssertNotReached();
    }

    //calculate residual
    r.setNegative(b); A.madd(x0,r);
    Real rnorm=r.norm();
    if(rnorm <= Sqr(tol)) {
      tol = rnorm;
      maxIters = i;
      return true;
    }
  }
  return false;
}









MatrixEquation::MatrixEquation(const Matrix& _A, const Vector& _b)
  :A(_A),b(_b)
{
}

bool MatrixEquation::LBackSubstitute(Vector& x) const
{
  if(!A.isSquare() || !IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dims in LBackSubstitute");
    return false;
  }
  x.resize(A.n);
  return Math::LBackSubstitute(A,b,x);
}

bool MatrixEquation::UBackSubstitute(Vector& x) const
{
  if(!A.isSquare() || !IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dims in UBackSubstitute");
    return false;
  }
  x.resize(A.n);
  return Math::UBackSubstitute(A,b,x);
}

bool MatrixEquation::LTBackSubstitute(Vector& x) const
{
  if(!A.isSquare() || !IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dims in LTBackSubstitute");
    return false;
  }
  x.resize(A.n);
  return Math::LtBackSubstitute(A,b,x);
}

bool MatrixEquation::Solve(Vector& x) const
{
  return Solve_LU(x);
}

bool MatrixEquation::Solve_Cholesky(Vector& x) const
{
  if(!IsValid() || !A.isSquare()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dimensions in Solve_Cholesky");
    return false;
  }
  LDLDecomposition<Real> chol;
  chol.verbose = 0;
  chol.set(A);
  return chol.backSub(b,x);
}

bool MatrixEquation::Solve_LU(Vector& x) const
{
  if(!IsValid() || !A.isSquare()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dimensions in Solve_LU");
    return false;
  }
  LUDecomposition<Real> lu;
  if(!lu.set(A)) return false;
  lu.backSub(b,x);
  return true;
}

bool MatrixEquation::Solve_SVD(Vector& x) const
{
  Assert(A.isSquare());
  Assert(A.n == b.n);
  SVDecomposition<Real> svd;
  if(!svd.set(A)) return false;
  svd.backSub(b,x);
  return true;
}

bool MatrixEquation::Solve_Jacobi(Vector& x,int maxIters,Real tol) const
{
  IterativeMethod it(A,b);
  it.InitialOnes(x);
  int iters=maxIters;
  return it.Solve(IterativeMethod::Jacobi,x,iters,tol);
}

bool MatrixEquation::Solve_GaussSeidel(Vector& x,int maxIters,Real tol) const
{
  IterativeMethod it(A,b);
  it.InitialOnes(x);
  int iters=maxIters;
  return it.Solve(IterativeMethod::GaussSeidel,x,iters,tol);
}

bool MatrixEquation::Solve_SOR(Vector& x,Real omega,int maxIters,Real tol) const
{
  IterativeMethod it(A,b,omega);
  it.InitialOnes(x);
  int iters=maxIters;
  return it.Solve(IterativeMethod::SOR,x,iters,tol);
}

bool MatrixEquation::LeastSquares(Vector& x) const
{
  if(!IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dimensions in LeastSquares()");
    return false;
  }
  if(LeastSquares_Cholesky(x)) {
    return true;
  }
  else {
    return LeastSquares_SVD(x);
  }
}

bool MatrixEquation::LeastSquares_Cholesky(Vector& x) const
{
  if(!IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dimensions in LeastSquares_Cholesky()");
    return false;
  }
  if(A.m < A.n) {
    // x = At(AAt)^-1 b
    // y = (AAt)^-1 b => AAt*y = b
    Matrix AAt;
    AAt.mulTransposeB(A,A);
    
    Vector y;
    MatrixEquation ls(AAt,b);
    if(ls.Solve_Cholesky(y)) {
      A.mulTranspose(y,x);
      return true;
    }
    return false;
  }
  else {
    //(AtA)x = Atb  =>  (AtA)^-1Atb = x
    Real scale = 1.0/A.maxAbsElement();
    Matrix At,AtA;
    //using At is better than just A
    At.setTranspose(A);
    At *= scale;
    AtA.mulTransposeB(At,At);
    
    Vector Atb;
    At.mul(b,Atb);
    Atb *= scale;
    MatrixEquation ls(AtA,Atb);
    return ls.Solve_Cholesky(x);
  }
}

bool MatrixEquation::LeastSquares_GaussSeidel(Vector& x,int maxIters,Real tol) const
{
  if(!IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid dimensions in LeastSquares_GaussSeidel()");
    return false;
  }
  if(A.m < A.n) {
    // x = At(AAt)^-1 b
    // y = (AAt)^-1 b => AAt*y = b
    Matrix AAt;
    AAt.mulTransposeB(A,A);
    
    Vector y;
    MatrixEquation ls(AAt,b);
    if(ls.Solve_GaussSeidel(y)) {
      A.mulTranspose(y,x);
      return true;
    }
    return false;
  }
  else {
    //(AtA)x = Atb  =>  (AtA)^-1Atb = x
    Matrix At,AtA;
    //using At is better than just A
    At.setTranspose(A);
    AtA.mulTransposeB(At,At);
    
    Vector Atb;
    At.mul(b,Atb);
    MatrixEquation ls(AtA,Atb);
    return ls.Solve_GaussSeidel(x,maxIters,tol);
  }
}

bool MatrixEquation::LeastSquares_QR(Vector& x) const
{
  //qr automatically does the least-squares
  Assert(IsValid());
  QRDecomposition<Real> qr;
  if(A.m <= A.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, not sure if QR with m<n works");
    if(!qr.set(A)) return false;
    qr.backSub(b,x);
    return true;
  }
  else {
    if(!qr.set(A)) return false;
    qr.backSub(b,x);
    return true;
  }
}

bool MatrixEquation::LeastSquares_SVD(Vector& x) const
{
  //svd automatically does the least-squares
  Assert(IsValid());
  SVDecomposition<Real> svd;
  if(A.m <= A.n) {
    if(!svd.set(A)) return false;
    svd.backSub(b,x);
    return true;
  }
  else {
    if(!svd.set(A)) return false;
    svd.backSub(b,x);
    return true;
    /*
    LOG4CXX_ERROR(KrisLibrary::logger(),"Doing the transpose SVD");
    Matrix At; At.setRefTranspose(A);
    if(!svd.set(At)) return false;
    svd.getInverse(At);
    LOG4CXX_ERROR(KrisLibrary::logger(),"Result"<<MatrixPrinter(At)<<"\n");
    Matrix Ainv; Ainv.setRefTranspose(At);
    Ainv.mul(b,x);
    return true;
    */
  }
}

bool MatrixEquation::AllSolutions(Vector& x0,Matrix& N) const
{
  return AllSolutions_RE(x0,N);
  //return AllSolutions_SVD(x0,N);
}

bool MatrixEquation::AllSolutions_RE(Vector& x0,Matrix& N) const
{
  if(A.n < A.m) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: matrix is overconstrained");
  }
  RowEchelon<Real> re;
  re.set(A,b);
  re.getAllSolutions(x0,N);
  return true;
}

bool MatrixEquation::AllSolutions_SVD(Vector& x0,Matrix& N) const
{
  if(A.n < A.m) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: matrix is overconstrained");
  }
  SVDecomposition<Real> svd;
  if(!svd.set(A)) return false;
  svd.backSub(b,x0);
  svd.getNullspace(N);
  return true;
}




/*
#include "sparsematrix.h"

int GaussSeidel(const SparseMatrix& A, Vector& x, const Vector& b,
		 int& maxIters, Real& tol)
{
  Assert(A.n == x.n);
  Assert(A.m == b.n);
  Assert(A.n == A.n);
  static Vector r;
  r.resize(A.n);

  Real normb = norm(b);
  Real resid;

  if (normb == 0.0) {
    x.setZero();
    tol = 0;
    maxIters = 0;
    return 0;
  }

  Real sum;
  for(int i=1; i<=maxIters; i++) {
    //do an iteration
    for(int j=0; j<A.m; j++) {
      sum = Zero;

      int* min = A.col_indices + A.row_offsets[j];
      int* max = A.col_indices + A.row_offsets[j+1];
      Real* fmin = A.val_array + A.row_offsets[j];
      Real* fmax = A.val_array + A.row_offsets[j+1];
      Real* f;
      int* k;
      Real Ajj = Zero;  //need to pick out the Ajj component
      for(k=min, f=fmin; k<max; k++, f++) {
	if(*k == j)
	  Ajj = *f;
	else
	  sum += (*f)*x(*k);
      }

      if(Ajj == Zero) {
	x(j) = 0.0;
      }
      else {
	x(j) = (b(j) - sum)/Ajj;
      }
    }

    //r = b - A*x;
    A.mul(x,r);
    r.sub(b,r);

    //check the tolerance of r
    if((resid = (norm(r) / normb)) < tol) {
      tol = resid;
      maxIters = i;
      return 0;
    }
  }

  tol = resid;
  return 1;
}
*/


} //namespace Math
