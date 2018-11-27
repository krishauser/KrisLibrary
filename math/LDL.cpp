#include <KrisLibrary/Logger.h>
#include "LDL.h"
#include "backsubstitute.h"
#include "DiagonalMatrix.h"
#include <errors.h>
#include <iostream>
using namespace std;

namespace Math {

template<class T>
LDLDecomposition<T>::LDLDecomposition()
  :zeroTolerance((T)1e-8),verbose(1)
{}

template<class T>
LDLDecomposition<T>::LDLDecomposition(const MatrixT& A)
  :zeroTolerance((T)1e-8),verbose(1)
{
  set(A);
}

template<class T>
void LDLDecomposition<T>::set(const MatrixT& A)
{
  Assert(A.m == A.n);
  LDL.resize(A.n,A.n);
  int i,j,k;
  T sum;
  for(i=0;i<A.n;i++) {
    sum = A(i,i);
    for(k=0;k<i;k++) sum -= LDL(k,k)*Sqr(LDL(i,k));
    LDL(i,i) = sum;
    if(FuzzyZero(sum,zeroTolerance)) {
      /*
      if(verbose >= 1)
	LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: LDLt decomposition has a zero element on diagonal "<<i);
      */
    }

    for(j=i+1;j<A.n;j++) {
      sum = A(i,j);
      for(int k=0;k<i;k++) sum -= LDL(k,k)*LDL(i,k)*LDL(j,k);
      if(LDL(i,i) == 0) {
	if(!FuzzyZero(sum,zeroTolerance)) {
	  if(verbose >= 1) LOG4CXX_ERROR(KrisLibrary::logger(),"LDLDecomposition: Zero diagonal, what to do with sum "<<sum<<"?");
	  sum = 0;
	}
      }
      else 
	sum /= LDL(i,i);
      LDL(j,i) = LDL(i,j) = sum;
    }
  }


  /*
  MatrixT L,LD,LDLt;
  VectorT D;
  getL(L);
  getD(D);
  //LOG4CXX_INFO(KrisLibrary::logger(),"A: "); A.print();
  //LOG4CXX_INFO(KrisLibrary::logger(),"L: "); L.print();
  //LOG4CXX_INFO(KrisLibrary::logger(),"D: "); D.print();
  LD = L;
  for(int i=0;i<A.n;i++)
    LD.scaleCol(i,LDL(i,i));
  LDLt.mulTransposeB(LD,L);
  //LOG4CXX_INFO(KrisLibrary::logger(),"LDLt: "); LDLt.print();
  LDLt -= A;
  LOG4CXX_ERROR(KrisLibrary::logger(),"Max error in LDL "<<LDLt.maxAbsElement());
  */
}

template<class T>
bool LDLDecomposition<T>::backSub(const VectorT& b, VectorT& x) const
{
  //LDLt*x=b
  //DLt*x=L^-1*b=y
  //Lt*x=D^-1*y=y'
  //x=(Lt^-1)y
  VectorT y;
  LBackSub(b,y);
  bool res=DBackSub(y,y);
  LTBackSub(y,x);
  return res;
}

template<class T>
bool LDLDecomposition<T>::backSub(const MatrixT& B, MatrixT& X) const
{
  X.resize(B.m,B.n);
  MatrixT temp(B.m,B.n);
  L1BackSubstitute(LDL,B,temp);
  VectorT tempi;
  bool res=true;
  for(int i=0;i<temp.n;i++) {
    temp.getColRef(i,tempi);
    if(!DBackSub(tempi,tempi)) res=false;
  }
  Lt1BackSubstitute(LDL,temp,X);
  return res;
}

template<class T>
void LDLDecomposition<T>::LBackSub(const VectorT& b, VectorT& x) const
{
  Assert(b.n == LDL.n);
  x.resize(LDL.n);
  L1BackSubstitute(LDL,b,x);
}

template<class T>
bool LDLDecomposition<T>::DBackSub(const VectorT& b, VectorT& x) const
{
  bool res=true;
  x.resize(b.n);
  Assert(b.n==x.n);
  for(int i=0;i<x.n;i++) {
    if(!FuzzyZero(LDL(i,i),zeroTolerance))
      x(i) = b(i)/LDL(i,i);
    else {
      if(!FuzzyZero(b(i),zeroTolerance)) {
	if(verbose >= 1) 
	  LOG4CXX_ERROR(KrisLibrary::logger(),"LDLDecomposition::DBackSub(): Warning, zero on the diagonal, b("<<i<<")="<<b(i));
	res = false;
	x(i) = Sign(b(i))*Inf;
      }
      else
	x(i) = 0;
    }
  }
  return res;
}

template<class T>
void LDLDecomposition<T>::LTBackSub(const VectorT& b, VectorT& x) const
{
  Assert(b.n == LDL.n);
  x.resize(LDL.n);
  Lt1BackSubstitute(LDL,b,x);
}

template<class T>
bool LDLDecomposition<T>::getInverse(MatrixT& Ainv) const
{
  Ainv.resize(LDL.n,LDL.n);
  bool res=true;
  VectorT temp(LDL.n,Zero),y,x;
  for(int i=0;i<LDL.n;i++) {
    temp(i)=One;
    LBackSub(temp,y);
    if(!DBackSub(y,y)) res=false;
    LTBackSub(y,x);
    //fill in a column
    for(int j=0;j<LDL.n;j++)
      Ainv(j,i)=x(j);
    temp(i)=Zero;
  }
  return true;
}

template<class T>
void LDLDecomposition<T>::getPseudoInverse(MatrixT& Ainv) const
{
  Ainv.resize(LDL.n,LDL.n);
  VectorT temp(LDL.n,Zero),y,x;
  for(int i=0;i<LDL.n;i++) {
    temp(i)=One;
    LBackSub(temp,y);
    for(int j=0;j<y.n;j++) {
      if(!FuzzyZero(LDL(j,j),zeroTolerance))
	y(j) = y(j)/LDL(j,j);
      else
	y(j) = 0.0;
    }
    LTBackSub(y,x);
    //fill in a column
    for(int j=0;j<LDL.n;j++)
      Ainv(j,i)=x(j);
    temp(i)=Zero;
  }

  T tol = Ainv.maxAbsElement()*Epsilon;
  for(int i=0;i<LDL.n;i++)
    for(int j=0;j<i;j++) {
      if(!FuzzyEquals(Ainv(i,j),Ainv(j,i),tol))
	LOG4CXX_INFO(KrisLibrary::logger(),Ainv);
      Assert(FuzzyEquals(Ainv(i,j),Ainv(j,i),tol));
      Ainv(i,j)=Ainv(j,i) = 0.5*(Ainv(i,j)+Ainv(j,i));
    }
}

template<class T>
void LDLDecomposition<T>::getL(MatrixT& L) const
{
  Assert(LDL.m == LDL.n);
  L.resize(LDL.m,LDL.n);
  for(int i=0;i<LDL.n;i++) {
    L(i,i) = One;
    for(int j=0;j<i;j++)
      L(i,j) = LDL(i,j);
    for(int j=i+1;j<LDL.n;j++)
      L(i,j) = Zero;
  }
}

template<class T>
void LDLDecomposition<T>::getD(VectorT& d) const
{
  Assert(LDL.m == LDL.n);
  d.resize(LDL.n);
  LDL.getDiagCopy(0,d);
}

template <class T>
void LDLDecomposition<T>::mulL(const Vector& x,Vector& y) const
{
  int n=LDL.n;
  Assert(x.n == n);
  y.resize(n);
  for(int i=0;i<n;i++) {
    Real sum = x(i);  //Lii = 1
    for(int j=0;j<i;j++)
      sum += LDL(i,j)*x(j);
    y(i) = sum;
  }
}

template <class T>
void LDLDecomposition<T>::mulLT(const Vector& x,Vector& y) const
{
  int n=LDL.n;
  Assert(x.n == n);
  y.resize(n);
  for(int i=0;i<n;i++) {
    Real sum = x(i);  //Lii = 1
    for(int j=i+1;j<n;j++)
      sum += LDL(j,i)*x(j);
    y(i) = sum;
  }
}

template <class T>
void LDLDecomposition<T>::mulD(const Vector& x,Vector& y) const
{
  int n=LDL.n;
  Assert(x.n == n);
  y.resize(n);
  for(int i=0;i<n;i++) y(i) = x(i)*LDL(i,i);
}

template <class T>
void LDLDecomposition<T>::getA(MatrixT& A) const
{
  MatrixT L,temp;
  DiagonalMatrixTemplate<T> D;
  getL(L);
  getD(D);
  D.postMultiply(L,temp);
  A.mulTransposeB(temp,L);
}

template<class T>
void LDLDecomposition<T>::update(const VectorT& _x)
{
  VectorT x = _x;  //make a copy, we'll change it
  int n=LDL.n;
  Assert(x.n == n);

  T alpha=1;
  for(int i=0;i<n;i++) {
    T deltai = LDL(i,i);
    T temp = alpha + Sqr(x(i))/deltai;
    deltai = deltai*temp;
    T gamma = x(i)/deltai;
    deltai = deltai / alpha;
    alpha = temp;
    LDL(i,i) = deltai;
    for(int k=i+1;k<n;k++) {
      x(k) -= x(i)*LDL(k,i);
      LDL(k,i) += gamma*x(k);
    }
  }
}

template <class T>
bool LDLDecomposition<T>::downdate(const VectorT& _x)
{
  VectorT x = _x;  //make a copy, we'll change it
  int n=LDL.n;
  Assert(x.n == n);

  T alpha=1;
  for(int i=0;i<n;i++) {
    T deltai = LDL(i,i);
    T temp = alpha - Sqr(x(i))/deltai;
    deltai = deltai*temp;
    if(FuzzyZero(deltai,zeroTolerance)) return false;
    T gamma = x(i)/deltai;
    deltai = deltai / alpha;
    alpha = temp;
    LDL(i,i) = deltai;
    for(int k=i+1;k<n;k++) {
      x(k) -= x(i)*LDL(k,i);
      LDL(k,i) -= gamma*x(k);
    }
  }
  return true;
}

template class LDLDecomposition<float>;
template class LDLDecomposition<double>;
//template class LDLDecomposition<Complex>;

} //namespace Math
