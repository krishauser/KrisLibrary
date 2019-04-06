#include <KrisLibrary/Logger.h>
#include "CholeskyDecomposition.h"
#include "backsubstitute.h"
#include "complex.h"
#include <iostream>
#include <errors.h>

namespace Math {

template <class T>
CholeskyDecomposition<T>::CholeskyDecomposition()
:zeroEpsilon((T)1e-10)
{}

template <class T>
CholeskyDecomposition<T>::CholeskyDecomposition(const MatrixT& A)
:zeroEpsilon((T)1e-10)
{
	set(A);
}

template <class T>
CholeskyDecomposition<T>::CholeskyDecomposition(const MatrixT& A,MatrixT& L)
:zeroEpsilon((T)1e-10)
{
  setDestination(L);
  set(A);
}

template <class T>
void CholeskyDecomposition<T>::setDestination(MatrixT& L_input)
{
  L.setRef(L_input);
}

template <class T>
bool CholeskyDecomposition<T>::set(const MatrixT& A)
{
	if(A.m != A.n) return false;
  int n=A.n; 
	L.resize(n,n);

  int i,j,k;
  T temp,lii;
  for(i=0; i<n; i++) {
    temp = A(i,i);
    for(k=0; k<i; k++)
      temp -= Sqr(L(i,k));

	  if(temp <= -Zero) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"CholeskyDecomposition: A is not positive definite!\n");
	    LOG4CXX_INFO(KrisLibrary::logger(),"   "<<i<<"'th row, temp is "<<temp);
	    return false;
	  }
	  lii = Sqrt(temp);
	  if(lii < zeroEpsilon) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"CholeskyDecomposition: A is not strictly positive definite!\n");
	    lii = (T)zeroEpsilon;
	  }
    L(i,i)=lii;

    for(j=i+1; j<n; j++) {
      temp = A(i,j);
      for(k=0; k<i; k++) 
        temp -= L(i,k)*L(j,k);
      L(j,i) = temp / lii;
    }

    for(j=0; j<i; j++)
      L(j,i) = Zero;
  }
  return true;
}

using namespace std;

template <class T>
bool CholeskyDecomposition<T>::setPerturbed(const MatrixT& A,T& lambda)
{
  if(A.m != A.n) return false;
  int n=A.n;
  L.resize(n,n);

  int i,j,k;
  T temp,lii;
  lambda = Zero;
  for(i=0; i<n; i++) {
    temp = A(i,i);
    for(k=0; k<i; k++)
      temp -= Sqr(L(i,k));

	  if(temp < zeroEpsilon) {
	    if(temp + lambda <= zeroEpsilon) 
	      lambda = -temp+zeroEpsilon;
	    lii = Sqrt(temp+lambda);
	  }
	  else {
	    lii = Sqrt(temp);
	  }
    L(i,i) = lii;

    for(j=i+1; j<n; j++) {
      temp = A(i,j);
      for(k=0; k<i; k++)
        temp -= L(i,k)*L(j,k);
      L(j,i) = temp / lii;
    }
  }

  if(lambda == Zero) return true;
  LOG4CXX_INFO(KrisLibrary::logger(),"Lambda is "<<lambda);

  for(i=0; i<n; i++) {
    temp = A(i,i);
    for(k=0; k<i; k++)
      temp -= Sqr(L(i,k));
	  //if(temp < lambda)
	  //  lii = Sqrt(Min(temp+lambda,lambda));
	  //else
	    lii = Sqrt(temp+lambda);
    L(i,i) = lii;

    for(j=i+1; j<n; j++) {
      temp = A(i,j);
      for(k=0; k<i; k++)
        temp -= L(i,k)*L(j,k);
      L(j,i) = temp / lii;
    }

    for(j=0; j<i; j++)
      L(j,i) = Zero;
  }
  return true;
}

template <class T>
void CholeskyDecomposition<T>::backSub(const VectorT& b, VectorT& x) const
{
	//LLt*x=b
	//Lt*x=L^-1*b=y
	//x=(Lt^-1)y
	VectorT y;
	LBackSub(b,y);
	LTBackSub(y,x);
}

template <class T>
void CholeskyDecomposition<T>::backSub(const MatrixT& B, MatrixT& X) const
{
  X.resize(B.m,B.n);
  MatrixT temp(B.m,B.n);
  if(!LBackSubstitute(L,B,temp)) FatalError("CholeskyDecomposition: LBackSubstitute failed!");
  if(!LtBackSubstitute(L,temp,X)) FatalError("CholeskyDecomposition: LtBackSubstitute failed!");
}

template <class T>
void CholeskyDecomposition<T>::LBackSub(const VectorT& b, VectorT& x) const
{
	x.resize(L.n);
	if(!LBackSubstitute(L,b,x)) FatalError("CholeskyDecomposition: LBackSubstitute failed!");
}

template <class T>
void CholeskyDecomposition<T>::LTBackSub(const VectorT& b, VectorT& x) const
{
	x.resize(L.n);
	if(!LtBackSubstitute(L,b,x)) FatalError("CholeskyDecomposition: LtBackSubstitute failed!");
}

template <class T>
void CholeskyDecomposition<T>::getInverse(MatrixT& Ainv) const
{
  Ainv.resize(L.n,L.n);
  VectorT temp(L.n,(T)Zero),y,x;
  for(int i=0;i<L.n;i++) {
    Ainv.getColRef(i,x);  //x &= col i of A
    temp(i)=One;
    LBackSub(temp,y);
    LTBackSub(y,x);
    temp(i)=Zero;
  }
}

template<class T>
void CholeskyDecomposition<T>::update(const VectorT& _x)
{
  VectorT x = _x;  //make a copy, we'll change it
  int n=L.n;
  Assert(x.n == n);
  T alpha=1;
  for(int i=0;i<n;i++) {
    T deltai = Sqr(L(i,i));
    T temp = alpha + Sqr(x(i))/deltai;
    deltai = deltai*temp;
    T gamma = x(i)/deltai;
    deltai = deltai / alpha;
    alpha = temp;
    L(i,i) = Sqrt(deltai);
    for(int k=i+1;k<n;k++) {
      x(k) -= x(i)*L(k,i);
      L(k,i) += gamma*x(k);
    }
  }
}

template <class T>
bool CholeskyDecomposition<T>::downdate(const VectorT& _x)
{
  VectorT x = _x;  //make a copy, we'll change it
  int n=L.n;
  Assert(x.n == n);
  T alpha=1;
  for(int i=0;i<n;i++) {
    T deltai = Sqr(L(i,i));
    T temp = alpha - Sqr(x(i))/deltai;
    deltai = deltai*temp;
    if(deltai == 0) return false;
    T gamma = x(i)/deltai;
    deltai = deltai / alpha;
    alpha = temp;
    if(deltai < 0) return false;
    L(i,i) = Sqrt(deltai);
    for(int k=i+1;k<n;k++) {
      x(k) -= x(i)*L(k,i);
      L(k,i) -= gamma*x(k);
    }
  }
  return true;
}



/*
//template instantiation for Complex
bool CholeskyDecomposition<Complex>::set(const MatrixT& A)
{
	if(A.m != A.n) return false;
  int n=A.n; 
	L.resize(n,n);

  int i,j,k;
  Complex temp,lii;
  for(i=0; i<n; i++) {
    temp = A(i,i);
    for(k=0; k<i; k++)
      temp -= Sqr(L(i,k));

    lii = Sqrt(temp);
	  if(Abs(lii) < zeroEpsilon.x) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"CholeskyDecomposition<Complex>: A is not strictly positive definite!\n");
	    return false;
	  }
    L(i,i)=lii;

    for(j=i+1; j<n; j++) {
      temp = A(i,j);
      for(k=0; k<i; k++) 
        temp -= L(i,k)*L(j,k);
      L(j,i) = temp / lii;
    }

    for(j=0; j<i; j++)
      L(j,i) = Zero;
  }
  return true;
}

bool CholeskyDecomposition<Complex>::setPerturbed(const MatrixT& A,Complex& lambda)
{
	LOG4CXX_INFO(KrisLibrary::logger(),"CholeskyDecomposition<Complex>: Perturbed decomposition isn't defined\n");
	return false;
}

*/

template class CholeskyDecomposition<float>;
template class CholeskyDecomposition<double>;
//template class CholeskyDecomposition<Complex>;

} //namespace Math
