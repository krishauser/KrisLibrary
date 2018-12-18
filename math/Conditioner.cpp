#include "Conditioner.h"
#include <assert.h>
using namespace Math;
using namespace std;

Conditioner_SymmDiag::Conditioner_SymmDiag(Matrix& A,Vector& b,Method m)
  :MatrixEquation(A,b)
{
  CalculateS(m);
  if(S.n!=0) {
    //S.preMultiply(A,A);
    //S.postMultiply(A,A);
    //S.mulVector(b,b);
    S.preMultiplyInverse(A,A);
    S.postMultiplyInverse(A,A);
    S.mulInverse(b,b);
  }
}

void Conditioner_SymmDiag::Post(Vector& x) const
{
  if(S.n!=0) {
    //S.mulVector(x,x);
    S.mulInverse(x,x);
  }
}

void Conditioner_SymmDiag::CalculateS(Method m)
{
  switch(m) {
  case None: break;
  case NormalizeDiagonal: CalculateS_NormalizeDiagonal(); break;
  }
}

void Conditioner_SymmDiag::CalculateS_NormalizeDiagonal()
{
  assert(A.m == A.n);
  S.resize(A.n);
  for(int i=0;i<A.n;i++) {
    Real Aii=Abs(A(i,i));
    if(Aii==Zero) S(i) = One;
    //else S(i) = Sqrt(Inv(Aii));
    else S(i) = Sqrt(Aii);
  }
}
