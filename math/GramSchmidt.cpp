#include <KrisLibrary/Logger.h>
#include "GramSchmidt.h"
#include "complex.h"
#include <iostream>
using namespace std;

namespace Math {

template <class T>
int OrthonormalBasis(const VectorTemplate<T>* x, VectorTemplate<T>* basis, int n)
{
  int k=0;			//counter to number of nonzero output vectors
  VectorTemplate<T> res;
  for(int i=0; i<n; i++) {
    res = x[i];
    for(int j=0; j<k; j++) {  //subtract projection onto basis[j]
      res.madd(basis[j],-basis[j].dot(res));  //here res instead of x[i] gives better numerical stability
    }
    basis[k]=res;
    T lensquared = basis[k].normSquared();
    if(lensquared != Zero) {
      basis[k].inplaceMul(Inv(Sqrt(lensquared)));
      k++;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"Redundant vector "<<i);
    }
  }
  return k;
}

template <class T>
int OrthogonalBasis(const VectorTemplate<T>* x, VectorTemplate<T>* basis, int n)
{
  int k=0;			//counter to number of nonzero output vectors
  T* basisSquared = new T[n];
  VectorTemplate<T> res;
  for(int i=0; i<n; i++) {
    res = x[i];
    for(int j=0; j<k; j++) {  //subtract projection onto basis[j]
      res.madd(basis[j], -basis[j].dot(res)/basisSquared[j]);  //here res rather than x[i] gives better numerical stability
    }
    basis[k] = res;
    basisSquared[k] = basis[k].normSquared();
    if(basisSquared[k] != Zero) {
      k++;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"Redundant vector "<<i);
    }
  }
  delete [] basisSquared;
  return k;
}

template <class T>
void Orthogonalize(VectorTemplate<T>& x,const VectorTemplate<T>* basis, int n)
{
  for(int i=0;i<n;i++)
    x.madd(basis[i],-basis[i].dot(x)/basis[i].normSquared());
}



#define DEFINEGRAMSCHMIDT(T) \
  template int OrthonormalBasis<T>(const VectorTemplate<T>* x, VectorTemplate<T>* basis, int n); \
  template int OrthogonalBasis<T>(const VectorTemplate<T>* x, VectorTemplate<T>* basis, int n); \
  template void Orthogonalize<T>(VectorTemplate<T>& x,const VectorTemplate<T>* basis, int n);
DEFINEGRAMSCHMIDT(float);
DEFINEGRAMSCHMIDT(double);
DEFINEGRAMSCHMIDT(Complex);


} //namespace Math
