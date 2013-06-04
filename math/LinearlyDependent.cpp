#include "LinearlyDependent.h"

namespace Math {

template <class T>
bool LinearlyDependent_Robust(const VectorTemplate<T>& a, const VectorTemplate<T>& b, T& c, bool& cb, T eps) 
{
  assert(a.n == b.n);
  T aDotB = a.dot(b);
  T aNorm2 = a.normSquared();
  if(aNorm2 > Abs(aDotB)) {
    cb = false;
    c = aDotB/aNorm2;
    T aNorm = Sqrt(aNorm2);
    T relEps = eps*aNorm;
    for(int i=0;i<a.n;i++) {
      if(!FuzzyEquals(c*a[i],b[i],relEps)) return false;
    }
    return true;
  }
  else {
    T bNorm2 = b.normSquared();
    cb = true;
    if(bNorm2 == Zero) {  //both a and b are 0
      c = One;
      return true;
    }
    c = aDotB/bNorm2;
    T bNorm = Sqrt(bNorm2);
    T relEps = eps*bNorm;
    for(int i=0;i<a.n;i++) {
      if(!FuzzyEquals(a[i],c*b[i],relEps)) return false;
    }
    return true;
  }
}


} //namespace Math
