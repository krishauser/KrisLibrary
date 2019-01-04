#ifndef SPLINE_POLYNOMIAL_H
#define SPLINE_POLYNOMIAL_H

#include <KrisLibrary/Logger.h>
#include <vector>
#include <iostream>
#include <assert.h>

namespace Spline {

/** @brief A simple polynomial class, p(x) = sum_{i=0}^n coef[i]*x^i.
 */
template <class T=double>
class Polynomial
{
 public:
  std::vector<T> coef;

  ///Initializes to the 0 polynomial
  Polynomial() {}
  ///Initializes to a constant polynomial
  Polynomial(T c);
  ///Sets the coefficient vector
  Polynomial(const std::vector<T>& _coef);
  ///Copy constructor
  Polynomial(const Polynomial<T>& p);

  ///cast constructor
  template <class T2>
  Polynomial(const Polynomial<T2>& p)
  {
    coef.resize(p.coef.size());
    for(size_t i=0;i<coef.size();i++)
      coef[i] = T(p.coef[i]);
  }

  void Resize(size_t s);
  ///Size is the size of the coefficient vector.  Similar to Degree but disregarding leading 0 coefficients
  size_t Size() const;
  ///The degree of the polynomial.  Correctly handles leading 0 coefficients
  int Degree() const;

  ///Sets coefficient i.  Automatically resizes the coefficient vector
  void SetCoef(size_t i,T value);
  ///Gets coefficient i.  If i is out of bounds of the coefficient vector, returns 0
  T GetCoef(size_t i) const;
 
  ///Pretty-prints this
  std::ostream& operator << (std::ostream& out) const;

  ///Evaluates the polynomial at x
  T Evaluate (T x) const;
  ///Evaluates the polynomial given another polynomial as x
  Polynomial<T> Evaluate(const Polynomial<T>& x) const;

  ///Evaluates the derivative at x
  T Derivative (T x) const;
  ///Evaluates the n'th derivative at x
  T Derivative (T x,int n) const;

  ///Returns the derivative of this polynomial
  Polynomial<T> Differentiate() const;

  ///Returns the antiderivative of this polynomial
  Polynomial<T> AntiDifferentiate() const;

  /// return the n'th derivative of this polynomial
  /// if n < 0, returns the -n'th antiderivative
  Polynomial<T> Differentiate(int n) const;

  ///The operator (x) is equivalent to evaluation
  inline T operator () (T x) const { return Evaluate(x); }
  ///The operator (x) is equivalent to evaluation
  inline Polynomial<T> operator () (const Polynomial<T>& x) const { return Evaluate(x); }

  ///Adds a constant offset
  void operator += (T val);
  ///Subtracts a constant offset
  void operator -= (T val);
  ///Scales by a constant
  void operator *= (T b);
  ///Divides by a constant (if b is 0, unexpected results will hold)
  void operator /= (T b);

  ///Adds a polynomial offset
  void operator += (const Polynomial<T>& b);
  ///Subtracts a polynomial offset
  void operator -= (const Polynomial<T>& b);
  ///Scales by a polynomial 
  void operator *= (const Polynomial<T>& b);
};


//unary -
template <class T>
inline Polynomial<T> operator - (const Polynomial<T>& a)
{
  Polynomial<T> res=a;
  res *= -1.0;
  return res;
}

template <class T>
inline Polynomial<T> operator + (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res += b;
  return res;
}

template <class T>
inline Polynomial<T> operator - (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res -= b;
  return res;
}

template <class T>
inline Polynomial<T> operator * (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res *= b;
  return res;
}

template <class T>
inline Polynomial<T> operator / (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res /= b;
  return res;
}


template <class T>
inline Polynomial<T> operator + (T a,const Polynomial<T>& b)
{
  Polynomial<T> res=b;
  res += a;
  return res;
}

template <class T>
inline Polynomial<T> operator - (T a,const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res -= b;
  return res;
}

template <class T>
inline Polynomial<T> operator * (T a,const Polynomial<T>& b)
{
  Polynomial<T> res=b;
  res *= a;
  return res;
}

//polynomial operations
template <class T>
inline Polynomial<T> operator + (const Polynomial<T>& a, const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res += b;
  return res;
}

template <class T>
inline Polynomial<T> operator - (const Polynomial<T>& a, const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res -= b;
  return res;
}

template <class T>
inline Polynomial<T> operator * (const Polynomial<T>& a, const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res *= b;
  return res;
}




template <class T>
Polynomial<T>::Polynomial(T c)
{
  coef.resize(1);
  coef[0] = c;
}
template <class T>
Polynomial<T>::Polynomial(const std::vector<T>& _coef)
:coef(_coef)
{}

template <class T>
Polynomial<T>::Polynomial(const Polynomial<T>& p)
:coef(p.coef)
{}

template <class T>
void Polynomial<T>::Resize(size_t s) { coef.resize(s,T(0)); }

template <class T>
size_t Polynomial<T>::Size() const { return coef.size(); }

template <class T>
int Polynomial<T>::Degree() const
{
  for(size_t i=0;i<coef.size();i++)
    if(coef[coef.size()-1-i]!=T(0))
      return int(coef.size()-1-i);
  return 0;
}

template <class T>
void Polynomial<T>::SetCoef(size_t i,T value)
{
  if(i >= coef.size())
    Resize(i+1);
  coef[i] = value;
}

template <class T>
T Polynomial<T>::GetCoef(size_t i) const
{
  if(i >= coef.size()) return T(0);
  return coef[i];
}

template <class T>
std::ostream& Polynomial<T>::operator << (std::ostream& out) const
{
  for(size_t i=0;i<coef.size();i++) {
    out << coef[coef.size()-1-i] << "x^" << coef.size()-1-i;
    if(i+1 != coef.size())
out << " + ";
  }
  return out;
}

template <class T>
T Polynomial<T>::Evaluate (T x) const
{
  //horner's rule
  T s = coef[coef.size()-1];
  for (size_t i=1;i<coef.size();i++)
    s = coef[coef.size()-i-1] + ( x * s );
  return s;
}

template <class T>
Polynomial<T> Polynomial<T>::Evaluate(const Polynomial<T>& x) const 
{
  Polynomial<T> s(coef[coef.size()-1]);
  for (size_t i=1;i<coef.size();i++) {
    s = Polynomial<T>(coef[coef.size()-i-1]) + ( x * s );
  }
  return s;
}

template <class T>
T Polynomial<T>::Derivative (T x) const
{
  T s = 0;
  T xi = 1;
  for (size_t i=1;i<coef.size();i++) {
    s += T(i)*coef[i]*xi;
    xi *= x;
  }
  return s;
}

template <class T>
T Polynomial<T>::Derivative (T x,int n) const
{
  assert(n >= 0);
  if(n == 0) return Evaluate(x);
  else if(n == 1) return Derivative(x);
  return Differentiate(n).Evaluate(x);
}

template <class T>
Polynomial<T> Polynomial<T>::Differentiate() const
{
  if(Size() <= 1) {
    return Polynomial<T>(0);
  }

  Polynomial<T> deriv;
  deriv.coef.resize(coef.size()-1);
  for(size_t i=0;i+1<coef.size();i++)
    deriv.coef[i] = (T(i+1))*coef[i+1];
  return deriv;
}

template <class T>
Polynomial<T> Polynomial<T>::AntiDifferentiate() const
{
  if(Size() <= 1) {
    return Polynomial<T>(0);
  }

  Polynomial<T> anti;
  anti.coef.resize(coef.size()+1);
  for(size_t i=0;i<coef.size();i++)
    anti.coef[i+1] = coef[i]/(T(i+1));
  return anti;
}

template <class T>
Polynomial<T> Polynomial<T>::Differentiate(int n) const
{
  if(n < 0) {
    if(n == -1) return AntiDifferentiate();
    else return Differentiate(n+1).AntiDifferentiate();
  }
  else {
    if(n >= (int)Size()) return Polynomial<T>(0);
    if(n == 0) return *this;
    if(n == 1) return Differentiate();
    return Differentiate(n-1).Differentiate();
  }
}

template <class T>
void Polynomial<T>::operator += (T val)
{
  if(coef.empty()) Resize(1);
  for(size_t i=0;i<coef.size();i++) coef[i] += val;
}

template <class T>
void Polynomial<T>::operator -= (T val)
{
  if(coef.empty()) Resize(1);
  for(size_t i=0;i<coef.size();i++) coef[i] -= val;
}

template <class T>
void Polynomial<T>::operator *= (T b)
{
  for (size_t i=0;i<coef.size();i++)
coef[i] *= b;
}

template <class T>
void Polynomial<T>::operator /= (T b)
{
  for (size_t i=0;i<coef.size();i++)
coef[i] /= b;
}

template <class T>
void Polynomial<T>::operator += (const Polynomial<T>& b)
{
  if(b.coef.size() > coef.size()) Resize(b.Size());
  for(size_t i=0;i<b.coef.size();i++) coef[i] += b.coef[i];
}

template <class T>
void Polynomial<T>::operator -= (const Polynomial<T>& b)
{
  if(b.coef.size() > coef.size()) Resize(b.Size());
  for(size_t i=0;i<b.coef.size();i++) coef[i] -= b.coef[i];
}

template <class T>
void Polynomial<T>::operator *= (const Polynomial<T>& b)
{
  std::vector<T> newCoef(Degree()+b.Degree()+1,T(0));
  for (size_t i=0;i<coef.size();i++)
    for (size_t j=0;j<b.coef.size();j++)
newCoef[i+j] += (coef[i] * b.coef[j]);
  swap(coef,newCoef);
}


} //namespace Spline

#endif
