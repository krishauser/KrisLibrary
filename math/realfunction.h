#ifndef MATH_REAL_FUNCTION_H
#define MATH_REAL_FUNCTION_H

#include "function.h"

namespace Math { 

//y=a*t+b
class LinearFunction : public RealFunction
{
public:
  LinearFunction(Real _a=One,Real _b=Zero) : a(_a),b(_b) {}
  virtual std::string Label() const { return "<a*t+b>"; }
  virtual Real Eval(Real t) { return a*t+b; }
  virtual Real Deriv(Real t) { return a; }
  virtual Real Deriv2(Real t) { return 0; }  

  Real a,b;
};

//y=1/t
class InverseFunction : public RealFunction
{
public:
  virtual std::string Label() const { return "<1/t>"; }
  virtual Real Eval(Real t) { return One/t; }
  virtual Real Deriv(Real t) { return -One/(t*t); }
  virtual Real Deriv2(Real t) { return Two/(t*t*t); }
};

//y=f(g(t))
class ComposeFunction : public RealFunction
{
public:
  ComposeFunction(RealFunction* _f,RealFunction* _g) : f(_f), g(_g) {}
  virtual std::string Label() const;
  virtual void PreEval(Real t);
  virtual Real Eval(Real t);
  virtual Real Deriv(Real t);
  virtual Real Deriv2(Real t);
  
  RealFunction *f,*g;
  Real gt;
};

//y=f(t)+g(t)
class AddFunction : public RealFunction
{
public:
  AddFunction(RealFunction* _f,RealFunction* _g) : f(_f), g(_g) {}
  virtual std::string Label() const;
  virtual void PreEval(Real t);
  virtual Real Eval(Real t);
  virtual Real Deriv(Real t);
  virtual Real Deriv2(Real t);
  
  RealFunction *f,*g;
};

//y=f(t)*g(t)
class MulFunction : public RealFunction
{
public:
  MulFunction(RealFunction* _f,RealFunction* _g) : f(_f), g(_g) {}
  virtual std::string Label() const;
  virtual void PreEval(Real t);
  virtual Real Eval(Real t);
  virtual Real Deriv(Real t);
  virtual Real Deriv2(Real t);
  
  RealFunction *f,*g;
  Real ft,gt;
};

} //namespace Math

#endif
