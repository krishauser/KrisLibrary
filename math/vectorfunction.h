#ifndef MATH_VECTOR_FUNCTION_H
#define MATH_VECTOR_FUNCTION_H

#include "function.h"
#include "vector.h"
#include "matrix.h"
#include <memory>
#include <vector>
#include <assert.h>

/** @ingroup Math
 * @file math/vectorfunction.h
 * @brief Various basic function classes.
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

///@brief
///A function g(t) that returns f(x+tn) for a scalar field f(x)
///If ref is true, then x,n are set to refer to the x,n provided in
///the constructor, rather than to copy.
///
///Note that this object is only valid for the duration that f
///is valid.
class ScalarFieldDirectionalFunction : public RealFunction
{
public:
  ScalarFieldDirectionalFunction(ScalarFieldFunction& _f,const Vector& x,const Vector& n,bool ref=false);
  virtual std::string Label() const;
  virtual void PreEval(Real t);
  virtual Real Eval(Real t);
  virtual Real Deriv(Real t);
  virtual Real Deriv2(Real t);
  virtual const Vector& LastEvaluation() const { return tmp; }

private:
  ScalarFieldFunction* f;
  Vector x;
  Vector n;
  Vector tmp;
};

///A function g(t) that returns f(x+t*ei), where ei is the ith standard
///basis vector. 
///If ref is true, then x is set to refer to the x provided in the constructor.
///
///Note that this object is only valid for the duration that f
///is valid.
class ScalarFieldProjectionFunction : public RealFunction
{
public:
  ScalarFieldProjectionFunction(ScalarFieldFunction& f,const Vector& x,int i);
  virtual std::string Label() const;
  virtual void PreEval(Real t);
  virtual Real Eval(Real t);
  virtual Real Deriv(Real t);
  virtual Real Deriv2(Real t);

private:
  ScalarFieldFunction* f;
  Real xi0;
  int i;
  Vector tmp;
};

/// A linear scalar field a'x+b
/// If ref is true, then a is set to refer to the a provided in
/// the constructor.
class LinearScalarFieldFunction : public ScalarFieldFunction
{
public:
  LinearScalarFieldFunction(const Vector& _a,Real _b,bool ref=false) {
    if(ref) {
      a.setRef(_a);
      b = _b;
    }
    else {
      a=_a;
      b=_b;
    }
  }

  virtual std::string Label() const { return "<a^t*x+b>"; }
  virtual Real Eval(const Vector& x) { return a.dot(x)+b; }
  virtual void Gradient(const Vector& x,Vector& grad) { grad = a; }
  virtual Real Gradient_i(const Vector& x,int i) { return a(i); }
  virtual void Hessian(const Vector& x,Matrix& H) { H.setZero(); }
  virtual Real Hessian_ij(const Vector& x,int i,int j) { return 0; }

  Vector a;
  Real b;
};

/// A quadratic scalar field x'Ax+b'x+c
/// If ref is true, then A,b are set to refer to the A,b provided in
/// the constructor.
class QuadraticScalarFieldFunction : public ScalarFieldFunction 
{
 public:
  QuadraticScalarFieldFunction(const Matrix& _A,const Vector& _b,Real _c,bool ref=false) {
    if(ref) {
      A.setRef(_A);
      b.setRef(_b);
      c = _c;
    }
    else {
      A=_A;
      b=_b;
      c=_c;
    }
  }
  virtual std::string Label() const { return "[0.5*x'Ax+bx+c]"; }
  virtual void PreEval(const Vector& x) { A.mul(x,Ax); }
  virtual Real Eval(const Vector& x) { return Half*x.dot(Ax) + x.dot(b) + c; }
  virtual void Gradient(const Vector& x,Vector& grad) 
  { grad.add(Ax,b); }
  virtual Real Gradient_i(const Vector& x,int i)
  { return Ax(i) + b(i); }
  virtual Real DirectionalDeriv(const Vector& x,const Vector& h)
  { return Ax.dot(h) + b.dot(h); }
  virtual void Hessian(const Vector& x,Matrix& H) { H = A; }
  virtual Real Hessian_ij(const Vector& x,int i,int j) { return A(i,j); }
  virtual Real DirectionalDeriv2(const Vector& x,const Vector& h)
  { Real sum = 0;
    for(int i=0;i<A.m;i++) sum += h(i)*A.dotRow(i,h);
    return sum;
  }

  Matrix A;
  Vector b;
  Real c;
  //temp
  Vector Ax;
};

/// A scalar field ||x||^2
struct NormSquaredScalarFieldFunction : public ScalarFieldFunction
{
  virtual std::string Label() const { return "<|x|^2>"; }
  virtual Real Eval(const Vector& x) { return x.normSquared(); }
  virtual void Gradient(const Vector& x,Vector& grad) { grad.add(x,x); }
  virtual Real Gradient_i(const Vector& x,int i) { return Two*x(i); }
  virtual void Hessian(const Vector& x,Matrix& H) { H.setIdentity(); H.inplaceMul(Two); }
  virtual Real Hessian_ij(const Vector& x,int i,int j) { return Two*Delta(i,j); }
};

/// A scalar field for the L-d norm, where d is passed into the constructor
struct NormScalarFieldFunction : public ScalarFieldFunction
{
  NormScalarFieldFunction(Real _degree=Two) : degree(_degree) {}
  virtual std::string Label() const;
  virtual void PreEval(const Vector& x);
  virtual Real Eval(const Vector& x);
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual Real Gradient_i(const Vector& x,int i);
  virtual void Hessian(const Vector& x,Matrix& H);
  virtual Real Hessian_ij(const Vector& x,int i,int j);

  Real degree;
  Real norm;
};

/// A scalar field min_i xi
class MinimumScalarFieldFunction : public ScalarFieldFunction
{
public:
  virtual std::string Label() const { return "<min_i xi>"; }
  virtual void PreEval(const Vector& x);
  virtual Real Eval(const Vector& x);
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual Real Gradient_i(const Vector& x,int i);
  virtual void Hessian(const Vector& x,Matrix& H) { H.setZero(); }
  virtual Real Hessian_ij(const Vector& x,Matrix& H) { return Zero; }

  Real xmin;
  int index;
};

/// A scalar field max_i xi
class MaximumScalarFieldFunction : public ScalarFieldFunction
{
public:
  virtual std::string Label() const { return "<max_i xi>"; }
  virtual void PreEval(const Vector& x);
  virtual Real Eval(const Vector& x);
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual Real Gradient_i(const Vector& x,int i);
  virtual void Hessian(const Vector& x,Matrix& H) { H.setZero(); }
  virtual Real Hessian_ij(const Vector& x,Matrix& H) { return Zero; }

  Real xmax;
  int index;
};

/// A scalar field h(x) = f(g(x))
class ComposeScalarFieldFunction : public ScalarFieldFunction
{
public:
  ComposeScalarFieldFunction(std::shared_ptr<RealFunction> _f,std::shared_ptr<ScalarFieldFunction> _g) : f(_f), g(_g) {}
  virtual std::string Label() const;
  virtual void PreEval(const Vector& x);
  virtual Real Eval(const Vector& x) { return f->Eval(gx); }
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual Real Gradient_i(const Vector& x,int i);
  virtual void Hessian(const Vector& x,Matrix& H);
  virtual Real Hessian_ij(const Vector& x,int i,int j); 
 
  std::shared_ptr<RealFunction> f;
  std::shared_ptr<ScalarFieldFunction> g;
  //temp
  Real gx;
};




class Vector3FieldFunction : public VectorFieldFunction
{
public:
  virtual ~Vector3FieldFunction() {}
  virtual void Curl(const Vector& x,Vector& curl)
  { 
    assert(x.n==3);
    curl.resize(3);
    curl(0)=Jacobian_ij(x,2,1)-Jacobian_ij(x,1,2);
    curl(1)=Jacobian_ij(x,0,2)-Jacobian_ij(x,2,0);
    curl(2)=Jacobian_ij(x,1,0)-Jacobian_ij(x,0,1);
  }
  virtual int NumDimensions() const { return 3; }
};

/// A linear vector field Ax+b
/// If ref is true, then A,b are set to refer to the A,b provided in
/// the constructor.
class LinearVectorFieldFunction : public VectorFieldFunction
{
 public:
  LinearVectorFieldFunction(const Matrix& _A,const Vector& _b,bool ref=false) {
    if(ref) {
      A.setRef(_A);
      b.setRef(_b);
    }
    else {
      A=_A;
      b=_b;
    }
  }
  virtual std::string Label() const { return "[A*x+b]"; }
  virtual int NumDimensions() const { return A.m; }
  virtual void Eval(const Vector& x,Vector& v) { A.mul(x,v); v+=b;  }
  virtual Real Eval_i(const Vector& x,int i) { return A.dotRow(i,x)+b(i); }
  virtual Real Jacobian_ij(const Vector& x,int i,int j) { return A(i,j); }
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji) { A.getRowCopy(i,Ji); }
  virtual void Jacobian_j(const Vector& x,int j,Vector& Jj) { A.getColCopy(j,Jj); }
  virtual void Jacobian(const Vector& x,Matrix& J) { J=A; }
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v) { A.mul(h,v); }
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi) { Hi.setZero(); }
  virtual Real Hessian_ijk(const Vector& x,int i,int j,int k) { return 0; }

  Matrix A;
  Vector b;
};

/// A scalar field function h(x) = f(g(x)) (g vector-valued)
class Compose_SF_VF_Function : public ScalarFieldFunction
{
public:
  Compose_SF_VF_Function(std::shared_ptr<ScalarFieldFunction> _f,std::shared_ptr<VectorFieldFunction> _g) : f(_f), g(_g) {}
  virtual std::string Label() const;
  virtual void PreEval(const Vector& x);
  virtual Real Eval(const Vector& x) { return f->Eval(gx); }
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual Real Gradient_i(const Vector& x,int i);
  virtual void Hessian(const Vector& x,Matrix& H);
  virtual Real Hessian_ij(const Vector& x,int i,int j);
  
  std::shared_ptr<ScalarFieldFunction> f;
  std::shared_ptr<VectorFieldFunction> g;
  //temp
  Vector gx;
  Vector gradf;
  Matrix Jg;
};

/// A vector field function h(x) = f(g(x)) (f,g vector fields)
class Compose_VF_VF_Function : public ScalarFieldFunction
{
public:
  Compose_VF_VF_Function(std::shared_ptr<VectorFieldFunction> _f,std::shared_ptr<VectorFieldFunction> _g) : f(_f), g(_g) {}
  virtual std::string Label() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x,Vector& v) { f->Eval(gx,v); }
  virtual Real Eval_i(const Vector& x,int i) { return f->Eval_i(gx,i); }
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void Jacobian_j(const Vector& x,int j,Vector& Jj);
 
  std::shared_ptr<VectorFieldFunction> f;
  std::shared_ptr<VectorFieldFunction> g;
  Vector gx;
  Matrix Jg,Jf;
};

/// A vector field function f(x) = (f1(x), ... , fn(x)) where f1...fn are
/// scalar fields
class ComponentVectorFieldFunction : public VectorFieldFunction
{
public:
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  std::vector<std::shared_ptr<ScalarFieldFunction> > functions;
};

/// A function g(x) that returns the i'th component of vector field f(x)
class VectorFieldProjectionFunction : public ScalarFieldFunction
{
public:
  VectorFieldProjectionFunction(VectorFieldFunction& f,int i);
  virtual std::string Label() const;
  virtual void PreEval(const Vector& x) { f->PreEval(x); }
  virtual Real Eval(const Vector& x);
  virtual void Gradient(const Vector& x,Vector& grad);

private:
  VectorFieldFunction* f;
  int i;
};


/// A vector field function f(x) = (f1(x), ... , fn(x)) where f1...fn are also
/// vector fields
class CompositeVectorFieldFunction : public VectorFieldFunction
{
public:
  CompositeVectorFieldFunction();
  CompositeVectorFieldFunction(std::shared_ptr<VectorFieldFunction> f1,std::shared_ptr<VectorFieldFunction>& f2);
  CompositeVectorFieldFunction(const std::vector<std::shared_ptr<VectorFieldFunction> >& fs);
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  int GetFunction(int &i) const;

  std::vector<std::shared_ptr<VectorFieldFunction> > functions;
};

/// A vector field function f(x) = g(x[xindices])[findices]
class IndexedVectorFieldFunction : public VectorFieldFunction
{
public:
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  std::shared_ptr<VectorFieldFunction> function;
  std::vector<int> xindices,findices;

  //temp
  Vector vf,xsub;
  Matrix Jf;
};

/// A vector field function f(x) = g(x0 | x0[xindices]=x)
class SliceVectorFieldFunction : public VectorFieldFunction
{
public:
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  std::shared_ptr<VectorFieldFunction> function;
  Vector x0;
  std::vector<int> xindices;

  //temp
  Vector vf,xfull;
  Matrix Jf;
};


/*@}*/

} //namespace Math

#endif
