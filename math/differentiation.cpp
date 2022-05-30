#include <KrisLibrary/Logger.h>
#include "differentiation.h"
#include "vectorfunction.h"
#include <vector>
#include <errors.h>
using namespace std;

namespace Math {

void Div(Vector& x,Real h)
{
  if(h < Epsilon)
    x /= h;
  else
    x *= One/h;
}

void Div(Matrix& x,Real h)
{
  if(h < Epsilon)
    x /= h;
  else
    x *= One/h;
}

Real dfCenteredDifference(RealFunction& f, Real x, Real h)
{
  return (f(x+h)-f(x-h))/(Two*h);
}

void dfCenteredDifference(VectorFunction& f, Real x, Real h, Vector& df)
{
  Vector temp(f.NumDimensions());
  f(x+h,temp);
  df = temp;
  f(x-h,temp);
  df -= temp;
  Div(df,Two*h);
}

Real dfCenteredDifferenceAdaptive(RealFunction& f, Real x, Real h0, Real tol)
{
  Real df1  = dfCenteredDifference(f,x,h0);
  Real df2  = dfCenteredDifference(f,x,h0*0.5);
  Real scale = 8.0/7.0;
  while(h0 > Epsilon) {
    if(scale*Abs(df1 - df2) < tol) return df2;
    df1 = df2;
    h0 *= 0.5;
    df2 = dfCenteredDifference(f,x,h0*0.5);
  }
  return df2;
}

void dfCenteredDifferenceAdaptive(VectorFunction& f, Real x, Real h0, Real tol, Vector& df)
{
  Vector df0(f.NumDimensions());
  Vector temp(f.NumDimensions());
  dfCenteredDifference(f,x,h0,df0);
  dfCenteredDifference(f,x,h0*0.5,df);
  Real scale = 8.0/7.0;
  while(h0 > Epsilon) {
    Real maxDiff = 0.0;
    for(int i=0;i<df.n;i++)
      maxDiff = Max(maxDiff,Abs(df[i]-df0[i]));
    if(scale*maxDiff < tol) return;
    df0 = df;
    h0 *= 0.5;
    dfCenteredDifference(f,x,h0*0.5,df);
  }
}


//centered differences for 2nd derivative
//f''(x) ~= 1/h^2*(f(x+h) - 2f(x) + f(x-h)) + O(h^2)
Real ddfCenteredDifference(RealFunction& f, Real x, Real h)
{
  return (f(x+h)-Two*f(x)+f(x-h))/Sqr(h);
}

void ddfCenteredDifference(VectorFunction& f, Real x, Real h, Vector& ddf)
{
  Vector temp(f.NumDimensions());
  f(x+h,temp);
  ddf = temp;
  f(x,temp);
  ddf.madd(temp,-Two);
  f(x-h,temp);
  ddf += temp;
  Div(ddf,Sqr(h));
}


void GradientForwardDifference(ScalarFieldFunction& f,Vector& x,Real h,Vector& g)
{
  if(g.empty()) g.resize(x.n);
  Assert(g.n == x.n);
  Real f0 = f(x),f1;
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    x(i) += h; f1=f(x);
    x(i) = xi;

    g(i) = (f1-f0);
  }
  Div(g,h);
}

void JacobianForwardDifference(VectorFieldFunction& f,Vector& x,Real h,Matrix& J)
{
  int nd = f.NumDimensions();
  Vector f0(nd),f1(nd);
  if(J.isEmpty()) J.resize(f.NumDimensions(),x.n);
  Assert(J.m == f.NumDimensions() && J.n == x.n);
  f(x,f0);
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    x(i) += h;  f(x,f1);
    x(i) = xi;

    Vector Ji; J.getColRef(i,Ji);
    Ji.sub(f1,f0);
  }
  Div(J,h);
}

void HessianForwardDifference(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H)
{
  Assert(H.hasDims(x.n,x.n));
  Real f00,f01,f10,f11;
  f00 = f(x);
  Real scale = Inv(Sqr(h));
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    for(int j=i;j<x.n;j++) {
      Real xj=x(j);
      x(j) += h;   f01 = f(x);
      x(i) += h;   f11 = f(x);
      x(j) = xj;   f10 = f(x);
      x(i) = xi; 

      H(i,j) = H(j,i) = (f11-f10-f01+f00)*scale;
    }
  }
}

void HessianForwardDifference_Grad(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H)
{
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  Vector g0(x.n);
  vector<Vector> g(x.n);
  f.PreEval(x);
  f.Gradient(x,g0);
  for(int i=0;i<x.n;i++) {
    g[i].resize(x.n);
    Real xi=x(i);
    x(i)+=h;
    f.PreEval(x);
    f.Gradient(x,g[i]);
    x(i) = xi;
  }
  Real scale = Inv(Two*h);
  for(int i=0;i<x.n;i++) {
    for(int j=i;j<x.n;j++) {
      H(i,j) = H(j,i) = (g[j](i) - g0(i) + g[i](j)-g0(j))*scale;
    }
  }
}

void HessianForwardDifference(VectorFieldFunction& f,Vector& x,Real h,vector<Matrix>& H)
{
  Assert((int)H.size() == f.NumDimensions());
  for(size_t i=0;i<H.size();i++)
    Assert(H[i].hasDims(x.n,x.n));
  int n=f.NumDimensions();
  Vector f00(n),f01(n),f10(n),f11(n);
  f(x,f00);
  Real scale = Inv(Sqr(h));
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    for(int j=i;j<x.n;j++) {
      Real xj=x(j);
      x(j) += h;   f(x,f01);
      x(i) += h;   f(x,f11);
      x(j) = xj;   f(x,f10);
      x(i) = xi; 

      for(size_t k=0;k<H.size();k++) 
        H[k](i,j) = H[k](j,i) = (f11[k]-f10[k]-f01[k]+f00[k])*scale;
    }
  }
}

void GradientCenteredDifference(ScalarFieldFunction& f,Vector& x,Real h,Vector& g)
{
  if(g.empty()) g.resize(x.n);
  Assert(g.n == x.n);
  Real twoh = h+h;
  Real f0,f1;
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    x(i) += h;   f1=f(x);
    x(i) -= twoh;  f0=f(x);
    x(i) = xi;

    g(i) = (f1-f0);
  }
  Div(g,twoh);
}

void GradientCenteredDifferenceAdaptive(ScalarFieldFunction& f,Vector& x,Real h0,Real tol,Vector& g)
{
  Assert(g.n == x.n);
  for(int i=0;i<x.n;i++) {
    ScalarFieldProjectionFunction fi(f,x,i);
    g(i) = dfCenteredDifferenceAdaptive(fi,0,h0,tol);
  }
}


void JacobianCenteredDifference(VectorFieldFunction& f,Vector& x,Real h,Matrix& J)
{
  int nd = f.NumDimensions();
  Vector f0(nd),f1(nd);
  if(J.isEmpty()) J.resize(f.NumDimensions(),x.n);
  Assert(J.m == f.NumDimensions() && J.n == x.n);
  Real twoh = h+h;
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    x(i) += h;  f(x,f1);
    x(i) -= twoh;  f(x,f0);
    x(i) = xi;

    Vector Ji; J.getColRef(i,Ji);
    Ji.sub(f1,f0);
  }
  Div(J,twoh);
}

void HessianCenteredDifference(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H)
{
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  Real f0,f_1,f_2,f1,f2;
  Real scale=Sqr(Inv(h))*Sqr(Half);
  Real iiscale=Sqr(Inv(h))/12;
  f0 = f(x);
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    //get diagonal entry
    x(i)+=h;   f1 = f(x); 
    x(i)+=h;   f2 = f(x); 
    x(i)=xi-h; f_1 = f(x); 
    x(i)-=h;   f_2 = f(x); 
    x(i) = xi;
    H(i,i) = (Real)(-f2+16.0*f1-30.0*f0+16.0*f_1-f_2)*iiscale;
    /*  This is a test of this 4th order approx vs the std 2nd order
    Real temp = (f1 - Two*f0 + f_1)/Sqr(h);
    LOG4CXX_INFO(KrisLibrary::logger()(),"f's: "<<f_2<<" "<<f_1<<" "<<f0<<" "<<f1<<" "<<f2<<" => H: "<<H(i<<"... or "<<i)    if(KrisLibrary::logger()()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    */

    for(int j=i+1;j<x.n;j++) {
      Real xj = x(j);
      //get dfj,df_j
      x(j) += h;  x(i) += h;   f2 = f(x);
      x(i) -= Two*h;  f1 = f(x);

      x(j) -= Two*h;   f_2 = f(x);
      x(i) += Two*h;   f_1 = f(x);
      x(i) = xi; x(j) = xj;

      H(i,j) = H(j,i) = (f2-f1-f_1+f_2)*scale;
    }
  }
}

void HessianCenteredDifference_Grad(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H)
{
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  vector<Vector> g0(x.n);
  vector<Vector> g1(x.n);
  for(int i=0;i<x.n;i++) {
    g0[i].resize(x.n);
    g1[i].resize(x.n);
    Real xi=x(i);
    x(i)+=h;
    f.PreEval(x);
    f.Gradient(x,g1[i]);
    x(i)-=Two*h;
    f.PreEval(x);
    f.Gradient(x,g0[i]);
    x(i) = xi;
  }
  Real scale = Inv(h)*(Real)0.25;
  for(int i=0;i<x.n;i++) {
    for(int j=i;j<x.n;j++) {
      H(i,j) = H(j,i) = (g1[j](i) - g0[j](i) + g1[i](j)-g0[i](j))*scale;
    }
  }
}

void HessianCenteredDifference(VectorFieldFunction& f,Vector& x,Real h,vector<Matrix>& H)
{
  Assert((int)H.size() == f.NumDimensions());
  int n=f.NumDimensions();
  for(size_t i=0;i<H.size();i++)
    Assert(H[i].hasDims(x.n,x.n));
  Vector f0(n),f_1(n),f_2(n),f1(n),f2(n);
  Real scale=Sqr(Inv(h))*Sqr(Half);
  Real iiscale=Sqr(Inv(h))/12;
  f(x,f0);
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    //get diagonal entry
    x(i)+=h;   f(x,f1); 
    x(i)+=h;   f(x,f2); 
    x(i)=xi-h; f(x,f_1); 
    x(i)-=h;   f(x,f_2); 
    x(i) = xi;
    for(int k=0;k<n;k++)
      H[k](i,i) = (Real)(-f2[k]+16.0*f1[k]-30.0*f0[k]+16.0*f_1[k]-f_2[k])*iiscale;
    /*  This is a test of this 4th order approx vs the std 2nd order
    Real temp = (f1 - Two*f0 + f_1)/Sqr(h);
    LOG4CXX_INFO(KrisLibrary::logger()(),"f's: "<<f_2<<" "<<f_1<<" "<<f0<<" "<<f1<<" "<<f2<<" => H: "<<H(i<<"... or "<<i)    if(KrisLibrary::logger()()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    */

    for(int j=i+1;j<x.n;j++) {
      Real xj = x(j);
      //get dfj,df_j
      x(j) += h;  x(i) += h;   f(x,f2);
      x(i) -= Two*h;  f(x,f1);

      x(j) -= Two*h;   f(x,f_2);
      x(i) += Two*h;   f(x,f_1);
      x(i) = xi; x(j) = xj;

      for(int k=0;k<n;k++)
        H[k](i,j) = H[k](j,i) = (f2[k]-f1[k]-f_1[k]+f_2[k])*scale;
    }
  }
}




void GradientForwardDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Vector& g)
{
  Assert(g.n == x.n);
  Assert(h.n == x.n);
  Real f0 = f(x),f1;
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    x(i) += h(i); f1=f(x);
    x(i) = xi;

    g(i) = (f1-f0)/h(i);
  }
}

void JacobianForwardDifference(VectorFieldFunction& f,Vector& x,const Vector& h,Matrix& J)
{
  Assert(h.n == x.n);
  int nd = f.NumDimensions();
  Vector f0(nd),f1(nd);
  if(J.isEmpty()) J.resize(f.NumDimensions(),x.n);
  Assert(J.m == f.NumDimensions() && J.n == x.n);
  f(x,f0);
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    x(i) += h(i);  f(x,f1);
    x(i) = xi;

    Vector Ji; J.getColRef(i,Ji);
    Ji.sub(f1,f0);
    Div(Ji,h(i));
  }
}

void HessianForwardDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H)
{
  Assert(h.n == x.n);
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  Real f00,f01,f10,f11;
  f00 = f(x);
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    for(int j=i;j<x.n;j++) {
      Real xj=x(j);
      x(j) += h(j);   f01 = f(x);
      x(i) += h(i);   f11 = f(x);
      x(j) = xj;      f10 = f(x);
      x(i) = xi;

      H(i,j) = H(j,i) = (f11-f10-f01+f00)/(h(i)*h(j));
    }
  }
}

void HessianForwardDifference_Grad(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H)
{
  Assert(h.n == x.n);
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  Vector g0(x.n);
  vector<Vector> g(x.n);
  f.PreEval(x);
  f.Gradient(x,g0);
  for(int i=0;i<x.n;i++) {
    g[i].resize(x.n);
    Real xi=x(i);
    x(i)+=h(i);
    f.PreEval(x);
    f.Gradient(x,g[i]);
    x(i) = xi;
  }
  for(int i=0;i<x.n;i++) {
    for(int j=i;j<x.n;j++) {
      H(i,j) = H(j,i) = Half*((g[j](i) - g0(i))/h(j) + (g[i](j)-g0(j))/h(i));
    }
  }
}


void GradientCenteredDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Vector& g)
{
  Assert(h.n == x.n);
  if(g.empty()) g.resize(x.n);
  Assert(g.n == x.n);
  Real f0,f1;
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    Real hi=h(i);
    Real twoh=hi+hi;
    x(i) += hi;   f1=f(x);
    x(i) -= twoh; f0=f(x);
    x(i) = xi;

    g(i) = (f1-f0)/twoh;
  }
}

void JacobianCenteredDifference(VectorFieldFunction& f,Vector& x,const Vector& h,Matrix& J)
{
  Assert(h.n == x.n);
  int nd = f.NumDimensions();
  Vector f0(nd),f1(nd);
  if(J.isEmpty()) J.resize(f.NumDimensions(),x.n);
  Assert(J.m == f.NumDimensions() && J.n == x.n);
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    Real hi=h(i);
    Real twoh=hi+hi;
    x(i) += hi;   f(x,f1);
    x(i) -= twoh; f(x,f0);
    x(i) = xi;

    Vector Ji; J.getColRef(i,Ji);
    Ji.sub(f1,f0);
    Div(Ji,twoh);
  }
}

void HessianCenteredDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H)
{
  Assert(h.n == x.n);
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  Real f0,f_1,f_2,f1,f2;
  f0 = f(x); 
  for(int i=0;i<x.n;i++) {
    Real xi=x(i);
    Real hi=h(i);
    //get diagonal entry
    x(i)+=hi;   f1 = f(x); 
    x(i)+=hi;   f2 = f(x); 
    x(i)=xi-hi; f_1 = f(x); 
    x(i)-=hi;   f_2 = f(x); 
    x(i)=xi;
    H(i,i) = (-f2+16*f1-30*f0+16*f_1-f_2)/(12*Sqr(hi));

    for(int j=i+1;j<x.n;j++) {
      Real xj = x(j);
      Real hj = h(j);
      //get dfj,df_j
      x(j) += hj;  x(i) += hi;   f2 = f(x);
      x(i) -= Two*hi;  f1 = f(x);

      x(j) -= Two*hj;   f_2 = f(x);
      x(i) += Two*hi;   f_1 = f(x);
      x(i) = xi; x(j) = xj;

      H(i,j) = H(j,i) = (f2-f1-f_1+f_2)/(hi*hj)*(Real)0.25;
    }
  }
}

void HessianCenteredDifference_Grad(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H)
{
  Assert(h.n == x.n);
  if(H.isEmpty()) H.resize(x.n,x.n);
  Assert(H.hasDims(x.n,x.n));
  vector<Vector> g0(x.n);
  vector<Vector> g1(x.n);
  for(int i=0;i<x.n;i++) {
    g0[i].resize(x.n);
    g1[i].resize(x.n);
    Real xi=x(i);
    x(i)+=h(i);
    f.PreEval(x);
    f.Gradient(x,g1[i]);
    x(i)-=Two*h(i);
    f.PreEval(x);
    f.Gradient(x,g0[i]);
    x(i)=xi;
  }
  for(int i=0;i<x.n;i++) {
    for(int j=i;j<x.n;j++) {
      H(i,j) = H(j,i) = ((g1[j](i) - g0[j](i))/h(j) + (g1[i](j)-g0[i](j))/h(i))*(Real)0.25;
    }
  }
}


} //namespace math
