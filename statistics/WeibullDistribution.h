#include <KrisLibrary/math/specialfunctions.h>

struct WeibullDistribution : public UnivariateProbabilityDistribution
{
  WeibullDistribution(Real _a,Real _b):a(_a),b(_b) {}
  virtual void GetParameters(Vector& parameters) {
    parameters.resize(2); 
    parameters(0)=a;
    parameters(1)=b;
  }
  virtual void SetParameters(const Vector& parameters) {
    Assert(parameters.n==2);
    a=parameters(0);
    b=parameters(1);
  }
  virtual Real PDF(Real value) {
    return (b/a)*Pow(value/a,b-1)*Exp(-Pow(value/a,b));
  }
  virtual Real CDF(Real value) {
    return 1.0 - Exp(-Pow(value/a,b));
  }
  virtual Real Minimum() { return 0; }
  virtual Real Maximum() { return Inf; }
  virtual Real Mean() { return a*Gamma(1.0+1.0/b); }
  virtual Real Median() { return a*Pow(Log2,1.0/b); }
  virtual Real Variance() {
    return Sqr(a)*Gamma(1.0+2.0/b)-Sqr(Expectation());
  }
  virtual Real Skewness() {
    Real s = StandardDeviation();
    Real mu = Mean();
    return (Gamma(1.0+3.0/b)*a*a*a - 3.0*mu*Sqr(s) - mu*mu*mu)/(s*s*s);
  }
  virtual Real Sample() {
    Real x=Rand();
    return  Pow(-Log(1.0-x)/a,1.0/b);
  }
  virtual Real CanSample() const { return true; }

  Real a,b;
};

