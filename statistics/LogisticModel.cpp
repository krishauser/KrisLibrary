#include <KrisLibrary/Logger.h>
#include "LogisticModel.h"
#include "DistributionCollector.h"
#include <optimization/Minimization.h>
#include <math/gaussian.h>
#include <math/differentiation.h>
#include <math/LDL.h>
#include <math/SVDecomposition.h>
#include <math/SelfTest.h>
#include <math/random.h>
#include <Timer.h>
#include <utils/StatCollector.h>
#include <iostream>
using namespace Statistics;
using namespace std;

//e^10 is ~2x10^4
//e^15 is ~3x10^6
//e^18 is ~5x10^8
const static Real kMaxExponent = 15.0;
const static Real kLogOnePlusExpMax = 8.0;

//evaluates x^T*A*y
inline Real EvalQuadraticForm(const Vector& x,const Matrix& A,const Vector& y)
{
  Real sum=0;
  for(int j=0;j<A.n;j++)
    sum += y(j) * A.dotCol(j,x);
  return sum;
}

inline Real Logit(Real v)
{
  if(v > kMaxExponent) return 1;
  else if(v < -kMaxExponent) return 0;
  Real ev=Exp(v);
  return ev/(One+ev);
}

struct LogitLikelihoodFunction : public ScalarFieldFunction
{
  LogitLikelihoodFunction(const vector<Vector>& _x,const vector<bool>& _res)
    :x(_x),res(_res)
  {}

  virtual Real Eval(const Vector& params) {
    Real sumLogLikelihood=0;
    for(size_t i=0;i<x.size();i++) {
      Assert(x[i].n == params.n);
      Real v = dot(x[i],params);
      //p = e^v/(1+e^v)
      //log p = v - log(1+e^v)
      //log 1-p = -log(1+e^v)
      Real log1pex; //log(1+e^v)
      if(v > kLogOnePlusExpMax) log1pex = v;  //almost nearly the identity
      else log1pex = Log(1+Exp(v));

      Real logLikelihood;
      if(res[i]) logLikelihood = v - log1pex;
      else logLikelihood = -log1pex;
      sumLogLikelihood += logLikelihood;
    }
    return -sumLogLikelihood;
  }

  virtual void Gradient(const Vector& params,Vector& grad) {
    grad.resize(params.n);
    grad.set(Zero);
    for(size_t i=0;i<x.size();i++) {
      Assert(x[i].n == params.n);
      Real v = dot(x[i],params);
      const Vector& dv = x[i];

      Real p=Logit(v);

      //p = e^v/(1+e^v)
      //log p = v - log(1+e^v)
      //log 1-p = -log(1+e^v)
      //d log p / dtheta = dv/dtheta - dv/dtheta*e^v/(1+e^v)
      // = dv/dtheta * 1/(1+e^v) = dv/dtheta*(1-p)
      //d log p / dtheta = 1/p * dp/dtheta = dv/dtheta*p*(1-p)/p
      //d log 1-p /dtheta = -dv/dtheta*e^v/(1+e^v) = -dv/dtheta*p
      //note that the coefficient is negative because of the negative loglikelihood
      if(res[i])
	grad.madd(dv,p-1.0);
      else
	grad.madd(dv,p);
    }
  }

  const vector<Vector>& x;
  const vector<bool>& res;
};

Real LogisticModel::Evaluate(const Vector& x) const
{
  Assert(x.n == coeffs.n);
  Real v = x.dot(coeffs);  //v is the odds ratio
  return Logit(v);
}

Real LogisticModel::EvaluateExpectation(const Vector& x,int numSamples) const
{
  Assert(x.n == coeffs.n);
  Real variance = EvalQuadraticForm(x,covariance,x);
  Real mean = x.dot(coeffs);
  Real stddev = Sqrt(variance);
  DistributionCollector dist;
  for(int sample=0;sample<numSamples;sample++) {
    Real v = mean + RandGaussian()*stddev;
    dist << Logit(v);
  }
  return dist.mean();
}

Real LogisticModel::EvaluateVariance(const Vector& x,int numSamples) const
{
  Assert(x.n == coeffs.n);
  Real variance = EvalQuadraticForm(x,covariance,x);
  Real mean = x.dot(coeffs);
  Real stddev = Sqrt(variance);
  DistributionCollector dist;
  for(int sample=0;sample<numSamples;sample++) {
    Real v = mean + RandGaussian()*stddev;
    dist << Logit(v);
  }
  return dist.variance();
}

void LogisticModel::MaximumLikelihood(const vector<Vector>& x,const vector<bool>& res,int numIters,Real tol)
{
  //likelihood of a point with v[i]=1 is p(x)
  //                           v[i]=0 is 1-p(x)
  //where p(x) = v/(1+v) with v=coeffs.x+offset

  Assert(!x.empty());
  int n=x[0].n;
  int constantTerm=n-1;
  for(size_t i=0;i<x.size();i++) {
    Assert(x[i].n == n);
    Assert(x[i](constantTerm) == 1.0);
  }

  Assert(x.size() == res.size());
  size_t numTrue = 0;
  for(size_t i=0;i<res.size();i++) if(res[i]) numTrue++;
  if(numTrue == 0) {
    coeffs.resize(n,Zero);
    coeffs(constantTerm) = -Inf;
    covariance.resize(n,n,Zero);
    return;
  }
  else if(numTrue==res.size()) {
    coeffs.resize(n,Zero);
    coeffs(constantTerm) = Inf;
    covariance.resize(n,n,Zero);
    return;
  }
  //p = e^c/(1+e^c) => p+e^c(p-1) = 0 => c = log(p/(1-p))
  Real p = Real(numTrue)/Real(x.size());

  LogitLikelihoodFunction f(x,res);
  Optimization::MinimizationProblem opt(&f);
  opt.x.resize(n);
  opt.x.setZero();
  opt.x(n) = Log(p)-Log(1-p);
  opt.tolx = tol;
  opt.tolf = tol;
  opt.tolgrad = tol;

  LOG4CXX_INFO(KrisLibrary::logger(),"Testing logit gradient...\n");
  Real atol=1e-2;
  Real rtol=1e-2;
  bool test=TestGradient(&f,opt.x,1e-3,atol,rtol);
  if(!test) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Testing failed!\n");
    Abort();
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Done.\n");
  }
  
  ConvergenceResult r=opt.SolveQuasiNewton_Ident(numIters);
  Assert(r == ConvergenceX || r == ConvergenceF || r == MaxItersReached);
  if(r == ConvergenceX || r == ConvergenceF){
    LOG4CXX_INFO(KrisLibrary::logger(),"Quasi-Newton method converged in "<<numIters<<" iterations");
  }
  else{
    LOG4CXX_INFO(KrisLibrary::logger(),"Maximum iterations reached");
  }

  coeffs.clear();
  coeffs = opt.x;

  Matrix informationMatrix(n,n,Zero);
  Vector dlog(opt.x.n);
  for(size_t i=0;i<x.size();i++) {
    Real p=Evaluate(x[i]);
    const Vector& dv = x[i];

    //p = e^v/(1+e^v)
    //dp/dtheta = d(e^v)/(1+e^v) - d(1+e^v)e^v/(1+e^v)^2
    // = x*p - x*e^v^2/(1+e^v)^2
    // = x*p(1-p)
    //1-p = 1/(1+e^v)
    //d(1-p)/dtheta = -x*e^v/(1+e^v)^2=-x*p(1-p)
    //dlog(p)/dtheta = 1/p*dp/dtheta = x*(1-p)
    //dlog(1-p)/dtheta = 1/(1-p)*d(1-p)/dtheta = x*p
   
    //o.p. gradient if x[i]=true + o.p gradient if x[i]=false
    for(int i=0;i<dlog.n;i++)
      for(int j=0;j<dlog.n;j++)
	informationMatrix(i,j) += dv(i)*dv(j)*p*(1.0-p);
  }
  //LOG4CXX_INFO(KrisLibrary::logger(),"InformationMatrix"<<MatrixPrinter(informationMatrix,MatrixPrinter::AsciiShade)<<"\n");

  LDLDecomposition<Real> ldl;
  ldl.set(informationMatrix);
  bool singular=false;
  for(int i=0;i<ldl.LDL.n;i++)
    if(FuzzyZero(ldl.LDL(i,i))) {
      singular=true;
      break;
    }
  if(singular) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Information matrix is singular, using SVD...");
    RobustSVD<Real> svd;
    if(!svd.set(informationMatrix)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error inverting information matrix...");
      ldl.getInverse(covariance);
    }
    else {
      //HACK
      svd.svd.U.copy(svd.svd.V);
      svd.getInverse(covariance);
    }
  }
  else ldl.getInverse(covariance);
  for(int i=0;i<covariance.m;i++) {
    if(covariance(i,i) < 0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, covariance matrix has negative on diagonal!");
      LOG4CXX_ERROR(KrisLibrary::logger(),covariance(i,i));
    }
    if(!IsFinite(covariance(i,i))) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, covariance matrix has inf/nan on diagonal!");
      LOG4CXX_ERROR(KrisLibrary::logger(),covariance(i,i));
    }
  }

  LOG4CXX_INFO(KrisLibrary::logger(),"Resulting log-likelihood: "<<-f(opt.x));

  DistributionCollector error;
  for(size_t i=0;i<x.size();i++) {
    if(res[i]) error << 1-Evaluate(x[i]);
    else error << Evaluate(x[i]);
  }
  LOG4CXX_ERROR(KrisLibrary::logger(),"Actual error min "<<error.xmin<<", max "<<error.xmax<<", mean "<<error.mean()<<" mse "<<error.sumsquared/error.n);
  /*
  LOG4CXX_INFO(KrisLibrary::logger(),"Results: ");
  for(size_t i=0;i<x.size();i++) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Actual value "<<res[i]<<", evaluated "<<Evaluate(x[i]));
  }
  */
}

namespace Statistics { 

ostream& operator << (ostream& out,const LogisticModel& model)
{
  out<<model.coeffs;
  return out;
}

istream& operator >> (istream& in,LogisticModel& model)
{
  in>>model.coeffs;
  return in;
}

} //namespace Statistics
