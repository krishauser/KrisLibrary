#include "OnlineOLS.h"
#include "OnlineLASSO.h"
#include "DataSet.h"
#include <spline/Polynomial.h>
#include <errors.h>
using namespace Statistics;

OnlineLeastSquares::OnlineLeastSquares()
  :numObservations(0),store(false)
{}

void OnlineLeastSquares::SetPrior(const Vector& priorcoeffs,Real strength)
{
  numObservations = 1;
  coeffs = priorcoeffs;
  Aty.mul(coeffs,strength);
  ldl.LDL.resize(coeffs.n,coeffs.n);
  ldl.LDL.setIdentity();
  for(int i=0;i<coeffs.n;i++)
    ldl.LDL(i,i) = strength;
}

void OnlineLeastSquares::SetPrior(const Vector& priorcoeffs,const Vector& strength)
{
  numObservations = 1;
  coeffs = priorcoeffs;
  Aty.resize(coeffs.n);
  for(int i=0;i<coeffs.n;i++)
    Aty(i) = coeffs(i)*strength(i);
  ldl.LDL.resize(coeffs.n,coeffs.n);
  ldl.LDL.setIdentity();
  for(int i=0;i<coeffs.n;i++)
    ldl.LDL(i,i) = strength(i);
}

bool OnlineLeastSquares::AddPoint(const Vector& _data,Real _outcome)
{
  if(coeffs.n != 0) {
    Assert(coeffs.n == _data.n);
  }
  if(!data.empty()) {
    Assert(_data.n == data[0].n);
  }
  if(store || (int)data.size() < _data.n) {
    data.push_back(_data);
    outcome.push_back(_outcome);
  }
  numObservations++;

  //update the LDL decomposition
  if(coeffs.n == 0) {
    //Assert(store != false);
    //first time
    if(Aty.n == 0) Aty.resize(_data.n,Zero);
    Aty.madd(_data,_outcome);
    if((int)data.size() >= _data.n) {  //enough points to be nondegenerate
      Matrix AtA(data[0].n,data[0].n);
      DataSet d;
      Vector xi,xj;
      d.SetObservations(data);
      for(int i=0;i<data[0].n;i++){
	d.GetElement(i,xi);
	for(int j=0;j<data[0].n;j++) {
	  d.GetElement(j,xj);
	  AtA(i,j) = xi.dot(xj);
	}
      }
      ldl.set(AtA);

      //estimate coeffs
      ldl.backSub(Aty,coeffs);
    }
  }
  else {
    ldl.update(_data);
    //check for degeneracy
    Vector d;
    ldl.getD(d);
    for(int i=0;i<d.n;i++)
      if(Abs(d(i)) < ldl.zeroTolerance) 
	return false;

    //could also compute Aty from existing coeffs
    Aty.madd(_data,_outcome);
    //compute coeffs = (AtA)^-1 Aty
    ldl.backSub(Aty,coeffs);
  }
  return true;
}

StochasticLeastSquares::StochasticLeastSquares()
  :numObservations(0),store(false)
{
  weightPolynomial.resize(2);
  weightPolynomial[0] = 1;
  weightPolynomial[1] = 0.1;
}

void StochasticLeastSquares::SetPrior(const Vector& _coeffs,int strength)
{
  coeffs = _coeffs;
  numObservations = strength;
}

void StochasticLeastSquares::AddPoint(const Vector& data,Real outcome)
{
  if(numObservations == 0) coeffs = data*outcome;
  numObservations += 1;
  Spline::Polynomial<double> P;
  P.coef = weightPolynomial;
  Real w = 1.0/P.Evaluate(numObservations);
  Real err = data.dot(coeffs) - outcome;
  Real l2 = data.dot(data);
  if(Abs(l2) > Epsilon) {
    coeffs.madd(data,-err*w/l2);
  }
  if(store) {
    this->data.push_back(data);
    this->outcome.push_back(outcome);
  }
}


StochasticPseudoLASSO::StochasticPseudoLASSO(Real _alpha)
  :alpha(_alpha),numObservations(0)
{
  weightPolynomial.resize(2);
  weightPolynomial[0] = 1;
  weightPolynomial[1] = 0.1;
}

void StochasticPseudoLASSO::SetPrior(const Vector& _coeffs,int strength)
{
  coeffs = _coeffs;
  numObservations = strength;
}

void StochasticPseudoLASSO::AddPoint(const Vector& data,Real outcome)
{
  if(numObservations==0) 
    coeffs.resize(data.size(),0.0);
  numObservations += 1;
  Real err = outcome - data.dot(coeffs);
  Real maxd = 0;
  int maxdindex = -1;
  for(int i=0;i<data.n;i++) {
    if(Abs(data(i)*err) > 0.5*alpha) {
      //candidate
      if(Abs(data(i)) > maxd) {
	maxd = Abs(data(i));
	maxdindex = i;
      }
    }
  }
  if(maxdindex < 0 || maxd < Epsilon) {
    //no need to change
    return;
  }
  Real xi = data(maxdindex);
  Real newthetai = (err*xi - alpha*0.5*Sign(err*xi))/Sqr(xi);

  Spline::Polynomial<double> P;
  P.coef = weightPolynomial;
  Real w = 1.0/P.Evaluate(numObservations);

  coeffs(maxdindex) += w * newthetai;
}

