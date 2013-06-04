#include "OnlineOLS.h"
#include "DataSet.h"
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
  if(numObservations != 0) {
    Assert(ldl.LDL.m == _data.n);
  }
  if(!data.empty()) {
    Assert(_data.n == data[0].n);
  }
  if(store) {
    data.push_back(_data);
    outcome.push_back(_outcome);
  }
  numObservations++;

  //update the LDL decomposition
  if(coeffs.n == 0) {
    Assert(store != false);
    //first time
    if(Aty.n == 0) Aty.resize(_data.n,Zero);
    Aty.madd(_data,_outcome);
    if((int)data.size() >= _data.n) {  //enough points to be nondegenerate
      DataSet d;
      d.SetObservations(data);
      ldl.set(d.data);

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
      if(Abs(d(i)) < ldl.zeroTolerance) return false;

    //could also compute Aty from existing coeffs
    Aty.madd(_data,_outcome);
    //compute coeffs = (AtA)^-1 Aty
    ldl.backSub(Aty,coeffs);
  }
  return true;
}

