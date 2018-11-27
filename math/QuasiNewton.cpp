#include <KrisLibrary/Logger.h>
#include "QuasiNewton.h"
#include <errors.h>
#include <iostream>
using namespace Math;
using namespace std;

QNHessianUpdater::QNHessianUpdater()
:verbose(0),tolerance(1e-5)
{}

bool QNHessianUpdater::IsPositiveDefinite(Real tol) const
{
  Vector d;
  ldl.LDL.getDiagRef(0,d);
  return d.minElement() >= tol;
}

void QNHessianUpdater::MakePositiveDefinite(Real resetValue)
{
  Vector d;
  ldl.LDL.getDiagRef(0,d);
  if(d.minElement() <= 0) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to maintain positive definiteness of hessian!");
    for(int i=0;i<d.n;i++)
if(d(i) < tolerance) d(i) = resetValue;
    //return ConvergenceError;
  }
}

bool QNHessianUpdater::UpdateBFGS(const Vector& s,const Vector& q)
{
  tempLDL=ldl.LDL;
  
  //multiply s by H => Hs
  ldl.mulLT(s,temp);
  ldl.mulD(temp,temp);
  ldl.mulL(temp,Hs);
  
  //first part
  Real qdots = q.dot(s);
  Assert(qdots > 0);
  upd.div(q,Sqrt(qdots));
  ldl.update(upd);
  
  //second part
  Real sHs = Hs.dot(s);
  Assert(sHs > 0);
  upd.div(Hs,Sqrt(sHs));
  if(!ldl.downdate(upd)) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to maintain strict positive definiteness of hessian!");
    ldl.LDL = tempLDL;
    return false;
  }

  //maintain positive definiteness artificially
  MakePositiveDefinite();
  return true;
}

bool QNHessianUpdater::UpdateDFS(const Vector& s,const Vector& q)
{
  tempLDL=ldl.LDL;

  Vector& Hq = Hs;
  //multiply q by H => Hq
  ldl.mulLT(q,temp);
  ldl.mulD(temp,temp);
  ldl.mulL(temp,Hq);
  
  //first part
  Real sdotq = s.dot(q);
  Assert(sdotq > 0);
  upd.div(s,Sqrt(sdotq));
  ldl.update(upd);
  
  //second part
  Real qHq = Hq.dot(q);
  Assert(qHq > 0);
  upd.div(Hq,Sqrt(qHq));
  if(!ldl.downdate(upd)) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to maintain strict positive definiteness of hessian!");
    ldl.LDL = tempLDL;
    return false;
  }

  //maintain positive definiteness artificially
  Vector d;
  ldl.LDL.getDiagRef(0,d);
  if(d.minElement() <= 0) {
    if(verbose>=1) LOG4CXX_INFO(KrisLibrary::logger(),"Unable to maintain positive definiteness of hessian!");
    for(int i=0;i<d.n;i++)
if(d(i) < tolerance) d(i) = 1;
    //return ConvergenceError;
  }
  return true;
}
