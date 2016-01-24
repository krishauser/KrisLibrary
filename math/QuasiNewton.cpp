#include "QuasiNewton.h"
#include <errors.h>
#include <iostream>
using namespace Math;
using namespace std;

bool QNHessianUpdater::UpdateBFGS(const Vector& s,const Vector& q)
{
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
    if(verbose>=1) cout<<"Unable to maintain strict positive definiteness of hessian!"<<endl;
    //TODO: fix the downdate
    return false;
  }
  return true;
}

bool QNHessianUpdater::UpdateDFS(const Vector& s,const Vector& q)
{
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
    if(verbose>=1) cout<<"Unable to maintain strict positive definiteness of hessian!"<<endl;
    //TODO: fix the downdate
    return false;
  }
  return true;
}
