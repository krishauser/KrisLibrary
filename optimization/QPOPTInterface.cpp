#include "QPOPTInterface.h"
#include <errors.h>
#include <iostream>
using namespace Optimization;
using namespace std;

#if HAVE_QPOPT

extern "C" {

#include <f2c.h>
#include <qpopt.h> 

} //extern "C"

LinearProgram::Result QPOPTInterface::Solve(const QuadraticProgram& qp,Results& res)
{
  //assert qp matrices are in block form
  Assert(qp.Pobj.isRowMajor() && qp.Pobj.jstride == 1);
  Matrix At;
  if(qp.A.isRowMajor()) {
    At.setTranspose(qp.A);
  }
  else {
    At.setRefTranspose(qp.A);
  }
  Assert(qp.l.stride == 1);
  Assert(qp.u.stride == 1);
  Assert(qp.q.stride == 1);
  Assert(qp.p.stride == 1);
  integer n=qp.Pobj.m;
  integer nc=qp.A.m;
  Assert(At.istride >= nc);
  Assert(qp.Pobj.istride >= n);
  integer Acolinc = At.istride;
  integer Hcolinc = qp.Pobj.istride;
  integer inform=0;
  integer iters=0;
  double obj=0;
  integer leniw=2*n+3;
  integer lenrw=2*n*n+8*n+5*nc;
  Vector lowerBound(n+nc),upperBound(n+nc);
  //first n constraints are variable bounds, nc are constraint bounds
  Real infTol = 1e30;
  for(int i=0;i<n;i++) {
    lowerBound(i) = qp.l(i);
    upperBound(i) = qp.u(i);
  }
  for(int i=0;i<nc;i++) {
    lowerBound(i+n) = qp.q(i);
    upperBound(i+n) = qp.p(i);
  }
  for(int i=0;i<n+nc;i++) {
    if(lowerBound(i) < -infTol) lowerBound(i) = -infTol;
    if(upperBound(i) > infTol) upperBound(i) = infTol;
  }
  if(res.x.n != n)  //don't change x's values if they are already given
    res.x.resize(n,Zero);
  res.Ax.resize(n+nc);
  res.clambda.resize(n+nc);
  res.istate.resize(n+nc);
  res.iworkspace.resize(leniw);
  res.rworkspace.resize(lenrw);
  qpopt_(&n,&nc,&Acolinc,&Hcolinc,
	 At.getStart(),
	 lowerBound.getStart(),upperBound.getStart(),qp.qobj.getStart(),
	 qp.Pobj.getStart(),(S_fp)qphess_,
	 &res.istate[0],res.x.getStart(),&inform,&iters,&obj,
	 res.Ax.getStart(),res.clambda.getStart(),
	 &res.iworkspace[0],&leniw,
	 &res.rworkspace[0],&lenrw);
  switch(inform) {
  case 0: return LinearProgram::Feasible;
  case 1: return LinearProgram::Feasible; //TODO: suboptimal
  case 2: return LinearProgram::Unbounded;
  case 3: return LinearProgram::Infeasible;
  case 4: return LinearProgram::Error;
  default: return LinearProgram::Error;
  }
}

void QPOPTInterface::SetVerbose(int verbose)
{
  char* param="Print level";
  ftnlen plen = strlen(param);
  integer value = 0;
  switch(verbose) {
  case 0: value=0; break;
  case 1: value=5; break;
  case 2: value=10; break;
  case 3: value=20; break;
  default: value=30; break;
  }
  qpprmi_(param,&value,plen);
}

void QPOPTInterface::SetWarmStart(bool warmStart)
{
  if(warmStart) {
    char* param="Warm start";
    ftnlen plen = strlen(param);
    qpprm_(param,plen);
  }
  else {
    char* param="Cold start";
    ftnlen plen = strlen(param);
    qpprm_(param,plen);
  }
}

#else

LinearProgram::Result QPOPTInterface::Solve(const QuadraticProgram& qp,Results& res)
{
  cerr<<"QPOPT not defined"<<endl;
  return LinearProgram::Error;
}

void QPOPTInterface::SetVerbose(int verbose)
{
  cerr<<"QPOPT not defined"<<endl;
}

void QPOPTInterface::SetWarmStart(bool warmStart)
{
  cerr<<"QPOPT not defined"<<endl;
}

#endif //HAVE_QPOPT
