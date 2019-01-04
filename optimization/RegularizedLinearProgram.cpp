#include <KrisLibrary/Logger.h>
#include "RegularizedLinearProgram.h"
#include "LPRobust.h"
#include <math/VectorPrinter.h>
#include <math/linalgebra.h>
#include <iostream>
#include <errors.h>
using namespace Optimization;
using namespace std;

RegularizedLinearProgram::RegularizedLinearProgram()
  :norm(Inf),verbose(0)
{}

bool RegularizedLinearProgram::IsValid() const
{
  if(norm != One && !IsInf(norm)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RegularizedLinearProgram::IsValid(): Invalid norm");
    return false;
  }
  if(!C.isEmpty() && C.n != c.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RegularizedLinearProgram::IsValid(): C.n != c.n");
    return false;
  }
  if(C.m != d.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RegularizedLinearProgram::IsValid(): C.m != d.n");
    return false;
  }
  if(!LinearConstraints::IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RegularizedLinearProgram::IsValid(): Constraints not valid");
    LOG4CXX_INFO(KrisLibrary::logger(),"A("<<A.m<<" x "<<A.n<<") p("<<p.n<<") q("<<q.n<<") l("<<l.n<<") u("<<u.n);
    return false;
  }
  if(!A.isEmpty() && c.n != A.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RegularizedLinearProgram::IsValid(): Constraint size does not match objective size");
    return false;
  }
  return true;
}

void RegularizedLinearProgram::Print(std::ostream& out) const
{
  out<<"TODO: print objective vector"<<endl;
  out<<"min c.x + L"<<norm<<" norm of: "<<endl;
  for(int i=0;i<C.m;i++) {
    Vector ci;
    C.getRowRef(i,ci);
    out<<"["<<VectorPrinter(ci)<<"].x - "<<d(i)<<endl;
  }
  out<<"w.r.t. x";
  if(!A.isEmpty()) {
    out<<" such that "<<endl;
    LinearConstraints::Print(out);
  }
}

void RegularizedLinearProgram::Assemble()
{
  if(C.isEmpty()) {
    lp.c.setRef(c);
    lp.minimize = true;
    lp.A.setRef(A);
    lp.l.setRef(l);
    lp.u.setRef(u);
    lp.p.setRef(p);
    lp.q.setRef(q);
    return;
  }
  //in the below comments, let 1 be the vector of all 1's
  if(norm == One) {
    /* Convert to 
      min_{x,e} ctx + 1te
      b <= Cx + Ie
      Cx - Ie <= d
    */
    lp.Resize(C.m*2+A.m,C.n+C.m);
    lp.minimize = true;
    for(int i=0;i<c.n;i++) lp.c(i) = c(i);
    for(int i=0;i<C.m;i++) lp.c(C.n+i) = One;
    lp.A.copySubMatrix(0,0,C);
    for(int i=0;i<C.m;i++) lp.A(i,i+C.n) = 1;
    lp.q.copySubVector(0,d);
    lp.A.copySubMatrix(C.m,0,C);
    for(int i=0;i<C.m;i++) lp.A(i+C.m,i+C.n) = -1;
    lp.p.copySubVector(C.m,d);

    if(A.m != 0) {
      lp.A.copySubMatrix(C.m*2,0,A);
      lp.q.copySubVector(C.m*2,q);
      lp.p.copySubVector(C.m*2,p);
    }
    if(!l.empty()) 
      lp.l.copySubVector(0,l);
    if(!u.empty()) 
      lp.u.copySubVector(0,u);
  }
  else {
    Assert(IsInf(norm));
    /* Convert to 
      min_{x,d} ctx + d
      b <= Cx + 1d
      Cx - 1d <= b
    */
    lp.Resize(C.m*2+A.m,C.n+1);
    lp.minimize = true;
    for(int i=0;i<c.n;i++) lp.c(i)=c(i);
    lp.c(C.n) = 1;
    lp.A.copySubMatrix(0,0,C);
    for(int i=0;i<C.m;i++) lp.A(i,C.n) = 1;
    lp.q.copySubVector(0,d);
    lp.A.copySubMatrix(C.m,0,C);
    for(int i=0;i<C.m;i++) lp.A(C.m+i,C.n) = -1;
    lp.p.copySubVector(C.m,d);

    if(!A.isEmpty()) {
      lp.A.copySubMatrix(C.m*2,0,A);
      lp.q.copySubVector(C.m*2,q);
      lp.p.copySubVector(C.m*2,p);
    }
    if(!l.empty()) 
      lp.l.copySubVector(0,l);
    if(!u.empty()) 
      lp.u.copySubVector(0,u);
  }
}

Real RegularizedLinearProgram::Objective(const Vector& x) const
{
  return c.dot(x);
}

Real RegularizedLinearProgram::Norm(const Vector& x) const
{
  if(C.isEmpty()) return 0;
  Vector temp;
  C.mul(x,temp);
  temp -= d;
  if(norm==One) {
    Real sum=0;
    for(int i=0;i<temp.n;i++)
      sum += Abs(temp(i));
    return sum;
  }
  else {
    Assert(IsInf(norm));
    return temp.maxAbsElement();
  }
}

Real RegularizedLinearProgram::ObjectiveNormSum(const Vector& x) const
{
  return Objective(x)+Norm(x);
}

LinearProgram::Result RegularizedLinearProgram::Solve(Vector& x) 
{
  RobustLPSolver lps;
  lps.verbose = verbose;
  lps.UpdateGLPK(lp);
  LinearProgram::Result res= lps.SolveGLPK();
  if(res==LinearProgram::Feasible) {
    x.resize(C.n);
    lps.xopt.getSubVectorCopy(0,x);
  }
  return res;
}
  





