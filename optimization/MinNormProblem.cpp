#include <KrisLibrary/Logger.h>
#include "MinNormProblem.h"
#include "LPRobust.h"
#include "BoundedLSQRSolver.h"
#include "LSQRInterface.h"
#include <math/linalgebra.h>
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
#include <iostream>
#include <errors.h>
using namespace Optimization;
using namespace std;

MinNormProblem::MinNormProblem()
  :norm(Inf),verbose(0)
{}

void MinNormProblem::AddVariables(int num)
{
  LinearConstraints::AddVariables(num);
  C.resizePersist(C.m,C.n+num,Zero);
}

void MinNormProblem::AddVariable(Real lj,Real uj)
{
  LinearConstraints::AddVariable(lj,uj);
  C.resizePersist(C.m,C.n+1,Zero);
}

bool MinNormProblem::IsValid() const
{
  if(norm != One && norm != Two && !IsInf(norm)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem::IsValid(): Invalid norm");
    return false;
  }
  if(C.isEmpty()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem::IsValid(): Empty problem");
    return false;
  }
  if(C.m != d.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem::IsValid(): C.m != d.n");
    return false;
  }
  if(!LinearConstraints::IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem::IsValid(): Constraints not valid");
    LOG4CXX_INFO(KrisLibrary::logger(),"A("<<A.m<<" x "<<A.n<<") p("<<p.n<<") q("<<q.n<<") l("<<l.n<<") u("<<u.n);
    return false;
  }
  if(!A.isEmpty() && C.n != A.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem::IsValid(): Constraint size does not match objective size");
    return false;
  }
  return true;
}

void MinNormProblem::Print(std::ostream& out) const
{
  out<<"min L"<<norm<<" norm of: "<<endl;
  for(int i=0;i<C.m;i++) {
    Vector ci;
    C.getRowRef(i,ci);
    out<<"["<<VectorPrinter(ci)<<"].x - "<<d(i)<<endl;
    if(i % 10 == 9 && (&out==&cout || &out==&cerr)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Press Enter to continue...");
      KrisLibrary::loggerWait();
    }
  }
  out<<"w.r.t. x";
  if(!A.isEmpty()) {
    out<<" such that "<<endl;
    LinearConstraints::Print(out);
  }
}

void MinNormProblem::Assemble()
{
  //in the below comments, let 1 be the vector of all 1's
  if(norm == One) {
    /* Convert to 
      min_{x,e} 1te
      b <= Cx + Ie
      Cx - Ie <= d
    */
    lp.Resize(C.m*2+A.m,C.n+C.m);
    lp.minimize = true;
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
  else if(norm == Two) {
    if(HasInequalities()) {
      qp.Pobj.mulTransposeA(C,C);
      C.mulTranspose(d,qp.qobj); qp.qobj.inplaceNegative();
      
      qp.SetRef(*this);  //sets up the linear constraints
    }
    else {  //no need to use quadratic program if no inequality constraints
    }
  }
  else {
    Assert(IsInf(norm));
    /* Convert to 
      min_{x,d} d
      b <= Cx + 1d
      Cx - 1d <= b
    */
    lp.Resize(C.m*2+A.m,C.n+1);
    lp.minimize = true;
    lp.c.setZero();
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

Real MinNormProblem::Norm(const Vector& x) const
{
  Vector temp;
  C.mul(x,temp);
  temp -= d;
  if(norm==One) {
    Real sum=0;
    for(int i=0;i<temp.n;i++)
      sum += Abs(temp(i));
    return sum;
  }
  else if(norm==Two) {
    return temp.norm();
  }
  else {
    Assert(IsInf(norm));
    return temp.maxAbsElement();
  }
}

LinearProgram::Result MinNormProblem::Solve(Vector& x) 
{
  if(norm == 2) {
    Assert(IsValid());

    if(HasInequalities()) {  //must use quadratic program for inequalities
      Assert(qp.Pobj.m == C.n);
      Assert(qp.A.isRef());
      /*
      QPActiveSetSolver solver(qp);
      solver.verbose = verbose;
      ConvergenceResult res=solver.Solve();
      if(res == ConvergenceError) {
	if(verbose >= 1)
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Quadratic program unable to solve constrained least-squares problem");
	KrisLibrary::loggerWait();
	return LinearProgram::Infeasible;
      }
      else if(res == MaxItersReached) {
	x = solver.x;
	return LinearProgram::Error;
      }
      else {
	x = solver.x;
	return LinearProgram::Feasible;
      }
      */
      FatalError("TODO: QP Solve");
    }
    else if(HasBounds()) {
      if(A.m != 0) {
	//Equalities done yet
	FatalError("Equalities and bounds not done yet");
	return LinearProgram::Error;
      }
      else {
	//solve a bounded least squares problem
	BoundedLSQRSolver lsqr(C,d,l,u);
	LinearProgram::Result res=lsqr.Solve(x);
	return res;
      }
    }
    else if(A.m != 0) {  //no inequality constraints, just equality
      //just transform the problem to an equivalent one
      //Ax=p => x = A#*p + N*y = x0 + N*y
      //where A# is pseudoinverse, N is nullspace
      //so lsq problem becomes min |C*N*y - (d-C*x0)|
      Assert(A.m <= A.n);  //otherwise overconstrained
      
      Matrix C_new, N;
      Vector d_new, x0, y;
      
      MatrixEquation eq(A,p);
      if(!eq.AllSolutions(x0,N)) {
	if(verbose >= 1)
	  LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem (norm 2): Error solving for all solutions to equality constraints");
	if(verbose >= 2) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Press any key to continue");
	  KrisLibrary::loggerWait();
	}
	return LinearProgram::Error;
      }
      if(verbose >= 2) {
	Vector r;
	eq.Residual(x0,r);
	if(r.norm() > 1e-4) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Residual of Aeq*x0=beq: "<<VectorPrinter(r));
	  LOG4CXX_INFO(KrisLibrary::logger(),"Norm is "<<r.norm());
	  if(r.norm() > 1e-2) {
	    LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(A));
	    LOG4CXX_INFO(KrisLibrary::logger(),"Press any key to continue");
	    KrisLibrary::loggerWait();
	    return LinearProgram::Error;
	  }
	  LOG4CXX_INFO(KrisLibrary::logger(),"Press any key to continue");
	  KrisLibrary::loggerWait();
	}
      }
      
      if(verbose >= 1) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Projecting problem on equality constraints");
	LOG4CXX_INFO(KrisLibrary::logger(),"Original dimension "<<A.n<<", nullspace dimension "<<N.n);
      }
      
      //set bnew
      C.mul(x0,d_new); d_new-=d; d_new.inplaceNegative();
      
      //set Cnew
      C_new.mul(C,N);
      
      if(verbose >= 2) {
	LOG4CXX_INFO(KrisLibrary::logger(),"x0: "<<VectorPrinter(x0));
	LOG4CXX_INFO(KrisLibrary::logger(),"N: "<<MatrixPrinter(N)<<"\n");
      }
      if(verbose >=1) LOG4CXX_INFO(KrisLibrary::logger(),"Solving transformed problem...");
      
      MatrixEquation ls(C_new,d_new);
      if(!ls.LeastSquares(y)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"LeastSquares: Error solving transformed least squares!!!");
	if(verbose >=1) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Press any key to continue");
	  KrisLibrary::loggerWait();
	}
	return LinearProgram::Error;
      }
      //go back to x
      x = x0;
      N.madd(y,x);
      return LinearProgram::Feasible;
    }
    else {
      MatrixEquation ls(C,d);
      if(!ls.LeastSquares(x)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error solving for least squares!!!");
	return LinearProgram::Error;
      }
      return LinearProgram::Feasible;
    }
  }
  else {
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
  LOG4CXX_INFO(KrisLibrary::logger(),"Not sure how we got here...");
  return LinearProgram::Error;
}
  





MinNormProblem_Sparse::MinNormProblem_Sparse()
  :norm(Inf),verbose(0)
{}


void MinNormProblem_Sparse::AddVariables(int num)
{
  LinearConstraints_Sparse::AddVariables(num);
  C.resize(C.m,C.n+num);
}

void MinNormProblem_Sparse::AddVariable(Real lj,Real uj)
{
  LinearConstraints_Sparse::AddVariable(lj,uj);
  C.resize(C.m,C.n+1);
}


bool MinNormProblem_Sparse::IsValid() const
{
  if(norm != One && norm != Two && !IsInf(norm)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem_Sparse::IsValid(): Invalid norm");
    return false;
  }
  if(C.isEmpty()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem_Sparse::IsValid(): Empty problem");
    return false;
  }
  if(C.m != d.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem_Sparse::IsValid(): C.m != d.n");
    return false;
  }
  if(!LinearConstraints_Sparse::IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem_Sparse::IsValid(): Constraints not valid");
    return false;
  }
  if(!A.isEmpty() && C.n != A.n) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MinNormProblem_Sparse::IsValid(): Constraint size does not match objective size");
    return false;
  }
  return true;
}

void MinNormProblem_Sparse::Print(std::ostream& out) const
{
  out<<"min L"<<norm<<" norm of: "<<endl;
  for(int i=0;i<C.m;i++) {
    SparseMatrix::ConstRowIterator j=C.rows[i].begin();
    if(j != C.rows[i].end()) {
      out<<j->second<<"*"<<"x["<<j->first<<"]";
      j++;
    }
    for(;j!=C.rows[i].end();j++) {
      if(j->second > 0)
	out<<"+"<<j->second<<"*x["<<j->first<<"]";
      else if(j->second < 0) 
	out<<"-"<<-j->second<<"*x["<<j->first<<"]";
    }
    out<<" - "<<d(i)<<endl;
  }
  out<<"w.r.t. x";
  if(!A.isEmpty()) {
    out<<" such that "<<endl;
    LinearConstraints_Sparse::Print(out);
  }
}

void MinNormProblem_Sparse::Assemble()
{
  //in the below comments, let 1 be the vector of all 1's
  if(norm == One) {
    /* Convert to 
      min_{x,e} 1te
      d <= Cx + Ie
      Cx - Ie <= d
    */
    lp.Resize(C.m*2+A.m,C.n+C.m);
    lp.minimize = true;
    for(int i=0;i<C.m;i++) lp.c(C.n+i) = One;
    lp.A.copySubMatrix(0,0,C);
    for(int i=0;i<C.m;i++) lp.A(i,i+C.n) = 1;
    lp.q.copySubVector(0,d);
    lp.A.copySubMatrix(C.m,0,C);
    for(int i=0;i<C.m;i++) lp.A(i+C.m,i+C.n) = -1;
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
  else if(norm == Two) {
    if(lp.HasInequalities()) {
      FatalError("Not done with sparse QP");
    }
    else {  //no need to use quadratic program if no inequality constraints
    }
  }
  else {
    Assert(IsInf(norm));
    //|Ci^Tx - di| <= e
    //-1e <= Ci^Tx - di <= 1e
    /* Convert to 
      min_{x,e} e
      d <= Cx + 1e
      Cx - 1e <= d
    */
    lp.Resize(C.m*2+A.m,C.n+1);
    lp.minimize = true;
    lp.c.setZero();
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

Real MinNormProblem_Sparse::Norm(const Vector& x) const
{
  Vector temp;
  C.mul(x,temp);
  temp -= d;
  if(norm==One) {
    Real sum=0;
    for(int i=0;i<temp.n;i++)
      sum += Abs(temp(i));
    return sum;
  }
  else if(norm==Two) {
    return temp.norm();
  }
  else {
    Assert(IsInf(norm));
    return temp.maxAbsElement();
  }
}

LinearProgram::Result MinNormProblem_Sparse::Solve(Vector& x) 
{
  if(norm == 2) {
    Assert(IsValid());

    if(HasInequalities()) {  //must use quadratic program for inequalities
      FatalError("Sparse QP not done yet");
      return LinearProgram::Error;
    }
    else if(A.m != 0) {  //no inequality constraints, just equality
      FatalError("Sparse LS with equality constraints not done yet");
      return LinearProgram::Error;
    }
    else {
      LSQRInterface lsqr;
      if(!lsqr.Solve(C,d)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error solving for least squares!!!");
	return LinearProgram::Error;
      }
      x = lsqr.x;
      return LinearProgram::Feasible;
    }
  }
  else {
    RobustLPSolver lps;
    lps.verbose = verbose;
    LinearProgram::Result res= lps.Solve(lp);
    if(res==LinearProgram::Feasible) {
      x.resize(C.n);
      lps.xopt.getSubVectorCopy(0,x);
    }
    return res;
  }
}
  
