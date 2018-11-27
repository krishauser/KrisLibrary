#include <KrisLibrary/Logger.h>
#include "LeastSquares.h"
#include "QuadraticProgram.h"
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
#include <math/MatrixEquationPrinter.h>
#include <math/linalgebra.h>
#include <errors.h>
#include <iostream>
using namespace Optimization;
using namespace std;

LeastSquares::LeastSquares()
  :initialPoint(NULL),verbose(0)
{}

bool LeastSquares::Solve(Vector& x) const
{
  Assert(A.m == b.n);
  Assert(Aeq.m == beq.n);
  Assert(Aineq.m == bineq.n);
  Assert(Aeq.n==0 || Aeq.n==A.n);
  Assert(Aineq.n==0 || Aineq.n==A.n);

  if(Aeq.m != 0 && Aineq.m != 0) {
    //QP_InteriorPoint qp;
    QuadraticProgram qp;
    qp.Pobj.mulTransposeA(A,A);
    A.mulTranspose(b,qp.qobj); qp.qobj.inplaceNegative();

    qp.SetSimpleForm(Aeq,beq,Aineq,bineq);

    FatalError("TODO: solve quadratic program");
  }

  if(Aeq.m != 0) {  //must transform the problem to an equivalent one
    //letting C=Aeq, d=beq
    //Cx=d => x = C#*d + N*y = x0 + N*y
    //where C# is pseudoinverse, N is nullspace
    //so lsq problem becomes min |A*N*y - (b-A*x0)|
    Assert(Aeq.m <= Aeq.n);  //otherwise overconstrained

    Matrix A_new, N;
    Vector b_new, x0, y;

    MatrixEquation eq(Aeq,beq);
    if(!eq.AllSolutions(x0,N)) {
      if(verbose >= 1)
	LOG4CXX_ERROR(KrisLibrary::logger(),"LeastSquares: Error solving for all solutions to equality constraints");
      if(verbose >= 2) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Press any key to continue");
	KrisLibrary::loggerWait();
      }
      return false;
    }
    if(verbose >= 2) {
      Vector r;
      eq.Residual(x0,r);
      if(r.norm() > 1e-4) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Residual of Aeq*x0=beq: "<<VectorPrinter(r));
	LOG4CXX_INFO(KrisLibrary::logger(),"Norm is "<<r.norm());
	if(r.norm() > 1e-2) {
	  LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(Aeq));
	  LOG4CXX_INFO(KrisLibrary::logger(),"Press any key to continue");
	  KrisLibrary::loggerWait();
	  return false;
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
    A.mul(x0,b_new); b_new-=b; b_new.inplaceNegative();

    //set Anew
    A_new.mul(A,N);

    if(verbose >= 2) {
      LOG4CXX_INFO(KrisLibrary::logger(),"x0: "<<VectorPrinter(x0));
      LOG4CXX_INFO(KrisLibrary::logger(),"N: "<<MatrixPrinter(N)<<"\n");
    }
    if(verbose >=1) LOG4CXX_INFO(KrisLibrary::logger(),"Solving transformed problem...");
      {
      MatrixEquation ls(A_new,b_new);
      if(!ls.LeastSquares(y)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"LeastSquares: Error solving transformed least squares!!!");
	if(verbose >=1) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Press any key to continue");
	  KrisLibrary::loggerWait();
	}
	return false;
      }
    }
    x=x0;
    N.madd(y,x);
    if(verbose >= 1) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Result of transformed problem: "<<VectorPrinter(y));
      LOG4CXX_INFO(KrisLibrary::logger(),"   in original space: "<<VectorPrinter(x));
    }
  }
  else {
    if(Aineq.m == 0) {  //can just do regular least squares
      MatrixEquation ls(A,b);
      if(!ls.LeastSquares(x)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error solving for least squares!!!");
	return false;
      }
    }
    else {
      //form constrained quadratic program
      //min 0.5*x^t*A^t*A*x - A^t*b
      if(Aineq.m < Aineq.n) LOG4CXX_WARN(KrisLibrary::logger(),"Warning: may not find solution, unbounded domain");
      QuadraticProgram qp;
      qp.Pobj.mulTransposeA(A,A);
      A.mulTranspose(b,qp.qobj); qp.qobj.inplaceNegative();
      qp.SetSimpleForm(Aeq,beq,Aineq,bineq);
      Assert(qp.IsValid());

      FatalError("TODO: solve QP");
    }
  }
  if(verbose >= 1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"LeastSquares solved.");
  }
  return true;
}


bool LeastSquares::SatisfiesInequalities(const Vector& x) const
{
  for(int i=0;i<Aineq.m;i++) 
      if(Aineq.dotRow(i,x) > bineq(i)) return false;
  return true;
}

bool LeastSquares::SatisfiesEqualities(const Vector& x,Real tol) const
{
  for(int i=0;i<Aeq.m;i++)
      if(!FuzzyEquals(Aeq.dotRow(i,x),beq(i),tol)) return false;
  return true;
}

Real LeastSquares::Objective(const Vector& x) const
{
  Vector r;
  A.mul(x,r); r-=b;
  return r.normSquared();
}

void LeastSquares::Print(ostream& out) const
{
  MatrixEquationPrinter eq;
  out<<"min |A*x-b|^2, where A*x-b is "<<endl;
  eq.PushMatrix(A);
  eq.PushTimes();
  eq.PushText("x");
  eq.PushSub();
  eq.PushVector(b);
  eq.Print(out);
  if(Aeq.m != 0 || Aineq.m != 0)
    out<<"Subject to"<<endl;
  if(Aeq.m != 0) {
    eq.Clear();
    eq.PushMatrix(Aeq);
    eq.PushTimes();
    eq.PushText("x");
    eq.PushEquals();
    eq.PushVector(beq);
    eq.Print(out);
  }
  if(Aeq.m != 0 && Aineq.m != 0)
    out<<"   and"<<endl;
  if(Aineq.m != 0) {
    eq.Clear();
    eq.PushMatrix(Aineq);
    eq.PushTimes();
    eq.PushText("x");
    eq.PushText("<=");
    eq.PushVector(bineq);
    eq.Print(out);
  }
}

void LeastSquares::PrintStats(const Vector& x,ostream& out) const
{
  Vector r;
  A.mul(x,r); r-=b;
  if(r.normSquared() < 1e-4)
    out<<"Objective met"<<endl;
  else {
    out<<"Objective not met, residual: "<<VectorPrinter(r)<<endl;
  }

  r.resize(Aeq.m);
  Aeq.mul(x,r); r-=beq;
  for(int i=0;i<r.n;i++) {
    if(r(i) > 1e-4) 
      out<<"Equality constraint "<<i<<" not satisfied by "<<r(i)<<endl;
  }

  r.resize(Aineq.m);
  Aineq.mul(x,r); r-=bineq;
  for(int i=0;i<r.n;i++) {
    if(r(i) > 0)
      out<<"Inequality constraint "<<i<<" not satisfied by "<<r(i)<<endl;
    else if(r(i) > -1e-3) {
      out<<"Inequality constraint "<<i<<" nearly met, by "<<r(i)<<endl;
    }
  }
}
