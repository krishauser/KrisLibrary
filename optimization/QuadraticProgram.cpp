#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "QuadraticProgram.h"
#include "LinearProgram.h"
#include "LPRobust.h"
#include <math/DiagonalMatrix.h>
#include <math/SVDecomposition.h>
#include <math/Conditioner.h>
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
#include <errors.h>
#include <iostream>
using namespace Optimization;
using namespace std;


struct HConditioner : public Conditioner_SymmDiag
{
  HConditioner(Matrix& A,Vector& b)
    :Conditioner_SymmDiag(A,b,NormalizeDiagonal)
	      //:Conditioner_SymmDiag(A,b,None)
  {}
};

Real SumOfSquaredEquality(const Matrix& A,const Vector& b,const Vector& x)
{
  Real sum=0;
  for(int i=0;i<A.m;i++) {
    sum += Sqr(A.dotRow(i,x)-b(i));
  }
  return sum;
}


void QuadraticProgram::Print(ostream& out) const
{
  LOG4CXX_INFO(logger,"min 1/2 x^T A x + x^T b with A="<<"\n");
  LOG4CXX_INFO(logger,MatrixPrinter(Pobj)<<"\n");
  LOG4CXX_INFO(logger,"and b="<<VectorPrinter(qobj)<<"\n");
  LOG4CXX_INFO(logger,"s.t."<<"\n");
  LinearConstraints::Print(out);
}

void QuadraticProgram::Resize(int m,int n)
{
  Pobj.resize(n,n);
  Pobj.setZero();
  qobj.resize(n,Zero);
  LinearConstraints::Resize(m,n);
}

bool QuadraticProgram::IsValid() const
{
  // Check sizes.
  if(!Pobj.isSquare()) { LOG4CXX_ERROR(logger, "ERROR: Pobj is not square." << "\n"); return false; }
  if(Pobj.m != qobj.n) { LOG4CXX_ERROR(logger, "ERROR: Pobj and qobj must have compatible sizes." << "\n"); return false; }
  if(A.n != 0) 
    if (A.n != qobj.n) { LOG4CXX_ERROR(logger, "ERROR: Aeq and qobj must have compatible sizes." << "\n"); return false; }
  if(!LinearConstraints::IsValid()) return false;
  return true;
}

Real QuadraticProgram::Objective(const Vector &x) const
{
  Vector v;
  Pobj.mul(x,v);
  return Half*dot(v,x) + dot(qobj,x);
}
