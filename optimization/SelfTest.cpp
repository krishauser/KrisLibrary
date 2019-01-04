#include <KrisLibrary/Logger.h>
#include "SelfTest.h"
#include "Minimization.h"
#include "NewtonSolver.h"
#include "LCP.h"
#include "QuadraticProgram.h"
//#include "QPActiveSetSolver.h"
#include "LSQRInterface.h"
#include <iostream>
#include <math/vectorfunction.h>
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
#include <math/linalgebra.h>
#include <math/random.h>
using namespace std;

namespace Optimization {

  void RandomSparseMatrix(SparseMatrix& A,int nnz,Real range)
  {
    A.setZero();
    for(int k=0;k<nnz;k++) {
      int i=RandInt(A.m);
      int j=RandInt(A.n);
      Real x=Rand(-range,range);
      A.insertEntry(i,j,x);
    }
  }
  void RandomVector(Vector& b,Real range)
  {
    for(int i=0;i<b.n;i++) b(i)=Rand(-range,range);
  }

  void TestLSQR(const SparseMatrix& A,const Vector& b)
  {
    Matrix mA;
    Vector x,temp;
    A.get(mA);
    x.resize(A.n);
    temp.resize(A.n);
    LSQRInterface lsqr;
    if(!lsqr.Solve(A,b)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Failed LSQR solve!");
      LOG4CXX_INFO(KrisLibrary::logger(),"A=");
      LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(mA));
      LOG4CXX_INFO(KrisLibrary::logger(),"b="<<VectorPrinter(b));
      return;
    }
    x=lsqr.x;
    MatrixEquation eq(mA,b);
    if(!eq.LeastSquares_Cholesky(temp)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Failed cholesky solve!");
      LOG4CXX_INFO(KrisLibrary::logger(),"A=");
      LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(mA));
      LOG4CXX_INFO(KrisLibrary::logger(),"b="<<VectorPrinter(b));
      return;
    }

    if(!x.isEqual(temp,1e-3)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Results differ!");
      LOG4CXX_INFO(KrisLibrary::logger(),"LSQR: "<<VectorPrinter(x));
      LOG4CXX_INFO(KrisLibrary::logger(),"Cholesky: "<<VectorPrinter(temp));
    }
  }

  void LSQRSelfTest() {
    SparseMatrix A;
    Vector b;
    {
      int m=5,n=3,nnz=10;
      A.resize(m,n);
      b.resize(m);
      RandomSparseMatrix(A,nnz,1);
      RandomVector(b,1);
      
      LOG4CXX_INFO(KrisLibrary::logger(),"Overconstrained test");
      TestLSQR(A,b);
    }

    {
      int m=3,n=5,nnz=10;
      A.resize(m,n);
      b.resize(m);
      RandomSparseMatrix(A,nnz,1);
      RandomVector(b,1);
      
      LOG4CXX_INFO(KrisLibrary::logger(),"Underconstrained test");
      TestLSQR(A,b);
    }
  }

  void NewtonEqualitySelfTest()
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Testing equality-constrained Newton solver...");
    Matrix A(2,2);
    Vector b(2);
    Real c;
    A.setIdentity(); A *= -Two;
    b.setZero();
    c=10;
    shared_ptr<ComponentVectorFieldFunction> C(new ComponentVectorFieldFunction());
    C->functions.push_back(make_shared<QuadraticScalarFieldFunction>(A,b,c));

    Vector p(2);
    p(0) = -1; p(1) = 0;
    Real q=0;
    shared_ptr<LinearScalarFieldFunction> f(new LinearScalarFieldFunction(p,q));

    NonlinearProgram nlp(f,C);
    nlp.minimize = true;
    nlp.inequalityLess = false;
    NewtonSolver newton(nlp);
    newton.x.resize(2); newton.x(0) = -1; newton.x(1) = -1;

    newton.verbose=1;
    newton.Init();
    int iters=100;
    ConvergenceResult res=newton.Solve(iters);
    LOG4CXX_INFO(KrisLibrary::logger(),"Result "<<res<<" iters "<<iters);
    LOG4CXX_INFO(KrisLibrary::logger(),"x = "<<VectorPrinter(newton.x));
    LOG4CXX_INFO(KrisLibrary::logger(),"done.");
  }

  void NewtonInequalitySelfTest()
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Testing inequality-constrained Newton solver...");
    Matrix A(2,2);
    Vector b(2);
    Real c;
    A.setIdentity(); A *= Two;
    b.setZero();
    c=-1;
    QuadraticScalarFieldFunction* c1 = new QuadraticScalarFieldFunction(A,b,c); 
    Vector j(2);  j(0) = 0; j(1) = 1;
    LinearScalarFieldFunction* d1 = new LinearScalarFieldFunction(j,-0.5);  //y greater than 0.5
    shared_ptr<ComponentVectorFieldFunction> C (new ComponentVectorFieldFunction);
    shared_ptr<ComponentVectorFieldFunction> D (new ComponentVectorFieldFunction);
    C->functions.resize(1); C->functions[0].reset(c1);
    D->functions.resize(1); D->functions[0].reset(d1);

    Vector p(2);
    p(0) = -1; p(1) = 0;
    shared_ptr<LinearScalarFieldFunction> f(new LinearScalarFieldFunction(p,0));

    NonlinearProgram nlp(f,C,D);
    nlp.minimize = true;
    nlp.inequalityLess = false;
    NewtonSolver newton(nlp);
    newton.x.resize(2); newton.x(0) = -1; newton.x(1) = 0.7;

    newton.verbose=1;
    newton.Init();
    int iters=100;
    ConvergenceResult res=newton.Solve(iters);
    LOG4CXX_INFO(KrisLibrary::logger(),"Result "<<res<<" iters "<<iters);
    LOG4CXX_INFO(KrisLibrary::logger(),"x = "<<VectorPrinter(newton.x));
    LOG4CXX_INFO(KrisLibrary::logger(),"done.");
  }

  void LCPSelfTest() {
    Matrix A(3,3);
    Vector b(3);
    {
      A(0,0) = 0; A(0,1) = 0; A(0,2) = 1;
      A(1,0) = 0; A(1,1) = 0; A(1,2) = 1;
      A(2,0) = -1; A(2,1) = -1; A(2,2) = 0;
      b(0) = -2;
      b(1) = -1;
      b(2) = 3;
      LemkeLCP lcp(A,b);
      lcp.verbose = 3;
      if(!lcp.Solve()) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"LCP didn't solve!");
	Abort();
      }
      Vector w,z;
      lcp.GetW(w);
      lcp.GetZ(z);
      LOG4CXX_INFO(KrisLibrary::logger(),"W: "<<VectorPrinter(w));
      LOG4CXX_INFO(KrisLibrary::logger(),"Z: "<<VectorPrinter(z));
    }
    {
      A(0,0) = -1; A(0,1) = -1; A(0,2) = 0;
      A(1,0) = 0; A(1,1) = 0; A(1,2) = 1;
      A(2,0) = 0; A(2,1) = 0; A(2,2) = 1;
      b(0) = 2;
      b(1) = -1;
      b(2) = -1;
      LemkeLCP lcp(A,b);
      lcp.verbose = 3;
      if(!lcp.Solve()) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"LCP didn't solve!");
	Abort();
      }
      Vector w,z;
      lcp.GetW(w);
      lcp.GetZ(z);
      LOG4CXX_INFO(KrisLibrary::logger(),"W: "<<VectorPrinter(w));
      LOG4CXX_INFO(KrisLibrary::logger(),"Z: "<<VectorPrinter(z));
    }
  }

  void QPSelfTest()
  {
    QuadraticProgram qp;
    qp.Resize(3,2);
    qp.Pobj.setZero();
    qp.Pobj(0,0) = 1;
    qp.Pobj(1,1) = 2;
    qp.qobj(0) = 5;
    qp.A(0,0) = -1;  qp.A(0,1) = 0;  qp.p(0) = 1;
    qp.A(1,0) = -1;  qp.A(1,1) = -1;  qp.p(1) = 2; 
    qp.A(2,0) = -1;  qp.A(2,1) = 1;  qp.p(2) = 2;
    /*
    qp.Resize.resize(1,2);
    qp.A(0,0) = 0;  qp.A(0,1) = 1; qp.p(0) = qp.q(0) = 1.5;
    */
    Assert(!qp.HasBounds());
    /*
    QPActiveSetSolver solver(qp);
    solver.verbose = 2;
    ConvergenceResult res = solver.Solve();
    Assert(res == ConvergenceF || res == ConvergenceX);

    //bound-constrained problems up to 20 dimensions
    LOG4CXX_INFO(KrisLibrary::logger(),"Testing bound-constrained problems...");
    solver.verbose = 0;
    for(int n=2;n<20;n++) {  //setup bound constraints
      //pick a minimum norm problem -- min ||x - x0||
      qp.Resize(n*2,n);
      qp.Pobj.setIdentity();
      Vector bmin(n),bmax(n);
      for(int i=0;i<n;i++) bmin(i) = -Rand();
      for(int i=0;i<n;i++) bmax(i) = Rand();
      qp.A.setZero();
      for(int i=0;i<n;i++) {
	qp.A(i,i) = 1;
	qp.p(i) = bmax(i);
      }
      for(int i=0;i<n;i++) {
	qp.A(i+n,i) = -1;
	qp.p(i+n) = -bmin(i);
      }

      //objectives:
      Vector x0(n);
      //1) inside bounds
      for(int i=0;i<n;i++) x0(i) = Rand(bmin(i),bmax(i));
      qp.qobj.setNegative(x0);
      res = solver.Solve();
      Assert(res == ConvergenceF || res == ConvergenceX);
      Assert(x0.isEqual(solver.x,1e-2));  //must be the same point
      //2) outside all bounds
      for(int i=0;i<n;i++) {
	Real posOffset = -Log(Rand(0.1,1.0));
	Assert(posOffset >= 0);
	if(RandBool()) x0(i) = bmin(i) - posOffset;
	else x0(i) = bmax(i) + posOffset;
      }
      qp.qobj.setNegative(x0);
      res = solver.Solve();
      Assert(res == ConvergenceF || res == ConvergenceX);
      for(int i=0;i<n;i++) {
	if(x0(i) < bmin(i)) x0(i) = bmin(i);
	else if(x0(i) > bmax(i)) x0(i) = bmax(i);
      }
      Assert(x0.isEqual(solver.x,1e-2));  //must be the same point
      //3) outside some bounds, but not others
      for(int i=0;i<n;i++) {
	x0(i) = Rand(bmin(i)-0.5*(bmax(i)-bmin(i)),bmax(i)+0.5*(bmax(i)-bmin(i)));
      }
      qp.qobj.setNegative(x0);
      res = solver.Solve();
      Assert(res == ConvergenceF || res == ConvergenceX);
      for(int i=0;i<n;i++) {
	if(x0(i) < bmin(i)) x0(i) = bmin(i);
	else if(x0(i) > bmax(i)) x0(i) = bmax(i);
      }
      Assert(x0.isEqual(solver.x,1e-2));  //must be the same point
    }
    */
    FatalError("TODO: QP Solve");

  }

struct RosenbrockFunction : public ScalarFieldFunction
{
  virtual Real Eval(const Vector& v) 
  {
    Real x=v(0),y=v(1);
    return Sqr(One-x) + 105*Sqr(y - Sqr(x));
  }
  virtual void Gradient(const Vector& v,Vector& grad)
  {
    Real x=v(0),y=v(1);
    grad.resize(2);
    grad(0) = -Two*(One-x) - 420*(y - Sqr(x))*x;
    grad(1) = 210*(y - Sqr(x));
  }
  virtual void Hessian(const Vector& v,Matrix& H)
  {
    Real x=v(0),y=v(1);
    H.resize(2,2);
    H(0,0) = Two - 420*y + 1260*Sqr(x);
    H(0,1) = H(1,0) = -420*x;
    H(1,1) = 210;
  }
};

void MinimizationSelfTest()
{
  RosenbrockFunction f;
  MinimizationProblem p(&f);
  p.x.resize(2);

  /*
  p.x.setZero();
  TestGradient(&f,p.x,1e-5,1e-3);
  TestHessian(&f,p.x,1e-4,1e-3);
  Assert(p.x.isZero(1e-8));
  */

  p.x.setZero();
  int iters = 1000;
  ConvergenceResult res = p.SolveSD(iters);
  LOG4CXX_INFO(KrisLibrary::logger(),"SD: result "<<res<<", "<<iters<<" iters, x="<<p.x(0)<<" "<<p.x(1)<<", value="<<p.fx);

  p.x.setZero();
  iters = 1000;
  res = p.SolveNewton(iters);
  LOG4CXX_INFO(KrisLibrary::logger(),"Newton: result "<<res<<", "<<iters<<" iters, x="<<p.x(0)<<" "<<p.x(1)<<", value="<<p.fx);

  p.x.setZero();
  iters = 1000;
  res = p.SolveQuasiNewton_Ident(iters);
  LOG4CXX_INFO(KrisLibrary::logger(),"QuasiNewton (I): result "<<res<<", "<<iters<<" iters, x="<<p.x(0)<<" "<<p.x(1)<<", value="<<p.fx);

  p.x.setZero();
  iters = 1000;
  res = p.SolveQuasiNewton_Diff(1e-4,iters);
  LOG4CXX_INFO(KrisLibrary::logger(),"QuasiNewton (D): result "<<res<<", "<<iters<<" iters, x="<<p.x(0)<<" "<<p.x(1)<<", value="<<p.fx);
}


  void SelfTest()
  {
    //LSQRSelfTest();
    QPSelfTest();
    //LCPSelfTest();
    //NewtonInequalitySelfTest();
  }

} //namespace Optimization
