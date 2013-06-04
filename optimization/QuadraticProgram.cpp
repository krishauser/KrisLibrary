#include "QuadraticProgram.h"
#include "QPInteriorPoint.h"
#include "LinearProgram.h"
#include "LPRobust.h"
#include <math/DiagonalMatrix.h>
#include <math/SVDecomposition.h>
#include <math/Conditioner.h>
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

Real Merit(const QPInteriorPoint& qp,const Vector& x,const Vector& lambda,Real t)
{
  //norm^2 gradient of lagrangian + sum of squared equality
  Vector lag;
  qp.qp.Pobj.mul(x,lag);
  lag += qp.qp.qobj;
  for(int i=0;i<qp.Aineq.m;i++) {
    Vector Ai;
    qp.Aineq.getRowRef(i,Ai);
    Real di = qp.bineq(i)-Ai.dot(x);
    lag.madd(Ai,One/(t*di));
  }
  for(int i=0;i<qp.Aeq.m;i++) {
    Vector Ai;
    qp.Aeq.getRowRef(i,Ai);
    lag.madd(Ai,lambda(i));
  }
  return lag.normSquared() + SumOfSquaredEquality(qp.Aeq,qp.beq,x);
}

void QuadraticProgram::Print(ostream& out) const
{
  cout<<"min 1/2 x^T A x + x^T b with A="<<endl;
  cout<<MatrixPrinter(Pobj)<<endl;
  cout<<"and b="<<VectorPrinter(qobj)<<endl;
  cout<<"s.t."<<endl;
  LinearConstraints::Print();
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
  if(!Pobj.isSquare()) { cout << "ERROR: Pobj is not square." << endl; return false; }
  if(Pobj.m != qobj.n) { cout << "ERROR: Pobj and qobj must have compatible sizes." << endl; return false; }
  if(A.n != 0) 
    if (A.n != qobj.n) { cout << "ERROR: Aeq and qobj must have compatible sizes." << endl; return false; }
  if(!LinearConstraints::IsValid()) return false;
  return true;
}

Real QuadraticProgram::Objective(const Vector &x) const
{
  Vector v;
  Pobj.mul(x,v);
  return Half*dot(v,x) + dot(qobj,x);
}

// Returns the value of the objective function, including the inequality barrier functions
Real QPInteriorPoint::Objective_Ineq(const Vector& x, Real t) const
{
  Real obj = t*qp.Objective(x);
  for(int i=0;i<qp.A.m;i++) {
    if(qp.ConstraintType(i) == LinearConstraints::Fixed)
      continue;
    if(!IsInf(qp.q(i)))  //lower bounded
      obj -= Log(qp.A.dotRow(i,x) - qp.q(i));
    if(!IsInf(qp.p(i)))
      obj -= Log(qp.p(i) - qp.A.dotRow(i,x));
  }
  for(int i=0;i<qp.A.n;i++) {
    if(qp.VariableType(i) == LinearConstraints::Fixed)
      continue;
    if(!IsInf(qp.l(i)))  //lower bounded
      obj -= Log(x(i) - qp.l(i));
    if(!IsInf(qp.u(i)))
      obj -= Log(qp.u(i) - x(i));
  }
  return obj;
}

bool QPInteriorPoint::FindFeasiblePoint(Vector& x0,int verbose) const 
{
  if (verbose>=1) cout << " - finding feasible point:" << endl;

  LinearProgram lp;
  lp.SetRef(qp);
  lp.c.resize(qp.qobj.n);
  lp.c.setZero();
  //lp.c(0)=1;

  RobustLPSolver lps;
  lps.UpdateGLPK(lp);
  LinearProgram::Result res=lps.SolveGLPK();//lps.Solve(lp);

  switch(res) {
  case LinearProgram::Infeasible:
    if (verbose>=1) {
      cout << " - linear program is infeasible" << endl;
    }
    return false;
  case LinearProgram::Error:
    if (verbose>=1) {
      cout << " - found NO feasible point" << endl;
    }
    return false;
  case LinearProgram::Feasible:
    if (verbose>=2) {
      cout << " - found a feasible point" << endl;
    }
    break;
  case LinearProgram::Unbounded:
    if (verbose>=1) {
      cout << " - LP is unbounded" << endl;
    }
    FatalError("TODO: solve for initial point in unbounded LP");
    break;
  }

  x0.resize(qp.qobj.n);
  x0 = lps.xopt;
  Assert(qp.SatisfiesInequalities(x0));
  Assert(qp.SatisfiesBounds(x0));
  Assert(qp.SatisfiesEqualities(x0));
  return true;
}

QPInteriorPoint::QPInteriorPoint(const QuadraticProgram& _qp)
  :qp(_qp),tol_zero(1e-3),tol_inner(1e-8),tol_outer(1e-5),
   maxOuterIters(40),maxInnerIters(20),
   stopWhenObjectiveIsNegative(false), verbose(0)
{
}

void QPInteriorPoint::SetStopCriterion()
{
  stopWhenObjectiveIsNegative = true;
}

void QPInteriorPoint::SetInitialPoint(const Vector& _x0)
{
  x0 = _x0;

  if(x0.n != qp.Pobj.n) {
    cout << "ERROR: x0 does not have a compatible size"<<endl;
  }

  // Check feasibility of x0
  if (!qp.SatisfiesInequalities(x0)) {
    cerr << "WARNING: initial x0 provided to QuadraticProgram was not actually feasible" << endl;
    x0.resize(0);
  }

  for(int i=0;i<qp.A.m;i++) {
    if(qp.ConstraintType(i) == LinearConstraints::Fixed) {
      cerr << "WARNING: Quadratic program has an equality constraint!" << endl;
    }
  }
  for(int i=0;i<qp.A.n;i++) {
    if(qp.VariableType(i) == LinearConstraints::Fixed) {
      cerr << "WARNING: Quadratic program has an equality-constrained variable!" << endl;
    }
  }
}


bool QPInteriorPoint::Solve() 
{
  if (verbose>=1) cout << "Solving QuadraticProgram:" << endl;

  if (x0.n == 0) {
    if (! FindFeasiblePoint(x0,verbose)) {
      cout << "ERROR: couldn't find an initial feasible point in QuadraticProgram::solve()" << endl;
      return false;
    }
  }

  if (verbose>=2)  { cout << "x0 = "<<VectorPrinter(x0)<<endl; }
	
	
  // Make sure starting x is always set to x0 when we begin to solve.
  // Make sure starting lambda is set to lambda0 (all 1's)
  if (x.n != 0) cout << "WARNING: x should be NULL at this point in QPInteriorPoint::Solve()" << endl;
  if (lambda.n != 0) cout << "WARNING: lambda should be NULL at this point in QPInteriorPoint::Solve()" << endl;
  x = x0;
  lambda.resize(Aeq.m,One);
    
  // Already checked that the initial point is feasible.
	
  //
  // Algorithm parameters
  //
    
  //	t	- indicates how far the search is along the "central path"; basically,
  //		  it just trades off how much we weight the goal vs. the inequality barriers.
  double t=0.01;
  //	mu	- multiple by which we increase t at each outer iteration.
  double mu=15;
  //	alpha	- ? - in backtracking line search, I think this indicates a goal decrement or something.
  //	beta	- in backtracking line search, how much we back off at each iteration
  double alpha=0.2;
  double beta=0.5;
  
  //
  // Outer Loop
  //

  DiagonalMatrix d;
  Matrix Hsub,H,HC_temp,HC;
  Vector g,rhs_temp,rhs;
  Vector dxdy,dx,dy;
  Vector xcur(x.n),lambdacur(lambda.n);
  qp.GetSimpleForm(Aeq,bineq,Aineq,bineq);

  Assert(lambda.n == Aeq.m);
  dxdy.resize(x.n+lambda.n);
  dx.setRef(dxdy,0,1,x.n);
  dy.setRef(dxdy,x.n,1,lambda.n);

  //cout<<"Pobj: "<<MatrixPrinter(Pobj)<<endl;
  //cout<<"qobj: "<<VectorPrinter(qobj)<<endl;
  //getchar();
  
  double gap=1.0;
  int iter_outer=0;
  while (gap>tol_outer) {   
    if(verbose >= 2) { cout << "Current x = "<<VectorPrinter(x)<<endl; }

    ++iter_outer;
    if (iter_outer > maxOuterIters) {
      cout << "WARNING: QPInteriorPoint outer loop did not converge within max iters" << endl;
      return false;
    }
    
    // Update position along the central path
    t *= mu;
    
    if (verbose>=2) cout << " Outer iter " << iter_outer << ", t=" << t << endl;
    
    //
    // Inner Loop
    //
    
    double dec=1.0;
    int iter_inner=0;
    while ((fabs(dec)>tol_inner) && (iter_inner <= maxInnerIters)) {
      ++iter_inner;
      /*
      if (iter_inner > kMaxIters_Inner) {
	cout << "WARNING: QPInteriorPoint inner loop did not converge within max iters" << endl;
	return false;
      }
      */
      
      // (1) Compute the direction to descend
      //        Non-equality constrained:     
      //		- Newton direction is dx = -(Hessian^-1)*(gradient)
      //		- H = Pobj+1/t(Aineq'*(diag(d.^2))*Aineq)
      //		- g = (Pobj*x+qobj) + 1/t(Aineq'*d)
      //        Equality constrained:  (y denotes lagrange multipliers)
      //		- Solve H*dx + Aeq'*dy = -g
      //                        Aeq*dx         = -beq
      //		- H = Pobj+1/t(Aineq'*(diag(d.^2))*Aineq)
      //		- g = (Pobj*x+qobj) + 1/t(Aineq'*d) + Aeq'*y
 
      //d = bineq-Aineq*x;
      Aineq.mul(x,d);  d.inplaceNegative();  d+=bineq;
      d.inplaceInverse();
      //Hsub = diag(d)*Aineq
      d.preMultiply(Aineq,Hsub);

      //H = Hsub^t Hsub + t*Pobj
      H.mulTransposeA(Hsub,Hsub);  //NOTE: transposeB is faster
      H *= One/t;
      H += qp.Pobj;
      
      //cout << "Size of H is " << H.m << " x " << H.m << endl;
      
      //g = 1/t*Aineq^t*d + Pobj*x + qobj + Aeq'*y;
      Aineq.mulTranspose(d,g);
      g *= One/t;
      qp.Pobj.madd(x,g); g+=qp.qobj;
      if(Aeq.m != 0) Aeq.maddTranspose(lambda,g);
      
      if(Aeq.m != 0) {
	Real equalityScale = 1;
	//form larger matrix
	assert(Aeq.n == H.n);
	HC.resize(H.n+Aeq.m,H.n+Aeq.m);
	HC.copySubMatrix(0,0,H);
	Matrix temp,temp2;
	temp.setRef(HC,H.n,0,1,1,Aeq.m,H.n);
	temp = Aeq;
	temp.setRef(HC,0,H.n,1,1,H.n,Aeq.m);
	temp2.setRefTranspose(temp);
	temp2.mul(Aeq,equalityScale);
	temp.setRef(HC,H.n,H.n);
	temp.setZero();

	//form rhs
	rhs.resize(g.n+Aeq.m);
	rhs.copySubVector(0,g);
	Vector vtemp;
	vtemp.setRef(rhs,g.n);
	Aeq.mul(x,vtemp); vtemp-=beq;
	vtemp *= equalityScale;
      }
      else {
	rhs.setRef(g);
	HC.setRef(H);
      }
      // Solve the system to find Newton direction dxdy
      // we use a better numerically conditioned matrix HC*scale
      //Note: conditioner modifies the matrices
      HC_temp=HC;
      rhs_temp=rhs;
      HConditioner Hinv(HC_temp,rhs_temp);
      //if(!Hinv.Solve_SVD(dx)) {
      if(!Hinv.Solve_LU(dxdy)) {
	RobustSVD<Real> svd;
	if(svd.set(Hinv.A)) {
	  svd.backSub(Hinv.b,dxdy);
	  cout<<"Solved by SVD"<<endl;
	  //cout<<"Singular values "<<svd.W<<endl;
	  cout<<"b "<<Hinv.b<<endl;
	  getchar();
	}
	else {
	  //cout<<"H Matrix "; H.print();
	  cout<<"Scaled H Matrix "<<endl<<MatrixPrinter(Hinv.A)<<endl;
	  cout<<"QP: Error performing H^-1*g!"<<endl;
	  return false;
	}
      }
      Hinv.Post(dxdy);

      /*
      cout<<"D: "<<VectorPrinter(d)<<endl;
      MatrixPrinter pHC(HC); pHC.mode=MatrixPrinter::AsciiShade;
      cout<<"HC: "<<pHC<<endl;
      cout<<"rhs: "<<VectorPrinter(rhs)<<endl;
      cout<<"dxdy: "<<VectorPrinter(dxdy)<<endl;
      */
      /*
      Vector res;
      HC.mul(dxdy,res);
      res -= rhs;
      if(res.norm() > 1e-2) {
	cout<<"QPInteriorPoint: solve residual: "<<res.norm()<<endl;
	cout<<"May want to use conditioned solver"<<endl;
	cout<<"Press return to continue"<<endl;
	getchar();
      }
      */

      dxdy.inplaceNegative();

      // (2) Line search along dx,dy, using backtracking
      Real dec = Abs(dxdy.dot(rhs));
      double s=1;
      Real merit_x = Merit(*this,x,lambda,t);
      if(verbose >= 3) {
	cout<<"Starting line search with merit "<<merit_x<<"..."<<endl;
	cout<<"Objective: "<<qp.Objective(x)<<"Min inequality "<<qp.InequalityMargin(x)<<", distance "<<SumOfSquaredEquality(Aeq,beq,x)<<endl;
      }
      //cout<<"    direction "; dx.print();

      //limit s by the inequalities
      Assert(qp.SatisfiesInequalities(x));	
      {
	Real smax = 2;
	for(int i=0;i<Aineq.m;i++) {
	  Real r = bineq(i) - Aineq.dotRow(i,x);
	  Real p = Aineq.dotRow(i,dx);
	  Assert(r >= Zero);
	  if(p > Zero)
	    smax = Min(smax,r/p);
	}
	Assert(smax >= 0.0);
	if(smax <= 1.0)
	  s = smax*(0.99);
      }

      int iter_backtrack=0;
      bool done_backtrack=false;
      while (! done_backtrack) {
	++iter_backtrack;
	if (verbose>=4) cout << "   Backtrack iter " << iter_backtrack << ", s=" << s << ", dec=" << dec << endl;
	xcur=x; xcur.madd(dx,s);   //xcur = x+s*dx
	if(lambda.n != 0) {
	  lambdacur=lambda; lambdacur.madd(dy,s);   //ycur = y+s*dy
	}
	bool sat_ineq=qp.SatisfiesInequalities(xcur);
	//inequalities will be satisfied unless there's some numerical error
	/*
	if(!sat_ineq) {
	  cout<<"WHAT?!?! s is "<<s<<" on iter "<<iter_backtrack<<endl;
	  cout<<"xcur "; xcur.print();
	  cout<<"dx "; dx.print();
	  getchar();
	  }*/
	Real merit = Merit(*this,xcur,lambdacur,t);
	if (sat_ineq && (merit <= merit_x-(alpha*s*dec))) {
	  x=xcur;
	  lambda=lambdacur;
	  done_backtrack = true;
	} 
	else {
	  if(!sat_ineq) {
	    cout<<"Inequalities not satisfied, margin "<<qp.InequalityMargin(xcur)<<"<0, orig margin="<<qp.InequalityMargin(x)<<"t="<<t<<endl;
	    s=0;
	    done_backtrack=true;
	  }
	  //cout<<"current opt value "<<Objective_Ineq(x,t)<<endl;
	  //cout<<"searched one "<<Objective_Ineq(xcur,t)<<endl;
	  //cout<<"current opt value "<<Objective(x)<<endl;
	  //cout<<"searched one "<<Objective(xcur)<<endl;
	  //cout<<"alpha*s*dec "<<alpha*s*dec<<endl;
	  //getchar();
	  s *= beta;
	}
      }

      /*
      cout<<"New merit: "<<Merit(*this,x,lambda,t)<<endl;
      cout<<"Distance "<<qp.EqualityMargin(x)<<endl;
      cout<<"Inequality distance "<<qp.InequalityMargin(x)<<endl;
      getchar();
      */

      if (verbose>=3) cout << "  Inner iter " << iter_inner << ", obj=" << qp.Objective(x) << ", s=" << s << ", dec=" << dec << ", gap=" << ((double) bineq.n) / t << endl;
      //if (verbose>=4) cout << "   Backtrack iter " << iter_backtrack << ", s=" << s << ", dec=" << dec << endl;
			
      double obj_cur = qp.Objective(x);
      if (stopWhenObjectiveIsNegative && (obj_cur < 0.0)) {
	return true;
      } else {
	//cout << " - current value of objective = " << obj_cur << endl;
      }

      //no progress
      if(s < 1e-10) {
	//cout<<"No progress in dir "; dx.print();
	break;
      }
    }
	
    gap = ((double) bineq.n) / t;
  }

  if(!qp.SatisfiesEqualities(x,tol_zero)) {
    cout<<"Final x does not satisfy equality constraints!"<<endl;
    cout<<"Final error "<<qp.EqualityError(x)<<endl;
    cout<<"Press a key to continue..."<<endl;
    getchar();
  }
  //cout << "Final x:" << endl; x.print();
    
  if (verbose>=1) cout << "Optimization was successful." << endl;
  return true;
}


