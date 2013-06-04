#include "QPActiveSetSolver.h"
#include "LPRobust.h"
#include <math/backsubstitute.h>
#include <errors.h>
using namespace Optimization;
using namespace std;

#define OPTIMIZED_UPDATE 1
#define TEST_OPTIMIZED_UPDATE 1

//if H is a symmetric matrix with inverse Hinv, forms the inverse of the
//matrix H' = [H     | h12]
//            [h12^t | h22]
bool GrowInverseUpdate(const Matrix& Hinv,const Vector& h12,Real h22,Matrix& out)
{
  Assert(Hinv.isSquare());
  Assert(h12.n == Hinv.m);
  int n=Hinv.m;
  out.resize(n+1,n+1);
  Vector temp;
  Hinv.mul(h12,temp);
  Real d=h22-h12.dot(temp);
  if(FuzzyEquals(d,Zero)) {
    return false;
  }
  
  Matrix M11;
  Vector M12,M21;
  M11.setRef(out,0,0,1,1,n,n);
  out.getColRef(n,M12);
  out.getRowRef(n,M21);
  M12.n--;
  M21.n--;

  M11 = Hinv;
  for(int i=0;i<n;i++) {
    for(int j=0;j<n;j++) {
      M11(i,j) -= temp(i)*temp(j)/d;
    }
  }
  M12.mul(temp,-1.0/d);
  M21 = M12;
  out(n,n) = 1.0/d;
  return true;
}

//if H is the cholesky decomposition of a symmetric matrix,
//forms the cholesky of the matrix:
//H'=[H     | h12]
//   [h12^t | h22]
bool GrowCholeskyUpdate(const CholeskyDecomposition<Real>& H,const Vector& h12,Real h22,CholeskyDecomposition<Real>& out)
{
  Assert(h12.n == H.L.m);
  Assert(H.L.isSquare());
  int n=H.L.m;
  out.L.resize(n+1,n+1);
  Matrix K11;
  Vector K12,K21;
  K11.setRef(out.L,0,0,1,1,n,n);
  out.L.getRowRef(n,K12);
  out.L.getColRef(n,K21);
  K12.n--;
  K21.n--;
  Assert(K12.n == h12.n);

  K11.copy(H.L);
  H.LBackSub(h12,K12);
  K21.setZero();
  Real det=h22 - K12.dot(K12);
  if(det < 0) {
    return false;
  }
  out.L(n,n) = Sqrt(det);
  return true;
}

//get the cholesky decomposition of H' = remove row/column k from the matrix H
void DeleteCholeskyUpdate(const CholeskyDecomposition<Real>& H,int k,CholeskyDecomposition<Real>& out)
{
  Assert(H.L.isSquare());
  int n=H.L.m;
  out.L.resize(n-1,n-1);
  Matrix L11,L31,L33;
  Vector L32;
  Matrix K11,K12,K21,K22;
  L11.setRef(H.L,0,0,1,1,k,k);
  K11.setRef(out.L,0,0,1,1,k,k);
  K11 = L11;
  if(k+1 == n) {
    return;
  }
  L31.setRef(H.L,k+1,0,1,1,n-k-1,k);
  L33.setRef(H.L,k+1,k+1,1,1,n-k-1,n-k-1);
  Vector temp;
  H.L.getColRef(k,temp);
  L32.setRef(temp,k+1);
  K12.setRef(out.L,0,k,1,1,k,L31.m);
  K21.setRef(out.L,k,0,1,1,L31.m,k);
  K22.setRef(out.L,k,k,1,1,L33.m,L33.n);

  K12.setZero();
  K21.copy(L31);
  //solve for K22K22t = L33L33t + L32L32t
  K22.copy(L33);
  CholeskyDecomposition<Real> K22chol;
  K22chol.L.setRef(K22);
  K22chol.update(L32);
  K22chol.L.clear();
}


//if H is the cholesky decomposition of a symmetric matrix,
//forms the cholesky of the matrix:
//H'=[H     | h12]
//   [h12^t | h22]
void GrowCholeskyUpdate(const LDLDecomposition<Real>& H,const Vector& h12,Real h22,LDLDecomposition<Real>& out)
{
  Assert(h12.n == H.LDL.m);
  Assert(H.LDL.isSquare());
  int n=H.LDL.m;
  out.LDL.resize(n+1,n+1);
  Matrix K11;
  Vector K12,K21;
  K11.setRef(out.LDL,0,0,1,1,n,n);
  out.LDL.getRowRef(n,K12);
  out.LDL.getColRef(n,K21);
  K12.n--;
  K21.n--;
  Assert(K12.n == h12.n);

  K11.copy(H.LDL);
  //h12 = LDK12
  Vector DK12;
  H.LBackSub(h12,DK12);
  H.DBackSub(DK12,K12);
  K21 = K12;
  out.LDL(n,n) = h22 - K12.dot(DK12);

  //TEST
  Matrix A,A2,Atest;
  H.getA(A);
  out.getA(A2);
  Atest.setRef(A2,0,0,1,1,n,n);
  if(!A.isEqual(Atest,1e-2)) {
    cout<<"GrowCholeskyUpdate: some update error in upper left"<<endl;
    cout<<"Original matrix:"<<endl;
    cout<<MatrixPrinter(A)<<endl;
    cout<<"New matrix:"<<endl;
    cout<<MatrixPrinter(Atest)<<endl;
    getchar();
  }
  A2.getColRef(n,K12);
  if(!FuzzyEquals(K12(n),h22,1e-2)) {
    cout<<"GrowCholeskyUpdate: some update error in lower right"<<endl;
    cout<<"Desired "<<h22<<", actual "<<K12(n)<<endl;
    cout<<"Original LDL: "<<endl;
    cout<<MatrixPrinter(H.LDL)<<endl;
    getchar();
  }
  K12.n--;
  if(!h12.isEqual(K12,1e-2)) {
    cout<<"GrowCholeskyUpdate: some update error in upper right"<<endl;
    cout<<"Desired "<<VectorPrinter(h12)<<endl;
    cout<<"Actual "<<VectorPrinter(K12)<<endl;
    cout<<"Original LDL: "<<endl;
    cout<<MatrixPrinter(H.LDL)<<endl;
    getchar();
  }
}

/*
  does making this grow update make the matrix singular?
  H'x=[H     | h12][x1]=0
  [h12^t | h22][x2]
  H x1 + h12 x2=0
  h12^t x1 + h22 x2=0
  
  x1 + H^-1 h12 x2=0
  if h22 = 0,
     h12^t x1 = 0
     h12^t x1 + h12^t H^-1 h12 x2=0
     => x2 = 0 (in which case x=0) or h12^t H^-1 h12 = 0
     => h12^t H^-1 h12 = 0
  otherwise:
     x2=-h12^t x1 / h22
     =>   H x1 - h12 h12^t / h22 x1 = 0
     =>   (H - h12 h12^t / h22) x1 = 0
*/    
bool IsGrowCholeskyUpdateSingular(const LDLDecomposition<Real>& H,const Vector& h12,Real h22)
{
  if(h22 == Zero) {
    Vector temp;
    H.backSub(h12,temp);
    return FuzzyZero(temp.dot(h12));
  }
  else {
    LDLDecomposition<Real> temp;
    temp.LDL = H.LDL;
    Vector x=h12;
    if(h22 > 0) {
      x.inplaceDiv(Sqrt(h22));
      return temp.downdate(x);
    }
    else {
      //it's an update, so this must be true
      return true;
    }
  }
}

bool DeleteCholeskyUpdate(const LDLDecomposition<Real>& H,int k,LDLDecomposition<Real>& out)
{
  Assert(H.LDL.isSquare());
  int n=H.LDL.m;
  out.LDL.resize(n-1,n-1);
  Matrix L11,L31,L33;
  Vector L32;
  Matrix K11,K12,K21,K22;
  L11.setRef(H.LDL,0,0,1,1,k,k);
  K11.setRef(out.LDL,0,0,1,1,k,k);
  K11 = L11;
  if(k+1 == n) {
    return true;
  }
  L31.setRef(H.LDL,k+1,0,1,1,n-k-1,k);
  L33.setRef(H.LDL,k+1,k+1,1,1,n-k-1,n-k-1);
  Vector temp;
  H.LDL.getColRef(k,temp);
  L32.setRef(temp,k+1);
  K12.setRef(out.LDL,0,k,1,1,k,L31.m);
  K21.setRef(out.LDL,k,0,1,1,L31.m,k);
  K22.setRef(out.LDL,k,k,1,1,L33.m,L33.n);

  K21.copy(L31);
  K12.setTranspose(K21);
  //solve for K22E2K22t = L33D3L33t + L32D2L32t
  K22.copy(L33);
  LDLDecomposition<Real> K22chol;
  Real D2 = H.LDL(k,k);
  K22chol.LDL.setRef(K22);
  temp.clear();
  temp.mul(L32,Sqrt(Abs(D2)));
  if(D2 < 0) {
    bool res=K22chol.downdate(temp);
    return res;
  }
  else K22chol.update(temp);
  K22chol.LDL.clear();
  return true;
}


QPActiveSetSolver::QPActiveSetSolver(const QuadraticProgram& _qp)
  :qp(_qp),tol_zero(1e-4),maxIters(100),stopWhenObjectiveIsNegative(false),verbose(0)
{}

ConvergenceResult QPActiveSetSolver::Solve()
{
  Assert(qp.IsValid());
  if(qp.HasBounds()) 
    FatalError("QPActiveSetSolver doesn't work with bound constraints yet!");

  activeSet.clear();
  isActive.resize(qp.A.m);
  fill(isActive.begin(),isActive.end(),false);

  if(!FindFeasiblePoint()) {
    if(verbose >= 1) cout<<"QPActiveSetSolver: Error, couldn't find a feasible point!"<<endl;
    return ConvergenceError;
  }
  //Assert(qp.InequalityMargin(x) >= 0);

  //compute initial augmented Hessian inverse
  A.m = 0;
  if(!ComputeHinv()) {
    if(verbose >= 1) cout<<"QPActiveSetSolver: Unable to set inverse of initial Hessian"<<endl;
    return ConvergenceError;
  }

  //fill the current active set, reserve enough space so that the data isn't cleared
  A.resize(qp.A.m,qp.Pobj.n);
  b.resize(A.m);
  //fill initial active set
  int numActive=0;
  for(int i=0;i<qp.A.m;i++) {
    if(qp.ConstraintType(i) == LinearConstraints::Fixed) {
      Vector temp;
      qp.A.getRowRef(i,temp);
      vtemp.resize(Hinv.LDL.n);
      vtemp.setZero();
      vtemp.copySubVector(0,temp);
      if(!IsGrowCholeskyUpdateSingular(Hinv,vtemp,0)) { //is an independent constraint
	LDLDecomposition<Real> cholesky;
	GrowCholeskyUpdate(Hinv,vtemp,0,cholesky);
	swap(Hinv.LDL,cholesky.LDL);
	cout<<"Initial LDL0"<<endl<<MatrixPrinter(cholesky.LDL)<<endl;

	A.copyRow(numActive,temp);
	b(numActive) = qp.p(i);
	isActive[i] = true;
	activeSet.push_back(i);
	numActive++;
      }
    }
  }
  A.m = numActive;
  b.n = A.m;

  cout<<"Active set: ";
  for(size_t i=0;i<activeSet.size();i++)
    cout<<activeSet[i]<<" ";
  cout<<endl;
  cout<<"Initial LDL "<<endl<<MatrixPrinter(Hinv.LDL)<<endl;
  ComputeHinv();
  cout<<"Initial augmented hessian "<<endl<<MatrixPrinter(mtemp)<<endl;
  cout<<"Initial LDL2 "<<endl<<MatrixPrinter(Hinv.LDL)<<endl;

  if(verbose >= 2) cout<<"Starting point: "<<x<<endl;
  if(verbose >= 1) cout<<"Starting objective: "<<qp.Objective(x)<<", equality error: "<<qp.EqualityError(x)<<", margin: "<<qp.InequalityMargin(x)<<endl;
  for(int iters=0;iters<maxIters;iters++) {
    ConvergenceResult res=Step();
    if(verbose >= 1) cout<<"Step "<<iters+1<<": "<<qp.Objective(x)<<", equality error: "<<qp.EqualityError(x)<<", margin: "<<qp.InequalityMargin(x)<<endl;
    if(verbose >= 2) cout<<"Position: "<<x<<endl;
    if(res == ConvergenceX || res == ConvergenceF) return res;
    if(res == ConvergenceError) return res;
  }
  return MaxItersReached;
}

ConvergenceResult QPActiveSetSolver::Step()
{
  Vector xdes,x0,u;
  if(!SolveCurOptimum(xdes,u)) {
    if(verbose >= 1) cout<<"QPActiveSetSolver::Step(): Error solving for optimum"<<endl;
    return ConvergenceError;
  }

  //either add the first inequality that it violates or remove an existing inequality if it will improve the optimum
  Real minu = Inf;
  int index=-1;
  for(int i=0;i<u.n;i++) {
    if(qp.ConstraintType(activeSet[i]) != LinearConstraints::Fixed) {
      if(u(i) < minu) {
	minu = u(i);
	index = i;
      }
    }
  }
  if(minu < -tol_zero) {  //remove the inequality
    cout<<"Removing from active set, lagrange multiplier is "<<minu<<endl;
    RemoveActiveSet(index);
  }
  else {  //move towards optimum
    /*
    for(size_t i=0;i<activeSet.size();i++) {
      int k=activeSet[i];
      Real error = qp.bineq(k) - qp.Aineq.dotRow(k,xdes);
      cout<<"Error for inequality "<<i<<": "<<error<<endl;
    }
    */
    x0 = x;
    xdes -= x;
    //x = x0 + t*xdes
    //solve for root of t for inactive equality constraints
    Real tmin = One;
    int index = -1;
    bool lowerBound = false;
    for(int i=0;i<qp.A.m;i++) {
      if(!isActive[i]) {  //q <= A*x <= p
	Vector ai;
	qp.A.getRowRef(i,ai);
	Real bi = qp.p(i);
	Real ci = qp.q(i);
	
	Real q = dot(ai,xdes);
	if(q > tol_zero) {
	  Real p = dot(ai,x0);
	  if(p > bi+bi*tol_zero) {
	    cout<<"QPActiveSetSolver: Possible numerical error, out of margin on inequality "<<i<<endl;
	    cout<<"Margin: "<<bi - p<<endl;
	    getchar();
	  }
	  //solve for t as root of equation p + t*q <= b
	  Real maxt = (bi-p)/q;
	  maxt = Max(maxt,Zero);
	  if(maxt < tmin) {
	    tmin = maxt;
	    index = i;
	    lowerBound = false;
	  }
	}
	if(q < -tol_zero) {
	  Real p = dot(ai,x0);
	  if(p < ci+ci*tol_zero) {
	    cout<<"QPActiveSetSolver: Possible numerical error, out of margin on inequality "<<i<<endl;
	    cout<<"Margin: "<<p-ci<<endl;
	    getchar();
	  }
	  //solve for t as root of equation p + t*q <= b
	  Real maxt = -(p-ci)/q;
	  maxt = Max(maxt,Zero);
	  if(maxt < tmin) {
	    tmin = maxt;
	    index = i;
	    lowerBound = true;
	  }
	}
      }
      /*
      else {
	Real p=qp.A.dotRow(i,x0);
	Real q=qp.A.dotRow(i,xdes);
	cout<<"Initial margin for active inequality "<<i<<": "<<qp.bineq(i)-p<<", change: "<<q<<endl;
      }
      */
    }
    //move forward maxt
    x = x0;
    x.madd(xdes,tmin);
    if(qp.InequalityMargin(x) < -1e-1) {
      cout<<"Warning: did not take care of all inequalities!"<<endl;
      for(int i=0;i<qp.A.m;i++) {
	Real val=qp.A.dotRow(i,x);
	if(qp.ConstraintType(i) == LinearConstraints::Fixed) 
	  continue;
	if(qp.HasLowerBound(qp.ConstraintType(i))) {
	  Real m = val - qp.q(i);
	  if(m < -1e-1) {
	    cout<<"Margin on inequality "<<i<<" is "<<m<<", active is "<<isActive[i]<<endl;
	    cout<<"b "<<qp.q(i)<<", p "<<qp.A.dotRow(i,x0)<<", q "<<qp.A.dotRow(i,xdes)<<endl;
	    cout<<"tmin = "<<tmin<<endl;
	  }
	}
	if(qp.HasUpperBound(qp.ConstraintType(i))) {
	  Real m = qp.p(i) - val;
	  if(m < -1e-1) {
	    cout<<"Margin on inequality "<<i<<" is "<<m<<", active is "<<isActive[i]<<endl;
	    cout<<"b "<<qp.p(i)<<", p "<<qp.A.dotRow(i,x0)<<", q "<<qp.A.dotRow(i,xdes)<<endl;
	    cout<<"tmin = "<<tmin<<endl;
	  }
	}
      }
      getchar();
    }
    if(index == -1) return ConvergenceF;
    else AddActiveSet(index,lowerBound);
  }
  return MaxItersReached;
}

bool QPActiveSetSolver::FindFeasiblePoint()
{
  if (verbose>=1) cout << " - finding feasible point:" << endl;

  LinearProgram lp;
  lp.SetRef(qp);
  lp.c.resize(qp.qobj.n);
  lp.c.setZero();
  lp.c(0)=1;

  RobustLPSolver lps;
  lps.UpdateGLPK(lp);
  LinearProgram::Result res=lps.SolveGLPK();//lps.Solve(lp);

  switch(res) {
  case LinearProgram::Infeasible:
    if (verbose>=1) {
      cout << " - linear program is infeasible" << endl;
      lp.Print(cout);
      getchar();
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
    {
      //clear the referenced bounds
      lp.u.clear();
      lp.l.clear();
      lp.u.resize(qp.u.n);
      lp.l.resize(qp.u.n);
      //solve a bounded version of the LP
      Real bound;
      for(bound=1.0; bound<1.0e30; bound *= 10.0) {
	for(int i=0;i<lp.u.n;i++) {
	  if(IsInf(qp.u(i))) lp.u(i) = bound;
	  else lp.u(i) = qp.u(i);
	  if(IsInf(qp.l(i))) lp.l(i) = -bound;
	  else lp.l(i) = qp.l(i);
	}
	lps.UpdateGLPK(lp);
	LinearProgram::Result res=lps.SolveGLPK();//lps.Solve(lp);
	if(res == LinearProgram::Feasible) break;
      }
      if(bound >= 1.0e30) {
	cerr<<"couldn't bound unbounded LP with reasonable size"<<endl;
	return LinearProgram::Error;
      }
    }
    break;
  }

  x.resize(qp.qobj.n);
  x = lps.xopt;
  /*
  cout<<"inequality margin: "<<qp.InequalityMargin(x)<<endl;
  Assert(qp.SatisfiesInequalities(x));
  Assert(qp.SatisfiesBounds(x));
  Assert(qp.SatisfiesEqualities(x));
  */
  return true;
}

void QPActiveSetSolver::AddActiveSet(int i,bool lowerBound)
{
  if(verbose >= 2) cout<<"Adding active set "<<i<<endl;
  Assert(!isActive[i]);
  Assert(qp.ConstraintType(i) != LinearConstraints::Fixed);
  activeSet.push_back(i);
  isActive[i] = true;

  //perform updates
  Vector ai;
  qp.A.getRowRef(i,ai);
#if OPTIMIZED_UPDATE
  //add ai to cholesky decomposition of H
  vtemp.resize(qp.Pobj.m+A.m);
  vtemp.setZero();
  if(lowerBound) {
    Vector temp;
    temp.setRef(vtemp,0,1,qp.Pobj.m);
    temp.setNegative(ai);
  }
  else {
    vtemp.copySubVector(0,ai);
  }
  /*
  CholeskyDecomposition<Real> temp;
  bool res=GrowCholeskyUpdate(Hinv,vtemp,ai.dot(Pz),temp);
  Assert(res == true);
  swap(temp.L,Hinv.L);
  */
  LDLDecomposition<Real> temp;
  Assert(!IsGrowCholeskyUpdateSingular(Hinv,vtemp,0));
  GrowCholeskyUpdate(Hinv,vtemp,0,temp);
#if !TEST_OPTIMIZED_UPDATE
  swap(temp.LDL,Hinv.LDL);
#endif  //!TEST_OPTIMIZED_UPDATE
#endif  //OPTIMIZED_UPDATE
  //add ai to constraint matrix
  int row = A.m;
  A.m++;
  b.n++;
  Assert(A.isValid());
  Assert(b.isValid());
  if(lowerBound) {
    Vector temp;
    A.getRowRef(row,temp);
    temp.setNegative(ai);
    b(row) = -qp.q(i);
  }
  else {
    A.copyRow(row,ai);
    b(row) = qp.p(i);
  }
#if TEST_OPTIMIZED_UPDATE
  //compare with Hinv calculated from scratch
  ComputeHinv();
  if(!temp.LDL.isEqual(Hinv.LDL,1e-3)) {
    cout<<"Constraint matrix cholesky decomposition incorrect on grow operation!"<<endl;
    cout<<MatrixPrinter(temp.LDL)<<" vs "<<endl;
    cout<<MatrixPrinter(Hinv.LDL)<<endl;
    getchar();
    Matrix H1,H2;
    temp.getA(H1);
    Hinv.getA(H2);
    //temp.getInverse(H1);
    //Hinv.getInverse(H2);
    cout<<"original matrices: "<<endl;
    cout<<MatrixPrinter(H1)<<" vs "<<endl;
    cout<<MatrixPrinter(H2)<<endl;
    cout<<"Grow vector "<<VectorPrinter(vtemp)<<endl;
    cout<<"Augmented hessian "<<endl<<MatrixPrinter(mtemp)<<endl;
    getchar();
  }
#endif  //TEST_OPTIMIZED_UPDATE
}

void QPActiveSetSolver::RemoveActiveSet(int index)
{
  cout<<"Removing active set "<<activeSet[index]<<endl;
  Assert(isActive[activeSet[index]]);
  Assert(qp.ConstraintType(index) != LinearConstraints::Fixed);
  isActive[activeSet[index]] = false;
  activeSet.erase(activeSet.begin()+index);

  //perform updates
#if OPTIMIZED_UPDATE
  //remove row Aeq.m+index from choleksy decomposition of H
  /*
  CholeskyDecomposition<Real> temp;
  DeleteCholeskyUpdate(Hinv,row,temp);
  swap(temp.L,Hinv.L);
  */
  LDLDecomposition<Real> temp;
  DeleteCholeskyUpdate(Hinv,qp.Pobj.m+index,temp);
#if !TEST_OPTIMIZED_UPDATE
  swap(temp.LDL,Hinv.LDL);
#endif  //!TEST_OPTIMIZED_UPDATE
#endif  //OPTIMIZED_UPDATE
  //remove row Aeq.m+index from constraint matrix by shifting later entries
  for(int i=index;i+1<A.m;i++) {
    Vector ai,an;
    A.getRowRef(i,ai);
    A.getRowRef(i+1,an);
    ai.copy(an);
    b(i) = b(i+1);
  }
  A.m--;
  b.n--;
  Assert(A.isValid());
  Assert(b.isValid());

#if TEST_OPTIMIZED_UPDATE
  ComputeHinv();
  if(!temp.LDL.isEqual(Hinv.LDL,1e-3)) {
    cout<<"Constraint matrix cholesky decomposition incorrect on delete operation!"<<endl;
    cout<<MatrixPrinter(temp.LDL)<<" vs "<<endl;
    cout<<MatrixPrinter(Hinv.LDL)<<endl;
    getchar();
  }
#endif
}

bool QPActiveSetSolver::ComputeHinv()
{
  mtemp.resize(qp.Pobj.m+A.m,qp.Pobj.n+A.m);
  mtemp.copySubMatrix(0,0,qp.Pobj);
  if(A.m != 0) {
    Matrix At; At.setRefTranspose(A); 
    mtemp.copySubMatrix(qp.Pobj.m,0,A);
    mtemp.copySubMatrix(0,qp.Pobj.n,At);
    Matrix corner;
    corner.setRef(mtemp,qp.Pobj.m,qp.Pobj.n);
    corner.setZero();
  }
  //return Hinv.set(mtemp);
  Hinv.LDL.resize(mtemp.m,mtemp.m);
  Hinv.set(mtemp);
  return true;
}

//TODO: optimize, could be done in O(n^2) steps rather than O(n^3)
//TODO: bound constraints
bool QPActiveSetSolver::SolveCurOptimum(Vector& x,Vector& u)
{
  if(verbose >= 2) {
    cout<<"   Active set: ";
    for(size_t i=0;i<activeSet.size();i++)
      cout<<activeSet[i]<<" ";
    cout<<endl;
  }

  if(A.m == 0) {
    Hinv.backSub(qp.qobj,x);
    x.inplaceNegative();
    return true;
  }

  //let P = qp.Pobj, q=qp.qobj, A = active set constraint matrix, b = active set rhs
  //derivative of Lagrangian gives
  //P*x+At*u = -q
  //A*x = b
#if OPTIMIZED_UPDATE
  //Hinv should be calculated already
#else
  if(!ComputeHinv()) {
    if(verbose >= 1) cout<<"QPActiveSetSolver: Unable to set inverse of augmented Hessian"<<endl;
    return false;
  }
#endif
  vtemp.resize(qp.Pobj.m + A.m);
  vtemp.copySubVector(0,qp.qobj);
  vtemp.inplaceNegative();
  vtemp.copySubVector(qp.Pobj.m,b);
  vtemp2.resize(vtemp.n);
  Hinv.backSub(vtemp,vtemp2);
  x.resize(qp.Pobj.m);
  u.resize(A.m);
  vtemp2.getSubVectorCopy(0,x);
  vtemp2.getSubVectorCopy(qp.Pobj.m,u);
  /*
  cout<<"H: "<<endl;
  cout<<MatrixPrinter(mtemp)<<endl;
  cout<<"rhs: "<<VectorPrinter(vtemp)<<endl;
  cout<<"res: "<<VectorPrinter(vtemp2)<<endl;
  getchar();
  */
  return true;
}

//speedups:
//let B = A with an additional row z
//How do you get H' = B*P^-1*Bt from H and z?  How do you get H'^-1?
//P^-1 Bt = [P^-1 At | P^-1 z]
//B P^-1 Bt = [B P^-1 At | B P^-1 z]
// = [ A P^-1 At  | A P^-1 z ]
//   [ zt P^-1 At | zt P^-1 z]
//Let H = A*P^-1*At, M = H^-1, h12 = A P^-1 z, h21 = h12t, h22 = zt P^-1 z
//We have
//H'^-1 = [M11 | M12]
//        [M21 | M22]
//where M11 = M - (Mh12h21M)/d
//      M12 = -Mh12/d
//      M21 = M12t
//      M22 = 1/d
//  and d = h22-h21Mh12

