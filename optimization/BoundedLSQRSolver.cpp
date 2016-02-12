#include "BoundedLSQRSolver.h"
#include <math/AABB.h>
#include <math/linalgebra.h>
#include <limits.h>
#include <iostream>
using namespace Optimization;
using namespace std;

BoundedLSQRSolver::BoundedLSQRSolver(const Matrix& _A,const Vector& _b,const Vector& _l,const Vector& _u)
{
  A.setRef(_A);
  b.setRef(_b);
  l.setRef(_l);
  u.setRef(_u);
  fTol = 1e-7;
  xTol = 1e-7;
  gradTol = 1e-8;
  //maxIters = INT_MAX;
  maxIters = 500;
  verbose = 0;
}

LinearProgram::Result BoundedLSQRSolver::Solve(Vector& x)
{
  assert(b.n == A.m);
  assert(l.n == u.n);
  assert(l.n == A.n);
  if(x.empty()) {
    x.resize(l.n,Zero);
    for(int i=0;i<x.n;i++) 
      if(IsFinite((l(i)+u(i))*0.5))
	x(i) = (l(i)+u(i))*0.5;
  }
  else {
    assert(x.n == l.n);
  }
  Assert(IsFinite(x));

  //cout<<"A: "<<A<<endl;
  //cout<<"X: "<<x<<endl;
  //cout<<"L: "<<l<<endl;
  //cout<<"U: "<<u<<endl;
  std::vector<int> activeSet;
  std::vector<int> activeIndex(x.n,-1);
  Matrix Aact(A.m,A.n);

  //get initial active set
  int k=0;
  for(int i=0;i<x.n;i++) {
    assert(l(i) <= u(i));
    if(x(i) <= l(i)) {
      x(i) = l(i);
    }
    else if(x(i) >= u(i)) {
      x(i) = u(i);
    }
    else {
      activeSet.push_back(i);
      activeIndex[i] = k;
      Vector Ak;
      A.getColRef(i,Ak);
      Aact.copyCol(k,Ak);
      k++;
    }
  }
  Aact.n = k;

  Vector vtemp;
  A.mul(x,vtemp);
  vtemp -= b;
  Real lastResidual=vtemp.norm();
  if(verbose >= 2) {
    cout<<"Active set 0"<<endl;
    for(size_t i=0;i<activeSet.size();i++) cout<<activeSet[i]<<", ";
    cout<<endl;
  }
  bool recomputeW;
  Vector w,Ax,xtemp,dx(x.n);
  for(int iters=0;iters<maxIters;iters++) {
    recomputeW = true;
    //Solve constrained lsqr step
    Vector bact=b;
    //subtract bound variables
    for(size_t i=0;i<activeIndex.size();i++)
      if(activeIndex[i] < 0) {
	Vector Ai;
	A.getColRef(i,Ai);
	bact.madd(Ai,-x(i));
      }
    xtemp.clear();
    MatrixEquation eq(Aact,bact);
    bool lsqrres=eq.LeastSquares(xtemp);
    if(!lsqrres) {
      if(verbose >= 1)
	fprintf(stderr,"Error solving LSQR problem\n");
      return LinearProgram::Error;
    }
    dx.set(Zero);
    for(size_t i=0;i<activeSet.size();i++) 
      dx[activeSet[i]] = xtemp[i]-x[activeSet[i]];
    if(!IsFinite(xtemp)) {
      cout<<"A: "<<Aact<<endl;
      cout<<"B: "<<bact<<endl;
      cout<<"Desired pos "<<xtemp<<endl;
      Matrix AtA;
      AtA.mulTransposeA(Aact,Aact);
      cout<<"AtA: "<<AtA<<endl;
      FatalError("Invalid solution to least squares problem");
    }
    Real alpha=1.0;
    int res=AABBLineSearch(x,dx,l,u,alpha);
    x.madd(dx,alpha);

    //remove indices that hit the bounds from the active set
    if(res < 0) {
      //no change in active set?  must be done.
    }
    else {
      if(alpha == 0) {
	if(verbose >= 2) printf("took a step of size 0, should come up with a different value\n");
	//round off caused us to want to go back into the variable, take it back out of the active set and set w to zero
	for(int i=0;i<x.n;i++) {
	  if(activeIndex[i] >= 0) {
	    if(x(i)==l(i) && dx(i) < 0) {
	      recomputeW = false;
	      w(i) = 0;
	    }
	    else if(x(i)==u(i) && dx(i) > 0) {
	      recomputeW = false;
	      w(i) = 0;
	    }
	  }
	}
	assert(recomputeW == false);
      }
      for(int i=0;i<x.n;i++) {
	if(i == res || (activeIndex[i] >= 0 && (x(i) > u(i)-Epsilon || x(i) < l(i)+Epsilon))) {
	  if(verbose >= 2) printf("Removing axis %d\n",i);
	  //remove from active set, rearrange Aact
	  int k=activeIndex[i];
	  activeSet[k] = activeSet.back();
	  activeIndex[activeSet.back()] = k;
	  Vector Anew;
	  A.getColRef(activeSet.back(),Anew);
	  activeIndex[i] = -1;
	  activeSet.resize(activeSet.size()-1);
	  Aact.copyCol(k,Anew);
	  Aact.n--;

	  //clean up numerical errors
	  if(u(i) - x(i) < x(i) - l(i)) 
	    x(i) = u(i);
	  else
	    x(i) = l(i);
	}
      }
    }

    if((int)activeSet.size()==x.n) {
      return LinearProgram::Feasible;
    }

    //see if we need to leave any bounds
    //compute negated gradient w
    if(recomputeW) {
      A.mul(x,Ax);
      A.mulTranspose(b-Ax,w);
    }
    Real maxDissatisfied = 0.0;
    int maxDissatisfiedIndex = -1;
    for(int i=0;i<x.n;i++) {
      if(activeIndex[i] < 0) { //check KKT condition on this index
	if(u(i) == l(i)) continue;
	if(x(i) == u(i)) {
	  if(-w(i) > maxDissatisfied) {
	    maxDissatisfied = -w(i);
	    maxDissatisfiedIndex = i;
	  }
	}
	else {
	  if(x(i) != l(i)) {
	    cout<<"Error: non-active variable is not on the bound?"<<endl;
	    cout<<x(i)<<" != "<<l(i)<<" or "<<u(i)<<endl;
	  }
	  Assert(x(i) == l(i));
	  if(w(i) > maxDissatisfied) {
	    maxDissatisfied = w(i);
	    maxDissatisfiedIndex = i;
	  }
	}
      }
    }
    if(maxDissatisfiedIndex < 0) {
      if(verbose >= 1) cout<<"Converged after "<<iters<<" iterations"<<endl;
      if(verbose >= 2) {
	cout<<"Type: ";
	for(size_t i=0;i<activeIndex.size();i++)
	  if(activeIndex[i] < 0) {
	    if(x(i) == l(i) && x(i)==u(i)) cout<<"E, ";
	    else if(x(i)==l(i)) cout<<"L, ";
	    else { Assert(x(i)==u(i)); cout<<"U, "; }
	  }
	  else cout<<"F, ";
	cout<<endl;
	cout<<"Gradient: "<<w<<endl;
      }
      return LinearProgram::Feasible;
    }
    else {
      //add this index to the active set
      int i=maxDissatisfiedIndex;
      if(verbose >= 2) printf("Adding axis %d, gradient value %g\n",i,maxDissatisfied);
      int k=(int)activeSet.size();
      activeSet.push_back(i);
      activeIndex[i] = k;
      Vector Ak;
      A.getColRef(i,Ak);
      Aact.copyCol(k,Ak);
      Aact.n++;
    }
    if(recomputeW) {
      if(verbose >= 2) {
	cout<<"Active set "<<iters+1<<endl;
	for(size_t i=0;i<activeSet.size();i++) cout<<activeSet[i]<<", ";
	cout<<endl;
      }
      A.mul(x,vtemp);
      vtemp -= b;
      Real newResidual = vtemp.norm();
      if(verbose >= 1) {
	cout<<"Difference between old and current residual: "<<newResidual<<" vs "<<lastResidual<<endl;
	cout<<"Delta x: "<<dx.norm()*alpha<<endl;
      }
      if(iters > 0) {
	if(FuzzyEquals(newResidual,lastResidual,fTol)) {
	  if(verbose >= 1) cout<<"Converged on f"<<endl;
	  return LinearProgram::Feasible;
	}
	lastResidual = newResidual;
	if(dx.norm()*alpha < xTol) {
	  if(verbose >= 1) cout<<"Converged on x"<<endl;
	  return LinearProgram::Feasible;
	}
      }
    }
  }
  if(verbose >= 1) {
    cout<<"Ran out of iterations, current x: "<<x<<endl;
    cout<<"Remaining gradient: "<<w<<endl;
  }
  return LinearProgram::Error;
}
