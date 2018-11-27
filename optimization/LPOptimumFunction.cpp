#include <KrisLibrary/Logger.h>
#include "LPOptimumFunction.h"
using namespace Optimization;
using namespace std;

LPOptimumFunction::LPOptimumFunction()
  :initialized(false),sparse(false)
{}

void LPOptimumFunction::PreEval(const Vector& x) 
{
  if(!initialized) {
    //first time
    InitLP(x);
    UpdateLP(x);
    
    if(sparse) glpk.Set(lps);
    else glpk.Set(lp);
    initialized = true;
  }
  else
    UpdateGLPK(x);
  
  solveResult = glpk.Solve(yopt);

  if(solveResult == LinearProgram::Error) {
    //could result from a problem with GLPK update -- initial basis may be
    //degenerate.  Reinitialize
    UpdateLP(x);
    if(sparse) glpk.Set(lps);
    else glpk.Set(lp);
    solveResult = glpk.Solve(yopt);
  }
}

void LPOptimumFunction::UpdateGLPK(const Vector& x)
{
  UpdateLP(x);
    
  if(sparse) glpk.Set(lps);
  else glpk.Set(lp);
}

Real LPOptimumFunction::Eval(const Vector& x)
{
  if(solveResult != LinearProgram::Feasible) return -Inf;
  if(sparse)
    return yopt.dot(lps.c);
  else
    return yopt.dot(lp.c);
}

void LPOptimumFunction::Gradient(const Vector& x,Vector& grad)
{
  grad.resize(x.n,Zero);
  if(solveResult != LinearProgram::Feasible) {
    return;
  }
  //look at status of each row/variable, build up constraint matrix
  if(sparse) {
    SparseMatrix Jc,Jai;
    Vector temp;
    if(LPJacobianC(x,Jc)) {
      Jc.mul(yopt,grad);
      if(!lps.minimize) grad.inplaceNegative();
      else if(!sparse && !lp.minimize) grad.inplaceNegative();
    }
    for(int i=0;i<lps.A.m;i++) {
      if(!glpk.GetRowBasic(i)) {
	//row is either at upper or lower bound
	Real multiplier = glpk.GetRowDual(i);
	//if at lower bound, what's the multiplier
	bool upper=false;
	if(lps.ConstraintType(i)==LinearConstraints::Fixed || lps.ConstraintType(i)==LinearConstraints::UpperBound) {
	  upper = true;
	}
	if(lps.ConstraintType(i)==LinearConstraints::Bounded) {
	  //test which one is closer
	  Real Ay = lps.A.dotRow(i,yopt);
	  if(Abs(Ay-lps.p(i)) < Abs(Ay-lps.q(i)))
	    upper=true;
	}
	if(upper) {
	  Assert(multiplier <= 0);
	  //use the upper bound
	  if(LPJacobianA_i(x,i,Jai)) {
	    Jai.mulTranspose(yopt,temp);
	    grad.madd(temp,-multiplier);
	  }
	  if(LPJacobianP_i(x,i,temp))
	    grad.madd(temp,-multiplier);
	}
	else {
	  Assert(multiplier >= 0);
	  if(LPJacobianA_i(x,i,Jai)) {
	    Jai.mulTranspose(yopt,temp);
	    grad.madd(temp,-multiplier);
	  }
	  if(LPJacobianQ_i(x,i,temp))
	    grad.madd(temp,-multiplier);
	}
      }
      else
	Assert(glpk.GetRowDual(i)==0.0);
    }
    for(int j=0;j<yopt.n;j++) {
      if(!glpk.GetVariableBasic(j)) {
	Real multiplier = glpk.GetVariableDual(j);
	if(Abs(yopt(j)-lps.u(j)) < Abs(yopt(j)-lps.l(j))) {
	  if(LPJacobianU_j(x,j,temp))
	    grad.madd(temp,-multiplier);
	}
	else {
	  if(LPJacobianL_j(x,j,temp))
	    grad.madd(temp,-multiplier);
	}
      }
      else 
	Assert(glpk.GetVariableDual(j)==0.0);
    }
  }
  else {
    Matrix Jc,Jai;
    Vector temp;
    if(LPJacobianC(x,Jc)) {
      Jc.mul(yopt,grad);
      if(!lp.minimize) grad.inplaceNegative();
    }
    /*
    LOG4CXX_INFO(KrisLibrary::logger(),"Row duals: ");
    for(int i=0;i<lp.A.m;i++)
      LOG4CXX_INFO(KrisLibrary::logger(),glpk.GetRowDual(i)<<" ");
    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Variable duals: ");
    for(int j=0;j<lp.A.n;j++)
      LOG4CXX_INFO(KrisLibrary::logger(),glpk.GetVariableDual(j)<<" ");
    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    */
    for(int i=0;i<lp.A.m;i++) {
      if(!glpk.GetRowBasic(i)) {
	//row is either at upper or lower bound
	Real multiplier = glpk.GetRowDual(i);
	//if at lower bound, what's the multiplier
	bool upper=false;
	if(lp.ConstraintType(i)==LinearConstraints::Fixed || lp.ConstraintType(i)==LinearConstraints::UpperBound) {
	  upper = true;
	}
	if(lp.ConstraintType(i)==LinearConstraints::Bounded) {
	  //test which one is closer
	  Real Ay = lp.A.dotRow(i,yopt);
	  if(Abs(Ay-lp.p(i)) < Abs(Ay-lp.q(i)))
	    upper=true;
	}
	if(upper) {
	  if(multiplier > Epsilon)
	    LOG4CXX_INFO(KrisLibrary::logger(),"Not understanding multipliers correctly -- upper bound "<<i<<" reached, multiplier = "<<multiplier);
	  Assert(multiplier <= Epsilon);
	  //use the upper bound
	  if(LPJacobianA_i(x,i,Jai)) {
	    Jai.mulTranspose(yopt,temp);
	    grad.madd(temp,-multiplier);
	  }
	  if(LPJacobianP_i(x,i,temp))
	    grad.madd(temp,-multiplier);
	}
	else {
	  if(multiplier < -Epsilon)
	    LOG4CXX_INFO(KrisLibrary::logger(),"Not understanding multipliers correctly -- lower bound "<<i<<" reached, multiplier = "<<multiplier);
	  Assert(multiplier >= -Epsilon);
	  if(LPJacobianA_i(x,i,Jai)) {
	    Jai.mulTranspose(yopt,temp);
	    grad.madd(temp,-multiplier);
	  }
	  if(LPJacobianQ_i(x,i,temp))
	    grad.madd(temp,-multiplier);
	}
      }
      else
	Assert(glpk.GetRowDual(i)==0.0);
    }
    for(int j=0;j<yopt.n;j++) {
      if(!glpk.GetVariableBasic(j)) {
	Real multiplier = glpk.GetVariableDual(j);
	if(Abs(yopt(j)-lp.u(j)) < Abs(yopt(j)-lp.l(j))) {
	  Assert(multiplier <= Epsilon);
	  if(LPJacobianU_j(x,j,temp))
	    grad.madd(temp,-multiplier);
	}
	else {
	  Assert(multiplier >= -Epsilon);
	  if(LPJacobianL_j(x,j,temp))
	    grad.madd(temp,-multiplier);
	}
      }
      else 
	Assert(glpk.GetVariableDual(j)==0.0);
    }
  }
}

Real LPOptimumFunction::Gradient_i(const Vector& x,int i)
{
  Vector grad;
  Gradient(x,grad);
  return grad(i);
}












LPOptimizerFunction::LPOptimizerFunction()
  :initialized(false),sparse(false)
{}

void LPOptimizerFunction::PreEval(const Vector& x) 
{
  if(!initialized) {
    //first time
    InitLP(x);
    UpdateLP(x);
    
    if(sparse) glpk.Set(lps);
    else glpk.Set(lp);
    initialized = true;
  }
  else
    UpdateGLPK(x);
  
  solveResult = glpk.Solve(yopt);
}

void LPOptimizerFunction::UpdateGLPK(const Vector& x)
{
  UpdateLP(x);
    
  if(sparse) glpk.Set(lps);
  else glpk.Set(lp);
}

void LPOptimizerFunction::Eval(const Vector& x,Vector& v)
{
  if(solveResult != LinearProgram::Feasible) {
    v.resize(NumDimensions(),0.0);
    return;
  }
  v = yopt;
}

int LPOptimizerFunction::NumDimensions() const
{
  if(sparse)
    return lps.A.m;
  else
    return lp.A.m;
}

void LPOptimizerFunction::Jacobian(const Vector& x,Matrix& J)
{
  J.resize(NumDimensions(),x.n,Zero);
  if(solveResult != LinearProgram::Feasible) {
    return;
  }
  //look at status of each row/variable, build up active constraint matrix
  if(sparse) {
    SparseMatrix Jai;
    SparseMatrix Aactive(yopt.n,yopt.n);
    Matrix Axderiv(yopt.n,x.n);
    int numActive = 0;
    Vector temp;
    for(int i=0;i<lps.A.m;i++) {
      if(!glpk.GetRowBasic(i)) {
	Assert(numActive < yopt.n);
	Aactive.rows[numActive] = lps.A.rows[i];

	Vector Axderivi(x.n,0.0);
	//row is either at upper or lower bound
	Real multiplier = glpk.GetRowDual(i);
	//if at lower bound, what's the multiplier
	bool upper=false;
	if(lps.ConstraintType(i)==LinearConstraints::Fixed || lps.ConstraintType(i)==LinearConstraints::UpperBound) {
	  upper = true;
	}
	if(lps.ConstraintType(i)==LinearConstraints::Bounded) {
	  //test which one is closer
	  Real Ay = lps.A.dotRow(i,yopt);
	  if(Abs(Ay-lps.p(i)) < Abs(Ay-lps.q(i)))
	    upper=true;
	}
	if(upper) {
	  Assert(multiplier <= 0);
	  //use the upper bound
	  if(LPJacobianA_i(x,i,Jai)) {
	    Jai.mulTranspose(yopt,temp);
	    Axderivi.madd(temp,multiplier);
	  }
	  if(LPJacobianP_i(x,i,temp))
	    Axderivi.madd(temp,-multiplier);
	}
	else {
	  Assert(multiplier >= 0);
	  if(LPJacobianA_i(x,i,Jai)) {
	    Jai.mulTranspose(yopt,temp);
	    Axderivi.madd(temp,multiplier);
	  }
	  if(LPJacobianQ_i(x,i,temp))
	    Axderivi.madd(temp,-multiplier);
	}
	Axderiv.copyRow(numActive,Axderivi);
	numActive++;
      }
      else
	Assert(glpk.GetRowDual(i)==0.0);
    }
    for(int j=0;j<yopt.n;j++) {
      if(!glpk.GetVariableBasic(j)) {
	Real multiplier = glpk.GetVariableDual(j);
	Vector Axderivi(x.n,0.0);
	if(Abs(yopt(j)-lps.u(j)) < Abs(yopt(j)-lps.l(j))) {
	  LPJacobianU_j(x,j,temp);
	  Axderivi.mul(temp,-multiplier);
	}
	else {
	  LPJacobianL_j(x,j,temp);
	  Axderivi.mul(temp,-multiplier);
	}
	Aactive(numActive,j)=1.0;
	Axderiv.copyRow(numActive,Axderivi);
	numActive++;
      }
      else 
	Assert(glpk.GetVariableDual(j)==0.0);
    }
    Assert(numActive == yopt.n);
    //solve J = Aactive^-1 * Axderiv
    FatalError("Not done yet, do sparse matrix inverse");
  }
  else {
    FatalError("Not done yet");
  }
}

void LPOptimizerFunction::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  Matrix J;
  Jacobian(x,J);
  J.getRowCopy(i,Ji);
}


