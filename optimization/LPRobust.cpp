#include "LPRobust.h"
#include <iostream>
#include <errors.h>
using namespace std;
using namespace Optimization;
enum { SOLVE_GLPK=RobustLPSolver::SOLVE_GLPK,
       SOLVE_LPSOLVE=RobustLPSolver::SOLVE_LPSOLVE,
       SOLVE_IP=RobustLPSolver::SOLVE_IP
};

void GetOrder(int start,int order[3])
{
  bool enabled[3];
  enabled[SOLVE_GLPK] = GLPKInterface::Enabled();
  enabled[SOLVE_LPSOLVE] = LPSolveInterface::Enabled();
  enabled[SOLVE_IP] = true;
  switch(start) {
  default:
  case 0:
    order[0] = SOLVE_GLPK; order[1] = SOLVE_LPSOLVE;   order[2] = SOLVE_IP;
    break;
  case 1:
    order[0] = SOLVE_LPSOLVE; order[1] = SOLVE_GLPK;   order[2] = SOLVE_IP;
    break;
  case 2:
    order[0] = SOLVE_IP; order[1] = SOLVE_GLPK;   order[2] = SOLVE_LPSOLVE;
    break;
  }
  for(int i=0;i<3;i++)
    if(!enabled[order[i]]) order[i]=-1;
}


RobustLPSolver::RobustLPSolver()
  :solveOrder(SOLVE_GLPK),verbose(0)
{
  Clear();
}

void RobustLPSolver::Clear()
{
  for(int i=0;i<3;i++) initialized[i]=false;
  solveOrder=SOLVE_GLPK;
}

LinearProgram::Result RobustLPSolver::Solve(const LinearProgram& lp)
{
  int order[3];
  GetOrder(solveOrder,order);
  LinearProgram::Result res;
  for(int i=0;i<3;i++) {
    if(order[i] == -1) res=LinearProgram::Error;
    else if(order[i] == SOLVE_GLPK) {
      UpdateGLPK(lp);
      res=SolveGLPK();
    }
    else if(order[i] == SOLVE_LPSOLVE) {
      UpdateLPSolve(lp);
      res=SolveLPSolve();
    }
    else if(order[i] == SOLVE_IP) {
      UpdateInteriorPoint(lp); 
      res=SolveInteriorPoint();
    }
    else {
      cerr<<"RobustLPSolver: Invalid solver type specified?"<<endl;
      Abort();
    }
    //res != LinearProgram::Infeasible
    if(res != LinearProgram::Error) {
      if(solveOrder != order[i]) {
	if(verbose) cout<<"RobustLPSolver: switching to solver "<<order[i]<<endl;
      }
      solveOrder=order[i];
      break;
    }
  }
  return res;
}

LinearProgram::Result RobustLPSolver::Solve(const LinearProgram_Sparse& lp)
{
  glpk.Set(lp);
  for(int i=0;i<3;i++) initialized[i]=false;
  initialized[SOLVE_GLPK]=true;
  return SolveGLPK();
}

LinearProgram::Result RobustLPSolver::Solve_NewObjective(const LinearProgram& lp)
{
  int order[3];
  GetOrder(solveOrder,order);
  LinearProgram::Result res;
  for(int i=0;i<3;i++) {
    if(order[i] == -1) res=LinearProgram::Error;
    else if(order[i] == SOLVE_GLPK) {
      if(!initialized[i]) UpdateGLPK(lp);
      else glpk.SetObjective(lp.c,lp.minimize);
      res=SolveGLPK();
    }
    else if(order[i] == SOLVE_LPSOLVE) {
      if(!initialized[i]) UpdateLPSolve(lp);
      else lp_solve.SetObjective(lp.c);
      UpdateLPSolve(lp);
      res=SolveLPSolve();
    }
    else if(order[i] == SOLVE_IP) {
      if(!initialized[i]) UpdateInteriorPoint(lp); 
      else lpi.SetObjective(lp.c);
      res=SolveInteriorPoint();
    }
    else {
      cerr<<"RobustLPSolver: Invalid solver type specified?"<<endl;
      Abort();
    }
    // && res != LinearProgram::Infeasible) {
    if(res != LinearProgram::Error) {
      if(solveOrder != order[i]) {
	if(verbose) cout<<"RobustLPSolver: switching to solver "<<order[i]<<endl;
      }
      solveOrder=order[i];
      break;
    }
  }
  return res;
}

LinearProgram::Result RobustLPSolver::Solve_NewObjective(const LinearProgram_Sparse& lp)
{
  LinearProgram::Result res;
  if(!initialized[SOLVE_GLPK]) {
    glpk.Set(lp);
    initialized[SOLVE_GLPK] = true;
  }
  else glpk.SetObjective(lp.c,lp.minimize);
  return SolveGLPK();
}


void RobustLPSolver::UpdateGLPK(const LinearProgram& lp)
{
  glpk.Set(lp);
  initialized[SOLVE_GLPK]=true;
}

void RobustLPSolver::UpdateLPSolve(const LinearProgram& lp)
{
  lp_solve.Set(lp);
  initialized[SOLVE_LPSOLVE]=true;
}

void RobustLPSolver::UpdateInteriorPoint(const LinearProgram& lp)
{
  if(!lpi.Set(lp)) {
    if(verbose) cout<<"Error with setting LP_InteriorPoint"<<endl;
    initialized[SOLVE_IP]=false;
  }
  else 
    initialized[SOLVE_IP]=true;
}

LinearProgram::Result RobustLPSolver::SolveLPSolve()
{
  assert(LPSolveInterface::Enabled());
  return lp_solve.Solve(xopt);
}

LinearProgram::Result RobustLPSolver::SolveGLPK()
{
  assert(GLPKInterface::Enabled());
  return glpk.Solve(xopt);
}

LinearProgram::Result RobustLPSolver::SolveInteriorPoint()
{
  if(!initialized[SOLVE_IP])
    return LinearProgram::Error;  
  lpi.solver.verbose = verbose-1;
  LP_InteriorPointSolver::Result res=lpi.Solve();
  switch(res) {
  case LP_InteriorPointSolver::Error:
    if(verbose) cout<<"Error solving LP_InteriorPoint"<<endl;
    return LinearProgram::Error;
  case LP_InteriorPointSolver::Infeasible:
    return LinearProgram::Infeasible;
  default:
    lpi.GetOptimum(xopt);
    return LinearProgram::Feasible;
  }
}

