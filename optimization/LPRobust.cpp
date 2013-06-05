#include "LPRobust.h"
#include <iostream>
#include <errors.h>
using namespace std;
using namespace Optimization;


RobustLPSolver::RobustLPSolver()
  :verbose(0)
{
  Clear();
}

void RobustLPSolver::Clear()
{
  initialized=false;
}

LinearProgram::Result RobustLPSolver::Solve(const LinearProgram& lp)
{
  UpdateGLPK(lp);
  LinearProgram::Result res=SolveGLPK();
  return res;
}

LinearProgram::Result RobustLPSolver::Solve(const LinearProgram_Sparse& lp)
{
  glpk.Set(lp);
  initialized=true;
  return SolveGLPK();
}

LinearProgram::Result RobustLPSolver::Solve_NewObjective(const LinearProgram& lp)
{
  if(!initialized) UpdateGLPK(lp);
  else glpk.SetObjective(lp.c,lp.minimize);
  LinearProgram::Result res=SolveGLPK();
  return res;
}

LinearProgram::Result RobustLPSolver::Solve_NewObjective(const LinearProgram_Sparse& lp)
{
  if(!initialized) {
    glpk.Set(lp);
    initialized = true;
  }
  else glpk.SetObjective(lp.c,lp.minimize);
  return SolveGLPK();
}


void RobustLPSolver::UpdateGLPK(const LinearProgram& lp)
{
  glpk.Set(lp);
  initialized=true;
}

LinearProgram::Result RobustLPSolver::SolveGLPK()
{
  assert(GLPKInterface::Enabled());
  return glpk.Solve(xopt);
}
