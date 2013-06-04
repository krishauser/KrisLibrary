#include "QPRobust.h"
#include "QPActiveSetSolver.h"
#include "QPInteriorPoint.h"
#include "QuadProgPPInterface.h"
#include "QPOPTInterface.h"
using namespace Optimization;

RobustQPSolver::RobustQPSolver()
  :verbose(0)
{}

void RobustQPSolver::Clear()
{
  xopt.clear();
}

LinearProgram::Result RobustQPSolver::Solve(const QuadraticProgram& qp)
{
  QPOPTInterface solver;
  QPOPTInterface::Results result;
  solver.SetVerbose(verbose);
  LinearProgram::Result res = solver.Solve(qp,result);
  xopt = result.x;
  return res;
  /*
  QPActiveSetSolver solver(qp);
  solver.verbose = verbose;
  ConvergenceResult res=solver.Solve();
  if(res == ConvergenceX || res == ConvergenceF) {
    std::swap(solver.x,xopt);
    return LinearProgram::Feasible;
  }
  else return LinearProgram::Error;
  */
  /*
  QPInteriorPoint solver(qp);
  if(!solver.FindFeasiblePoint(solver.x0)) {
    return LinearProgram::Infeasible;
  }
  else if(!solver.Solve()) {
    return LinearProgram::Error;
  }
  else {
    xopt = solver.GetOptimum();
    return LinearProgram::Feasible;
  }
  */
  //QuadProgPPInterface solver;
  //return solver.Solve(qp,xopt);
}
