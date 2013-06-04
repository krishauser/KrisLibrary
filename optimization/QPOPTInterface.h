#ifndef QPOPT_INTERFACE_H
#define QPOPT_INTERFACE_H

#include "QuadraticProgram.h"
#include <vector>

namespace Optimization {

/** @ingroup Optimization 
 * @brief An interface to the QPOPT quadratic programming solver.
 * Enable with the HAVE_QPOPT=1 preprocessor define.
 */
struct QPOPTInterface
{
  struct Results {
    Vector x;   //solution vector
    Vector Ax;  //constraint matrix times x
    Vector clambda;  //constraint lagrange multipliers
    std::vector<long int> istate;  //state of constraints
    std::vector<long int> iworkspace;  //workspace
    std::vector<double> rworkspace;  //workspace
  };

  void SetVerbose(int verbose);
  void SetWarmStart(bool warmStart);
  LinearProgram::Result Solve(const QuadraticProgram& qp,Results& res);
};

} //namespace Optimization

#endif
