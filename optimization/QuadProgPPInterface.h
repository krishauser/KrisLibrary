#ifndef QUADPROGPP_INTERFACE_H
#define QUADPROGPP_INTERFACE_H

#include "QuadraticProgram.h"

namespace Optimization {

/** @ingroup Optimization 
 * @brief An interface to the QuadProg++ quadratic programming solver.
 * Enable with the HAVE_QUADPROGPP=1 preprocessor define.
 *
 * Note: I tested this, and it seems to be pretty unreliable.
 */
struct QuadProgPPInterface
{
  LinearProgram::Result Solve(const QuadraticProgram& qp,Vector& xopt);
};

} //namespace Optimization

#endif
