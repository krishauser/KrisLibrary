#ifndef OPTIMIZATION_QUADRATIC_PROGRAM_H
#define OPTIMIZATION_QUADRATIC_PROGRAM_H

#include <KrisLibrary/math/matrix.h>
#include "LinearProgram.h"

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief Quadratic program definition
 *
 * Defines the QP
 * min		0.5*x'*Pobj*x +qobj'*x <br>
 *  s.t.	q <= Ax <= p
 *              l <= x <= u
 */
struct QuadraticProgram  : public LinearConstraints
{
  void Print(std::ostream& out) const;
  void Resize(int m,int n);
  bool IsValid() const;
  Real Objective(const Vector& x) const;

  Matrix Pobj;
  Vector qobj;
};

} //namespace Optimization

#endif

