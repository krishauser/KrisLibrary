#ifndef GEOMETRY_MONOTONE_CHAIN_H
#define GEOMETRY_MONOTONE_CHAIN_H

#include "primitives.h"
#include <vector>

namespace Geometry {

  using namespace Math;
  using namespace Math3D;

  /** @ingroup Geometry
   * @brief A polyline with vertices ordered in nondecreasing order.
   *
   * Need v[i].x <= v[i+1].x
   * if equality holds, then v[i].y < v[i+1].y.
   */
struct XMonotoneChain
{
  Real eval(Real x) const;
  void upperEnvelope(const XMonotoneChain&);
  Real minimum(Real a,Real b,Real* x);
  bool isValid() const;
  static void SelfTest();

  std::vector<Vector2> v;
};

} //namespace Geometry

#endif
