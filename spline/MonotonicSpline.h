#ifndef SPLINE_MONOTONIC_SPLINE_H
#define SPLINE_MONOTONIC_SPLINE_H

#include "BSpline.h"

namespace Spline {

/**@brief A b-spline based monotonic spline.
 *
 * Forward map: t(u), u in [0,n-1]  where n=t.size()
 * Inverse map: u(t), u in [t[0],t[n-1]]
 */
struct MonotonicSpline 
{
  bool IsValid() const;
  Real UtoT(Real u) const;   //t(u)
  Real TDeriv(Real u) const;  //t'(u)
  Real TDeriv2(Real u) const;  //t''(u)
  Real TtoU(Real time) const;   //u(t)
  Real UDeriv(Real time) const;  //u'(t)
  Real UDeriv2(Real time) const;  //u''(t)

  BSplineBasis basis;
  std::vector<Real> t;
};

} //namespace Spline

#endif


