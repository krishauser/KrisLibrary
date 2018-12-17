#ifndef SPLINE_TIME_SEGMENTATION_H
#define SPLINE_TIME_SEGMENTATION_H

#include <KrisLibrary/math/math.h>
#include <vector>
#include <algorithm>

namespace Spline {

  using namespace Math;

///@brief Divides a real-valued range t[0],t[1],...,t[n-1] into segments
struct TimeSegmentation : public std::vector<Real>
{
  static int Map(const std::vector<Real>& timing,Real t) {
    if(timing.empty() || t < timing.front())    return -1;
    std::vector<Real>::const_iterator i=--std::upper_bound(timing.begin(),timing.end(),t);
    if(i == timing.end()) { return (int)timing.size()-1; }
    else { return int(i-timing.begin()); }
  }

  ///Same as above, but calculates a parameter u such that
  ///t=(1-u)*t[i]+u*t[i+1].
  static int Map(const std::vector<Real>& timing,Real t,Real& param) { 
    if(timing.empty() || t < timing.front())  { param=0.0; return -1; }
    std::vector<Real>::const_iterator i=--std::upper_bound(timing.begin(),timing.end(),t),n;
    if(i == timing.end() || i==--timing.end()) { param=1.0; return (int)timing.size()-1; }
    else { n=i; ++n; }
    param = (t-*i)/(*n-*i);
    return int(i-timing.begin());
  }

  ///Returns an index i such that t is in [t[i],t[i+1]).  If it is beyond
  ///the end of the times, returns n-1.  If it is before the first
  ///time, returns -1.
  int Map(Real t) const { 
    return TimeSegmentation::Map(*this,t);
  }

  ///Same as above, but calculates a parameter u such that
  ///t=(1-u)*t[i]+u*t[i+1].
  int Map(Real t,Real& param) const { 
    return TimeSegmentation::Map(*this,t,param);
  }

};

} //namespace Spline

#endif
