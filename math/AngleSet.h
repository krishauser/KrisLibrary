#ifndef MATH_ANGLE_SET_H
#define MATH_ANGLE_SET_H

#include "angle.h"
#include <vector>

namespace Math {

/** @ingroup Math
 * @brief A set of AngleIntervals.
 */
struct AngleSet : public std::vector<AngleInterval>
{
  typedef std::vector<AngleInterval> BaseT;

  inline void SetCircle() { resize(1); (*this)[0].setCircle(); }
  inline void SetEmpty() { clear(); }
  void Union(const BaseT&);
  void Intersect(const BaseT&);
  void Subtract(const BaseT&);
  void Union(const AngleInterval&);
  void Intersect(const AngleInterval&);
  void Subtract(const AngleInterval&);
};

} //namespace Math

#endif
