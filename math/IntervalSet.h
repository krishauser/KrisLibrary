#ifndef MATH_INTERVAL_SET_H
#define MATH_INTERVAL_SET_H

#include "Interval.h"
#include <vector>

namespace Math {

struct OpenIntervalSet : public std::vector<OpenInterval>
{
  typedef std::vector<OpenInterval> BaseT;
  typedef std::vector<ClosedInterval> ClosedBaseT;

  inline void SetFull() { resize(1); (*this)[0].setFull(); }
  inline void SetEmpty() { clear(); }
  void Union(const BaseT&);
  void Intersect(const BaseT&);
  void Subtract(const ClosedBaseT&);
  void Union(const OpenInterval&);
  void Intersect(const OpenInterval&);
  void Subtract(const ClosedInterval&);
};

struct ClosedIntervalSet : public std::vector<ClosedInterval>
{
  typedef std::vector<ClosedInterval> BaseT;
  typedef std::vector<OpenInterval> OpenBaseT;

  inline void SetFull() { resize(1); (*this)[0].setFull(); }
  inline void SetEmpty() { clear(); }
  void Union(const BaseT&);
  void Intersect(const BaseT&);
  void Subtract(const OpenBaseT&);
  void Union(const ClosedInterval&);
  void Intersect(const ClosedInterval&);
  void Subtract(const OpenInterval&);
};

} //namespace Math

#endif
