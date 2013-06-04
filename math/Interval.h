#ifndef MATH_INTERVAL_H
#define MATH_INTERVAL_H

#include "math.h"
#include "infnan.h"

namespace Math {

struct Interval;
struct OpenInterval;
struct ClosedInterval;

/** @ingroup Math
 * @brief A generic interval {a,b}, where { and } can be either (,[ or ),].
 */
struct Interval
{
  inline Interval();
  inline Interval(Real a, Real b);
  inline void set(Real a, Real b);
  inline void setFull();
  inline void setEmpty();
  inline bool isFull() const;  

  Real a,b;
};

/** @ingroup Math
 * @brief An open interval (a,b)
 */
struct OpenInterval : public Interval
{
  inline OpenInterval();
  inline OpenInterval(Real a, Real b);
  inline void setIntersection(const OpenInterval& i, const OpenInterval& j);
  inline void setUnion(const OpenInterval& i, const OpenInterval& j);
  inline void expand(Real x);
  inline bool isEmpty() const;
  inline bool contains(Real x) const;
  inline bool contains(const OpenInterval&) const;
  inline bool contains(const ClosedInterval&) const;
  inline bool intersects(const OpenInterval&) const;
  inline bool intersects(const ClosedInterval&) const;
};

/** @ingroup Math
 * @brief A closed interval [a,b]
 */
struct ClosedInterval : public Interval
{
  inline ClosedInterval();
  inline ClosedInterval(Real a, Real b);
  inline void setIntersection(const ClosedInterval& i, const ClosedInterval& j);
  inline void setUnion(const ClosedInterval& i, const ClosedInterval& j);
  inline void expand(Real x);
  inline bool isEmpty() const;
  inline bool contains(Real x) const;
  inline bool contains(const Interval&) const;
  inline bool intersects(const ClosedInterval&) const;
  inline bool intersects(const OpenInterval&) const;
};

Interval::Interval()
{}

Interval::Interval(Real _a, Real _b)
:a(_a),b(_b)
{}

void Interval::set(Real _a, Real _b)
{
	a = _a;
	b = _b;
}

void Interval::setFull()
{
	a = -Inf;
	b = Inf;
}

void Interval::setEmpty()
{
	a = Inf;
	b = -Inf;
}

bool Interval::isFull() const
{
  return (IsInf(a)==-1 && IsInf(b)==1);
}



OpenInterval::OpenInterval()
{}

OpenInterval::OpenInterval(Real _a, Real _b)
:Interval(_a,_b)
{}

void OpenInterval::setIntersection(const OpenInterval& i, const OpenInterval& j) 
{
	a = Max(i.a,j.a);
	b = Min(i.b,j.b);
}

void OpenInterval::setUnion(const OpenInterval& i, const OpenInterval& j)
{
	a = Min(i.a,j.a);
	b = Min(i.b,j.b);
}

void OpenInterval::expand(Real x)
{
	if(x < a) a = x;
	else if (x > b) b = x;
}

bool OpenInterval::isEmpty() const
{
  return a >= b;
}

bool OpenInterval::contains(Real x) const
{
  //subtle point -- if a or b is +/- infinity, and x is +/- infinity, 
  //this should return true
  return (x > a && x < b) || isFull();
}

bool OpenInterval::contains(const OpenInterval& i) const
{
  //ANSI C requires inf==inf and -inf==-inf to return true
  return a <= i.a && i.b <= b;
}

bool OpenInterval::contains(const ClosedInterval& i) const
{
  if(IsInf(a) == -1) {
    if(IsInf(b)) return true;
    return (i.b < b);
  }
  else if(IsInf(b)) return (a < i.a);
  else return (a < i.a && i.b < b);
}

bool OpenInterval::intersects(const OpenInterval& i) const
{
  if(IsInf(a) == -1) {
    if(IsInf(b) == 1) return !i.isEmpty();
    return (i.a < b);
  }
  else if(IsInf(b)) return (a < i.b);
  else return (a < i.b && i.a < b);
}



ClosedInterval::ClosedInterval()
{}

ClosedInterval::ClosedInterval(Real _a, Real _b)
:Interval(_a,_b)
{}

void ClosedInterval::setIntersection(const ClosedInterval& i, const ClosedInterval& j)
{
	a = Max(i.a,j.a);
	b = Min(i.b,j.b);
}

void ClosedInterval::setUnion(const ClosedInterval& i, const ClosedInterval& j)
{
	a = Min(i.a,j.a);
	b = Min(i.b,j.b);
}

void ClosedInterval::expand(Real x)
{
	if(x < a) a = x;
	else if (x > b) b = x;
}

bool ClosedInterval::isEmpty() const
{
	return a > b;
}

bool ClosedInterval::contains(Real x) const
{
	return x >= a && x <= b;
}

bool ClosedInterval::contains(const Interval& i) const
{
	return a <= i.a && i.b <= b;
}

bool ClosedInterval::intersects(const ClosedInterval& i) const
{
	return a <= i.b && i.a <= b;
}

bool ClosedInterval::intersects(const OpenInterval& i) const
{
  if(IsInf(a) == -1) {
    if(IsInf(b) == 1) return !i.isEmpty();
    return (i.a < b);
  }
  else if(IsInf(b) == 1)  return (a < i.b);
  else return (a < i.b && i.a < b);
}

}

#endif
