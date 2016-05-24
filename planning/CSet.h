#ifndef PLANNING_CSET_H
#define PLANNING_CSET_H

#include <KrisLibrary/math/vector.h>
#if HAVE_BOOST
#include <boost/function.hpp>
#endif

namespace Optimization {
  class NonlinearProgram;
} //namespace Math
using namespace Math;
typedef Vector Config;

/** @ingroup MotionPlanning
 * @brief A subset of a CSpace, which establishes a constraint for a configuration
 * to be feasible. 
 */
class CSet
{
public:
  #if HAVE_BOOST
  typedef boost::function1<bool,const Config&> CPredicate;
  #else
  typedef bool (*CPredicate) (const Config&);
  #endif //HAVE_BOOST

  CSet();
  CSet(CPredicate f);
  virtual ~CSet () {}
  ///Returns the number of dimensions this accepts (-1) for all dimensions
  virtual int NumDimensions() const { return -1; }
  ///Returns true if this constraint is satisfied at configuration x
  virtual bool Contains(const Config& x);
  ///If possible, project x onto a nearby feasible configuration and return true.
  ///If not, return false.
  virtual bool Project(Config& x) { return Contains(x); }

  ///If true, the Sample() function is implemented
  virtual bool IsSampleable() const { return false; }

  ///If IsSampleable is true, draws a config at random from the set
  virtual void Sample(Config& x) {}

  ///If possible, give the obstacle as a feasible solution to the nonlinear program
  ///g(x) <= 0, h(x) = 0.
  virtual Optimization::NonlinearProgram* Numeric() { return NULL; }

  ///If true, the feasible set is convex with respect to the CSpace's interpolation function
  virtual bool IsConvex() const { return false; }

  ///If overloaded, return the CSpace distance from x to infeasibility.  Positive means
  ///feasible, 0 means boundary, negative means infeasible.
  virtual Real ObstacleDistance(const Config& x) { return -Inf; }

  CPredicate test;
};

#endif
