#ifndef MATH_ANGLE_H
#define MATH_ANGLE_H

#include "math.h"

/** @file math/angle.h
 * @ingroup Math
 * @brief Planar (R2) rotation utilities.
 */

namespace Math {
  /** @addtogroup Math */
  /** @{*/

/// Normalizes an angle to [0,2pi)
Real AngleNormalize(Real a);
/// Returns the closest distance to get to a from b (range [-pi,pi])
Real AngleDiff(Real a, Real b);
/// Returns the CCW distance needed to get to a from b
Real AngleCCWDiff(Real a, Real b);
/// Interpolates between rotations a and b (u from 0 to 1)
Real AngleInterp(Real a, Real b, Real u);
inline bool IsValidAngle(Real a);

/** 
 * @brief A contiguous range of angles.
 *
 * Represents an interval [a,b] by an offset (c) and a range (d).
 * If a is clockwise from b, c=a, d=b-a (angular difference)
 * otherwise, c=b, d=a-b.
 * null set is represented as c=infty.
 */
struct AngleInterval
{
  void setPoint(Real a) { c=a; d=0; }
  /// sets the range from a CCW to b
  void setRange(Real a, Real b);
  /// sets the smallest range containing both a and b
  void setMinRange(Real a, Real b);
  inline void setEmpty() { c=Inf; d=0; }
  inline void setCircle() { c=0; d=TwoPi; }
  void setCompliment(const AngleInterval& i);
  /// Returns the smallest contiguous interval that contains the intersection
  /// (because it may have several components)
  void setIntersection(const AngleInterval& i1, const AngleInterval& i2);
  /// Returns the smallest contiguous interval that contains the union
  /// (because it may have several components)
  void setUnion(const AngleInterval& i1, const AngleInterval& i2);
  void setCosGreater(Real y);  ///< set the range [a,b] of x s.t. cos(x) >= y
  void setSinGreater(Real y);  ///< set the range [a,b] of x s.t. sin(x) >= y
  void setCosLess(Real y);  ///< set the range [a,b] of x s.t. cos(x) <= y
  void setSinLess(Real y);  ///< set the range [a,b] of x s.t. sin(x) <= y

  void inplaceCompliment();
  inline void inplaceIntersection(const AngleInterval& i) {
    AngleInterval j=*this;
    setIntersection(i,j);
  }
  inline void inplaceUnion(const AngleInterval& i) {
    AngleInterval j=*this;
    setUnion(i,j);
  }
  inline void inplaceShift(Real theta) { c = AngleNormalize(c+theta); }

  bool contains(Real a) const;
  bool contains(const AngleInterval& i) const;
  bool intersects(const AngleInterval& i) const;
  void normalize();
  Real clamp(Real a) const;

  inline bool isEmpty() const { return c==Inf; }
  inline bool isFull() const { return d>=TwoPi; }
  inline bool isPoint() const { return d==0; }

  Real c,d;
};

/// returns all solutions y to cos(y)=x in a,b
inline void Acos_All(Real x,Real& a,Real& b) { a=Acos(x); b=TwoPi-a; }
/// returns all solutions y to sin(y)=x in a,b
inline void Asin_All(Real x,Real& a,Real& b) { a=Asin(x); b=Pi-a; a=AngleNormalize(a); }
/// returns all solutions y to tan(y)=x in a,b
inline void Atan_All(Real x,Real& a,Real& b) { a=Atan(x); b=AngleNormalize(a+Pi); a=AngleNormalize(a); }

/// A "cos/sin" equation is a f(x) = cos(x) + b sin(x)
/// which can be expressed as f(x) = c*cos(x+d)
void TransformCosSin_Cos(Real a,Real b,Real& c,Real& d);
/// Same as above, but returns c*sin(x+d)
void TransformCosSin_Sin(Real a,Real b,Real& c,Real& d);

/// Solves a cos(x) + b sin(x) = c
/// returns at most 2 different values of x, or false if there is no solution.
bool SolveCosSinEquation(Real a,Real b,Real c,Real& t1,Real& t2);

/// Same as above, but a cos(x) + b sin(x) >= c
void SolveCosSinGreater(Real a,Real b,Real c,AngleInterval& i);

  /*@}*/
} //namespace Math

#endif
