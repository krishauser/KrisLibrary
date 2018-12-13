#ifndef MATH3D_BASIS_H
#define MATH3D_BASIS_H

#include "primitives.h"

/** @file math3d/basis.h
 * @ingroup Math3D
 * @brief Utilities to compute special orthogonal bases (that is, rotation
 * matrices).
 */

namespace Math3D {

  /** @addtogroup Math3D */
  /*@{*/

  /// Computes the perpendicular vector y s.t. x,y is a rotation from the 
  /// standard basis.
  inline void GetCanonicalBasis(const Vector2& x,Vector2&y) {
    y.setPerpendicular(x);
  } 
  /// Computes vectors y,z s.t. x,y,z is a (minimal) rotation from the 
  /// standard basis.
  inline void GetCanonicalBasis(const Vector3& x,Vector3&y,Vector3& z) {
    Real scale;
    if(FuzzyEquals(x.x,1.0)) scale = 0;
    else if(FuzzyEquals(x.x,-1.0)) {  //A complete flip of the basis
      y.set(0.0,-1.0,0.0);
      z.set(0.0,0.0,1.0);
      return;
    }
    else scale = (1.0-x.x)/(1.0-Sqr(x.x));
    y.x = -x.y;
    y.y = x.x + scale*Sqr(x.z);
    y.z = -scale*x.y*x.z;
    z.x = -x.z;
    z.y = -scale*x.y*x.z;
    z.z = x.x + scale*Sqr(x.y);
  }

  /*@}*/

} //namespace Math3D

#endif
