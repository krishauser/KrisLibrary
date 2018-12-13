#ifndef MATH3D_POLAR_H
#define MATH3D_POLAR_H

#include "primitives.h"

/** @file math3d/polar.h
 * @ingroup Math3D
 * @brief Conversions between rectangular and polar forms.

 * polar coords: (r,theta) : (x,y) = (r cos theta, r sin theta)
 *   r in [0,inf), theta in [0,2pi)
 *
 * cylindrical coords: (r,theta,z) : (x,y,z) = (r cos theta, r sin theta, z)
 *   r in [0,inf), theta in [0,2pi)
 *
 * spherical coords: (r,theta,phi) : (x,y,z) = (r cos theta cos phi, r sin theta cos phi, r sin phi)
 *   r in [0,inf), theta in [0,2pi), phi in [-pi, pi]
 */


namespace Math3D {

  /** @addtogroup Math3D */
  /*@{*/

//conversion routines
void PolarToRectangular(const Vector2& polar, Vector2& rect);
void RectangularToPolar(const Vector2& rect, Vector2& polar);

void SphericalToRectangular(const Vector3& sphere, Vector3& rect);
void RectangularToSpherical(const Vector3& rect, Vector3& sphere);

void CylindricalToRectangular(const Vector3& cyl, Vector3& rect);
void RectangularToCylindrical(const Vector3& rect, Vector3& cyl);

//derivatives of polar coordinates in rectangular coordinates
void PolarDR(const Vector2& polar, Vector2& drect);
void PolarDTheta(const Vector2& polar, Vector2& drect);

void SphericalDR(const Vector3& sphere, Vector3& drect);
void SphericalDTheta(const Vector3& sphere, Vector3& drect);
void SphericalDPhi(const Vector3& sphere, Vector3& drect);

void CylindricalDR(const Vector3& cyl, Vector3& drect);
void CylindricalDTheta(const Vector3& cyl, Vector3& drect);
void CylindricalDZ(const Vector3& cyl, Vector3& drect);

  /*@}*/

} //namespace Math3D

#endif
