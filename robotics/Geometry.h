#ifndef ROBOTICS_GEOMETRY_H
#define ROBOTICS_GEOMETRY_H

#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math3d/Sphere3D.h>
#include <KrisLibrary/math3d/Circle3D.h>
using namespace Math3D;

/** @file Geometry.h @ingroup Kinematics
 * @brief Basic geometric utilities for rotations, intersections.
 */

/** @addtogroup Kinematics */
/**@{*/

/// Returns the RigidTransform that moves localpt onto pt, with 
/// the given rotation r.
template <class Rot>
void GetRotationAboutLocalPoint(const Vector3& localpt,const Vector3& pt,const Rot& r,RigidTransform& T);

/// Returns the RigidTransform that rotates by r about the point pt.
template <class Rot>
void GetRotationAboutPoint(const Vector3& pt,const Rot& r,RigidTransform& T);

/// Returns true if the solid ball a intersects the circle boundary b.
bool BallCircleCollision(const Sphere3D& a, const Circle3D& b);

/// Calculates the closest points on the boundaries of circles a,b.
void CircleCircleClosestPoints(const Circle3D& a, const Circle3D& b,
			       Vector3& pa, Vector3& pb);

/** @brief Calculates the intersection region of the balls a,b.
 *
 * Returns the dimension of the intersection <br>
 * 0 - no intersection <br>
 * 1 - point (region returned as the center of c) <br>
 * 2 - circle (region returned as c) <br>
 * 3 - ball (a is inside b) <br>
 * 4 - ball (b is inside a)
 */
int BallBallIntersection(const Sphere3D& a,const Sphere3D& b,Circle3D& c);

void CollisionSelfTest();

/**@}*/

#endif
