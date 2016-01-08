#ifndef ANGLE_BRACKET_H
#define ANGLE_BRACKET_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math/angle.h>
using namespace Math3D;
using namespace std;

/** @file AngleBracket.h
 * @ingroup Kinematics
 * The angle bracket of a joint (centered at the origin)
 * is the range of angles that can bring the local point
 * p into the given geometrical object (often, the workspace
 * of the rest of the kinematic chain used in RLG).
 *
 * The format of these functions is AngleBracket_[n]D_[obj]
 * where n is the dimension, and obj is the object.
 * The arguments are (p, {object params}).
 * Return value is the interval of valid angles.
 * (inf,theta) is returned if no valid angles, theta is closest possible angle
 */

/**@addtogroup Kinematics*/
/*@{*/

///Simplest form, assumes p and the disk D lie on the x axis
///(Actually in 2d)
///Point is (p,0).  Disk's center is (c,0), radius is r.
AngleInterval AngleBracket_1D_Disk(Real p, Real c, Real r);

///Full 2D version
AngleInterval AngleBracket_2D_Disk(const Vector2& p, const Vector2& c, Real r);

///3D version
///p rotates around the z axis.  The ball B is centered at c, with radius r.
AngleInterval AngleBracket_3D_Ball(const Vector3& p, const Vector3& c, Real r);

///same as above, but axis w is arbitrary
AngleInterval AngleBracket_3D_Ball(const Vector3& p, const Vector3& w, const Vector3& c, Real r);

///same as above, but axis can be x,y,or z axis (x=0,y=1,z=2)
AngleInterval AngleBracket_3D_Ball(const Vector3& p, int axis, const Vector3& c, Real r);

/*@}*/

#endif
