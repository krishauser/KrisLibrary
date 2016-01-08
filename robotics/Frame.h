#ifndef ROBOTICS_FRAME_H
#define ROBOTICS_FRAME_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
using namespace Math3D;

typedef RigidTransform2D Frame2D;
typedef RigidTransform Frame3D;

//returns cross product [w]x
inline Vector2 cross(Real w,const Vector2& x)
{
	return Vector2(x.y*w,-x.x*w);
}

//returns cross product matrix [w]
inline Matrix2 crossProductMatrix2(Real w)
{
	return Matrix2(Vector2(0,w),Vector2(-w,0));
}


//given the w (the 3,3 component of the homogeneous matrix) of frame b, this does the appropriate frame transform
inline Frame2D FrameMulWB(const Frame2D& a, const Frame2D& b, Real wb)
{
	Frame2D x;
	a.R.mul(b.t,x.t);
	x.t.madd(a.t,wb);
	x.R.mul(a.R,b.R);
	return x;
}

inline Frame2D FrameMulWB1(const Frame2D& a, const Frame2D& b)
{
	Frame2D x;
	a.R.mul(b.t,x.t);
	x.t += a.t;
	x.R.mul(a.R,b.R);
	return x;
}

inline Frame2D FrameMulWB0(const Frame2D& a, const Frame2D& b)
{
	Frame2D x;
	a.R.mul(b.t,x.t);
	x.R.mul(a.R,b.R);
	return x;
}

//wb is the w coordinate of b
inline Vector2 FrameMulWB(const Frame2D& a, const Vector2& b, Real wb)
{
	Vector2 x;
	a.R.mul(b,x);
	x.madd(b,wb);
	return x;
}

inline Vector2 FrameMulWB1(const Frame2D& a, const Vector2& b)
{
	Vector2 x;
	a.R.mul(b,x);
	x += a.t;
	return x;
}

inline Vector2 FrameMulWB0(const Frame2D& a, const Vector2& b)
{
	Vector2 x;
	a.R.mul(b,x);
	return x;
}



//3D versions

//given the w (the 4,4 component of the homogeneous matrix) of frame b, this does the appropriate frame transform
inline Frame3D FrameMulWB(const Frame3D& a, const Frame3D& b, Real wb)
{
	Frame3D x;
	a.R.mul(b.t,x.t);
	x.t.madd(a.t,wb);
	x.R.mul(a.R,b.R);
	return x;
}

inline Frame3D FrameMulWB1(const Frame3D& a, const Frame3D& b)
{
	Frame3D x;
	a.R.mul(b.t,x.t);
	x.t += a.t;
	x.R.mul(a.R,b.R);
	return x;
}

inline Frame3D FrameMulWB0(const Frame3D& a, const Frame3D& b)
{
	Frame3D x;
	a.R.mul(b.t,x.t);
	x.R.mul(a.R,b.R);
	return x;
}

//wb is the w coordinate of b
inline Vector3 FrameMulWB(const Frame3D& a, const Vector3& b, Real wb)
{
	Vector3 x;
	a.R.mul(b,x);
	x.madd(b,wb);
	return x;
}

inline Vector3 FrameMulWB1(const Frame3D& a, const Vector3& b)
{
	Vector3 x;
	a.R.mul(b,x);
	x += a.t;
	return x;
}

inline Vector3 FrameMulWB0(const Frame3D& a, const Vector3& b)
{
	Vector3 x;
	a.R.mul(b,x);
	return x;
}

#endif
