#ifndef MATH3D_VECTOR_INLINES_H
#define MATH3D_VECTOR_INLINES_H

#include "inlinetypes.h"

namespace Math3D {

/*****************************vec2 operations*******************************/

//x = 0
inline void vec2_zero(vec2_t x)
{
	x[0]=x[1]=Zero;
}

//x = (a,b)
inline void vec2_make(vec2_t x, Real a, Real b)
{
	x[0] = a;
	x[1] = b;
}

//x = a
inline void vec2_equal(vec2_t x, const vec2_t a)
{
	x[0]=a[0];
	x[1]=a[1];
}

//x = a+b
inline void vec2_add(vec2_t x, const vec2_t a, const vec2_t b)
{
	x[0] = a[0]+b[0];
	x[1] = a[1]+b[1];
}

//x = a-b
inline void vec2_sub(vec2_t x, const vec2_t a, const vec2_t b)
{
	x[0] = a[0]-b[0];
	x[1] = a[1]-b[1];
}

//x = a*c
inline void vec2_multiply(vec2_t x, const vec2_t a, Real c)
{
	x[0] = a[0]*c;
	x[1] = a[1]*c;
}

//x += a*c
inline void vec2_madd(vec2_t x, const vec2_t a, Real c)
{
	x[0] += a[0]*c;
	x[1] += a[1]*c;
}

//x = -a
inline void vec2_negate(vec2_t x, const vec2_t a)
{
	x[0] = -a[0];
	x[1] = -a[1];
}

//returns dot(a,b)
inline Real vec2_dot(const vec2_t a, const vec2_t b)
{
	return a[0]*b[0] + a[1]*b[1];
}

//returns a cross b
inline float vec2_cross(const vec2_t a, const vec2_t b)
{
	return a[0]*b[1] - a[1]*b[0];
}

//returns |x|
inline Real vec2_length(const vec2_t x)
{
	return Sqrt(vec2_dot(x,x));
}

//normalizes x
inline void vec2_normalize(vec2_t x)
{
	Real l = Sqrt(vec2_dot(x,x));
	if(FuzzyZero(l)) return;
	vec2_multiply(x, x, Inv(l));
}

//x = orthogonal projection of a onto b = <a,b>/<b,b>*b
inline void vec2_project(vec2_t x, const vec2_t a, const vec2_t b)
{
	vec2_multiply(x, b, vec2_dot(a,b)/vec2_dot(b,b));
}


/*****************************vec3 operations*******************************/

//x = 0
inline void vec3_zero(vec3_t x)
{
	x[0]=x[1]=x[2]=Zero;
}

//x = (a,b,c)
inline void vec3_make(vec3_t x, Real a, Real b, Real c)
{
	x[0] = a;
	x[1] = b;
	x[2] = c;
}

//x = a
inline void vec3_equal(vec3_t x, const vec3_t a)
{
	x[0]=a[0];
	x[1]=a[1];
	x[2]=a[2];
}

//x = a+b
inline void vec3_add(vec3_t x, const vec3_t a, const vec3_t b)
{
	x[0] = a[0]+b[0];
	x[1] = a[1]+b[1];
	x[2] = a[2]+b[2];
}

//x = a-b
inline void vec3_sub(vec3_t x, const vec3_t a, const vec3_t b)
{
	x[0] = a[0]-b[0];
	x[1] = a[1]-b[1];
	x[2] = a[2]-b[2];
}

//x = a*c
inline void vec3_multiply(vec3_t x, const vec3_t a, Real c)
{
	x[0] = a[0]*c;
	x[1] = a[1]*c;
	x[2] = a[2]*c;
}

//x += a*c
inline void vec3_madd(vec3_t x, const vec3_t a, Real c)
{
	x[0] += a[0]*c;
	x[1] += a[1]*c;
	x[2] += a[2]*c;
}

//x = -a
inline void vec3_negate(vec3_t x, const vec3_t a)
{
	x[0] = -a[0];
	x[1] = -a[1];
	x[2] = -a[2];
}

//returns dot(a,b)
inline Real vec3_dot(const vec3_t a, const vec3_t b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

//x = a cross b
inline void vec3_cross(vec3_t x, const vec3_t a, const vec3_t b)
{
	x[0] = a[1]*b[2] - a[2]*b[1];
	x[1] = a[2]*b[0] - a[0]*b[2];
	x[2] = a[0]*b[1] - a[1]*b[0];
}

//returns |x|
inline Real vec3_length(const vec3_t x)
{
	return Sqrt(vec3_dot(x,x));
}

//normalizes x
inline void vec3_normalize(vec3_t x)
{
	Real l = Sqrt(vec3_dot(x,x));
	if(FuzzyZero(l)) return;
	vec3_multiply(x, x, Inv(l));
}

//x = orthogonal projection of a onto b = <a,b>/<b,b>*b
inline void vec3_project(vec3_t x, const vec3_t a, const vec3_t b)
{
	vec3_multiply(x, b, vec3_dot(a,b)/vec3_dot(b,b));
}


/*****************************vec4 operations*******************************/


//x = 0
inline void vec4_zero(vec4_t x)
{
	x[0]=x[1]=x[2]=x[3]=Zero;
}

//x = (a,b,c,d)
inline void vec4_make(vec4_t x, Real a, Real b, Real c, Real d)
{
	x[0]=a;
	x[1]=b;
	x[2]=c;
	x[3]=d;
}

//x = a
inline void vec4_equal(vec4_t x, const vec4_t a)
{
	x[0]=a[0];
	x[1]=a[1];
	x[2]=a[2];
	x[3]=a[3];
}

//x = a+b
inline void vec4_add(vec4_t x, const vec4_t a, const vec4_t b)
{
	x[0] = a[0]+b[0];
	x[1] = a[1]+b[1];
	x[2] = a[2]+b[2];
	x[3] = a[3]+b[3];
}

//x = a-b
inline void vec4_sub(vec4_t x, const vec4_t a, const vec4_t b)
{
	x[0] = a[0]-b[0];
	x[1] = a[1]-b[1];
	x[2] = a[2]-b[2];
	x[3] = a[3]-b[3];
}

//x = a*c
inline void vec4_multiply(vec4_t x, const vec4_t a, Real c)
{
	x[0] = a[0]*c;
	x[1] = a[1]*c;
	x[2] = a[2]*c;
	x[3] = a[3]*c;
}

//x += a*c
inline void vec4_madd(vec4_t x, const vec4_t a, Real c)
{
	x[0] += a[0]*c;
	x[1] += a[1]*c;
	x[2] += a[2]*c;
	x[3] += a[3]*c;
}

//x = -a
inline void vec4_negate(vec4_t x, const vec4_t a)
{
	x[0] = -a[0];
	x[1] = -a[1];
	x[2] = -a[2];
	x[3] = -a[3];
}

//returns dot(a,b)
inline Real vec4_dot(const vec4_t a, const vec4_t b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

//normalizes x
inline void vec4_normalize(vec4_t x)
{
	Real l = Sqrt(vec4_dot(x,x));
	if(FuzzyZero(l)) return;
	vec4_multiply(x, x, Inv(l));
}

//x = homogeneous orthogonal projection of a onto plane b:(n,d) = a - <a,b>/<n,n>*n
inline void vec4_plane_project(vec4_t x, const vec4_t a, const vec4_t b)
{
	vec4_t n;
	vec3_equal(n, b);
	n[3] = Zero;
	vec4_multiply(x, n, vec4_dot(a,b)/vec3_dot(b,b));
	vec4_sub(x, a, x);
}

} //namespace Math3D


#endif
