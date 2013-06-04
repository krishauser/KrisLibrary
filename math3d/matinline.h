#ifndef MATRIX_INLINES_H
#define MATRIX_INLINES_H

#include "vecinline.h"

namespace Math3D {

/*****************************matrix operations*******************************/

//x = 0
inline void matrix4_zero(matrix4_t x)
{
	int i,j;
	for(i=0; i<4; i++)
		for(j=0; j<4; j++)
			x[i][j] = Zero;
}

//x = I
inline void matrix4_identity(matrix4_t x)
{
	matrix4_zero(x);
	x[0][0] = x[1][1] = x[2][2] = x[3][3] = One;
}

//x = ( xb yb zb trans)
inline void matrix4_from_basis(matrix4_t x, const vec4_t xbasis, const vec4_t ybasis, const vec4_t zbasis, const vec4_t trans)
{
	vec4_equal(x[0], xbasis);
	vec4_equal(x[1], ybasis);
	vec4_equal(x[2], zbasis);
	vec4_equal(x[3], trans);
}

//reverse of above
inline void matrix4_to_basis(const matrix4_t x, vec4_t xbasis, vec4_t ybasis, vec4_t zbasis, vec4_t trans)
{
	vec4_equal(xbasis, x[0]);
	vec4_equal(ybasis, x[1]);
	vec4_equal(zbasis, x[2]);
	vec4_equal(trans, x[3]);
}

//x = a
inline void matrix4_equal(matrix4_t x, const matrix4_t a)
{
	int i,j;
	for(i=0; i<4; i++)
		for(j=0; j<4; j++)
			x[i][j] = a[i][j];
}

//x = a+b
inline void matrix4_add(matrix4_t x, const matrix4_t a, const matrix4_t b)
{
	int i,j;
	for(i=0; i<4; i++)
		for(j=0; j<4; j++)
			x[i][j] = a[i][j] + b[i][j];
}

//x = a-b
inline void matrix4_sub(matrix4_t x, const matrix4_t a, const matrix4_t b)
{
	int i,j;
	for(i=0; i<4; i++)
		for(j=0; j<4; j++)
			x[i][j] = a[i][j] - b[i][j];
}

//x = a*b
inline void matrix4_multiply(matrix4_t x, const matrix4_t a, const matrix4_t b)
{
	int i,j,k;

    matrix4_t temp;
    matrix4_zero(temp);

    for(i=0; i<4; i++ ) 
        for(j=0; j<4; j++ ) 
            for(k=0; k<4; k++ ) 
                temp[i][j] += b[i][k] * a[k][j];

	matrix4_equal(x, temp);
}

//x = a^t
inline void matrix4_transpose(matrix4_t x, const matrix4_t a)
{
	int i,j;
	for(i=0; i<4; i++)
		for(j=0; j<4; j++)
			x[i][j] = a[j][i];
}

//x = A*v
inline void matrix4_vec3_multiply(vec3_t x, const matrix4_t a, const vec3_t v)
{
	x[0] = a[0][0]*v[0] + a[1][0]*v[1] + a[2][0]*v[2] + a[3][0];
	x[1] = a[0][1]*v[0] + a[1][1]*v[1] + a[2][1]*v[2] + a[3][1];
	x[2] = a[0][2]*v[0] + a[1][2]*v[1] + a[2][2]*v[2] + a[3][2];
}

//x = A*v (the 3*3 part)
inline void matrix4_normal_multiply(vec3_t x, const matrix4_t a, const vec3_t v)
{
	x[0] = a[0][0]*v[0] + a[1][0]*v[1] + a[2][0]*v[2];
	x[1] = a[0][1]*v[0] + a[1][1]*v[1] + a[2][1]*v[2];
	x[2] = a[0][2]*v[0] + a[1][2]*v[1] + a[2][2]*v[2];
}

//x = At*v
inline void matrix4_vec3_multiply_transpose(vec3_t x, const matrix4_t a, const vec3_t v)
{
  x[0] = a[0][0]*v[0] + a[0][1]*v[1] + a[0][2]*v[2];
  x[1] = a[1][0]*v[0] + a[1][1]*v[1] + a[1][2]*v[2];
  x[2] = a[2][0]*v[0] + a[2][1]*v[1] + a[2][2]*v[2];
}

#define matrix4_normal_multiply_transpose matrix4_vec3_multiply_transpose

//x = A*v
inline void matrix4_vec4_multiply(vec4_t x, const matrix4_t a, const vec4_t v)
{
	x[0] = a[0][0]*v[0] + a[1][0]*v[1] + a[2][0]*v[2] + a[3][0]*v[3];
	x[1] = a[0][1]*v[0] + a[1][1]*v[1] + a[2][1]*v[2] + a[3][1]*v[3];
	x[2] = a[0][2]*v[0] + a[1][2]*v[1] + a[2][2]*v[2] + a[3][2]*v[3];
	x[2] = a[0][3]*v[0] + a[1][3]*v[1] + a[2][3]*v[2] + a[3][3]*v[3];
}


//scales the rotation part of the matrix by (s,s,s)
inline void matrix4_scale3(matrix4_t x, Real scale)
{
	int i,j;

	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			x[i][j] *= scale;
}

//scales the rotation part of the matrix by a 3 vector
inline void matrix4_scale3(matrix4_t x, const vec3_t v_scale)
{
	int i,j;

	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			x[i][j] *= v_scale[i];
}

inline Real matrix3_determinant(const matrix4_t a)
{
	return a[0][0] * (a[1][1]*a[2][2] - a[2][1]*a[1][2])
		+ a[1][0] * (a[2][1]*a[0][2] - a[2][2]*a[0][1])
		+ a[2][0] * (a[0][1]*a[1][2] - a[0][2]*a[1][1]);
}

//x = matrix such that x*w = (v x w) for all w
inline void matrix4_cross_product(matrix4_t x, const vec3_t v)
{
	matrix4_zero(x);
	x[1][0] = -v[2];
	x[2][1] = -v[0];
	x[2][0] = v[1];

	x[0][1] = v[2];
	x[1][2] = v[0];
	x[0][2] = -v[1];
	x[3][3] = 1.0;
}

inline void matrix4_uncross(vec3_t x, const matrix4_t a)
{
	x[0] = a[1][2];
	x[1] = a[2][0];
	x[2] = a[0][1];
}


void matrix4_invert( matrix4_t x, const matrix4_t a );


void matrix4_rotation(matrix4_t x, const vec3_t r);
void matrix4_rotation_axis(vec3_t r, const matrix4_t R);

inline void matrix4_rotate_x(matrix4_t x, Real fRads)
{
	matrix4_identity(x);
	Real cx = Cos(fRads);
	Real sx = Sin(fRads);
	x[1][1] = cx;
	x[2][2] = cx;
	x[1][2] = sx;
	x[2][1] = -sx;
}

inline void matrix4_rotate_y(matrix4_t x, Real fRads)
{
	matrix4_identity(x);
	Real cy = Cos(fRads);
	Real sy = Sin(fRads);
	x[0][0] = cy;
	x[2][2] = cy;
	x[0][2] = -sy;
	x[2][0] = sy;
}

inline void matrix4_rotate_z(matrix4_t x, Real fRads)
{
	matrix4_identity(x);
	Real cz = Cos(fRads);
	Real sz = Sin(fRads);
	x[0][0] = cz;
	x[1][1] = cz;
	x[0][1] = sz;
	x[1][0] = -sz;
}


//x = matrix such that x*w = proj (w on v) for all w
//= v*v^t / <v,v>
inline void matrix4_projection(matrix4_t x, const vec3_t v)
{
	vec3_t temp;
	matrix4_identity(x);
	vec3_make(temp, One,Zero,Zero);
	vec3_project(x[0], temp, v);
	vec3_make(temp, Zero,One,Zero);
	vec3_project(x[1], temp, v);
	vec3_make(temp, Zero,Zero,One);
	vec3_project(x[2], temp, v);
}

//x = matrix such that x*w = proj (w on plane defined by p) for all w
//= I - v*p^t / v^t*p
inline void matrix4_plane_projection(matrix4_t x, const vec4_t p, vec3_t v)
{
	Real c = - One / vec3_dot(v,p);
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
		{
			if(i == 3)
				x[i][j] = Zero;
			else
				x[i][j] = v[i]*p[j] * c;

			if(i == j)
				x[i][j] += One;
		}
}

//if a is a rigid body transformation (a rotation + a translation)
//x = a^-1
//( R(r)*T(t) )^-1 = T(-t)*R(r)^t = R(r)^t * T(R(r)^t(-t))
inline void matrix4_rigid_body_inverse(matrix4_t x, const matrix4_t a)
{
	matrix4_transpose(x, a);
	x[0][3] = x[1][3] = x[2][3] = Zero;
	vec3_t r;
	matrix4_normal_multiply(r, x, a[3]);
	vec3_negate(x[3], r);
}

} //namespace Math

#endif
