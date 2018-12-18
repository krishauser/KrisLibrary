#ifndef MATH3D_QUATERNION_INLINE_H
#define MATH3D_QUATERNION_INLINE_H

#include "vecinline.h"

namespace Math3D {

typedef vec4_t quat_t;

#define quat_zero vec4_zero
#define quat_equal vec4_equal
#define quat_multiply vec4_multiply
#define quat_add vec4_add
#define quat_dot vec4_dot
#define quat_normalize vec4_normalize

void quat_slerp(quat_t x, const quat_t a, const quat_t b, Real t);

}

#endif
