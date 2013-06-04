#ifndef MATH3D_ROTATION_FIT_H
#define MATH3D_ROTATION_FIT_H

#include "AABB3D.h"
#include <vector>

/** @file math3d/rotationfit.h
 * @ingroup Math3D
 * @brief Least-squares fitting for rotations (and rigid body transformations)
 */

namespace Math3D {
  /** @addtogroup Math3D */
  /*@{*/

/** @brief Calculates the least squares rotation fit
 * min_R sum||R*a[i]-b[i]||^2.  Returns the sum-of-squared errors
 */
Real RotationFit(const std::vector<Vector3>& a,const std::vector<Vector3>& b,Matrix3& R);

/** @brief Calculates the least squares rotation angle
 * min_theta sum||R(theta,z)*a[i]-b[i]||^2.
 */
void AxisRotationFit(const std::vector<Vector3>& a,const std::vector<Vector3>& b,const Vector3& z,Real& theta);

/** @brief Calculate the least squares rotation fit 
 * min_R,t sum||R*a[i]+t-b[i]||^2.  Returns the sum-of-squared errors.
 *
 * Here, a and b provide temporary storage, their contents are modified
 */
Real TransformFit(std::vector<Vector3>& a,std::vector<Vector3>& b,
		  Matrix3& R,Vector3& t);

/** @brief Solve the mixed point/vector fitting problem
 * min sum_j wj||R*aj+t-bj||^2 + sum_k vk||Rc-d||^2
 *
 * R is a rotation, t is a translation, a and b are points, c and d are vectors, and w and v are optional weights.
 */
Real WeightedTransformFit(const std::vector<Point3D>& a,const std::vector<Point3D>& b,
			  const std::vector<Real>& w,
			  const std::vector<Vector3>& c,const std::vector<Vector3>& d,
			  const std::vector<Real>& v,
			  Matrix3& R,Vector3& t);

/** @brief Calculate the translations and rotations that match a,b in an
 * axis-aligned frame.  The cov vector stores the covariance in each axis.
 */
Real FitFrames(const std::vector<Vector3>& a,const std::vector<Vector3>& b,
	       RigidTransform& Ta,RigidTransform& Tb,Vector3& cov);

} //namespace Math3D

#endif
