#include "interpolate.h"
#include "rotation.h"
#include <math/angle.h>

namespace Math3D {

void interpolateRotation(const Matrix2& a, const Matrix2& b, Real u, Matrix2& x )
{
  Real atheta = Atan2(a(1,0),a(0,0));
  Real btheta = Atan2(b(1,0),b(0,0));
  Real xtheta = AngleInterp(atheta,btheta,u);
  x.setRotate(xtheta);
}

void interpolateRotation(const Matrix3& a, const Matrix3& b, Real u, Matrix3& x)
{
  QuaternionRotation qa,qb,qx;
  qa.setMatrix(a);
  qb.setMatrix(b);
  qx.slerp(qa,qb,u);
  qx.inplaceNormalize();
  qx.getMatrix(x);
}

} //namespace Math3d
