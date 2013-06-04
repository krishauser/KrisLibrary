#include "transform.h"
#include <math3d/rotation.h>


void SetOrbitTransform(const Vector3& rot, const Vector3& target, float dist, RigidTransform& xform)
{
  //rot is (pitch yaw roll) = (x y z)
  //proper transform is Rz(roll)Rx(pitch)Ry(yaw)
  EulerAngleRotation r(rot.z,rot.y,rot.x);
  r.getMatrixZXY(xform.R);

  Vector3 zb;
  xform.R.getCol3(zb);
  xform.t = target + dist*zb;
}

void SetFreeTransform(const Vector3& pos, const Vector3& rot, RigidTransform& xform)
{
  xform.t = pos;
  EulerAngleRotation r(rot.z,rot.y,rot.x);
  r.getMatrixZXY(xform.R);
}

void SetTargetTransform(const Vector3& pos, const Vector3& tgt, const Vector3& up, RigidTransform& xform)
{
  xform.t = pos;
  Vector3 z = tgt-pos;
  normalize(z);
  Vector3 x;
  x.setCross(z,up);
  normalize(x);
  Vector3 y;
  y.setCross(z,x);
  xform.R.set(x,y,z);
}

void GetOrbitTransform(const RigidTransform& xform, Vector3& rot, Vector3& target, float dist)
{
  EulerAngleRotation r;
  r.setMatrixZXY(xform.R);
  rot.set(r.z,r.y,r.x);

  Vector3 z;
  xform.R.getCol3(z);
  target = xform.t + z*dist;
}

void GetFreeTransform(const RigidTransform& xform, Vector3& pos, Vector3& rot)
{
  EulerAngleRotation r;
  r.setMatrixZXY(xform.R);
  rot.set(r.z,r.y,r.x);
  pos = xform.t;
}

void GetTargetTransform(const RigidTransform& xform, Vector3& pos, Vector3& tgt, Vector3& up, float tgtDist)
{
  //y dir
  xform.R.getCol2(up);

  //position
  pos = xform.t;

  //z dir
  Vector3 z;
  xform.R.getCol3(z);

  tgt = pos + z*tgtDist;
}
