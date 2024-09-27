#include <KrisLibrary/Logger.h>
#include "camera.h"
#include <math3d/rotation.h>
#include <iostream>
using namespace std;

namespace Camera {



CameraConventions::Orientation CameraConventions::OpenGL=XYZ; // = XYZ
CameraConventions::Orientation CameraConventions::OpenCV=XYnZ;   // = XYnZ
CameraConventions::Orientation CameraConventions::ROS=XYnZ;   // = XYnZ

void CameraConventions::getOrientationMatrix(Orientation o,Matrix3& mat)
{
  switch(o) {
  case XYZ:
    mat.setIdentity();
    break;
  case XYnZ:
    mat.setIdentity();
    mat(2,2) = -One;
    break;
  case XZY:
    mat.setZero();
    mat(0,0) = mat(1,2) = mat(2,1) = One;
    break;
  case XZnY:
    mat.setZero();
    mat(0,0) = mat(1,2) = One;
    mat(2,1) = -One;
    break;
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown orientation to CameraConventions::getOrientationMatrix");
    abort();
    break;
  }
}

void CameraConventions::orient(Orientation o,Matrix3& mat)
{
  Vector3 ry,rz;
  switch(o) {
  case XYZ:
    break;
  case XYnZ:
    mat.getRow3(rz);
    rz.inplaceNegative();
    mat.setRow3(rz);
    break;
  case XZY:
    mat.getRow2(ry);
    mat.getRow3(rz);
    mat.setRow3(ry);
    mat.setRow2(rz);
    break;
  case XZnY:
    mat.getRow2(ry);
    mat.getRow3(rz);
    rz.inplaceNegative();
    mat.setRow3(ry);
    mat.setRow2(rz);
    break;
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown orientation to CameraConventions::Orient");
    abort();
    break;
  }
}

void CameraConventions::unorient(Orientation o,Matrix3& mat)
{
  Vector3 ry,rz;
  switch(o) {
  case XYZ:
    break;
  case XYnZ:
    mat.getRow3(rz);
    rz.inplaceNegative();
    mat.setRow3(rz);
    break;
  case XZY:
    mat.getRow2(ry);
    mat.getRow3(rz);
    mat.setRow3(ry);
    mat.setRow2(rz);
    break;
  case XZnY:
    mat.getRow2(ry);
    mat.getRow3(rz);
    ry.inplaceNegative();
    mat.setRow3(ry);
    mat.setRow2(rz);
    break;
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown orientation to CameraConventions::Unorient");
    abort();
    break;
  }
}

RigidTransform CameraConventions::setFree(const Vector3& pos, const Vector3& rot, Orientation o)
{
  //rot is (pitch yaw roll) = (x y z)
  //world->camera transform is Rz(roll)Rx(pitch)Ry(yaw)
  //so inverse is or Ry(yaw)Rx(pitch)Rz(roll)
  RigidTransform xform;
  xform.t = pos;
  EulerAngleRotation r(rot.y,rot.x,rot.z);
  r.getMatrixYXZ(xform.R);
  orient(o,xform.R);
  return xform;
}

RigidTransform CameraConventions::setOrbit(const Vector3& rot, const Vector3& target, Real dist, Orientation o)
{
  RigidTransform xform;
  EulerAngleRotation r(rot.y,rot.x,rot.z);
  r.getMatrixYXZ(xform.R);
  orient(o,xform.R);

  Vector3 zb;
  xform.R.getCol3(zb);
  xform.t = target + dist*zb;
  return xform;
}

RigidTransform CameraConventions::setTarget(const Vector3& pos, const Vector3& tgt, const Vector3& up)
{
  RigidTransform xform;
  xform.t = pos;
  Vector3 z = tgt-pos;
  normalize(z);
  Vector3 x;
  x.setCross(z,up);
  normalize(x);
  Vector3 y;
  y.setCross(z,x);
  xform.R.set(x,y,z);
  return xform;
}

RigidTransform CameraConventions::setGLCameraMatrix(const Matrix4& m)
{
  Matrix4 minv;
  if(!minv.setInverse(m)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Camera modelview matrix not invertible");
    RigidTransform xform;
    xform.setIdentity();
    return xform;
  }
  RigidTransform xform;
  xform.set(minv);
  return xform;
}

void CameraConventions::getGLCameraMatrix(const RigidTransform& xform,Matrix4& mat)
{
  RigidTransform inv;
  inv.setInverse(xform);
  inv.get(mat);
}



void CameraConventions::getFree(const RigidTransform& xform, Vector3& pos, Vector3& rot, Orientation o)
{
  Matrix3 R=xform.R;
  unorient(o,R);
  EulerAngleRotation r;
  r.setMatrixYXZ(R);
  rot.set(r.y,r.x,r.z);
  pos = xform.t;
}

void CameraConventions::getOrbit(const RigidTransform& xform,Vector3& rot, Vector3& target, Real tgtdist, Orientation o)
{
  Matrix3 R=xform.R;
  unorient(o,R);
  EulerAngleRotation r;
  r.setMatrixYXZ(R);
  rot.set(r.y,r.x,r.z);

  Vector3 z;
  xform.R.getCol3(z);
  target = xform.t - z*tgtdist;
}

void CameraConventions::getTarget(const RigidTransform& xform, Vector3& pos, Vector3& tgt, Vector3& up, Real tgtDist) 
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


void CameraController_Free::toCameraPose(RigidTransform& xform) const
{
  xform = CameraConventions::setFree(pos,rot,ori);
}

void CameraController_Free::fromCameraPose(const RigidTransform& xform)
{
  CameraConventions::getFree(xform,pos,rot,ori);
}

void CameraController_Target::toCameraPose(RigidTransform& xform) const
{
  xform = CameraConventions::setTarget(pos,tgt,up);
}

void CameraController_Target::fromCameraPose(const RigidTransform& xform, Real tgtdist)
{
  CameraConventions::getTarget(xform,pos,tgt,up,tgtdist);
}

void CameraController_Orbit::toCameraPose(RigidTransform& xform) const
{
  xform = CameraConventions::setOrbit(rot,tgt,dist,ori);
}

void CameraController_Orbit::fromCameraPose(const RigidTransform& xform, Real tgtdist)
{
  CameraConventions::getOrbit(xform,rot,tgt,tgtdist,ori);
  dist = tgtdist;
}

} //namespace Camera