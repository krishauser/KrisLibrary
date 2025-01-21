#include <KrisLibrary/Logger.h>
#include "camera.h"
#include <math3d/rotation.h>
#include <iostream>
using namespace std;

namespace Camera {



CameraConventions::CamOrientation CameraConventions::OpenGL=XYnZ;
CameraConventions::CamOrientation CameraConventions::OpenCV=XnYZ;
CameraConventions::CamOrientation CameraConventions::ROS=XnYZ;

Vector3 CameraConventions::forward(CamOrientation o)
{
  switch(o) {
    case XYnZ: return Vector3(0,0,-1);
    case XnYZ: return Vector3(0,0,1);
    default:
      LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown orientation to CameraConventions::forward");
      abort();
      return Vector3(0.0);
  }
}

Vector3 CameraConventions::backward(CamOrientation o)
{
  return -forward(o);
}

Vector3 CameraConventions::right(CamOrientation o)
{
  return Vector3(1,0,0);
}

Vector3 CameraConventions::left(CamOrientation o)
{
  return Vector3(-1,0,0);
}

Vector3 CameraConventions::up(CamOrientation o)
{
  switch(o) {
    case XYnZ: return Vector3(0,1,0);
    case XnYZ: return Vector3(0,-1,0);
    default:
      LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown orientation to CameraConventions::forward");
      abort();
      return Vector3(0.0);
  }
}

Vector3 CameraConventions::down(CamOrientation o)
{
  return -up(o);
}

Matrix3 CameraConventions::orientationMatrix(CamOrientation osrc,CamOrientation otgt)
{
  Matrix3 mat;
  if(osrc == otgt) {
    mat.setIdentity();
    return mat;
  }
  mat.setIdentity();
  mat(1,1) = -1;
  mat(2,2) = -1;
  return mat;
}

Matrix3 CameraConventions::zeroPose(CamOrientation o, WorldOrientation wo)
{
  Matrix3 R;
  R.setZero();
  if(wo == Zup) {
    if(o == XnYZ) {
      R(0,0) = 1;
      R(1,2) = 1;
      R(2,1) = -1;
    }
    else {
      R(0,0) = 1;
      R(1,2) = -1;
      R(2,1) = 1;
    }
  }
  else {
    if(o == XnYZ) {
      R(0,0) = 1;
      R(1,1) = -1;
      R(2,2) = -1;
    }
    else {
      R(0,0) = 1;
      R(1,1) = 1;
      R(2,2) = 1;
    }
  }
  return R;
}

RigidTransform CameraConventions::setFree(const Vector3& pos, const Vector3& rot, CamOrientation o, WorldOrientation wo)
{
  //rot is (pitch yaw roll) = (x y z)
  //world->camera transform is Rz(roll)Rx(pitch)Ry(yaw)
  //so inverse is or Ry(yaw)Rx(pitch)Rz(roll)
  Matrix3 zero = zeroPose(o,wo);
  RigidTransform xform;
  xform.t = pos;
  EulerAngleRotation r(rot.y,rot.x,rot.z);
  if(o == XnYZ) {
    r.y = -r.y;
    r.z = -r.z;
  }
  r.getMatrixYXZ(xform.R);
  xform.R = zero*xform.R;
  return xform;
}

RigidTransform CameraConventions::setOrbit(const Vector3& rot, const Vector3& target, Real dist, CamOrientation o, WorldOrientation wo)
{
  Matrix3 zero = zeroPose(o,wo);
  RigidTransform xform;
  EulerAngleRotation r(rot.y,rot.x,rot.z);
  if(o == XnYZ) {
    r.y = -r.y;
    r.z = -r.z;
  }
  r.getMatrixYXZ(xform.R);
  xform.R = zero*xform.R;

  Vector3 back = xform.R * backward(o);
  xform.t = target + dist*back;
  return xform;
}

RigidTransform CameraConventions::setTarget(const Vector3& pos, const Vector3& tgt, const Vector3& up, CamOrientation o)
{
  RigidTransform xform;
  xform.t = pos;
  Vector3 z = tgt-pos;
  normalize(z);
  if(o == XYnZ) z.inplaceNegative();
  Vector3 x;
  x.setCross(z,up);
  normalize(x);
  Vector3 y;
  y.setCross(z,x);
  xform.R.set(x,y,z);
  return xform;
}

RigidTransform CameraConventions::setGLModelviewMatrix(const Matrix4& m)
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

void CameraConventions::getGLModelviewMatrix(const RigidTransform& xform,Matrix4& mat)
{
  RigidTransform inv;
  inv.setInverse(xform);
  inv.get(mat);
}



void CameraConventions::getFree(const RigidTransform& xform, Vector3& pos, Vector3& rot, CamOrientation o,WorldOrientation wo)
{
  Matrix3 zero = zeroPose(o,wo);
  Matrix3 R; R.mulTransposeA(zero,xform.R);
  EulerAngleRotation r;
  r.setMatrixYXZ(R);
  rot.set(r.y,r.x,r.z);
  if(o == XnYZ) {
    rot.y = -rot.y;
    rot.z = -rot.z;
  }
  pos = xform.t;
}

void CameraConventions::getOrbit(const RigidTransform& xform,Vector3& rot, Vector3& target, Real tgtdist, CamOrientation o,WorldOrientation wo)
{
  Matrix3 zero = zeroPose(o,wo);
  Matrix3 R; R.mulTransposeA(zero,xform.R);
  EulerAngleRotation r;
  r.setMatrixYXZ(R);
  rot.set(r.y,r.x,r.z);
  if(o == XnYZ) {
    rot.y = -rot.y;
    rot.z = -rot.z;
  }
  Vector3 z;
  xform.R.getCol3(z);
  target = xform.t - z*tgtdist;
}

void CameraConventions::getTarget(const RigidTransform& xform, Vector3& pos, Vector3& tgt, Vector3& up, Real tgtDist, CamOrientation o) 
{
  //y dir
  xform.R.getCol2(up);
  if(o == XnYZ) up.inplaceNegative();

  //position
  pos = xform.t;

  //z dir
  Vector3 z;
  xform.R.getCol3(z);

  if(o == XnYZ) z.inplaceNegative();
  tgt = pos + z*tgtDist;
}


void CameraController_Free::toCameraPose(RigidTransform& xform) const
{
  xform = CameraConventions::setFree(pos,rot,ori,wori);
}

void CameraController_Free::fromCameraPose(const RigidTransform& xform)
{
  CameraConventions::getFree(xform,pos,rot,ori,wori);
}

void CameraController_Target::toCameraPose(RigidTransform& xform) const
{
  xform = CameraConventions::setTarget(pos,tgt,up,ori);
}

void CameraController_Target::fromCameraPose(const RigidTransform& xform, Real tgtdist)
{
  CameraConventions::getTarget(xform,pos,tgt,up,tgtdist,ori);
}

void CameraController_Orbit::toCameraPose(RigidTransform& xform) const
{
  xform = CameraConventions::setOrbit(rot,tgt,dist,ori,wori);
}

void CameraController_Orbit::fromCameraPose(const RigidTransform& xform, Real tgtdist)
{
  CameraConventions::getOrbit(xform,rot,tgt,tgtdist,ori);
  dist = tgtdist;
}

} //namespace Camera