#ifndef CAMERA_H
#define CAMERA_H

#include <KrisLibrary/math3d/primitives.h>

namespace Camera {

using namespace Math3D;

// Notes:
// Remember: multiplying by the transform of the camera gets you
// from camera to world frame.  camera->world
//
// For free cameras, Rz(roll)Rx(pitch)Ry(yaw) gives the camera frame
// in C0 coords.  The "rot" vector has (x,y,z) = (pitch,yaw,roll),
// which gives the intuition that you're rotating about the camera's
// rotation axes.  Positive roll rotates the camera ccw.  Positive
// pitch tilts the camera upward.  Positive yaw rotates the camera
// left.
//
// C0 coords define how the world coords line up with the initial free
// camera coords by the Orientation enum.  It basically defines the
// y axis of the camera for the yaw.
//
// Orientation ABC denotes A=left in world coordinates,B=up,C=backward(!!).
// An 'n' denotes the negative direction.
//
// Target cameras explicitly give the up direction in world coordinates
// and therefore there is no need for a C0 orientation.

struct Camera
{
  enum Orientation {XYZ,XYnZ,XZY,XZnY};
  static void GetOrientationMatrix(Orientation o,Matrix3& mat);
  static void Orient(Orientation o,Matrix3& mat);
  static void Unorient(Orientation o,Matrix3& mat);

  inline const Real* xDir() const { return xform.R.col1(); }
  inline const Real* yDir() const { return xform.R.col2(); }
  inline const Real* zDir() const { return xform.R.col3(); }
  inline const Vector3& position() const { return xform.t; }

  void setFree(const Vector3& pos, const Vector3& rot, Orientation o=XYZ);
  void setTarget(const Vector3& pos, const Vector3& tgt, const Vector3& up);
  void setOrbit(const Vector3& rot, const Vector3& target, Real dist, Orientation o=XYZ);
  void setCameraMatrix(const Matrix4&);

  void getCameraMatrix(Matrix4&) const;
  void getFree(Vector3& pos, Vector3& rot, Orientation o=XYZ) const;
  void getTarget(Vector3& pos, Vector3& tgt, Vector3& up, Real tgtdist=One) const;
  void getOrbit(Vector3& rot, Vector3& target, Real tgtdist=One, Orientation o=XYZ) const;

  RigidTransform xform;
};

// CameraController_XXXX
// These classes hold the state that is attached to different camera
// control types.  Use toCamera and fromCamera to create the camera
// structure.

struct CameraController_Free
{
  void toCamera(Camera&) const;
  void fromCamera(const Camera&);

  inline Real& pitch() { return rot.x; }
  inline Real& yaw() { return rot.y; }
  inline Real& roll() { return rot.z; }

  Vector3 pos;
  Vector3 rot;
  Camera::Orientation ori;
};

struct CameraController_Target
{
  void toCamera(Camera&) const;
  void fromCamera(const Camera&, Real tgtdist);

  Vector3 pos;
  Vector3 tgt;
  Vector3 up;
};

struct CameraController_Orbit
{
  void toCamera(Camera&) const;
  void fromCamera(const Camera&, Real tgtdist);

  inline Real& pitch() { return rot.x; }
  inline Real& yaw() { return rot.z; }
  inline Real& roll() { return rot.y; }

  Vector3 rot;
  Vector3 tgt;
  Real dist;
  Camera::Orientation ori;
};

} //namespace Camera

#endif
