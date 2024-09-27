#ifndef CAMERA_H
#define CAMERA_H

#include <KrisLibrary/math3d/primitives.h>

namespace Camera {

using namespace Math3D;

/** @brief Converts between different camera conventions.
 * 
 * Orientation ABC denotes A=right in world coordinates,B=up,C=backward(!!).
 * An 'n' denotes the negative direction.  OpenGL uses XYZ orientation.
 * Other common conventions (OpenCV, ROS) use XYnZ.
 * 
 * The pose of the camera is a rigid transform from the camera to
 * the world frame.  camera->world
 * 
 * OpenGL camera matrix maps from world -> camera.
 *
 * For free cameras, Rz(roll)Rx(pitch)Ry(yaw) gives the camera frame
 * in C0 coords.  The "rot" vector has (x,y,z) = (pitch,yaw,roll),
 * which gives the intuition that you're rotating about the camera's
 * rotation axes.  Positive roll rotates the camera ccw.  Positive
 * pitch tilts the camera upward.  Positive yaw rotates the camera
 * left.
 *
 * C0 coords define how the world coords line up with the initial free
 * camera coords by the Orientation enum.  It basically defines the
 * y axis of the camera for the yaw.
 *
 * Target cameras explicitly give the up direction in world coordinates
 * and therefore there is no need for a C0 orientation.
 */
class CameraConventions
{
public:
  enum Orientation {XYZ,XYnZ,XZY,XZnY};
  static Orientation OpenGL; // = XYZ
  static Orientation OpenCV;    // = XYnZ
  static Orientation ROS;    // = XYnZ
  static void getOrientationMatrix(Orientation o,Matrix3& mat);
  static void orient(Orientation o,Matrix3& mat);
  static void unorient(Orientation o,Matrix3& mat);

  static RigidTransform setFree(const Vector3& pos, const Vector3& rot, Orientation o=XYZ);
  static RigidTransform setTarget(const Vector3& pos, const Vector3& tgt, const Vector3& up);
  static RigidTransform setOrbit(const Vector3& rot, const Vector3& target, Real dist, Orientation o=XYZ);
  /// Returns an extrinsic transform from an OpenGL camera matrix 
  static RigidTransform setGLCameraMatrix(const Matrix4&);
  static void getGLCameraMatrix(const RigidTransform& xform, Matrix4&);
  static void getFree(const RigidTransform& xform, Vector3& pos, Vector3& rot, Orientation o=XYZ);
  static void getTarget(const RigidTransform& xform, Vector3& pos, Vector3& tgt, Vector3& up, Real tgtdist=One);
  static void getOrbit(const RigidTransform& xform, Vector3& rot, Vector3& target, Real tgtdist=One, Orientation o=XYZ);
};

// CameraController_XXXX
// These classes hold the state that is attached to different camera
// control types.  Use toCameraPose and fromCameraPose to get the camera
// pose.

struct CameraController_Free
{
  void toCameraPose(RigidTransform&) const;
  void fromCameraPose(const RigidTransform&);

  inline Real& pitch() { return rot.x; }
  inline Real& yaw() { return rot.y; }
  inline Real& roll() { return rot.z; }

  Vector3 pos;
  Vector3 rot;
  CameraConventions::Orientation ori;
};

struct CameraController_Target
{
  void toCameraPose(RigidTransform&) const;
  void fromCameraPose(const RigidTransform&, Real tgtdist);

  Vector3 pos;
  Vector3 tgt;
  Vector3 up;
};

struct CameraController_Orbit
{
  void toCameraPose(RigidTransform&) const;
  void fromCameraPose(const RigidTransform&, Real tgtdist);

  inline Real& pitch() { return rot.x; }
  inline Real& yaw() { return rot.z; }
  inline Real& roll() { return rot.y; }

  Vector3 rot;
  Vector3 tgt;
  Real dist;
  CameraConventions::Orientation ori;
};

} //namespace Camera

#endif
