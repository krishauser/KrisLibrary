#ifndef CAMERA_H
#define CAMERA_H

#include <KrisLibrary/math3d/primitives.h>

namespace Camera {

using namespace Math3D;

/** @brief Converts between different camera conventions.
 * 
 * Camera Orientation ABC denotes A=right in world coordinates, B=up,
 * C=forward(). An 'n' denotes the negative direction.  OpenGL uses
 * XYnZ orientation. Other common conventions (OpenCV, ROS) use XnYZ.
 * 
 * The pose of the camera is a rigid transform from the camera to
 * the world frame.  (camera->world)
 * 
 * OpenGL camera matrix maps from world -> camera.
 * 
 * Most camera controllers have a zero orientation that looks "forward"
 * with the camera's right direction aligned to the x axis and the up
 * direction aligned to the world up. 
 *
 * For free cameras, Rz(roll)Rx(pitch)Ry(yaw) gives the camera frame
 * in C0 coords.  The "rot" vector has (x,y,z) = (pitch,yaw,roll),
 * which gives the intuition that you're rotating about the camera's
 * local axes.  Positive roll rotates the camera ccw.  Positive
 * pitch tilts the camera upward.  Positive yaw rotates the camera
 * left.
 *
 * Target cameras explicitly give the up direction in world coordinates
 * and therefore there is no need for a world orientation.
 * 
 * Orbit cameras follow the same convention as free cameras but are
 * offset back from the target.
 * 
 * To convert from one convention to another, pre-multiply the posee by
 * orientationMatrix(osrc,otgt).
 */
class CameraConventions
{
public:
  enum CamOrientation {XYnZ,XnYZ};
  enum WorldOrientation {Zup,Yup};
  static CamOrientation OpenGL; // = XYnZ
  static CamOrientation OpenCV;    // = XnYZ
  static CamOrientation ROS;    // = XnYZ
  /// Returns the forward vector for the given orientation 
  static Vector3 forward(CamOrientation o);
  /// Returns the backward vector for the given orientation 
  static Vector3 backward(CamOrientation o);
  /// Returns the right vector for the given orientation 
  static Vector3 right(CamOrientation o);
  /// Returns the left vector for the given orientation 
  static Vector3 left(CamOrientation o);
  /// Returns the up vector for the given orientation 
  static Vector3 up(CamOrientation o);
  /// Returns the down vector for the given orientation 
  static Vector3 down(CamOrientation o);
  /// Returns the matrix that orients an object from osrc orientation to otgt orientation
  static Matrix3 orientationMatrix(CamOrientation osrc,CamOrientation otgt);
  
  /// Returns the zero pose for the given camera / world orientation with cam up aligned 
  /// with the world up and cam right aligned with the world x.
  static Matrix3 zeroPose(CamOrientation o, WorldOrientation wo);

  static RigidTransform setFree(const Vector3& pos, const Vector3& rot, CamOrientation o, WorldOrientation wo=Zup);
  static RigidTransform setTarget(const Vector3& pos, const Vector3& tgt, const Vector3& up, CamOrientation o);
  static RigidTransform setOrbit(const Vector3& rot, const Vector3& target, Real dist, CamOrientation o, WorldOrientation wo=Zup);
  static void getFree(const RigidTransform& xform, Vector3& pos, Vector3& rot, CamOrientation o, WorldOrientation wo=Zup);
  static void getTarget(const RigidTransform& xform, Vector3& pos, Vector3& tgt, Vector3& up, Real tgtdist, CamOrientation o);
  static void getOrbit(const RigidTransform& xform, Vector3& rot, Vector3& target, Real tgtdist, CamOrientation o, WorldOrientation wo=Zup);

  /// Returns a camera pose from an OpenGL modelview matrix 
  static RigidTransform setGLModelviewMatrix(const Matrix4&);
  /// Returns an OpenGL modelview matrix from a camera pose
  static void getGLModelviewMatrix(const RigidTransform& xform, Matrix4&);
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
  CameraConventions::CamOrientation ori;
  CameraConventions::WorldOrientation wori;
};

struct CameraController_Target
{
  void toCameraPose(RigidTransform&) const;
  void fromCameraPose(const RigidTransform&, Real tgtdist);

  Vector3 pos;
  Vector3 tgt;
  Vector3 up;
  CameraConventions::CamOrientation ori;
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
  CameraConventions::CamOrientation ori;
  CameraConventions::WorldOrientation wori;
};

} //namespace Camera

#endif
