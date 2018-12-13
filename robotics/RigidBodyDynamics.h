#ifndef RIGID_BODY_DYNAMICS_H
#define RIGID_BODY_DYNAMICS_H

#include <KrisLibrary/math3d/primitives.h>
using namespace Math3D;

/** @ingroup Robotics
 * @brief A class for simulating a rigid body.  Uses Euler integration.
 */
class RigidBodyDynamics
{
 public:
  RigidBodyDynamics();
  void SetInertiaMatrix(const Matrix3& H);
  void SetInertiaMatrix(const Vector3& Hdiag);
  void ZeroForces();
  void AddForce(const Vector3& f);
  void AddMoment(const Vector3& m);
  void AddForceAtPoint(const Vector3& f,const Vector3& p);
  void AddLocalForceAtPoint(const Vector3& fLocal,const Vector3& pLocal);
  //simulate using euler integration
  void Advance(Real dt);

  //helpers
  Vector3 Accel() const;
  Vector3 AngularAccel() const;
  void AccelJacobian(const Vector3& forcePt,Matrix3& J) const;
  void AngularAccelJacobian(const Vector3& forcePt,Matrix3& J) const;
  Vector3 Momentum() const;
  Vector3 AngularMomentum() const;
  Matrix3 WorldInertia() const;
  Real KineticEnergy() const;

  ///Mass
  Real m;
  ///Local inertia matrix and inertia inverse
  Matrix3 H,Hinv;
  ///Transform about the COM
  RigidTransform T;
  ///World space velocity and angular velocity
  Vector3 v,w;

  //world coordinate force and moment accumulators
  Vector3 faccum,maccum;
};


#endif
