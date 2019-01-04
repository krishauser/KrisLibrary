#ifndef ROBOTICS_WRENCH_H
#define ROBOTICS_WRENCH_H

#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/matrix.h>
#include <KrisLibrary/math3d/primitives.h>
using namespace Math;
using namespace Math3D;

struct RigidBodyVelocity
{
  void setTransformed(const RigidBodyVelocity& w,const RigidTransform& T);
  void setShifted(const RigidBodyVelocity& w,const Vector3& shift);

  Vector3 v,w;
};

struct Wrench
{
  void setForceAtPoint(const Vector3& f,const Vector3& p);
  void setTransformed(const Wrench& w,const RigidTransform& T);
  void setShifted(const Wrench& w,const Vector3& shift);

  Vector3 f,m;
};

struct SpatialVector : public Vector
{
  SpatialVector();
  void set(const Vector3& a,const Vector3& b);
  void get(Vector3& a,Vector3& b) const;
  void setForce(const Vector3& force,const Vector3& moment)
  { set(force,moment); }
  void setVelocity(const Vector3& linVel,const Vector3& angVel)
  { set(linVel,angVel); }
  void setAccel(const Vector3& linAccel,const Vector3& angAccel)
  { set(linAccel,angAccel); }
};

struct SpatialMatrix : public Matrix
{
  SpatialMatrix();
  void setUpperLeft(const Matrix3& mat11);
  void setLowerRight(const Matrix3& mat22);
  void setUpperRight(const Matrix3& mat12);
  void setLowerLeft(const Matrix3& mat21);
  void getUpperLeft(Matrix3& mat11) const;
  void getLowerRight(Matrix3& mat22) const;
  void getUpperRight(Matrix3& mat12) const;
  void getLowerLeft(Matrix3& mat21) const;
  void setForceShift(const Vector3& origMomentCenter,const Vector3& newMomentCenter);
  void setForceTransform(const RigidTransform& T);
  void setVelocityShift(const Vector3& origRefPoint,const Vector3& newRefPoint);
  void setVelocityTransform(const RigidTransform& T);
  void setMassMatrix(const Real& mass,const Matrix3& inertia);
};

#endif
