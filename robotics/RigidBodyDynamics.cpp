#include "RigidBodyDynamics.h"
#include <math3d/rotation.h>

RigidBodyDynamics::RigidBodyDynamics()
{
  m = 1;
  H.setIdentity();
  Hinv.setIdentity();
  T.R.setIdentity();
  T.t.setZero();
  v.setZero();
  w.setZero();
  faccum.setZero();
  maccum.setZero();
}

void RigidBodyDynamics::SetInertiaMatrix(const Matrix3& _H)
{
  H=_H;
  Hinv.setInverse(_H);
}

void RigidBodyDynamics::SetInertiaMatrix(const Vector3& Hdiag)
{
  H.setZero();
  Hinv.setZero();
  H(0,0) = Hdiag.x;
  H(1,1) = Hdiag.y;
  H(2,2) = Hdiag.z;
  Hinv(0,0) = 1.0/Hdiag.x;
  Hinv(1,1) = 1.0/Hdiag.y;
  Hinv(2,2) = 1.0/Hdiag.z;
}

void RigidBodyDynamics::ZeroForces()
{
  faccum.setZero();
  maccum.setZero();
}

void RigidBodyDynamics::AddForce(const Vector3& f)
{
  faccum += f;
}

void RigidBodyDynamics::AddMoment(const Vector3& m)
{
  maccum += m;
}

void RigidBodyDynamics::AddForceAtPoint(const Vector3& f,const Vector3& p)
{
  faccum += f;
  maccum += cross(p-T.t,f);
}

void RigidBodyDynamics::AddLocalForceAtPoint(const Vector3& flocal,const Vector3& plocal)
{
  faccum += T.R*flocal;
  maccum += T.R*cross(plocal,flocal);  
}

void RigidBodyDynamics::Advance(Real dt)
{
  Vector3 dv = Accel();
  Vector3 dw = AngularAccel();

  v += dt*dv;
  w += dt*dw;

  T.t += dt*v;
  QuaternionRotation q,qw;
  q.setMatrix(T.R);
  qw.setMoment(MomentRotation(w*dt));
  q = q*qw;
  q.inplaceNormalize();
  q.getMatrix(T.R);
}

Vector3 RigidBodyDynamics::Accel() const
{
  return faccum/m;
}

Vector3 RigidBodyDynamics::AngularAccel() const
{
  Matrix3 Hworld;
  Matrix3 HinvWorld;
  Hworld.mulTransposeB(T.R*H,T.R);    
  HinvWorld.mulTransposeB(T.R*Hinv,T.R);

  Vector3 moment = maccum + cross(w,Hworld*w);
  return HinvWorld*moment;
}

void RigidBodyDynamics::AccelJacobian(const Vector3& forcePt,Matrix3& J) const
{
  J.setZero();
  J(0,0) = J(1,1) = J(2,2) = 1.0/m;
}

void RigidBodyDynamics::AngularAccelJacobian(const Vector3& forcePt,Matrix3& J) const
{
  //w' = T.R*Hinv*T.R^-1 (forcept - T.t) x  force
  Matrix3 HinvWorld;
  HinvWorld.mulTransposeB(T.R*Hinv,T.R);
  Matrix3 cp;
  cp.setCrossProduct(forcePt - T.t);
  J = HinvWorld*cp;
}

Vector3 RigidBodyDynamics::Momentum() const
{
  return m*v;
}

Vector3 RigidBodyDynamics::AngularMomentum() const
{
  return H*w;
}

Matrix3 RigidBodyDynamics::WorldInertia() const
{
  Matrix3 Hworld;
  Hworld.mulTransposeB(T.R*H,T.R);
  return Hworld;
}

Real RigidBodyDynamics::KineticEnergy() const
{
  return 0.5*(m*v.normSquared()+dot(w,WorldInertia()*w));
}
