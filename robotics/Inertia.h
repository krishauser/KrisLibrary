#ifndef ROBOTICS_INERTIA_H
#define ROBOTICS_INERTIA_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/geometry3d.h>

using namespace Math;
using namespace Math3D;

//computes the inertia matrix I about the point refPt for a rigid body with com at com,
//with mass mass, and inertia matrix Icom (about the center of mass).
inline void InertiaMatrixAboutPoint(const Vector3& com,Real mass,const Matrix3& Icom,const Vector3& refPt,Matrix3& I)
{
  //I = Icom - mass*[com-refPt]^2
  Matrix3 temp;
  temp.setCrossProduct(com-refPt);
  I.mul(temp,temp);
  I.inplaceMul(-mass);
  I += Icom;
}

//computes the inertia matrix for an axis-aligned box with dims (x,y,z) and mass m
inline void BoxInertiaMatrix(Real x,Real y,Real z,Real m,Matrix3& I)
{
  I.setZero();
  I(0,0) = m*(Sqr(y)+Sqr(z))/12.0;
  I(1,1) = m*(Sqr(x)+Sqr(z))/12.0;
  I(2,2) = m*(Sqr(x)+Sqr(y))/12.0;
}

//computes the inertia matrix for a solid sphere with radius r and mass m
inline void SphereInertiaMatrix(Real r,Real m,Matrix3& I)
{
  I.setZero();
  I(0,0) = I(1,1) = I(2,2) = 0.4*Sqr(r)*m;
}

//computes the inertia matrix for a hollow sphere with radius r and mass m
inline void HollowSphereInertiaMatrix(Real r,Real m,Matrix3& I)
{
  I.setZero();
  I(0,0) = I(1,1) = I(2,2) = 2.0/3.0*Sqr(r)*m;
}

//computes the inertia matrix for an axis-aligned ellipsoid with dimensions x,y,z and mass m
inline void EllipsoidInertiaMatrix(Real x,Real y,Real z,Real m,Matrix3& I)
{
  I.setZero();
  I(0,0) = m*(Sqr(y)+Sqr(z))*0.2;
  I(1,1) = m*(Sqr(x)+Sqr(z))*0.2;
  I(2,2) = m*(Sqr(x)+Sqr(y))*0.2;
}

//computes the inertia matrix for z-oriented cylinder with radius r, height h, and mass m
inline void CylinderInertiaMatrix(Real r,Real h,Real m,Matrix3& I)
{
  I.setZero();
  I(0,0) = I(1,1) = m*(Sqr(h)/12.0+Sqr(r)/4.0);
  I(2,2) = m*Sqr(r)*0.5;
}

//computes the inertia matrix for z-oriented capsule with radius r, height h, and mass m
inline void CapsuleInertiaMatrix(Real r,Real h,Real m,Matrix3& I)
{
  I.setZero();
  //mass = density*(Pi*r^2*h+4/3*Pi*r^3) = d*(a+b) = M1 + M2
  //=> d = mass/(a+b)
  Real a = Pi*r*r*h;
  Real b = 4.0/3.0*Pi*r*r*r;
  Real density = m/(a+b);
  Real M1 = density*a;
  Real M2 = density*b;
  I(0,0) = I(1,1) = M1*(0.25*Sqr(r) + Sqr(h)/12.0) + M2*(0.4*Sqr(r) + 0.375*r*h + 0.25*Sqr(h));
  I(2,2) = (M1*0.5 + M2*0.4)*Sqr(r);
}

Vector3 CenterOfMass(const GeometricPrimitive3D& geom);
Matrix3 InertiaMatrix(const GeometricPrimitive3D& geom,Real mass);

#endif
