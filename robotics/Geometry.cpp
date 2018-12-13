#include "Geometry.h"
#include <math/misc.h>
#include <math3d/misc.h>
#include <math3d/rotation.h>
#include <math3d/basis.h>

template <class Rot>
void GetRotationAboutLocalPoint(const Vector3& localpt,const Vector3& pt,const Rot& r,RigidTransform& T)
{
  //T*p = R*(p-localpt)+pt = R*p+pt-R*localpt
  r.getMatrix(T.R);
  T.R.mul(localpt,T.t);
  T.t.inplaceNegative();
  T.t += pt;
}

template <class Rot>
void GetRotationAboutPoint(const Vector3& pt,const Rot& r,RigidTransform& T)
{
  //T*p = R*(p-pt)+pt = R*p+pt-R*pt
  r.getMatrix(T.R);
  T.R.mul(pt,T.t);
  T.t.inplaceNegative();
  T.t += pt;
}

//Matrix3 specialization

template <> void GetRotationAboutPoint<Matrix3>(const Vector3& pt,const Matrix3& r,RigidTransform& T)
{
  //T*p = R*(p-pt)+pt = R*p+pt-R*pt
  T.R=r;
  T.R.mul(pt,T.t);
  T.t.inplaceNegative();
  T.t += pt;
}

template <> void GetRotationAboutLocalPoint<Matrix3>(const Vector3& localpt,const Vector3& pt,const Matrix3& r,RigidTransform& T)
{
  //T*p = R*(p-localpt)+pt = R*p+pt-R*localpt
  T.R=r;
  T.R.mul(localpt,T.t);
  T.t.inplaceNegative();
  T.t += pt;
}

#define DECLARETEMPLATES(RotType) \
template void GetRotationAboutPoint(const Vector3& pt,const RotType& r,RigidTransform& T); \
template void GetRotationAboutLocalPoint(const Vector3& localpt,const Vector3& pt,const RotType& r,RigidTransform& T);

//DECLARETEMPLATES(EulerAngleRotation);
DECLARETEMPLATES(AngleAxisRotation);
DECLARETEMPLATES(MomentRotation);
//DECLARETEMPLATES(Matrix3);





bool BallCircleCollision(const Sphere3D& a, const Circle3D& b)
{
  return b.boundaryIntersects(a);
}

void CircleCircleClosestPoints(const Circle3D& a, const Circle3D& b,
			       Vector3& pa, Vector3& pb)
{
  Vector3 dir = b.center - a.center;

  //project the directions onto the circles' planes
  Vector3 da = dir;
  da.madd(a.axis,-dot(da,a.axis));
  Real n = da.norm();
  if(FuzzyZero(n)) {  //pick any point on the circle
    Vector3 xb,yb;
    GetCanonicalBasis(a.axis,xb,yb);
    da = a.radius*xb;
    dir = b.center - (a.center+da);
  }
  else 
    da *= a.radius/n;

  Vector3 db = -dir;
  db.madd(b.axis,-dot(db,b.axis));
  n = db.norm();
  if(FuzzyZero(n)) {  //pick any point on the circle
    Vector3 xb,yb;
    GetCanonicalBasis(b.axis,xb,yb);
    db = b.radius*xb;
  }
  else 
    db *= b.radius/n;

  pa = a.center + da;
  pb = b.center + db;
}

int BallBallIntersection(const Sphere3D& a,const Sphere3D& b,Circle3D& c)
{
  Vector3 delta; delta.sub(b.center,a.center);
  Real d = delta.norm();
  if(d > a.radius + b.radius) return 0;
  if(a.radius + d <= b.radius) return 3;
  if(b.radius + d <= a.radius) return 4;
  assert(d != Zero);
  Real da = Half*((Sqr(a.radius)-Sqr(b.radius))/d+d);
  delta /= d;
  c.axis = delta;
  c.center = a.center; c.center.madd(delta,da);
  Assert(da < a.radius);
  c.radius = pythag_leg(da,a.radius);
  if(da == a.radius) return 1;
  return 2;
}



#include <iostream>
using namespace std;

void CollisionSelfTest()
{
  Sphere3D s;
  Circle3D ca,cb;

  ca.axis.set(0,0,1);
  ca.center.setZero();
  ca.radius = 1;
  s.center.set(0.5,0.2,0.1);
  s.radius = 0.3;
  assert(!BallCircleCollision(s,ca));

  s.center.set(5,0,0.3);
  s.radius = 5;
  assert(BallCircleCollision(s,ca));

  cb.axis.set(1,0,0);
  cb.center.set(1,0,5);
  cb.radius = 1;

  Vector3 p,q;
  CircleCircleClosestPoints(ca,cb,p,q);
  assert(FuzzyEquals((p-q).norm(),4));

  cb.center.set(0,0,5);
  CircleCircleClosestPoints(ca,cb,p,q);
  assert(FuzzyEquals((p-q).norm(),Sqrt((Real)17.0)));

  cb.axis.set(0,0,1);
  CircleCircleClosestPoints(ca,cb,p,q);
  assert(FuzzyEquals((p-q).norm(),5));
}
