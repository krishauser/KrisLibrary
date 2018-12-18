#include <KrisLibrary/Logger.h>
#include "Rotation.h"
#include <math/misc.h>
#include <math3d/basis.h>
#include <math3d/misc.h>
#include <iostream>
using namespace std;

const static Real angleEps=(Real)1e-3;

Real TraceToTheta(Real trace)
{
  Real s=(trace - One)*Half;
  if(s >= One) {
    if(s > One+Epsilon) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TraceToTheta(): Warning- trace of matrix is greater than 1");
    }
    return Zero;
  }
  else if(s <= -One) {
    if(s < -One-Epsilon) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TraceToTheta(): Warning- trace of matrix is less than 1");
    }
    return Pi;
  }
  else
    return Acos(s);
}

inline Real Cube(Real x) { return x*x*x; }

//NOTE: this is not the same as getCrossProduct
inline void uncross(const Matrix3& r,Vector3& v)
{
  v.x=r.data[1][2]-r.data[2][1];
  v.y=r.data[2][0]-r.data[0][2];
  v.z=r.data[0][1]-r.data[1][0];
}


DirectionCosines::DirectionCosines()
: Vector(9) {}

DirectionCosines::DirectionCosines(const DirectionCosines& c) 
{ copy(c); }

DirectionCosines::DirectionCosines(const Vector& x)  //NOTE: shallow copy!
{
  assert(x.n==9);
  setRef(x);
}

DirectionCosines::DirectionCosines(const Matrix3& m)
  :Vector(9)
{
  setMatrix(m);
}

void DirectionCosines::setMatrix(const Matrix3& m)
{
  for(int i=0;i<3;i++) a(i)=m.data[0][i];
  for(int i=0;i<3;i++) b(i)=m.data[1][i];
  for(int i=0;i<3;i++) c(i)=m.data[2][i];
}

void DirectionCosines::getMatrix(Matrix3& m) const
{
  for(int i=0;i<3;i++) m.data[0][i]=a(i);
  for(int i=0;i<3;i++) m.data[1][i]=b(i);
  for(int i=0;i<3;i++) m.data[2][i]=c(i);
}

void DirectionCosines::getMoment(MomentRotation& mr) const
{
  Matrix3 m;
  getMatrix(m);
  if(!mr.setMatrix(m)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"DirectionCosines::getMoment: failed, must not be a valid rotation?\n");
    mr.setZero();
  }
}

void DirectionCosines::getMomentJacobian(Matrix& J) const
{
  //calculate theta
  Real trace=a(0)+b(1)+c(2);
  Real theta=TraceToTheta(trace);

  if(FuzzyEquals(theta,Zero,angleEps)) {
    J.resize(3,9,Zero);
    J(0,5) = Half; J(0,7) = -Half;
    J(1,2) = -Half; J(0,6) = Half;
    J(0,1) = Half; J(1,4) = -Half;
    return;
  }
  if(FuzzyEquals(theta,Pi)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Discontinuity: theta is pi");
    J.resize(3,9,Zero);
    J(0,0) = Pi*0.25/Sqrt((a(0)+One)*Half);
    J(1,4) = Pi*0.25/Sqrt((b(1)+One)*Half);
    J(2,8) = Pi*0.25/Sqrt((c(2)+One)*Half);
    return;
  }
  Real sinctheta = Sinc(theta);
  Real dsinctheta = Sinc_Dx(theta);
  Real scale = Half/sinctheta;
  //Real gamma = theta*(trace-One)/Cube(2*Sin(theta))-One/Sqr(2*Sin(theta));
  //Real gamma = (theta*Cos(theta)-Sin(theta))/(4*Cube(Sin(theta)));
  Real gamma = dsinctheta/Sqr(sinctheta)/(4*Sin(theta));
  Vector3 v;  //uncross
  v.x = b(2)-c(1);
  v.y = c(0)-a(2);
  v.z = a(1)-b(0);
  J.resize(3,9,Zero);
  J(0,0) = J(0,4) = J(0,8) = v.x*gamma; 
  J(1,0) = J(1,4) = J(1,8) = v.y*gamma; 
  J(2,0) = J(2,4) = J(2,8) = v.z*gamma; 
  J(0,5) = scale; J(0,7) = -scale;
  J(1,2) = -scale; J(1,6) = scale;
  J(2,1) = scale; J(2,4) = -scale;
}

void DirectionCosines::getQuaternion(QuaternionRotation& q) const
{
  Matrix3 R;
  getMatrix(R);
  q.setMatrix(R);
}

void DirectionCosines::getQuaternionJacobian(Matrix& J) const
{
  Real trace=a(0)+b(1)+c(2);
  Real tr=trace+One;
  if(tr > 1e-2) {
    Real s=Sqrt(tr);
    Real s3=tr*s;
    Real s5=tr*tr*s;
    Vector3 v;  //uncross
    v.x = b(2)-c(1);
    v.y = c(0)-a(2);
    v.z = a(1)-b(0);
    J.resize(4,9,Zero);
    J(0,0) = J(0,4) = J(0,8) = 0.25/s3;
    J(1,0) = J(1,4) = J(1,8) = -v.x/s5;
    J(2,0) = J(2,4) = J(2,8) = -v.y/s5; 
    J(2,3) = J(3,4) = J(3,8) = -v.z/s5; 
    Real scale=Two/s;
    J(1,5) = scale; J(1,7) = -scale;
    J(2,2) = -scale; J(2,6) = scale;
    J(3,1) = scale; J(3,4) = -scale;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Close to pi rotations not done yet");
    J.setZero();
    KrisLibrary::loggerWait();
  }
}

/*
void DirectionCosines::momentJacobian(const MomentRotation& m,Matrix& J)
{
}
*/

void DirectionCosines::quaternionJacobian(const Quaternion& q,Matrix& J)
{
  J.resize(9,4);
  J(0,1)=q.w;  J(0,2)=q.x;  J(0,3)=-q.y; J(0,4)=-q.z;
  J(1,1)=-q.z; J(1,2)=q.y;  J(1,3)=q.x;  J(1,4)=-q.w;
  J(2,1)=q.y;  J(2,2)=q.z;  J(2,3)=q.w;  J(2,4)=q.x;

  J(3,1)=-q.z; J(3,2)=q.y;  J(3,3)=q.x;  J(3,4)=-q.w;
  J(4,1)=q.w;  J(4,2)=-q.x; J(4,3)=q.y;  J(4,4)=-q.z;
  J(5,1)=-q.x; J(5,2)=-q.w; J(5,3)=q.z;  J(5,4)=q.y;

  J(6,1)=-q.y;  J(6,2)=q.z; J(6,3)=-q.w; J(6,4)=q.x;
  J(7,1)=q.x;  J(7,2)=q.w;  J(7,3)=q.z;  J(7,4)=q.y;
  J(8,1)=q.w;  J(8,2)=-q.x; J(8,3)=-q.y; J(8,4)=q.z;
  J.inplaceMul(Half);
}


bool GetRotationCenter(const RigidTransform& T,Vector3& p)
{
  //T*p=R*p+t=p => t=(I-R)*p
  Matrix3 I_R;
  I_R.setIdentity();
  I_R -= T.R;
  Matrix3 Ainv;
  if(!Ainv.setInverse(I_R)) {
    p.set(Inf);
    return false;
  }
  Ainv.mul(T.t,p);
  return true;
}

//get a rotation matrix that rotates x to y
void GetMinimalRotation(const Vector3& x,const Vector3& y,Matrix3& R)
{
  AngleAxisRotation aa;
  aa.axis.setCross(x,y);
  Real n=aa.axis.norm();
  //there may be a better way, similar to GetCanonicalBasis...
  if(FuzzyZero(n)) {
    Real d=dot(x,y);
    assert(Abs(d) > 0.9);
    assert(Abs(d) <= 1.0+Epsilon);
    
    if(FuzzyEquals(d,One)) aa.angle=0;
    else if(FuzzyEquals(d,-One)) aa.angle=Pi;
    else aa.angle = Acos(d);
    Vector3 temp;
    GetCanonicalBasis(x,aa.axis,temp);
  }
  else {
    Real cosAngle = dot(x,y);
    if(cosAngle > 1.0 || cosAngle < -1.0) {
      if(!FuzzyEquals(cosAngle,1.0) && !FuzzyEquals(cosAngle,-1.0)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"GetMinimalRotation: Warning: vectors aren't normalized?");
	KrisLibrary::loggerWait();
      }
      aa.angle=Acos(Clamp(cosAngle,-1.0,1.0));
    } 
    else 
      aa.angle = Acos(cosAngle);
    aa.axis *= Inv(n);
  }
  aa.getMatrix(R);
}

//get a rotation matrix that rotates x to be orthogonal to y
void GetMinimalRotationToPlane(const Vector3& x,const Vector3& y,Matrix3& R)
{
  AngleAxisRotation aa;
  aa.axis.setCross(x,y);
  Real n=aa.axis.norm();
  if(FuzzyZero(n)) {
    Real d=dot(x,y);
    Assert(Abs(d) > 0.9);
    Assert(Abs(d) <= 1.0+Epsilon);
    
    if(FuzzyEquals(d,One)) aa.angle=Pi_2;
    else if(FuzzyEquals(d,-One)) aa.angle=-Pi_2;
    else aa.angle = -Asin(d);
    Vector3 temp;
    x.getOrthogonalBasis(aa.axis,temp);
  }
  else {
    aa.angle = -Asin(dot(x,y));
    aa.axis *= Inv(n);
  }
  aa.getMatrix(R);
}






void MatrixDerivative(const Matrix3& R,const Vector3& z,Matrix3& dR)
{
  Vector3 temp;
  R.getCol1(temp); dR.setCol1(cross(z,temp));
  R.getCol2(temp); dR.setCol2(cross(z,temp));
  R.getCol3(temp); dR.setCol3(cross(z,temp));
}

void DirectionCosinesDerivative(const Vector& x,const Vector3& z,Vector& dx)
{
  Vector3 temp;
  temp.set(x(0),x(1),x(2)); temp=cross(z,temp); temp.get(dx(0),dx(1),dx(2));
  temp.set(x(3),x(4),x(5)); temp=cross(z,temp); temp.get(dx(3),dx(4),dx(5));
  temp.set(x(6),x(7),x(8)); temp=cross(z,temp); temp.get(dx(6),dx(7),dx(8));
}

void MomentDerivative(const MomentRotation& m,const Vector3& z,Vector3& dm)
{
  Matrix3 R;
  m.getMatrix(R);
  MomentDerivative(m,R,z,dm);
}

void MomentDerivative(const Matrix3& R,const Vector3& z,Vector3& dm)
{
  Real trace=R.trace();
  Real theta=TraceToTheta(trace);
  if(FuzzyEquals(theta,Zero,angleEps)) {
    dm = z;
    return;
  }

  Matrix3 dR;
  Vector3 vdR;
  MatrixDerivative(R,z,dR);
  uncross(dR,vdR);
  Real tracedR = dR.trace();

  if(FuzzyEquals(Abs(theta),Pi)) {
    dm.x = Pi*0.25/Sqrt((R(0,0)+1.0)*Half)*dR(0,0);
    dm.y = Pi*0.25/Sqrt((R(1,1)+1.0)*Half)*dR(1,1);
    dm.z = Pi*0.25/Sqrt((R(2,2)+1.0)*Half)*dR(2,2);
    if(!IsFinite(dm.x)) dm.x = 0;
    if(!IsFinite(dm.y)) dm.y = 0;
    if(!IsFinite(dm.z)) dm.z = 0;
    return;
  } 

  Real sinctheta = Sinc(theta);
  Real dsinctheta = Sinc_Dx(theta);
  Real scale = Half/sinctheta;
  //Real gamma = theta*(trace-One)/Cube(2*Sin(theta))-One/Sqr(2*Sin(theta));
  //Real gamma = (theta*Cos(theta)-Sin(theta))/(4*Cube(Sin(theta)));
  Real gamma = dsinctheta/Sqr(sinctheta)/(4*Sin(theta));

  uncross(R,dm);

  dm.inplaceMul(tracedR*gamma);
  dm.madd(vdR,scale);
}

void MomentDerivative(const Vector3& m,const Matrix3& R,const Vector3& z,Vector3& dm)
{
  Real trace=R.trace();
  Real theta=TraceToTheta(trace);

  if(FuzzyEquals(theta,Zero,angleEps)) {
    dm = z;
    return;
  }

  Matrix3 dR;
  Vector3 vdR;
  MatrixDerivative(R,z,dR);
  uncross(dR,vdR);
  Real tracedR = dR.trace();

  if(FuzzyEquals(Abs(theta),Pi)) {
    dm.x = Pi*0.25/Sqrt((R(0,0)+1)*Half)*dR(0,0);
    dm.y = Pi*0.25/Sqrt((R(1,1)+1)*Half)*dR(1,1);
    dm.z = Pi*0.25/Sqrt((R(2,2)+1)*Half)*dR(2,2);
    if(!IsFinite(dm.x)) dm.x = 0;
    if(!IsFinite(dm.y)) dm.y = 0;
    if(!IsFinite(dm.z)) dm.z = 0;
    return;
  } 

  Real sinctheta = Sinc(theta);
  Real dsinctheta = Sinc_Dx(theta);
  Real scale = Half/sinctheta;
  //since m = theta/2*sin(theta)*uncross(R)
  //dm = above gamma * 2sinc(theta) * m => gamma' = gamma*2sinc(theta)

  dm=m;
  Real gamma2 = dsinctheta/sinctheta/(2*Sin(theta));
  
  dm.inplaceMul(tracedR*gamma2);
  dm.madd(vdR,scale);
}




void QuaternionDerivative(const Matrix3& R,const Vector3& z,Quaternion& dq)
{
  Real trace=R.trace();
  Real tr=trace+One;
  if(tr > 1e-2) {
    Real s=Sqrt(tr);
    Real s3=tr*s;
    Real s5=tr*tr*s;
    Vector3 v;  //uncross
    uncross(R,v);
    dq.w = 0.25/s3*trace;
    dq.x = -v.x/s5*trace;
    dq.y = -v.y/s5*trace;
    dq.z = -v.z/s5*trace;
    Real scale=Two/s;
    dq.x += v.x*scale;
    dq.y += v.y*scale;
    dq.z += v.z*scale;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Close to pi rotations not done yet");
    dq.setZero();
    KrisLibrary::loggerWait();
  }
}

//helper function, used in the following two functions
//calculates the moments about each euler angle, returns them in
//the 3 columns of A
void EulerAngleMoments(const Vector3& theta,int u,int v,int w,Matrix3& A)
{
  Assert(0<=u&&u<3);
  Assert(0<=v&&v<3);
  Assert(0<=w&&w<3);
  Matrix3 Ru,Rv;
  Vector3 eu(Zero),ev(Zero),ew(Zero);
  eu[u]=One;
  ev[v]=One;
  ew[w]=One;
  switch(u) {
  case 0: Ru.setRotateX(theta.x); break;
  case 1: Ru.setRotateY(theta.x); break;
  default: Ru.setRotateZ(theta.x); break;
  }
  switch(v) {
  case 0: Rv.setRotateX(theta.y); break;
  case 1: Rv.setRotateY(theta.y); break;
  default: Rv.setRotateZ(theta.y); break;
  }
  A.set(eu,Ru*ev,Ru*Rv*ew);
}

bool EulerAngleDerivative(const Vector3& theta,const Vector3& z,int u,int v,int w,Vector3& dtheta)
{
  Matrix3 A;
  EulerAngleMoments(theta,u,v,w,A);
  Matrix3 Ainv;
  bool res=Ainv.setInverse(A);
  if(!res) return false;
  dtheta = Ainv*z;
  return true;
}

void AngularVelocityEulerAngle(const Vector3& theta,const Vector3& dtheta,int u,int v,int w,Vector3& z)
{
  Matrix3 A;
  EulerAngleMoments(theta,u,v,w,A);
  A.mul(dtheta,z);
}

Real MatrixAbsoluteAngle(const Matrix3& R)
{
  Real s=(R.trace() - One)*Half;
  if(!IsFinite(s)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MatrixAbsoluteAngle(): Warning- trace of matrix is not finite!");
    LOG4CXX_ERROR(KrisLibrary::logger(),R);
    Abort();
  }
  if(s >= One) {
    if(s > One+Epsilon) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MatrixAbsoluteAngle(): Warning- trace of matrix is greater than 3");
    }
    return Zero;
  }
  else if(s <= -One) {
    if(s < -One-Epsilon) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MatrixAbsoluteAngle(): Warning- trace of matrix is less than -1");
    }
    return Pi;
  }
  else 
    return Acos(s);
}

Real MatrixAngleDerivative(const Matrix3& R,const Vector3& z)
{
  AngleAxisRotation aa;
  Assert(IsFinite(R));
  if(!aa.setMatrix(R)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"MatrixAngleDerivative: matrix is not a valid rotation matrix\n");
    return 0.0;
  }
  if(FuzzyZero(aa.angle)) {
    //the small-angle formula gives dtheta ~= 1
    return One;
  }
  else if(FuzzyEquals(aa.angle,Pi)) {
    return -One;
  }
  else {
    return dot(aa.axis,z);
  }
}

Real MatrixAngleAboutAxis(const Matrix3& R,const Vector3& a)
{
  Real tr_across_R = a.x*(R(2,1)-R(1,2)) + a.y*(R(0,2)-R(2,0)) + a.z*(R(1,0)-R(0,1));
  Real tr_aat_R = Sqr(a.x)*R(0,0) + Sqr(a.y)*R(1,1) + Sqr(a.z)*R(2,2) +
    a.x*a.y*(R(1,0)+R(0,1)) + a.x*a.z*(R(2,0)+R(0,2)) + a.y*a.z*(R(2,1)+R(1,2));
  Real tr_R = R.trace();
  return Atan2(-tr_across_R,-(tr_aat_R-tr_R));
}



void MatrixDerivative2(const Matrix3& R,const Vector3& w,const Vector3& a,Matrix3& ddR)
{
  Matrix3 across,wcross,temp;
  across.setCrossProduct(a);
  wcross.setCrossProduct(w);
  temp.mul(wcross,wcross);
  temp += across;
  ddR.mul(temp,R);
}




void ForwardDifferenceAngularVelocity(const Matrix3& R0,const Matrix3& R1,Real h,Vector3& z)
{
  //R' = [z]R0 ~= (R1-R0)/h 
  Matrix3 temp;
  temp.mulTransposeB(R1,R0);
  temp.getCrossProduct(z);
  z *= 1.0/h;
}

void CenteredDifferenceAngularVelocity(const Matrix3& R_1,const Matrix3& R0,const Matrix3& R1,Real h,Vector3& z)
{
  //R' = [z]R0 ~= (R1-R_1)/2h 
  Matrix3 temp,temp2;
  temp2.sub(R1,R_1);
  temp.mulTransposeB(temp2,R0);
  temp.getCrossProduct(z);
  z *= 0.5/h;
}

void CenteredDifferenceAngularAccel(const Matrix3& R_1,const Matrix3& R0,const Matrix3& R1,Real h,Vector3& a)
{
  Vector3 w;
  CenteredDifferenceAngularVelocity(R_1,R0,R1,h,w);
  //R'' = ([a]+[w]^2)R0 ~= (R_1 - 2R0 + R1)/h^2
  //=> [a] ~= ((R_1+R1)*R0t - 2I)/h^2 - [w]^2
  Matrix3 temp,ddr;
  temp.add(R_1,R1);
  //ddr.madd(R0,-2.0);  the I term isn't necessary
  ddr.mulTransposeB(temp,R0);
  ddr *= 1.0/Sqr(h);

  Matrix3 wcross;
  wcross.setCrossProduct(w);
  temp.mul(wcross,wcross);
  ddr -= temp;
  ddr.getCrossProduct(a);
}

Real AxisRotationMagnitude(const Matrix3& R,const Vector3& a)
{
  Real cterm = R.trace() + a.dot(R*a);
  //TODO: it looks like I had a sign error in my math?  Double check tihs.
  Real sterm = -(a.x*(R(1,2)-R(2,1)) + a.y*(R(2,0)-R(0,2)) + a.z*(R(0,1)-R(1,0)));
  return Atan2(sterm,cterm);
}

void NormalizeRotation(Matrix3& R)
{
  //TODO: fix this so it works with very non-orthogonal matrices
  QuaternionRotation q;
  q.setMatrix(R);
  q.getMatrix(R);
}
