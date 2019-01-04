#include <KrisLibrary/Logger.h>
#include "rotation.h"
#include <math/misc.h>
#include <iostream>
using namespace std;

namespace Math3D {

const static Real angleEps=(Real)1e-3;

EulerAngleRotation::EulerAngleRotation()
{}

EulerAngleRotation::EulerAngleRotation(const EulerAngleRotation& r)
  :Vector3(r)
{}

EulerAngleRotation::EulerAngleRotation(const Vector3& v)
  :Vector3(v)
{}

EulerAngleRotation::EulerAngleRotation(Real a, Real b, Real c)
  :Vector3(a,b,c)
{}


bool EulerAngleRotation::setMatrix(int u,int v,int w,const Matrix3& m)
{
  if(u==0&&v==1&&w==2) {
    return setMatrixXYZ(m);
  }
  else if(u==2&&v==1&&w==0) {
    return setMatrixZYX(m);
  }
  else if(u==2&&v==0&&w==1) {
    return setMatrixZXY(m);
  }
  else if(u==1&&v==0&&w==2) {
    return setMatrixYXZ(m);
  }
  else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Not done with general euler angle rotation setMatrix\n");
    return false;
  }
}

bool EulerAngleRotation::setMatrixXYZ(const Matrix3& m)
{
  Real a,b,c;
  b=Asin(m(0,2));  //m(0,2)=sb
  Real cb = Cos(b);
  if (Abs(cb) > Epsilon) {
    Real ca, cc;
    ca = m(2,2)/cb;   //m(2,2)=ca*cb
    if(!(Abs(ca) <= 1+Epsilon)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"EulerAngleRotation::setMatrixXYZ: Matrix element m22/cos(m02) greater than 1\n");
      return false;
    }
    ca = Clamp(ca,-One,One);
    if (Sign(m(1,2)) == Sign(cb)) //m(1,2)=-sa*cb
      a = TwoPi - Acos(ca);
    else
 	    a = Acos(ca);

    cc = m(0,0) / cb;  //m(0,0)=cb*cc
    cc = Clamp(cc,-One,One);
    if (Sign(m(0,1)) != Sign(cb)) //m(0,1)=cb*sc
  		c = Acos(cc);
    else
      c = TwoPi - Acos(cc);
  }
  else {
    // b is close to 90 degrees, i.e. cb=0
    // this reduces the degrees of freedom, so we can set c=0
    c = 0;
    if(!(Abs(m(1,1)) <= 1+Epsilon)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"EulerAngleRotation::setMatrixXYZ: Matrix element greater than 1\n");
      return false;
    }
    a = Acos(Clamp(m(1,1),-One,One)); //m(1,1)=ca
    if(Sign(Sin(a)) != Sign(m(2,1)))  //m(2,1)=sa
      a = TwoPi - a;
  }

  x=(Real)a;
  y=(Real)b;
  z=(Real)c;
  return true;
}

bool EulerAngleRotation::setMatrixZYX(const Matrix3& m)
{
  Real a,b,c;
  b=-Asin(m(2,0));  //m(2,0)=-sb
  Real cb = Cos(b);
  if (Abs(cb) > Epsilon) {
    Real ca, cc;
    ca = m(0,0)/cb;   //m(0,0)=ca*cb
    ca = Clamp(ca,-One,One);
    if (Sign(m(1,0)) == Sign(cb)) //m(1,0)=sa*cb
      a = Acos(ca);
    else
      a = TwoPi - Acos(ca);

    cc = m(2,2) / cb;  //m(2,2)=cb*cc
    cc = Clamp(cc,-One,One);
    if (Sign(m(2,1)) == Sign(cb)) //m(2,1)=cb*sc
  		c = Acos(cc);
    else
      c = TwoPi - Acos(cc);
  }
  else {
    // b is close to 90 degrees, i.e. cb=0
    // this reduces the degrees of freedom, so we can set c=0
    c = 0;
    //m(0,1)=-sa
    a = -Asin(m(0,1));
    if(Sign(Cos(a)) != Sign(m(1,1))) //m(1,1)=ca
      a = Pi - a;
  }

  x=(Real)a;
  y=(Real)b;
  z=(Real)c;
  //TODO: domain error checking
  return true;
}

bool EulerAngleRotation::setMatrixZXY(const Matrix3& m)
{
  Real a,b,c;
  b=Asin(m(2,1));  //m(2,1)=sb
  Real cb = Cos(b);
  if (Abs(cb) > Epsilon) {
    Real ca, cc;
    ca = m(1,1)/cb;   //m(1,1)=ca*cb
    ca = Clamp(ca,-One,One);
    if (Sign(m(0,1)) != Sign(cb)) //m(0,1)=-sa*cb
      a = Acos(ca);
    else
      a = TwoPi - Acos(ca);

    cc = m(2,2) / cb;  //m(2,2)=ca*cc
    cc = Clamp(cc,-One,One);
    if (Sign(m(2,0)) != Sign(cb)) //m(2,0)=-cb*sc
      c = Acos(cc);
    else
      c = TwoPi - Acos(cc);
  }
  else {
    // b is close to 90 degrees, i.e. cb=0
    // this reduces the degrees of freedom, so we can set c=0
    c = 0;
    //m(1,0)=sa
    a = Asin(m(1,0));
    if(Sign(Cos(a)) != Sign(m(0,0))) //m(0,0)=ca
      a = Pi - a;
  }

  x=(Real)a;
  y=(Real)b;
  z=(Real)c;
  //TODO: domain error checking
  return true;
}

bool EulerAngleRotation::setMatrixYXZ(const Matrix3& m)
{
  Real a,b,c;
  b=-Asin(m(1,2));  //m(1,2)=-sb
  Real cb = Cos(b);
  if (Abs(cb) > Epsilon) {
    Real ca, cc;
    ca = m(2,2)/cb;   //m(2,2)=ca*cb
    ca = Clamp(ca,-One,One);
    if (Sign(m(0,2)) == Sign(cb)) //m(0,2)=sa*cb
      a = Acos(ca);
    else
      a = TwoPi - Acos(ca);

    cc = m(1,1) / cb;  //m(1,1)=cb*cc
    cc = Clamp(cc,-One,One);
    if (Sign(m(1,0)) == Sign(cb)) //m(1,0)=cb*sc
      c = Acos(cc);
    else
      c = TwoPi - Acos(cc);
  }
  else {
    // b is close to 90 degrees, i.e. cb=0
    // this reduces the degrees of freedom, so we can set c=0
    c = 0;
    //m(2,0)=-sa
    a = -Asin(m(2,0));
    if(Sign(Cos(a)) != Sign(m(0,0))) //m(0,0)=ca
      a = Pi - a;
  }

  x=(Real)a;
  y=(Real)b;
  z=(Real)c;
  //TODO: domain error checking
  return true;
}

void EulerAngleRotation::getMatrix(int u,int v,int w,Matrix3& m) const
{
  Matrix3 Ru,Rv,Rw;
  switch(u) {
  case 0:  Ru.setRotateX(x);  break;
  case 1:  Ru.setRotateY(x);  break;
  case 2:  Ru.setRotateZ(x);  break;
    default: LOG4CXX_ERROR(KrisLibrary::logger(),"EulerAngleRotation::getMatrix(): Invalid axis "<<u); break;
  }
  switch(v) {
  case 0:  Rv.setRotateX(y);  break;
  case 1:  Rv.setRotateY(y);  break;
  case 2:  Rv.setRotateZ(y);  break;
    default: LOG4CXX_ERROR(KrisLibrary::logger(),"EulerAngleRotation::getMatrix(): Invalid axis "<<v); break;
  }
  switch(w) {
  case 0:  Rw.setRotateX(z);  break;
  case 1:  Rw.setRotateY(z);  break;
  case 2:  Rw.setRotateZ(z);  break;
    default: LOG4CXX_ERROR(KrisLibrary::logger(),"EulerAngleRotation::getMatrix(): Invalid axis "<<w); break;
  }
  m = Ru*Rv*Rw;
}


void EulerAngleRotation::getMatrixXYZ(Matrix3& m) const
{
  Real ca = Cos(x);
  Real cb = Cos(y);
  Real cc = Cos(z);
  Real sa = Sin(x);
  Real sb = Sin(y);
  Real sc = Sin(z);

  //remember data is column major
  m.data[0][0]=cb*cc;
  m.data[0][1]=sa*sb*cc+ca*sc;
  m.data[0][2]=sa*sc-ca*sb*cc; 
  m.data[1][0]=-cb*sc;
  m.data[1][1]=ca*cc-sa*sb*sc; 
  m.data[1][2]=ca*sb*sc+sa*cc;
  m.data[2][0]=sb; 
  m.data[2][1]=-sa*cb; 
  m.data[2][2]=ca*cb;
}

void EulerAngleRotation::getMatrixZYX(Matrix3& m) const
{
  Real ca = Cos(x);
  Real cb = Cos(y);
  Real cc = Cos(z);
  Real sa = Sin(x);
  Real sb = Sin(y);
  Real sc = Sin(z);

  //remember data is column major
  m.data[0][0]=ca*cb;
  m.data[0][1]=sa*cb;
  m.data[0][2]=-sb; 
  m.data[1][0]=ca*sb*sc-sa*cc;
  m.data[1][1]=sa*sb*sc+ca*cc; 
  m.data[1][2]=cb*sc;
  m.data[2][0]=ca*sb*cc+sa*sc; 
  m.data[2][1]=sa*sb*cc-ca*sc; 
  m.data[2][2]=cb*cc;
}

void EulerAngleRotation::getMatrixZXY(Matrix3& m) const
{
  Real ca = Cos(x);
  Real cb = Cos(y);
  Real cc = Cos(z);
  Real sa = Sin(x);
  Real sb = Sin(y);
  Real sc = Sin(z);

  //remember data is column major
  m.data[0][0]=-sa*sb*sc+ca*cc;
  m.data[0][1]=ca*sb*sc+sa*cc;
  m.data[0][2]=-cb*sc; 
  m.data[1][0]=-sa*cb;
  m.data[1][1]=ca*cb; 
  m.data[1][2]=sb;
  m.data[2][0]=sa*sb*cc+ca*sc; 
  m.data[2][1]=-ca*sb*cc+sa*sc; 
  m.data[2][2]=cb*cc;
}

void EulerAngleRotation::getMatrixYXZ(Matrix3& m) const
{
  Matrix3 ma,mb,mc;
  ma.setRotateY(x);
  mb.setRotateX(y);
  mc.setRotateZ(z);
  m = ma*mb*mc;
}


AngleAxisRotation::AngleAxisRotation()
{}

AngleAxisRotation::AngleAxisRotation(const AngleAxisRotation& r)
  :angle(r.angle), axis(r.axis)
{}

AngleAxisRotation::AngleAxisRotation(Real _angle, const Vector3& _axis)
{
  set(_angle, _axis);
}

AngleAxisRotation::AngleAxisRotation(const MomentRotation& r) 
{
  setMoment(r);
}

void AngleAxisRotation::set(const AngleAxisRotation& r)
{
  angle = r.angle;
  axis = r.axis;
}

void AngleAxisRotation::set(Real _angle, const Vector3& _axis)
{
  angle = _angle;
  setAxis(_axis);
}

void AngleAxisRotation::setAxis(const Vector3& _axis)
{
  Real l2 = _axis.normSquared();
  if(FuzzyEquals(l2,One))
    axis = _axis;
  else
    axis.div(_axis, Inv(Sqrt(l2)));
}

void AngleAxisRotation::setIdentity()
{
  angle = Zero;
  axis.set(One,Zero,Zero);
}

void AngleAxisRotation::transformPoint(const Vector3& in,Vector3& out) const
{
  Real cm = Cos(angle);
  Real sm = Sin(angle);

  //m = s[r]-c[r][r]+rrt = s[r]-c(rrt-I)+rrt = cI + rrt(1-c) + s[r]
  //=> mv = cv + (1-c)*(r.v)r + s*(r x v)
  out.setCross(axis,in);
  out *= sm;
  out.madd(axis,(One-cm)*dot(axis,in));
  out.madd(in,cm);
}

bool AngleAxisRotation::setMatrix(const Matrix3& r)
{
  MomentRotation m;
  if(!m.setMatrix(r)) return false;
  setMoment(m);
  return true;
}

void AngleAxisRotation::getMatrix(Matrix3& m) const
{
  if(angle == Zero) {
    m.setIdentity();
    return;
  }

  Real cm = Cos(angle);
  Real sm = Sin(angle);

  //m = s[r]-c[r][r]+rrt = s[r]-c(rrt-I)+rrt = cI + rrt(1-c) + s[r]
  m.setCrossProduct(axis);
  m.inplaceMul(sm);
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++)
      m(i,j) += axis[i]*axis[j]*(One-cm);
    m(i,i) += cm;
  }
  /*
  Matrix3 rmat;
  rmat.setCrossProduct(axis);

  m.mul(rmat, rmat);
  m.inplaceScale(-cm);

  rmat.inplaceScale(sm);
  m += rmat;

  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      m.data[i][j] += axis[i]*axis[j];
  */
}

void AngleAxisRotation::setMoment(const MomentRotation& r)
{
  Real l = r.norm();
  angle = l;
  if(l == Zero)
    axis.setZero();
  else
    axis.div(r, l);
}

void AngleAxisRotation::getMoment(MomentRotation& r) const
{
  r.mul(axis, angle);
}

MomentRotation::MomentRotation()
{}

MomentRotation::MomentRotation(const MomentRotation& r)
  :Vector3(r)
{}

MomentRotation::MomentRotation(Real x,Real y,Real z)
  :Vector3(x,y,z)
{}

MomentRotation::MomentRotation(const Vector3& v)
  :Vector3(v)
{}

MomentRotation::MomentRotation(const AngleAxisRotation& a)
{
  setAngleAxis(a);
}

void MomentRotation::transformPoint(const Vector3& in,Vector3& out) const
{
  AngleAxisRotation a;
  getAngleAxis(a);
  a.transformPoint(in,out);
}

bool MomentRotation::setMatrix(const Matrix3& r)
{
  //calculate theta
  Real theta;
  Real c=(r.trace() - One)*Half;
  if(!IsFinite(c)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"MomentRotation::setMatrix(): Warning- trace of matrix is not finite!");
    LOG4CXX_ERROR(KrisLibrary::logger(),r);
    return false;
  }
  if(c >= One) {
    if(c > One+Epsilon) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MomentRotation::setMatrix(): Warning- trace of matrix is greater than 3");
      LOG4CXX_ERROR(KrisLibrary::logger(),"  Matrix:"<<r<<"\n");
    }
    c = One;
    theta = Zero;
  }
  else if(c <= -One) {
    if(c < -One-Epsilon) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"MomentRotation::setMatrix(): Warning- trace of matrix is less than -1");
      LOG4CXX_ERROR(KrisLibrary::logger(),"  Matrix:"<<r<<"\n");
    }
    c = -One;
    theta = Pi;
  }
  else 
    theta = Acos(c);

  if(FuzzyEquals(theta,Pi,angleEps)) {
    //can't do normal version because the scale factor reaches a singularity
    x = Sqrt(Max((r.data[0][0]-c)/(1.0-c),0.0));
    y = Sqrt(Max((r.data[1][1]-c)/(1.0-c),0.0));
    z = Sqrt(Max((r.data[2][2]-c)/(1.0-c),0.0));
    x = theta*x;
    y = theta*y;
    z = theta*z;
    //determined up to sign changes, we know r12+r21~=4xy,r13+r31~=4xz,r23+r32~=4yz
    Real xy=r(0,1)+r(1,0),xz=r(0,2)+r(2,0),yz=r(1,2)+r(2,1);
    if(x > y) {
      if(x > z) {
	//x is largest
	if(xy < 0) y=-y;
	if(xz < 0) z=-z;
      }
      else {
	//z is largest
	if(yz < 0) y=-y;
	if(xz < 0) x=-x;
      }
    }
    else {
      if(y > z) {
	//y is largest
	if(xy < 0) x=-x;
	if(yz < 0) z=-z;
      }
      else {
	//z is largest
	if(yz < 0) y=-y;
	if(xz < 0) x=-x;
      }
    }
    Matrix3 test;
    getMatrix(test);
    if(!test.isEqual(r,2e-3)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"MomentRotation::setMatrix(): Numerical error occurred, matrix is probably not a rotation?\n");
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"Input*Input^T (should be orthogonal)\n");
      Matrix3 ortho;
      ortho.mulTransposeB(r,r);
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<ortho(0,0)<<" "<<ortho(0,1)<<" "<<ortho(0,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<ortho(1,0)<<" "<<ortho(1,1)<<" "<<ortho(1,2));           
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<ortho(2,0)<<" "<<ortho(2,1)<<" "<<ortho(2,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"Test errors with moment "<<x<<" "<<y<<" "<<z);
      test -= r;
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<test(0,0)<<" "<<test(0,1)<<" "<<test(0,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<test(1,0)<<" "<<test(1,1)<<" "<<test(1,2));            
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<test(2,0)<<" "<<test(2,1)<<" "<<test(2,2));      
            return false;
    }

    Assert(IsFinite(x));
    Assert(IsFinite(y));
    Assert(IsFinite(z));
    return true;
  }
  //Matrix3 r_cross;
  //r_cross.setTranspose(r);
  //r_cross.sub(r, r_cross);
  //r_cross.inplaceScale(scale);

  //this is a better method for angles close to pi
  if(FuzzyEquals(theta,Pi,0.5)) {
    //c ~= -1
    x = Sqrt(Max((r.data[0][0]-c)/(1.0-c),0.0));
    y = Sqrt(Max((r.data[1][1]-c)/(1.0-c),0.0));
    z = Sqrt(Max((r.data[2][2]-c)/(1.0-c),0.0));
    double eps = Pi-theta;
    x *= Sign(eps)*Sign(r.data[1][2]-r.data[2][1]);
    y *= Sign(eps)*Sign(r.data[2][0]-r.data[0][2]);
    z *= Sign(eps)*Sign(r.data[0][1]-r.data[1][0]);
    x *= theta;
    y *= theta;
    z *= theta;
  }
  else {
    //Real scale = Half*theta/Sin(theta);
    Real scale = Half/Sinc(theta);  //avoids the singularity at 0
    Assert(IsFinite(scale));

    x = (r.data[1][2]-r.data[2][1]) * scale;
    y = (r.data[2][0]-r.data[0][2]) * scale;
    z = (r.data[0][1]-r.data[1][0]) * scale;
  }
  Assert(IsFinite(x));
  Assert(IsFinite(y));
  Assert(IsFinite(z));
  Matrix3 test;
  getMatrix(test);
  if (!test.isEqual(r, 1e-3)) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(), "MomentRotation::setMatrix(): Numerical error occurred, matrix is probably not a rotation?\n");
        LOG4CXX_ERROR(KrisLibrary::logger(),"Input:\n");
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2));            
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2));            
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2)); 
        LOG4CXX_ERROR(KrisLibrary::logger(),"Input*Input^T (should be orthogonal)\n");
    Matrix3 ortho;
    ortho.mulTransposeB(r,r);
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<ortho(0,0)<<" "<<ortho(0,1)<<" "<<ortho(0,2));            
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<ortho(1,0)<<" "<<ortho(1,1)<<" "<<ortho(1,2));            
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<ortho(2,0)<<" "<<ortho(2,1)<<" "<<ortho(2,2)); 
        LOG4CXX_ERROR(KrisLibrary::logger(),"Moment "<<x<<" "<<y<<" "<<z<<" (angle "<<theta);
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<test(0,0)<<" "<<test(0,1)<<" "<<test(0,2));            
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<test(1,0)<<" "<<test(1,1)<<" "<<test(1,2));            
        LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<test(2,0)<<" "<<test(2,1)<<" "<<test(2,2)); 
        LOG4CXX_ERROR(KrisLibrary::logger(),"Sinc function should not be near 0: "<<Sinc(theta));
	  return false;
  }
  return true;
}

void MomentRotation::getMatrix(Matrix3& m) const
{
  AngleAxisRotation a;
  a.setMoment(*this);
  a.getMatrix(m);
}

void MomentRotation::setAngleAxis(const AngleAxisRotation& a)
{
  mul(a.axis, a.angle);
}

void MomentRotation::getAngleAxis(AngleAxisRotation& a) const
{
  a.setMoment(*this);
}

QuaternionRotation::QuaternionRotation()
{}

QuaternionRotation::QuaternionRotation(Real w, Real x, Real y, Real z)
:Quaternion(w,x,y,z)
{}

QuaternionRotation::QuaternionRotation(const QuaternionRotation& q)
:Quaternion(q)
{}

QuaternionRotation::QuaternionRotation(const Quaternion& q)
:Quaternion(q)
{}


void QuaternionRotation::setAngleAxis(const AngleAxisRotation& r)
{
  Real imscale = Sin(r.angle*Half);
  w = Cos(r.angle*Half);
  x = imscale*r.axis.x;
  y = imscale*r.axis.y;
  z = imscale*r.axis.z;
}


void QuaternionRotation::setMoment(const MomentRotation& m)
{
  setAngleAxis(AngleAxisRotation(m));
}

bool QuaternionRotation::setMatrix(const Matrix3& m)
{
  Real tr, s;
  tr = m.trace() + One;

  // check the diagonal
  if (tr > 1e-5) {
    s = Sqrt (tr);
    w = s * Half;
    s = Half / s;
    x = (m.data[1][2] - m.data[2][1]) * s;
    y = (m.data[2][0] - m.data[0][2]) * s;
    z = (m.data[0][1] - m.data[1][0]) * s;
  }
  else {
    //it's a rotation of 180 degrees
    const static int nxt[3] = {1, 2, 0};
    int i, j, k;
    // diagonal is negative
    i = 0;
    if (m.data[1][1] > m.data[0][0]) i = 1;
    if (m.data[2][2] > m.data[i][i]) i = 2;
    j = nxt[i];
    k = nxt[j];
    
    Real q[4];
    s = Sqrt ((m.data[i][i] - (m.data[j][j] + m.data[k][k])) + One);
    q[i] = s * Half;
    
    if (FuzzyZero(s)) {
      //assume i=0,j=1,k=2
      //m(0,0) - m(1,1) - m(2,2) = 1
      //m(0,0) + m(1,1) + m(2,2) = -1
      // so m(0,0) = 0
      // since m(1,1) > m(2,2), 
      // m(2,2) < -0.5
      // m(1,1) > -0.5
      // m(1,1) = -1 - m(2,2)
      LOG4CXX_ERROR(KrisLibrary::logger(),"QuaternionRotation::setMatrix(): s is zero, what do we do?");
      LOG4CXX_ERROR(KrisLibrary::logger(),"May be a non-rotation matrix");
      LOG4CXX_ERROR(KrisLibrary::logger(),m);
      for(int i=0;i<4;i++) q[i]=0;
      return false;
    }
    else {
      s = Half / s;
      q[3] = (m.data[j][k] - m.data[k][j]) * s;
      q[j] = (m.data[j][i] + m.data[i][j]) * s;
      q[k] = (m.data[k][i] + m.data[k][i]) * s;
    }
    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];
  }
  Real n = norm();
  if(!FuzzyEquals(n,One)) {
    operator *= (1.0/n);
  }
  Matrix3 temp;
  getMatrix(temp);
  if(!temp.isEqual(m,1e-2)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Very different matrix in QuaternionRotation::setMatrix()!");
    LOG4CXX_ERROR(KrisLibrary::logger(),m<<" vs ");
    LOG4CXX_ERROR(KrisLibrary::logger(),temp);
    return false;
  }
  return true;
}

void QuaternionRotation::getAngleAxis(AngleAxisRotation& r) const
{
  r.angle = Two*Acos(w);
  Real imscale = One/imNorm();
  r.axis.set(x*imscale, y*imscale, z*imscale);
}

void QuaternionRotation::getMoment(MomentRotation& r) const
{
  if(w>=One||w<=-One) r.setZero();
  else {
    Real angle = Two*Acos(w);
    Real den=pythag_leg(w,One);
    if(FuzzyEquals(den,Zero)) {
      r.setZero();
    }
    else {
      Real imscale = angle/den;
      r.set(x*imscale, y*imscale, z*imscale);
    }
  }
}

void QuaternionRotation::getMatrix(Matrix3& m) const {
  Real wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

  // calculate coefficients
  x2 = x + x; y2 = y + y; z2 = z + z;
  xx = x * x2;   xy = x * y2;   xz = x * z2;
  yy = y * y2;   yz = y * z2;   zz = z * z2;
  wx = w * x2;   wy = w * y2;   wz = w * z2;

  m.data[0][0] = One - (yy + zz);    m.data[1][0] = xy - wz; 			m.data[2][0] = xz + wy;
  m.data[0][1] = xy + wz;            m.data[1][1] = One - (xx + zz);	m.data[2][1] = yz - wx;
  m.data[0][2] = xz - wy;            m.data[1][2] = yz + wx;			m.data[2][2] = One - (xx + yy);
}

void QuaternionRotation::transform(const Vector3& v, Vector3& out) const
{
  Quaternion qinv, x(0,v), tmp;
  qinv.setConjugate(*this);
  tmp.mul(x,*this);
  x.mul(qinv,tmp);
  out.x=x.x;
  out.y=x.y;
  out.z=x.z;
}

void QuaternionRotation::slerp(const Quaternion& a, const Quaternion& b, Real t)
{
  //a + b unit quaternions?
  /* angle theta is defined as
     cos(theta)*|a||b| = <a,b>
  */
  Real d = dot(a,b);
  Real bscale=One;
  if(d < 0) { bscale=-One; d=-d; }
  if(d > One) {
    if(d < One+angleEps) {  //axes are the same, there's some numerical error
      set(b);
      return;
    }
    LOG4CXX_ERROR(KrisLibrary::logger(),"QuaternionRotation::slerp(): dot product is > 1, are quaternions not normalized?");
    if(!FuzzyEquals(a.norm(),One,angleEps)) 
      LOG4CXX_ERROR(KrisLibrary::logger(),"   a is not normalized. Norm="<<a.norm());
    if(!FuzzyEquals(b.norm(),One,angleEps)) 
      LOG4CXX_ERROR(KrisLibrary::logger(),"   b is not normalized. Norm="<<b.norm());
    Quaternion na,nb;
    na.setNormalized(a);
    nb.setNormalized(b);
    slerp(na,nb,t);
    return;
  }
  
  if(d >= One-angleEps) { //verrry close, close enough for linear interp
    mul(a,(One-t));
    madd(b,bscale*t);
  }
  else {
    Real theta = Acos(d);
    Real sininv = Sin(theta);
    sininv = One/sininv;
    
    //this = (Sin((One-t)*theta)*sininv) * a +  (Sin(t*theta)*sininv) * b;
    Real a_coeff = Sin((One-t)*theta)*sininv;
    Real b_coeff = Sin(t*theta)*sininv;
    mul(a, a_coeff);
    madd(b, bscale*b_coeff);
  }
}

void QuaternionRotation::mag(const Quaternion& a, Real t)
{
  //just like slerp(identity transform, a)

  if(a.w == One) {	//a is identity
    set(a);
    return;
  }
  else if(a.w == -One) { //a is a rotation of 180 degrees
	LOG4CXX_ERROR(KrisLibrary::logger(),"QuaternionRotation.mag(): Quaternion is a rotation of 180 degrees");
    return;
  }


  Real theta = Acos(a.w);
  Real sininv = Sin(theta);
  sininv = One/sininv;

  Real a_coeff = Sin((One-t)*theta)*sininv;
  Real b_coeff = (Sin(t*theta)*sininv);
  mul(a, b_coeff);
  w += a_coeff;
}


void SLerp(const Quaternion& a,
	   const Quaternion& b,
	   Quaternion& out,
	   Real t)
{
  //a + b unit quaternions?
  /* angle theta is defined as
     cos(theta)*|a||b| = <a,b>
  */
  Real d = dot(a,b);
  if(d == One) {	//axes are the same axis
    out.set(b);
    return;
  }
  else if(d == -One) {	//axes are opposing axis
    LOG4CXX_ERROR(KrisLibrary::logger(),"SLerp(): Quaternions on opposing sides of unit sphere");
    return;
  }
  Real bscale = (d < 0 ? -One:One);

  Real theta = Acos(d);
  Real sininv = Sin(theta);
  sininv = One/sininv;

  //out = (Sin((One-t)*theta)*sininv) * a +  (Sin(t*theta)*sininv) * b;
  Real a_coeff = Sin((One-t)*theta)*sininv;
  Real b_coeff = Sin(t*theta)*sininv;
  out.mul(a, a_coeff);
  out.madd(b, bscale*b_coeff);
}

void SCerp(const Quaternion& q_1,
	   const Quaternion& q0,
	   const Quaternion& q1,
	   const Quaternion& q2,
	   Quaternion& out,
	   Real t)
{
  //calc control pts
  Quaternion c0,c1;
  Quaternion p,q;

  static const Real Third = (Real)0.3333333333333333333333333333;
  SLerp(q0, q_1, p, -1.0);
  SLerp(p, q1, q, 0.5);
  SLerp(q0, q, c0, Third);

  SLerp(q1, q2, p, -1.0);
  SLerp(p, q0, q, 0.5);
  SLerp(q1, q, c1, Third);

  SBezier(q0, c0, c1, q1, out, t);
}

void SBezier(const Quaternion& q0,
	     const Quaternion& c0,
	     const Quaternion& c1,
	     const Quaternion& q1,
	     Quaternion& out,
	     Real t)
{
  Quaternion a,b,c,d,e;
  SLerp(q0,c0, a,t);
  SLerp(c0,c1, b,t);
  SLerp(c1,q1, c,t);

  SLerp(a,b, d,t);
  SLerp(b,c, e,t);

  SLerp(d,e, out,t);
}



void SetMatrixRotationZYX(Matrix3& m, const Vector3& v)
{
	EulerAngleRotation e(v);
	e.getMatrixZYX(m);
}

void SetMatrixRotationZYX(Matrix4& m, const Vector3& v)
{
	Matrix3 m3;
	SetMatrixRotationZYX(m3,v);
	m.set(m3);
}

void SetMatrixRotationVector(Matrix3& m, const Vector3& v)
{
	MomentRotation r(v);
	r.getMatrix(m);
}

void SetMatrixRotationVector(Matrix4& m, const Vector3& v)
{
	Matrix3 m3;
	SetMatrixRotationVector(m3,v);
	m.set(m3);
}

void SetMatrixRotationVector(Matrix3& m, const Vector3& v, Real angle)
{
	AngleAxisRotation r(angle,v);
	r.getMatrix(m);
}

void SetMatrixRotationVector(Matrix4& m, const Vector3& v, Real angle)
{
	Matrix3 m3;
	SetMatrixRotationVector(m3,v,angle);
	m.set(m3);
}

void SetMatrixRotationQuaternion(Matrix3& m, const Quaternion& q)
{
	QuaternionRotation r(q);
	r.getMatrix(m);
}

void SetMatrixRotationQuaternion(Matrix4& m, const Quaternion& q)
{
	Matrix3 m3;
	SetMatrixRotationQuaternion(m3,q);
	m.set(m3);
}

} //namespace Math3D
