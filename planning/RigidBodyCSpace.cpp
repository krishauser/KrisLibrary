#include "RigidBodyCSpace.h"
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math3d/random.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/math/angle.h>
using namespace Math3D;
using namespace std;


R2CSpace::R2CSpace(const Vector2& bmin,const Vector2& bmax)
:BoxCSpace(Vector(2,bmin),Vector(2,bmax))
{}

std::string R2CSpace::VariableName(int i)
{
  if(i==0) return "x";
  return "y";
}

void R2CSpace::SetDomain(const Vector2& bmin,const Vector2& bmax)
{
  BoxCSpace::SetDomain(Vector(2,bmin),Vector(2,bmax));
}

void R2CSpace::GetDomain(Vector2& _bmin,Vector2& _bmax)
{
  _bmin.set(bmin[0],bmin[1]);
  _bmax.set(bmax[0],bmax[1]);
}


R3CSpace::R3CSpace(const Vector3& bmin,const Vector3& bmax)
:BoxCSpace(Vector(3,bmin),Vector(3,bmax))
{}

std::string R3CSpace::VariableName(int i)
{
  if(i==0) return "x";
  else if(i==1) return "y";
  return "z";
}

void R3CSpace::SetDomain(const Vector3& bmin,const Vector3& bmax)
{
  BoxCSpace::SetDomain(Vector(3,bmin),Vector(3,bmax));
}

void R3CSpace::GetDomain(Vector3& _bmin,Vector3& _bmax)
{
  _bmin.set(bmin[0],bmin[1],bmin[2]);
  _bmax.set(bmax[0],bmax[1],bmax[2]);
}


std::string SO2CSpace::VariableName(int i)
{
  return "theta";
}


void SO2CSpace::Sample(Config& x)
{
  x.resize(1);
  x(0) = Rand(0,TwoPi);
}

void SO2CSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  CSpace::SampleNeighborhood(c,r,x);
  x[0] = AngleNormalize(x[0]);
}

void SO2CSpace::Interpolate(const Config& a,const Config& b,Real u,Config& out)
{
  /*
  if(a[0] != AngleNormalize(a[0]))
    printf("SO2CSpace::Interpolate: Warning, point a is out of range: %g to %g\n",a[0],b[0]);
  if(b[0] != AngleNormalize(b[0]))
    printf("SO2CSpace::Interpolate: Warning, point b is out of range: %g to %g\n",a[0],b[0]);
  */
  out.resize(1);
  out(0)=AngleInterp(AngleNormalize(a(0)),AngleNormalize(b(0)),u);
}

Real SO2CSpace::Distance(const Config& a,const Config& b)
{
  /*
  if(a[0] != AngleNormalize(a[0]))
    printf("SO2CSpace::Distance: Warning, point a is out of range: %g to %g\n",a[0],b[0]);
  if(b[0] != AngleNormalize(b[0]))
    printf("SO2CSpace::Distance: Warning, point b is out of range: %g to %g\n",a[0],b[0]);
  */
  return Abs(AngleDiff(AngleNormalize(a(0)),AngleNormalize(b(0))));
}

void SO2CSpace::Properties(PropertyMap& pmap)
{
  pmap.set("cartesian",0);
  pmap.set("geodesic",1);
  pmap.set("metric","so2");
  pmap.set("volume",TwoPi);
  pmap.setArray("minimum",vector<double>(1,0));
  pmap.setArray("maximum",vector<double>(1,TwoPi));
}


void SO2CSpace::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx)
{
  dx.resize(1);
  dx[0] = AngleDiff(a(0),b(0));
}

void SO2CSpace::Integrate(const Config& a,const Vector& da,Config& b)
{ 
  b = a+da;
  b[0] = AngleNormalize(b[0]);
}

void SO2CSpace::GetRotation(const Config& x,Math3D::Matrix2& R)
{
  R.setRotate(x[0]);
}

void SO2CSpace::SetRotation(const Math3D::Matrix2& R,Config& x)
{
  x.resize(1);
  x[0] = Atan2(R(0,0),R(1,0));
}
  
SE2CSpace::SE2CSpace(Real bmin,Real bmax)
:MultiCSpace(make_shared<R2CSpace>(bmin,bmax),make_shared<SO2CSpace>())
{}

SE2CSpace::SE2CSpace(const Math3D::Vector2& bmin,const Math3D::Vector2& bmax)
:MultiCSpace(make_shared<R2CSpace>(bmin,bmax),make_shared<SO2CSpace>())
{}


void SE2CSpace::GetTransform(const Config& x,Math3D::RigidTransform2D& T)
{
  T.t.set(x[0],x[1]);
  T.R.setRotate(x[2]);
}

void SE2CSpace::SetTransform(const Math3D::RigidTransform2D& T,Config& x)
{
  x.resize(3);
  T.t.get(x[0],x[1]);
  x[2] = Atan2(T.R(0,0),T.R(1,0));
}

void SE2CSpace::SetAngleWeight(Real weight)
{
  distanceWeights.resize(2);
  distanceWeights[0] = 1;
  distanceWeights[1] = weight;
}

void SE2CSpace::SetDomain(const Math3D::Vector2& bmin,const Math3D::Vector2& bmax)
{
  R2CSpace* box = dynamic_cast<R2CSpace*>(&*components[0]);
  box->SetDomain(bmin,bmax);
}


std::string SO3CSpace::VariableName(int i)
{
  if(i==0) return "R_x";
  else if(i==1) return "R_y";
  return "R_z";
}


void SO3CSpace::Sample(Config& x)
{
  QuaternionRotation q = RandRotation();
  Matrix3 R;
  q.getMatrix(R);
  SetRotation(R,x);
}

void SO3CSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  AngleAxisRotation aa;
  aa.angle = Rand(0,r);
  SampleSphere(1.0,aa.axis);
  Matrix3 R,dR;
  aa.getMatrix(R);
  GetRotation(c,R);
  SetRotation(R*dR,x);
}

void SO3CSpace::Interpolate(const Config& a,const Config& b,Real u,Config& out)
{
  Matrix3 Ra,Rb,Rout;
  GetRotation(a,Ra);
  GetRotation(b,Rb);
  interpolateRotation(Ra,Rb,u,Rout);
  SetRotation(Rout,out);
}

Real SO3CSpace::Distance(const Config& a,const Config& b)
{
  Matrix3 Ra,Rb;
  GetRotation(a,Ra);
  GetRotation(b,Rb);
  Matrix3 Rrel;
  Rrel.mulTransposeB(Ra,Rb);
  AngleAxisRotation aa;
  aa.setMatrix(Rrel);
  return Abs(aa.angle);
}

void SO3CSpace::Properties(PropertyMap& pmap)
{
  pmap.set("cartesian",0);
  pmap.set("geodesic",1);
  pmap.set("metric","so3");
  pmap.set("volume",Pow(Pi,3.0));
  pmap.setArray("minimum",vector<double>(3,-Pi));
  pmap.setArray("maximum",vector<double>(3,Pi));
  pmap.set("diameter",TwoPi);
}


void SO3CSpace::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx)
{
  FatalError("Not done yet\n");
}

void SO3CSpace::Integrate(const Config& a,const Vector& da,Config& b)
{ 
  Matrix3 Ra,Rd;
  GetRotation(a,Ra);
  GetRotation(da,Rd);
  Matrix3 Rb = Ra*Rd;
  SetRotation(Rb,b);
}

void SO3CSpace::GetRotation(const Config& x,Math3D::Matrix3& R)
{
  MomentRotation m(x(0),x(1),x(2));
  m.getMatrix(R);
}

void SO3CSpace::SetRotation(const Math3D::Matrix3& R,Config& x)
{
  MomentRotation m;
  m.setMatrix(R);
  x.resize(3);
  m.get(x(0),x(1),x(2));
}



SE3CSpace::SE3CSpace(Real bmin,Real bmax)
:MultiCSpace(make_shared<R3CSpace>(bmin,bmax),make_shared<SO3CSpace>())
{}

SE3CSpace::SE3CSpace(const Math3D::Vector3& bmin,const Math3D::Vector3& bmax)
:MultiCSpace(make_shared<R3CSpace>(bmin,bmax),make_shared<SO3CSpace>())
{}


void SE3CSpace::GetTransform(const Config& x,Math3D::RigidTransform& T)
{
  T.t.set(x[0],x[1],x[2]);
  Config xr;
  xr.setRef(x,3,1,3);
  SO3CSpace::GetRotation(xr,T.R);
}

void SE3CSpace::SetTransform(const Math3D::RigidTransform& T,Config& x)
{
  x.resize(6);
  T.t.get(x[0],x[1],x[2]);
  Config xr;
  xr.setRef(x,3,1,3);
  SO3CSpace::SetRotation(T.R,xr);
}

void SE3CSpace::SetAngleWeight(Real weight)
{
  distanceWeights.resize(2);
  distanceWeights[0] = 1;
  distanceWeights[1] = weight;
}

void SE3CSpace::SetDomain(const Math3D::Vector3& bmin,const Math3D::Vector3& bmax)
{
  R3CSpace* box = dynamic_cast<R3CSpace*>(&*components[0]);
  box->SetDomain(bmin,bmax);
}
