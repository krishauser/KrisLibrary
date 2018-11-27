#include <KrisLibrary/Logger.h>
#include "AngleBracket.h"
#include <math/misc.h>
#include <math3d/basis.h>
#include "errors.h"


//Simplest form, assumes p and the disk D lie on the x axis
//(Actually in 2d)
//Point is (p,0).  Disk's center is (c,0), radius is r.
AngleInterval AngleBracket_1D_Disk(Real p, Real c, Real r)
{
  Assert(p >= 0);
  AngleInterval i;
  //if the range of p is included in D, return whole circle
  if(Abs(c) <= r-p)
    i.setCircle();
  //if the circle and disk don't intersect, return empty set
  else if(!(Abs(c) <= r+p) || !(p <= Abs(c)+r)) { //this takes care of NAN's
    i.setEmpty();
    //LOG4CXX_INFO(KrisLibrary::logger(),"AngleBracket_1D_Disk: Empty");
    //LOG4CXX_INFO(KrisLibrary::logger(),i.c<<" -> "<<i.d);
  }
  else {   //otherwise, calculate the intersection
    Real t = (p*p-r*r+c*c)*Half/c;
    if(!(Abs(t) <= Abs(p))) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR!: calculated t="<<t<<", p="<<p<<", c="<<c<<", r="<<r);
    }
    Assert(Abs(t) <= Abs(p));
    Assert(Abs(t) != Zero);
    Real theta = Acos(t/p);
    i.setRange(-theta,theta);
    //LOG4CXX_INFO(KrisLibrary::logger(),"calculated theta="<<theta);
  }
  return i;
}

AngleInterval AngleBracket_2D_Disk(const Vector2& p, const Vector2& c, Real r)
{
  //polar coordinates of p and c
  Real ptheta,pr;
  Real ctheta,cr;
  pr = p.norm();
  cr = c.norm();
  if(pr == Zero) {    //point is at origin
    AngleInterval i;
    if(cr <= r) i.setCircle();
    else {
      i.setEmpty();
      i.d = 0;
      //LOG4CXX_INFO(KrisLibrary::logger(),"AngleBracket_2D_Disk: Empty 1");
      //LOG4CXX_INFO(KrisLibrary::logger(),i.c<<" -> "<<i.d);
    }
    return i;
  }
  else if(cr == Zero) {  //circle is at origin
    AngleInterval i;
    if(pr <= r) i.setCircle();
    else {
      i.setEmpty();
      i.d = 0;
      //LOG4CXX_INFO(KrisLibrary::logger(),"AngleBracket_2D_Disk: Empty 2");
      //LOG4CXX_INFO(KrisLibrary::logger(),i.c<<" -> "<<i.d);
    }
    return i;
  }
  else {
    AngleInterval i = AngleBracket_1D_Disk(pr,cr,r);
    ptheta = Atan2(p.y,p.x);
    ctheta = Atan2(c.y,c.x);
    if(!i.isEmpty())
      i.inplaceShift(ctheta-ptheta);
    else {
      i.d=ctheta-ptheta;
      //LOG4CXX_INFO(KrisLibrary::logger(),"AngleBracket_2D_Disk: Empty 3");
      //LOG4CXX_INFO(KrisLibrary::logger(),i.c<<" -> "<<i.d);
      //LOG4CXX_INFO(KrisLibrary::logger(),"p: "<<p<<", c: "<<c);
    }
    return i;
  }
}

//p rotates around the z axis.  The ball B is centered at c, with radius r.
AngleInterval AngleBracket_3D_Ball(const Vector3& p, const Vector3& c, Real r)
{
  //LOG4CXX_INFO(KrisLibrary::logger(),"3d ball bracket around z: "<<p<<", "<<c<<", "<<r);
  Real d = c.z-p.z;
  if(Abs(d) > r) {
    AngleInterval i;
    i.setEmpty();
    //LOG4CXX_INFO(KrisLibrary::logger(),"AngleBracket_3D_Ball: Empty 3");
    if(c.x==0 && c.y==0)
      i.d=0;
    else if(p.x==0 && p.y==0)
      i.d=0;
    else {
      Real ptheta = Atan2(p.y,p.x);
      Real ctheta = Atan2(c.y,c.x);
      i.d=ctheta-ptheta;
    }
    //LOG4CXX_INFO(KrisLibrary::logger(),i.c<<" -> "<<i.d);
    return i;
  }
  //slice the sphere with the x,y plane through p.z
  Real r2 = pythag_leg(d,r);
  //LOG4CXX_INFO(KrisLibrary::logger(),"r2 "<<r2);
  return AngleBracket_2D_Disk(Vector2(p.x,p.y),Vector2(c.x,c.y),r2);
}

//same as above, but axis w is arbitrary
AngleInterval AngleBracket_3D_Ball(const Vector3& p, const Vector3& w, const Vector3& c, Real r)
{
  Assert(FuzzyEquals(w.normSquared(),One));
  Vector3 x,y;
  GetCanonicalBasis(w,x,y);
  Vector3 p2,c2;
  p2.set(dot(x,p),dot(y,p),dot(w,p));
  c2.set(dot(x,c),dot(y,c),dot(w,c));
  return AngleBracket_3D_Ball(p2,c2,r);
}

AngleInterval AngleBracket_3D_Ball(const Vector3& p, int axis, const Vector3& c, Real r)
{
  switch(axis) {
  case 0:
    return AngleBracket_3D_Ball(Vector3(p.y,p.z,p.x),Vector3(c.y,c.z,c.x),r);
  case 1:
    return AngleBracket_3D_Ball(Vector3(p.z,p.x,p.y),Vector3(c.z,c.x,c.y),r);
  case 2:
    return AngleBracket_3D_Ball(p,c,r);
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error: Invalid axis "<< axis);
    break;
  }
  AngleInterval empty; empty.setEmpty();
  return empty;
}
