#include <KrisLibrary/Logger.h>
#include "angle.h"
#include "misc.h"
#include "utils.h"
#include <errors.h>
#include <stdio.h>

#include <iostream>
using namespace std;

namespace Math {

Real AngleNormalize(Real a)
{
  Real an=Mod(a,TwoPi);
  if(an < 0) return an+TwoPi;
  return an;
}

Real AngleDiff(Real a, Real b)
{
  Real diff=a-b;
  if(diff < -Pi) return diff+TwoPi;
  if(diff > Pi) return diff-TwoPi;
  return diff;
}

Real AngleCCWDiff(Real a, Real b)
{
  if(a >= b) return a-b;
  else return a+TwoPi-b;
}

Real AngleInterp(Real a, Real b, Real u)
{
  Real d = AngleDiff(b,a);
  Assert(d >= -Pi && d <= Pi);
  return AngleNormalize(a+u*d);
}

inline bool IsValidAngle(Real a)
{
  return a>=Zero && a<=TwoPi;
}


void AngleInterval::setRange(Real a, Real b)
{
  c = a; 
  d = AngleCCWDiff(b,a);
}

void AngleInterval::setMinRange(Real a, Real b)
{
  d = AngleDiff(a,b);
  if(d<0) { c=a; d=-d; }
  else  c=b;
}

void AngleInterval::setIntersection(const AngleInterval& i1, const AngleInterval& i2)
{
  if(i1.isFull()) {
    c = i2.c;
    d = i2.d;
  }
  else if(i2.isFull()) {
    c = i1.c;
    d = i1.d;
  }
  else if(i1.contains(i2.c)) {
    c = i2.c;
    d = Min(i2.d,i1.d-AngleCCWDiff(i2.c,i1.c));
  }
  else if(i2.contains(i1.c)) {
    c = i1.c;
    d = Min(i1.d,i2.d-AngleCCWDiff(i1.c,i2.c));
  }
  else
    setEmpty();
  if(!i1.contains(*this)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in i1\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Intersect "<<i1.c<<"->"<<i1.d<<" with "<<i2.c<<"->"<<i2.d);
    LOG4CXX_INFO(KrisLibrary::logger(),"Result: "<<c<<"->"<<d);
  }
  if(!i2.contains(*this)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in i2\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Intersect "<<i1.c<<"->"<<i1.d<<" with "<<i2.c<<"->"<<i2.d);
    LOG4CXX_INFO(KrisLibrary::logger(),"Result: "<<c<<"->"<<d);
  }
  Assert(i1.contains(*this));
  Assert(i2.contains(*this));
}

void AngleInterval::setUnion(const AngleInterval& i1, const AngleInterval& i2)
{
  if(i1.isEmpty()) {
    c=i2.c;
    d=i2.d;
  }
  else if(i2.isEmpty()) {
    c=i1.c;
    d=i1.d;
  }
  else if(i1.contains(i2.c)) {
    c = i1.c;
    d = Max(i1.d,i2.d+AngleCCWDiff(i2.c,i1.c));
  }
  else if(i2.contains(i1.c)) {
    c = i2.c;
    d = Max(i2.d,i1.d+AngleCCWDiff(i1.c,i2.c));
  }
  else {  //we have 2 choices, pick the smaller interval
    Real d12 = AngleCCWDiff(i1.c+i1.d,i2.c);
    Real d21 = AngleCCWDiff(i2.c+i2.d,i1.c);
    if(d12 < d21) {
      c = i2.c;
      d = d12;
    }
    else {
      c = i1.c;
      d = d21;
    }
  }
}

void AngleInterval::setCompliment(const AngleInterval& i)
{
  c = AngleNormalize(i.c+i.d);
  d = TwoPi-i.d;
}

void AngleInterval::inplaceCompliment()
{
  c = AngleNormalize(c+d);
  d = TwoPi-d;
}

void AngleInterval::normalize()
{
  if (d<0) {
    c=AngleNormalize(c+d);
    d=-d;
  }
  else
    c=AngleNormalize(c);
}

Real AngleInterval::clamp(Real x) const
{
  Real dc=AngleCCWDiff(x,c);
  if(dc <= d) return x;
  Real da=AngleCCWDiff(c,x);
  Real db=AngleCCWDiff(x,AngleNormalize(c+d));
  if(da < db) return c;
  else return AngleNormalize(c+d);
}

bool AngleInterval::contains(const AngleInterval& i) const
{
  if(i.isEmpty()) return true;
  if(isFull()) return true;
  if(isEmpty()) return false;
  if(i.d > d) return false;
  //tricky at pi
  if(i.d == Pi && d == Pi) return c==i.c;
  Real delta1 = AngleCCWDiff(i.c,c);
  Real delta2 = AngleCCWDiff(AngleNormalize(i.c+i.d),c);
  return delta1 <= d+Epsilon && delta2 <=d+Epsilon; 
}

bool AngleInterval::contains(Real a) const
{
  if(isEmpty()) return false;
  Real delta = AngleCCWDiff(a,c);
  return delta <= d;
}

bool AngleInterval::intersects(const AngleInterval& i) const
{
  return contains(i.c) || i.contains(c);
}

void AngleInterval::setCosGreater(Real y)
{
  if(y >= One) setEmpty();
  else if(y <= -One) setCircle();
  else {
    c=Acos(y);
    d=c+c;
    c=AngleNormalize(-c);
  }
}

void AngleInterval::setSinGreater(Real y)
{
  if(y > One) setEmpty();
  else if(y <= -One) setCircle();
  else {
    c=Asin(y);
    d=Pi-c;
    c=AngleNormalize(c);
  }
}

void AngleInterval::setCosLess(Real y)
{
  if(y < -One) setEmpty();
  else if(y >= One) setCircle();
  else {
    c=Acos(y);
    d=TwoPi-Two*c;
  }
}

void AngleInterval::setSinLess(Real y)
{
  if(y < -One) setEmpty();
  else if(y >= One) setCircle();
  else {
    c=Asin(y);
    d=Pi+c;
    c=Pi-c;
  }
}

void TransformCosSin_Cos(Real a,Real b,Real& c,Real& d)
{
  //use cos(x+d) = cos(x)cos(d) - sin(x)sin(d)
  //=> a=c*cos(d), b=-c*sin(d)
  //=>c^2 = a^2+b^2
  if(a==0 && b==0) { c=d=0; }
  else {
    d = Atan2(-b,a);
    c = pythag(a,b);
  }
  Real x=0.5;
  if(!FuzzyEquals(c*Cos(x+d),a*Cos(x)+b*Sin(x))) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in TransformCosSin\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"a: "<<a<<", b: "<<b);
    LOG4CXX_INFO(KrisLibrary::logger(),"c: "<<c<<", d: "<<d);
    LOG4CXX_INFO(KrisLibrary::logger(),"f(x): "<<a*Cos(x)+b*Sin(x));
    LOG4CXX_INFO(KrisLibrary::logger(),"g(x): "<<c*Cos(x+d));
  }
  Assert(FuzzyEquals(c*Cos(x+d),a*Cos(x)+b*Sin(x)));
}

void TransformCosSin_Sin(Real a,Real b,Real& c,Real& d)
{
  //use sin(x+d) = sin(x)cos(d) + cos(x)sin(d)
  //=> a=c*sin(d), b=c*cos(d)
  //=> c^2 = a^2+b^2
  if(a==0 && b==0) { c=d=0; }
  else {
    d = Atan2(a,b);
    c = pythag(a,b);
  }
  Real x=0.5;
  if(!FuzzyEquals(c*Sin(x+d),a*Cos(x)+b*Sin(x))) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in TransformCosSin\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"a: "<<a<<", b: "<<b);
    LOG4CXX_INFO(KrisLibrary::logger(),"c: "<<c<<", d: "<<d);
    LOG4CXX_INFO(KrisLibrary::logger(),"f(x): "<<a*Cos(x)+b*Sin(x));
    LOG4CXX_INFO(KrisLibrary::logger(),"g(x): "<<c*Sin(x+d));
  }
  Assert(FuzzyEquals(c*Sin(x+d),a*Cos(x)+b*Sin(x)));
}

bool SolveCosSinEquation(Real a,Real b,Real c,Real& t1,Real& t2)
{
  if(a==0 && b==0) {
    if(c==0) { t1=0; t2=TwoPi; return true; }
    return false;
  }
  else {
    Real cs,ds;
    TransformCosSin_Sin(a,b,cs,ds);
    //cs*sin(x+ds)=c  =>  x=asin(c/cs)-ds
    //or pi-asin(c/cs)-ds
    if(c > cs || c < -cs) return false;
    t1 = Asin(c/cs);
    t2 = Pi-t1;
    t1 -= ds;
    t2 -= ds;
    Assert(FuzzyEquals(a*Cos(t1)+b*Sin(t1),c));
    Assert(FuzzyEquals(a*Cos(t2)+b*Sin(t2),c));
  }
  return true;
}

void SolveCosSinGreater(Real a,Real b,Real c,AngleInterval& i)
{
  Real scale,ofs;
  TransformCosSin_Cos(a,b,scale,ofs);
  i.setCosLess(c/scale);
  i.inplaceShift(-ofs);
}


} //namespace Math
