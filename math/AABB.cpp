#include <KrisLibrary/Logger.h>
#include "AABB.h"
#include <utils/AnyValue.h>

namespace Math {

int AABBLineSearch(const Vector& x0,const Vector& dx,const Vector& bmin,const Vector& bmax,Real& t)
{
  Assert(x0.n == dx.n);
  Assert(x0.n == bmin.n);
  Assert(x0.n == bmax.n);
  int res=-1;
  for(int i=0;i<bmax.n;i++) {
    Assert(x0(i) <= bmax(i));
    Assert(x0(i) >= bmin(i));
    if(x0(i) + t*dx(i) > bmax(i)) {
      //if x0 + t*dx = bmax-e
      //then x0+t*dx = bmin+e
      //what if dx is small?
      //t = (bmax-x0-e)/dx+e2;
      //=> x0+t dx = x0 + (bmax-x0-e) + e2 dx
      //           = (bmax-e) + e2 dx
      t = (bmax(i)-x0(i))/dx(i)*(1 - Epsilon);
      res = i;
    }
    if(x0(i) + t*dx(i) < bmin(i)) {
      //if x0 + t*dx = bmin+e
      //then x0+t*dx = bmin+e
      t = (bmin(i)-x0(i))/dx(i)*(1 - Epsilon);
      res = i;
    }
    if(!(x0(i)+t*dx(i) <= bmax(i))) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error: "<<i<<": "<<x0(i)<<"+"<<t<<"*"<<dx(i)<<"="<<x0(i)+t*dx(i)<<" <= "<<bmax(i));
    }
    Assert(x0(i)+t*dx(i) <= bmax(i));
    if(!(x0(i)+t*dx(i) >= bmin(i))) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error: "<<i<<": "<<x0(i)<<"+"<<t<<"*"<<dx(i)<<"="<<x0(i)+t*dx(i)<<" >= "<<bmin(i));
    }
    Assert(x0(i)+t*dx(i) >= bmin(i));
  }
  return res;

}

inline bool ClipLine1D(Real q, Real p, Real& umin, Real& umax)
{
   Real r;
   if(p<0) {			//entering
     r=-q/p;
     if(r > umax) return false;
     if(r > umin) umin = r;
   }
   else if(p>0) {
     r=-q/p;
     if(r < umin) return false;
     if(r < umax) umax = r;
   }
   else {
     if(q>0) return false;
   }
   return true;
}

bool AABBClipLine(const Vector& x0,const Vector& dx,
		  const Vector& bmin,const Vector& bmax,
		  Real& u0,Real& u1)
{
  Assert(x0.n == dx.n);
  Assert(x0.n == bmin.n);
  Assert(x0.n == bmax.n);
  for(int i=0;i<x0.n;i++) {
    //for each face, p is dot(dx, normal), q is signed dist to plane (dot(v,normal)-offset)
    if(!ClipLine1D(bmin(i) - x0(i), -dx(i), u0,u1)) return false;
    if(!ClipLine1D(x0(i) - bmax(i), dx(i), u0,u1)) return false;
  }
  return true;
}

} //namespace Math
