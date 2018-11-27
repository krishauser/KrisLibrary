#include <KrisLibrary/Logger.h>
#include "Line3D.h"
#include "Segment3D.h"
#include "clip.h"
#include "misc.h"
#include "errors.h"
#include <iostream>
using namespace Math3D;
using namespace std;

bool Line3D::Read(File& f)
{
	if(!source.Read(f)) return false;
	if(!direction.Read(f)) return false;
	return true;
}

bool Line3D::Write(File& f) const
{
	if(!source.Write(f)) return false;
	if(!direction.Write(f)) return false;
	return true;
}

void Line3D::setPoints(const Point3D& a, const Point3D& b)
{
	source = a;
	direction.sub(b,a);
}

void Line3D::setSegment(const Segment3D& s)
{
	source = s.a;
	direction.sub(s.b,s.a);
}

void Line3D::setTransformed(const Line3D& l, const Matrix4& xform)
{
	xform.mulPoint(l.source,source);
	xform.mulVector(l.direction,direction);
}

void Line3D::eval(Real t, Point3D& out) const
{
	out = source;
	out.madd(direction,t);
}

Real Line3D::closestPointParameter(const Point3D& in) const
{
	Real denom = dot(direction,direction);
	if(denom == Zero) return Zero;
	return dot(in-source,direction)/denom;
}

Real Line3D::closestPoint(const Point3D& in, Point3D& out) const
{
	Real t=closestPointParameter(in);
	eval(t,out);
	return t;
}

Real Line3D::closestPoint(const Point3D& in, Point3D& out, Real tmin, Real tmax) const
{
	Real denom = dot(direction,direction);
	Real numer = dot(in-source,direction);
	//t = numer/denom with denom >= 0
	Real t;
	if(numer<=tmin*denom) t=tmin;
	else if(numer>=tmax*denom) t=tmax;
	else t = numer/denom;
	eval(t,out);
	return t;
}

Real Line3D::distance(const Point3D& pt) const
{
  Point3D closest;
  closestPoint(pt,closest);
  return (pt-closest).norm();
}

//a generalized line test for rays a + t*as, b + u*bs
bool Line3D::intersects(const Line3D& l, Real* t, Real* u, Real epsilon) const
{
	//take the vector normal to both lines, project their offsets onto the lines
	//if they are coplanar, do some more checking
	Vector3 n = cross(direction,l.direction);
	Vector3 local = l.source - source;

	if(n.isZero())	//a,b parallel
	{
		//project l onto this line (in local coords)
		Real projDist = dot(local,direction)/dot(direction,direction);
		if (DistanceLEQ(local,projDist*direction,epsilon)) {
			if(t) *t=projDist;
			if(u) *u=0;
			return true;
		}
		return false;
	}

	if(Abs(dot(n, local))<=epsilon)
	{
		//get the coordinates of "shift" on the plane
		//an orthogonal basis is B={a.slope, n*a.slope}
		//get bslope' and boffset' in B coordinates
		//bslope' = B = (b1,b2) = (dot(bslope,aslope)/dot(aslope,aslope), dot(bslope,na)/dot(na,na))
		//boffset' = A = (a1,a2) = (dot(bofs,aslope)/dot(aslope,aslope), dot(bofs,na)/dot(na,na))
		//get an equation R = A + Bt
		//get the t value of the intersection point with the x axis (x,0), 0 = a2 + b2*t, t = -a2/b2 = dot(bofs,na)/dot(bslope,na)

		Vector3 na = cross(n,direction);
		Real myt = -dot(local,na)/dot(l.direction,na);
		if(t)
		{
			*t = myt;
		}
		if(u)
		{
			Real al2=Inv(dot(direction,direction));
			Real a1,b1;
			a1 = dot(local,direction)*al2;
			b1 = dot(l.direction,direction)*al2;
			*u = a1 + b1*myt;
		}
		return true;
	}
	return false;
}

void Line3D::closestPoint(const Line3D& l, Real& t, Real& u) const
{
  //take the vector normal to both lines, project their offsets onto the lines
  //if they are coplanar, do some more checking
  Vector3 n = cross(direction,l.direction);
  Vector3 local = l.source - source;
  
  if(n.isZero()) { //a,b parallel
    //project l onto this line (in local coords)
    t = dot(local,direction)/dot(direction,direction);
    u = 0;
    return;
  }
  
  //get the coordinates of "shift" on the plane
  //an orthogonal basis is B={a.slope, n*a.slope}
  //get bslope' and boffset' in B coordinates
  //bslope' = B = (b1,b2) = (dot(bslope,aslope)/dot(aslope,aslope), dot(bslope,na)/dot(na,na))
  //boffset' = A = (a1,a2) = (dot(bofs,aslope)/dot(aslope,aslope), dot(bofs,na)/dot(na,na))
  //get an equation R = A + Bt
  //get the t value of the intersection point with the x axis (x,0), 0 = a2 + b2*t, t = -a2/b2 = dot(bofs,na)/dot(bslope,na)

  //THIS IS MESSED UP
  /*
  Vector3 na = cross(n,direction);
  t = -dot(local,na)/dot(l.direction,na);

  Real al2=Inv(dot(direction,direction));
  Real a1,b1;
  a1 = dot(local,direction)*al2;
  b1 = dot(l.direction,direction)*al2;
  u = a1 + b1*t;
  */
  Matrix2 AtA,AtAinv;
  AtA(0,0) = dot(direction,direction);
  AtA(1,0) = -dot(l.direction,direction);
  AtA(0,1) = -dot(l.direction,direction);
  AtA(1,1) = dot(l.direction,l.direction);
  bool res=AtA.getInverse(AtAinv);
  if(!res) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, Line3D closest points matrix inverse failed\n");
    t=u=0;
    return;
  }
  Vector2 tu = AtAinv*Vector2(dot(direction,local),-dot(l.direction,local));
  t = tu.x;
  u = tu.y;
}

void Line3D::getAABB(AABB3D& bb,Real tmin,Real tmax) const
{
	Point3D a,b;
	eval(tmin,a);
	eval(tmax,b);
	bb.setPoint(a);
	bb.expand(b);
}

bool Line3D::lineIntersects(const AABB3D& bb) const
{
	Real u1=-Inf,u2=Inf;
	return intersects(bb,u1,u2);
}

bool Line3D::rayIntersects(const AABB3D& bb) const
{
	Real u1=0,u2=Inf;
	return intersects(bb,u1,u2);
}

bool Line3D::intersects(const AABB3D& bb, Real& u1, Real& u2) const
{
  return ClipLine(source, direction, bb, u1,u2);
}

Real Line3D::distance(const AABB3D& bb) const
{
  Real tclosest;
  Vector3 bbclosest;
  return distance(bb,tclosest,bbclosest);
}

Real Line3D::distance(const AABB3D& bb, Real& tclosest, Vector3& bbclosest) const
{
  Real tmin=-Inf,tmax=Inf;
  if(intersects(bb,tmin,tmax)) {
    tclosest = tmin;
    eval(tmin,bbclosest);
    return 0;
  }
  //recompute matrices to get distances between axis-aligned segments to this
  Matrix2 AtA,Ax,Ay,Az;
  AtA(0,0) = dot(direction,direction);
  AtA(0,1) = AtA(1,0) = -direction.x;
  AtA(1,1) = 1;
  bool res=AtA.getInverse(Ax);
  if(!res) Ax.setZero();
  AtA(0,1) = AtA(1,0) = -direction.y;
  res=AtA.getInverse(Ay);
  if(!res) Ay.setZero();
  AtA(0,1) = AtA(1,0) = -direction.z;
  res=AtA.getInverse(Az);
  if(!res) Az.setZero();
  Vector3 bmin = bb.bmin - source, bmax = bb.bmax - source;
  //for an x-aligned segment, (t,u)^T = Ax*(direction^T x,-bmin.x)^T gives the
  //parameter on this segment t and the x value of the point bmin.x+u
  Real dxmin=direction.x*bmin.x,dxmax=direction.x*bmax.x;
  Real dymin=direction.y*bmin.y,dymax=direction.y*bmax.y;
  Real dzmin=direction.z*bmin.z,dzmax=direction.z*bmax.z;
  Real dps [8] = {dzmin+dymin+dxmin,
		  dzmin+dymin+dxmax,
		  dzmin+dymax+dxmin,
		  dzmin+dymax+dxmax,
		  dzmax+dymin+dxmin,
		  dzmax+dymin+dxmax,
		  dzmax+dymax+dxmin,
		  dzmax+dymax+dxmax};
  Real bxt = -Ax(0,1)*bmin.x;
  Real bxu = -Ax(1,1)*bmin.x;
  Real byt = -Ay(0,1)*bmin.y;
  Real byu = -Ay(1,1)*bmin.y;
  Real bzt = -Az(0,1)*bmin.z;
  Real bzu = -Az(1,1)*bmin.z;
  Real tx[4] ={ Ax(0,0)*dps[0]+bxt,
		Ax(0,0)*dps[2]+bxt,
		Ax(0,0)*dps[4]+bxt,
		Ax(0,0)*dps[6]+bxt};
  Real ux[4] ={ Ax(1,0)*dps[0]+bxu,
		Ax(1,0)*dps[2]+bxu,
		Ax(1,0)*dps[4]+bxu,
		Ax(1,0)*dps[6]+bxu};
  Real ty[4] ={ Ay(0,0)*dps[0]+byt,
		Ay(0,0)*dps[1]+byt,
		Ay(0,0)*dps[4]+byt,
		Ay(0,0)*dps[5]+byt};
  Real uy[4] ={ Ay(1,0)*dps[0]+byu,
		Ay(1,0)*dps[1]+byu,
		Ay(1,0)*dps[4]+byu,
		Ay(1,0)*dps[5]+byu};
  Real tz[4] ={ Az(0,0)*dps[0]+bzt,
		Az(0,0)*dps[1]+bzt,
		Az(0,0)*dps[2]+bzt,
		Az(0,0)*dps[3]+bzt};
  Real uz[4] ={ Az(1,0)*dps[0]+bzu,
		Az(1,0)*dps[1]+bzu,
		Az(1,0)*dps[2]+bzu,
		Az(1,0)*dps[3]+bzu};
  bool testcorners [8] = {0,0,0,0,0,0,0,0};
  Vector3 bbtemp,ltemp;
  Real dmin = Inf;
  //check the x's
  for(int i=0;i<4;i++) {
    if(ux[i] < 0) 
      testcorners[i] = true;
    else if(ux[i] > (bmax.x-bmin.x)) 
      testcorners[i+1] = true;
    else {
      Vector3 diff = (-tx[i])*direction;
      diff.x += ux[i] + bmin.x;
      diff.y += (i&1? bmax.y : bmin.y);
      diff.z += (i&2? bmax.z : bmin.z);
      Real d2 = diff.normSquared();
      if(d2 < dmin) {
	tclosest = tx[i];
	eval(tclosest,ltemp);
	dmin = bb.distanceSquared(ltemp,bbclosest);
      }
    }
  }
  //check the y's
  for(int i=0;i<4;i++) {
    if(uy[i] < 0) 
      testcorners[i] = true;
    else if(uy[i] > (bmax.y-bmin.y)) 
      testcorners[i+2] = true;
    else {
      Vector3 diff = (-ty[i])*direction;
      diff.y += uy[i] + bmin.y;
      diff.x += (i&1? bmax.x : bmin.x);
      diff.z += (i&2? bmax.z : bmin.z);
      Real d2 = diff.normSquared();
      if(d2 < dmin) {
	tclosest = ty[i];
	eval(tclosest,ltemp);
	dmin = bb.distanceSquared(ltemp,bbclosest);
      }
    }
  }
  //check the z's
  for(int i=0;i<4;i++) {
    if(uz[i] < 0) 
      testcorners[i] = true;
    else if(uz[i] > (bmax.z-bmin.z)) 
      testcorners[i+4] = true;
    else {
      Vector3 diff = (-tz[i])*direction;
      diff.z += uz[i] + bmin.z;
      diff.x += (i&1 ? bmax.x : bmin.x);
      diff.y += (i&2 ? bmax.y : bmin.y);
      Real d2 = diff.normSquared();
      if(d2 < dmin) {
	tclosest = tz[i];
	eval(tclosest,ltemp);
	dmin = bb.distanceSquared(ltemp,bbclosest);
      }
    }
  }
  //check the corners
  for(int i=0;i<8;i++) {
    if(testcorners[i]) {
      Vector3 pt;
      pt.x = (i & 1 ? bb.bmax.x : bb.bmin.x);
      pt.y = (i & 2 ? bb.bmax.y : bb.bmin.y);
      pt.z = (i & 4 ? bb.bmax.z : bb.bmin.z);
      Real ttemp = closestPoint(pt,ltemp);
      Real d2 = pt.distanceSquared(ltemp);
      if(d2 < dmin) {
	dmin = d2;
	tclosest = ttemp;
	bbclosest = pt;
      }
    }
  }
  return Sqrt(dmin);
}

namespace Math3D
{
  ostream& operator << (ostream& out,const Line3D& line)
  {
    out<<line.source<<"  "<<line.direction;
    return out;
  }
  istream& operator >> (istream& in,Line3D& line)
  {
    in>>line.source>>line.direction;
    return in;
  }
}
