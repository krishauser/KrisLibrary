#include "Line3D.h"
#include "Segment3D.h"
#include "clip.h"
#include "misc.h"
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
    fprintf(stderr,"Warning, Line3D closest points matrix inverse failed\n");
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
