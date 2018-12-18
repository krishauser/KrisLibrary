#include <KrisLibrary/Logger.h>
#include "WorkspaceBound.h"
#include "AngleBracket.h"
#include "Geometry.h"
#include <math/IntervalSet.h>
#include <math/AngleSet.h>
#include <math3d/rotation.h>
#include <math3d/basis.h>
#include <iostream>

#define wsEpsilon 0.001

ostream& operator <<(ostream& out,const AngleInterval& i)
{
  out<<i.c<<" -> "<<i.d;
  return out;
}

ostream& operator <<(ostream& out,const AngleSet& i)
{
  for(size_t k=0;k<i.size();k++)
    out<<i[k]<<",  ";
  return out;
}

ostream& operator << (ostream& out,const HollowBall& h)
{
  out<<"Center "<<h.center<<" radius ["<<h.innerRadius<<","<<h.outerRadius<<"]";
  return out;
}

ostream& operator << (ostream& out,const WorkspaceBound& b)
{
  if(b.IsFull()) {
    out<<"full space";
  }
  else if(b.IsEmpty()) {
    out<<"empty";
  }
  else {
    if(b.balls.size() > 1) {
      out<<"intersection of:";
      for(size_t j=0;j<b.balls.size();j++) {
	const HollowBall& h=b.balls[j];
	out<<endl<<h;
      }
    }
    else out<<b.balls[0];
  }
  return out;
}



//returns the number of resulting intervals
//if 1, returns it in p
//if 2, returns them in p and q
int IntervalSubtract(const ClosedInterval& a,const OpenInterval& b,ClosedInterval& p,ClosedInterval& q)
{
  if(a.intersects(b)) {
    if(a.contains(b.a) && a.contains(b.b)) {
      p.a=a.a; p.b=b.a;
      q.a=b.b; q.b=a.b;
      return 2;
    }
    else if(a.contains(b.a)) {
      p.a=a.a; p.b=b.a;
      return 1;
    }
    else if(a.contains(b.b)) {
      p.a=b.b; p.b=a.b;
      return 1;
    }
    else {
      Assert(b.contains(a.a) && b.contains(a.b));
      return 0;
    }
  }
  else {
    p=a;
    return 1;
  }
}

int AngleIntervalSubtract(const AngleInterval& a,const AngleInterval& b,AngleInterval& p,AngleInterval& q)
{
  if(a.intersects(b)) {
    if(a.contains(b)) {
      p.setRange(a.c,b.c);
      q.setRange(AngleNormalize(b.c+b.d),AngleNormalize(a.c+a.d));
      return 2;
    }
    else if(a.contains(b.c)) {
      p.setRange(a.c,b.c);
      return 1;
    }
    else if(a.contains(AngleNormalize(b.c+b.d))) {
      p.setRange(AngleNormalize(b.c+b.d),AngleNormalize(a.c+a.d));
      return 1;
    }
    else {
      Assert(b.contains(a));
      return 0;
    }
  }
  else {
    p=a;
    return 1;
  }
}

Vector3 AxisSweptPoint::center() const
{
  Vector3 c;
  axis.closestPoint(p,c);
  return c;
}

Real AxisSweptPoint::radius() const
{
  return axis.distance(p);
}

void AxisSweptPoint::getCircle(Circle3D& c) const
{
  c.center=center();
  c.radius=radius();
}

Vector3 AxisSweptPoint::eval(Real theta) const
{
  Vector3 ploc=p-axis.source;
  AngleAxisRotation aa;
  aa.axis=axis.direction;
  aa.angle=theta;
  Vector3 temp;
  aa.transformPoint(ploc,temp);
  return temp+axis.source;
}





bool HollowBall::contains(const Vector3& pt) const
{
  Real d = pt.distance(center);
  return innerRadius <= d && d <= outerRadius;
}

bool HollowBall::contains(const HollowBall& b) const
{
  if(b.outerRadius > outerRadius) return false;
  Real d = b.center.distance(center);
  ClosedInterval i,ib;
  if(isBall()) {
    i.a = -outerRadius;
    i.b = outerRadius;
    ib.a = d-b.outerRadius;
    ib.b = d+b.outerRadius;
    return i.contains(ib);
  }
  else {
    //first positive side
    i.a = innerRadius;
    i.b = outerRadius;
    if(d > b.outerRadius) {  //check b as a ball
      ib.a = d-b.outerRadius;
      ib.b = d+b.outerRadius;
      return i.contains(ib);
    }
    //check b in two parts -- positive, negative
    ib.a = d+Max(b.innerRadius,Zero);
    ib.b = d+b.outerRadius;
    if(!i.contains(ib)) return false;
    //negative part
    i.a = -outerRadius;
    i.b = -innerRadius;
    ib.a = d-b.outerRadius;
    ib.b = d-Max(b.innerRadius,Zero);
    return i.contains(ib);
  }
}

bool HollowBall::containsBall(const Sphere3D& s) const
{
  Real d = s.center.distance(center);
  ClosedInterval i,ib;
  i.a = Max(innerRadius,Zero);
  i.b = outerRadius;
  ib.a = d-s.radius;
  ib.b = d+s.radius;
  return i.contains(ib);
}

bool HollowBall::containsSphere(const Sphere3D& s) const
{
  Real d = s.center.distance(center);
  ClosedInterval i;
  i.a = Max(innerRadius,Zero);
  i.b = outerRadius;
  return i.contains(d-s.radius) && i.contains(d+s.radius);
}

bool HollowBall::intersects(const Line3D& line, ClosedIntervalSet& segs) const
{
  Sphere3D s;
  s.center=center;
  s.radius=outerRadius+wsEpsilon;
  ClosedInterval i;
  OpenInterval j;
  if(!s.intersects(line,&i.a,&i.b)) return false;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Outer sphere intersects at "<<i.a<<","<<i.b);

  segs.resize(1);
  segs[0] = i;
  if(innerRadius >= 0) {
    s.radius=innerRadius-wsEpsilon;
    if(s.intersects(line,&j.a,&j.b)) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Inner sphere intersects at "<<j.a<<","<<j.b);
      ClosedInterval p,q;
      int num=IntervalSubtract(i,j,p,q);
      Assert(num==2);
      segs.resize(num);
      segs[0] = p;
      segs[1] = q;
    }
  }
  return true;
}

bool HollowBall::intersects(const AxisSweptPoint& c, AngleSet& arcs) const
{
  Vector3 ccenter = c.center();
  AngleInterval i,j;
  i = AngleBracket_3D_Ball(c.p-ccenter,c.axis.direction,
			   center-ccenter,outerRadius+wsEpsilon);
  if(i.isEmpty()) { arcs.resize(1); arcs[0]=i; return false; }
  arcs.resize(1);
  arcs[0]=i;
  if(innerRadius > 0) {
    j = AngleBracket_3D_Ball(c.p-ccenter,c.axis.direction,
			     center-ccenter,innerRadius-wsEpsilon);
    if(!j.isEmpty()) {
      AngleInterval p,q;
      int num=AngleIntervalSubtract(i,j,p,q);
      //LOG4CXX_INFO(KrisLibrary::logger(),"Subtracting interval!");
      //LOG4CXX_INFO(KrisLibrary::logger(),i<<"  -  "<<j);
      //LOG4CXX_INFO(KrisLibrary::logger()," = "<<p<<", "<<q);
      Assert(num == 2);
      arcs.resize(num);
      arcs[0]=p;
      arcs[1]=q;
    }
  }
  return true;
}

bool HollowBall::intersects(const Arc3D& arc, AngleSet& arcs) const
{
  const AxisSweptPoint& c=arc;
  if(!intersects(c,arcs)) return false;
  //intersect arcs with the arc's arc
  arcs.Intersect(arc.interval);
  return !arcs.empty();
}

Real HollowBall::closestPoint(const Line3D& line) const
{
  return line.closestPointParameter(center);
}

Real HollowBall::closestPoint(const AxisSweptPoint& p) const
{
  Vector3 x,y;
  GetCanonicalBasis(p.axis.direction,x,y);
  Vector2 c2,p2,s2;
  c2.x = x.dot(center);
  c2.y = y.dot(center);
  p2.x = x.dot(p.p);
  p2.y = y.dot(p.p);
  s2.x = x.dot(p.axis.source);
  s2.y = y.dot(p.axis.source);
  c2-=s2;
  p2-=s2;
  Real ac = Atan2(c2.y,c2.x);
  Real ap = Atan2(p2.y,p2.x);
  return ac-ap;
}

//returns false if the balls don't intersect
bool BoundIntersection(const HollowBall& a,const HollowBall& b,HollowBall& out)
{
  Sphere3D sa,sb;
  Circle3D c;
  sa.center = a.center; sa.radius = a.outerRadius;
  sb.center = b.center; sb.radius = b.outerRadius;
  int res = BallBallIntersection(sa,sb,c);
  if(res==2) {  //it's a circle
    Assert(c.radius <= a.outerRadius+Epsilon && c.radius <= b.outerRadius+Epsilon);
    if(!sa.contains(c.center)) return false;
    if(!sb.contains(c.center)) return false;
    Assert(sa.contains(c.center));
    Assert(sb.contains(c.center));

    //If a and b lie on opposite sides of c's plane,
    //use the sphere that contains c.
    //Otherwise, the smallest circle that contains the intersection
    //is the smaller of a and b
    Real offset=c.axis.dot(c.center);
    if(Sign(c.axis.dot(a.center)-offset) == Sign(c.axis.dot(b.center)-offset)) {
      //on same side of plane
      if(b.outerRadius < a.outerRadius) res=4;
      else res=3;
    }
  }
  switch(res) {
  case 0: 
    out.center = Half*(a.center + b.center);
    out.outerRadius=0;
    out.innerRadius=-(a.center-b.center).norm();
    return false;
  case 1:  //point
  case 2:  //circle
    out.center=c.center;
    out.outerRadius=c.radius;
    out.innerRadius=0;  //TODO: find this?
    break;
  case 3:  //a completely contained within b
    Assert(a.outerRadius <= b.outerRadius);
    out = a;
    {
      Real rad=a.center.distance(b.center);
      if(rad + a.outerRadius < b.innerRadius)  //a contained within shell of b
	return false;
    }
    break;
  case 4:
    //b contained completely in a
    Assert(a.outerRadius >= b.outerRadius);
    out = b;
    {
      Real rad=a.center.distance(b.center);
      if(rad + b.outerRadius < a.innerRadius)  //a contained within shell of b
	return false;
    }
    break;
  }
  return true;
}






WorkspaceBound::WorkspaceBound()
{
  maxAngle=TwoPi;
}

void WorkspaceBound::SetEmpty()
{
  HollowBall b;
  b.setEmpty();
  balls.resize(1);
  balls[0] = b;
  maxAngle = -Inf;
}

void WorkspaceBound::SetFull()
{
  balls.clear();
  maxAngle=TwoPi;
}

void WorkspaceBound::SetPoint(const Vector3& p)
{
  HollowBall b;
  b.setPoint(p);
  balls.resize(1);
  balls[0] = b;
  maxAngle = Zero;
}

void WorkspaceBound::SetArc(const Arc3D& a)
{
  balls.clear();
  HollowBall b;
  b.center = a.center();
  b.outerRadius = b.innerRadius = a.radius();
  balls.push_back(b);
  maxAngle = a.interval.d;
  if(maxAngle < Pi) {  //there's another limiting ball
    Vector3 p1,p2;
    p1 = a.eval(a.interval.c);
    p2 = a.eval(a.interval.c+a.interval.d);
    b.center = Half*(p1+p2);
    b.outerRadius = Half*(p1-p2).norm();
    b.innerRadius = -Inf;
    balls.push_back(b);
  }
}

void WorkspaceBound::SetCircle(const Circle3D& c)
{
  HollowBall b;
  b.center=c.center;
  b.outerRadius = b.innerRadius = c.radius;
  balls.resize(1);
  balls[0]=b;
  maxAngle = TwoPi;
}

void WorkspaceBound::SetSphere(const Sphere3D& s)
{
  balls.resize(1);
  balls[0].setSphere(s);
  maxAngle = TwoPi;
}

void WorkspaceBound::SetTransformed(const WorkspaceBound& w, RigidTransform& T)
{
  balls.resize(w.balls.size());
  for(size_t i=0;i<w.balls.size();i++) {
    T.mulPoint(w.balls[i].center,balls[i].center);
    balls[i].outerRadius = w.balls[i].outerRadius;
    balls[i].innerRadius = w.balls[i].innerRadius;
  }
  maxAngle = w.maxAngle;
}

bool WorkspaceBound::SetIntersection(const WorkspaceBound& a, const WorkspaceBound& b)
{
  //LOG4CXX_INFO(KrisLibrary::logger(),"Setting intersection of "<<a<<" and "<<b);
  if(a.IsEmpty() || b.IsEmpty()) { SetEmpty(); return false; }
  if(a.balls.empty()) { *this = b; return true; }
  if(b.balls.empty()) { *this = a; return true; }
  HollowBall bounda,boundb;
  a.GetBounds(bounda);
  b.GetBounds(boundb);
  HollowBall isect;
  if(!BoundIntersection(bounda,boundb,isect)) { SetEmpty(); return false; }
  balls.resize(a.balls.size()+b.balls.size());
  copy(a.balls.begin(),a.balls.end(),balls.begin());
  copy(b.balls.begin(),b.balls.end(),balls.begin()+a.balls.size());
  maxAngle=Max(a.maxAngle,b.maxAngle);
  RemoveRedundancies();
  //LOG4CXX_INFO(KrisLibrary::logger(),"Result is "<<*this);
  return true;
}

bool WorkspaceBound::InplaceIntersection(const WorkspaceBound& b)
{
  WorkspaceBound temp=*this;
  return SetIntersection(temp,b);
}

void WorkspaceBound::SetMinkowskiSum(const WorkspaceBound& a,const WorkspaceBound& b)
{
  if(a.IsEmpty() || b.IsEmpty()) { 
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, minkowski sum is empty"); 
    SetEmpty(); 
    return;
  }
  HollowBall bb;
  b.GetBounds(bb);
  balls.resize(a.balls.size());
  for(size_t i=0;i<a.balls.size();i++) {
    Real r = bb.center.norm() + bb.outerRadius; 
    balls[i].center = a.balls[i].center;
    balls[i].outerRadius = a.balls[i].outerRadius + r;
    balls[i].innerRadius = a.balls[i].innerRadius - r;
  }
  maxAngle = a.maxAngle + b.maxAngle;
}

void WorkspaceBound::InplaceMinkowskiSum(const WorkspaceBound& b)
{
  WorkspaceBound temp=*this;
  return SetMinkowskiSum(temp,b);
}

void WorkspaceBound::RemoveRedundancies()
{
  vector<HollowBall> newBalls;
  for(size_t i=0;i<balls.size();i++) {
    bool redund=false;
    if(balls[i].isFull()) continue;
    for(size_t j=0;j<balls.size();j++) {
      if(j==i) continue;
      if(balls[i].contains(balls[j])) {
	//eliminate balls[i]
	redund = true;
	break;
      }
    }
    if(!redund) newBalls.push_back(balls[i]);
  }
  swap(balls,newBalls);
}

bool WorkspaceBound::IsEmpty() const
{
  if(maxAngle < 0) return true;
  for(size_t i=0;i<balls.size();i++)
    if(balls[i].isEmpty()) return true;
  return false;
}

bool WorkspaceBound::IsFull() const
{
  if(maxAngle < TwoPi) return false;
  for(size_t i=0;i<balls.size();i++)
    if(!balls[i].isFull()) return false;
  return true;
}

bool WorkspaceBound::IsBall() const
{
  if(maxAngle < TwoPi) return false;
  if(balls.size() > 1) return false;
  if(balls.empty()) return true;
  return balls[0].isBall();
}

Real WorkspaceBound::GetWidth() const
{
  Sphere3D s;
  GetBounds(s);
  if(maxAngle >= Pi) return Two*s.radius;
  //TODO: not accurate for innerradius != outerRadius
  else return Sqrt(Two-Two*Cos(maxAngle))*s.radius;
}

void WorkspaceBound::GetBounds(AABB3D& bb) const
{
  Sphere3D s;
  GetBounds(s);
  s.getAABB(bb);
}

void WorkspaceBound::GetBounds(Sphere3D& s) const
{
  if(IsEmpty()) {
    s.center.setZero();
    s.radius = -Inf;
  }
  else if(balls.empty()) {
    s.center.setZero();
    s.radius = Inf;
  }
  else if(balls.size() == 1) {
    balls[0].getOuterSphere(s);
  }
  else {
    HollowBall b;
    GetBounds(b);
    b.getOuterSphere(s);
  }
}

void WorkspaceBound::GetBounds(HollowBall& b) const
{
  if(balls.empty()) {
    b.setFull();
  }
  else {
    b=balls[0];
    for(size_t i=1;i<balls.size();i++) {
      BoundIntersection(b,balls[i],b);
    }
  }
}

bool WorkspaceBound::Contains(const Vector3& pt) const
{
  for(size_t i=0;i<balls.size();i++)
    if(!balls[i].contains(pt)) return false;
  return true;
}

bool WorkspaceBound::Intersects(const Line3D& line, ClosedIntervalSet& segs) const
{
  if(balls.empty()) {
    segs.SetFull();
    return true;
  }
  if(!balls[0].intersects(line,segs)) return false;
  ClosedIntervalSet temp;
  for(size_t i=1;i<balls.size();i++) {
    if(!balls[i].intersects(line,temp)) return false;
    segs.Intersect(temp);
    if(segs.empty()) return false;
  }
  return true;
}

bool WorkspaceBound::Intersects(const AxisSweptPoint& c, AngleSet& arcs) const
{
  if(balls.empty()) {
    arcs.SetCircle();
    return true;
  }
  if(!balls[0].intersects(c,arcs)) return false;
  AngleSet temp;
  for(size_t i=1;i<balls.size();i++) {
    if(!balls[i].intersects(c,temp)) { return false; }
    arcs.Intersect(temp);
    if(arcs.empty()) return false;
  }
  return true;
}

bool WorkspaceBound::Intersects(const Arc3D& arc, AngleSet& arcs) const
{
  const AxisSweptPoint& c=arc;
  if(!Intersects(c,arcs)) return false;
  //intersect arcs with the arc's arc
  arcs.Intersect(arc.interval);
  return !arcs.empty();
}

Real WorkspaceBound::ClosestPoint(const Line3D& line) const
{
  HollowBall h;
  GetBounds(h);
  return h.closestPoint(line);
}

Real WorkspaceBound::ClosestPoint(const AxisSweptPoint& p) const
{
  HollowBall h;
  GetBounds(h);
  return h.closestPoint(p);
}













#if 0

WorkspaceBound::WorkspaceBound()
{
  center.setZero();
  outerRadius=Inf;
  innerRadius=0;
  maxAngle=TwoPi;
}

void WorkspaceBound::SetEmpty()
{
  maxAngle = -Inf;
}

void WorkspaceBound::SetFull()
{
  outerRadius=Inf;
  innerRadius=0;
  maxAngle=TwoPi;
}

void WorkspaceBound::SetPoint(const Vector3& p)
{
  center = p;
  outerRadius = innerRadius = Zero;
  maxAngle = Zero;
}

void WorkspaceBound::SetArc(const Arc& a)
{
  if(a.interval.d >= Pi) {
    center = a.center();
    outerRadius = innerRadius = a.radius();
    maxAngle = a.interval.d;
  }
  else {
    Vector3 p1,p2;
    p1 = a.eval(a.interval.c);
    p2 = a.eval(a.interval.c+a.interval.d);
    center = Half*(p1+p2);
    outerRadius = Half*(p1-p2).norm();
    maxAngle = a.interval.d;
  }
}

void WorkspaceBound::SetCircle(const Circle3D& c)
{
  center=c.center;
  outerRadius = innerRadius = c.radius;
  maxAngle = TwoPi;
}

void WorkspaceBound::SetSphere(const Sphere3D& s)
{
  center = s.center;
  outerRadius = innerRadius = s.radius;
  maxAngle = TwoPi;
}

void WorkspaceBound::SetTransformed(const WorkspaceBound& w, RigidTransform& T)
{
  T.mulPoint(w.center,center);
  outerRadius = w.outerRadius;
  innerRadius = w.innerRadius;
  maxAngle = w.maxAngle;
}

bool WorkspaceBound::SetIntersection(const WorkspaceBound& a, const WorkspaceBound& b)
{
  if(a.IsEmpty() || b.IsEmpty()) { SetEmpty(); return false; }
  Sphere3D sa,sb;
  Circle3D c;
  sa.center = a.center; sa.radius = a.outerRadius;
  sb.center = b.center; sb.radius = b.outerRadius;
  int res = BallBallIntersection(sa,sb,c);
  Real solidAngle = Min(a.maxAngle*a.outerRadius,b.maxAngle*b.outerRadius);
  if(res==2) {  //it's a circle
    Assert(c.radius <= a.outerRadius+Epsilon && c.radius <= b.outerRadius+Epsilon);
    if(!sa.contains(c.center) || !sb.contains(c.center)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Uh... circle intersection of balls isn't contained in them");
      LOG4CXX_INFO(KrisLibrary::logger(),sa.center<<" r "<<sa.radius);
      LOG4CXX_INFO(KrisLibrary::logger(),sb.center<<" r "<<sb.radius);
      LOG4CXX_INFO(KrisLibrary::logger(),"Center "<<c.center<<" radius "<<c.radius);
      KrisLibrary::loggerWait();
    }
    Assert(sa.contains(c.center));
    Assert(sb.contains(c.center));

    //If a and b lie on opposite sides of c's plane,
    //use the sphere that contains c.
    //Otherwise, the smallest circle that contains the intersection
    //is the smaller of a and b
    Real offset=c.axis.dot(c.center);
    if(Sign(c.axis.dot(a.center)-offset) == Sign(c.axis.dot(b.center)-offset)) {
      //on same side of plane
      if(b.outerRadius < a.outerRadius) res=4;
      else res=3;
    }
  }
  switch(res) {
  case 0: 
    center = Half*(a.center + b.center);
    outerRadius=0;
    innerRadius=-(a.center-b.center).norm();
    maxAngle=-Inf;
    break;
  case 1:  //point
  case 2:  //circle
    center=c.center;
    outerRadius=c.radius;
    innerRadius=0;  //TODO: find this?
    //get the maxAngle that gives the right solid angle
    if(solidAngle >= outerRadius*TwoPi) maxAngle = TwoPi;
    else maxAngle = solidAngle/outerRadius;
    break;
  case 3:  //a completely contained within b
    Assert(a.outerRadius <= b.outerRadius);
    operator = (a);
    if(solidAngle < maxAngle*outerRadius) 
      maxAngle = solidAngle / outerRadius;
    break;
  case 4:
    //b contained completely in a
    Assert(a.outerRadius >= b.outerRadius);
    operator = (b);
    if(solidAngle < maxAngle*outerRadius) 
      maxAngle = solidAngle / outerRadius;
    break;
  }
  return true;
}

bool WorkspaceBound::InplaceIntersection(const WorkspaceBound& b)
{
  WorkspaceBound temp=*this;
  return SetIntersection(temp,b);
}

/*
bool WorkspaceBound::SetIntersection(const WorkspaceBound& w)
{
  Circle3D c;
  int res = BallBallIntersection(*this,w,c);
  Real solidAngle = Min(maxAngle*radius,w.maxAngle*w.radius);
  if(res==2) {
    Assert(c.radius <= radius+Epsilon && c.radius <= w.radius+Epsilon);
    Assert(contains(c.center));
    Assert(w.contains(c.center));

    //If a and b lie on opposite sides of c's plane,
    //use the sphere that contains c.
    //Otherwise, the smallest circle that contains the intersection
    //is the smaller of a and b
    Real offset=c.axis.dot(c.center);
    if(Sign(c.axis.dot(center)-offset) == Sign(c.axis.dot(w.center)-offset)) {
      //on same side of plane
      if(w.radius < radius) res=4;
      else res=3;
    }
  }
  switch(res) {
  case 0: 
    center = Half*(center + w.center);
    radius=-(center-w.center).norm();
    maxAngle=0;
    break;
  case 1:
  case 2:
    center=c.center;
    radius=c.radius;
    //get the maxAngle that gives the right solid angle
    if(solidAngle >= radius*TwoPi) maxAngle = TwoPi;
    else maxAngle = solidAngle/radius;
    break;
  case 3:  //completely contained within w
    Assert(radius <= w.radius);
    if(solidAngle < maxAngle*radius) {
      maxAngle = solidAngle / radius;
      return true;
    }
    return false;
  case 4:
    //w contained completely in this
    Assert(radius >= w.radius);
    center = w.center;
    radius = w.radius;
    if(solidAngle >= radius*TwoPi) maxAngle = TwoPi;
    else maxAngle = solidAngle/radius;
    break;
  }
  return true;
}
*/

void WorkspaceBound::SetMinkowskiSum(const WorkspaceBound& a,const WorkspaceBound& b)
{
  center = a.center;//+b.center;
  Real r = b.center.norm() + b.outerRadius; 
  outerRadius = a.outerRadius + r;
  innerRadius = a.innerRadius - r;
  maxAngle = a.maxAngle + b.maxAngle;
}

void WorkspaceBound::InplaceMinkowskiSum(const WorkspaceBound& b)
{
  WorkspaceBound temp=*this;
  return SetMinkowskiSum(temp,b);
}

bool WorkspaceBound::IsEmpty() const
{
  return maxAngle < 0;
}

bool WorkspaceBound::IsSphere() const
{
  return maxAngle >= TwoPi && innerRadius <= 0;
}

Real WorkspaceBound::GetWidth() const
{
  if(maxAngle >= Pi) return Two*outerRadius;
  //TODO: not accurate for innerradius != outerRadius
  else return Sqrt(Two-Two*Cos(maxAngle))*outerRadius;
}

//void WorkspaceBound::GetBounds(AABB3D&) const;
void WorkspaceBound::GetBounds(Sphere3D& s) const
{
  s.center=center;
  s.radius=outerRadius;
}

#endif
