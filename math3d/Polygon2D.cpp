#include <KrisLibrary/Logger.h>
#include "Polygon2D.h"
#include "geometry2d.h"
#include <math/Interval.h>
#include <math/angle.h>
#include "misc.h"
#include "clip.h"
#include "interpolate.h"
#include <iostream>
#include <errors.h>
using namespace Math3D;
using namespace std;

bool Polygon2D::ccw() const
{
  size_t i=0,j=1,k=2;
  if(vertices.size() < 3) return true;
  for(i=0;i<vertices.size();i++) {
    if(Orient2D(vertices[i],vertices[j],vertices[k]) < Zero)
      return false;
    j=k; k++;
    if(k >= vertices.size()) k=0;
  }
  return true;
}

bool Polygon2D::nonCrossing() const
{
  Segment2D ei,ej;
  //faster method would compare monotonic pieces
  for(size_t i=0;i<vertices.size();i++) {
    getEdge(i,ei);
    for(size_t j=0;j+1<i;j++) {  //don't compare adjacent segments
      getEdge(i,ej);
      if(ei.intersects(ej)) return false;
    }
  }
  return true;
}

Real Polygon2D::area() const
{
  Real sum = 0;
  for(size_t i=0;i<vertices.size();i++) {
    size_t j=next(i);
    sum += vertices[i].x*vertices[j].y - vertices[j].x*vertices[i].y;
  }
  return sum*Half;
}

Vector2 Polygon2D::centroid() const
{
  Vector2 c(Zero);
  for(size_t i=0;i<vertices.size();i++) {
    size_t j=next(i);
    c.madd(vertices[i]+vertices[j],vertices[i].x*vertices[j].y - vertices[j].x*vertices[i].y);
  }
  return c/(6.0*area());
}


void Polygon2D::getEdge(int i,Segment2D& ei) const
{
  size_t j=next(i);
  ei.a=vertices[i];
  ei.b=vertices[j];
}

void Polygon2D::getPlane(int i,Plane2D& pi) const
{
  size_t j=next(i);
  //must make planes point outside, normal order points inward
  pi.setPoints(vertices[j],vertices[i]);
}

Real weight(const Vector2& a,const Vector2& b,const Vector2& c)
{
  Real dA,dB,dC;
  dA=b.distanceSquared(c);
  dB=a.distanceSquared(c);
  dC=a.distanceSquared(b);
  Real A = Orient2D(a,b,c)*Half;
  const static Real Sqrt3=Sqrt(3.0);
  return Half*(dA+dB+dC)/(Sqrt3*A);
}

void OutputTriangles(const vector<vector<int> >& index,
		     const vector<Vector2>& vertices,
		     int i,int k,
		     vector<Triangle2D>& tris)
{
  int j=index[i][k];
  if(j<0) return;
  Triangle2D tri;
  tri.set(vertices[i],vertices[k],vertices[j]);
  tris.push_back(tri);
  OutputTriangles(index,vertices,i,j,tris);
  OutputTriangles(index,vertices,k,k,tris);
}

void Polygon2D::triangulateConvex(std::vector<Triangle2D>& tris) const
{
  Assert(vertices.size() >= 3);
  //basic triangulation
  tris.resize(vertices.size()-2);
  for(size_t i=0;i+2<vertices.size();i++)
    tris[i].set(vertices[0],vertices[i+1],vertices[i+2]);

  //minimal triangulation - dynamic programming approach
  vector<vector<Real> > cost(vertices.size());
  vector<vector<int> > index(vertices.size());
  for(size_t i=0;i<cost.size();i++) {
    cost[i].resize(vertices.size(),0);
    index[i].resize(vertices.size(),-1);
  }
  for(size_t i=0;i+2<vertices.size();i++) {
    cost[i][i+2] = weight(vertices[i],vertices[i+2],vertices[i+1]);
    index[i][i+2]=i+1;
  }
  for(size_t diag=3;diag<vertices.size();diag++) {
    for(size_t i=0;i+diag<vertices.size();i++) {
      size_t k=i+diag;
      Real minVal = Inf;
      int minIndex=-1;
      for(size_t j=i+1;j<k;j++) {
	Real valj = cost[i][j]+cost[j][k]+weight(vertices[i],vertices[j],vertices[k]);
	if(valj < minVal) {
	  minVal = valj;
	  minIndex=(int)j;
	}
      }
      cost[i][k]=minVal;
      index[i][k]=minIndex;
    }
  }
  tris.resize(0);
  OutputTriangles(index,vertices,0,vertices.size()-1,tris);
  Assert(tris.size()==vertices.size()-3);
}

void Polygon2D::setTransformed(const Polygon2D& in, const Matrix3& T)
{
	vertices.resize(in.vertices.size());
	for(size_t i=0; i<vertices.size(); i++)
		T.mulPoint(in.vertices[i], vertices[i]);
}

bool Polygon2D::planeSplits(const Plane2D& p) const
{
	ClosedInterval x;	x.setEmpty();
	for(size_t i=0; i<vertices.size(); i++) {
		x.expand(p.distance(vertices[i]));
		if(x.contains(Zero))
			return true;
	}
	return false;
}

bool Polygon2D::planePos(const Plane2D& p) const
{
	for(size_t i=0; i<vertices.size(); i++)	{
		if(p.distance(vertices[i]) < Zero)
			return false;
	}
	return true;
}

bool Polygon2D::planeNeg(const Plane2D& p) const
{
	for(size_t i=0; i<vertices.size(); i++) {
		if(p.distance(vertices[i]) > Zero)
			return false;
	}
	return true;
}

bool Polygon2D::raySplits(const Vector2& a,const Vector2& b) const
{
	ClosedInterval x;	x.setEmpty();
	for(size_t i=0; i<vertices.size(); i++) {
		x.expand(Orient2D(a,b,vertices[i]));
		if(x.contains(Zero))
			return true;
	}
	return false;
}

bool Polygon2D::rayLeft(const Vector2& a,const Vector2& b) const
{
	for(size_t i=0; i<vertices.size(); i++) {
		if(Orient2D(a,b,vertices[i]) < Zero)
			return false;
	}
	return true;
}

bool Polygon2D::rayRight(const Vector2& a,const Vector2& b) const
{
	for(size_t i=0; i<vertices.size(); i++) {
		if(Orient2D(a,b,vertices[i]) > Zero)
			return false;
	}
	return true;
}

int Polygon2D::residue(const Vector2& x) const
{
  //do the residue with a vertical ray
  int res=0;
  for(size_t i=0;i<vertices.size();i++) {
    const Vector2& vi=vertices[i];
    const Vector2& vj=vertices[next(i)];
    if(vi.x < x.x) {
      if(vj.x > x.x) { //crosses x=x axis left to right
	//does it cross the axis above x?
	Real x1=vi.x-x.x;
	Real x2=vj.x-x.x;
	Real y1=vi.y-x.y;
	Real y2=vj.y-x.y;
	//Real y=(-x2*y1+x1*y2)/(vi.x-vj.x);
	//if(y >= Zero) res--;
	//if(Sign(-x2*y1+x1*y2) == Sign(vi.x-vj.x) or 0) res--;
	if(Sign(-x2*y1+x1*y2) != 1) res--;
      }
      else if(vj.x == x.x) { //ends up on x.x
	//count as if it's x+eps, which means it doesn't count at all
      }
    }
    else if(vi.x > x.x) {
      if(vj.x < x.x) { //crosses x=x axis right to left
	//does it cross the axis above x?
	Real x1=vi.x-x.x;
	Real x2=vj.x-x.x;
	Real y1=vi.y-x.y;
	Real y2=vj.y-x.y;
	if(Sign(-x2*y1+x1*y2) != -1) res++;
      }
      else if(vj.x == x.x) {  //ends up on x.x
	//count it if it crosses the axis x=x
	if(vj.y >= x.y) res++;
      }
    }
    else if(vi.x == x.x) { //starts out on x.x
      //count only if it goes to the right
      if(vj.x > x.x) {
	if(vi.y > x.y) res--;
      }
    }
  }
  return res;
}

//bool Polygon2D::intersectsBoundary(const Polygon2D& other) const;

Real Polygon2D::boundaryDistance(const Point2D& v) const
{
  if(vertices.size() == 0) return 0;
  else if(vertices.size() == 1) return (v-vertices[0]).norm();
  else {
    Segment2D s; s.a=vertices[0]; s.b=vertices[1];
    return s.distance(v);
  }
  Segment2D s;
  getEdge(0,s);
  Real dmax=s.distance(v);
  for(size_t i=1; i<vertices.size(); i++) {
    getEdge(i,s);
    dmax = Max(dmax,s.distance(v));
  }
  return dmax;
}

bool Polygon2D::intersects(const Line2D& l, Real& tmin, Real& tmax) const
{
  FatalError("Polygon2D::intersects(Line2D) not defined yet");
  return false;
  Segment2D s;
  for(size_t i=0;i<vertices.size();i++) {
    getEdge(i,s);
  }
}

bool Polygon2D::intersects(const Line2D& l) const
{
  Real tmin=-Inf,tmax=Inf;
  return intersects(l,tmin,tmax);
}

bool Polygon2D::intersects(const Segment2D& l) const
{
  for(size_t i=0;i<vertices.size();i++) {
    if(l.intersects(vertices[i],vertices[next(i)])) return true;
  }
  //either l is completely inside or outside the polygon
  if(residue(l.a)==0 && residue(l.b)==0) return false;
  //must be completely inside the polygon
  return true;
}

void Polygon2D::getAABB(AABB2D& bb) const
{
  if(vertices.size() == 0) {
    bb.minimize();
    return;
  }
  bb.setPoint(vertices[0]);
  for(size_t i=1; i<vertices.size(); i++)
    bb.expand(vertices[i]);
}







bool ConvexPolygon2D::isValid() const
{
  return ccw();
}

bool ConvexPolygon2D::intersects(const ConvexPolygon2D& other) const
{
  size_t i;
  for(i=0; i<other.vertices.size(); i++) {
    if(rayRight(other.vertices[i],other.vertices[next(i)])) return false;
  }
  for(i=0; i<vertices.size(); i++) {
    if(other.rayRight(vertices[i],vertices[next(i)])) return false;
  }
  return true;
}


bool ConvexPolygon2D::contains(const Point2D& v) const
{
  for(size_t i=0; i<vertices.size(); i++)
    if(Orient2D(vertices[i],vertices[next(i)],v) < Zero) return false;
  return true;
}

bool ConvexPolygon2D::withinEdgeDistance(const Point2D& v, Real dist) const
{
  Plane2D p;
  for(size_t i=0; i<vertices.size(); i++) {
    getPlane((int)i,p);
    if(p.distance(v) > dist) return false;
  }
  return true;
}

Real ConvexPolygon2D::edgeDistance(const Point2D& v) const
{
  if(vertices.size() == 0) return 0;
  else if(vertices.size() == 1) return (v-vertices[0]).norm();
  else {
    Segment2D s; s.a=vertices[0]; s.b=vertices[1];
    return s.distance(v);
  }
  Plane2D p;
  getPlane(0,p);
  Real dmax=p.distance(v);
  for(size_t i=1; i<vertices.size(); i++) {
    getPlane(i,p);
    dmax = Max(dmax,p.distance(v));
  }
  return dmax;
}

bool ConvexPolygon2D::intersects(const Line2D& l, Real& tmin, Real& tmax) const
{
	return ClipLine(l.source,l.direction,*this,tmin,tmax);
}

bool ConvexPolygon2D::intersects(const Line2D& l) const
{
	Real tmin=-Inf, tmax=Inf;
	return intersects(l,tmin,tmax);
}

int ConvexPolygon2D::planeIntersections(const Plane2D& p,int& e1,int& e2,Real& u1,Real& u2) const
{
  int num=0;
  Real di,dj=p.distance(vertices[0]);
  for(size_t i=0;i<vertices.size();i++) {
    di=dj;
    dj=p.distance(vertices[next(i)]);
    if((di<Zero && dj>Zero) ||
       (di>Zero && dj<Zero) ||
       (di==Zero && dj!=Zero)) {
      if(num==0) {
	u1=di/(di-dj);
	e1=i;
      }
      else if(num==1) {
	u2=di/(di-dj);
	e2=i;
      }
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"More than 1 intersection???");
	abort();
      }
      num++;
    }
  }
  return num;
}

//if v represents a circular list, erase the elements [a..b)
template <class T>
typename std::vector<T>::iterator CircularDelete(std::vector<T>& v,int a,int b)
{
  if(a>b) { //wraps around
    v.erase(v.begin()+a,v.end());
    return v.erase(v.begin(),v.begin()+b);
  }
  else {
    return v.erase(v.begin()+a,v.begin()+b);
  }
}

void ConvexPolygon2D::halfspaceIntersection(const Plane2D& p)
{
  int e1,e2;
  Real u1,u2;

  //get the negative side of the halfspace
  int n=planeIntersections(p,e1,e2,u1,u2);
  if(n==0) {
    if(p.distance(vertices[0]) > Zero) { //all on outside of b
      vertices.clear();
      return;
    }
  }
  else if(n==1) {
    Assert(u1==Zero);
    //split at e1
    if(p.distance(vertices[next(e1)]) > Zero) { //all on outside of b
      //clear to a point
      Vector2 v=vertices[e1];
      vertices.clear();
      vertices.push_back(v);
    }
  }
  else if(n==2) {
    //split between e1 and e2
    //order them so e1 is on inside, e2 on outside
    if(p.distance(vertices[e1]) > Zero) {
      Assert(p.distance(vertices[e2]) <= Zero);
      swap(e1,e2);
      swap(u1,u2);
    }
    Vector2 v1,v2;
    interpolate(vertices[e1],vertices[next(e1)],u1,v1);
    interpolate(vertices[e2],vertices[next(e2)],u2,v2);
    //old order is ... e1 f1 ... e2 f2 ...
    //new order is ... e1 v1 v2 f2 ...
    vector<Vector2>::iterator f2=CircularDelete(vertices,e1+1,e2+1);
    vertices.insert(vertices.insert(f2,v2),v1);
  }
}

void ConvexPolygon2D::setIntersection(const ConvexPolygon2D& a,const ConvexPolygon2D& b)
{
  *this = a;
  //progressively do halfspace intersection for each plane of b
  Plane2D p;
  for(size_t i=0;i<b.vertices.size();i++) {
    b.getPlane(i,p);
    halfspaceIntersection(p);
  }
}
