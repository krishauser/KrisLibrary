#include "Polygon3D.h"
#include "geometry3d.h"
#include "Polygon2D.h"
#include <math/matrix.h>
#include <math/linalgebra.h>
#include <iostream>
#include <errors.h>
using namespace std;

namespace Math3D {

void Polygon3D::triangulateConvex(std::vector<Triangle3D>& tris) const
{
  Assert(vertices.size() >= 3);
  tris.resize(vertices.size()-2);
  for(size_t i=0;i+2<vertices.size();i++)
    tris[i].set(vertices[0],vertices[i+1],vertices[i+2]);
}

Real Polygon3D::areaConvex() const
{
  Real sum=0;
  Triangle3D temp;
  for(size_t i=1;i+1<vertices.size();i++) {
    temp.set(vertices[0],vertices[i],vertices[i+1]);
    sum += temp.area();
  }
  return sum;
}

Vector3 Polygon3D::centroidConvex() const
{
  const static Real Third = 1.0/3.0;
  Vector3 c(Zero);
  Real sum=0;
  Triangle3D temp;
  for(size_t i=1;i+1<vertices.size();i++) {
    temp.set(vertices[0],vertices[i],vertices[i+1]);
    Real area = temp.area();
    c.madd((temp.a+temp.b+temp.c),area*Third);
    sum += area;
  }
  if(sum == 0) return c;
  return c/sum;
}

void Polygon3D::setTransformed(const Polygon2D& in, const Matrix4& T)
{
  vertices.resize(in.vertices.size());
  Vector3 temp;
  for(size_t i=0; i<vertices.size(); i++) {
    temp.set(in.vertices[i].x,in.vertices[i].y,0);
    T.mulPoint(temp, vertices[i]);
  }
}

void Polygon3D::setTransformed(const Polygon3D& in, const Matrix4& T)
{
  vertices.resize(in.vertices.size());
  for(size_t i=0; i<vertices.size(); i++)
    vertices[i] = T*in.vertices[i];
}

void Polygon3D::getEdge(int i,Segment3D& ei) const
{
  ei.a = vertices[i];
  ei.b = vertices[next(i)];
}

void Polygon3D::getPlane(int i,Plane3D& p) const
{
  Assert(vertices.size() >= 3);
  size_t j=next(i);
  size_t k=next(j);
  p.setPoints(vertices[i],vertices[j],vertices[k]);
}

//uses a standard least-squares fit, picking a certain dependent coordinate
//as a function of the other variables.
//i.e. suppose z is the dependent coordinate.  Then zi = a xi + b yi + c
//is the plane estimated with the least-squares fit.
//The resulting plane is a x + b y - z + c
void Polygon3D::getPlaneFit(Plane3D& p) const
{
  Assert(vertices.size() >= 3);
  if(vertices.size()==3) {
    getPlane(0,p);
    return;
  }

  AABB3D aabb;
  getAABB(aabb);
  Vector3 dims=aabb.bmax-aabb.bmin;
  Assert(dims.x >= 0);
  Assert(dims.y >= 0);
  Assert(dims.z >= 0);

  //pick dimension to do least-squares on
  Real min=Min(dims.x,dims.y,dims.z);

  Matrix A; Vector b;
  A.resize(vertices.size(),3); b.resize(vertices.size());

  if(min == dims.x) {  //least squares on x
    for(size_t i=0;i<vertices.size();i++) {
      A(i,0) = vertices[i].y;
      A(i,1) = vertices[i].z;
      A(i,2) = 1;
      b(i) = vertices[i].x;
    }
  }
  else if(min == dims.y) { //least squares on y
    for(size_t i=0;i<vertices.size();i++) {
      A(i,0) = vertices[i].x;
      A(i,1) = vertices[i].z;
      A(i,2) = 1;
      b(i) = vertices[i].y;
    }
  }
  else {  //least squares on z
    for(size_t i=0;i<vertices.size();i++) {
      A(i,0) = vertices[i].x;
      A(i,1) = vertices[i].y;
      A(i,2) = 1;
      b(i) = vertices[i].z;
    }
  }
  MatrixEquation meq(A,b);
  Vector x;
  bool res=meq.LeastSquares_QR(x);
  Assert(res == true);
  Assert(x.n == 3);
  if(min == dims.x) {
    p.normal.set(-1,x(0),x(1));
    p.offset = -x(2);
  }
  else if(min == dims.y) {
    p.normal.set(x(0),-1,x(1));
    p.offset = -x(2);
  }
  else {
    p.normal.set(x(0),x(1),-1);
    p.offset = -x(2);
  }

  //normalize plane, and make sure the polygon goes ccw when looking down on the normal
  Real len=p.normal.norm();
  Plane3D temp;
  getPlane(0,temp);
  if(temp.normal.dot(p.normal) < 0) len = -len;

  p.offset /= len;
  p.normal /= len;
}

Real Polygon3D::maxDistance(const Plane3D& p) const
{
  Real dmax=0;
  for(size_t i=0;i<vertices.size();i++) {
    Real d = p.distance(vertices[i]);
    if(d > dmax) dmax=d;
  }
  return dmax;
}

void Polygon3D::getPlanarPolygon(Polygon2D& p,Matrix4& T) const
{
  Plane3D fitPlane;
  getPlaneFit(fitPlane);
  Vector3 xb,yb;
  fitPlane.normal.getOrthogonalBasis(xb,yb);
  //TODO: reduce numerical errors by normalizing to the mean point?
  T.set(xb,yb,fitPlane.normal,fitPlane.normal*fitPlane.offset);
  Matrix4 Tinv;
  Tinv.setInverse(T);
  p.vertices.resize(vertices.size());
  for(size_t i=0;i<vertices.size();i++) {
    p.vertices[i].x = dot(xb,vertices[i]);
    p.vertices[i].y = dot(yb,vertices[i]);
    /*
    Vector3 temp;
    Tinv.mulPoint(vertices[i],temp);
    Assert(FuzzyEquals(temp.x,p.vertices[i].x));
    Assert(FuzzyEquals(temp.y,p.vertices[i].y));
    */
  }
}

void Polygon3D::getAABB(AABB3D& bb) const
{
  if(vertices.size() == 0) {
    bb.minimize();
    return;
  }
  bb.setPoint(vertices[0]);
  for(size_t i=1; i<vertices.size(); i++)
    bb.expand(vertices[i]);
}
  
//  bool Read(File& f);
//  bool Write(File& f) const;


std::ostream& operator << (std::ostream& out,const Polygon3D& b)
{
  out<<b.vertices.size()<<"    ";
  for(size_t i=0;i<b.vertices.size();i++)
    out<<b.vertices[i]<<"  ";
  return out;
}


std::istream& operator >> (std::istream& in, Polygon3D& b)
{
  size_t n;
  in>>n;
  b.vertices.resize(n);
  for(size_t i=0;i<b.vertices.size();i++)
    in >> b.vertices[i];
  return in;
}

} //namespace Math3D
