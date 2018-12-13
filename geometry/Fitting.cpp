#include "Fitting.h"
#include <errors.h>
//#include <math3d/LinearAlgebra.h>
#include <math3d/Circle2D.h>
#include <math3d/Line2D.h>
#include <math3d/Line3D.h>
#include <math3d/Plane3D.h>
#include <math/matrix.h>
#include <math/vector.h>
#include <math/SVDecomposition.h>
using namespace Math;
using namespace std;

namespace Geometry {

//m = m+xy^t
void Rank1Update(Matrix2& m,const Vector2& x,const Vector2& y)
{
  //note that the matrix data variable is column-major, not row major
  m.data[0][0]+=x.x*y.x;
  m.data[0][1]+=x.y*y.x;

  m.data[1][0]+=x.x*y.y;
  m.data[1][1]+=x.y*y.y;
}

//m = m+xy^t
void Rank1Update(Matrix3& m,const Vector3& x,const Vector3& y)
{
  //note that the matrix data variable is column-major, not row major
  m.data[0][0]+=x.x*y.x;
  m.data[0][1]+=x.y*y.x;
  m.data[0][2]+=x.z*y.x;

  m.data[1][0]+=x.x*y.y;
  m.data[1][1]+=x.y*y.y;
  m.data[1][2]+=x.z*y.y;

  m.data[2][0]+=x.x*y.z;
  m.data[2][1]+=x.y*y.z;
  m.data[2][2]+=x.z*y.z;
}

Vector2 GetMean(const vector<Vector2>& pts)
{
  Vector2 sum(Zero);
  for(size_t i=0;i<pts.size();i++)
    sum += pts[i];
  return sum / pts.size();
}

Vector3 GetMean(const vector<Vector3>& pts)
{
  Vector3 sum(Zero);
  for(size_t i=0;i<pts.size();i++)
    sum += pts[i];
  return sum / pts.size();
}

void GetCovariance(const vector<Vector2>& pts,Matrix2& C)
{
  C.setZero();
  Vector2 mean=GetMean(pts);
  for(size_t i=0;i<pts.size();i++) {
    Vector2 p = pts[i]-mean;
    Rank1Update(C,p,p);
  }
}

void GetCovariance(const vector<Vector3>& pts,Matrix3& C)
{
  C.setZero();
  Vector3 mean=GetMean(pts);
  for(size_t i=0;i<pts.size();i++) {
    Vector3 p = pts[i]-mean;
    Rank1Update(C,p,p);
  }
}

bool FitGaussian(const vector<Vector2>& pts,Vector2& mean,Matrix2& R,Vector2& axes)
{
  mean = GetMean(pts);
  Matrix A(pts.size(),2);
  for(size_t i=0;i<pts.size();i++) 
    (pts[i]-mean).get(A(i,0),A(i,1));
  SVDecomposition<Real> svd;
  if(!svd.set(A)) {
    return false;
  }

  svd.sortSVs();
  axes.set(svd.W(0),svd.W(1));
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++)
      R(i,j) = svd.V(i,j);
  return true;
}

bool FitGaussian(const vector<Vector3>& pts,Vector3& mean,Matrix3& R,Vector3& axes)
{
  mean = GetMean(pts);
  Matrix A(pts.size(),3);
  for(size_t i=0;i<pts.size();i++) 
    (pts[i]-mean).get(A(i,0),A(i,1),A(i,2));
  SVDecomposition<Real> svd;
  if(!svd.set(A)) {
    return false;
  }

  svd.sortSVs();
  axes.set(svd.W(0),svd.W(1),svd.W(2));
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      R(i,j) = svd.V(i,j);
  return true;
}

bool FitLine(const vector<Vector2>& pts,Line2D& l)
{
  Vector2 mean = GetMean(pts);
  Matrix A(pts.size(),2);
  for(size_t i=0;i<pts.size();i++) 
    (pts[i]-mean).get(A(i,0),A(i,1));
  SVDecomposition<Real> svd;
  if(!svd.set(A)) {
    return false;
  }

  svd.sortSVs();
  //take the first singular value
  Vector sv;
  svd.V.getColRef(0,sv);
  l.direction.set(sv(0),sv(1));
  l.direction.inplaceNormalize();
  l.source = mean;
  return true;
}

bool FitLine(const vector<Vector3>& pts,Line3D& l)
{
  Vector3 mean = GetMean(pts);
  Matrix A(pts.size(),3);
  for(size_t i=0;i<pts.size();i++) 
    (pts[i]-mean).get(A(i,0),A(i,1),A(i,2));
  SVDecomposition<Real> svd;
  if(!svd.set(A)) {
    return false;
  }

  svd.sortSVs();
  //take the first singular value
  Vector sv;
  svd.V.getColRef(0,sv);
  l.direction.set(sv(0),sv(1),sv(2));
  l.direction.inplaceNormalize();
  l.source = mean;
  return true;
}

bool FitPlane(const vector<Vector3>& pts,Plane3D& p)
{
  Vector3 mean = GetMean(pts);
  Matrix A(pts.size(),3);
  for(size_t i=0;i<pts.size();i++) 
    (pts[i]-mean).get(A(i,0),A(i,1),A(i,2));
  SVDecomposition<Real> svd;
  if(!svd.set(A)) {
    return false;
  }

  svd.sortSVs();
  //take the last singular value
  Vector sv;
  svd.V.getColRef(2,sv);
  p.normal.set(sv(0),sv(1),sv(2));
  p.normal.inplaceNormalize();
  p.offset = dot(p.normal,mean);
  return true;
}

//circle equation |(x,y)-(a,b)|^2 = c^2
//f(a,b,c) = x^2+y^2-2ax-2yb+a^2+b^2-c^2 = 0
//let u=-2a, v=-2b, w = a^2+b^2-c^2
//f(u,v,w) = x^2+y^2+ux+vy+w
//min g(u,v,w) = 1/2 sum_(x,y) f(u,v,w)^2
//dg/du = sum_(x,y) df/du(u,v,w) f(u,v,w)
//      = sum_(x,y) x f(u,v,w)
//dg/dv = sum_(x,y) y f(u,v,w)
//dg/dw = sum_(x,y) f(u,v,w)
//dg/dparams = [sum x^2, sum xy,  sum x] *params + [sum x(x^2+y^2)] = 0
//             [sum xy,  sum y^2, sum y]           [sum y(x^2+y^2)]
//             [sum x,   sum y,   sum 1]           [sum (x^2+y^2) ]
//least squares fit
//A = matrix of (x,y,1)
//b = matrix of (x^2+y^2)
bool FitCircle(const std::vector<Vector2>& pts,Circle2D& c)
{
  Vector2 mean=GetMean(pts);
  Matrix3 A(Zero);
  Vector3 b(Zero);
  for(size_t i=0;i<pts.size();i++) {
    Vector3 v(pts[i].x-mean.x,pts[i].y-mean.x,1.0);
    Rank1Update(A,v,v);
    Real n = pts[i].distanceSquared(mean);
    b.x += (pts[i].x-mean.x)*n;
    b.y += (pts[i].y-mean.x)*n;
    b.z += n;
  }
  Matrix3 Ainv;
  if(!Ainv.setInverse(A)) return false;
  Vector3 params = Ainv*b;
  //recover circle from params
  c.center.x = -params.x*0.5;
  c.center.y = -params.y*0.5;
  c.radius = c.center.normSquared()-params.z;
  if(c.radius < 0) return false;
  c.radius = Sqrt(c.radius);
  c.center += mean;
  return true;
}

} //namespace Geometry
