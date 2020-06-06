#include "ConvexHull3D.h"
#include "SOLID/SOLID.h"
#include <iostream>
#include <KrisLibrary/utils/AnyValue.h>

using namespace std;

namespace Math3D {

/** @ingroup Math3D
 * @brief A 3D convex hull class
 *
 * This class usess SOLID3 library as backend and not so many function are exposed to the user.
 * 
 */

void ConvexHull3D::setPoints(const Vector& a) {
  int asize = a.size();
  if(asize % 3 != 0) {
    std::cout << "The size of point vector is not dividable by 3!\n";
    int use_size = int(asize / 3.) * 3;
    points.resize(use_size);
    std::copy(a.begin(), a.begin() + use_size, points.begin());
  }
  else{
    points = a;
  }
}

void ConvexHull3D::setPoints(const std::vector<Vector3> & a) {
  points.resize(a.size() * 3);
  int idx = 0;
  for(auto &ai : a) {
    points[idx++] = ai.x;
    points[idx++] = ai.y;
    points[idx++] = ai.z;
  }
}

void ConvexHull3D::setPoints(const std::vector<Vector> & a) {
  points.resize(a.size() * 3);
  int idx = 0;
  for(auto &ai : a) {
    points[idx++] = ai[0];
    points[idx++] = ai[1];
    points[idx++] = ai[2];
  }
}

std::tuple<double, Vector3, Vector3> dist_func(DT_ObjectHandle object1, DT_ObjectHandle object2) {
    DT_SetAccuracy((DT_Scalar)1e-6);
    DT_SetTolerance((DT_Scalar)(1e-6));
    DT_Vector3 point1, point2, point3, point4;
    DT_Scalar dist = DT_GetClosestPair(object1, object2, point1, point2);
    Vector3 p1, p2;
    if(dist > 1e-6) {  // consider not colliding
      p1.x = point1[0];
      p1.y = point1[1];
      p1.z = point1[2];
      p2.x = point2[0];
      p2.y = point2[1];
      p2.z = point2[2];
      return {dist, p1, p2};
    }
    else{
        DT_Bool is_pene = DT_GetPenDepth(object1, object2, point3, point4);
        p1.x = point3[0];
        p1.y = point3[1];
        p1.z = point3[2];
        p2.x = point4[0];
        p2.y = point4[1];
        p2.z = point4[2];
        if(is_pene) {
          dist = -(p1 - p2).norm();
          return {dist, p1, p2};
        }
        else{
          p1.x = point1[0];
          p1.y = point1[1];
          p1.z = point1[2];
          p2.x = point2[0];
          p2.y = point2[1];
          p2.z = point2[2];
          return {dist, p1, p2};
        }
    }
}

// compute the distance from a hull to another hull
double ConvexHull3D::distance(const ConvexHull3D &h) {
    return std::get<0>(closest_points(h));
}

double ConvexHull3D::Distance(const Vector3 &pnt) {
  ConvexHull3D pnt_hull;
  std::vector<Vector3> vpoint{pnt};
  pnt_hull.setPoints(vpoint);
  return distance(pnt_hull);
}

Real ConvexHull3D::ClosestPoints(const Vector3& pt,Vector3& cp,Vector3& direction) const {
  // first create hull and compute a lot
  ConvexHull3D pnt_hull;
  std::vector<Vector3> vpoint{pt};
  pnt_hull.setPoints(vpoint);
  return ClosestPoints(pnt_hull, cp, direction);
}

Real ConvexHull3D::ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const {
  double dist = 0;
  std::tie(dist, cp, direction) = closest_points(g);
  auto Unit = [](const Vector3& v)
  {
    Real n = v.norm();
    if(Math::FuzzyZero(n)) return Vector3(0.0);
    else return v*(1.0/n);
  };
  if(dist >= 0) {
    direction = Unit(direction - cp);
  }
  else{
    direction = Unit(cp - direction);
  }
  return dist;
}

std::tuple<double, Vector3, Vector3> ConvexHull3D::closest_points(const ConvexHull3D &h) const {
  DT_VertexBaseHandle base1 = DT_NewVertexBase(points.getPointer(), (DT_Size)(3 * sizeof(double)));
  DT_VertexBaseHandle base2 = DT_NewVertexBase(h.points.getPointer(), (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_VertexRange(0, points.size() / 3);
  DT_EndPolytope();
  DT_ShapeHandle cube2 = DT_NewPolytope(base2);
  DT_VertexRange(0, h.points.size() / 3);
  DT_EndPolytope();
  DT_ObjectHandle object1 = DT_CreateObject(NULL, cube1);
  DT_ObjectHandle object2 = DT_CreateObject(NULL, cube2);
  return dist_func(object1, object2);
}

//TODO: here I transform all points and this may not be efficient
void ConvexHull3D::Transform(const Matrix4 &T) {
  std::cout << "Transform points\n";
  int n_point = points.size() / 3;
  for(int i = 0; i < n_point; i++) {
    Vector3 temp(points[3 * i], points[3 * i + 1], points[3 * i + 2]);
    Vector3 rst;
    T.mulPoint(temp, rst);
    points[3 * i] = rst.x;
    points[3 * i + 1] = rst.y;
    points[3 * i + 2] = rst.z;
  }
}

AABB3D ConvexHull3D::GetAABB() const {
  int n_point = points.size() / 3;
  Vector3 bmin(points.getPointer()), bmax(points.getPointer());
  for(int i = 0; i < n_point; i++) {
    for(int j = 0; j < 3; j++) {
      if(bmin[j] > points[3 * i + j])
        bmin[j] = points[3 * i + j];
      if(bmax[j] < points[3 * i + j])
        bmax[j] = points[3 * i + j];
    }
  }
  return AABB3D(bmin, bmax);
}

Box3D ConvexHull3D::GetBB() const {  //TODO: is there a smart way to compute bb?
  AABB3D ab3d = GetAABB();
  Box3D b;
  b.set(ab3d);
  return b;
}

CollisionConvexHull3D::CollisionConvexHull3D(const ConvexHull3D& hull) {
  DT_VertexBaseHandle base1 = DT_NewVertexBase(hull.points.getPointer(), (DT_Size)(3 * sizeof(double)));
  DT_VertexBaseHandle base2 = DT_NewVertexBase(hull.points.getPointer(), (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_VertexRange(0, hull.points.size() / 3);
  DT_EndPolytope();
  this->object = DT_CreateObject(NULL, cube1);
}

double CollisionConvexHull3D::Distance(const Vector3 &pnt, const RigidTransform *tran) {
  DT_VertexBaseHandle base1 = DT_NewVertexBase(pnt.data, (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_ObjectHandle obj1 = DT_CreateObject(NULL, cube1);
  DT_VertexRange(0, 1);
  DT_EndPolytope();
  this->_update_transform(tran);
  return std::get<0>(dist_func(this->object, obj1));
}

Real CollisionConvexHull3D::ClosestPoints(const Vector3& pnt,Vector3& cp,Vector3& direction, const RigidTransform *tran) {
  DT_VertexBaseHandle base1 = DT_NewVertexBase(pnt.data, (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_ObjectHandle obj1 = DT_CreateObject(NULL, cube1);
  DT_VertexRange(0, 1);
  DT_EndPolytope();
  this->_update_transform(tran);
  double dist = 0;
  std::tie(dist, cp, direction) = dist_func(this->object, obj1);
  auto Unit = [](const Vector3& v)
  {
    Real n = v.norm();
    if(Math::FuzzyZero(n)) return Vector3(0.0);
    else return v*(1.0/n);
  };
  if(dist >= 0) {
    direction = Unit(direction - cp);
  }
  else{
    direction = Unit(cp - direction);
  }
  return dist;
}

// compute the closest points between two convexhull3d with collision data and store results somewhere
Real CollisionConvexHull3D::ClosestPoints(CollisionConvexHull3D& g, Vector3& cp, Vector3& direction, const RigidTransform *tran, const RigidTransform *tran2){
  this->_update_transform(tran);
  g._update_transform(tran2);
  double dist = 0;
  std::tie(dist, cp, direction) = dist_func(this->object, g.object);
  auto Unit = [](const Vector3& v)
  {
    Real n = v.norm();
    if(Math::FuzzyZero(n)) return Vector3(0.0);
    else return v*(1.0/n);
  };
  if(dist >= 0) {
    direction = Unit(direction - cp);
  }
  else{
    direction = Unit(cp - direction);
  }
  return dist;
}

void CollisionConvexHull3D::_update_transform(const RigidTransform *tranptr) {
  if(tranptr == nullptr) {
    for(int i = 0; i < 16; i++)
      this->transform[i] = 0;
    for(int i = 0; i < 4; i++)
      this->transform[4 * i + i] = 1;
  }
  else{
    const RigidTransform &tran = *tranptr;
    this->transform[0] = tran.R.data[0][0]; this->transform[1] = tran.R.data[1][0]; this->transform[2] = tran.R.data[2][0]; this->transform[3] = 0;
    this->transform[4] = tran.R.data[0][1]; this->transform[5] = tran.R.data[1][1]; this->transform[6] = tran.R.data[2][1]; this->transform[7] = 0;
    this->transform[8] = tran.R.data[0][2]; this->transform[9] = tran.R.data[1][2]; this->transform[10] = tran.R.data[2][2]; this->transform[11] = 0;
    this->transform[12] = tran.t[0]; this->transform[13] = tran.t[1]; this->transform[14] = tran.t[2]; this->transform[15] = 1;
  }
  DT_SetMatrixd(this->object, this->transform);
  return;
}

std::ostream& operator << (std::ostream& out,const ConvexHull3D& b)
{
  out<<b.points.size() / 3<<"    ";
  for(size_t i=0;i<b.points.size();i++){
    out<<b.points[i]<<"  ";
    if(i % 3 == 2)
      out << "\n";
  }
  return out;
}


std::istream& operator >> (std::istream& in, ConvexHull3D& b)
{
  size_t n;
  in>>n;
  b.points.resize(n * 3);
  for(size_t i=0;i<b.points.size();i++)
    in >> b.points[i];
  return in;
}

} //namespace Math3D
