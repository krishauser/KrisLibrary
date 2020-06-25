#include "ConvexHull3D.h"
#include <iostream>
#include <limits>
#include <KrisLibrary/utils/AnyValue.h>

using namespace std;
using namespace Math3D;

namespace Geometry {

/** @ingroup Geometry
 * @brief A 3D convex hull class
 *
 * This class usess SOLID3 library as backend and not so many function are exposed to the user.
 * 
 */

void ConvexHull3D::setPoints(const Vector& a) {
  type = Polytope;
  int asize = a.size();
  std::vector<double> points;
  if(asize % 3 != 0) {
    std::cout << "The size of point vector is not dividable by 3!\n";
    int use_size = int(asize / 3.) * 3;
    points.resize(use_size);
    std::copy(a.begin(), a.begin() + use_size, points.begin());
  }
  else{
    points = a;
  }
  data = points;
}

void ConvexHull3D::setPoints(const std::vector<double>& a) {
  type = Polytope;
  std::vector<double> points(a.begin(), a.end());
  data = points;
}

void ConvexHull3D::setPoints(const std::vector<Vector3> & a) {
  type = Polytope;
  std::vector<double> points;
  points.resize(a.size() * 3);
  int idx = 0;
  for(auto &ai : a) {
    points[idx++] = ai.x;
    points[idx++] = ai.y;
    points[idx++] = ai.z;
  }
  data = points;
}

void ConvexHull3D::setPoints(const std::vector<Vector> & a) {
  type = Polytope;
  std::vector<double> points;
  points.resize(a.size() * 3);
  int idx = 0;
  for(auto &ai : a) {
    points[idx++] = ai[0];
    points[idx++] = ai[1];
    points[idx++] = ai[2];
  }
  data = points;
}


void ConvexHull3D::from_hulls(const ConvexHull3D &hull1, const ConvexHull3D &hull2, bool is_free) {
  if(is_free) {
    this->type = ConvexHull3D::HullFree;
  }
  else{
    this->type = ConvexHull3D::Hull;
  }
  data = std::make_pair(hull1, hull2);
}

std::tuple<double, Vector3, Vector3> dist_func(DT_ObjectHandle object1, DT_ObjectHandle object2) {
    DT_SetAccuracy((DT_Scalar)1e-6);
    DT_SetTolerance((DT_Scalar)(1e-6));
    DT_Vector3 point1, point2, point3, point4;
    DT_Scalar dist = DT_GetClosestPair(object1, object2, point1, point2);
    //std::cout << "dist = " << dist << std::endl;
    Vector3 p1, p2;
    if(dist > 1e-3) {  // consider not colliding
      p1.set(point1);
      p2.set(point2);
      return std::make_tuple(dist, p1, p2);
    }
    else{
        DT_Bool is_pene = DT_GetPenDepth(object1, object2, point3, point4);
        p1.set(point3);
        p2.set(point4);
        if(is_pene) {
          dist = -(p1 - p2).norm();
          if(dist == 0) {
            std::cout << "------!!!!!!Potential distance computation error at ispene, return 0!!!!!!------\n";
          }
          return std::make_tuple(dist, p1, p2);
        }
        else{
          p1.set(point1);
          p2.set(point2);
          if(dist == 0){
            std::cout << "------!!!!!!Potential distance computation error, return 0, use workaround!!!!!!------\n";
            if(object1 == object2) {
              std::cout << "Work around is not feasible, oops\n";
              return std::make_tuple(dist, p1, p2);
            }
            // a workaround is to temporarily move one object for some distance, compute it and move it back
            double m[16];
            DT_GetMatrixd(object1, m);
            m[12] += 1e-5;
            m[13] += 1e-5;
            m[14] += 1e-5;
            DT_SetMatrixd(object1, m);
            auto rst = dist_func(object1, object2);
            m[12] -= 1e-5;
            m[13] -= 1e-5;
            m[14] -= 1e-5;
            DT_SetMatrixd(object1, m);
            return rst;
          }
          return std::make_tuple(dist, p1, p2);
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
  auto &points = this->points();
  auto &hpoints = h.points();
  DT_VertexBaseHandle base1 = DT_NewVertexBase(points.data(), (DT_Size)(3 * sizeof(double)));
  DT_VertexBaseHandle base2 = DT_NewVertexBase(hpoints.data(), (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_VertexRange(0, points.size() / 3);
  DT_EndPolytope();
  DT_ShapeHandle cube2 = DT_NewPolytope(base2);
  DT_VertexRange(0, hpoints.size() / 3);
  DT_EndPolytope();
  DT_ObjectHandle object1 = DT_CreateObject(NULL, cube1);
  DT_ObjectHandle object2 = DT_CreateObject(NULL, cube2);
  return dist_func(object1, object2);
}

//TODO: here I transform all points and this may not be efficient
void ConvexHull3D::Transform(const Matrix4 &T) {
  Assert(type == Polytope);
  //std::cout << "Transform points\n";
  auto points = this->points();
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
  Assert(type == Polytope);
  auto &points = this->points();
  int n_point = points.size() / 3;
  Vector3 bmin(points.data()), bmax(points.data());
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

std::vector<double> &ConvexHull3D::points() {
  Assert(type == Polytope);
  return *AnyCast<vector<double> >(&this->data);
}

const std::vector<double> &ConvexHull3D::points() const {
  Assert(type == Polytope);
  return *AnyCast<vector<double> >(&this->data);
}

// create the shape handle...
DT_ShapeHandle ConvexHull3D::shape_handle() const{
  switch(type) {
    case Hull: {  // if it's a hull
      const prch3d &prdata = *AnyCast<prch3d>(&this->data);
      DT_ShapeHandle handle1 = prdata.first.shape_handle();
      DT_ShapeHandle handle2 = prdata.second.shape_handle();
      return DT_NewHull(handle1, handle2);
    }
    case HullFree : {
      const prch3d &prdata = *AnyCast<prch3d>(&this->data);
      DT_ShapeHandle handle1 = prdata.first.shape_handle();
      DT_ShapeHandle handle2 = prdata.second.shape_handle();
      return DT_NewHullFree(handle1, handle2);
    }
    case Polytope: {
      std::vector<double> points = this->points();
      DT_VertexBaseHandle base1 = DT_NewVertexBase(points.data(), (DT_Size)(3 * sizeof(double)));
      DT_ShapeHandle cube1 = DT_NewPolytope(base1);
      DT_VertexRange(0, points.size() / 3);
      DT_EndPolytope();
      return cube1;
    }
    case Trans: {
      std::vector<double> points = this->points();
      DT_VertexBaseHandle base1 = DT_NewVertexBase(points.data(), (DT_Size)(3 * sizeof(double)));
      DT_ShapeHandle cube1 = DT_NewPolytope(base1);
      DT_VertexRange(0, points.size() / 3);
      DT_EndPolytope();
      return cube1;
    }
    default: {
      FatalError("Not implemented for shape other than Hull and Polytope\n");
    }
  }
}

CollisionConvexHull3D::CollisionConvexHull3D(const ConvexHull3D& hull) {
  type = hull.type;
  switch(hull.type){
    case ConvexHull3D::Composite:
    {
      std::vector<DT_ObjectHandle> v_handles;
      const std::vector<ConvexHull3D> &v_hulls = *AnyCast<std::vector<ConvexHull3D> >(&hull.data);
      for(auto &hull: v_hulls) {
        v_handles.push_back(DT_CreateObject(nullptr, hull.shape_handle()));
      }
      break;
    }
    case ConvexHull3D::Hull:  // hull of two objects
    {
      DT_ShapeHandle shape = hull.shape_handle();
      data = DT_CreateObject(nullptr, shape);
      break;
    }
    case ConvexHull3D::HullFree:
    {
      DT_ShapeHandle shape = hull.shape_handle();
      data = DT_CreateObject(nullptr, shape);
      break;
    }
    case ConvexHull3D::Polytope:
    {
      DT_ShapeHandle shape = hull.shape_handle();
      data = DT_CreateObject(nullptr, shape);
      break;
    }
    case ConvexHull3D::Trans:
    {
      DT_ShapeHandle shape = hull.shape_handle();
      DT_ShapeHandle hull_handle = DT_NewHullTran(shape);
      data = DT_CreateObject(nullptr, hull_handle);
      break;
    }
    default:
    {
      FatalError("CollisionConvexHull3D initialization currently only support Polytope and Hull\n");
    }
  }
}

CollisionConvexHull3D::CollisionConvexHull3D(const ConvexHull3D& hull1, const ConvexHull3D& hull2, bool is_free) {
  assert(hull1.type == ConvexHull3D::Polytope && hull2.type == ConvexHull3D::Polytope);
  type = ConvexHull3D::HullFree;
  DT_ShapeHandle handle1 = hull1.shape_handle();
  DT_ShapeHandle handle2 = hull2.shape_handle();
  if(is_free){
    DT_ShapeHandle shape = DT_NewHullFree(handle1, handle2);
    data = DT_CreateObject(nullptr, shape);
  }
  else{
    DT_ShapeHandle shape = DT_NewHull(handle1, handle2);
    data = DT_CreateObject(nullptr, shape);
  }
}

DT_ObjectHandle& CollisionConvexHull3D::object() {
  Assert(type != ConvexHull3D::Composite);  // only composite has to be handled differently
  return *AnyCast<DT_ObjectHandle>(&this->data);
}

std::vector<DT_ObjectHandle> & CollisionConvexHull3D::objects() {
  Assert(type == ConvexHull3D::Composite);
  return *AnyCast<std::vector<DT_ObjectHandle> >(&this->data);
}

Vector3 __Unit__(const Vector3& v)
{
  Real n = v.norm();
  if(Math::FuzzyZero(n)) return Vector3(0.0);
  else return v*(1.0/n);
};

double CollisionConvexHull3D::Distance(const Vector3 &pnt, const RigidTransform *tran) {
  DT_VertexBaseHandle base1 = DT_NewVertexBase(pnt.data, (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_ObjectHandle obj1 = DT_CreateObject(NULL, cube1);
  DT_VertexRange(0, 1);
  DT_EndPolytope();
  this->_update_transform(tran);
  if(type == ConvexHull3D::Composite) {
    double dist = std::numeric_limits<double>::infinity();  // return the minimum
    std::vector<DT_ObjectHandle> &v_hdls = this->objects();
    for(auto &hdl : v_hdls) {
      double new_dist = std::get<0>(dist_func(hdl, obj1));
      if(new_dist < dist)
        dist = new_dist;
    }
    return  dist;
  }
  else{
    this->_update_transform(tran);
    return std::get<0>(dist_func(this->object(), obj1));
  }
}

Real CollisionConvexHull3D::ClosestPoints(const Vector3& pnt,Vector3& cp,Vector3& direction, const RigidTransform *tran) {
  DT_VertexBaseHandle base1 = DT_NewVertexBase(pnt.data, (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_ObjectHandle obj1 = DT_CreateObject(NULL, cube1);
  DT_VertexRange(0, 1);
  DT_EndPolytope();
  this->_update_transform(tran);
  double dist = std::numeric_limits<double>::infinity();

  if(this->type == ConvexHull3D::Composite) {
    double new_dist;
    Vector3 new_cp, new_direction;
    for(auto &obj : this->objects()) {
      std::tie(new_dist, new_cp, new_direction) = dist_func(obj, obj1);
      if(new_dist < dist) {
        dist = new_dist;
        cp = new_cp;
        direction = new_direction;
      }
    }
  }
  else{
    std::tie(dist, cp, direction) = dist_func(this->object(), obj1);
  }
  if(dist >= 0)
    direction = __Unit__(direction - cp);
  else
    direction = __Unit__(cp - direction);
  return dist;
}

// compute the closest points between two convexhull3d with collision data and store results somewhere
Real CollisionConvexHull3D::ClosestPoints(CollisionConvexHull3D& g, Vector3& cp, Vector3& direction, const RigidTransform *tran, const RigidTransform *tran2){
  this->_update_transform(tran);
  g._update_transform(tran2);
  double dist = std::numeric_limits<double>::infinity();
  double _dist;
  Vector3 _cp, _direction;
  if(this->type == ConvexHull3D::Composite) {
    if(g.type == ConvexHull3D::Composite) {  // vector vs vector, just get the minimum one of them
      for(auto &obj1 : this->objects()) {
        for(auto &obj2 : g.objects()) {
          std::tie(_dist, _cp, _direction) = dist_func(obj1, obj2);
          if(_dist < dist) {
            dist = _dist;
            cp = _cp;
            direction = _direction;
          }
        }
      }
    }
    else{
      for(auto &obj1: this->objects()) {  // vector vs single
        std::tie(_dist, _cp, _direction) = dist_func(obj1, g.object());
        if(_dist < dist) {
          dist = _dist;
          cp = _cp;
          direction = _direction;
        }
      }
    }
  }
  else if(g.type == ConvexHull3D::Composite) {  // single vs object
    for(auto &obj2 : g.objects()) {
      std::tie(_dist, _cp, _direction) = dist_func(this->object(), obj2);
      if(_dist < dist) {
        dist = _dist;
        cp = _cp;
        direction = _direction;
      }
    }
  }
  else{
    std::tie(dist, cp, direction) = dist_func(this->object(), g.object());
  }
  // change direction based on distance
  if(dist >= 0) {
    direction = __Unit__(direction - cp);
  }
  else{
    direction = __Unit__(cp - direction);
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
    // this->transform[0] = tran.R.data[0][0]; this->transform[1] = tran.R.data[1][0]; this->transform[2] = tran.R.data[2][0]; this->transform[3] = 0;
    // this->transform[4] = tran.R.data[0][1]; this->transform[5] = tran.R.data[1][1]; this->transform[6] = tran.R.data[2][1]; this->transform[7] = 0;
    // this->transform[8] = tran.R.data[0][2]; this->transform[9] = tran.R.data[1][2]; this->transform[10] = tran.R.data[2][2]; this->transform[11] = 0;
    this->transform[0] = tran.R.data[0][0]; this->transform[1] = tran.R.data[0][1]; this->transform[2] = tran.R.data[0][2]; this->transform[3] = 0;
    this->transform[4] = tran.R.data[1][0]; this->transform[5] = tran.R.data[1][1]; this->transform[6] = tran.R.data[1][2]; this->transform[7] = 0;
    this->transform[8] = tran.R.data[2][0]; this->transform[9] = tran.R.data[2][1]; this->transform[10] = tran.R.data[2][2]; this->transform[11] = 0;
    this->transform[12] = tran.t[0]; this->transform[13] = tran.t[1]; this->transform[14] = tran.t[2]; this->transform[15] = 1;
  }
  if(this->type == ConvexHull3D::Composite) {
    for(auto &obj: this->objects())
      DT_SetMatrixd(obj, this->transform);
  }
  else{
    DT_SetMatrixd(this->object(), this->transform);
  }
  return;
}

void CollisionConvexHull3D::_update_relative_transform(const RigidTransform *tranptr) {
  Assert(this->type == ConvexHull3D::Trans);
  const RigidTransform &tran = *tranptr;
  DT_ObjectHandle &object = this->object();
  double transform[16];  // this overwrites the other one
  transform[0] = tran.R.data[0][0]; transform[1] = tran.R.data[0][1]; transform[2] = tran.R.data[0][2]; transform[3] = 0;
  transform[4] = tran.R.data[1][0]; transform[5] = tran.R.data[1][1]; transform[6] = tran.R.data[1][2]; transform[7] = 0;
  transform[8] = tran.R.data[2][0]; transform[9] = tran.R.data[2][1]; transform[10] = tran.R.data[2][2]; transform[11] = 0;
  transform[12] = tran.t[0]; transform[13] = tran.t[1]; transform[14] = tran.t[2]; transform[15] = 1;
  //std::cout << "Hull3D update transform\n";
  DT_SetRelativeMatrixd(object, transform);
}

void CollisionConvexHull3D::_update_free_relative_transform(const RigidTransform *tranptr) {
  Assert(this->type == ConvexHull3D::HullFree);
  const RigidTransform &tran = *tranptr;
  DT_ObjectHandle &object = this->object();
  double transform[16];  // this overwrites the other one
  transform[0] = tran.R.data[0][0]; transform[1] = tran.R.data[0][1]; transform[2] = tran.R.data[0][2]; transform[3] = 0;
  transform[4] = tran.R.data[1][0]; transform[5] = tran.R.data[1][1]; transform[6] = tran.R.data[1][2]; transform[7] = 0;
  transform[8] = tran.R.data[2][0]; transform[9] = tran.R.data[2][1]; transform[10] = tran.R.data[2][2]; transform[11] = 0;
  transform[12] = tran.t[0]; transform[13] = tran.t[1]; transform[14] = tran.t[2]; transform[15] = 1;
  //std::cout << "Hull3D update transform\n";
  DT_SetFreeRelativeMatrixd(object, transform);
}

void CollisionConvexHull3D::_find_support(const double *dir, double *out) {
  DT_ObjectHandle object = this->object();
  DT_GetSupport(object, dir, out);
}

std::ostream& operator << (std::ostream& out,const ConvexHull3D& b)
{
  vector<double> points = *AnyCast<vector<double> >(&b.data);
  out<<points.size() / 3<<"    ";
  for(size_t i=0;i<points.size();i++){
    out<<points[i]<<"  ";
    if(i % 3 == 2)
      out << "\n";
  }
  return out;
}


std::istream& operator >> (std::istream& in, ConvexHull3D& b)
{
  size_t n;
  in>>n;
  // b.points.resize(n * 3);
  // for(size_t i=0;i<b.points.size();i++)
  //   in >> b.points[i];
  return in;
}

} //namespace Geometry
