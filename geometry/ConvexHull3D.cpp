#include "ConvexHull3D.h"
#include <iostream>
#include <limits>
#include <KrisLibrary/utils/AnyValue.h>
#include "SOLID.h"
extern "C" {
#include <qhull/qhull_a.h>
}
#include <stdio.h>

using namespace std;
using namespace Math3D;

Vector3 __Unit__(const Vector3& v)
{
  Real n = v.norm();
  if(Math::FuzzyZero(n)) return Vector3(0.0);
  else return v*(1.0/n);
};

/*-------------------------------------------------
-rand & srand- generate pseudo-random number between 1 and 2^31 -2
  from Park & Miller's minimimal standard random number generator
  Communications of the ACM, 31:1192-1201, 1988.
notes:
  does not use 0 or 2^31 -1
  this is silently enforced by qh_srand()
  copied from geom2.c
*/
static int seed = 1;  /* global static */

extern "C" int qh_rand( void) {
#define qh_rand_a 16807
#define qh_rand_m 2147483647
#define qh_rand_q 127773  /* m div a */
#define qh_rand_r 2836    /* m mod a */
  int lo, hi, test;

  hi = seed / qh_rand_q;  /* seed div q */
  lo = seed % qh_rand_q;  /* seed mod q */
  test = qh_rand_a * lo - qh_rand_r * hi;
  if (test > 0)
    seed= test;
  else
    seed= test + qh_rand_m;
  return seed;
} /* rand */

extern "C" void qh_srand( int newseed) {
  if (newseed < 1)
    seed= 1;
  else if (newseed >= qh_rand_m)
    seed= qh_rand_m - 1;
  else
    seed= newseed;
} /* qh_srand */


namespace Geometry {

static char qhull_options[] = "qhull Qts i Tv";


bool ConvexHull3D_Qhull(const vector<Vector3>& points,vector<vector<int> >& facets)
{
  int curlong, totlong, exitcode;
  
    facetT *facet;
    vertexT *vertex;
    vertexT **vertexp;

    qh_init_A(stdin, stdout, stderr, 0, NULL);
    if ((exitcode = setjmp(qh errexit))) 
    {
      return false;
    }
    qh_initflags(qhull_options);
    qh_init_B(const_cast<double*>(&points[0].x), points.size(), 3, False);
    qh_qhull();
    qh_check_output();
    
    facets.resize(0);
    FORALLfacets 
    {
      setT *vertices = qh_facet3vertex(facet);

      vector<int> facet;  
      FOREACHvertex_(vertices) 
      {
        facet.push_back(qh_pointid(vertex->point));
      }
      facets.push_back(facet);
    }

    qh NOerrexit = True;
    qh_freeqhull(!qh_ALL);
    qh_memfreeshort(&curlong, &totlong);

  return true;
}

bool ConvexHull3D_Qhull(const vector<double>& points,vector<vector<int> >& facets)
{
  int curlong, totlong, exitcode;
  
    facetT *facet;
    vertexT *vertex;
    vertexT **vertexp;

    qh_init_A(stdin, stdout, stderr, 0, NULL);
    if ((exitcode = setjmp(qh errexit))) 
    {
      return false;
    }
    qh_initflags(qhull_options);
    qh_init_B(const_cast<double*>(&points[0]), points.size()/3, 3, False);
    qh_qhull();
    qh_check_output();
    
    facets.resize(0);
    FORALLfacets 
    {
      setT *vertices = qh_facet3vertex(facet);

      vector<int> facet;  
      FOREACHvertex_(vertices) 
      {
        facet.push_back(qh_pointid(vertex->point));
      }
      facets.push_back(facet);
    }

    qh NOerrexit = True;
    qh_freeqhull(!qh_ALL);
    qh_memfreeshort(&curlong, &totlong);

  return true;
}

bool ConvexHull3D_Qhull(const std::vector<Vector3>& points,std::vector<IntTriple>& tris)
{
  vector<vector<int> > facets;
  if(!ConvexHull3D_Qhull(points,facets)) return false;
  tris.resize(0);
  for(size_t i=0;i<facets.size();i++) {
    const auto& f = facets[i];
    assert(f.size() >= 3);
    for(size_t j=1;j+1<f.size();j++) 
      tris.push_back(IntTriple(f[0],f[j],f[j+1]));
  }
  return true;
}


ConvexHull3D::ShapeHandleContainer::ShapeHandleContainer(DT_ShapeHandle _data)
:data(_data)
{}

ConvexHull3D::ShapeHandleContainer::~ShapeHandleContainer()
{
  if(data) DT_DeleteShape(data);
}


ConvexHull3D::ConvexHull3D()
:type(Empty)
{}

DT_ShapeHandle _MakePolytope(const vector<double>& points)
{
  DT_VertexBaseHandle base1 = DT_NewVertexBase(points.data(), (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_VertexRange(0, points.size() / 3);
  DT_EndPolytope();
  return cube1;
}

void ConvexHull3D::SetPoint(const Vector3& a) {
  type = Point;
  data = a;
  DT_Vector3 data;
  a.get(data);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewPoint(data));
}

void ConvexHull3D::SetPoints(const Vector& a) {
  int asize = a.size();
  std::vector<double> points;
  if(asize % 3 != 0) {
    int use_size = int(asize / 3.) * 3;
    points.resize(use_size);
    std::copy(a.begin(), a.begin() + use_size, points.begin());
  }
  else{
    points = a;
  }
  SetPoints(points);
}

void ConvexHull3D::SetPoints(const std::vector<double>& points) {
  type = Polytope;
  data = points;
  shapeHandle = make_shared<ShapeHandleContainer>(_MakePolytope(points));
}

void ConvexHull3D::SetPoints(const std::vector<Vector3> & a) {
  std::vector<double> points;
  points.resize(a.size() * 3);
  int idx = 0;
  for(auto &ai : a) {
    points[idx++] = ai.x;
    points[idx++] = ai.y;
    points[idx++] = ai.z;
  }
  SetPoints(points);
}

void ConvexHull3D::SetPoints(const std::vector<Vector> & a) {
  std::vector<double> points;
  points.resize(a.size() * 3);
  int idx = 0;
  for(auto &ai : a) {
    points[idx++] = ai[0];
    points[idx++] = ai[1];
    points[idx++] = ai[2];
  }
  SetPoints(points);
}

void ConvexHull3D::SetTrans(const ConvexHull3D &hull, const RigidTransform& xform)
{
  this->type = ConvexHull3D::Trans;
  data = std::make_pair(hull,xform);
  double xform_data[16];
  Matrix4(xform).get(xform_data);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewTransform(hull.shapeHandle->data,xform_data));
}

void ConvexHull3D::SetHull(const ConvexHull3D &hull1, const ConvexHull3D &hull2)
{
  this->type = ConvexHull3D::Hull;
  data = std::make_pair(hull1, hull2);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewHullFree(hull1.shapeHandle->data, hull2.shapeHandle->data));
}

std::tuple<Real, Vector3, Vector3> dist_func(DT_ObjectHandle object1, DT_ObjectHandle object2) {
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
Real ConvexHull3D::Distance(const ConvexHull3D &h) {
    return std::get<0>(ClosestPoints(h));
}

Real ConvexHull3D::Distance(const Vector3 &pnt) {
  ConvexHull3D pnt_hull;
  pnt_hull.SetPoint(pnt);
  return Distance(pnt_hull);
}

Real ConvexHull3D::ClosestPoints(const Vector3& pt,Vector3& cp,Vector3& direction) const {
  // first create hull and compute a lot
  ConvexHull3D pnt_hull;
  std::vector<Vector3> vpoint{pt};
  pnt_hull.SetPoints(vpoint);
  return ClosestPoints(pnt_hull, cp, direction);
}

Real ConvexHull3D::ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const {
  Real dist = 0;
  std::tie(dist, cp, direction) = ClosestPoints(g);
  if(dist >= 0) {
    direction = __Unit__(direction - cp);
  }
  else{
    direction = __Unit__(cp - direction);
  }
  return dist;
}

std::tuple<Real, Vector3, Vector3> ConvexHull3D::ClosestPoints(const ConvexHull3D &h) const {
  if(!shapeHandle || !h.shapeHandle) return std::make_tuple(Inf,Vector3(0.0),Vector3(0.0));
  DT_ShapeHandle cube1 = shapeHandle->data;
  DT_ShapeHandle cube2 = h.shapeHandle->data;
  DT_ObjectHandle object1 = DT_CreateObject(NULL, cube1);
  DT_ObjectHandle object2 = DT_CreateObject(NULL, cube2);
  auto res = dist_func(object1, object2);
  DT_DestroyObject(object1);
  DT_DestroyObject(object2);
  return res;
}

//TODO: here I transform all points and this may not be efficient
void ConvexHull3D::Transform(const Matrix4 &T) {
  if(type != Polytope) FatalError("TODO: Implement Transform for types other than Polytope");
  //std::cout << "Transform points\n";
  auto points = this->AsPolytope();
  int n_point = points.size() / 3;
  for(int i = 0; i < n_point; i++) {
    Vector3 temp(points[3 * i], points[3 * i + 1], points[3 * i + 2]);
    Vector3 rst;
    T.mulPoint(temp, rst);
    points[3 * i] = rst.x;
    points[3 * i + 1] = rst.y;
    points[3 * i + 2] = rst.z;
  }
  SetPoints(points);
}

AABB3D ConvexHull3D::GetAABB() const {
  if(type != Polytope) FatalError("TODO: Implement GetAABB for types other than Polytope");
  const auto &points = this->AsPolytope();
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

ConvexHull3D::PolytopeData& ConvexHull3D::AsPolytope() {
  Assert(type == Polytope);
  return *AnyCast<PolytopeData>(&this->data);
}

const ConvexHull3D::PolytopeData&ConvexHull3D::AsPolytope() const {
  Assert(type == Polytope);
  return *AnyCast<PolytopeData>(&this->data);
}

const ConvexHull3D::TransData& ConvexHull3D::AsTrans() const {
  Assert(type == Trans);
  return *AnyCast<TransData>(&this->data);
}

const ConvexHull3D::PointData& ConvexHull3D::AsPoint() const {
  Assert(type == Point);
  return *AnyCast<PointData>(&this->data);
}

const ConvexHull3D::HullData& ConvexHull3D::AsHull() const {
  Assert(type == Hull);
  return *AnyCast<HullData>(&this->data);
}

size_t ConvexHull3D::NumPrimitives() const
{
  if(type == Polytope)
    return AsPolytope().size() / 3;
  else if(type == Hull) {
    auto items = AsHull();
    return items.first.NumPrimitives() + items.second.NumPrimitives();
  }
  else if(type == Trans)
    return AsTrans().first.NumPrimitives();
  else if(type == Empty)
    return 0;
  else
    return 1;
}

GeometricPrimitive3D ConvexHull3D::GetPrimitive(int elem) const
{
  if(type == Polytope) {
    auto pnt = AsPolytope();
    Vector3 point(pnt[elem * 3], pnt[elem * 3 + 1], pnt[elem * 3 + 2]);
    return GeometricPrimitive3D(point);
  }
  else if(type == Hull) {
    auto items = AsHull();
    if(elem < (int)items.first.NumPrimitives())
      return items.first.GetPrimitive(elem);
    else
      return items.second.GetPrimitive(elem-items.first.NumPrimitives());
  }
  else if(type == Trans) {
    auto data = AsTrans();
    auto g = data.first.GetPrimitive(elem);
    g.Transform(data.second);
    return g;
  } 
  else if(type == Point)
    return GeometricPrimitive3D(AsPoint());
  else
    FatalError("Can't get primitive of this type yet");
  return GeometricPrimitive3D();
}


CollisionConvexHull3D::CollisionConvexHull3D(const ConvexHull3D& hull) {
  type = hull.type;
  DT_ShapeHandle shape = hull.shapeHandle->data;
  objectHandle = DT_CreateObject(nullptr, shape);
}

CollisionConvexHull3D::CollisionConvexHull3D()  // no value, waiting to be initialized
{
  objectHandle = NULL;
}

CollisionConvexHull3D::~CollisionConvexHull3D()
{
  if(objectHandle) DT_DestroyObject(objectHandle);
}



double CollisionConvexHull3D::Distance(const Vector3 &pnt, const RigidTransform *tran) {
  ConvexHull3D cpt;
  cpt.SetPoint(pnt);
  CollisionConvexHull3D ccpt(cpt);
  this->UpdateTransform(tran);
  return std::get<0>(dist_func(objectHandle, ccpt.objectHandle));
}

Real CollisionConvexHull3D::ClosestPoints(const Vector3& pnt,Vector3& cp,Vector3& direction, const RigidTransform *tran) {
  ConvexHull3D cpt;
  cpt.SetPoint(pnt);
  CollisionConvexHull3D ccpt(cpt);
  Real dist;
  std::tie(dist, cp, direction) = dist_func(objectHandle, ccpt.objectHandle);
  // change direction based on distance
  if(dist >= 0) {
    direction = __Unit__(direction - cp);
  }
  else{
    direction = __Unit__(cp - direction);
  }
  return dist;
}

// compute the closest points between two convexhull3d with collision data and store results somewhere
Real CollisionConvexHull3D::ClosestPoints(CollisionConvexHull3D& g, Vector3& cp, Vector3& direction, const RigidTransform *tran, const RigidTransform *tran2){
  this->UpdateTransform(tran);
  g.UpdateTransform(tran2);
  double dist;
  std::tie(dist, cp, direction) = dist_func(this->objectHandle, g.objectHandle);
  // change direction based on distance
  if(dist >= 0) {
    direction = __Unit__(direction - cp);
  }
  else{
    direction = __Unit__(cp - direction);
  }
  return dist;
}

void CollisionConvexHull3D::UpdateTransform(const RigidTransform *tranptr) {
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
  DT_SetMatrixd(objectHandle, this->transform);
  return;
}

void CollisionConvexHull3D::UpdateHullSecondRelativeTransform(const RigidTransform *tranptr) {
  const RigidTransform &tran = *tranptr;
  double transform[16];  // this overwrites the other one
  transform[0] = tran.R.data[0][0]; transform[1] = tran.R.data[0][1]; transform[2] = tran.R.data[0][2]; transform[3] = 0;
  transform[4] = tran.R.data[1][0]; transform[5] = tran.R.data[1][1]; transform[6] = tran.R.data[1][2]; transform[7] = 0;
  transform[8] = tran.R.data[2][0]; transform[9] = tran.R.data[2][1]; transform[10] = tran.R.data[2][2]; transform[11] = 0;
  transform[12] = tran.t[0]; transform[13] = tran.t[1]; transform[14] = tran.t[2]; transform[15] = 1;
  //std::cout << "Hull3D update transform\n";
  if(this->type == ConvexHull3D::Hull)
    DT_SetChildRelativeMatrixd(objectHandle, 1, transform);
}

Vector3 CollisionConvexHull3D::FindSupport(const Vector3& dir) const {
  double out[3];
  DT_GetSupport(objectHandle, dir, out);
  return Vector3(out);
}

std::ostream& operator << (std::ostream& out,const ConvexHull3D& b)
{
  if(b.type != ConvexHull3D::Polytope) FatalError("TODO: output << any type but Polytope");
  const vector<double>& points = b.AsPolytope();
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
