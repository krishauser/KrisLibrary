#include "ConvexHull3D.h"
#include <iostream>
#include <limits>
#include <KrisLibrary/utils/AnyValue.h>
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/Timer.h>
#include "SOLID.h"
extern "C" {
#include <qhull/qhull_a.h>
}
#include <stdio.h>

DECLARE_LOGGER(Geometry)

using namespace std;
using namespace Math3D;


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

    Timer timer;

    qh NOerrexit = False;
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
    if(timer.ElapsedTime() > 0.25)
      LOG4CXX_INFO(GET_LOGGER(Geometry),"QHull determined "<<facets.size()<<" facets for "<<points.size()<<" points in "<<timer.ElapsedTime()<<"s");

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

    Timer timer;

    qh NOerrexit = False;
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

    if(timer.ElapsedTime() > 0.25)
      LOG4CXX_INFO(GET_LOGGER(Geometry),"QHull determined"<<facets.size()<<" facets for "<<points.size()<<" points in "<<timer.ElapsedTime()<<"s");

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


ConvexHull3D::ShapeHandleContainer::ShapeHandleContainer(DT_ShapeHandle _data, DT_ShapeHandle _data2)
:data(_data),data2(_data2)
{
  Assert(data!=NULL);
  //printf("CREATING SHAPE HANDLE CONTAINER WITH DATA %p\n",data);
  object_ref = DT_CreateObject(NULL, data);
}

ConvexHull3D::ShapeHandleContainer::~ShapeHandleContainer()
{
  if(object_ref) {
    DT_DestroyObject(object_ref);
  }
  if(data) {
    //printf("DELETING SHAPE HANDLE %p IN CONTAINER\n",data);
    DT_DeleteShape(data);
  }
  if(data2) {
    //printf("DELETING SHAPE HANDLE %p IN CONTAINER\n",data2);
    DT_DeleteShape(data2);
  }
}


ConvexHull3D::ConvexHull3D()
:type(Empty)
{}

ConvexHull3D::ConvexHull3D(const ConvexHull3D& rhs)
:type(Empty)
{
  *this = rhs;
}

const ConvexHull3D& ConvexHull3D::operator = (const ConvexHull3D& rhs)
{
  type = rhs.type;
  data = rhs.data;
  shapeHandle = rhs.shapeHandle; //Note: shared data
  return *this;
}

DT_ShapeHandle _MakePolytope(const vector<double>& points)
{
  DT_VertexBaseHandle base1 = DT_NewVertexBase(points.data(), (DT_Size)(3 * sizeof(double)));
  DT_ShapeHandle cube1 = DT_NewPolytope(base1);
  DT_VertexRange(0, points.size() / 3);
  DT_EndPolytope();
  return cube1;
}

bool ConvexHull3D::Set(const GeometricPrimitive3D& g)
{
  switch(g.type) {
  case GeometricPrimitive3D::Type::Point:
    {
      Set(*AnyCast_Raw<Vector3>(&g.data));
      break;
    }
    case GeometricPrimitive3D::Type::Segment:
    {
      Set(*AnyCast_Raw<Segment3D>(&g.data));
      break;
    }
    case GeometricPrimitive3D::Type::Triangle:
    {
      const Triangle3D& t = *AnyCast_Raw<Triangle3D>(&g.data);
      Set(t);
      break;
    }
    case GeometricPrimitive3D::Type::AABB:
    {
      const AABB3D& b = *AnyCast_Raw<AABB3D>(&g.data);
      Set(b);
      break;
    }
    case GeometricPrimitive3D::Type::Box:
    {
      const Box3D& b = *AnyCast_Raw<Box3D>(&g.data);
      Set(b);
      break;
    }
    case GeometricPrimitive3D::Type::Sphere:
    {
      const Sphere3D& s = *AnyCast_Raw<Sphere3D>(&g.data);
      Set(s);
      break;
    }
    case GeometricPrimitive3D::Type::Polygon:
    {
      const Polygon3D& p = *AnyCast_Raw<Polygon3D>(&g.data);
      SetPoints(p.vertices);
      break;
    }
    default:
    {
      LOG4CXX_WARN(GET_LOGGER(Geometry),"ConvexHull3D::Set: Unsupported geometric primitive type "<<g.type);
      return false;
    }
  }
  return true;
}

void ConvexHull3D::Set(const Vector3& a) {
  type = Point;
  data = a;
  DT_Vector3 data;
  a.get(data);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewPoint(data));
}

void ConvexHull3D::Set(const Sphere3D& s)
{
  type = Sphere;
  data = s;
  if(s.center.isZero()) {
    shapeHandle = make_shared<ShapeHandleContainer>(DT_NewSphere(s.radius));
  }
  else {
    RigidTransform xform;
    xform.R.setIdentity();
    xform.t = s.center;
    double xform_data[16];
    Matrix4(xform).get(xform_data);
    DT_ShapeHandle sphere = DT_NewSphere(s.radius);
    shapeHandle = make_shared<ShapeHandleContainer>(DT_NewTransform(sphere,xform_data),sphere);
  }
}

void ConvexHull3D::Set(const Segment3D& s) {
  type = LineSegment;
  data = s;
  DT_Vector3 src,dest;
  s.a.get(src);
  s.b.get(dest);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewLineSegment(src,dest));
}

void ConvexHull3D::Set(const Triangle3D& t)
{
  vector<Vector3> verts = {t.a,t.b,t.c};
  SetPoints(verts);
}

void ConvexHull3D::Set(const AABB3D& bb)
{
  type = Box;
  Box3D b;
  b.set(bb);
  data = b;
  DT_Vector3 dim;
  (bb.bmax - bb.bmin).get(dim);
  RigidTransform xform;
  xform.R.setIdentity();
  xform.t = ((bb.bmax + bb.bmin) * 0.5);
  double xform_data[16];
  Matrix4(xform).get(xform_data);
  DT_ShapeHandle box = DT_NewBox(dim[0],dim[1],dim[2]);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewTransform(box,xform_data),box);
}

void ConvexHull3D::Set(const Box3D& b)
{
  type = Box;
  data = b;
  RigidTransform xform;
  b.getTransform(xform);
  xform.t += xform.R*b.dims*0.5;  //shift to center
  double xform_data[16];
  Matrix4(xform).get(xform_data);
  DT_ShapeHandle box = DT_NewBox(b.dims.x,b.dims.y,b.dims.z);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewTransform(box,xform_data),box);
}

void ConvexHull3D::SetPointBuffer(const Vector& a) {
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
  SetPointBuffer(points);
}

void ConvexHull3D::SetPointBuffer(const std::vector<double>& points) {
  type = Polytope;
  data = points;
  shapeHandle = make_shared<ShapeHandleContainer>(_MakePolytope(AsPolytope()));
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
  SetPointBuffer(points);
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

bool ConvexHull3D::Contains(const Vector3& pnt) const
{
  if(!shapeHandle) return false;
  ConvexHull3D pnt_hull;
  pnt_hull.Set(pnt);
  return Collides(pnt_hull);
}


bool ConvexHull3D::RayCast(const Ray3D& r,Real* param,Real maxParam) const
{
  if(!shapeHandle) return false;
  if(type != ConvexHull3D::Type::Box && type != ConvexHull3D::Type::Sphere) {
    //SOLID has not implemented these yet
    return false;
  }
  maxParam = Min(maxParam,10000.0);
  Vector3 target = r.source + r.direction;
  Vector3 pt;
  DT_Scalar distance;
  DT_Vector3 normal;
  DT_Bool collide = DT_ObjectRayCast(shapeHandle->object_ref, r.source, target, maxParam, &distance, normal);
  if(collide) {
    if(param) *param = distance;
    return true;
  }
  return false;
}


bool ConvexHull3D::Collides(const ConvexHull3D& g,Vector3* common_point) const
{
  DT_SetAccuracy((DT_Scalar)1e-6);
  DT_SetTolerance((DT_Scalar)(1e-6));
  DT_Vector3 point1;
  DT_Bool collide = DT_GetCommonPoint(shapeHandle->object_ref, g.shapeHandle->object_ref, point1);
  if(collide && common_point) 
    common_point->set(point1);
  return collide;
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
            LOG4CXX_INFO(GET_LOGGER(Geometry), "ConvexHull3D::Distance: potential distance computation error, distance is 0");
          }
          return std::make_tuple(dist, p1, p2);
        }
        else{
          p1.set(point1);
          p2.set(point2);
          if(dist == 0){
            LOG4CXX_INFO(GET_LOGGER(Geometry), "ConvexHull3D::Distance: potential distance computation error, distance is 0 & using workaround");
            if(object1 == object2) {
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
Real ConvexHull3D::Distance(const ConvexHull3D &h) const {
    return std::get<0>(ClosestPoints(h));
}

Real ConvexHull3D::Distance(const Vector3 &pnt) const {
  ConvexHull3D pnt_hull;
  pnt_hull.Set(pnt);
  return Distance(pnt_hull);
}

Real ConvexHull3D::ClosestPoint(const Vector3& pt,Vector3& cp,Vector3& direction) const {
  // first create hull and compute a lot
  ConvexHull3D pnt_hull;
  pnt_hull.Set(pt);
  return ClosestPoints(pnt_hull, cp, direction);
}

Real ConvexHull3D::ClosestPoints(const ConvexHull3D& g, Vector3& cp, Vector3& direction) const {
  Real dist = 0;
  std::tie(dist, cp, direction) = ClosestPoints(g);
  if(dist >= 0) {
    direction.setNormalized(direction - cp);
  }
  else{
    direction.setNormalized(cp - direction);
  }
  return dist;
}

std::tuple<Real, Vector3, Vector3> ConvexHull3D::ClosestPoints(const ConvexHull3D &h) const {
  if(!shapeHandle || !h.shapeHandle) return std::make_tuple(Inf,Vector3(0.0),Vector3(0.0));
  auto res = dist_func(shapeHandle->object_ref, h.shapeHandle->object_ref);
  return res;
}

void ConvexHull3D::Transform(const Matrix4 &T)
{
  if(type != Polytope) FatalError("TODO: Implement Transform for types other than Polytope");
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
  SetPointBuffer(points);
}

AABB3D ConvexHull3D::GetAABB() const
{
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

const ConvexHull3D::SphereData& ConvexHull3D::AsSphere() const {
  Assert(type == Sphere);
  return *AnyCast<SphereData>(&this->data);
}

const ConvexHull3D::LineSegmentData& ConvexHull3D::AsLineSegment() const {
  Assert(type == LineSegment);
  return *AnyCast<LineSegmentData>(&this->data);
}

const ConvexHull3D::BoxData& ConvexHull3D::AsBox() const
{
  Assert(type == Box);
  return *AnyCast<BoxData>(&this->data);
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
    const auto& items = AsHull();
    return items.first.NumPrimitives() + items.second.NumPrimitives();
  }
  else if(type == Trans)
    return AsTrans().first.NumPrimitives();
  else if(type == Empty)
    return 0;
  else
    return 1;
}

vector<Vector3> ConvexHull3D::GetPoints() const
{
  vector<Vector3> res;
  if(type == Polytope) {
    const auto& pts = AsPolytope();
    res.resize(pts.size()/3);
    for(size_t i=0;i<pts.size();i+=3) 
      res[i/3].set(&pts[i]);
    return res;
  }
  else if(type == Hull) {
    const auto& items = AsHull();
    res = items.first.GetPoints();
    vector<Vector3> other = items.second.GetPoints();
    res.insert(res.end(),other.begin(),other.end());
    return res;
  }
  else if(type == Trans) {
    const auto& data = AsTrans();
    res = data.first.GetPoints();
    for(size_t i=0;i<res.size();i++)
      res[i] = data.second * res[i];
    return res;
  }
  else if(type == Trans) {
    const auto& data = AsTrans();
    res = data.first.GetPoints();
    for(size_t i=0;i<res.size();i++)
      res[i] = data.second * res[i];
    return res;
  }
  else if(type == Point) {
    res.resize(1);
    res[0] = AsPoint();
    return res;
  }
  else if(type == LineSegment) {
    res.resize(2);
    res[0] = AsLineSegment().a;
    res[1] = AsLineSegment().b;
    return res;
  }
  FatalError("Can't get points for this type yet");
  return res;
}

GeometricPrimitive3D ConvexHull3D::GetPrimitive(int elem) const
{
  if(type == Polytope) {
    const auto& pnt = AsPolytope();
    Vector3 point(pnt[elem * 3], pnt[elem * 3 + 1], pnt[elem * 3 + 2]);
    return GeometricPrimitive3D(point);
  }
  else if(type == Hull) {
    const auto& items = AsHull();
    if(elem < (int)items.first.NumPrimitives())
      return items.first.GetPrimitive(elem);
    else
      return items.second.GetPrimitive(elem-items.first.NumPrimitives());
  }
  else if(type == Trans) {
    const auto& data = AsTrans();
    auto g = data.first.GetPrimitive(elem);
    g.Transform(data.second);
    return g;
  } 
  else if(type == Point)
    return GeometricPrimitive3D(AsPoint());
  else if(type == LineSegment)
    return GeometricPrimitive3D(AsLineSegment());
  else
    FatalError("Can't get primitive of this type yet");
  return GeometricPrimitive3D();
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
