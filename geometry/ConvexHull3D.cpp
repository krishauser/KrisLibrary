#include "ConvexHull3D.h"
#include "CollisionConvexHull.h"
#include "CollisionPrimitive.h"
#include "Conversions.h"
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


ConvexHull3D::ShapeHandleContainer::ShapeHandleContainer(DT_ShapeHandle _data)
:data(_data)
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

void ConvexHull3D::SetPoint(const Vector3& a) {
  type = Point;
  data = a;
  DT_Vector3 data;
  a.get(data);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewPoint(data));
}

void ConvexHull3D::SetLineSegment(const Segment3D& s) {
  type = LineSegment;
  data = s;
  DT_Vector3 src,dest;
  s.a.get(src);
  s.b.get(dest);
  shapeHandle = make_shared<ShapeHandleContainer>(DT_NewLineSegment(src,dest));
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

bool ConvexHull3D::Contains(const Vector3& pnt) const
{
  if(!shapeHandle) return false;
  ConvexHull3D pnt_hull;
  pnt_hull.SetPoint(pnt);
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
Real ConvexHull3D::Distance(const ConvexHull3D &h) const {
    return std::get<0>(ClosestPoints(h));
}

Real ConvexHull3D::Distance(const Vector3 &pnt) const {
  ConvexHull3D pnt_hull;
  pnt_hull.SetPoint(pnt);
  return Distance(pnt_hull);
}

Real ConvexHull3D::ClosestPoint(const Vector3& pt,Vector3& cp,Vector3& direction) const {
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

const ConvexHull3D::LineSegmentData& ConvexHull3D::AsLineSegment() const {
  Assert(type == LineSegment);
  return *AnyCast<LineSegmentData>(&this->data);
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



Geometry3DConvexHull::Geometry3DConvexHull(const ConvexHull3D& _data)
: data(_data)
{
}

bool Geometry3DConvexHull::Empty() const
{
    return data.type == ConvexHull3D::Empty;
}

size_t Geometry3DConvexHull::NumElements() const 
{
    return data.NumPrimitives();
}

shared_ptr<Geometry3D> Geometry3DConvexHull::GetElement(int elem) const
{
  return shared_ptr<Geometry3D>(new Geometry3DPrimitive(data.GetPrimitive(elem)));
}

bool Geometry3DConvexHull::Load(istream &in)
{
    in >> data;
    if(in) {
        return true;
    }
    return false;
}

bool Geometry3DConvexHull::Save(ostream& out) const
{
    out << data << endl;
    return true;
}

bool Geometry3DConvexHull::Transform(const Matrix4 &T)
{
    data.Transform(T);
    return true;
}

Geometry3D* Geometry3DConvexHull::Copy() const
{
    return new Geometry3DConvexHull(data);
}

AABB3D Geometry3DConvexHull::GetAABB() const
{
    return data.GetAABB();
}

bool Geometry3DConvexHull::Support(const Vector3& dir,Vector3& pt) const
{
  vector<Vector3> pts = data.GetPoints();
  if(pts.empty()) return false;
  Real farthest = -Inf;
  for(size_t i=0;i<pts.size();i++) {
    Real d = dir.dot(pts[i]);
    if(d > farthest) {
      pt = pts[i];
      farthest = d;
    }
  }
  return true;
}

Geometry3D* Geometry3DConvexHull::ConvertTo(Type restype, Real param,Real domainExpansion) const
{
    return NULL;
}

bool Geometry3DConvexHull::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion)
{
    if(geom->GetType() == Type::TriangleMesh) {
        MeshConvexDecomposition(dynamic_cast<const Geometry3DTriangleMesh*>(geom)->data,data,param);
        return true;
    }
    else if(geom->GetType() == Type::PointCloud) {
        PointCloudToConvexHull(dynamic_cast<const Geometry3DPointCloud*>(geom)->data,data);
        return true;
    }
    else {
        shared_ptr<Geometry3D> g2(geom->Convert(Type::TriangleMesh,param));
        if(g2) {
            return ConvertFrom(g2.get(),param);
        }
        g2.reset(geom->Convert(Type::PointCloud,param));
        if(g2) {
            return ConvertFrom(g2.get(),param);
        }
        return false;
    }
}

bool Geometry3DConvexHull::Union(const vector<Geometry3D*>& geoms)
{
  for(auto g:geoms) {
    if(g->GetType() != Type::ConvexHull) return false;
  }
  vector<Vector3> points;
  for(auto g:geoms) {
    Geometry3DConvexHull* ch = dynamic_cast<Geometry3DConvexHull*>(g);
    auto gpts = ch->data.GetPoints();
    points.insert(points.end(),gpts.begin(),gpts.end());
  }
  ConvexHull3D ch;
  ch.SetPoints(points);
  data = ch;
  return true;
}

bool Geometry3DConvexHull::Merge(const Geometry3D* geom,const RigidTransform* Tgeom)
{
  if(geom->GetType() != Type::ConvexHull) return false;
  if(Tgeom) {
    auto temp = geom->Copy();
    temp->Transform(*Tgeom);
    bool res = Merge(temp);
    delete temp;
    return res;
  }
  const Geometry3DConvexHull* ch = dynamic_cast<const Geometry3DConvexHull*>(geom);
  vector<Vector3> points = data.GetPoints();
  vector<Vector3> other = ch->data.GetPoints();
  points.insert(points.end(),other.begin(),other.end());
  data.SetPoints(points);
  return true;
}



Collider3DConvexHull::ObjectHandleContainer::ObjectHandleContainer(DT_ObjectHandle _data)
:data(_data)
{
  //printf("CREATING OBJECT HANDLE CONTAINER\n");
}

Collider3DConvexHull::ObjectHandleContainer::~ObjectHandleContainer()
{
  if(data) {
    //printf("DESTROYING OBJECT HANDLE IN CONTAINER\n");
    DT_DestroyObject(data);
  }
}

Collider3DConvexHull::Collider3DConvexHull(shared_ptr<Geometry3DConvexHull> _data)
:data(_data)
{
  Reset();
}

Collider3DConvexHull::Collider3DConvexHull(const ConvexHull3D& hull)
{
  data.reset(new Geometry3DConvexHull(hull));
  Reset();
}

AABB3D Collider3DConvexHull::GetAABB() const
{
    DT_Vector3 bmin, bmax;
    DT_GetBBox(objectHandle->data, bmin, bmax);
    AABB3D res;
    for(int i = 0; i < 3; i++){
      res.bmin[i] = bmin[i];
      res.bmax[i] = bmax[i];
    }
    return res;
}

void Collider3DConvexHull::Reset()
{
  type = data->data.type;
  auto shapeHandle = data->data.shapeHandle->data;
  Assert(shapeHandle != NULL);
  objectHandle = make_shared<ObjectHandleContainer>(DT_CreateObject(nullptr, shapeHandle));
  RigidTransform ident;
  ident.setIdentity();
  SetTransform(ident);
}

bool Collider3DConvexHull::Contains(const Vector3& pnt,bool& result)
{
  ConvexHull3D cpt;
  cpt.SetPoint(pnt);
  Collider3DConvexHull ccpt(cpt);
  result = Collides(ccpt);
  return true;
}

bool Collider3DConvexHull::Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{
  switch(geom->GetType()) {
  case Type::Primitive:
    {
    auto* b=dynamic_cast<Collider3DPrimitive*>(geom);
    bool collides;
    if(!Collides(b->data->data,collides)) return false;
    if(collides) {
      elements1.push_back(0);
      elements2.push_back(0);
    }
    return true;
    }
  case Type::ConvexHull:
    {
    auto* b=dynamic_cast<Collider3DConvexHull*>(geom);
    if(Collides(*b)) {
      elements1.push_back(0);
      elements2.push_back(0);
    }
    return true;
    }
  default:
    return false;
  }
}


bool Collider3DConvexHull::WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{
  switch(geom->GetType()) {
  case Type::Primitive:
    {
    auto* b=dynamic_cast<Collider3DPrimitive*>(geom);
    Real dist;
    Vector3 cp,dir;
    if(!ClosestPoint(b->data->data,dist,cp,dir)) return false;
    if(dist <= d) {
      elements1.push_back(0);
      elements2.push_back(0);
    }
    return true;
    }
  case Type::ConvexHull:
    {
    auto* b=dynamic_cast<Collider3DConvexHull*>(geom);
    Real dist;
    Vector3 cp,dir;
    dist = ClosestPoint(*b,cp,dir);
    if(dist <= d) {
      elements1.push_back(0);
      elements2.push_back(0);
    }
    return true;
    }
  default:
    return false;
  }
}

bool Collider3DConvexHull::Collides(const GeometricPrimitive3D& primitive,bool& result)
{
  if(primitive.type == GeometricPrimitive3D::Point) {
    if(!Contains(*AnyCast<Vector3>(&primitive.data),result)) return false;
    return true;
  }
  else if(primitive.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D* s = AnyCast<Sphere3D>(&primitive.data);
    Real dist;
    if(!Distance(s->center,dist)) return false;
    result = (dist <= s->radius);
    return true;
  }
  //TODO: can also do boxes and AABBs
  return false;
}

bool Collider3DConvexHull::Collides(const Collider3DConvexHull& geometry, Vector3* common_point) const
{
  DT_SetAccuracy((DT_Scalar)1e-6);
  DT_SetTolerance((DT_Scalar)(1e-6));
  DT_Vector3 point1;
  DT_Bool collide = DT_GetCommonPoint(objectHandle->data, geometry.objectHandle->data, point1);
  if(collide && common_point)
    common_point->set(point1);
  return collide;
}

bool Collider3DConvexHull::Distance(const Vector3 &pnt,Real& distance)
{
  ConvexHull3D cpt;
  cpt.SetPoint(pnt);
  Collider3DConvexHull ccpt(cpt);
  distance = std::get<0>(dist_func(objectHandle->data, ccpt.objectHandle->data));
  return true;
}

bool Collider3DConvexHull::Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res)
{
    res.hasClosestPoints = true;
    res.hasElements = true;
    res.elem2 = 0;
    res.cp2 = pt;

    res.elem1 = 0;
    res.hasDirections = true;
    res.d = ClosestPoint(pt, res.cp1, res.dir1);
    res.dir2.setNegative(res.dir1);
    Transform1(res, GetTransform());
    return true;
}

bool Collider3DConvexHull::Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& result)
{
  switch(geom->GetType()) {
  case Type::Primitive:
  {
    auto* prim = dynamic_cast<Collider3DPrimitive*>(geom);
    auto wp = prim->data->data;
    wp.Transform(prim->T);
    Vector3 cp,dir;
    if(!ClosestPoint(wp,result.d,cp,dir)) return false;
    result.hasPenetration = true;
    result.hasClosestPoints = true;
    result.hasDirections = true;
    result.cp1 = cp;
    result.cp2 = cp+dir*result.d;
    result.dir1 = dir;
    result.dir2 = -dir;
    return false;
  }
  case Type::ConvexHull:
  {
      // use AABB to exclude them
      AABB3D aabb1 = this->GetAABB();
      AABB3D aabb2 = geom->GetAABB();
      // std::cout << "BBox1 " << aabb1.bmin.x << " " << aabb1.bmin.y << " " << aabb1.bmin.z << " and " << aabb1.bmax.x << " " << aabb1.bmax.y << " " << aabb1.bmax.z << "\n";
      // std::cout << "BBox2 " << aabb2.bmin.x << " " << aabb2.bmin.y << " " << aabb2.bmin.z << " and " << aabb2.bmax.x << " " << aabb2.bmax.y << " " << aabb2.bmax.z << "\n";
      double d = aabb1.distance(aabb2);
      // std::cout << "d = " << d << std::endl;
      if(d > settings.upperBound) {
        result.d = d;
        result.hasElements = false;
        result.hasClosestPoints = false;
        result.hasPenetration = false;
        result.hasDirections = false;
        return true;
      }
      Collider3DConvexHull* b = dynamic_cast<Collider3DConvexHull*>(geom);
      Vector3 cp, direction;
      Real dist = ClosestPoint(*b, cp, direction);
      //std::cout << "Call closest point and get " << dist << "\n";
      result.hasElements = false;
      result.hasPenetration = true;
      result.hasClosestPoints = true;
      result.hasDirections = true;
      result.d = dist;
      result.dir1 = direction;
      result.dir2 = -direction;
      result.cp1 = cp;
      result.cp2 = cp + dist * direction;
      return true;
  }
  default:
    return false;
  }
}


Real Collider3DConvexHull::ClosestPoint(const Vector3& pnt,Vector3& cp,Vector3& direction) const
{
  ConvexHull3D cpt;
  cpt.SetPoint(pnt);
  Collider3DConvexHull ccpt(cpt);
  Real dist;
  std::tie(dist, cp, direction) = dist_func(objectHandle->data, ccpt.objectHandle->data);
  // change direction based on distance
  if(dist >= 0) {
    direction.setNormalized(direction - cp);
  }
  else{
    direction.setNormalized(cp - direction);
  }
  return dist;
}

bool Collider3DConvexHull::ClosestPoint(const GeometricPrimitive3D& primitive,Real& dist,Vector3& cp,Vector3& direction) const
{
  if(primitive.type == GeometricPrimitive3D::Point) {
    auto* pt = AnyCast<Vector3>(&primitive.data);
    dist = ClosestPoint(*pt,cp,direction);
    return true;
  }
  else if(primitive.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D* s = AnyCast<Sphere3D>(&primitive.data);
    dist = ClosestPoint(s->center,cp,direction);
    dist -= s->radius;
    return true;
  }
  //TODO: can also do boxes and AABBs
  return false;
}

// compute the closest points between two convexhull3d with collision data and store results somewhere
Real Collider3DConvexHull::ClosestPoint(const Collider3DConvexHull& g, Vector3& cp, Vector3& direction) const
{
  double dist;
  std::tie(dist, cp, direction) = dist_func(this->objectHandle->data, g.objectHandle->data);
  // change direction based on distance
  if(dist >= 0) {
    direction.setNormalized(direction - cp);
  }
  else{
    direction.setNormalized(cp - direction);
  }
  return dist;
}

void Collider3DConvexHull::SetTransform(const RigidTransform &tran)
{
  T = tran;
  double transform[16];
  transform[0] = tran.R.data[0][0]; transform[1] = tran.R.data[0][1]; transform[2] = tran.R.data[0][2]; transform[3] = 0;
  transform[4] = tran.R.data[1][0]; transform[5] = tran.R.data[1][1]; transform[6] = tran.R.data[1][2]; transform[7] = 0;
  transform[8] = tran.R.data[2][0]; transform[9] = tran.R.data[2][1]; transform[10] = tran.R.data[2][2]; transform[11] = 0;
  transform[12] = tran.t[0]; transform[13] = tran.t[1]; transform[14] = tran.t[2]; transform[15] = 1;
  DT_SetMatrixd(objectHandle->data, transform);
}

void Collider3DConvexHull::UpdateHullSecondRelativeTransform(const RigidTransform &tran) {
  double transform[16];  // this overwrites the other one
  transform[0] = tran.R.data[0][0]; transform[1] = tran.R.data[0][1]; transform[2] = tran.R.data[0][2]; transform[3] = 0;
  transform[4] = tran.R.data[1][0]; transform[5] = tran.R.data[1][1]; transform[6] = tran.R.data[1][2]; transform[7] = 0;
  transform[8] = tran.R.data[2][0]; transform[9] = tran.R.data[2][1]; transform[10] = tran.R.data[2][2]; transform[11] = 0;
  transform[12] = tran.t[0]; transform[13] = tran.t[1]; transform[14] = tran.t[2]; transform[15] = 1;
  //std::cout << "Hull3D update transform\n";
  if(this->type == ConvexHull3D::Hull)
    DT_SetChildRelativeMatrixd(objectHandle->data, 1, transform);
  else
    FatalError("Invalid call to UpdateHullSecondRelativeTransform, not a hull object");
}

bool Collider3DConvexHull::Support(const Vector3& dir,Vector3& pt)
{
  double out[3];
  DT_GetSupport(objectHandle->data, dir, out);
  pt = Vector3(out);
  return true;
}


bool Collider3DConvexHull::RayCast(const Ray3D& r,Real margin,Real& distance,int& element)
{
  if(type != ConvexHull3D::Type::Box && type != ConvexHull3D::Type::Sphere) {
    //SOLID has not implemented these yet
    return false;
  }
  element = -1;
  Vector3 pt;
  if(!Support(r.direction,pt)) return true;
  Vector3 target = r.source + r.direction;
  DT_Vector3 normal;
  if(DT_ObjectRayCast(objectHandle->data, r.source, target, 10000,&distance,normal))
  {
    element = 0;
    distance -= margin;
  }
  else if(margin > 0) { //need to find closest point between ray and object
    Real fwd = r.direction.dot(pt-r.source);
    if(fwd < margin) return true;
    ConvexHull3D ray_ch;
    Segment3D long_segment;
    long_segment.a = r.source;
    long_segment.b = r.source + r.direction*(2*(fwd+margin));
    ray_ch.SetLineSegment(long_segment);
    Collider3DConvexHull ray_cch(ray_ch);
    Vector3 dir;
    ray_cch.ClosestPoint(*this,pt,dir);
    distance = r.direction.dot(pt-r.source);
    //TODO: better measurement of collision point with margin
    distance -= margin;
    element = 0;
  }
  return true;
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
