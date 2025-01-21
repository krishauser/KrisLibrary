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
using namespace Geometry;

namespace Geometry {
    std::tuple<Real, Vector3, Vector3> dist_func(DT_ObjectHandle object1, DT_ObjectHandle object2);  //defined in ConvexHull3D.cpp
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

bool Collider3DConvexHull::Merge(Collider3D* cgeom)
{
  if(cgeom->GetType() != Type::ConvexHull) return false;
  objectHandle.reset(); //need to clear reference to inner geometry before merging
  bool res=Collider3D::Merge(cgeom);
  Assert(res);
  return res;
}

bool Collider3DConvexHull::Contains(const Vector3& pnt,bool& result)
{
  ConvexHull3D cpt;
  cpt.Set(pnt);
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
    GeometricPrimitive3D bw = b->data->data;
    bw.Transform(b->GetTransform());
    if(!Collides(bw,collides)) return false;
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
    GeometricPrimitive3D bw = b->data->data;
    bw.Transform(b->GetTransform());
    if(!ClosestPoint(bw,dist,cp,dir)) return false;
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
  else {
    ConvexHull3D ch_prim;
    if(!ch_prim.Set(primitive)) return false;
    DT_SetAccuracy((DT_Scalar)1e-6);
    DT_SetTolerance((DT_Scalar)(1e-6));
    DT_Vector3 point1;
    result = DT_GetCommonPoint(objectHandle->data, ch_prim.shapeHandle->object_ref, point1);
    return true;
  }
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
  cpt.Set(pnt);
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
    //Transform1(res, GetTransform());
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
    return true;
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
  cpt.Set(pnt);
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
  else {
    ConvexHull3D ch_prim;
    if(!ch_prim.Set(primitive)) return false;
    Collider3DConvexHull cch_prim(ch_prim);
    dist = ClosestPoint(cch_prim,cp,direction);
    return true;
  }
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

bool Collider3DConvexHull::Contacts(Collider3D* geom,const ContactsQuerySettings& settings,ContactsQueryResult& result)
{
  DistanceQuerySettings dsettings;
  dsettings.upperBound = settings.padding1 + settings.padding2;
  DistanceQueryResult dresult;
  if(!Distance(geom,dsettings,dresult)) return false;
  result.contacts.resize(0);
  //single point of contact from the closest points
  if(dresult.d > settings.padding1 + settings.padding2) {
    return true;
  }
  if(!dresult.hasClosestPoints) return false;
  result.contacts.resize(1);
  result.contacts[0].n = dresult.dir1;
  result.contacts[0].p1 = dresult.cp1 + dresult.dir1 * settings.padding1;
  result.contacts[0].p2 = dresult.cp2 - dresult.dir1 * settings.padding2;
  result.contacts[0].depth = dresult.d - settings.padding1 - settings.padding2;
  result.contacts[0].elem1 = result.contacts[0].elem2 = 0;
  result.contacts[0].unreliable = false;
  return true;
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
    ray_ch.Set(long_segment);
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
