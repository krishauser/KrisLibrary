#include "AnyGeometryTypeImpl.h"
#include <KrisLibrary/Logger.h>
#include "Conversions.h"
#include "ROI.h"
#include "Slice.h"
#include <math3d/geometry3d.h>
#include <meshing/VolumeGrid.h>
#include <meshing/Voxelize.h>
#include "CollisionPrimitive.h"
#include "CollisionConvexHull.h"
#include "CollisionMesh.h"
#include "CollisionPointCloud.h"
#include "CollisionImplicitSurface.h"
#include "CollisionOccupancyGrid.h"
#include <fstream>

DECLARE_LOGGER(Geometry)


namespace Geometry {

void Flip(DistanceQueryResult &res)
{
  std::swap(res.elem1, res.elem2);
  std::swap(res.group_elem1, res.group_elem2);
  std::swap(res.cp1, res.cp2);
  std::swap(res.dir1, res.dir2);
}

void Transform1(DistanceQueryResult &res, const RigidTransform &T)
{
  if (res.hasClosestPoints)
    res.cp1 = T * res.cp1;
  if (res.hasDirections)
    res.dir1 = T.R * res.dir1;
}

void Transform2(DistanceQueryResult &res, const RigidTransform &T)
{
  if (res.hasClosestPoints)
    res.cp2 = T * res.cp2;
  if (res.hasDirections)
    res.dir2 = T.R * res.dir2;
}

void Offset1(DistanceQueryResult &res, Real offset)
{
  res.d -= offset;
  if (res.hasDirections)
    res.cp1.madd(res.dir1, offset);
}

void Offset2(DistanceQueryResult &res, Real offset)
{
  res.d -= offset;
  if (res.hasDirections)
    res.cp2.madd(res.dir2, offset);
}

void Offset(DistanceQueryResult &res, Real offset1, Real offset2)
{
  Offset1(res,offset1);
  Offset2(res,offset2);
}

void SetCP2(DistanceQueryResult &res)
{
  res.cp2 = res.cp1;
  res.cp2.madd(res.dir1, res.d);
  res.dir2.setNegative(res.dir1);
}

void PushGroup1(DistanceQueryResult &res, int i)
{
  if (res.group_elem1.empty())
  {
    res.group_elem1.resize(2);
    res.group_elem1[0] = i;
    res.group_elem1[1] = res.elem1;
  }
  else
  {
    res.group_elem1.insert(res.group_elem1.begin(), i);
  }
  res.elem1 = i;
}

void PushGroup2(DistanceQueryResult &res, int i)
{
  if (res.group_elem2.empty())
  {
    res.group_elem2.resize(2);
    res.group_elem2[0] = i;
    res.group_elem2[1] = res.elem2;
  }
  else
  {
    res.group_elem2.insert(res.group_elem2.begin(), i);
  }
  res.elem2 = i;
}

void ReverseContact(ContactsQueryResult::ContactPair &contact)
{
  std::swap(contact.p1, contact.p2);
  contact.n.inplaceNegative();
  std::swap(contact.elem1, contact.elem2);
}

} //Geometry

using namespace Geometry;

DistanceQueryResult::DistanceQueryResult()
    : hasPenetration(0), hasElements(0), hasClosestPoints(0), hasDirections(0), d(Inf), elem1(-1), elem2(-1)
{
}

DistanceQuerySettings::DistanceQuerySettings()
    : relErr(0), absErr(0), upperBound(Inf)
{
}


ContactsQuerySettings::ContactsQuerySettings()
    : padding1(0), padding2(0), maxcontacts(std::numeric_limits<size_t>::max()), cluster(false)
{
}

ContactsQueryResult::ContactsQueryResult()
    : clustered(false)
{
}



Geometry3D* Geometry3D::Make(Type type)
{
    switch(type) {
    case Type::Primitive: return new Geometry3DPrimitive();
    case Type::TriangleMesh: return new Geometry3DTriangleMesh();
    case Type::PointCloud: return new Geometry3DPointCloud();
    case Type::ImplicitSurface: return new Geometry3DImplicitSurface();
    case Type::OccupancyGrid: return new Geometry3DOccupancyGrid();
    case Type::ConvexHull: return new Geometry3DConvexHull();
    case Type::Group: return new Geometry3DGroup();
    default:
        return NULL;
    }
}

Geometry3D* Geometry3D::Make(const char* typestr)
{
  if (0==strcmp(typestr,"Primitive")) return new Geometry3DPrimitive();
  else if(0==strcmp(typestr,"TriangleMesh")) return new Geometry3DTriangleMesh();
  else if(0==strcmp(typestr,"PointCloud")) return new Geometry3DPointCloud();
  else if(0==strcmp(typestr,"ImplicitSurface")) return new Geometry3DImplicitSurface();
  else if(0==strcmp(typestr,"OccupancyGrid")) return new Geometry3DOccupancyGrid();
  else if(0==strcmp(typestr,"ConvexHull")) return new Geometry3DConvexHull();
  else if(0==strcmp(typestr,"Group")) return new Geometry3DGroup();
  else return NULL;
}

const char *Geometry3D::TypeName(Type type)
{
  switch (type)
  {
  case Type::Primitive:
    return "Primitive";
  case Type::ConvexHull:
    return "ConvexHull";
  case Type::TriangleMesh:
    return "TriangleMesh";
  case Type::PointCloud:
    return "PointCloud";
  case Type::ImplicitSurface:
    return "ImplicitSurface";
  case Type::OccupancyGrid:
    return "OccupancyGrid";
  case Type::Group:
    return "Group";
  default:
    return "Error";
  }
}



vector<string> Geometry3D::FileExtensions() const
{
    vector<string> res;
    const char* ext = FileExtension();
    if(ext) res.push_back(ext);
    return res;
}

bool Geometry3D::Load(const char* fn)
{
    ifstream in(fn,ios::in);
    if(!in) return false;
    return Load(in);
}

bool Geometry3D::Save(const char* fn) const
{
    ofstream out(fn,ios::out);
    if(!out) return false;
    return Save(out);
}

bool Geometry3D::Transform(const RigidTransform& T)
{
    return Transform(Matrix4(T));
}

Geometry3DGroup::Geometry3DGroup()
{}

Geometry3DGroup::Geometry3DGroup(const vector<AnyGeometry3D>& _data)
:data(_data)
{}

Geometry3DGroup::~Geometry3DGroup ()
{}

bool Geometry3DGroup::Empty() const { return data.empty(); }

size_t Geometry3DGroup::NumElements() const { return data.size(); }

Geometry3D* Geometry3DGroup::ConvertTo(Type restype, Real param,Real domainExpansion) const
{
    vector<AnyGeometry3D> convert(data.size());
    for (size_t i = 0; i < data.size(); i++) {
        auto* res = data[i].data->ConvertTo(restype, param, domainExpansion);
        if(!res) 
            return NULL;
        convert[i].type = res->GetType();
        convert[i].data.reset(res);
    }
    auto* res = Geometry3D::Make(restype);
    vector<Geometry3D*> impls(convert.size());
    for(size_t i=0;i<convert.size();i++) 
        impls[i] = convert[i].data.get();
    if(!res->Merge(impls)) {
        delete res;
        return NULL;
    }
    return res;
}

Geometry3D* Geometry3DGroup::Remesh(Real resolution,bool refine,bool coarsen) const
{
    if(resolution <= 0) return NULL;
    auto* res = new Geometry3DGroup;
    res->data.resize(data.size());
    for (size_t i = 0; i < data.size(); i++) {
        if(!data[i].Remesh(resolution, res->data[i], refine, coarsen)) {
            delete res;
            return NULL;
        }
    }
    return res;
}

Geometry3D* Geometry3DGroup::Slice(const RigidTransform& T,Real tol) const
{
    auto* res = new Geometry3DGroup();
    res->data.resize(data.size());
    for (size_t i = 0; i < data.size(); i++) {
        if(!data[i].Slice(T, res->data[i], tol)) {
            delete res;
            return NULL;
        }
    }
    return res;
}

Geometry3D* Geometry3DGroup::ExtractROI(const AABB3D& bb,int flags) const
{
    Geometry3DGroup* res = new Geometry3DGroup;
    res->data.resize(data.size());
    for (size_t i = 0; i < data.size(); i++) {
        if(!data[i].ExtractROI(bb, res->data[i], flags)) {
            delete res;
            return NULL;
        }
    }
    return res;
}

Geometry3D* Geometry3DGroup::ExtractROI(const Box3D& bb,int flags) const
{
    Geometry3DGroup* res = new Geometry3DGroup;
    res->data.resize(data.size());
    for (size_t i = 0; i < data.size(); i++) {
        if(!data[i].ExtractROI(bb, res->data[i], flags)) {
            delete res;
            return NULL;
        }
    }
    return res;
}

bool Geometry3DGroup::Load(istream& in)
{
    int n;
    in >> n;
    if (!in || n < 0)
      return false;
    data.resize(n);
    string typestr;
    for (size_t i = 0; i < data.size(); i++) {
        if (!data[i].Load(in))
            return false;
    }
    return true;
}


bool Geometry3DGroup::Save(ostream& out) const
{
    out << data.size() << endl;
    for (size_t i = 0; i < data.size(); i++)
        if (!data[i].Save(out))
            return false;
    return true;
}

bool Geometry3DGroup::Transform(const Matrix4 &T)
{
    for (size_t i = 0; i < data.size(); i++)
        if(!data[i].Transform(T)) return false;
    return true;
}

AABB3D Geometry3DGroup::GetAABB() const
{
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < data.size(); i++)
    {
        AABB3D itembb = data[i].GetAABB();
        bb.setUnion(itembb);
    }
    return bb;
}


Collider3D* Collider3D::Make(shared_ptr<Geometry3D> data)
{
    switch (data->GetType()) {
    case Type::Primitive: return new Collider3DPrimitive(dynamic_pointer_cast<Geometry3DPrimitive>(data));
    case Type::TriangleMesh: return new Collider3DTriangleMesh(dynamic_pointer_cast<Geometry3DTriangleMesh>(data));
    case Type::PointCloud: return new Collider3DPointCloud(dynamic_pointer_cast<Geometry3DPointCloud>(data));
    case Type::ConvexHull: return new Collider3DConvexHull(dynamic_pointer_cast<Geometry3DConvexHull>(data));
    case Type::ImplicitSurface: return new Collider3DImplicitSurface(dynamic_pointer_cast<Geometry3DImplicitSurface>(data));
    case Type::OccupancyGrid: return new Collider3DOccupancyGrid(dynamic_pointer_cast<Geometry3DOccupancyGrid>(data));
    case Type::Group: return new Collider3DGroup(dynamic_pointer_cast<Geometry3DGroup>(data));
    }
    return NULL;
}

bool Collider3D::Collides(Collider3D* geom,bool& result)
{
    vector<int> elements1,elements2;
    if(!Collides(geom,elements1,elements2,1)) return false;
    result = !elements1.empty();
    return true;
}

bool Collider3D::Distance(const Vector3& pt,Real& result)
{
    DistanceQuerySettings settings;
    DistanceQueryResult res;
    if(!Distance(pt,settings,res)) return false;
    result = res.d;
    return true;
}

bool Collider3D::Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res)
{
    shared_ptr<Geometry3D> geom(new Geometry3DPrimitive(GeometricPrimitive3D(pt)));
    auto* cgeom = Collider3D::Make(geom);
    if(!Distance(cgeom,settings,res)) {
        delete cgeom;
        return false;
    }
    delete cgeom;
    return true;
}

bool Collider3D::WithinDistance(Collider3D* geom,Real d,bool& result)
{
    vector<int> elements1,elements2;
    if(!WithinDistance(geom,d,elements1,elements2,1)) return false;
    result = !elements1.empty();
    return true;
}

Collider3D* Collider3D::Slice(const RigidTransform& T,Real tol) const
{
    Geometry3D* sgeom = GetData()->Slice(T,tol);
    if(!sgeom) return NULL;
    shared_ptr<Geometry3D> sgeom_ptr(sgeom);
    return Collider3D::Make(sgeom_ptr);
}

Collider3D* Collider3D::ExtractROI(const AABB3D& bb,int flag) const
{
    Geometry3D* sgeom = GetData()->ExtractROI(bb,flag);
    if(!sgeom) return NULL;
    shared_ptr<Geometry3D> sgeom_ptr(sgeom);
    return Collider3D::Make(sgeom_ptr);
}

Collider3D* Collider3D::ExtractROI(const Box3D& bb,int flag) const
{
    Geometry3D* sgeom = GetData()->ExtractROI(bb,flag);
    if(!sgeom) return NULL;
    shared_ptr<Geometry3D> sgeom_ptr(sgeom);
    return Collider3D::Make(sgeom_ptr);
}

Collider3D* Collider3D::Copy() const
{
    return Make(shared_ptr<Geometry3D>(GetData()->Copy()));
}

bool Collider3D::Merge(const vector<Collider3D*>& cgeoms)
{
    if(cgeoms.empty()) return false;
    RigidTransform T0 = GetTransform();
    vector<Geometry3D*> geoms(geoms.size());
    for (size_t i = 0; i < cgeoms.size(); i++) {
        RigidTransform Ti = cgeoms[i]->GetTransform();
        RigidTransform Trel; Trel.mulInverseA(T0,Ti);
        geoms[i] = cgeoms[i]->GetData().get();
        geoms[i] = geoms[i]->ConvertTo(geoms[i]->GetType());
        geoms[i]->Transform(Trel);
    }
    if(!GetData()->Merge(geoms)) {
        return false;
    }
    Reset();
    return true;
}

Collider3D* Collider3D::ConvertTo(Type restype,Real param,Real domainExpansion)
{
    auto* converted = GetData()->ConvertTo(restype,param,domainExpansion);
    if(!converted) return NULL;
    auto* res = Collider3D::Make(shared_ptr<Geometry3D>(converted));
    res->SetTransform(GetTransform());
    return res;
}

bool Collider3D::ConvertFrom(Collider3D* geom,Real param,Real domainExpansion)
{
    if(!GetData()->ConvertFrom(geom->GetData().get(),param,domainExpansion)) return false;
    SetTransform(geom->GetTransform());
    return true;
}

Collider3DGroup::Collider3DGroup(shared_ptr<Geometry3DGroup> _data)
:data(_data),collisionData(_data->data.size())
{
  for(size_t i=0;i<data->data.size();i++) {
    collisionData[i].type = data->data[i].type;
    collisionData[i].data = data->data[i].data;
  }
}

Collider3DGroup::~Collider3DGroup() {}

AABB3D Collider3DGroup::GetAABBTight() const
{
    const vector<AnyCollisionGeometry3D> &items = collisionData;
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < items.size(); i++)
      bb.setUnion(items[i].GetAABBTight());
    return bb;
}

AABB3D Collider3D::GetAABB() const
{
    Box3D b = GetBB();
    AABB3D bb;
    b.getAABB(bb);
    return bb;
}

Box3D Collider3D::GetBB() const
{
  Box3D b;
  AABB3D bblocal = GetData()->GetAABB();
  b.setTransformed(bblocal, GetTransform());
  return b;
}

AABB3D Collider3DGroup::GetAABB() const
{
    const vector<AnyCollisionGeometry3D> &items = collisionData;
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < items.size(); i++)
      bb.setUnion(items[i].GetAABB());
    return bb;
}

Box3D Collider3DGroup::GetBB() const
{
    Box3D b;
    b.set(GetAABB());
    return b;
}

void Collider3DGroup::SetTransform(const RigidTransform& T)
{
    vector<AnyCollisionGeometry3D> &items = collisionData;
    for (size_t i = 0; i < items.size(); i++)
        items[i].SetTransform(T);
}


bool Collider3DGroup::Distance(const Vector3& pt,Real& result)
{
    vector<AnyCollisionGeometry3D> &items = collisionData;
    result = Inf;
    for (size_t i = 0; i < items.size(); i++) {
        Real d;
        if(!items[i].collider->Distance(pt,d)) return false;
        result = Min(result,d);
    }
    return true;
}

bool Collider3DGroup::Distance(const Vector3 &pt, const DistanceQuerySettings &settings,DistanceQueryResult& res)
{
    vector<AnyCollisionGeometry3D> &items = collisionData;
    DistanceQuerySettings modsettings = settings;
    for (size_t i = 0; i < items.size(); i++)
    {
      DistanceQueryResult ires = items[i].Distance(pt, modsettings);
      if (ires.d < res.d)
      {
        res = ires;
        PushGroup1(res, (int)i);
        modsettings.upperBound = res.d;
      }
    }
    return true;
}

bool Collider3DGroup::WithinDistance(Collider3D* geom,Real d,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  for (size_t i = 0; i < collisionData.size(); i++)
  {
    vector<int> ei1, ei2;
    if (collisionData[i].collider->WithinDistance(geom, d, ei1, ei2, maxContacts - (int)elements1.size()))
    {
      for (size_t j = 0; j < ei1.size(); j++)
      {
        elements1.push_back((int)i);
        elements2.push_back((int)ei2[j]);
      }
      if (elements2.size() >= maxContacts)
        return true;
    }
    else if (geom->WithinDistance(collisionData[i].collider.get(), d, ei2, ei1, maxContacts - (int)elements1.size())) {
      for (size_t j = 0; j < ei1.size(); j++)
      {
        elements1.push_back((int)i);
        elements2.push_back((int)ei2[j]);
      }
      if (elements2.size() >= maxContacts)
        return true;
    }
    else return false;
  }
  return true;
}

bool Collider3DGroup::RayCast(const Ray3D& r,Real margin,Real& distance,int& element) 
{
  vector<AnyCollisionGeometry3D> &items = collisionData;
  distance = Inf;
  element = -1;
  for (size_t i = 0; i < items.size(); i++)
  {
    Real d;
    int elem;
    if (items[i].RayCast(r, &d, &elem))
    {
      if (d < distance)
      {
        distance = d;
        element = (int)i;
      }
    }
  }
  return true;
}

bool Collider3DGroup::Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) 
{
  vector<Geometry::AnyCollisionGeometry3D> &items = collisionData;
  int n = 0;
  for (size_t i = 0; i < items.size(); i++)
  {
    ContactsQuerySettings modsettings = settings;
    modsettings.maxcontacts -= n;
    modsettings.padding1 += items[i].margin;
    ContactsQueryResult modresult;
    if(items[i].collider->Contacts(other,modsettings,modresult)) {
      for (auto &c : modresult.contacts)
        c.elem1 = (int)i;
      res.contacts.insert(res.contacts.end(), modresult.contacts.begin(), modresult.contacts.end());
    }
    else {
      swap(modsettings.padding1,modsettings.padding2);
      if(other->Contacts(items[i].collider.get(),modsettings,modresult)) {
        for (auto &c : modresult.contacts) {
          c.elem2 = (int)i;
          ReverseContact(c);
        }
        res.contacts.insert(res.contacts.end(), modresult.contacts.begin(), modresult.contacts.end());
      }
      else return false;
    }
    n += modresult.contacts.size();
    if (n >= (int)settings.maxcontacts)
      return true;
  }
  return true;
}

Collider3D* Collider3DGroup::Slice(const RigidTransform& T,Real tol) const
{
  const vector<AnyCollisionGeometry3D> &items = collisionData;
  auto* res = new Collider3DGroup(shared_ptr<Geometry3DGroup>(new Geometry3DGroup()));
  res->data->data.resize(items.size());
  res->collisionData.resize(items.size());
  for (size_t i = 0; i < items.size(); i++) {
    if(!items[i].Slice(T, res->collisionData[i], tol)) {
      delete res;
      return NULL;
    }
    res->data->data[i] = res->collisionData[i];
  }
  return res;
}

Collider3D* Collider3DGroup::ExtractROI(const AABB3D& bb,int flags) const
{
  const vector<AnyCollisionGeometry3D> &items = collisionData;
  auto* res = new Collider3DGroup(make_shared<Geometry3DGroup>());
  for (size_t i = 0; i < items.size(); i++) {
    if(!items[i].ExtractROI(bb,res->collisionData[i],flags)) return NULL;
    res->data->data[i] = res->collisionData[i];
  }
  return res;
}

Collider3D* Collider3DGroup::ExtractROI(const Box3D& bb,int flags) const
{
  const vector<AnyCollisionGeometry3D> &items = collisionData;
  auto* res = new Collider3DGroup(make_shared<Geometry3DGroup>());
  for (size_t i = 0; i < items.size(); i++) {
    if(!items[i].ExtractROI(bb,res->collisionData[i],flags)) return NULL;
    res->data->data[i] = res->collisionData[i];
  }
  return res;
}

Collider3D* Collider3DGroup::ConvertTo(Type restype, Real param,Real domainExpansion)
{
    vector<AnyCollisionGeometry3D> &items = collisionData;
    vector<AnyCollisionGeometry3D> resitems(items.size());
    for(size_t i=0;i<items.size();i++) {
      auto* res = items[i].collider->ConvertTo(restype,param,domainExpansion);
      if(!res) return NULL;
      resitems[i].type = res->GetType();
      resitems[i].data = res->GetData();
      resitems[i].collider.reset(res);
    }
    auto* res_geom = Geometry3D::Make(restype);
    auto* res = Collider3D::Make(shared_ptr<Geometry3D>(res_geom));
    vector<Collider3D*> resgeoms(resitems.size());
    for(size_t i=0;i<resitems.size();i++)
        resgeoms[i] = resitems[i].collider.get();
    if(!res->Merge(resgeoms)) {
        delete res;
        return NULL;
    }
    return res;
}
