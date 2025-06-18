#include "AnyGeometry.h"
#include "GeometryTypeImpl.h"
#include "CollisionPrimitive.h"
#include "CollisionConvexHull.h"
#include "CollisionMesh.h"
#include "CollisionPointCloud.h"
#include "CollisionImplicitSurface.h"
#include "CollisionOccupancyGrid.h"
#include "CollisionHeightmap.h"
#include <KrisLibrary/meshing/IO.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/structs/Heap.h>
#include <KrisLibrary/Logger.h>
#include <Timer.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>


DEFINE_LOGGER(Geometry)


#define POINT_CLOUD_MESH_COLLIDE_ALL_POINTS 0

using namespace Geometry;


AnyGeometry3D::AnyGeometry3D()
    : type(Type::Primitive), data(new Geometry3DPrimitive())
{
}

AnyGeometry3D::AnyGeometry3D(const GeometricPrimitive3D &primitive)
    : type(Type::Primitive), data(new Geometry3DPrimitive(primitive))
{
}

AnyGeometry3D::AnyGeometry3D(const ConvexHull3D &cvxhull)
    : type(Type::ConvexHull), data(new Geometry3DConvexHull(cvxhull))
{
}

AnyGeometry3D::AnyGeometry3D(const Meshing::TriMesh &mesh)
    : type(Type::TriangleMesh), data(new Geometry3DTriangleMesh(mesh))
{
}

AnyGeometry3D::AnyGeometry3D(const Meshing::PointCloud3D &pc)
    : type(Type::PointCloud), data(new Geometry3DPointCloud(pc))
{
}

AnyGeometry3D::AnyGeometry3D(const Meshing::VolumeGrid &grid, int value_type)
    : type(Type::ImplicitSurface)
{
  if(value_type == VolumeGridOccupancyGrid || value_type == VolumeGridDensity) {
    type = Type::OccupancyGrid;
    data.reset(new Geometry3DOccupancyGrid(grid));
  }
  else {
    data.reset(new Geometry3DImplicitSurface(grid));
  }
}

AnyGeometry3D::AnyGeometry3D(const Meshing::Heightmap& hm)
    : type(Type::Heightmap)
{
  data.reset(new Geometry3DHeightmap(hm));
}

AnyGeometry3D::AnyGeometry3D(const vector<AnyGeometry3D> &group)
    : type(Type::Group)
{
  auto* ptr = new Geometry3DGroup();
  ptr->data = group;
  data.reset(ptr);
}

AnyGeometry3D& AnyGeometry3D::operator = (const AnyGeometry3D& rhs)
{
  type = rhs.type;
  data.reset(rhs.data->Copy());
  return *this;
}

AnyGeometry3D::AnyGeometry3D(const AnyGeometry3D& geom)
{
  *this = geom;
}

const GeometricPrimitive3D &AnyGeometry3D::AsPrimitive() const { return dynamic_cast<Geometry3DPrimitive*>(data.get())->data; }
const ConvexHull3D& AnyGeometry3D::AsConvexHull() const { return dynamic_cast<Geometry3DConvexHull*>(data.get())->data; };
const Meshing::TriMesh &AnyGeometry3D::AsTriangleMesh() const { return dynamic_cast<Geometry3DTriangleMesh*>(data.get())->data; };
const Meshing::PointCloud3D &AnyGeometry3D::AsPointCloud() const { return dynamic_cast<Geometry3DPointCloud*>(data.get())->data; };
const Meshing::VolumeGrid &AnyGeometry3D::AsImplicitSurface() const { return dynamic_cast<Geometry3DImplicitSurface*>(data.get())->data; };
const Meshing::VolumeGrid &AnyGeometry3D::AsOccupancyGrid() const { return dynamic_cast<Geometry3DOccupancyGrid*>(data.get())->data; };
const Meshing::Heightmap &AnyGeometry3D::AsHeightmap() const { return dynamic_cast<Geometry3DHeightmap*>(data.get())->data; };
const vector<AnyGeometry3D> &AnyGeometry3D::AsGroup() const { return dynamic_cast<Geometry3DGroup*>(data.get())->data; };
GeometricPrimitive3D &AnyGeometry3D::AsPrimitive() { return dynamic_cast<Geometry3DPrimitive*>(data.get())->data; }
ConvexHull3D& AnyGeometry3D::AsConvexHull() { return dynamic_cast<Geometry3DConvexHull*>(data.get())->data; };
Meshing::TriMesh &AnyGeometry3D::AsTriangleMesh() { return dynamic_cast<Geometry3DTriangleMesh*>(data.get())->data; };
Meshing::PointCloud3D &AnyGeometry3D::AsPointCloud() { return dynamic_cast<Geometry3DPointCloud*>(data.get())->data; };
Meshing::VolumeGrid &AnyGeometry3D::AsImplicitSurface() { return dynamic_cast<Geometry3DImplicitSurface*>(data.get())->data; };
Meshing::VolumeGrid &AnyGeometry3D::AsOccupancyGrid() { return dynamic_cast<Geometry3DOccupancyGrid*>(data.get())->data; };
Meshing::Heightmap &AnyGeometry3D::AsHeightmap() { return dynamic_cast<Geometry3DHeightmap*>(data.get())->data; };
vector<AnyGeometry3D> &AnyGeometry3D::AsGroup() { return dynamic_cast<Geometry3DGroup*>(data.get())->data; };


//appearance casts
GLDraw::GeometryAppearance *AnyGeometry3D::TriangleMeshAppearanceData() { return dynamic_cast<Geometry3DTriangleMesh*>(data.get())->appearance.get(); };
const GLDraw::GeometryAppearance *AnyGeometry3D::TriangleMeshAppearanceData() const { return dynamic_cast<Geometry3DTriangleMesh*>(data.get())->appearance.get(); };

bool AnyGeometry3D::Empty() const
{
  if(!data) return true;
  return data->Empty();
}

void AnyGeometry3D::Union(const vector<AnyGeometry3D> &geoms)
{
  vector<shared_ptr<Geometry3D> > nonempty;
  for (size_t i = 0; i < geoms.size(); i++)
    if (!geoms[i].Empty())
      nonempty.push_back(geoms[i].data);
  if (nonempty.empty())
    *this = AnyGeometry3D();
  else if (nonempty.size() == 1)
  {
    data = nonempty[0];
    type = nonempty[0]->GetType();
  }
  else
  {
    //TODO: merge to a different type?
    type = nonempty[0]->GetType();
    auto* geom = Geometry3D::Make(type);
    vector<Geometry3D*> nonempty_ptrs(nonempty.size());
    for(size_t i=0;i<nonempty.size();i++)
      nonempty_ptrs[i] = nonempty[i].get();
    if(geom->Union(nonempty_ptrs)) {
      data.reset(geom);
    }
    else {
      delete geom;
      type = Type::Group;
      auto* group = new Geometry3DGroup();
      group->data = geoms;
      data.reset(group);
    }
  }
}

bool AnyGeometry3D::Merge(const AnyGeometry3D& other, const RigidTransform* Tgeom)
{
  if(!data) {
    data.reset(other.data->Copy());
    if(Tgeom) Transform(*Tgeom);
    return true;
  }
  if(data->Merge(other.data.get(),Tgeom)) return true;
  return false;
}

bool AnyGeometry3D::Convert(Type restype, AnyGeometry3D &res, Real param) const
{
  if(!data) {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"AnyGeometry3D::Convert(): Converting empty geometry");
    return false;
  }
  res.data.reset(data->Convert(restype,param));
  if(res.data) {
    res.type = restype;
    return true;
  }
  return false;
}

bool AnyGeometry3D::Remesh(Real resolution,AnyGeometry3D& res,bool refine,bool coarsen) const
{
  if(!data) return false;
  Geometry3D* newgeom = data->Remesh(resolution,refine,coarsen);
  if(newgeom) {
    res.type = newgeom->GetType();
    res.data.reset(newgeom);
    return true;
  }
  return false;
}

bool AnyGeometry3D::Slice(const RigidTransform& T,AnyGeometry3D& res,Real tol) const
{
  if(!data) return false;
  Geometry3D* newgeom = data->Slice(T,tol);
  if(newgeom) {
    res.type = newgeom->GetType();
    res.data.reset(newgeom);
    return true;
  }
  return false;
}

bool AnyGeometry3D::ExtractROI(const AABB3D& bb,AnyGeometry3D& res,int flags) const
{
  if(!data) return false;
  Geometry3D* newgeom = data->ExtractROI(bb,flags);
  if(newgeom) {
    res.type = newgeom->GetType();
    res.data.reset(newgeom);
    return true;
  }
  return false;
}
  
bool AnyGeometry3D::ExtractROI(const Box3D& bb,AnyGeometry3D& res,int flags) const
{
  if(!data) return false;
  Geometry3D* newgeom = data->ExtractROI(bb,flags);
  if(newgeom) {
    res.type = newgeom->GetType();
    res.data.reset(newgeom);
    return true;
  }
  return false;
}

size_t AnyGeometry3D::NumElements() const
{
  if(!data) return 0;
  return data->NumElements();
}

GeometricPrimitive3D AnyGeometry3D::GetElement(int elem) const
{
  if (elem < 0 || elem >= (int)NumElements())
    FatalError("Invalid element index specified");
  auto res = data->GetElement(elem);
  if(res->GetType() == Type::Primitive) {
    auto prim = dynamic_cast<Geometry3DPrimitive*>(res.get())->data;
    return prim;
  }
  FatalError("Element isn't a primitive");
  return GeometricPrimitive3D();
}

bool AnyGeometry3D::CanLoadExt(const char *ext)
{
  if(!ext) return false;
  if(Meshing::CanLoadTriMeshExt(ext) || 0 == strcmp(ext, "pcd") || 0 == strcmp(ext, "vol") || 0 == strcmp(ext, "sdf") || 0 == strcmp(ext, "occ") || 0 == strcmp(ext, "geom") || 0 == strcmp(ext, "json") ) return true;
  for(int i=0;i<=int(Type::Group);i++) {
    auto* test = Geometry3D::Make(Type(i));
    auto exts = test->FileExtensions();
    delete test;
    for(auto s : exts) {
      if(s==ext) return true;
    }
  }
  return false;
}

bool AnyGeometry3D::CanSaveExt(const char *ext)
{
  if(!ext) return false;
  if(Meshing::CanSaveTriMeshExt(ext) || 0 == strcmp(ext, "pcd") || 0 == strcmp(ext, "vol") || 0 == strcmp(ext, "sdf") || 0 == strcmp(ext, "occ") || 0 == strcmp(ext, "geom") || 0 == strcmp(ext, "json") ) return true;
  for(int i=0;i<=int(Type::Group);i++) {
    auto* test = Geometry3D::Make(Type(i));
    auto exts = test->FileExtensions();
    delete test;
    for(auto s : exts) {
      if(s==ext) return true;
    }
  }
  return false;
}

bool AnyGeometry3D::Load(const char *fn)
{
  if(data && (type != Type::Primitive && AsPrimitive().type != GeometricPrimitive3D::Empty)) {
    //try loading with the current data type
    if(data->Load(fn)) return true;
  }

  const char *ext = FileExtension(fn);
  if(ext == NULL) {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyGeometry::Load(): Could not determine file extension of "<<fn);
    return false;
  }
  if (Meshing::CanLoadTriMeshExt(ext))
  {
    vector<Meshing::TriMesh> meshes;
    vector<GLDraw::GeometryAppearance> appearances;
    if(!Meshing::LoadAssimp(fn,meshes,appearances)) {
      meshes.resize(1);
      appearances.resize(1);
      if (!Meshing::Import(fn, meshes[0], appearances[0]))
        return false;
    }
    GLDraw::GeometryAppearance blank;
    if(meshes.size()==1) {
      type = Type::TriangleMesh;
      auto* mesh = new Geometry3DTriangleMesh(meshes[0]);
      GLDraw::GeometryAppearance& temp=appearances[0];
      if (temp.faceColor != blank.faceColor ||
        !temp.vertexColors.empty() ||
        !temp.faceColors.empty()) { //loaded appearance data
        mesh->appearance.reset(new GLDraw::GeometryAppearance(temp));
      }
      data.reset(mesh);
    }
    else {
      LOG4CXX_INFO(GET_LOGGER(Geometry),"Loading "<<meshes.size()<<" meshes from "<<fn<<" into Group");
      type = Type::Group;
      auto* group = new Geometry3DGroup();
      group->data.resize(meshes.size());
      for(size_t i=0;i<meshes.size();i++) {
        group->data[i].type = Type::TriangleMesh;
        GLDraw::GeometryAppearance& temp=appearances[i];
        if (temp.faceColor != blank.faceColor ||
          !temp.vertexColors.empty() ||
          !temp.faceColors.empty()) { //loaded appearance data
          group->data[i].data.reset(new Geometry3DTriangleMesh(meshes[i],make_shared<GLDraw::GeometryAppearance>(appearances[i])));
        }
        else {
          group->data[i].data.reset(new Geometry3DTriangleMesh(meshes[i]));
        }
      }
      data.reset(group);
      type = data->GetType();
    }
    return true;
    /*
    type = TriangleMesh;
    data = Meshing::TriMesh();
    GLDraw::GeometryAppearance blank, temp;
    if (!Meshing::Import(fn, this->AsTriangleMesh(), temp))
      return false;
    if (temp.faceColor != blank.faceColor ||
        !temp.vertexColors.empty() ||
        !temp.faceColors.empty()) //loaded appearance data
      appearanceData = temp;
    */
    return true;
  }
  else if (0 == strcmp(ext, "pcd"))
  {
    data.reset(new Geometry3DPointCloud());
    if(!data->Load(fn)) {
      return false;
    }
    type = Type::PointCloud;
    return true;
  }
  else if (0 == strcmp(ext, "vol") || 0 == strcmp(ext, "sdf"))
  {
    data.reset(new Geometry3DImplicitSurface());
    if(!data->Load(fn)) {
      return false;
    }
    type = Type::ImplicitSurface;
    return true;
  }
  else if (0 == strcmp(ext, "occ"))
  {
    data.reset(new Geometry3DOccupancyGrid());
    if(!data->Load(fn)) {
      return false;
    }
    type = Type::OccupancyGrid;
    return true;
  }
  else if (0 == strcmp(ext, "geom"))
  {
    ifstream in(fn,ios::in);
    if(!in) {
      LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyGeometry::Load(): Could not open file "<<fn);
      return false;
    }
    return Load(in);
    // data.reset(new Geometry3DPrimitive());
    // if(!data->Load(fn)) {
    //   return false;
    // }
    // type = Type::Primitive;
    // return true;
  }
  else if (0 == strcmp(ext, "json"))
  {
    data.reset(new Geometry3DHeightmap());
    if(!data->Load(fn)) {
      return false;
    }
    type = Type::Heightmap;
    return true;
  }
  else
  {
    //unknown extension, try all types
    for(int i=0;i<=int(Type::Group);i++) {
      auto* res = Geometry3D::Make(Type(i));
      if(res->Load(fn)) {
        type = res->GetType();
        data.reset(res);
        return true;
      }
      delete res;
    }
  }
  return true;
}

bool AnyGeometry3D::Save(const char *fn) const
{
  if(!data) return false;
  return data->Save(fn);
}

bool AnyGeometry3D::Load(istream &in)
{
  string typestr;
  in >> typestr;
  data.reset(Geometry3D::Make(typestr.c_str()));
  if(!data) return false;
  if(!data->Load(in)) {
    return false;
  }
  type = data->GetType();
  if(!in) {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyGeometry::Load(): Something went wrong inputting item of type "<<typestr);
    return false;
  }
  return true;
}

bool AnyGeometry3D::Save(ostream &out) const
{
  out << TypeName() << endl;
  return data->Save(out);
}

bool AnyGeometry3D::Transform(const RigidTransform &T)
{
  if(!data) return false;
  return data->Transform(T);
}

bool AnyGeometry3D::Transform(const Matrix4 &T)
{
  if(!data) return false;
  return data->Transform(T);
}

AABB3D AnyGeometry3D::GetAABB() const
{
  if(!data) {
    AABB3D bb;
    bb.minimize();
    return bb;
  }
  return data->GetAABB();
}




AnyCollisionGeometry3D::AnyCollisionGeometry3D()
    : margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const GeometricPrimitive3D &primitive)
    : AnyGeometry3D(primitive), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const ConvexHull3D &hull)
    : AnyGeometry3D(hull), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::TriMesh &mesh)
    : AnyGeometry3D(mesh), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::PointCloud3D &pc)
    : AnyGeometry3D(pc), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::VolumeGrid &grid,int value_type)
    : AnyGeometry3D(grid,value_type), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::Heightmap &hm)
    : AnyGeometry3D(hm), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const vector<AnyGeometry3D> &items)
    : AnyGeometry3D(items), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyGeometry3D &geom)
    : AnyGeometry3D(geom), margin(0), collisionHint(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyCollisionGeometry3D &geom)
{
  *this = geom;
}

AnyCollisionGeometry3D &AnyCollisionGeometry3D::operator=(const AnyCollisionGeometry3D &geom)
{
  AnyGeometry3D::operator=(geom);
  margin = geom.margin;
  currentTransform = geom.currentTransform;
  collisionHint = geom.collisionHint;

  //TODO: collider copies don't preserve references to the copied geometry pointer
  if (geom.collider) {
    collider.reset(geom.collider->Copy(data));
    collider->SetTransform(currentTransform);
  }
  else if(collider) {
    collider.reset();
  }
  return *this;
}

const Collider3DPrimitive &AnyCollisionGeometry3D::PrimitiveCollisionData() const { return *dynamic_cast<const Collider3DPrimitive*>(collider.get()); }
const Collider3DConvexHull &AnyCollisionGeometry3D::ConvexHullCollisionData() const { return *dynamic_cast<const Collider3DConvexHull*>(collider.get()); }
const CollisionMesh &AnyCollisionGeometry3D::TriangleMeshCollisionData() const { return dynamic_cast<const Collider3DTriangleMesh*>(collider.get())->collisionData; }
const CollisionPointCloud &AnyCollisionGeometry3D::PointCloudCollisionData() const { return dynamic_cast<const Collider3DPointCloud*>(collider.get())->collisionData; }
const Collider3DImplicitSurface &AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() const { return *dynamic_cast<const Collider3DImplicitSurface*>(collider.get()); }
const Collider3DOccupancyGrid &AnyCollisionGeometry3D::OccupancyGridCollisionData() const { return *dynamic_cast<const Collider3DOccupancyGrid*>(collider.get()); }
const Collider3DHeightmap &AnyCollisionGeometry3D::HeightmapCollisionData() const { return *dynamic_cast<const Collider3DHeightmap*>(collider.get()); }
const vector<AnyCollisionGeometry3D> &AnyCollisionGeometry3D::GroupCollisionData() const { return dynamic_cast<const Collider3DGroup*>(collider.get())->collisionData; }
Collider3DPrimitive &AnyCollisionGeometry3D::PrimitiveCollisionData() { return *dynamic_cast<Collider3DPrimitive*>(collider.get()); }
Collider3DConvexHull &AnyCollisionGeometry3D::ConvexHullCollisionData() { return *dynamic_cast<Collider3DConvexHull*>(collider.get()); }
CollisionMesh &AnyCollisionGeometry3D::TriangleMeshCollisionData() { return dynamic_cast<Collider3DTriangleMesh*>(collider.get())->collisionData; }
CollisionPointCloud &AnyCollisionGeometry3D::PointCloudCollisionData() { return dynamic_cast<Collider3DPointCloud*>(collider.get())->collisionData; }
Collider3DImplicitSurface &AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() { return *dynamic_cast<Collider3DImplicitSurface*>(collider.get()); }
Collider3DOccupancyGrid &AnyCollisionGeometry3D::OccupancyGridCollisionData() { return *dynamic_cast<Collider3DOccupancyGrid*>(collider.get()); }
Collider3DHeightmap &AnyCollisionGeometry3D::HeightmapCollisionData() { return *dynamic_cast<Collider3DHeightmap*>(collider.get()); }
vector<AnyCollisionGeometry3D> &AnyCollisionGeometry3D::GroupCollisionData() { return dynamic_cast<Collider3DGroup*>(collider.get())->collisionData; }

void AnyCollisionGeometry3D::InitCollisionData()
{
  if (!collider)
    ReinitCollisionData();
}

void AnyCollisionGeometry3D::ReinitCollisionData()
{
  Assert(type == data->GetType());
  RigidTransform T = GetTransform();
  collider.reset(Collider3D::Make(data));
  collider->SetTransform(T);
  Assert(type == collider->GetType());
}

void AnyCollisionGeometry3D::Union(const vector<AnyGeometry3D>& geoms)
{
  AnyGeometry3D::Union(geoms);
  ClearCollisionData();
}


void AnyCollisionGeometry3D::Union(const vector<AnyCollisionGeometry3D> &geoms)
{
  vector<int> nonempty;
  for (size_t i = 0; i < geoms.size(); i++) {
    if (!geoms[i].Empty()) {
      nonempty.push_back((int)i);
      if(&geoms[i] == this) {
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyCollisionGeometry3D::Union(): Cannot union with self");
        return;
      }
    }
  }
  if (nonempty.empty())
    *this = AnyGeometry3D();
  else if (nonempty.size() == 1)
  {
    *this = geoms[nonempty[0]];
  }
  else
  {
    type = geoms[nonempty[0]].type;
    data.reset(Geometry3D::Make(type));
    collider.reset(Collider3D::Make(data));
    vector<Collider3D*> cgeoms(nonempty.size());
    for(size_t i=0;i<nonempty.size();i++)
      cgeoms[i] = geoms[nonempty[i]].collider.get();
    if(!collider->Union(cgeoms)) {
        type = Type::Group;
        vector<AnyGeometry3D> geomdatas(geoms.size());
        for(size_t i=0;i<geoms.size();i++)
          geomdatas[i] = geoms[i];
        data.reset(new Geometry3DGroup(geomdatas));
        collider.reset(Collider3D::Make(data));
    }
  }
}

bool AnyCollisionGeometry3D::Merge(const AnyCollisionGeometry3D& other)
{
  if(this == &other) {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyCollisionGeometry3D::Merge(): Cannot merge with self");
    return false;
  }
  Assert(collider != other.collider);
  if(collider && other.collider) {
    if(collider->Merge(other.collider.get())) {
      return true;
    }
    return false;
  }
  RigidTransform Tlocal;
  Tlocal.mulInverseA(currentTransform,other.currentTransform);
  if(!AnyGeometry3D::Merge(other,&Tlocal)) return false;
  ClearCollisionData();
  return true;
}


bool AnyCollisionGeometry3D::Convert(Type restype, AnyCollisionGeometry3D &res, Real param)
{
  if(!data) {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"AnyGeometry3D::Convert(): Converting empty geometry");
    return false;
  }
  res.type = restype;
  res.margin = margin;
  res.currentTransform = currentTransform;
  if(!collider) {
    //default: convert geometry only, leave collider empty
    res.collider.reset();
    res.data.reset(data->Convert(restype,param,margin));
    if(res.data) {
      Assert(restype == res.data->GetType());
      res.type = restype;
      return true;
    }
    return false;
  }
  res.collider.reset(collider->Convert(restype,param,margin));
  if(res.collider) {
    res.data = res.collider->GetData();
    Assert(res.data->GetType() == restype);
    res.type = res.data->GetType();
    return true;
  }
  return false;
}

AABB3D AnyCollisionGeometry3D::GetAABBTight() const
{
  if(!collider) {
    return GetAABB();
  }
  AABB3D bb=collider->GetAABBTight();
  if (margin != 0) {
    bb.bmin -= Vector3(margin);
    bb.bmax += Vector3(margin);
  }
  return bb;
}

AABB3D AnyCollisionGeometry3D::GetAABB() const
{
  if (!collider)
  {
    Box3D b = GetBB();
    AABB3D bb;
    b.getAABB(bb);
    if(!IsFinite(bb.bmin.x))
      printf("AnyCollisionGeometry3D::GetAABB(): Geometry of type %s didn't return a finite bounding box\n",TypeName());
    return bb;
  }
  AABB3D bb=collider->GetAABB();
  if(!IsFinite(bb.bmin.x))
      printf("AnyCollisionGeometry3D::GetAABB():Collider of type %s didn't return a finite bounding box\n",TypeName());  
  if (margin != 0) {
    bb.bmin -= Vector3(margin);
    bb.bmax += Vector3(margin);
  }
  return bb;
}

Box3D AnyCollisionGeometry3D::GetBB() const
{
  Box3D b;  
  if (!collider)
  {
    AABB3D bblocal = AnyGeometry3D::GetAABB();
    b.setTransformed(bblocal, currentTransform);
  }
  else
  {
    b = collider->GetBB();
  }
  //expand by the margin
  if (margin != 0)
  {
    b.expand(margin);
  }
  return b;
}

RigidTransform AnyCollisionGeometry3D::GetTransform() const
{
  return currentTransform;
}

void AnyCollisionGeometry3D::SetTransform(const RigidTransform &T)
{
  Assert(type == data->GetType());
  currentTransform = T;
  if(collider) collider->SetTransform(T);
}

bool AnyCollisionGeometry3D::Transform(const RigidTransform& T)
{
  if(!AnyGeometry3D::Transform(T)) return false;
  if(collider) {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"AnyCollisionGeometry3D::Transform(): Collision data structure will need to be reset (performance warning)");
    ReinitCollisionData();
  }
  return true;
}

bool AnyCollisionGeometry3D::Transform(const Matrix4& mat)
{
  if(!AnyGeometry3D::Transform(mat)) return false;
  if(collider) {
    LOG4CXX_INFO(GET_LOGGER(Geometry),"AnyCollisionGeometry3D::Transform(): Collision data structure will need to be reset (performance warning)");
    ReinitCollisionData();
  }
  return true;
}

Real AnyCollisionGeometry3D::Distance(const Vector3 &pt)
{
  InitCollisionData();
  Real d=Inf;
  if(collider->Distance(pt,d)) return d;
  return Inf;
}

AnyDistanceQueryResult AnyCollisionGeometry3D::Distance(const Vector3 &pt, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  InitCollisionData();
  if(collider->Distance(pt,settings,res)) return res;
  res.d = Inf;
  return res;
}

bool AnyCollisionGeometry3D::Support(const Vector3& dir,Vector3& pt)
{
  if(!collider) {
    Vector3 dirlocal,ptlocal;
    currentTransform.R.mulTranspose(dir,dirlocal);
    if(!data->Support(dir,ptlocal)) return false;
    pt = currentTransform*ptlocal;
    return true;
  }
  return collider->Support(dir,pt);
}

bool AnyCollisionGeometry3D::Contains(const Vector3& pt)
{
  InitCollisionData();
  bool res;
  if(!collider->Contains(pt,res)) return false;
  return res;
}

bool AnyCollisionGeometry3D::Collides(AnyCollisionGeometry3D &geom)
{
  InitCollisionData();
  geom.InitCollisionData();
  if(margin + geom.margin == 0) {
    if(type < geom.type) {
      bool result;
      if(geom.collider->Collides(collider.get(),result)) { return result; }
      if(collider->Collides(geom.collider.get(),result)) { return result; }
      return false;
    }
    else {
      bool result;
      if(collider->Collides(geom.collider.get(),result)) { return result; }
      if(geom.collider->Collides(collider.get(),result)) { return result; }
      return false;
    }
  }
  else {
    if(type < geom.type) {
      bool result;
      if(geom.collider->WithinDistance(collider.get(),margin+geom.margin,result)) return result;
      if(collider->WithinDistance(geom.collider.get(),margin+geom.margin,result)) return result;
      return false;
    }
    else {
      bool result;
      if(collider->WithinDistance(geom.collider.get(),margin+geom.margin,result)) return result;
      if(geom.collider->WithinDistance(collider.get(),margin+geom.margin,result)) return result;
      return false;
    }
  }
}

bool AnyCollisionGeometry3D::Collides(AnyCollisionGeometry3D &geom,
                                      vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  InitCollisionData();
  geom.InitCollisionData();
  if(margin + geom.margin == 0) {
    if(type < geom.type) {
      if(geom.collider->Collides(collider.get(),elements2,elements1,maxContacts)) { return !elements1.empty(); }
      if(collider->Collides(geom.collider.get(),elements1,elements2,maxContacts)) { return !elements1.empty(); }
      return false;
    }
    else {
      if(collider->Collides(geom.collider.get(),elements1,elements2,maxContacts)) { return !elements1.empty(); }
      if(geom.collider->Collides(collider.get(),elements2,elements1,maxContacts)) { return !elements1.empty(); }
      return false;
    }
  }
  else {
    if(type < geom.type) {
      if(geom.collider->WithinDistance(collider.get(),margin+geom.margin,elements2,elements1,maxContacts)) { return !elements1.empty(); }
      if(collider->WithinDistance(geom.collider.get(),margin+geom.margin,elements1,elements2,maxContacts)) { return !elements1.empty(); }
      return false;
    }
    else {
      if(collider->WithinDistance(geom.collider.get(),margin+geom.margin,elements1,elements2,maxContacts)) { return !elements1.empty(); }
      if(geom.collider->WithinDistance(collider.get(),margin+geom.margin,elements2,elements1,maxContacts)) { return !elements1.empty(); }
      return false;
    }
  }
}

Real AnyCollisionGeometry3D::Distance(AnyCollisionGeometry3D &geom)
{
  InitCollisionData();
  geom.InitCollisionData();
  AnyDistanceQuerySettings settings;
  return Distance(geom, settings).d;
}

AnyDistanceQueryResult AnyCollisionGeometry3D::Distance(AnyCollisionGeometry3D &geom, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult result;
  result.hasElements = true;
  InitCollisionData();
  geom.InitCollisionData();
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += margin + geom.margin;
  if(type < geom.type) {
    if(geom.collider->Distance(collider.get(),modsettings,result)) { Flip(result); Offset(result,margin,geom.margin); return result; }
    if(collider->Distance(geom.collider.get(),modsettings,result)) { Offset(result,margin,geom.margin); return result; }
    result.d = Inf;
    return result;
  }
  else {
    if(collider->Distance(geom.collider.get(),modsettings,result)) { Offset(result,margin,geom.margin); return result; }
    if(geom.collider->Distance(collider.get(),modsettings,result)) { Flip(result); Offset(result,margin,geom.margin); return result; }
    result.d = Inf;
    return result;
  }
  return result;
}

bool AnyCollisionGeometry3D::WithinDistance(AnyCollisionGeometry3D &geom, Real tol)
{
  InitCollisionData();
  geom.InitCollisionData();
  tol += margin + geom.margin;
  bool result;
  if(type < geom.type) {
    if(geom.collider->WithinDistance(collider.get(),tol,result)) { return result; }
    if(collider->WithinDistance(geom.collider.get(),tol,result)) { return result; }
    return false;
  }
  else {
    if(collider->WithinDistance(geom.collider.get(),tol,result)) { return result; }
    if(geom.collider->WithinDistance(collider.get(),tol,result)) { return result; }
    return false;
  }
}

bool AnyCollisionGeometry3D::WithinDistance(AnyCollisionGeometry3D &geom, Real tol,
                                            vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  InitCollisionData();
  geom.InitCollisionData();
  tol += margin + geom.margin;
  if(type < geom.type) {
    if(geom.collider->WithinDistance(collider.get(),tol,elements2,elements1,maxContacts)) { return !elements1.empty(); }
    if(collider->WithinDistance(geom.collider.get(),tol,elements1,elements2,maxContacts)) { return !elements1.empty(); }
    return false;
  }
  else {
    if(collider->WithinDistance(geom.collider.get(),tol,elements1,elements2,maxContacts)) { return !elements1.empty(); }
    if(geom.collider->WithinDistance(collider.get(),tol,elements2,elements1,maxContacts)) { return !elements1.empty(); }
    return false;
  }
}

bool AnyCollisionGeometry3D::RayCast(const Ray3D &r, Real *distance, int *element)
{
  InitCollisionData();
  Real tempdist;
  int tempelem;
  if(!distance) distance = &tempdist;
  if(!element) element = &tempelem;
  *distance = Inf;
  *element = -1;
  if(collider->RayCast(r,margin,*distance,*element)) {
    return (*element >= 0);
  }
  return false;
}

  
bool AnyCollisionGeometry3D::Slice(const RigidTransform& T,AnyCollisionGeometry3D& res,Real tol) const
{
  if(!collider) { //fall back to non-accelerated slicing
    RigidTransform Tlocal;
    Tlocal.mulInverseA(currentTransform,T);
    if(!AnyGeometry3D::Slice(Tlocal,res,tol)) return false;
    res.currentTransform = currentTransform;
    return true;
  }
  Collider3D* coll = collider->Slice(T,tol);
  if(!coll) return false;
  res.collider.reset(coll);
  res.data = coll->GetData();
  res.type = coll->GetType();
  res.SetTransform(T);
  return true;
}



bool AnyCollisionGeometry3D::ExtractROI(const AABB3D& bb,AnyCollisionGeometry3D& res,int flags) const
{
  if(!collider) { //fall back to non-accelerated ROI
    RigidTransform Tinv;
    Tinv.setInverse(currentTransform);
    Box3D blocal;
    blocal.setTransformed(bb,Tinv);
    if(!AnyGeometry3D::ExtractROI(blocal,res,flags)) return false;
    res.currentTransform = currentTransform;
    return true;
  }
  auto* coll = collider->ExtractROI(bb,flags);
  if(!coll) return false;
  res.collider.reset(coll);
  res.data = coll->GetData();
  res.type = coll->GetType();
  res.SetTransform(coll->GetTransform());;
  return true;
}

bool AnyCollisionGeometry3D::ExtractROI(const Box3D& bb,AnyCollisionGeometry3D& res,int flags) const
{
  if(!collider) { //fall back to non-accelerated ROI
    RigidTransform Tinv;
    Tinv.setInverse(currentTransform);
    Box3D blocal;
    blocal.setTransformed(bb,Tinv);
    if(!AnyGeometry3D::ExtractROI(blocal,res,flags)) return false;
    res.currentTransform = currentTransform;
    return true;
  }
  auto* coll = collider->ExtractROI(bb,flags);
  if(!coll) return false;
  res.collider.reset(coll);
  res.data = coll->GetData();
  res.type = coll->GetType();
  res.SetTransform(coll->GetTransform());;
  return true;
}
















AnyContactsQueryResult AnyCollisionGeometry3D::Contacts(AnyCollisionGeometry3D &other, const AnyContactsQuerySettings &settings)
{
  InitCollisionData();
  other.InitCollisionData();
  Assert(type == data->GetType());
  Assert(collider->GetType() == type);
  Assert(collider->GetData().get()  == data.get());
  Assert(other.type == other.data->GetType());
  Assert(other.collider->GetType() == other.type);
  Assert(other.collider->GetData().get()  == other.data.get());
  AnyContactsQueryResult res;
  AnyContactsQuerySettings modsettings = settings;
  if (settings.cluster)
    modsettings.maxcontacts = std::numeric_limits<size_t>::max();
  modsettings.padding1 += margin;
  modsettings.padding2 += other.margin;
  if(type < other.type) {
    std::swap(modsettings.padding1,modsettings.padding2);
    if(other.collider->Contacts(collider.get(),modsettings,res)) {
      for(size_t i=0;i<res.contacts.size();i++)
        ReverseContact(res.contacts[i]);
      res.clustered = false;
      return res;
    }
    std::swap(modsettings.padding1,modsettings.padding2);
    if(collider->Contacts(other.collider.get(),modsettings,res)) {
      res.clustered = false;
      return res;
    }
  }
  else {
    if(collider->Contacts(other.collider.get(),modsettings,res)) {
      res.clustered = false;
      return res;
    }
    std::swap(modsettings.padding1,modsettings.padding2);
    if(other.collider->Contacts(collider.get(),modsettings,res)) {
      for(size_t i=0;i<res.contacts.size();i++)
        ReverseContact(res.contacts[i]);
      res.clustered = false;
      return res;
    }
    std::swap(modsettings.padding1,modsettings.padding2);
  }
  return res;
}

AnyCollisionQuery::AnyCollisionQuery()
    : a(NULL), b(NULL)
{
}

AnyCollisionQuery::AnyCollisionQuery(AnyCollisionGeometry3D &_a, AnyCollisionGeometry3D &_b)
    : a(&_a), b(&_b)
{
}

AnyCollisionQuery::AnyCollisionQuery(const AnyCollisionQuery &q)
    : a(q.a), b(q.b), qmesh(q.qmesh)
{
}

bool UpdateQMesh(AnyCollisionQuery *q)
{
  if (q->a->type == AnyGeometry3D::Type::TriangleMesh && q->b->type == AnyGeometry3D::Type::TriangleMesh)
  {
    if (!q->qmesh.m1)
    {
      q->a->InitCollisionData();
      q->b->InitCollisionData();
      q->qmesh = CollisionMeshQueryEnhanced(q->a->TriangleMeshCollisionData(), q->b->TriangleMeshCollisionData());
      Assert(q->qmesh.m1);
      Assert(q->qmesh.m2);
    }
    else
    {
      Assert(q->qmesh.m2);
    }
    q->qmesh.margin1 = q->a->margin;
    q->qmesh.margin2 = q->b->margin;
    return true;
  }
  return false;
}

bool AnyCollisionQuery::Collide()
{
  if (!a || !b)
    return false;
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if (UpdateQMesh(this))
  {
    if (qmesh.Collide())
    {
      qmesh.CollisionPairs(elements1, elements2);
      return true;
    }
    return false;
  }
  return a->Collides(*b, elements1, elements2, 1);
}

bool AnyCollisionQuery::CollideAll()
{
  if (!a || !b)
    return false;
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if (UpdateQMesh(this))
  {
    if (qmesh.CollideAll())
    {
      qmesh.CollisionPairs(elements1, elements2);
      return true;
    }
    return false;
  }
  return a->Collides(*b, elements1, elements2);
}

bool AnyCollisionQuery::WithinDistance(Real d)
{
  if (!a || !b)
    return false;
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if (UpdateQMesh(this))
  {
    if (qmesh.WithinDistance(d))
    {
      elements1.resize(1);
      elements2.resize(1);
      points1.resize(1);
      points2.resize(1);
      qmesh.TolerancePair(elements1[0], elements2[0]);
      qmesh.TolerancePoints(points1[0], points2[0]);
      return true;
    }
    return false;
  }
  return a->WithinDistance(*b, d, elements1, elements2, 1);
}

bool AnyCollisionQuery::WithinDistanceAll(Real d)
{
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if (UpdateQMesh(this))
  {
    if (qmesh.WithinDistanceAll(d))
    {
      qmesh.TolerancePairs(elements1, elements2);
      qmesh.TolerancePoints(points1, points2);
      return true;
    }
  }
  return a->WithinDistance(*b, d, elements1, elements2);
}

Real AnyCollisionQuery::PenetrationDepth()
{
  if (!a || !b)
    return -Inf;
  if (UpdateQMesh(this))
  {
    return qmesh.PenetrationDepth();
  }
  return -a->Distance(*b);
}

Real AnyCollisionQuery::Distance(Real absErr, Real relErr, Real bound)
{
  if (!a || !b)
    return Inf;
  elements1.resize(1);
  elements2.resize(1);
  points1.resize(0);
  points2.resize(0);
  if (UpdateQMesh(this))
  {
    points1.resize(1);
    points2.resize(1);
    Real res = qmesh.Distance(absErr, relErr, bound);
    qmesh.ClosestPair(elements1[0], elements2[0]);
    qmesh.ClosestPoints(points1[0], points2[0]);
    return res;
  }
  AnyDistanceQuerySettings settings;
  settings.absErr = absErr;
  settings.relErr = relErr;
  settings.upperBound = bound;
  AnyDistanceQueryResult res = a->Distance(*b, settings);
  if (res.hasElements)
  {
    elements1[0] = res.elem1;
    elements2[0] = res.elem2;
  }
  if (res.hasClosestPoints)
  {
    points1.resize(1);
    points2.resize(1);
    points1[0] = res.cp1;
    points2[0] = res.cp2;
  }
  return res.d;
}

void AnyCollisionQuery::InteractingPairs(std::vector<int> &t1, std::vector<int> &t2) const
{
  t1 = elements1;
  t2 = elements2;
}

void AnyCollisionQuery::InteractingPoints(std::vector<Vector3> &p1, std::vector<Vector3> &p2) const
{
  if (points1.empty() && !elements1.empty())
  {
    //need to compute points from elements
    FatalError("TODO: compute interacting points from interacting elements");
  }
  else
  {
    p1 = points1;
    p2 = points2;
  }
}
