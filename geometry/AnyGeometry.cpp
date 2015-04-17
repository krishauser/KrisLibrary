#include "AnyGeometry.h"
#include <math3d/geometry3d.h>
#include <meshing/VolumeGrid.h>
#include <meshing/Voxelize.h>
#include "CollisionPointCloud.h"
#include <utils/stringutils.h>
#include <meshing/IO.h>
#include <Timer.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>

using namespace Geometry;

AnyGeometry3D::AnyGeometry3D()
  :type(Primitive),data(GeometricPrimitive3D())
{}

AnyGeometry3D::AnyGeometry3D(const GeometricPrimitive3D& primitive)
  :type(Primitive),data(primitive)
{}

AnyGeometry3D::AnyGeometry3D(const Meshing::TriMesh& mesh)
  :type(TriangleMesh),data(mesh)
{}

AnyGeometry3D::AnyGeometry3D(const Meshing::PointCloud3D& pc)
  :type(PointCloud),data(pc)
{}

AnyGeometry3D::AnyGeometry3D(const Meshing::VolumeGrid& grid)
  :type(ImplicitSurface),data(grid)
{}

AnyGeometry3D::AnyGeometry3D(const vector<AnyGeometry3D>& group)
  :type(Group),data(group)
{}

AnyGeometry3D::AnyGeometry3D(const AnyGeometry3D& geom)
  :type(geom.type),data(geom.data)
{}

const GeometricPrimitive3D& AnyGeometry3D::AsPrimitive() const { return *AnyCast<GeometricPrimitive3D>(&data); }
const Meshing::TriMesh& AnyGeometry3D::AsTriangleMesh() const { return *AnyCast<Meshing::TriMesh>(&data); }
const Meshing::PointCloud3D& AnyGeometry3D::AsPointCloud() const { return *AnyCast<Meshing::PointCloud3D>(&data); }
const Meshing::VolumeGrid& AnyGeometry3D::AsImplicitSurface() const { return *AnyCast<Meshing::VolumeGrid>(&data); }
const vector<AnyGeometry3D>& AnyGeometry3D::AsGroup() const { return *AnyCast<vector<AnyGeometry3D> >(&data); }
GeometricPrimitive3D& AnyGeometry3D::AsPrimitive() { return *AnyCast<GeometricPrimitive3D>(&data); }
Meshing::TriMesh& AnyGeometry3D::AsTriangleMesh() { return *AnyCast<Meshing::TriMesh>(&data); }
Meshing::PointCloud3D& AnyGeometry3D::AsPointCloud() { return *AnyCast<Meshing::PointCloud3D>(&data); }
Meshing::VolumeGrid& AnyGeometry3D::AsImplicitSurface() { return *AnyCast<Meshing::VolumeGrid>(&data); }
vector<AnyGeometry3D>& AnyGeometry3D::AsGroup() { return *AnyCast<vector<AnyGeometry3D> >(&data); }

const char* AnyGeometry3D::TypeName(Type type)
{
  switch(type) { 
  case Primitive: return "Primitive";
  case TriangleMesh: return "TriangleMesh";
  case PointCloud: return "PointCloud";
  case ImplicitSurface: return "ImplicitSurface";
  case Group: return "Group";
  default: return "Error";
  }
}

bool AnyGeometry3D::Empty() const
{
  switch(type) {
  case Primitive:
    return AsPrimitive().type == GeometricPrimitive3D::Empty;
  case TriangleMesh:
    return AsTriangleMesh().tris.empty();
  case PointCloud:
    return AsPointCloud().points.empty();
  case ImplicitSurface:
    return false;
  case Group:
    return AsGroup().empty();
  }
  return false;
}

void AnyGeometry3D::Merge(const vector<AnyGeometry3D>& geoms)
{
  vector<int> nonempty;
  for(size_t i=0;i<geoms.size();i++)
    if(!geoms[i].Empty()) nonempty.push_back(i);
  if(nonempty.empty()) *this = AnyGeometry3D();
  else  if(nonempty.size()==1) {
    *this = geoms[nonempty[0]];
  }
  else {
    type = geoms[nonempty[0]].type;
    bool group = false;
    for(size_t i=1;i<nonempty.size();i++)
      if(geoms[nonempty[i]].type != type) {
	//its' a group type
	group = true;
      }
    switch(type) {
    case Primitive:
      group = true;
      break;
    case TriangleMesh:
      {
	Meshing::TriMesh merged;
	vector<Meshing::TriMesh> items(nonempty.size());
	for(size_t i=0;i<nonempty.size();i++)
	  items[i] = geoms[nonempty[i]].AsTriangleMesh();
	merged.Merge(items);
	data = merged;
      }
      break;
    case PointCloud:
      group = true;
      break;
    case ImplicitSurface:
      group = true;
      break;
    case Group:
      //don't need to add an extra level of hierarchy
      {
	vector<AnyGeometry3D> items;
	for(size_t i=0;i<nonempty.size();i++)
	  items.insert(items.end(),geoms[nonempty[i]].AsGroup().begin(),geoms[nonempty[i]].AsGroup().end());
	data = items;
      }
      break;
    }
    if(group) {
      type = Group;
      data = geoms;
    }
  }
}

size_t AnyGeometry3D::NumElements() const
{
  switch(type) {
  case Primitive:
    if(AsPrimitive().type == GeometricPrimitive3D::Empty) return 0;
    return 1;
  case TriangleMesh:
    return AsTriangleMesh().tris.size();
  case PointCloud:
    return AsPointCloud().points.size();
  case ImplicitSurface:
    {
      IntTriple size = AsImplicitSurface().value.size();
      return size.a*size.b*size.c;
    }
  case Group:
    return AsGroup().size();
  }
  return 0;
}

bool AnyGeometry3D::CanLoadExt(const char* ext)
{
  return Meshing::CanLoadTriMeshExt(ext) || 0==strcmp(ext,"pcd") || 0==strcmp(ext,"vol");
}

bool AnyGeometry3D::CanSaveExt(const char* ext)
{
  return Meshing::CanSaveTriMeshExt(ext) || 0==strcmp(ext,"pcd") || 0==strcmp(ext,"vol");
}

bool AnyGeometry3D::Load(const char* fn)
{
  const char* ext = FileExtension(fn);
  if(Meshing::CanLoadTriMeshExt(ext)) {
    type = TriangleMesh;
    data = Meshing::TriMesh();
    return Meshing::Import(fn,this->AsTriangleMesh());
  }
  else if(0==strcmp(ext,"pcd")) {
    type = PointCloud;
    data = Meshing::PointCloud3D();
    return this->AsPointCloud().LoadPCL(fn);
  }
  else if(0==strcmp(ext,"vol")) {
    type = ImplicitSurface;
    data = Meshing::VolumeGrid();
    ifstream in(fn,ios::in);
    if(!in) return false;
    in >> this->AsImplicitSurface();
    if(!in) return false;
    in.close();
    return true;
  }
  else {
    ifstream in(fn,ios::in);
    if(!in) return false;
    if(!Load(in)) return false;
    in.close();
    return true;
  }
  return true;
}

bool AnyGeometry3D::Save(const char* fn) const
{
  const char* ext = FileExtension(fn);
  switch(type) {
  case Primitive:
    break;
  case TriangleMesh:
    if(Meshing::CanSaveTriMeshExt(ext)) {
      return Meshing::Export(fn,this->AsTriangleMesh());
    }
    break;
  case PointCloud:
    if(0==strcmp(ext,"pcd")) {
      return this->AsPointCloud().SavePCL(fn);
    }
    break;
  case ImplicitSurface:
    {
    ofstream out(fn,ios::out);
    if(!out) return false;
    out<<this->AsImplicitSurface();
    out<<endl;
    out.close();
    return true;
    }
    break;
  case Group:
    break;
  }
  //default save
  ofstream out(fn,ios::out);
  if(!out) return false;
  if(!Save(out)) return false;
  out.close();
  return true;
}

bool AnyGeometry3D::Load(istream& in)
{
  string typestr;
  in>>typestr;
  if(typestr == "Primitive") {
    GeometricPrimitive3D geom;
    in>>geom;
    if(in) {
      type = Primitive;
      data = geom;
      return true;
    }
    else {
      fprintf(stderr,"Failed to load Primitive type\n");
      return false;
    }
  }
  else if(typestr == "TriangleMesh") {
    type = TriangleMesh;
    data = Meshing::TriMesh();
    in >> this->AsTriangleMesh();
  }
  else if(typestr == "PointCloud") {
    type = PointCloud;
    data = Meshing::PointCloud3D();
    if(!AsPointCloud().LoadPCL(in)) return false;
    return true;
  }
  else if(typestr == "ImplicitSurface") {
    type = ImplicitSurface;
    data = Meshing::VolumeGrid();
    in >> this->AsImplicitSurface();
  }
  else if(typestr == "Group") {
    fprintf(stderr,"AnyGeometry::Load(): TODO: groups\n");
    return false;
  }
  if(!in) return false;
  return true;
}

bool AnyGeometry3D::Save(ostream& out) const
{
  out<<TypeName()<<endl;
  switch(type) {
  case Primitive:
    out<<this->AsPrimitive()<<endl;
    break;
  case TriangleMesh:
    out<<this->AsTriangleMesh()<<endl;
    break;
  case PointCloud:
    if(!AsPointCloud().SavePCL(out)) return false;
    break;
  case ImplicitSurface:
    out<<this->AsImplicitSurface()<<endl;
    break;
  case Group:
    fprintf(stderr,"AnyGeometry::Save(): TODO: groups\n");
    return false;
  }
  return true;
}

void AnyGeometry3D::Transform(const RigidTransform& T)
{
  return Transform(Matrix4(T));
}


void AnyGeometry3D::Transform(const Matrix4& T)
{
  switch(type) {
  case Primitive:
    AsPrimitive().Transform(T);
    break;
  case TriangleMesh:
    AsTriangleMesh().Transform(T);
    break;
  case PointCloud:
    AsPointCloud().Transform(T);
    break;
  case ImplicitSurface:
    {
      if(T(0,1) != 0 || T(0,2) != 0 || T(1,2) != 0 || T(1,0) != 0 || T(2,0) != 0 || T(2,1) != 0 ) {
	FatalError("Cannot transform volume grid except via translation / scale");
      }
      AsImplicitSurface().bb.bmin = T*AsImplicitSurface().bb.bmin;
      AsImplicitSurface().bb.bmax = T*AsImplicitSurface().bb.bmax;
    }
    break;
  case Group:
    {
      vector<AnyGeometry3D>& items = AsGroup();
      for(size_t i=0;i<items.size();i++)
	items[i].Transform(T);
    }
    break;
  }
}

//bool AnyGeometry3D::Load(TiXmlElement* in);
//bool AnyGeometry3D::Save(TiXmlElement* out) const;

AABB3D AnyGeometry3D::GetAABB() const
{
  AABB3D bb;
  bb.minimize();
  switch(type) {
  case Primitive:
    return AsPrimitive().GetAABB();
  case TriangleMesh:
    AsTriangleMesh().GetAABB(bb.bmin,bb.bmax);
    return bb;
  case PointCloud:
    {
      const Meshing::PointCloud3D& pc=AsPointCloud();
      for(size_t i=0;i<pc.points.size();i++)
	bb.expand(pc.points[i]);
    }
    return bb;
  case ImplicitSurface:
    AsImplicitSurface().bb;
    break;
  case Group:
    {
      const vector<AnyGeometry3D>& items = AsGroup();
      for(size_t i=0;i<items.size();i++) {
	AABB3D itembb = items[i].GetAABB();
	bb.setUnion(itembb);
      }
    }
    break;
  }
  return bb;
}




AnyCollisionGeometry3D::AnyCollisionGeometry3D()
  :margin(0)
{}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const GeometricPrimitive3D& primitive)
  :AnyGeometry3D(primitive),margin(0)
{
  InitCollisions();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::TriMesh& mesh)
  :AnyGeometry3D(mesh),margin(0)
{
  InitCollisions();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::PointCloud3D& pc)
  :AnyGeometry3D(pc),margin(0)
{
  InitCollisions();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::VolumeGrid& grid)
  :AnyGeometry3D(grid),margin(0)
{
  InitCollisions();
}


AnyCollisionGeometry3D::AnyCollisionGeometry3D(const vector<AnyGeometry3D>& items)
  :AnyGeometry3D(items),margin(0)
{
  InitCollisions();
}


AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyGeometry3D& geom)
  :AnyGeometry3D(geom),margin(0)
{
  InitCollisions();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyCollisionGeometry3D& geom)
  :AnyGeometry3D(geom),margin(geom.margin)
{
  InitCollisions();
}

  const RigidTransform& AnyCollisionGeometry3D::PrimitiveCollisionData() const { return *AnyCast<RigidTransform>(&collisionData); }
  const CollisionMesh& AnyCollisionGeometry3D::TriangleMeshCollisionData() const { return *AnyCast<CollisionMesh>(&collisionData); }
  const CollisionPointCloud& AnyCollisionGeometry3D::PointCloudCollisionData() const { return *AnyCast<CollisionPointCloud>(&collisionData); }
  const RigidTransform& AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() const { return *AnyCast<RigidTransform>(&collisionData); }
  const vector<AnyCollisionGeometry3D>& AnyCollisionGeometry3D::GroupCollisionData() const { return *AnyCast<vector<AnyCollisionGeometry3D> >(&collisionData); }
  RigidTransform& AnyCollisionGeometry3D::PrimitiveCollisionData() { return *AnyCast<RigidTransform>(&collisionData); }
  CollisionMesh& AnyCollisionGeometry3D::TriangleMeshCollisionData() { return *AnyCast<CollisionMesh>(&collisionData); }
  CollisionPointCloud& AnyCollisionGeometry3D::PointCloudCollisionData() { return *AnyCast<CollisionPointCloud>(&collisionData); }
  RigidTransform& AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() { return *AnyCast<RigidTransform>(&collisionData); }
  vector<AnyCollisionGeometry3D>& AnyCollisionGeometry3D::GroupCollisionData() { return *AnyCast<vector<AnyCollisionGeometry3D> >(&collisionData); }

void AnyCollisionGeometry3D::InitCollisions()
{
  switch(type) {
  case Primitive:
  case ImplicitSurface:
    {
      RigidTransform T; T.setIdentity();
      collisionData = T;
    }
    break;
  case TriangleMesh:
    collisionData = CollisionMesh(AsTriangleMesh());
    break;
  case PointCloud:
    collisionData = CollisionPointCloud(AsPointCloud());
    break;
  case Group:
    {
      collisionData = vector<AnyCollisionGeometry3D>();
      vector<AnyCollisionGeometry3D>& colitems = GroupCollisionData();
      vector<AnyGeometry3D>& items = AsGroup();
      colitems.resize(items.size());
      for(size_t i=0;i<items.size();i++)
	colitems[i] = AnyCollisionGeometry3D(items[i]);
    }
    break;
  }
}

AABB3D AnyCollisionGeometry3D::GetAABB() const
{
  if(collisionData.empty()) return AnyGeometry3D::GetAABB();
  switch(type) {
  case Primitive:
    {
      const RigidTransform& T=PrimitiveCollisionData();
      const GeometricPrimitive3D& g=AsPrimitive();
      GeometricPrimitive3D gT(g);
      gT.Transform(T);
      AABB3D res = gT.GetAABB();
      if(margin != 0) {
	res.bmin -= Vector3(margin);
	res.bmax += Vector3(margin);
      }
      return res;
    }
    break;
  case TriangleMesh:
  case PointCloud:
  case ImplicitSurface:
    {
      AABB3D bb;
      Box3D b = GetBB();
      b.getAABB(bb);
      return bb;
    }
  case Group:
    {
      const vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      AABB3D bb;
      bb.minimize();
      for(size_t i=0;i<items.size();i++)
	bb.setUnion(items[i].GetAABB());
      if(margin != 0) {
	bb.bmin -= Vector3(margin);
	bb.bmax += Vector3(margin);
      }
      return bb;
    }
  }
  AssertNotReached();
  AABB3D bb;
  return bb;
}

Box3D AnyCollisionGeometry3D::GetBB() const
{
  Box3D b;
  if(collisionData.empty()) {
    if(type == Primitive) {
      b = AsPrimitive().GetBB();
    }
    else {
      AABB3D aabb = AnyGeometry3D::GetAABB();
      b.set(aabb);
    }
  }
  else {
    switch(type) {
    case Primitive:
      {
	Box3D blocal = AsPrimitive().GetBB();
	b.setTransformed(blocal,PrimitiveCollisionData());
      }
      break;
    case TriangleMesh:
      ::GetBB(TriangleMeshCollisionData(),b);
      break;
    case PointCloud:
      ::GetBB(PointCloudCollisionData(),b);
      break;
    case ImplicitSurface:
      b.setTransformed(AsImplicitSurface().bb,ImplicitSurfaceCollisionData());
      break;
    case Group:
      {
	AABB3D bb = GetAABB();
	b.set(bb);
      }
      break;
    }
  }
  //expand by the margin
  if(margin != 0) {
    b.dims += Vector3(margin*2.0);
    b.origin -= margin * (b.xbasis+b.ybasis+b.zbasis);
  }
  return b;
}

RigidTransform AnyCollisionGeometry3D::GetTransform() const
{
  if(collisionData.empty()) {
    RigidTransform T;
    T.setIdentity();
    return T;
  }
  switch(type) {
  case Primitive:
  case ImplicitSurface:
    return *AnyCast<RigidTransform>(&collisionData);
  case TriangleMesh:
    return TriangleMeshCollisionData().currentTransform;
  case PointCloud:
    return PointCloudCollisionData().currentTransform;
  case Group:
    return GroupCollisionData()[0].GetTransform();
  }
  AssertNotReached();
  RigidTransform T;
  return T;
}

void AnyCollisionGeometry3D::SetTransform(const RigidTransform& T)
{
  if(collisionData.empty()) InitCollisions();
  switch(type) {
  case Primitive:
  case ImplicitSurface:
    *AnyCast<RigidTransform>(&collisionData) = T;
    break;
  case TriangleMesh:
    TriangleMeshCollisionData().UpdateTransform(T);
    break;
  case PointCloud:
    PointCloudCollisionData().currentTransform = T;
    break;
  case Group:
    {
      vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      for(size_t i=0;i<items.size();i++)
	items[i].SetTransform(T);
    }
    break;
  }
}

Real AnyCollisionGeometry3D::Distance(const Vector3& pt) const
{
  Vector3 ptlocal;
  GetTransform().mulInverse(pt,ptlocal);
  switch(type) {
  case Primitive:
    return Max(AsPrimitive().Distance(ptlocal)-margin,0.0);
  case ImplicitSurface:
    return AsImplicitSurface().TrilinearInterpolate(ptlocal);
  case TriangleMesh:
    {
      Vector3 cp;
      ClosestPoint(TriangleMeshCollisionData(),pt,cp);
      return Min(pt.distance(cp)-margin,0.0);
    }
  case PointCloud:
    {
      const CollisionPointCloud& pc = PointCloudCollisionData();
      Vector3 cp;
      int id;
      if(!pc.octree->NearestNeighbor(ptlocal,cp,id)) return Inf;
      return Min(cp.distance(ptlocal)-margin,0.0);
      /*
      Real dmin = Inf;
      for(size_t i=0;i<pc.points.size();i++)
	dmin = Min(dmin,ptlocal.distanceSquared(pc.points[i]));
      return Min(Sqrt(dmin)-margin,0.0);
      */
    }
  case Group:
    {
      const vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      Real dmin = Inf;
      for(size_t i=0;i<items.size();i++)
	dmin = Min(dmin,items[i].Distance(pt));
      return Min(Sqrt(dmin)-margin,0.0);
    }
  }
  return Inf;
}



bool Collides(const Meshing::VolumeGrid& grid,const GeometricPrimitive3D& a,Real margin,
	      vector<int>& gridelements,size_t maxContacts)
{
  if(a.type != GeometricPrimitive3D::Point && a.type != GeometricPrimitive3D::Sphere) {
    FatalError("Can't collide an implicit surface and a non-sphere primitive yet\n");
  }
  if(a.type == GeometricPrimitive3D::Point) {
    const Vector3& pt = *AnyCast<Vector3>(&a.data);
    bool res = (grid.TrilinearInterpolate(pt) + a.Distance(grid.bb) <= margin);
    if(res) {
      gridelements.resize(1);
      IntTriple cell;
      grid.GetIndex(pt,cell);
      gridelements[0] = cell.a*grid.value.n*grid.value.p + cell.b*grid.value.p + cell.c;
    }
    return res;
  }
  const Sphere3D* s=AnyCast<Sphere3D>(&a.data);
  bool res = (grid.TrilinearInterpolate(s->center) + a.Distance(grid.bb) <= margin+s->radius);  
  if(res) {
    gridelements.resize(1);
    IntTriple cell;
    grid.GetIndex(s->center,cell);
    gridelements[0] = cell.a*grid.value.n*grid.value.p + cell.b*grid.value.p + cell.c;
  }
  return res;
}

bool Collides(const GeometricPrimitive3D& a,const GeometricPrimitive3D& b,Real margin)
{
  if(margin==0) return a.Collides(b);
  return a.Distance(b) <= margin;
}

bool Collides(const GeometricPrimitive3D& a,const Meshing::VolumeGrid& b,const RigidTransform& Tb,Real margin,
	      vector<int>& gridelements,size_t maxContacts)
{
  GeometricPrimitive3D alocal=a;
  RigidTransform Tbinv; Tbinv.setInverse(Tb);
  alocal.Transform(Tbinv);
  return Collides(b,alocal,margin,gridelements,maxContacts);
}

bool Collides(const GeometricPrimitive3D& a,const CollisionMesh& c,Real margin,
	      vector<int>& meshelements,size_t maxContacts)
{
  NearbyTriangles(c,a,margin,meshelements,maxContacts);
  return !meshelements.empty();
}

bool Collides(const GeometricPrimitive3D& a,const CollisionPointCloud& b,Real margin,vector<int>& pcelements,size_t maxContacts)
{
  NearbyPoints(b,a,margin,pcelements,maxContacts);
  return !pcelements.empty();
}





bool Collides(const Meshing::VolumeGrid& a,const RigidTransform& Ta,const Meshing::VolumeGrid& b,const RigidTransform& Tb,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  FatalError("Volume grid to volume grid collisions not done\n");
  return false;
}

bool Collides(const Meshing::VolumeGrid& a,const RigidTransform& Ta,const CollisionMesh& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  FatalError("Volume grid to triangle mesh collisions not done\n");
  return false;
}

bool Collides(const CollisionMesh& a,const CollisionMesh& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  NearbyTriangles(a,b,margin,elements1,elements2,maxContacts);
  return !elements1.empty();
}


bool Collides(const GeometricPrimitive3D& a,const RigidTransform& Ta,Real margin,const AnyCollisionGeometry3D& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  if(a.type == GeometricPrimitive3D::Empty) return false;
  GeometricPrimitive3D aw=a;
  aw.Transform(Ta);
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      if(::Collides(aw,bw,margin+b.margin)) {
	elements1.push_back(0);
	elements2.push_back(0);
	return true;
      }
      return false;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    if(::Collides(aw,b.AsImplicitSurface(),b.GetTransform(),margin+b.margin,elements2,maxContacts)) {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::TriangleMesh:
    if(::Collides(aw,b.TriangleMeshCollisionData(),margin+b.margin,elements2,maxContacts)) {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::PointCloud:
    if(::Collides(aw,b.PointCloudCollisionData(),margin+b.margin,elements2,maxContacts)) {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      elements1.resize(0);
      elements2.resize(0);
      for(size_t i=0;i<bitems.size();i++) {
	vector<int> e1,e2;
	if(Collides(a,Ta,margin+b.margin,bitems[i],e1,e2,maxContacts)) {
	  for(size_t j=0;j<e1.size();j++) {
	    elements1.push_back(e1[j]);
	    elements2.push_back((int)i);
	  }
	  if(elements2.size() >= maxContacts) return true;
	}
      }
      return !elements1.empty();
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}


bool Collides(const Meshing::VolumeGrid& a,const RigidTransform& Ta,Real margin,const AnyCollisionGeometry3D& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      if(::Collides(bw,a,Ta,margin+b.margin,elements1,maxContacts)) {
	elements2.push_back(0);
	return true;
      }
      return false;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    return ::Collides(a,Ta,b.AsImplicitSurface(),b.GetTransform(),margin+b.margin,elements1,elements2,maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(a,Ta,b.TriangleMeshCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
    FatalError("Point cloud testing should be prioritized");
    break;
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      elements1.resize(0);
      elements2.resize(0);
      for(size_t i=0;i<bitems.size();i++) {
	vector<int> e1,e2;
	if(Collides(a,Ta,margin+b.margin,bitems[i],e1,e2,maxContacts)) {
	  for(size_t j=0;j<e1.size();j++) {
	    elements1.push_back(e1[j]);
	    elements2.push_back((int)i);
	  }
	  if(elements2.size() >= maxContacts) return true;
	}
      }
      return !elements1.empty();
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const CollisionMesh& a,Real margin,const AnyCollisionGeometry3D& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      if(::Collides(bw,a,margin+b.margin,elements1,maxContacts)) {
	elements2.push_back(0);
	return true;
      }
      return false;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    return ::Collides(b.AsImplicitSurface(),b.GetTransform(),a,margin+b.margin,elements2,elements1,maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(a,b.TriangleMeshCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
    FatalError("Point cloud testing should be prioritized");
    break;
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      elements1.resize(0);
      elements2.resize(0);
      for(size_t i=0;i<bitems.size();i++) {
	vector<int> e1,e2;
	if(Collides(a,margin+b.margin,bitems[i],e1,e2,maxContacts)) {
	  for(size_t j=0;j<e1.size();j++) {
	    elements1.push_back(e1[j]);
	    elements2.push_back((int)i);
	  }
	  if(elements2.size() >= maxContacts) return true;
	}
      }
      return !elements1.empty();
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}


static Real gWithinDistanceMargin = 0;
static const CollisionPointCloud* gWithinDistancePC = NULL;
static const AnyCollisionGeometry3D* gWithinDistanceGeom = NULL;
static vector<int>* gWithinDistanceElements1 = NULL;
static vector<int>* gWithinDistanceElements2 = NULL;
static size_t gWithinDistanceMaxContacts = 0;
static GeometricPrimitive3D point_primitive(Vector3(0.0));
bool withinDistance_PC_AnyGeom(void* obj)
{
  Point3D* p = reinterpret_cast<Point3D*>(obj);
  Vector3 pw = gWithinDistancePC->currentTransform*(*p);
  RigidTransform Tident; Tident.R.setIdentity(); Tident.t = pw;
  vector<int> temp;
  if(Collides(point_primitive,Tident,gWithinDistanceMargin,*gWithinDistanceGeom,temp,*gWithinDistanceElements1,gWithinDistanceMaxContacts)) {
    printf("Colliding point %g %g %g distance %g\n",pw.x,pw.y,pw.z,gWithinDistanceGeom->Distance(pw));
    gWithinDistanceElements2->push_back(p-&gWithinDistancePC->points[0]);
    if(gWithinDistanceElements1->size() >= gWithinDistanceMaxContacts) 
      return false;
  }
  return true;
}

bool Collides(const CollisionPointCloud& a,Real margin,const AnyCollisionGeometry3D& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      if(::Collides(bw,a,margin+b.margin,elements1,maxContacts)) {
	elements2.push_back(0);
	return true;
      }
      return false;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
  case AnyCollisionGeometry3D::TriangleMesh:
  case AnyCollisionGeometry3D::PointCloud:
    {
      Box3D abb;
      GetBB(a,abb);
      Box3D bbb = b.GetBB();
      Box3D bbbexpanded = bbb;
      bbbexpanded.dims += Vector3(margin*2.0);
      bbbexpanded.origin -= margin*(bbb.xbasis+bbb.ybasis+bbb.zbasis);
      //quick reject test
      if(!abb.intersectsApprox(bbbexpanded)) return false;
      RigidTransform Tw_a;
      Tw_a.setInverse(a.currentTransform);
      Box3D bbb_a;
      bbb_a.setTransformed(bbbexpanded,Tw_a);
      AABB3D baabb_a;
      bbb_a.getAABB(baabb_a);
      baabb_a.setIntersection(a.bblocal);

      //octree-vs b's bounding box method (not terribly smart, could be
      //improved by descending bounding box hierarchies)
      Timer timer;
      vector<Vector3> apoints;
      vector<int> aids;
      a.octree->BoxQuery(bbb_a,apoints,aids);
      RigidTransform Tident; Tident.setIdentity();
      //test all points, linearly
      for(size_t i=0;i<apoints.size();i++) {
	Vector3 p_w = a.currentTransform*apoints[i];
	Tident.t = p_w;
	vector<int> temp;
	if(Collides(point_primitive,Tident,margin,b,temp,elements1,maxContacts)) {
	  elements2.push_back(i);
	  if(elements2.size() >= maxContacts) {
	    printf("Collision in time %g\n",timer.ElapsedTime());
	    return true;
	  }
	}
      }
      printf("No collision in time %g\n",timer.ElapsedTime());
      return false;

      /*
      //grid method
      GridSubdivision::Index imin,imax;
      a.grid.PointToIndex(Vector(3,baabb_a.bmin),imin);
      a.grid.PointToIndex(Vector(3,baabb_a.bmax),imax);
      int numCells = (imax[0]-imin[0]+1)*(imax[1]-imin[1]+1)*(imax[2]-imin[2]+1);
      Timer timer;
      if(numCells >(int) a.points.size()) {
	RigidTransform Tident; Tident.setIdentity();
	//test all points, linearly
	for(size_t i=0;i<a.points.size();i++) {
	  Vector3 p_w = a.currentTransform*a.points[i];
	  Tident.t = p_w;
	  vector<int> temp;
	  if(Collides(point_primitive,Tident,margin,b,temp,elements1,maxContacts)) {
	    elements2.push_back(i);
	    if(elements2.size() >= maxContacts) {
	      printf("Collision in time %g\n",timer.ElapsedTime());
	      return true;
	    }
	  }
	}
	printf("No collision in time %g\n",timer.ElapsedTime());
	return false;
      }
      else {
	printf("Box testing\n");
	gWithinDistanceMargin = margin;
	gWithinDistancePC = &a;
	gWithinDistanceGeom = &b;
	gWithinDistanceElements1 = &elements1;
	gWithinDistanceElements2 = &elements2;
	gWithinDistanceMaxContacts = maxContacts;
	bool collisionFree = a.grid.IndexQuery(imin,imax,withinDistance_PC_AnyGeom);
	if(collisionFree) printf("No collision in time %g\n",timer.ElapsedTime());
	else printf("Collision in time %g\n",timer.ElapsedTime());
	return !collisionFree;
      }
      */
    }
    return false;
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      elements1.resize(0);
      elements2.resize(0);
      for(size_t i=0;i<bitems.size();i++) {
	vector<int> e1,e2;
	if(Collides(a,margin+b.margin,bitems[i],e1,e2,maxContacts)) {
	  for(size_t j=0;j<e1.size();j++) {
	    elements1.push_back(e1[j]);
	    elements2.push_back((int)i);
	  }
	  if(elements2.size() >= maxContacts) return true;
	}
      }
      return !elements1.empty();
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const vector<AnyCollisionGeometry3D>& group,Real margin,const AnyCollisionGeometry3D& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  for(size_t i=0;i<group.size();i++) {
    vector<int> ei1,ei2;
    if(group[i].WithinDistance(b,margin,ei1,ei2,maxContacts-(int)elements1.size())) {
      for(size_t j=0;j<ei1.size();j++) {
	elements1.push_back((int)i);
	elements2.push_back((int)ei2[j]);
      }
      if(elements2.size()>=maxContacts) return true;
    }
  }
  return !elements2.empty();
}


bool AnyCollisionGeometry3D::Collides(const AnyCollisionGeometry3D& geom) const
{
  vector<int> elem1,elem2;
  return Collides(geom,elem1,elem2,1);
}

bool AnyCollisionGeometry3D::Collides(const AnyCollisionGeometry3D& geom,
				      vector<int>& elements1,vector<int>& elements2,size_t maxContacts) const
{
  //prioritize point cloud testing
  if(geom.type == PointCloud && type != PointCloud)
    return geom.Collides(*this,elements2,elements1,maxContacts);
  //otherwise...
  switch(type) {
  case Primitive:
    return ::Collides(AsPrimitive(),GetTransform(),margin,geom,elements1,elements2,maxContacts);
  case ImplicitSurface:
    return ::Collides(AsImplicitSurface(),GetTransform(),margin,geom,elements1,elements2,maxContacts);
  case TriangleMesh:
    return ::Collides(TriangleMeshCollisionData(),margin,geom,elements1,elements2,maxContacts);
  case PointCloud:
    return ::Collides(PointCloudCollisionData(),margin,geom,elements1,elements2,maxContacts);
  case Group:
    return ::Collides(GroupCollisionData(),margin,geom,elements1,elements2,maxContacts);
  default:
    FatalError("Invalid type");
  }
  return false;
}

Real AnyCollisionGeometry3D::Distance(const AnyCollisionGeometry3D& geom) const
{
  int elem1,elem2;
  return Distance(geom,elem1,elem2);
}

Real AnyCollisionGeometry3D::Distance(const AnyCollisionGeometry3D& geom,int& elem1,int& elem2) const
{
  FatalError("Distance not implemented yet\n");
  return Inf;
}

bool AnyCollisionGeometry3D::WithinDistance(const AnyCollisionGeometry3D& geom,Real tol) const
{
  vector<int> elem1,elem2;
  return WithinDistance(geom,tol,elem1,elem2,1);
}

bool AnyCollisionGeometry3D::WithinDistance(const AnyCollisionGeometry3D& geom,Real tol,
					    vector<int>& elements1,vector<int>& elements2,size_t maxContacts) const
{
  switch(type) {
  case Primitive:
    return ::Collides(AsPrimitive(),GetTransform(),margin+tol,geom,elements1,elements2,maxContacts);
  case ImplicitSurface:
    return ::Collides(AsImplicitSurface(),GetTransform(),margin+tol,geom,elements1,elements2,maxContacts);
  case TriangleMesh:
    return ::Collides(TriangleMeshCollisionData(),margin+tol,geom,elements1,elements2,maxContacts);
  case PointCloud:
    return ::Collides(PointCloudCollisionData(),margin+tol,geom,elements1,elements2,maxContacts);
  case Group:
    return ::Collides(GroupCollisionData(),margin+tol,geom,elements1,elements2,maxContacts);
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool AnyCollisionGeometry3D::RayCast(const Ray3D& r,Real* distance,int* element) const
{
  switch(type) {
  case Primitive:
    {
      RigidTransform T=PrimitiveCollisionData(),Tinv;
      Tinv.setInverse(T);
      Ray3D rlocal; rlocal.setTransformed(r,Tinv);
      Vector3 localpt;
      if(AsPrimitive().RayCast(rlocal,localpt)) {
	if(distance) {
	  *distance = localpt.distance(rlocal.source);
	  //TODO: this isn't perfect if the margin is > 0 -- will miss silouettes
	  *distance -= margin;
	}
	if(element) *element = 0;
	return true;
      }
      return false;
    }
    break;
  case ImplicitSurface:
    FatalError("Can't ray-cast implicit surfaces yet\n");
    break;
  case TriangleMesh:
    {
      Vector3 worldpt;
      int tri = ::RayCast(TriangleMeshCollisionData(),r,worldpt);
      if(tri >= 0) {
	if(distance) {
	  *distance = worldpt.distance(r.source);
	  //TODO: this isn't perfect if the margin is > 0 -- will miss silhouettes
	  *distance -= margin;
	}
	if(element) *element = tri;
	return true;
      }
      return false;
    }
  case PointCloud:
    {
      const CollisionPointCloud& pc = PointCloudCollisionData();
      Vector3 pt;
      int res = ::RayCast(pc,margin,r,pt);
      if(res < 0) return false;
      if(distance) {
	Vector3 temp;
	*distance = r.closestPoint(pt,temp);
      }
      if(element) *element = res;
      return true;
    }
  case Group:
    {
      const vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      Real closest = Inf;
      for(size_t i=0;i<items.size();i++) {
	Real d;
	int elem;
	if(items[i].RayCast(r,&d,&elem)) {
	  if(d < closest) {
	    closest = d;
	    if(element) *element = (int)i;
	  }
	}
      }
      if(distance) *distance = closest;
      return !IsInf(closest);
    }
  }  
  return false;
}

AnyCollisionQuery::AnyCollisionQuery()
  :a(NULL),b(NULL)
{}

AnyCollisionQuery::AnyCollisionQuery(const AnyCollisionGeometry3D& _a,const AnyCollisionGeometry3D& _b)
  :a(&_a),b(&_b)
{
  if(a->type == AnyGeometry3D::TriangleMesh && b->type == AnyGeometry3D::TriangleMesh) {
    qmesh = CollisionMeshQueryEnhanced(a->TriangleMeshCollisionData(),b->TriangleMeshCollisionData());
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
  }
}

AnyCollisionQuery::AnyCollisionQuery(const AnyCollisionQuery& q)
  :a(q.a),b(q.b),qmesh(q.qmesh)
{}

bool AnyCollisionQuery::Collide()
{
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(qmesh.m1) {
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
    if(qmesh.Collide()) {
      qmesh.CollisionPairs(elements1,elements2);
      return true;
    }
    return false;
  }
  return a->Collides(*b,elements1,elements2,1);
}

bool AnyCollisionQuery::CollideAll()
{
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(qmesh.m1) {
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
    if(qmesh.CollideAll()) {
      qmesh.CollisionPairs(elements1,elements2);
      return true;
    }
    return false;
  }
  return a->Collides(*b,elements1,elements2);
}

bool AnyCollisionQuery::WithinDistance(Real d)
{
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(qmesh.m1) {
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
    if(qmesh.WithinDistance(d)) {
      elements1.resize(1);
      elements2.resize(1);
      points1.resize(1);
      points2.resize(1);
      qmesh.TolerancePair(elements1[0],elements2[0]);
      qmesh.TolerancePoints(points1[0],points2[0]);
      return true;
    }
    return false;
  }
  return a->WithinDistance(*b,d,elements1,elements2,1);
}

bool AnyCollisionQuery::WithinDistanceAll(Real d)
{
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(qmesh.m1) {
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
    if(qmesh.WithinDistanceAll(d)) {
      qmesh.TolerancePairs(elements1,elements2);
      qmesh.TolerancePoints(points1,points2);
      return true;
    }
  }
  return a->WithinDistance(*b,d,elements1,elements2);
}

Real AnyCollisionQuery::PenetrationDepth()
{
  if(qmesh.m1) {
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
    return qmesh.PenetrationDepth();
  }
  return -a->Distance(*b);
}

Real AnyCollisionQuery::Distance(Real absErr,Real relErr,Real bound)
{
  elements1.resize(1);
  elements2.resize(1);
  points1.resize(0);
  points2.resize(0);
  if(qmesh.m1) {
    qmesh.margin1 = a->margin;
    qmesh.margin2 = b->margin;
    points1.resize(1);
    points2.resize(1);
    Real res = qmesh.Distance(absErr,relErr,bound);
    qmesh.ClosestPair(elements1[0],elements2[0]);
    qmesh.ClosestPoints(points1[0],points2[0]);
    return res;
  }
  return a->Distance(*b,elements1[0],elements2[0]);
}

void AnyCollisionQuery::InteractingPairs(std::vector<int>& t1,std::vector<int>& t2) const
{
  t1 = elements1;
  t2 = elements2;
}

void AnyCollisionQuery::InteractingPoints(std::vector<Vector3>& p1,std::vector<Vector3>& p2) const
{
  if(points1.empty() && !elements1.empty()) {
    //need to compute points from elements
    FatalError("TODO: compute interacting points from interacting elements\n");
  }
  else {
    p1 = points1;
    p2 = points2;
  }
}

