#include "AnyGeometry.h"
#include <utils/stringutils.h>
#include <meshing/IO.h>
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

AnyGeometry3D::AnyGeometry3D(const AnyGeometry3D& geom)
  :type(geom.type),data(geom.data)
{}

const char* AnyGeometry3D::TypeName(Type type)
{
  switch(type) { 
  case Primitive: return "Primitive";
  case TriangleMesh: return "TriangleMesh";
  case PointCloud: return "PointCloud";
  case ImplicitSurface: return "ImplicitSurface";
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
    for(size_t i=0;i<nonempty.size();i++)
      if(geoms[nonempty[i]].type != type) 
	FatalError("Can't mix multiple geometry types together yet\n");
    switch(type) {
    case Primitive:
      FatalError("Can't mix multiple primitive geometries together yet\n");
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
      FatalError("Can't mix multiple point clouds together yet\n");
      break;
    case ImplicitSurface:
      FatalError("Can't mix multiple volume grids together yet\n");
      break;
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
    FatalError("Can't load geometric primitives yet");
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
  if(!in) return false;
  return true;
}

bool AnyGeometry3D::Save(ostream& out) const
{
  out<<TypeName()<<endl;
  switch(type) {
  case Primitive:
    FatalError("Can't save geometric primitives yet");
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
      return gT.GetAABB();
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
  }
  AssertNotReached();
  AABB3D bb;
  return bb;
}

Box3D AnyCollisionGeometry3D::GetBB() const
{
  Box3D b;
  if(collisionData.empty()) {
    if(type == Primitive) 
      return AsPrimitive().GetBB();
    b.set(AnyGeometry3D::GetAABB());
    return b;
  }
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
      Real dmin = Inf;
      for(size_t i=0;i<pc.points.size();i++)
	dmin = Min(dmin,ptlocal.distanceSquared(pc.points[i]));
      return Min(Sqrt(dmin)-margin,0.0);
    }
  }
  return Inf;
}



bool Collides(const Meshing::VolumeGrid& grid,const GeometricPrimitive3D& a,Real margin,
	      vector<int>& gridelements,size_t maxContacts)
{
  if(a.type != GeometricPrimitive3D::Point && a.type != GeometricPrimitive3D::Sphere) {
    FatalError("Can't collide an implicit surface and a primitive yet\n");
  }
  if(a.type == GeometricPrimitive3D::Point) {
    const Vector3& pt = *AnyCast<Vector3>(&a.data);
    bool res = (grid.TrilinearInterpolate(pt) <= margin);
    if(res) {
      gridelements.resize(1);
      IntTriple cell;
      grid.GetIndex(pt,cell);
      gridelements[0] = cell.a*grid.value.n*grid.value.p + cell.b*grid.value.p + cell.c;
    }
    return res;
  }
  const Sphere3D* s=AnyCast<Sphere3D>(&a.data);
  bool res = (grid.TrilinearInterpolate(s->center) <= margin+s->radius);  
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

bool Collides(const Meshing::VolumeGrid& a,const RigidTransform& Ta,const CollisionPointCloud& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  RigidTransform btoa; btoa.mulInverseA(Ta,b.currentTransform);
  for(size_t i=0;i<b.points.size();i++) {
    if(a.TrilinearInterpolate(btoa*b.points[i]) <= margin) {
      IntTriple cell;
      a.GetIndex(b.points[i],cell);
      elements1.push_back(cell.a*a.value.n*a.value.p + cell.b*a.value.p + cell.c);
      elements2.push_back(i);
      if(elements1.size() >= maxContacts) return true;
    }
  }
  return false;
}

bool Collides(const CollisionMesh& a,const CollisionMesh& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  NearbyTriangles(a,b,margin,elements1,elements2,maxContacts);
  return !elements1.empty();
}

bool Collides(const CollisionMesh& a,const CollisionPointCloud& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  Box3D abb,bbb;
  GetBB(a,abb);
  GetBB(b,bbb);
  if(!abb.intersects(bbb)) return false;
  for(size_t i=0;i<b.points.size();i++) {
    vector<int> nearTris;
    NearbyTriangles(a,b.currentTransform*b.points[i],margin,nearTris,maxContacts-elements1.size());
    for(size_t k=0;k<nearTris.size();k++) {
      elements1.push_back(nearTris[k]);
      elements2.push_back(i);
    }
    if(elements1.size()>=maxContacts) return true;
  }
  return false;
}

bool Collides(const CollisionPointCloud& a,const CollisionPointCloud& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  Box3D abb,bbb;
  GetBB(a,abb);
  GetBB(b,bbb);
  if(!abb.intersects(bbb)) return false;
  FatalError("Can't yet do point cloud-point cloud collision detection");
  return false;
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
    return ::Collides(a,Ta,b.PointCloudCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
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
    return ::Collides(a,b.PointCloudCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  default:
    FatalError("Invalid type");
  }
  return false;
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
    return ::Collides(b.AsImplicitSurface(),b.GetTransform(),a,margin+b.margin,elements2,elements1,maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(b.TriangleMeshCollisionData(),a,margin+b.margin,elements2,elements1,maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
    return ::Collides(a,b.PointCloudCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  default:
    FatalError("Invalid type");
  }
  return false;
}


bool AnyCollisionGeometry3D::Collides(const AnyCollisionGeometry3D& geom) const
{
  vector<int> elem1,elem2;
  return Collides(geom,elem1,elem2,1);
}

bool AnyCollisionGeometry3D::Collides(const AnyCollisionGeometry3D& geom,
				      vector<int>& elements1,vector<int>& elements2,size_t maxContacts) const
{
  switch(type) {
  case Primitive:
    return ::Collides(AsPrimitive(),GetTransform(),margin,geom,elements1,elements2,maxContacts);
  case ImplicitSurface:
    return ::Collides(AsImplicitSurface(),GetTransform(),margin,geom,elements1,elements2,maxContacts);
  case TriangleMesh:
    return ::Collides(TriangleMeshCollisionData(),margin,geom,elements1,elements2,maxContacts);
  case PointCloud:
    return ::Collides(PointCloudCollisionData(),margin,geom,elements1,elements2,maxContacts);
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
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool AnyCollisionGeometry3D::RayCast(const Ray3D& r,Real* distance,int* element) const
{
  switch(type) {
  case Primitive:
    FatalError("Can't ray-cast primitives yet\n");
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
      Ray3D rlocal;
      pc.currentTransform.mulInverse(r.source,rlocal.source);
      pc.currentTransform.R.mulTranspose(r.direction,rlocal.direction);
      Real closest = Inf;
      int closestpt = -1;
      Sphere3D s;
      s.radius = margin;
      Vector3 pr;
      for(size_t i=0;i<pc.points.size();i++) {
	s.center = pc.points[i];
	Real tmin,tmax;
	if(s.intersects(r,&tmin,&tmax)) {
	  if(tmax >= 0 && tmin < closest) {
	    closest = Max(tmin,0.0);
	    closestpt = i;
	  }
	}
      }
      if(distance) *distance = closest;
      if(element) *element = closestpt;
      return closestpt >= 0;
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
  if(qmesh.m1) return qmesh.PenetrationDepth();
  return -a->Distance(*b);
}

Real AnyCollisionQuery::Distance(Real absErr,Real relErr,Real bound)
{
  elements1.resize(1);
  elements2.resize(1);
  points1.resize(0);
  points2.resize(0);
  if(qmesh.m1) {
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

