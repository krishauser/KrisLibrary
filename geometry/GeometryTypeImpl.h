#ifndef ANY_GEOMETRY_TYPE_IMPL_H
#define ANY_GEOMETRY_TYPE_IMPL_H

#include "GeometryType.h"
#include "ConvexHull3D.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/Heightmap.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>

namespace Geometry {

void Flip(DistanceQueryResult &res);
void Transform1(DistanceQueryResult &res, const RigidTransform &T);
void Transform2(DistanceQueryResult &res, const RigidTransform &T);
void Offset1(DistanceQueryResult &res, Real offset);
void Offset2(DistanceQueryResult &res, Real offset);
void Offset(DistanceQueryResult &res, Real offset1, Real offset2);
void SetCP2(DistanceQueryResult &res);
void PushGroup1(DistanceQueryResult &res, int i);
void PushGroup2(DistanceQueryResult &res, int i);
void ReverseContact(ContactsQueryResult::ContactPair &contact);

class Geometry3DPrimitive : public Geometry3D
{
public:
  Geometry3DPrimitive();
  Geometry3DPrimitive(const GeometricPrimitive3D& _data);
  Geometry3DPrimitive(GeometricPrimitive3D&& _data);
  virtual ~Geometry3DPrimitive ();
  virtual Type GetType() const override { return Type::Primitive; }
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override { return data.type == GeometricPrimitive3D::Empty; }
  virtual size_t NumElements() const override { return 1; }
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override { return shared_ptr<Geometry3D>(new Geometry3DPrimitive(data)); }
  virtual bool Transform(const Matrix4& mat) override;
  virtual Geometry3D* Copy() const override { return new Geometry3DPrimitive(data); }
  virtual AABB3D GetAABB() const override;
  virtual bool Support(const Vector3& dir,Vector3& pt) const override;
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;

  GeometricPrimitive3D data;
};

class Geometry3DTriangleMesh : public Geometry3D
{
public:
  Geometry3DTriangleMesh();
  Geometry3DTriangleMesh(const Meshing::TriMesh& data);
  Geometry3DTriangleMesh(const Meshing::TriMesh& data,shared_ptr<GLDraw::GeometryAppearance> appearance);
  Geometry3DTriangleMesh(Meshing::TriMesh&& data);
  virtual ~Geometry3DTriangleMesh ();
  virtual Type GetType() const override { return Type::TriangleMesh; }
  virtual const char* FileExtension() const override { return "off"; }
  virtual vector<string> FileExtensions() const override;
  virtual bool Load(const char *fn) override;
  virtual bool Save(const char *fn) const override;
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override { return data.tris.empty(); }
  virtual size_t NumElements() const override { return data.tris.size(); }
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override;
  virtual bool Transform(const Matrix4& mat) override;
  virtual Geometry3D* Copy() const override { return new Geometry3DTriangleMesh(data,appearance); }
  virtual AABB3D GetAABB() const override;
  virtual bool Support(const Vector3& dir,Vector3& pt) const override;
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) override;
  virtual Geometry3D* Remesh(Real resolution,bool refine=true,bool coarsen=true) const override;
  virtual bool Union(const vector<Geometry3D*>& geoms) override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;
  virtual Geometry3D* Slice(const RigidTransform& T,Real tol=0) const override;
  virtual Geometry3D* ExtractROI(const AABB3D& bb,int flag=1) const override;
  virtual Geometry3D* ExtractROI(const Box3D& bb,int flag=1) const override;

  Meshing::TriMesh data;
  shared_ptr<GLDraw::GeometryAppearance> appearance;
};

class Geometry3DPointCloud : public Geometry3D
{
public:
  Geometry3DPointCloud();
  Geometry3DPointCloud(const Meshing::PointCloud3D& data);
  Geometry3DPointCloud(Meshing::PointCloud3D&& data);
  virtual ~Geometry3DPointCloud ();
  virtual Type GetType() const override { return Type::PointCloud; }
  virtual const char* FileExtension() const override { return "pcd"; }
  virtual bool Load(const char *fn) override;
  virtual bool Save(const char *fn) const override;
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override { return data.points.empty(); }
  virtual size_t NumElements() const override { return data.points.size(); }
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override;
  virtual bool Transform(const Matrix4& mat) override;
  virtual Geometry3D* Copy() const override { return new Geometry3DPointCloud(data); }
  virtual AABB3D GetAABB() const override;
  virtual bool Support(const Vector3& dir,Vector3& pt) const override;
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) override;
  virtual Geometry3D* Remesh(Real resolution,bool refine=true,bool coarsen=true) const override;
  virtual bool Union(const vector<Geometry3D*>& geoms) override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;
  virtual Geometry3D* Slice(const RigidTransform& T,Real tol=0) const override;
  virtual Geometry3D* ExtractROI(const AABB3D& bb,int flag=1) const override;
  virtual Geometry3D* ExtractROI(const Box3D& bb,int flag=1) const override;

  Meshing::PointCloud3D data;
};

class Geometry3DConvexHull : public Geometry3D
{
public:
  Geometry3DConvexHull() {}
  Geometry3DConvexHull(const ConvexHull3D& _data) : data(_data) {}
  virtual ~Geometry3DConvexHull () {}
  virtual Type GetType() const { return Type::ConvexHull; }
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override;
  virtual size_t NumElements() const override;
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override;
  virtual bool Transform(const Matrix4& mat) override;
  virtual Geometry3D* Copy() const override { return new Geometry3DConvexHull(data); }
  virtual AABB3D GetAABB() const override;
  virtual bool Support(const Vector3& dir,Vector3& pt) const override;
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) override;
  virtual bool Union(const vector<Geometry3D*>& geoms) override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;

  ConvexHull3D data;
};

class Geometry3DVolume : public Geometry3D
{
public:
  Geometry3DVolume() {}
  Geometry3DVolume(const Meshing::VolumeGrid& _data) : data(_data) {}
  Geometry3DVolume(Meshing::VolumeGrid&& _data) : data(_data) {}
  virtual ~Geometry3DVolume () {}
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override;
  virtual size_t NumElements() const override;
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override;
  virtual AABB3D GetAABB() const override;
  virtual bool Transform(const Matrix4& mat) override;
  void ResizeTo(const Geometry3D* geom,Real resolution,Real domainExpansion=0);
  int IndexToElement(const IntTriple& idx) const;
  IntTriple ElementToIndex(int elem) const;

  Meshing::VolumeGrid data;
};

class Geometry3DImplicitSurface : public Geometry3DVolume
{
public:
  Geometry3DImplicitSurface(Real _truncation = Inf);
  Geometry3DImplicitSurface(const Meshing::VolumeGrid& _data, Real _truncation = Inf);
  Geometry3DImplicitSurface(Meshing::VolumeGrid&& _data, Real _truncation = Inf);
  virtual ~Geometry3DImplicitSurface () {}
  virtual Type GetType() const override { return Type::ImplicitSurface; }
  virtual Geometry3D* Copy() const override { return new Geometry3DImplicitSurface(data); }
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) override;
  virtual Geometry3D* Remesh(Real resolution,bool refine,bool coarsen) const override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;

  Real truncationDistance;
};

class Geometry3DOccupancyGrid : public Geometry3DVolume
{
public:
  Geometry3DOccupancyGrid(Real _threshold = 0.5);
  Geometry3DOccupancyGrid(const Meshing::VolumeGrid& _data,Real _threshold = 0.5);
  Geometry3DOccupancyGrid(Meshing::VolumeGrid&& _data,Real _threshold = 0.5);
  virtual ~Geometry3DOccupancyGrid () {}
  virtual Type GetType() const override { return Type::OccupancyGrid; }
  virtual Geometry3D* Copy() const override { return new Geometry3DOccupancyGrid(data); }
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) override;
  virtual Geometry3D* Remesh(Real resolution,bool refine,bool coarsen) const override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;

  Real occupancyThreshold;
};

class Geometry3DHeightmap : public Geometry3D
{
public:
  Geometry3DHeightmap() {}
  Geometry3DHeightmap(const Meshing::Heightmap& _data) : data(_data) {}
  Geometry3DHeightmap(Meshing::Heightmap&& _data) : data(_data) {}
  virtual ~Geometry3DHeightmap () {}
  virtual Type GetType() const override { return Type::Heightmap; }
  virtual Geometry3D* Copy() const override { return new Geometry3DHeightmap(data); }
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override;
  virtual size_t NumElements() const override;
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override;
  virtual AABB3D GetAABB() const override;
  virtual bool Transform(const Matrix4& mat) override;
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) override;
  virtual Geometry3D* Remesh(Real resolution,bool refine,bool coarsen) const override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;

  Meshing::Heightmap data;
};

class Geometry3DGroup : public Geometry3D
{
public:
  Geometry3DGroup();
  Geometry3DGroup(const vector<AnyGeometry3D>& _data);
  virtual ~Geometry3DGroup ();
  virtual Type GetType() const override { return Type::Group; }
  virtual bool Load(istream& in) override;
  virtual bool Save(ostream& out) const override;
  virtual bool Empty() const override;
  virtual size_t NumElements() const override;
  virtual shared_ptr<Geometry3D> GetElement(int elem) const override;
  virtual bool Transform(const Matrix4& mat) override;
  virtual Geometry3D* Copy() const override { return new Geometry3DGroup(data); }
  virtual AABB3D GetAABB() const override;
  virtual bool Support(const Vector3& dir,Vector3& pt) const override;
  virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const override;
  virtual Geometry3D* Remesh(Real resolution,bool refine=true,bool coarsen=true) const override;
  virtual bool Union(const vector<Geometry3D*>& geoms) override;
  virtual bool Merge(const Geometry3D* geom,const RigidTransform* Tgeom=NULL) override;
  virtual Geometry3D* Slice(const RigidTransform& T,Real tol=0) const override;
  virtual Geometry3D* ExtractROI(const AABB3D& bb,int flag=1) const override;
  virtual Geometry3D* ExtractROI(const Box3D& bb,int flag=1) const override;

  vector<AnyGeometry3D> data;
};

class Collider3DGroup : public Collider3D
{
public:
  Collider3DGroup(shared_ptr<Geometry3DGroup> data);
  virtual ~Collider3DGroup();
  virtual shared_ptr<Geometry3D> GetData() const override { return dynamic_pointer_cast<Geometry3D>(data); }
  virtual void Reset() override;
  virtual RigidTransform GetTransform() const override;
  virtual void SetTransform(const RigidTransform& T) override;
  virtual Collider3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) override;
  virtual bool Union(const vector<Collider3D*>& geoms);
  virtual bool Merge(const Collider3D* geom);
  virtual AABB3D GetAABB() const override;
  virtual AABB3D GetAABBTight() const override;
  virtual Box3D GetBB() const override;
  virtual Collider3D* Slice(const RigidTransform& T,Real tol=0) const override;
  virtual Collider3D* ExtractROI(const AABB3D& bb,int flag=1) const override;
  virtual Collider3D* ExtractROI(const Box3D& bb,int flag=1) const override;
  virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
  virtual bool Contains(const Vector3& pt,bool& result) override;
  virtual bool Distance(const Vector3& pt,Real& result) override;
  virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
  virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res) override;
  virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) override;
  virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) override;
  virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) override;

  shared_ptr<Geometry3DGroup> data;
  vector<AnyCollisionGeometry3D> collisionData;
};


} //namespace Geometry

#endif //ANY_GEOMETRY_TYPE_IMPL_H

