#include <KrisLibrary/Logger.h>
#include "AnyGeometry.h"
#include "Conversions.h"
#include <math3d/geometry3d.h>
#include <meshing/VolumeGrid.h>
#include <meshing/Voxelize.h>
#include <GLdraw/GeometryAppearance.h>
#include "CollisionPointCloud.h"
#include "CollisionImplicitSurface.h"
#include <utils/stringutils.h>
#include <meshing/IO.h>
#include <Timer.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>


#include "PQP/include/PQP.h"


using namespace Geometry;

void Flip(AnyDistanceQueryResult& res)
{
  std::swap(res.elem1,res.elem2);
  std::swap(res.group_elem1,res.group_elem2);
  std::swap(res.cp1,res.cp2);
  std::swap(res.dir1,res.dir2);
}

void Transform1(AnyDistanceQueryResult& res,const RigidTransform& T)
{
  if(res.hasClosestPoints) res.cp1 = T*res.cp1;
  if(res.hasDirections) res.dir1 = T.R*res.dir1;
}

void Transform2(AnyDistanceQueryResult& res,const RigidTransform& T)
{
  if(res.hasClosestPoints) res.cp2 = T*res.cp2;
  if(res.hasDirections) res.dir2 = T.R*res.dir2;
}

void Offset1(AnyDistanceQueryResult& res,Real offset)
{
  res.d -= offset;
  if(res.hasDirections)
    res.cp1.madd(res.dir1,offset);
}

void Offset2(AnyDistanceQueryResult& res,Real offset)
{
  res.d -= offset;
  if(res.hasDirections)
    res.cp2.madd(res.dir2,offset);
}

void SetCP2(AnyDistanceQueryResult& res)
{
  res.cp2 = res.cp1;
  res.cp2.madd(res.dir1,res.d);
  res.dir2.setNegative(res.dir1);
}

void PushGroup1(AnyDistanceQueryResult& res,int i)
{
  if(res.group_elem1.empty()) {
    res.group_elem1.resize(2);
    res.group_elem1[0] = i;
    res.group_elem1[1] = res.elem1;
  }
  else {
    res.group_elem1.insert(res.group_elem1.begin(),i);
  }
  res.elem1 = i;
}

void PushGroup2(AnyDistanceQueryResult& res,int i)
{
  if(res.group_elem2.empty()) {
    res.group_elem2.resize(2);
    res.group_elem2[0] = i;
    res.group_elem2[1] = res.elem2;
  }
  else {
    res.group_elem2.insert(res.group_elem2.begin(),i);
  }
  res.elem2 = i;
}

int PointIndex(const CollisionImplicitSurface& s,const Vector3& ptworld)
{
  Vector3 plocal;
  s.currentTransform.mulInverse(ptworld,plocal);
  IntTriple cell;
  s.baseGrid.GetIndex(plocal,cell);
  if(cell.a < 0) cell.a = 0;
  if(cell.a >= s.baseGrid.value.m) cell.a = s.baseGrid.value.m-1;
  if(cell.b < 0) cell.b = 0;
  if(cell.b >= s.baseGrid.value.n) cell.b = s.baseGrid.value.n-1;
  if(cell.c < 0) cell.c = 0;
  if(cell.c >= s.baseGrid.value.p) cell.c = s.baseGrid.value.p-1;
  return cell.a*s.baseGrid.value.n*s.baseGrid.value.p + cell.b*s.baseGrid.value.p + cell.c;
}

AnyDistanceQueryResult::AnyDistanceQueryResult()
:hasPenetration(0),hasElements(0),hasClosestPoints(0),hasDirections(0),d(Inf),elem1(-1),elem2(-1)
{}

AnyDistanceQuerySettings::AnyDistanceQuerySettings()
:relErr(0),absErr(0),upperBound(Inf)
{}



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



const GeometricPrimitive3D& AnyGeometry3D::AsPrimitive() const { return *AnyCast_Raw<GeometricPrimitive3D>(&data); }
const Meshing::TriMesh& AnyGeometry3D::AsTriangleMesh() const { return *AnyCast_Raw<Meshing::TriMesh>(&data); }
const Meshing::PointCloud3D& AnyGeometry3D::AsPointCloud() const { return *AnyCast_Raw<Meshing::PointCloud3D>(&data); }
const Meshing::VolumeGrid& AnyGeometry3D::AsImplicitSurface() const { return *AnyCast_Raw<Meshing::VolumeGrid>(&data); }
const vector<AnyGeometry3D>& AnyGeometry3D::AsGroup() const { return *AnyCast_Raw<vector<AnyGeometry3D> >(&data); }
GeometricPrimitive3D& AnyGeometry3D::AsPrimitive() { return *AnyCast_Raw<GeometricPrimitive3D>(&data); }
Meshing::TriMesh& AnyGeometry3D::AsTriangleMesh() { return *AnyCast_Raw<Meshing::TriMesh>(&data); }
Meshing::PointCloud3D& AnyGeometry3D::AsPointCloud() { return *AnyCast_Raw<Meshing::PointCloud3D>(&data); }
Meshing::VolumeGrid& AnyGeometry3D::AsImplicitSurface() { return *AnyCast_Raw<Meshing::VolumeGrid>(&data); }
vector<AnyGeometry3D>& AnyGeometry3D::AsGroup() { return *AnyCast_Raw<vector<AnyGeometry3D> >(&data); }

//appearance casts
GLDraw::GeometryAppearance* AnyGeometry3D::TriangleMeshAppearanceData() { return AnyCast<GLDraw::GeometryAppearance>(&appearanceData); }
const GLDraw::GeometryAppearance* AnyGeometry3D::TriangleMeshAppearanceData() const { return AnyCast<GLDraw::GeometryAppearance>(&appearanceData); }

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
    if(!geoms[i].Empty()) nonempty.push_back((int)i);
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

bool AnyGeometry3D::Convert(Type restype,AnyGeometry3D& res,double param) const
{
  if(type == restype) {
    res.type = type;
    res.data = data;
    res.appearanceData = appearanceData;
    return true;
  }
  if(restype == Group) return false;
  Assert(param >= 0);
  switch(type) {
    case Primitive:
      switch(restype) {
        case TriangleMesh:
        {
          if(param == 0) param = 16;
          else {
            AABB3D bb = GetAABB();
            Real w = (bb.bmax-bb.bmin).maxAbsElement();
            const GeometricPrimitive3D& g = AsPrimitive();
            if(g.type == GeometricPrimitive3D::Cylinder) 
              w = AnyCast_Raw<Math3D::Cylinder3D>(&g.data)->radius*2;
            param = int(w/param);
          }
          Meshing::TriMesh mesh;
          PrimitiveToMesh(AsPrimitive(),mesh,int(param));
          res = AnyGeometry3D(mesh);
          return true;
        }
        case PointCloud:
        {
          AnyGeometry3D mesh;
          Convert(TriangleMesh,mesh,param);
          Meshing::PointCloud3D pc;
          MeshToPointCloud(mesh.AsTriangleMesh(),pc);
          res = AnyGeometry3D(pc);
          return true;
        }
        case ImplicitSurface:
        {
          if(param == 0) {
            AABB3D bb = GetAABB();
            Real w = (bb.bmax-bb.bmin).maxAbsElement();
            param = w * 0.05;
          }
          Meshing::VolumeGrid grid;
          PrimitiveToImplicitSurface(AsPrimitive(),grid,param);
          res = AnyGeometry3D(grid);
          return true;
        }
        default:
          break;
      }
      break;
    case PointCloud:
      switch(restype) {
        case TriangleMesh:
        {
          if(!AsPointCloud().IsStructured()) {
            LOG4CXX_WARN(KrisLibrary::logger(),"AnyGeometry3D::Convert: Point cloud is not structured, cannot convert to triangle mesh");
            return false;
          }
          if(param == 0) param = Inf;
          Meshing::TriMesh mesh;
          PointCloudToMesh(AsPointCloud(),mesh,param);
          res = AnyGeometry3D(mesh);
          return true;
        }
        case ImplicitSurface:
        {
          return false;
        }
        default:
          break;
      }
    case TriangleMesh:
      switch(restype) {
        case PointCloud:
        {
          if(param == 0) param = Inf;
          Meshing::PointCloud3D pc;
          MeshToPointCloud(AsTriangleMesh(),pc,param);
          res = AnyGeometry3D(pc);
          return true;
        }
        case ImplicitSurface:
        {
          if(param == 0) {
            const Meshing::TriMesh& mesh = AsTriangleMesh();
            if(mesh.tris.empty()) return false;
            Real sumlengths = 0;
            for(size_t i=0;i<mesh.tris.size();i++) {
              sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
              sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
              sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
            }
            Real avglength = sumlengths / (3*mesh.tris.size());
            param = avglength/2;
            Vector3 bmin,bmax;
            mesh.GetAABB(bmin,bmax);
            param = Min(param,0.25*(bmax.x - bmin.x));
            param = Min(param,0.25*(bmax.y - bmin.y));
            param = Min(param,0.25*(bmax.z - bmin.z));
            cout<<"Auto-determined grid resolution "<<param<<endl;
          }
          Meshing::VolumeGrid grid;
          CollisionMesh mesh(AsTriangleMesh());
          mesh.CalcTriNeighbors();
          MeshToImplicitSurface_FMM(mesh,grid,param);
          cout<<"FMM grid bounding box "<<grid.bb<<endl;
          res = AnyGeometry3D(grid);
          return true;
        }
        default:
          break;
      }
      break;
    case ImplicitSurface:
      switch(restype) {
        case PointCloud:
        {
          AnyGeometry3D mesh;
          Convert(TriangleMesh,mesh,param);
          Meshing::PointCloud3D pc;
          MeshToPointCloud(mesh.AsTriangleMesh(),pc);
          res = AnyGeometry3D(pc);
          return true;
        }
        case TriangleMesh:
        {
          //TODO: use offset properly
          const Meshing::VolumeGrid& grid = AsImplicitSurface();
          //if(param != 0) grid.Add(param);
          Meshing::TriMesh mesh;
          ImplicitSurfaceToMesh(grid,mesh);
          //if(param != 0) grid.Add(-param);
          res = AnyGeometry3D(mesh);
          return true;
        }
        default:
          break;
      }
      break;
    case Group:
      {
        const vector<AnyGeometry3D>& items = AsGroup();
        vector<AnyGeometry3D> converted(items.size());
        for(size_t i=0;i<items.size();i++) 
          items[i].Convert(restype,converted[i],param);
        res.Merge(items);
        return true;
      }
      break;
    default:
      break;
  }
  return false;
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

GeometricPrimitive3D AnyGeometry3D::GetElement(int elem) const
{
  if(elem < 0 || elem >= (int)NumElements()) FatalError("Invalid element index specified");
  if(type == TriangleMesh) {
    Math3D::Triangle3D tri;
    AsTriangleMesh().GetTriangle(elem,tri);
    return GeometricPrimitive3D(tri);
  }
  else if(type == PointCloud) {
    return GeometricPrimitive3D(AsPointCloud().points[elem]);
  }
  else if(type == Primitive) {
    return AsPrimitive();
  }
  else if(type == ImplicitSurface) {
    const Meshing::VolumeGrid& grid = AsImplicitSurface();
    IntTriple size = grid.value.size();
    //elem = cell.a*size.b*size.c + cell.b*size.c + cell.c;
    IntTriple cell;
    cell.a = elem/(size.b*size.c);
    cell.b = (elem/size.c)%size.b;
    cell.c = elem%size.c;
    AABB3D bb;
    grid.GetCell(cell,bb);
    return GeometricPrimitive3D(bb);
  }
  else if(type == Group) {
    const vector<AnyGeometry3D>& items = AsGroup();
    if(items[elem].type != Primitive)
      FatalError("Can't retrieve single element of Group geometry as a GeometricPrimitive3D");
    return items[elem].AsPrimitive();
  }
  else {
    FatalError("Invalid type?");
    return GeometricPrimitive3D();
  }
}

bool AnyGeometry3D::CanLoadExt(const char* ext)
{
  return Meshing::CanLoadTriMeshExt(ext) || 0==strcmp(ext,"pcd") || 0==strcmp(ext,"vol") || 0==strcmp(ext,"geom") || 0==strcmp(ext,"group");
}

bool AnyGeometry3D::CanSaveExt(const char* ext)
{
  return Meshing::CanSaveTriMeshExt(ext) || 0==strcmp(ext,"pcd") || 0==strcmp(ext,"vol") || 0==strcmp(ext,"geom") || 0==strcmp(ext,"group");
}

bool AnyGeometry3D::Load(const char* fn)
{
  const char* ext = FileExtension(fn);
  if(Meshing::CanLoadTriMeshExt(ext)) {
    type = TriangleMesh;
    data = Meshing::TriMesh();
    GLDraw::GeometryAppearance blank,temp;
    if(!Meshing::Import(fn,this->AsTriangleMesh(),temp)) return false;
    if(temp.faceColor != blank.faceColor ||
       !temp.vertexColors.empty() ||
       !temp.faceColors.empty()) //loaded appearance data
      appearanceData = temp;
    return true;
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
  else if(0==strcmp(ext,"geom")) {
    ifstream in(fn,ios::in);
    if(!in) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"AnyGeometry3D::Load: File "<<fn);
      return false;
    }
    if(!Load(in)) return false;
    in.close();
    return true;
  }
  else {
    ifstream in(fn,ios::in);
    if(!in) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"AnyGeometry3D::Load: File "<<fn);
      return false;
    }
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
    else {
      LOG4CXX_WARN(KrisLibrary::logger(),"AnyGeometry3D::Save: Unknown mesh file extension "<<fn);
    }
    break;
  case PointCloud:
    if(0==strcmp(ext,"pcd")) {
      return this->AsPointCloud().SavePCL(fn);
    }
    else {
      LOG4CXX_WARN(KrisLibrary::logger(),"AnyGeometry3D::Save: Unknown point cloud file extension "<<fn);
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"Failed to load Primitive type\n");
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
    int n;
    in >> n;
    if (!in || n < 0) return false;
    vector<AnyGeometry3D> grp(n);
    for(size_t i=0;i<grp.size();i++)
      if(!grp[i].Load(in)) return false;
    type = Group;
    data = grp;
    return true;
  }
  else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"AnyGeometry::Load(): Unknown type "<<typestr.c_str());
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
    {
      const vector<AnyGeometry3D>& grp = this->AsGroup();
      out<<grp.size()<<endl;
      for(size_t i=0;i<grp.size();i++)
        if(!grp[i].Save(out)) return false;
      return true;
    }
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
    AsPointCloud().GetAABB(bb.bmin,bb.bmax);
    return bb;
  case ImplicitSurface:
    return AsImplicitSurface().bb;
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
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const GeometricPrimitive3D& primitive)
  :AnyGeometry3D(primitive),margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::TriMesh& mesh)
  :AnyGeometry3D(mesh),margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::PointCloud3D& pc)
  :AnyGeometry3D(pc),margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::VolumeGrid& grid)
  :AnyGeometry3D(grid),margin(0)
{
  currentTransform.setIdentity();
}


AnyCollisionGeometry3D::AnyCollisionGeometry3D(const vector<AnyGeometry3D>& items)
  :AnyGeometry3D(items),margin(0)
{
  currentTransform.setIdentity();
}


AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyGeometry3D& geom)
  :AnyGeometry3D(geom),margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyCollisionGeometry3D& geom)
{
  *this = geom;
}

AnyCollisionGeometry3D& AnyCollisionGeometry3D::operator =(const AnyCollisionGeometry3D& geom)
{
  AnyGeometry3D::operator = (geom);
  margin = geom.margin;
  currentTransform = geom.currentTransform;

  if(!geom.collisionData.empty()) {
    switch(type) {
    case Primitive:
    case ImplicitSurface:
      {
        const CollisionImplicitSurface& cmesh = geom.ImplicitSurfaceCollisionData();
        collisionData = CollisionImplicitSurface(cmesh);
      }
      break;
    case TriangleMesh:
      {
        const CollisionMesh& cmesh = geom.TriangleMeshCollisionData();
        collisionData = CollisionMesh(cmesh);
      }
      break;
    case PointCloud:
      {
        const CollisionPointCloud& cmesh = geom.PointCloudCollisionData();    
        collisionData = CollisionPointCloud(cmesh);
      }
      break;
    case Group:
      {
        collisionData = vector<AnyCollisionGeometry3D>();
        vector<AnyCollisionGeometry3D>& colitems = GroupCollisionData();
        const vector<AnyCollisionGeometry3D>& geomitems = geom.GroupCollisionData();
        colitems.resize(geomitems.size());
        for(size_t i=0;i<geomitems.size();i++)
          colitems[i] = AnyCollisionGeometry3D(geomitems[i]);
      }
      break;
    }
  }
  return *this;
}

  const RigidTransform& AnyCollisionGeometry3D::PrimitiveCollisionData() const { return currentTransform; }
  const CollisionMesh& AnyCollisionGeometry3D::TriangleMeshCollisionData() const { return *AnyCast_Raw<CollisionMesh>(&collisionData); }
  const CollisionPointCloud& AnyCollisionGeometry3D::PointCloudCollisionData() const { return *AnyCast_Raw<CollisionPointCloud>(&collisionData); }
  const CollisionImplicitSurface& AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() const { return *AnyCast_Raw<CollisionImplicitSurface>(&collisionData); }
  const vector<AnyCollisionGeometry3D>& AnyCollisionGeometry3D::GroupCollisionData() const { return *AnyCast_Raw<vector<AnyCollisionGeometry3D> >(&collisionData); }
  RigidTransform& AnyCollisionGeometry3D::PrimitiveCollisionData() { return currentTransform; }
  CollisionMesh& AnyCollisionGeometry3D::TriangleMeshCollisionData() { return *AnyCast_Raw<CollisionMesh>(&collisionData); }
  CollisionPointCloud& AnyCollisionGeometry3D::PointCloudCollisionData() { return *AnyCast_Raw<CollisionPointCloud>(&collisionData); }
  CollisionImplicitSurface& AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() { return *AnyCast_Raw<CollisionImplicitSurface>(&collisionData); }
  vector<AnyCollisionGeometry3D>& AnyCollisionGeometry3D::GroupCollisionData() { return *AnyCast_Raw<vector<AnyCollisionGeometry3D> >(&collisionData); }

void AnyCollisionGeometry3D::InitCollisionData()
{
  if(collisionData.empty())
    ReinitCollisionData();
}

void AnyCollisionGeometry3D::ReinitCollisionData()
{
  RigidTransform T = GetTransform();
  switch(type) {
  case Primitive:
    collisionData = int(0);
    break;
  case ImplicitSurface:
    collisionData = CollisionImplicitSurface(AsImplicitSurface());
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
      for(size_t i=0;i<items.size();i++) {
        colitems[i] = AnyCollisionGeometry3D(items[i]);
        colitems[i].ReinitCollisionData();
        Assert(colitems[i].CollisionDataInitialized());
      }
      vector<AnyCollisionGeometry3D>& bitems = GroupCollisionData();
      for(size_t i=0;i<bitems.size();i++) 
        Assert(bitems[i].CollisionDataInitialized());
    }
    break;
  }
  SetTransform(T);
  assert(!collisionData.empty());
}

bool AnyCollisionGeometry3D::Convert(Type restype,AnyCollisionGeometry3D& res,double param)
{
  if(type == TriangleMesh && restype == ImplicitSurface) {
    Assert(CollisionDataInitialized());
    Assert(!TriangleMeshCollisionData().triNeighbors.empty());
    if(param == 0) {
      const Meshing::TriMesh& mesh = AsTriangleMesh();
      if(mesh.tris.empty()) return false;
      Real sumlengths = 0;
      for(size_t i=0;i<mesh.tris.size();i++) {
        sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
        sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
        sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
      }
      Real avglength = sumlengths / (3*mesh.tris.size());
      param = avglength/2;
      Vector3 bmin,bmax;
      mesh.GetAABB(bmin,bmax);
      param = Min(param,0.25*(bmax.x - bmin.x));
      param = Min(param,0.25*(bmax.y - bmin.y));
      param = Min(param,0.25*(bmax.z - bmin.z));
      cout<<"Auto-determined grid resolution "<<param<<endl;
    }
    Meshing::VolumeGrid grid;
    RigidTransform Torig,Tident; 
    Tident.setIdentity();
    CollisionMesh& mesh = TriangleMeshCollisionData();
    mesh.GetTransform(Torig);
    mesh.UpdateTransform(Tident);
    MeshToImplicitSurface_FMM(mesh,grid,param);
    //MeshToImplicitSurface_SpaceCarving(TriangleMeshCollisionData(),grid,param,40);
    mesh.UpdateTransform(Torig);
    res = AnyCollisionGeometry3D(grid);
    cout<<"Grid bb "<<grid.bb<<endl;
    res.SetTransform(GetTransform());
    return true;
  }
  if(!AnyGeometry3D::Convert(restype,res,param)) return false;
  res.SetTransform(GetTransform());
  return true;
}

AABB3D AnyCollisionGeometry3D::GetAABBTight() const
{
  switch(type) {
  case Primitive:
  case ImplicitSurface:
    return GetAABB();
    break;
  case TriangleMesh:
    {
      const CollisionMesh& m = TriangleMeshCollisionData();
      AABB3D bb;
      bb.minimize();
      for(size_t i=0;i<m.verts.size();i++)
        bb.expand(m.currentTransform*m.verts[i]);
      return bb;
    }
    break;
  case PointCloud:
    {
      const CollisionPointCloud& pc = PointCloudCollisionData();
      AABB3D bb;
      bb.minimize();
      for(size_t i=0;i<pc.points.size();i++)
        bb.expand(pc.currentTransform*pc.points[i]);
      return bb;
    }
    break;
  case Group:
    {
      const vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      AABB3D bb;
      bb.minimize();
      for(size_t i=0;i<items.size();i++)
  bb.setUnion(items[i].GetAABBTight());
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

AABB3D AnyCollisionGeometry3D::GetAABB() const
{
  if(collisionData.empty()) {
    Box3D b = GetBB();
    AABB3D bb;
    b.getAABB(bb);
    return bb;
  }
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
    AABB3D bblocal = AnyGeometry3D::GetAABB();
    b.setTransformed(bblocal,currentTransform);
  }
  else {
    switch(type) {
    case Primitive:
      b.setTransformed(AsPrimitive().GetBB(),PrimitiveCollisionData());
      break;
    case TriangleMesh:
      ::GetBB(TriangleMeshCollisionData(),b);
      break;
    case PointCloud:
      ::GetBB(PointCloudCollisionData(),b);
      break;
    case ImplicitSurface:
      b.setTransformed(AsImplicitSurface().bb,ImplicitSurfaceCollisionData().currentTransform);
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
  return currentTransform;
}

void AnyCollisionGeometry3D::SetTransform(const RigidTransform& T)
{
  currentTransform = T;
  if(!collisionData.empty()) {
    switch(type) {
    case Primitive:
      break;
    case ImplicitSurface:
      ImplicitSurfaceCollisionData().currentTransform = T;
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
}

Real AnyCollisionGeometry3D::Distance(const Vector3& pt)
{
  InitCollisionData();
  switch(type) {
  case Primitive:
    {
      Vector3 ptlocal;
      GetTransform().mulInverse(pt,ptlocal);
      return Max(AsPrimitive().Distance(ptlocal)-margin,0.0);
    }
  case ImplicitSurface:
    {
      const CollisionImplicitSurface& vg = ImplicitSurfaceCollisionData();
      return ::Distance(vg,pt);
    }
    break;
  case TriangleMesh:
    {
      Vector3 cp;
      ClosestPoint(TriangleMeshCollisionData(),pt,cp);
      return Min(pt.distance(cp)-margin,0.0);
    }
  case PointCloud:
    {
      const CollisionPointCloud& pc = PointCloudCollisionData();
      Vector3 ptlocal;
      GetTransform().mulInverse(pt,ptlocal);
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
      vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      Real dmin = Inf;
      for(size_t i=0;i<items.size();i++)
	dmin = Min(dmin,items[i].Distance(pt));
      return Min(Sqrt(dmin)-margin,0.0);
    }
  }
  return Inf;
}

AnyDistanceQueryResult AnyCollisionGeometry3D::Distance(const Vector3& pt,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  res.hasClosestPoints = true;
  res.hasElements = true;
  res.elem2 = 0;
  res.cp2 = pt;
  InitCollisionData();
  switch(type) {
  case Primitive:
    {
      Vector3 ptlocal;
      GetTransform().mulInverse(pt,ptlocal);
      const GeometricPrimitive3D& g = AsPrimitive();
      Assert(g.SupportsClosestPoints(GeometricPrimitive3D::Point));
      res.elem1 = 0;
      res.hasDirections = true;
      res.d = g.ClosestPoints(ptlocal,res.cp1,res.dir1);
      res.dir2.setNegative(res.dir1);
      Transform1(res,GetTransform());
      Offset1(res,margin);
      return res;
    }
  case ImplicitSurface:
    {
      const CollisionImplicitSurface& vg = ImplicitSurfaceCollisionData();
      res.d = ::Distance(vg,pt,res.cp1,res.dir2);
      res.dir1.setNegative(res.dir2);
      res.hasPenetration = true;
      res.hasDirections = true;
      res.elem1 = PointIndex(vg,res.cp1);
      Offset1(res,margin);
      //cout<<"Doing ImplicitSurface - point collision detection, with direction "<<res.dir2<<endl;
      return res;
    }
  case TriangleMesh:
    {
      int tri = ClosestPoint(TriangleMeshCollisionData(),pt,res.cp1);
      res.elem2 = tri;
      res.d = Min(pt.distance(res.cp1)-margin,0.0);
      return res;
    }
  case PointCloud:
    {
      Vector3 ptlocal;
      GetTransform().mulInverse(pt,ptlocal);
      const CollisionPointCloud& pc = PointCloudCollisionData();
      if(!pc.octree->NearestNeighbor(ptlocal,res.cp1,res.elem1)) return res;
      res.d = res.cp1.distance(ptlocal)-margin;
      Transform1(res,GetTransform());
      return res;
    }
  case Group:
    {
      vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
      AnyDistanceQuerySettings modsettings = settings;
      modsettings.upperBound += margin;
      for(size_t i=0;i<items.size();i++) {
        AnyDistanceQueryResult ires = items[i].Distance(pt,modsettings);
        if(ires.d < res.d) {
          res = ires;
          PushGroup1(res,(int)i);
          modsettings.upperBound = res.d + margin;
        }
      }
      Offset1(res,margin);
      return res;
    }
  }
  return res;
}



bool Collides(const CollisionImplicitSurface& grid,const GeometricPrimitive3D& a,Real margin,
	      vector<int>& gridelements,size_t maxContacts)
{
  if(a.type != GeometricPrimitive3D::Point && a.type != GeometricPrimitive3D::Sphere) {
    FatalError("Can't collide an implicit surface and a non-sphere primitive yet\n");
  }
  Vector3 gclosest,aclosest,grad;
  if(::Distance(grid,a,gclosest,aclosest,grad) <= margin) {
    gridelements.resize(1);
    gridelements[0] = PointIndex(grid,gclosest);
    return true;
  }
  return false;
}

bool Collides(const GeometricPrimitive3D& a,const GeometricPrimitive3D& b,Real margin)
{
  if(margin==0) return a.Collides(b);
  return a.Distance(b) <= margin;
}

bool Collides(const GeometricPrimitive3D& a,const CollisionImplicitSurface& b,Real margin,
	      vector<int>& gridelements,size_t maxContacts)
{
  return Collides(b,a,margin,gridelements,maxContacts);
}

bool Collides(const GeometricPrimitive3D& a,const CollisionMesh& c,Real margin,
	      vector<int>& meshelements,size_t maxContacts)
{
  NearbyTriangles(c,a,margin,meshelements,(int)maxContacts);
  return !meshelements.empty();
}

bool Collides(const GeometricPrimitive3D& a,const CollisionPointCloud& b,Real margin,vector<int>& pcelements,size_t maxContacts)
{
  NearbyPoints(b,a,margin,pcelements,maxContacts);
  return !pcelements.empty();
}





bool Collides(const CollisionImplicitSurface& a,const CollisionImplicitSurface& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  FatalError("Volume grid to volume grid collisions not done\n");
  return false;
}

bool Collides(const CollisionImplicitSurface& a,const CollisionMesh& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  FatalError("Volume grid to triangle mesh collisions not done\n");
  return false;
}

bool Collides(const CollisionMesh& a,const CollisionMesh& b,Real margin,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  if(maxContacts==1) {
    CollisionMeshQueryEnhanced query(a,b);
    query.margin1 = 0;
    query.margin2 = margin;
    bool res = query.Collide();
    if(res) {
      query.CollisionPairs(elements1,elements2);
      assert(elements1.size()==1);
      assert(elements2.size()==1);
    }
    return res;
  }
  NearbyTriangles(a,b,margin,elements1,elements2,(int)maxContacts);
  return !elements1.empty();
}

//forward declarations
bool Collides(const GeometricPrimitive3D& a,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts);
bool Collides(const CollisionMesh& a,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts);
bool Collides(const CollisionPointCloud& a,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts);
bool Collides(const CollisionImplicitSurface& a,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts);

template <class T>
bool Collides(const T& a,vector<AnyCollisionGeometry3D>& bitems,Real margin,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  elements1.resize(0);
  elements2.resize(0);
  for(size_t i=0;i<bitems.size();i++) {
    assert(bitems[i].CollisionDataInitialized());
    vector<int> e1,e2;
    if(Collides(a,margin,bitems[i],e1,e2,maxContacts-elements2.size())) {
      for(size_t j=0;j<e1.size();j++) {
        elements1.push_back(e1[j]);
        elements2.push_back((int)i);
      }
    }
    if(elements2.size() >= maxContacts) return true;
  }
  return !elements1.empty();
}

/*
static Real gWithinDistanceMargin = 0;
static const CollisionPointCloud* gWithinDistancePC = NULL;
static AnyCollisionGeometry3D* gWithinDistanceGeom = NULL;
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
    LOG4CXX_INFO(KrisLibrary::logger(),"Colliding point "<<pw.x<<" "<<pw.y<<" "<<pw.z<<" distance "<<gWithinDistanceGeom->Distance(pw));
    gWithinDistanceElements2->push_back(p-&gWithinDistancePC->points[0]);
    if(gWithinDistanceElements1->size() >= gWithinDistanceMaxContacts) 
      return false;
  }
  return true;
}
*/


inline void Copy(const PQP_REAL p[3],Vector3& x)
{
  x.set(p[0],p[1],p[2]);
}

inline void Copy(const Vector3& x,PQP_REAL p[3])
{
  p[0] = x.x;
  p[1] = x.y;
  p[2] = x.z;
}


inline void BVToBox(const BV& b,Box3D& box)
{
  Copy(b.d,box.dims);
  Copy(b.To,box.origin);
  //box.xbasis.set(b.R[0][0],b.R[0][1],b.R[0][2]);
  //box.ybasis.set(b.R[1][0],b.R[1][1],b.R[1][2]);
  //box.zbasis.set(b.R[2][0],b.R[2][1],b.R[2][2]);
  box.xbasis.set(b.R[0][0],b.R[1][0],b.R[2][0]);
  box.ybasis.set(b.R[0][1],b.R[1][1],b.R[2][1]);
  box.zbasis.set(b.R[0][2],b.R[1][2],b.R[2][2]);

  //move the box to have origin at the corner
  box.origin -= box.dims.x*box.xbasis;
  box.origin -= box.dims.y*box.ybasis;
  box.origin -= box.dims.z*box.zbasis;
  box.dims *= 2;
}


inline bool Collide(const Triangle3D& tri,const Sphere3D& s)
{
  Vector3 pt = tri.closestPoint(s.center);
  return s.contains(pt);
}

inline Real Volume(const AABB3D& bb)
{
  Vector3 d = bb.bmax-bb.bmin;
  return d.x*d.y*d.z;
}

inline Real Volume(const OctreeNode& n)
{
  return Volume(n.bb);
}

inline Real Volume(const BV& b)
{
  return 8.0*b.d[0]*b.d[1]*b.d[2];
}

class PointMeshCollider
{
public:
  const CollisionPointCloud& pc;
  const CollisionMesh& mesh;
  RigidTransform Tba,Twa,Tab;
  Real margin;
  size_t maxContacts;
  vector<int> pcpoints,meshtris;
  PointMeshCollider(const CollisionPointCloud& a,const CollisionMesh& b,Real _margin)
    :pc(a),mesh(b),margin(_margin),maxContacts(1)
  {
    Twa.setInverse(a.currentTransform);
    Tba.mul(Twa,b.currentTransform);
    Tab.setInverse(Tba);
  }
  bool Recurse(size_t _maxContacts=1)
  {
    maxContacts=_maxContacts;
    _Recurse(0,0);
    return !pcpoints.empty();
  }
  bool Prune(const OctreeNode& pcnode,const BV& meshnode) {
    Box3D meshbox,meshbox_pc;
    BVToBox(meshnode,meshbox);
    meshbox_pc.setTransformed(meshbox,Tba);
    if(margin==0)
      return !meshbox_pc.intersects(pcnode.bb);
    else {
      AABB3D expanded_bb = pcnode.bb;
      expanded_bb.bmin -= Vector3(margin);
      expanded_bb.bmax += Vector3(margin);
      return !meshbox_pc.intersects(expanded_bb);
    }
  }
  bool _Recurse(int pcOctreeNode,int meshBVHNode) {
    const OctreeNode& pcnode = pc.octree->Node(pcOctreeNode);
    const BV& meshnode = mesh.pqpModel->b[meshBVHNode];
    //returns true to keep recursing
    if(Prune(pcnode,meshnode))
      return true;
    if(pc.octree->IsLeaf(pcnode)) {
      if(meshnode.Leaf()) {
	int t = -meshnode.first_child-1;
	Triangle3D tri;
	Copy(mesh.pqpModel->tris[t].p1,tri.a);
	Copy(mesh.pqpModel->tris[t].p2,tri.b);
	Copy(mesh.pqpModel->tris[t].p3,tri.c);
	tri.a = Tba * tri.a;
	tri.b = Tba * tri.b;
	tri.c = Tba * tri.c;
	//collide the triangle and points
	vector<Vector3> pts;
	vector<int> pcids;
	pc.octree->GetPoints(pcOctreeNode,pts);
	pc.octree->GetPointIDs(pcOctreeNode,pcids);
	Sphere3D temp;
	temp.radius = margin;
	for(size_t i=0;i<pts.size();i++) {
	  temp.center = pts[i];
	  if(Collide(tri,temp)) {
	    pcpoints.push_back(pcids[i]);
	    meshtris.push_back(mesh.pqpModel->tris[t].id);
	    if(pcpoints.size() >= maxContacts) return false;
	  }
	}
	//continue
	return true;
      }
      else {
	//split mesh BVH
	return _RecurseSplitMesh(pcOctreeNode,meshBVHNode);
      }
    }
    else {
      if(meshnode.Leaf()) {
	//split octree node
	return _RecurseSplitOctree(pcOctreeNode,meshBVHNode);
      }
      else {
	//determine which BVH to split
	Real vpc = Volume(pcnode);
	Real vmesh = Volume(meshnode);
	if(vpc < vmesh)
	  return _RecurseSplitMesh(pcOctreeNode,meshBVHNode);
	else
	  return _RecurseSplitOctree(pcOctreeNode,meshBVHNode);
      }
    }
  }
  bool _RecurseSplitMesh(int pcOctreeNode,int meshBVHNode) {
    int c1=mesh.pqpModel->b[meshBVHNode].first_child;
    int c2=c1+1;
    if(!_Recurse(pcOctreeNode,c1)) return false;
    if(!_Recurse(pcOctreeNode,c2)) return false;
    return true;
  }
  bool _RecurseSplitOctree(int pcOctreeNode,int meshBVHNode) {
    const OctreeNode& pcnode = pc.octree->Node(pcOctreeNode);
    for(int i=0;i<8;i++)
      if(!_Recurse(pcnode.childIndices[i],meshBVHNode)) return false;
    return true;
  }
};


bool Collides(const CollisionPointCloud& a,Real margin,const CollisionMesh& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  PointMeshCollider collider(a,b,margin);
  bool res=collider.Recurse(maxContacts);
  if(res) {
    elements1=collider.pcpoints;
    elements2=collider.meshtris;
    return true;
  }
  return false;
}

bool Collides(const CollisionPointCloud& a,Real margin,const CollisionPointCloud& b,
	      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  return Geometry::Collides(a,b,margin,elements1,elements2,maxContacts);
}

bool Collides(const CollisionPointCloud& a,Real margin,const CollisionImplicitSurface& b,
  vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  bool res=Geometry::Collides(b,a,margin,elements1,maxContacts);
  elements2.resize(elements1.size());
  for(size_t i=0;i<elements1.size();i++)
    elements2[i] = PointIndex(b,a.currentTransform*a.points[elements1[i]]);
  return res;
}



bool Collides(const GeometricPrimitive3D& a,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  Assert(b.CollisionDataInitialized());
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      if(::Collides(a,bw,margin+b.margin)) {
        elements1.push_back(0);
        elements2.push_back(0);
        return true;
      }
      return false;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    if(::Collides(b.ImplicitSurfaceCollisionData(),a,margin+b.margin,elements2,maxContacts)) {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::TriangleMesh:
    if(::Collides(a,b.TriangleMeshCollisionData(),margin+b.margin,elements2,maxContacts)) {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::PointCloud:
    if(::Collides(a,b.PointCloudCollisionData(),margin+b.margin,elements2,maxContacts)) {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::Group:
    {
      vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      return ::Collides(a,bitems,margin+b.margin,elements1,elements2,maxContacts);
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}


bool Collides(const GeometricPrimitive3D& a,const RigidTransform& Ta,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  if(a.type == GeometricPrimitive3D::Empty) return false;
  GeometricPrimitive3D aw=a;
  aw.Transform(Ta);
  return Collides(aw,margin,b,elements1,elements2,maxContacts);
}


bool Collides(const CollisionImplicitSurface& a,Real margin,AnyCollisionGeometry3D& b,
        vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      if(::Collides(a,bw,margin+b.margin,elements1,maxContacts)) {
  elements2.push_back(0);
  return true;
      }
      return false;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    return ::Collides(a,b.ImplicitSurfaceCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(a,b.TriangleMeshCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
    {
      const CollisionPointCloud& pc = b.PointCloudCollisionData();
      bool res = Geometry::Collides(a,pc,margin,elements2,maxContacts);
      elements1.resize(elements2.size());
      for(size_t i=0;i<elements2.size();i++) 
        elements1[i] = PointIndex(a,b.currentTransform*pc.points[elements2[i]]);
      return res;
    }
    break;
  case AnyCollisionGeometry3D::Group:
    {
      vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      return ::Collides(a,bitems,margin+b.margin,elements1,elements2,maxContacts);
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const CollisionMesh& a,Real margin,AnyCollisionGeometry3D& b,
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
    return ::Collides(b.ImplicitSurfaceCollisionData(),a,margin+b.margin,elements2,elements1,maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(a,b.TriangleMeshCollisionData(),margin+b.margin,elements1,elements2,maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
    return ::Collides(b.PointCloudCollisionData(),margin+b.margin,a,elements2,elements1,maxContacts);
  case AnyCollisionGeometry3D::Group:
    {
      vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      return ::Collides(a,bitems,margin+b.margin,elements1,elements2,maxContacts);
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const CollisionPointCloud& a,Real margin,AnyCollisionGeometry3D& b,
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
  case AnyCollisionGeometry3D::TriangleMesh:
    {
      bool res=::Collides(a,margin,b.TriangleMeshCollisionData(),elements1,elements2,maxContacts);
      return res;
    }
  case AnyCollisionGeometry3D::PointCloud:
    {
      bool res=::Collides(a,margin,b.PointCloudCollisionData(),elements1,elements2,maxContacts);
      return res;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    {
      bool res=::Collides(a,margin,b.ImplicitSurfaceCollisionData(),elements1,elements2,maxContacts);
      return res;
    }
  case AnyCollisionGeometry3D::Group:
    {
      vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      return ::Collides(a,bitems,margin+b.margin,elements1,elements2,maxContacts);
    }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(vector<AnyCollisionGeometry3D>& group,Real margin,AnyCollisionGeometry3D& b,
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



bool AnyCollisionGeometry3D::Collides(AnyCollisionGeometry3D& geom)
{
  InitCollisionData();
  geom.InitCollisionData();
  vector<int> elem1,elem2;
  return Collides(geom,elem1,elem2,1);
}

bool AnyCollisionGeometry3D::Collides(AnyCollisionGeometry3D& geom,
				      vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  InitCollisionData();
  geom.InitCollisionData();
  //prioritize point cloud testing
  if(geom.type == PointCloud && type != PointCloud)
    return geom.Collides(*this,elements2,elements1,maxContacts);
  //otherwise...
  switch(type) {
  case Primitive:
    return ::Collides(AsPrimitive(),GetTransform(),margin,geom,elements1,elements2,maxContacts);
  case ImplicitSurface:
    return ::Collides(ImplicitSurfaceCollisionData(),margin,geom,elements1,elements2,maxContacts);
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


AnyDistanceQueryResult Distance(const GeometricPrimitive3D& a,const GeometricPrimitive3D& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.elem1=0;
  res.elem2=0;
  res.hasPenetration = true;
  if(a.SupportsClosestPoints(b.type)) {
    res.hasClosestPoints = true;
    res.hasDirections = true;
    res.d = a.ClosestPoints(b,res.cp1,res.dir1);
    SetCP2(res);
  }
  else
    res.d = a.Distance(b);
  return res;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D& a,const CollisionImplicitSurface& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.elem1 = 0;
  res.d = Geometry::Distance(b,a,res.cp2,res.cp1,res.dir1);
  res.elem2 = PointIndex(b,res.cp2);
  res.dir2.setNegative(res.dir1);
  return res;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D& a,const CollisionPointCloud& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.elem1 = 0;
  res.d = Distance(b,a,res.elem2,settings.upperBound);
  //TODO: get contact point and direction?
  return res;
}

AnyDistanceQueryResult Distance(const CollisionImplicitSurface& a,const CollisionPointCloud& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.d = Geometry::Distance(a,b,res.elem2,settings.upperBound);
  res.cp2 = b.currentTransform*b.points[res.elem2];
  Geometry::Distance(a,res.cp2,res.cp1,res.dir1);
  res.dir2.setNegative(res.dir1);
  res.elem1 = PointIndex(a,res.cp1);
  return res;
}

AnyDistanceQueryResult Distance(const CollisionMesh& a,const CollisionMesh& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  CollisionMeshQuery q(a,b);
  res.d=q.Distance(settings.absErr,settings.relErr,settings.upperBound);
  q.ClosestPair(res.elem1,res.elem2);
  q.ClosestPoints(res.cp1,res.cp2);
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}

//advance declarations
AnyDistanceQueryResult Distance(const GeometricPrimitive3D& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings);
AnyDistanceQueryResult Distance(const CollisionImplicitSurface& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings);
AnyDistanceQueryResult Distance(const CollisionMesh& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings);
AnyDistanceQueryResult Distance(const CollisionPointCloud& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings);

//note modifies settings
template <class T>
AnyDistanceQueryResult Distance_Group(const T& a,const vector<AnyCollisionGeometry3D>& bitems,AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  for(size_t i=0;i<bitems.size();i++) {
    AnyDistanceQueryResult ires = ::Distance(a,bitems[i],settings);
    if(ires.d < res.d) {
      res = ires;
      PushGroup2(res,(int)i);
      settings.upperBound = res.d;
    }
  }
  return res;
}


AnyDistanceQueryResult Distance(const GeometricPrimitive3D& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  if(a.type == GeometricPrimitive3D::Empty) return res;
  Assert(b.CollisionDataInitialized());
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      res = Distance(a,bw,modsettings);
      Offset2(res,b.margin);
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    {
      res = Distance(a,b.ImplicitSurfaceCollisionData(),modsettings);
      Offset2(res,b.margin);
    }
    break;
  case AnyCollisionGeometry3D::TriangleMesh:
    fprintf(stderr,"Unable to do primitive/triangle mesh distance yet\n");
    break;
  case AnyCollisionGeometry3D::PointCloud:
    {
      res = Distance(a,b.PointCloudCollisionData(),modsettings);
      Offset2(res,b.margin);
    }
    break;
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      res = Distance_Group(a,bitems,modsettings);
      Offset2(res,b.margin);
      return res;
    }
  default:
    FatalError("Invalid type");
  }
  return res;
}


AnyDistanceQueryResult Distance(const CollisionImplicitSurface& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      res = Distance(bw,a,modsettings);
      Flip(res);
      Offset2(res,b.margin);
    }
    break;
  case AnyCollisionGeometry3D::ImplicitSurface:
    fprintf(stderr,"Unable to do implicit surface/implicit surface distance yet\n");
    break;
  case AnyCollisionGeometry3D::TriangleMesh:
    fprintf(stderr,"Unable to do implicit surface/triangle mesh distance yet\n");
    break;
  case AnyCollisionGeometry3D::PointCloud:
    {
      res = Distance(a,b.PointCloudCollisionData(),modsettings);
      Offset2(res,b.margin);
      break;
    }
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      res = ::Distance_Group(a,bitems,modsettings);
      Offset2(res,b.margin);
      return res;
    }
  default:
    FatalError("Invalid type");
  }
  return res;
}

AnyDistanceQueryResult Distance(const CollisionMesh& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    fprintf(stderr,"Unable to do triangle mesh/primitive distance yet\n");
    break;
  case AnyCollisionGeometry3D::ImplicitSurface:
    fprintf(stderr,"Unable to do triangle mesh/implicit surface distance yet\n");
    break;
  case AnyCollisionGeometry3D::TriangleMesh:
    {
      res = Distance(a,b.TriangleMeshCollisionData(),modsettings);
      Offset2(res,b.margin);
      return res;
    }
  case AnyCollisionGeometry3D::PointCloud:
    fprintf(stderr,"Unable to do triangle mesh/point cloud distance yet\n");
    break;
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      res = ::Distance_Group(a,bitems,modsettings);
      Offset2(res,b.margin);
      return res;
    }
  default:
    FatalError("Invalid type");
  }
  return res;
}


AnyDistanceQueryResult Distance(const CollisionPointCloud& a,const AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch(b.type) {
  case AnyCollisionGeometry3D::Primitive:
    {
      GeometricPrimitive3D bw=b.AsPrimitive();
      bw.Transform(b.GetTransform());
      res = ::Distance(bw,a,modsettings);
      Flip(res);
      Offset2(res,b.margin);
      return res;
      //return ::Distance(bw,a,elem1) - (margin+b.margin);
    }
  case AnyCollisionGeometry3D::TriangleMesh:
    {
      fprintf(stderr,"Unable to do point cloud/triangle mesh distance yet\n");
      //return ::Distance(a,b.TriangleMeshCollisionData(),elem1,elem2) - (margin+b.margin);
      return res;
    }
  case AnyCollisionGeometry3D::PointCloud:
    {
      fprintf(stderr,"Unable to do point cloud/point cloud distance yet\n");
      //return ::Distance(a,b.PointCloudCollisionData(),elem1,elem2) - (margin+b.margin);
      return res;
    }
  case AnyCollisionGeometry3D::ImplicitSurface:
    {
      res = ::Distance(b.ImplicitSurfaceCollisionData(),a,modsettings);
      Flip(res);
      Offset2(res,b.margin);
      return res;
    }
  case AnyCollisionGeometry3D::Group:
    {
      const vector<AnyCollisionGeometry3D>& bitems = b.GroupCollisionData();
      res = ::Distance_Group(a,bitems,modsettings);
      Offset2(res,b.margin);
      return res;
    }
  default:
    FatalError("Invalid type");
  }
  return res;
}

AnyDistanceQueryResult Distance(vector<AnyCollisionGeometry3D>& group,AnyCollisionGeometry3D& b,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  for(size_t i=0;i<group.size();i++) {
    AnyDistanceQueryResult ires=group[i].Distance(b,modsettings);
    if(ires.d < res.d) {
      res = ires;
      PushGroup1(res,int(i));
      modsettings.upperBound = res.d;
    }
  }
  return res;
}

Real AnyCollisionGeometry3D::Distance(AnyCollisionGeometry3D& geom)
{
  InitCollisionData();
  geom.InitCollisionData();
  AnyDistanceQuerySettings settings;
  return Distance(geom,settings).d;
}

AnyDistanceQueryResult AnyCollisionGeometry3D::Distance(AnyCollisionGeometry3D& geom,const AnyDistanceQuerySettings& settings)
{
  AnyDistanceQueryResult result;
  result.hasElements = true;
  InitCollisionData();
  geom.InitCollisionData();
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += margin;
  switch(type) {
  case Primitive:
    {
      GeometricPrimitive3D aw = AsPrimitive();
      aw.Transform(GetTransform());
      result = ::Distance(aw,geom,modsettings);
      Offset1(result,margin);
      return result;
    }
  case ImplicitSurface:
    result = ::Distance(ImplicitSurfaceCollisionData(),geom,modsettings);
    Offset1(result,margin);
    return result;
  case TriangleMesh:
    result = ::Distance(TriangleMeshCollisionData(),geom,modsettings);
    Offset1(result,margin);
    return result;
  case PointCloud:
    result = ::Distance(PointCloudCollisionData(),geom,modsettings);
    Offset1(result,margin);
    return result;
  case Group:
    result = ::Distance(GroupCollisionData(),geom,modsettings);
    Offset1(result,margin);
    return result;
  default:
    FatalError("Invalid type");
  }
  return result;
}

bool AnyCollisionGeometry3D::WithinDistance(AnyCollisionGeometry3D& geom,Real tol)
{
  InitCollisionData();
  geom.InitCollisionData();
  vector<int> elem1,elem2;
  return WithinDistance(geom,tol,elem1,elem2,1);
}

bool AnyCollisionGeometry3D::WithinDistance(AnyCollisionGeometry3D& geom,Real tol,
					    vector<int>& elements1,vector<int>& elements2,size_t maxContacts)
{
  InitCollisionData();
  geom.InitCollisionData();
  switch(type) {
  case Primitive:
    return ::Collides(AsPrimitive(),GetTransform(),margin+tol,geom,elements1,elements2,maxContacts);
  case ImplicitSurface:
    return ::Collides(ImplicitSurfaceCollisionData(),margin+tol,geom,elements1,elements2,maxContacts);
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

bool AnyCollisionGeometry3D::RayCast(const Ray3D& r,Real* distance,int* element)
{
  InitCollisionData();
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
      vector<AnyCollisionGeometry3D>& items = GroupCollisionData();
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

AnyCollisionQuery::AnyCollisionQuery(AnyCollisionGeometry3D& _a,AnyCollisionGeometry3D& _b)
  :a(&_a),b(&_b)
{
}

AnyCollisionQuery::AnyCollisionQuery(const AnyCollisionQuery& q)
  :a(q.a),b(q.b),qmesh(q.qmesh)
{}


bool UpdateQMesh(AnyCollisionQuery* q) 
{
  if(q->a->type == AnyGeometry3D::TriangleMesh && q->b->type == AnyGeometry3D::TriangleMesh) {
    if(!q->qmesh.m1) {
      q->a->InitCollisionData();
      q->b->InitCollisionData();
      q->qmesh = CollisionMeshQueryEnhanced(q->a->TriangleMeshCollisionData(),q->b->TriangleMeshCollisionData());
      Assert(q->qmesh.m1);
      Assert(q->qmesh.m2);
    }
    else {
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
  if(!a || !b) return false;
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(UpdateQMesh(this)) {
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
  if(!a || !b) return false;
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(UpdateQMesh(this)) {
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
  if(!a || !b) return false;
  elements1.resize(0);
  elements2.resize(0);
  points1.resize(0);
  points2.resize(0);
  if(UpdateQMesh(this)) {
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
  if(UpdateQMesh(this)) {
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
  if(!a || !b) return -Inf;
  if(UpdateQMesh(this)) {
    return qmesh.PenetrationDepth();
  }
  return -a->Distance(*b);
}

Real AnyCollisionQuery::Distance(Real absErr,Real relErr,Real bound)
{
  if(!a || !b) return Inf;
  elements1.resize(1);
  elements2.resize(1);
  points1.resize(0);
  points2.resize(0);
  if(UpdateQMesh(this)) {
    points1.resize(1);
    points2.resize(1);
    Real res = qmesh.Distance(absErr,relErr,bound);
    qmesh.ClosestPair(elements1[0],elements2[0]);
    qmesh.ClosestPoints(points1[0],points2[0]);
    return res;
  }
  AnyDistanceQuerySettings settings;
  settings.absErr = absErr;
  settings.relErr = relErr;
  settings.upperBound = bound;
  AnyDistanceQueryResult res = a->Distance(*b,settings);
  if(res.hasElements) {
    elements1[0] = res.elem1;
    elements2[0] = res.elem2;
  }
  if(res.hasClosestPoints) {
    points1.resize(1);
    points2.resize(1);
    points1[0] = res.cp1;
    points2[0] = res.cp2;
  }
  return res.d;
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

