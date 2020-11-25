#include <KrisLibrary/Logger.h>
#include "AnyGeometry.h"
#include "Conversions.h"
#include <math3d/geometry3d.h>
#include <meshing/VolumeGrid.h>
#include <meshing/Voxelize.h>
#include <meshing/Meshing.h>
#include <GLdraw/GeometryAppearance.h>
#include "CollisionPointCloud.h"
#include "CollisionImplicitSurface.h"
#include "ConvexHull3D.h"
#include <utils/stringutils.h>
#include <meshing/IO.h>
#include <structs/Heap.h>
#include <Timer.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>

#include "PQP/include/PQP.h"
#include "SOLID.h"

DEFINE_LOGGER(Geometry)

using namespace Geometry;

namespace Geometry
{
  //declared in CollisionMesh
  void RigidTransformToPQP(const RigidTransform &f, PQP_REAL R[3][3], PQP_REAL T[3]);
} //namespace Geometry

void Flip(AnyDistanceQueryResult &res)
{
  std::swap(res.elem1, res.elem2);
  std::swap(res.group_elem1, res.group_elem2);
  std::swap(res.cp1, res.cp2);
  std::swap(res.dir1, res.dir2);
}

void Transform1(AnyDistanceQueryResult &res, const RigidTransform &T)
{
  if (res.hasClosestPoints)
    res.cp1 = T * res.cp1;
  if (res.hasDirections)
    res.dir1 = T.R * res.dir1;
}

void Transform2(AnyDistanceQueryResult &res, const RigidTransform &T)
{
  if (res.hasClosestPoints)
    res.cp2 = T * res.cp2;
  if (res.hasDirections)
    res.dir2 = T.R * res.dir2;
}

void Offset1(AnyDistanceQueryResult &res, Real offset)
{
  res.d -= offset;
  if (res.hasDirections)
    res.cp1.madd(res.dir1, offset);
}

void Offset2(AnyDistanceQueryResult &res, Real offset)
{
  res.d -= offset;
  if (res.hasDirections)
    res.cp2.madd(res.dir2, offset);
}

void SetCP2(AnyDistanceQueryResult &res)
{
  res.cp2 = res.cp1;
  res.cp2.madd(res.dir1, res.d);
  res.dir2.setNegative(res.dir1);
}

void PushGroup1(AnyDistanceQueryResult &res, int i)
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

void PushGroup2(AnyDistanceQueryResult &res, int i)
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

int PointIndex(const CollisionImplicitSurface &s, const Vector3 &ptworld)
{
  Vector3 plocal;
  s.currentTransform.mulInverse(ptworld, plocal);
  IntTriple cell;
  s.baseGrid.GetIndex(plocal, cell);
  if (cell.a < 0)
    cell.a = 0;
  if (cell.a >= s.baseGrid.value.m)
    cell.a = s.baseGrid.value.m - 1;
  if (cell.b < 0)
    cell.b = 0;
  if (cell.b >= s.baseGrid.value.n)
    cell.b = s.baseGrid.value.n - 1;
  if (cell.c < 0)
    cell.c = 0;
  if (cell.c >= s.baseGrid.value.p)
    cell.c = s.baseGrid.value.p - 1;
  return cell.a * s.baseGrid.value.n * s.baseGrid.value.p + cell.b * s.baseGrid.value.p + cell.c;
}

AnyDistanceQueryResult::AnyDistanceQueryResult()
    : hasPenetration(0), hasElements(0), hasClosestPoints(0), hasDirections(0), d(Inf), elem1(-1), elem2(-1)
{
}

AnyDistanceQuerySettings::AnyDistanceQuerySettings()
    : relErr(0), absErr(0), upperBound(Inf)
{
}

AnyGeometry3D::AnyGeometry3D()
    : type(Primitive), data(GeometricPrimitive3D())
{
}

AnyGeometry3D::AnyGeometry3D(const GeometricPrimitive3D &primitive)
    : type(Primitive), data(primitive)
{
}

AnyGeometry3D::AnyGeometry3D(const ConvexHull3D &cvxhull)
    : type(ConvexHull), data(cvxhull)
{
}

AnyGeometry3D::AnyGeometry3D(const Meshing::TriMesh &mesh)
    : type(TriangleMesh), data(mesh)
{
}

AnyGeometry3D::AnyGeometry3D(const Meshing::PointCloud3D &pc)
    : type(PointCloud), data(pc)
{
}

AnyGeometry3D::AnyGeometry3D(const Meshing::VolumeGrid &grid)
    : type(ImplicitSurface), data(grid)
{
}

AnyGeometry3D::AnyGeometry3D(const vector<AnyGeometry3D> &group)
    : type(Group), data(group)
{
}

const GeometricPrimitive3D &AnyGeometry3D::AsPrimitive() const { return *AnyCast_Raw<GeometricPrimitive3D>(&data); }
const ConvexHull3D& AnyGeometry3D::AsConvexHull() const { return *AnyCast_Raw<ConvexHull3D>(&data); };
const Meshing::TriMesh &AnyGeometry3D::AsTriangleMesh() const { return *AnyCast_Raw<Meshing::TriMesh>(&data); }
const Meshing::PointCloud3D &AnyGeometry3D::AsPointCloud() const { return *AnyCast_Raw<Meshing::PointCloud3D>(&data); }
const Meshing::VolumeGrid &AnyGeometry3D::AsImplicitSurface() const { return *AnyCast_Raw<Meshing::VolumeGrid>(&data); }
const vector<AnyGeometry3D> &AnyGeometry3D::AsGroup() const { return *AnyCast_Raw<vector<AnyGeometry3D>>(&data); }
GeometricPrimitive3D &AnyGeometry3D::AsPrimitive() { return *AnyCast_Raw<GeometricPrimitive3D>(&data); }
ConvexHull3D& AnyGeometry3D::AsConvexHull() { return *AnyCast_Raw<ConvexHull3D>(&data); };
Meshing::TriMesh &AnyGeometry3D::AsTriangleMesh() { return *AnyCast_Raw<Meshing::TriMesh>(&data); }
Meshing::PointCloud3D &AnyGeometry3D::AsPointCloud() { return *AnyCast_Raw<Meshing::PointCloud3D>(&data); }
Meshing::VolumeGrid &AnyGeometry3D::AsImplicitSurface() { return *AnyCast_Raw<Meshing::VolumeGrid>(&data); }
vector<AnyGeometry3D> &AnyGeometry3D::AsGroup() { return *AnyCast_Raw<vector<AnyGeometry3D>>(&data); }


//appearance casts
GLDraw::GeometryAppearance *AnyGeometry3D::TriangleMeshAppearanceData() { return AnyCast<GLDraw::GeometryAppearance>(&appearanceData); }
const GLDraw::GeometryAppearance *AnyGeometry3D::TriangleMeshAppearanceData() const { return AnyCast<GLDraw::GeometryAppearance>(&appearanceData); }

const char *AnyGeometry3D::TypeName(Type type)
{
  switch (type)
  {
  case Primitive:
    return "Primitive";
  case ConvexHull:
    return "ConvexHull";
  case TriangleMesh:
    return "TriangleMesh";
  case PointCloud:
    return "PointCloud";
  case ImplicitSurface:
    return "ImplicitSurface";
  case Group:
    return "Group";
  default:
    return "Error";
  }
}

bool AnyGeometry3D::Empty() const
{
  switch (type)
  {
  case Primitive:
    return AsPrimitive().type == GeometricPrimitive3D::Empty;
  case ConvexHull:
    return AsConvexHull().data.empty();  //TODO: this is not enough...
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

void AnyGeometry3D::Merge(const vector<AnyGeometry3D> &geoms)
{
  vector<int> nonempty;
  for (size_t i = 0; i < geoms.size(); i++)
    if (!geoms[i].Empty())
      nonempty.push_back((int)i);
  if (nonempty.empty())
    *this = AnyGeometry3D();
  else if (nonempty.size() == 1)
  {
    *this = geoms[nonempty[0]];
  }
  else
  {
    type = geoms[nonempty[0]].type;
    bool group = false;
    for (size_t i = 1; i < nonempty.size(); i++)
      if (geoms[nonempty[i]].type != type)
      {
        //its' a group type
        group = true;
      }
    switch (type)
    {
    case Primitive:
      group = true;
      break;
    case ConvexHull:
      group = true;
      break;
    case TriangleMesh:
    {
      Meshing::TriMesh merged;
      vector<Meshing::TriMesh> items(nonempty.size());
      for (size_t i = 0; i < nonempty.size(); i++)
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
        for (size_t i = 0; i < nonempty.size(); i++)
          items.insert(items.end(), geoms[nonempty[i]].AsGroup().begin(), geoms[nonempty[i]].AsGroup().end());
        data = items;
      }
      break;
    }
    if (group)
    {
      type = Group;
      data = geoms;
    }
  }
}

bool AnyGeometry3D::Convert(Type restype, AnyGeometry3D &res, Real param) const
{
  if (type == restype)
  {
    res.type = type;
    res.data = data;
    res.appearanceData = appearanceData;
    return true;
  }
  if (restype == Group)
    return false;
  if (type != ImplicitSurface)
    Assert(param >= 0);
  switch (type)
  {
  case Primitive:
    switch (restype)
    {
    case TriangleMesh:
    {
      if (param == 0)
        param = 16;
      else
      {
        AABB3D bb = GetAABB();
        Real w = (bb.bmax - bb.bmin).maxAbsElement();
        const GeometricPrimitive3D &g = AsPrimitive();
        if (g.type == GeometricPrimitive3D::Cylinder)
          w = AnyCast_Raw<Math3D::Cylinder3D>(&g.data)->radius * 2;
        param = int(w / param);
      }
      Meshing::TriMesh mesh;
      PrimitiveToMesh(AsPrimitive(), mesh, int(param));
      res = AnyGeometry3D(mesh);
      return true;
    }
    case PointCloud:
    {
      AnyGeometry3D mesh;
      Convert(TriangleMesh, mesh, param);
      Meshing::PointCloud3D pc;
      if (param == 0)
        param = Inf;
      MeshToPointCloud(mesh.AsTriangleMesh(), pc, param, true);
      res = AnyGeometry3D(pc);
      return true;
    }
    case ImplicitSurface:
    {
      if (param == 0)
      {
        AABB3D bb = GetAABB();
        Real w = (bb.bmax - bb.bmin).maxAbsElement();
        param = w * 0.05;
      }
      Meshing::VolumeGrid grid;
      PrimitiveToImplicitSurface(AsPrimitive(), grid, param);
      res = AnyGeometry3D(grid);
      return true;
    }
    case ConvexHull:
    {
      AnyGeometry3D mesh;
      Convert(TriangleMesh,mesh,param);
      mesh.Convert(ConvexHull,res,param);
      return true;
    }
    default:
      break;
    }
    break;
  case ConvexHull:
    switch(restype) {
      case TriangleMesh:
      {
        Meshing::TriMesh mesh;
        ConvexHullToMesh(AsConvexHull(),mesh);
        res = AnyGeometry3D(mesh);
        return true;
      }
      case PointCloud:
      {
        AnyGeometry3D mesh;
        Convert(TriangleMesh,mesh,param);
        if (param == 0) param = Inf;
        mesh.Convert(PointCloud, res, param);
      }
      default:
        LOG4CXX_WARN(GET_LOGGER(Geometry), "Cannot convert from ConvexHull to anything except for TriangleMesh and PointCloud");
        break;
    }
  case PointCloud:
    switch (restype)
    {
    case TriangleMesh:
    {
      if (!AsPointCloud().IsStructured())
      {
        LOG4CXX_WARN(GET_LOGGER(Geometry), "AnyGeometry3D::Convert: Point cloud is not structured, cannot convert to triangle mesh");
        return false;
      }
      if (param == 0)
        param = Inf;
      Meshing::TriMesh mesh;
      PointCloudToMesh(AsPointCloud(), mesh, param);
      res = AnyGeometry3D(mesh);
      return true;
    }
    case ImplicitSurface:
    {
      return false;
    }
    case ConvexHull: {
      ConvexHull3D chull;
      PointCloudToConvexHull(AsPointCloud(),chull);
      res.type = ConvexHull;
      res.data = chull;
      return true;
    }
    default:
      break;
    }
  case TriangleMesh:
    switch (restype)
    {
    case PointCloud:
    {
      if (param == 0)
        param = Inf;
      Meshing::PointCloud3D pc;
      MeshToPointCloud(AsTriangleMesh(), pc, param, true);
      res = AnyGeometry3D(pc);
      return true;
    }
    case ImplicitSurface:
    {
      if (param == 0)
      {
        const Meshing::TriMesh &mesh = AsTriangleMesh();
        if (mesh.tris.empty())
          return false;
        Real sumlengths = 0;
        for (size_t i = 0; i < mesh.tris.size(); i++)
        {
          sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
          sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
          sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
        }
        Real avglength = sumlengths / (3 * mesh.tris.size());
        param = avglength / 2;
        Vector3 bmin, bmax;
        mesh.GetAABB(bmin, bmax);
        param = Min(param, 0.25 * (bmax.x - bmin.x));
        param = Min(param, 0.25 * (bmax.y - bmin.y));
        param = Min(param, 0.25 * (bmax.z - bmin.z));
        LOG4CXX_INFO(GET_LOGGER(Geometry), "AnyGeometry::Convert: Auto-determined grid resolution " << param);
      }
      Meshing::VolumeGrid grid;
      CollisionMesh mesh(AsTriangleMesh());
      mesh.CalcTriNeighbors();
      MeshToImplicitSurface_FMM(mesh, grid, param);
      LOG4CXX_INFO(GET_LOGGER(Geometry), "AnyGeometry::Convert: FMM grid bounding box " << grid.bb);
      res = AnyGeometry3D(grid);
      return true;
    }
    case ConvexHull:
    {
      const Meshing::TriMesh &mesh = AsTriangleMesh();
      ConvexHull3D chull;
      MeshConvexDecomposition(mesh,chull,param);
      res.type = ConvexHull;
      res.data = chull;
      return true;
    }
    default:
      break;
    }
    break;
  case ImplicitSurface:
    switch (restype)
    {
    case PointCloud:
    {
      AnyGeometry3D mesh;
      Convert(TriangleMesh, mesh, param);
      Meshing::PointCloud3D pc;
      MeshToPointCloud(mesh.AsTriangleMesh(), pc, true);
      res = AnyGeometry3D(pc);
      return true;
    }
    case TriangleMesh:
    {
      const Meshing::VolumeGrid &grid = AsImplicitSurface();
      Meshing::TriMesh mesh;
      ImplicitSurfaceToMesh(grid, mesh, param);
      res = AnyGeometry3D(mesh);
      return true;
    }
    case ConvexHull:
    {
      AnyGeometry3D mesh;
      Convert(TriangleMesh, mesh, param);
      mesh.Convert(ConvexHull,res,0);
      return true;
    }
    default:
      break;
    }
    break;
  case Group:
  {
    const vector<AnyGeometry3D> &items = AsGroup();
    vector<AnyGeometry3D> converted(items.size());
    for (size_t i = 0; i < items.size(); i++)
      items[i].Convert(restype, converted[i], param);
    res.Merge(items);
    return true;
  }
  break;
  default:
    break;
  }
  return false;
}

bool AnyGeometry3D::Remesh(Real resolution,AnyGeometry3D& res,bool refine,bool coarsen) const
{
  switch(type)
  {
  case Primitive:
  case ConvexHull:
    res.type = type;
    res.data = data;
    res.appearanceData = appearanceData;
    return true;
  case TriangleMesh:
    {
      if(resolution <= 0) return false;
      Meshing::TriMeshWithTopology mesh;
      mesh.verts = AsTriangleMesh().verts;
      mesh.tris = AsTriangleMesh().tris;
      if(refine)
        Meshing::SubdivideToResolution(mesh,resolution);
      res = AnyGeometry3D(mesh);
      return true;
    }
  case PointCloud:
    {
      if(resolution <= 0) return false;
      const Meshing::PointCloud3D& pc = AsPointCloud();
      Meshing::PointCloud3D output;
      if(coarsen) {
        GridSubdivision3D grid(resolution);
        GridSubdivision3D::Index index;
        for(size_t i=0;i<pc.points.size();i++) {
          grid.PointToIndex(pc.points[i],index);
          grid.Insert(index,(void*)&pc.points[i]);
        }
        output.points.reserve(grid.buckets.size());
        output.properties.reserve(grid.buckets.size());
        output.propertyNames = pc.propertyNames;
        output.settings = pc.settings;
        if(pc.IsStructured()) {
          output.settings.erase(output.settings.find("width")); 
          output.settings.erase(output.settings.find("height")); 
        }
        for(auto i=grid.buckets.begin();i!=grid.buckets.end();i++) {
          Vector3 ptavg(Zero);
          int n=(int)i->second.size();
          Real scale = 1.0/n;
          for(auto pt:i->second) {
            int ptindex = (const Vector3*)pt - &pc.points[0];
            ptavg += pc.points[ptindex];
          }
          output.points.push_back(ptavg*scale);
          output.properties.push_back(Vector(pc.propertyNames.size()));
          for(size_t j=0;j<pc.propertyNames.size();j++) {
            if(pc.propertyNames[j]=="rgb" || pc.propertyNames[j]=="rgba" || pc.propertyNames[j]=="c") {
              //int numchannels = (int)pc.propertyNames[j].length();
              int propavg[4] = {0,0,0,0};
              for(auto pt:i->second) {
                int ptindex = (const Vector3*)pt - &pc.points[0];
                int prop = (int)pc.properties[ptindex][j];
                propavg[0] += (prop&0xff);
                propavg[1] += ((prop>>8)&0xff);
                propavg[2] += ((prop>>16)&0xff);
                propavg[3] += ((prop>>24)&0xff);
              }
              for(int k=0;k<4;k++)
                propavg[k] = (propavg[k]/n)&0xff;
              output.properties.back()[j] = (Real)(propavg[0] | (propavg[1] << 8) | (propavg[2] << 16) | (propavg[3] << 24));
            }
            else {
              Real propavg = 0;
              for(auto pt:i->second) {
                int ptindex = (const Vector3*)pt - &pc.points[0];
                propavg += pc.properties[ptindex][j];
              }
              output.properties.back()[j] = propavg*scale;
            }
          }
        }
      }
      else
        output = pc;
      res = AnyGeometry3D(output);
      return true;
    }
  case ImplicitSurface:
    {
      const Meshing::VolumeGrid& grid =AsImplicitSurface();
      Vector3 size=grid.GetCellSize();
      if((resolution < size.x && refine) || (resolution > size.x && coarsen) ||
        (resolution < size.y && refine) || (resolution > size.y && coarsen) ||
        (resolution < size.z && refine) || (resolution > size.z && coarsen)) {
        int m = (int)Ceil((grid.bb.bmax.x-grid.bb.bmin.x) / resolution);
        int n = (int)Ceil((grid.bb.bmax.y-grid.bb.bmin.y) / resolution);
        int p = (int)Ceil((grid.bb.bmax.z-grid.bb.bmin.z) / resolution);
        Meshing::VolumeGrid output;
        output.Resize(m,n,p);
        output.bb = grid.bb;
        output.ResampleTrilinear(grid);
        res = AnyGeometry3D(output);
        return true;
      }
      else {
        res = *this;
        return true;
      }
    }
    break;
  case Group:
    {
      if(resolution <= 0) return false;
      const vector<AnyGeometry3D> &items = AsGroup();
      vector<AnyGeometry3D> converted(items.size());
      for (size_t i = 0; i < items.size(); i++)
        items[i].Remesh(resolution, converted[i], refine, coarsen);
      res = AnyGeometry3D(items);
      return true;
    }
  default:
    FatalError("Unhandled type?");
    return false;
  }
}

size_t AnyGeometry3D::NumElements() const
{
  switch (type)
  {
  case Primitive:
    if (AsPrimitive().type == GeometricPrimitive3D::Empty)
      return 0;
    return 1;
  case ConvexHull:
  {
    return AsConvexHull().NumPrimitives();
  }
  case TriangleMesh:
    return AsTriangleMesh().tris.size();
  case PointCloud:
    return AsPointCloud().points.size();
  case ImplicitSurface:
  {
    IntTriple size = AsImplicitSurface().value.size();
    return size.a * size.b * size.c;
  }
  case Group:
    return AsGroup().size();
  }
  return 0;
}

GeometricPrimitive3D AnyGeometry3D::GetElement(int elem) const
{
  if (elem < 0 || elem >= (int)NumElements())
    FatalError("Invalid element index specified");
  if (type == TriangleMesh)
  {
    Math3D::Triangle3D tri;
    AsTriangleMesh().GetTriangle(elem, tri);
    return GeometricPrimitive3D(tri);
  }
  else if (type == ConvexHull)
  {
    return AsConvexHull().GetPrimitive(elem);
  }
  else if (type == PointCloud)
  {
    return GeometricPrimitive3D(AsPointCloud().points[elem]);
  }
  else if (type == Primitive)
  {
    return AsPrimitive();
  }
  else if (type == ImplicitSurface)
  {
    const Meshing::VolumeGrid &grid = AsImplicitSurface();
    IntTriple size = grid.value.size();
    //elem = cell.a*size.b*size.c + cell.b*size.c + cell.c;
    IntTriple cell;
    cell.a = elem / (size.b * size.c);
    cell.b = (elem / size.c) % size.b;
    cell.c = elem % size.c;
    AABB3D bb;
    grid.GetCell(cell, bb);
    return GeometricPrimitive3D(bb);
  }
  else if (type == Group)
  {
    const vector<AnyGeometry3D> &items = AsGroup();
    if (items[elem].type != Primitive)
      FatalError("Can't retrieve single element of Group geometry as a GeometricPrimitive3D");
    return items[elem].AsPrimitive();
  }
  else
  {
    FatalError("Invalid type?");
    return GeometricPrimitive3D();
  }
}

//TODO: what does convex hull support?
bool AnyGeometry3D::CanLoadExt(const char *ext)
{
  return Meshing::CanLoadTriMeshExt(ext) || 0 == strcmp(ext, "pcd") || 0 == strcmp(ext, "vol") || 0 == strcmp(ext, "geom") || 0 == strcmp(ext, "group");
}

bool AnyGeometry3D::CanSaveExt(const char *ext)
{
  return Meshing::CanSaveTriMeshExt(ext) || 0 == strcmp(ext, "pcd") || 0 == strcmp(ext, "vol") || 0 == strcmp(ext, "geom") || 0 == strcmp(ext, "group");
}

// TODO: What does convex hull support?
bool AnyGeometry3D::Load(const char *fn)
{
  const char *ext = FileExtension(fn);
  if (Meshing::CanLoadTriMeshExt(ext))
  {
    type = TriangleMesh;
    data = Meshing::TriMesh();
    GLDraw::GeometryAppearance blank, temp;
    if (!Meshing::Import(fn, this->AsTriangleMesh(), temp))
      return false;
    if (temp.faceColor != blank.faceColor ||
        !temp.vertexColors.empty() ||
        !temp.faceColors.empty()) //loaded appearance data
      appearanceData = temp;
    return true;
  }
  else if (0 == strcmp(ext, "pcd"))
  {
    type = PointCloud;
    data = Meshing::PointCloud3D();
    return this->AsPointCloud().LoadPCL(fn);
  }
  else if (0 == strcmp(ext, "vol"))
  {
    type = ImplicitSurface;
    data = Meshing::VolumeGrid();
    ifstream in(fn, ios::in);
    if (!in)
      return false;
    in >> this->AsImplicitSurface();
    if (!in)
      return false;
    in.close();
    return true;
  }
  else if (0 == strcmp(ext, "geom"))
  {
    ifstream in(fn, ios::in);
    if (!in)
    {
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "AnyGeometry3D::Load: File " << fn);
      return false;
    }
    if (!Load(in))
      return false;
    in.close();
    return true;
  }
  else
  {
    ifstream in(fn, ios::in);
    if (!in)
    {
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "AnyGeometry3D::Load: File " << fn);
      return false;
    }
    if (!Load(in))
      return false;
    in.close();
    return true;
  }
  return true;
}

//TODO: how to save convex hull?
bool AnyGeometry3D::Save(const char *fn) const
{
  const char *ext = FileExtension(fn);
  switch (type)
  {
  case Primitive:
    break;
  case ConvexHull:  // TODO: what should we do here
    break;
  case TriangleMesh:
    if (Meshing::CanSaveTriMeshExt(ext))
    {
      return Meshing::Export(fn, this->AsTriangleMesh());
    }
    else
    {
      LOG4CXX_WARN(GET_LOGGER(Geometry), "AnyGeometry3D::Save: Unknown mesh file extension " << fn);
    }
    break;
  case PointCloud:
    if (0 == strcmp(ext, "pcd"))
    {
      return this->AsPointCloud().SavePCL(fn);
    }
    else
    {
      LOG4CXX_WARN(GET_LOGGER(Geometry), "AnyGeometry3D::Save: Unknown point cloud file extension " << fn);
    }
    break;
  case ImplicitSurface:
  {
    ofstream out(fn, ios::out);
    if (!out)
      return false;
    out << this->AsImplicitSurface();
    out << endl;
    out.close();
    return true;
  }
  break;
  case Group:
    break;
  }
  //default save
  ofstream out(fn, ios::out);
  if (!out)
    return false;
  if (!Save(out))
    return false;
  out.close();
  return true;
}

bool AnyGeometry3D::Load(istream &in)
{
  string typestr;
  in >> typestr;
  if (typestr == "Primitive")
  {
    GeometricPrimitive3D geom;
    in >> geom;
    if (in)
    {
      type = Primitive;
      data = geom;
      return true;
    }
    else
    {
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "Failed to load Primitive type\n");
      return false;
    }
  }
  else if (typestr == "ConvexHull") {
    ConvexHull3D hull;
    in >> hull;
    if(in) {
      type = ConvexHull;
      data = hull;
      return true;
    }
  }
  else if (typestr == "TriangleMesh")
  {
    type = TriangleMesh;
    data = Meshing::TriMesh();
    auto pos = in.tellg();
    //try OFF and tri formats
    if(!LoadOFF(in,this->AsTriangleMesh())) {
      in.seekg(pos);
      in >> this->AsTriangleMesh();
    }
  }
  else if (typestr == "PointCloud")
  {
    type = PointCloud;
    data = Meshing::PointCloud3D();
    if (!AsPointCloud().LoadPCL(in))
      return false;
    return true;
  }
  else if (typestr == "ImplicitSurface")
  {
    type = ImplicitSurface;
    data = Meshing::VolumeGrid();
    in >> this->AsImplicitSurface();
  }
  else if (typestr == "Group")
  {
    int n;
    in >> n;
    if (!in || n < 0)
      return false;
    vector<AnyGeometry3D> grp(n);
    for (size_t i = 0; i < grp.size(); i++)
      if (!grp[i].Load(in))
        return false;
    type = Group;
    data = grp;
    return true;
  }
  else {
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyGeometry::Load(): Unknown type "<<typestr);
        return false;
  }
  if(!in) {
    LOG4CXX_ERROR(GET_LOGGER(Geometry),"AnyGeometry::Load(): Something went wrong inputting item of type "<<typestr);
    return false;
  }
  return true;
}

bool AnyGeometry3D::Save(ostream &out) const
{
  out << TypeName() << endl;
  switch (type)
  {
  case Primitive:
    out << this->AsPrimitive() << endl;
    break;
  case ConvexHull:
    out << this->AsConvexHull() << endl;
    break;
  case TriangleMesh:
    out << this->AsTriangleMesh() << endl;
    break;
  case PointCloud:
    if (!AsPointCloud().SavePCL(out))
      return false;
    break;
  case ImplicitSurface:
    out << this->AsImplicitSurface() << endl;
    break;
  case Group:
  {
    const vector<AnyGeometry3D> &grp = this->AsGroup();
    out << grp.size() << endl;
    for (size_t i = 0; i < grp.size(); i++)
      if (!grp[i].Save(out))
        return false;
    return true;
  }
  }
  return true;
}

void AnyGeometry3D::Transform(const RigidTransform &T)
{
  return Transform(Matrix4(T));
}

void AnyGeometry3D::Transform(const Matrix4 &T)
{
  switch (type)
  {
  case Primitive:
    AsPrimitive().Transform(T);
    break;
  case ConvexHull:
    AsConvexHull().Transform(T);
    break;
  case TriangleMesh:
    AsTriangleMesh().Transform(T);
    break;
  case PointCloud:
    AsPointCloud().Transform(T);
    break;
  case ImplicitSurface:
  {
    if (T(0, 1) != 0 || T(0, 2) != 0 || T(1, 2) != 0 || T(1, 0) != 0 || T(2, 0) != 0 || T(2, 1) != 0)
    {
      FatalError("Cannot transform volume grid except via translation / scale");
    }
    AsImplicitSurface().bb.bmin = T * AsImplicitSurface().bb.bmin;
    AsImplicitSurface().bb.bmax = T * AsImplicitSurface().bb.bmax;
  }
  break;
  case Group:
  {
    vector<AnyGeometry3D> &items = AsGroup();
    for (size_t i = 0; i < items.size(); i++)
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
  switch (type)
  {
  case Primitive:
    return AsPrimitive().GetAABB();
  case ConvexHull:
    return AsConvexHull().GetAABB();
  case TriangleMesh:
    AsTriangleMesh().GetAABB(bb.bmin, bb.bmax);
    return bb;
  case PointCloud:
    AsPointCloud().GetAABB(bb.bmin, bb.bmax);
    return bb;
  case ImplicitSurface:
    return AsImplicitSurface().bb;
    break;
  case Group:
  {
    const vector<AnyGeometry3D> &items = AsGroup();
    for (size_t i = 0; i < items.size(); i++)
    {
      AABB3D itembb = items[i].GetAABB();
      bb.setUnion(itembb);
    }
  }
  break;
  }
  return bb;
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D()
    : margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const GeometricPrimitive3D &primitive)
    : AnyGeometry3D(primitive), margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const ConvexHull3D &hull)
    : AnyGeometry3D(hull), margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::TriMesh &mesh)
    : AnyGeometry3D(mesh), margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::PointCloud3D &pc)
    : AnyGeometry3D(pc), margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const Meshing::VolumeGrid &grid)
    : AnyGeometry3D(grid), margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const vector<AnyGeometry3D> &items)
    : AnyGeometry3D(items), margin(0)
{
  currentTransform.setIdentity();
}

AnyCollisionGeometry3D::AnyCollisionGeometry3D(const AnyGeometry3D &geom)
    : AnyGeometry3D(geom), margin(0)
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

  if (!geom.collisionData.empty())
  {
    switch (type)
    {
    case Primitive:
    //everything is done in AnyGeometry3D
    break;
    case ConvexHull:
    {
      const ConvexHull3D& ch = geom.AsConvexHull();
      collisionData = CollisionConvexHull3D(ch);
    }
    break;
    case ImplicitSurface:
    {
      const CollisionImplicitSurface &cmesh = geom.ImplicitSurfaceCollisionData();
      collisionData = CollisionImplicitSurface(cmesh);
    }
    break;
    case TriangleMesh:
    {
      const CollisionMesh &cmesh = geom.TriangleMeshCollisionData();
      collisionData = CollisionMesh(cmesh);
    }
    break;
    case PointCloud:
    {
      const CollisionPointCloud &cmesh = geom.PointCloudCollisionData();
      collisionData = CollisionPointCloud(cmesh);
    }
    break;
    case Group:
    {
      collisionData = vector<AnyCollisionGeometry3D>();
      vector<AnyCollisionGeometry3D> &colitems = GroupCollisionData();
      const vector<AnyCollisionGeometry3D> &geomitems = geom.GroupCollisionData();
      colitems.resize(geomitems.size());
      for (size_t i = 0; i < geomitems.size(); i++)
        colitems[i] = AnyCollisionGeometry3D(geomitems[i]);
    }
    break;
    }
  }
  return *this;
}

const RigidTransform &AnyCollisionGeometry3D::PrimitiveCollisionData() const { return currentTransform; }
const CollisionConvexHull3D &AnyCollisionGeometry3D::ConvexHullCollisionData() const { return *AnyCast_Raw<CollisionConvexHull3D>(&collisionData);}
const CollisionMesh &AnyCollisionGeometry3D::TriangleMeshCollisionData() const { return *AnyCast_Raw<CollisionMesh>(&collisionData); }
const CollisionPointCloud &AnyCollisionGeometry3D::PointCloudCollisionData() const { return *AnyCast_Raw<CollisionPointCloud>(&collisionData); }
const CollisionImplicitSurface &AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() const { return *AnyCast_Raw<CollisionImplicitSurface>(&collisionData); }
const vector<AnyCollisionGeometry3D> &AnyCollisionGeometry3D::GroupCollisionData() const { return *AnyCast_Raw<vector<AnyCollisionGeometry3D>>(&collisionData); }
RigidTransform &AnyCollisionGeometry3D::PrimitiveCollisionData() { return currentTransform; }
CollisionConvexHull3D &AnyCollisionGeometry3D::ConvexHullCollisionData() { return *AnyCast_Raw<CollisionConvexHull3D>(&collisionData);}
CollisionMesh &AnyCollisionGeometry3D::TriangleMeshCollisionData() { return *AnyCast_Raw<CollisionMesh>(&collisionData); }
CollisionPointCloud &AnyCollisionGeometry3D::PointCloudCollisionData() { return *AnyCast_Raw<CollisionPointCloud>(&collisionData); }
CollisionImplicitSurface &AnyCollisionGeometry3D::ImplicitSurfaceCollisionData() { return *AnyCast_Raw<CollisionImplicitSurface>(&collisionData); }
vector<AnyCollisionGeometry3D> &AnyCollisionGeometry3D::GroupCollisionData() { return *AnyCast_Raw<vector<AnyCollisionGeometry3D>>(&collisionData); }

void AnyCollisionGeometry3D::InitCollisionData()
{
  if (collisionData.empty())
    ReinitCollisionData();
}

void AnyCollisionGeometry3D::ReinitCollisionData()
{
  RigidTransform T = GetTransform();
  switch (type)
  {
  case Primitive:
    collisionData = int(0);
    break;
  case ConvexHull:
    collisionData = CollisionConvexHull3D(AsConvexHull());
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
    vector<AnyCollisionGeometry3D> &colitems = GroupCollisionData();
    vector<AnyGeometry3D> &items = AsGroup();
    colitems.resize(items.size());
    for (size_t i = 0; i < items.size(); i++)
    {
      colitems[i] = AnyCollisionGeometry3D(items[i]);
      colitems[i].ReinitCollisionData();
      Assert(colitems[i].CollisionDataInitialized());
    }
    vector<AnyCollisionGeometry3D> &bitems = GroupCollisionData();
    for (size_t i = 0; i < bitems.size(); i++)
      Assert(bitems[i].CollisionDataInitialized());
  }
  break;
  }
  SetTransform(T);
  assert(!collisionData.empty());
}

bool AnyCollisionGeometry3D::Convert(Type restype, AnyCollisionGeometry3D &res, Real param)
{
  if (type == TriangleMesh && restype == ImplicitSurface)
  {
    InitCollisionData();
    if(TriangleMeshCollisionData().triNeighbors.empty())
      TriangleMeshCollisionData().CalcTriNeighbors();
    if (param == 0)
    {
      const Meshing::TriMesh &mesh = AsTriangleMesh();
      if (mesh.tris.empty())
        return false;
      Real sumlengths = 0;
      for (size_t i = 0; i < mesh.tris.size(); i++)
      {
        sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
        sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
        sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
      }
      Real avglength = sumlengths / (3 * mesh.tris.size());
      param = avglength / 2;
      Vector3 bmin, bmax;
      mesh.GetAABB(bmin, bmax);
      param = Min(param, 0.25 * (bmax.x - bmin.x));
      param = Min(param, 0.25 * (bmax.y - bmin.y));
      param = Min(param, 0.25 * (bmax.z - bmin.z));
      LOG4CXX_INFO(GET_LOGGER(Geometry), "Auto-determined grid resolution " << param);
    }
    Meshing::VolumeGrid grid;
    RigidTransform Torig, Tident;
    Tident.setIdentity();
    CollisionMesh &mesh = TriangleMeshCollisionData();
    mesh.GetTransform(Torig);
    mesh.UpdateTransform(Tident);
    MeshToImplicitSurface_FMM(mesh, grid, param);
    //MeshToImplicitSurface_SpaceCarving(TriangleMeshCollisionData(),grid,param,40);
    mesh.UpdateTransform(Torig);
    res = AnyCollisionGeometry3D(grid);
    LOG4CXX_INFO(GET_LOGGER(Geometry), "Grid bb " << grid.bb);
    res.SetTransform(GetTransform());
    return true;
  }
  if (!AnyGeometry3D::Convert(restype, res, param))
    return false;
  res.SetTransform(GetTransform());
  // res.ReinitCollisionData();
  return true;
}

AABB3D AnyCollisionGeometry3D::GetAABBTight() const
{
  if (collisionData.empty())
  {
    FatalError("GetAABBTight: TODO: return a tight AABB when the collision data is not yet initialized");
  }
  switch (type)
  {
  case Primitive:  // TODO: so no aabb for primitive?
    return GetAABB();
  case ConvexHull: // TODO: what is this supposed to do
    return GetAABB();
  case ImplicitSurface:
    return GetAABB();
  case TriangleMesh:
  {
    const CollisionMesh &m = TriangleMeshCollisionData();
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < m.verts.size(); i++)
      bb.expand(m.currentTransform * m.verts[i]);
    return bb;
  }
  break;
  case PointCloud:
  {
    const CollisionPointCloud &pc = PointCloudCollisionData();
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < pc.points.size(); i++)
      bb.expand(pc.currentTransform * pc.points[i]);
    return bb;
  }
  break;
  case Group:
  {
    const vector<AnyCollisionGeometry3D> &items = GroupCollisionData();
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < items.size(); i++)
      bb.setUnion(items[i].GetAABBTight());
    if (margin != 0)
    {
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
  if (collisionData.empty())
  {
    Box3D b = GetBB();
    AABB3D bb;
    b.getAABB(bb);
    return bb;
  }
  switch (type)
  {
  case Primitive:  //TODO: so this means O(n) computation, transform geometry and find aabb
  {
    const RigidTransform &T = PrimitiveCollisionData();
    const GeometricPrimitive3D &g = AsPrimitive();
    GeometricPrimitive3D gT(g);
    gT.Transform(T);
    AABB3D res = gT.GetAABB();
    if (margin != 0)
    {
      res.bmin -= Vector3(margin);
      res.bmax += Vector3(margin);
    }
    return res;
  }
  break;
  case ConvexHull:
  {
    const CollisionConvexHull3D &cdata = this->ConvexHullCollisionData();
    DT_Vector3 bmin, bmax;
    DT_GetBBox(cdata.objectHandle->data, bmin, bmax);
    AABB3D res;
    for(int i = 0; i < 3; i++){
      res.bmin[i] = bmin[i];
      res.bmax[i] = bmax[i];
    }
    // const RigidTransform &T = this->currentTransform;
    // const ConvexHull3D &g = this->AsConvexHull();
    // ConvexHull3D gT(g);
    // gT.Transform(T);
    // AABB3D res = gT.GetAABB();
    // {
    //   res.bmin -= Vector3(margin);
    //   res.bmax += Vector3(margin);
    // }
    return res;
  }
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
    const vector<AnyCollisionGeometry3D> &items = GroupCollisionData();
    AABB3D bb;
    bb.minimize();
    for (size_t i = 0; i < items.size(); i++)
      bb.setUnion(items[i].GetAABB());
    if (margin != 0)
    {
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
  if (collisionData.empty())
  {
    AABB3D bblocal = AnyGeometry3D::GetAABB();
    b.setTransformed(bblocal, currentTransform);
  }
  else
  {
    switch (type)
    {
    case Primitive:
      b.setTransformed(AsPrimitive().GetBB(), PrimitiveCollisionData());
      break;
    case ConvexHull:  //TODO: how to implement getBB function for convex hull if not axis aligned?
      b.setTransformed(AsConvexHull().GetBB(), this->currentTransform);
      break;
    case TriangleMesh:
      ::GetBB(TriangleMeshCollisionData(), b);
      break;
    case PointCloud:
      ::GetBB(PointCloudCollisionData(), b);
      break;
    case ImplicitSurface:
      b.setTransformed(AsImplicitSurface().bb, ImplicitSurfaceCollisionData().currentTransform);
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
  if (margin != 0)
  {
    b.dims += Vector3(margin * 2.0);
    b.origin -= margin * (b.xbasis + b.ybasis + b.zbasis);
  }
  return b;
}

RigidTransform AnyCollisionGeometry3D::GetTransform() const
{
  return currentTransform;
}

void AnyCollisionGeometry3D::SetTransform(const RigidTransform &T)
{
  currentTransform = T;
  this->InitCollisionData();
  if (!collisionData.empty())
  {
    switch (type)
    {
    case Primitive:
      break;
    case ConvexHull:
      ConvexHullCollisionData().UpdateTransform(T);
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
      vector<AnyCollisionGeometry3D> &items = GroupCollisionData();
      for (size_t i = 0; i < items.size(); i++)
        items[i].SetTransform(T);
    }
    break;
    }
  }
}


Real AnyCollisionGeometry3D::Distance(const Vector3 &pt)
{
  InitCollisionData();
  switch (type)
  {
  case Primitive:
  {
    Vector3 ptlocal;
    GetTransform().mulInverse(pt, ptlocal);
    return Max(AsPrimitive().Distance(ptlocal) - margin, 0.0);
  }
  case ConvexHull:
  {
    Vector3 ptlocal;
    GetTransform().mulInverse(pt, ptlocal);
    return Max(ConvexHullCollisionData().Distance(ptlocal) - margin, 0.0);
  }
  case ImplicitSurface:
  {
    const CollisionImplicitSurface &vg = ImplicitSurfaceCollisionData();
    return Geometry::Distance(vg, pt) - margin;
  }
  break;
  case TriangleMesh:
  {
    const CollisionMesh &cm = TriangleMeshCollisionData();
    return Geometry::Distance(cm, pt) - margin;
  }
  case PointCloud:
  {
    const CollisionPointCloud &pc = PointCloudCollisionData();
    return Geometry::Distance(pc, pt) - margin;
  }
  case Group:
  {
    vector<AnyCollisionGeometry3D> &items = GroupCollisionData();
    Real dmin = Inf;
    for (size_t i = 0; i < items.size(); i++)
      dmin = Min(dmin, items[i].Distance(pt));
    return dmin - margin;
  }
  }
  return Inf;
}

AnyDistanceQueryResult AnyCollisionGeometry3D::Distance(const Vector3 &pt, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasClosestPoints = true;
  res.hasElements = true;
  res.elem2 = 0;
  res.cp2 = pt;
  InitCollisionData();
  switch (type)
  {
  case Primitive:
  {
    Vector3 ptlocal;
    GetTransform().mulInverse(pt, ptlocal);
    const GeometricPrimitive3D &g = AsPrimitive();
    Assert(g.SupportsClosestPoints(GeometricPrimitive3D::Point));
    res.elem1 = 0;
    res.hasDirections = true;
    res.d = g.ClosestPoints(ptlocal, res.cp1, res.dir1);
    res.dir2.setNegative(res.dir1);
    Transform1(res, GetTransform());
    Offset1(res, margin);
    return res;
  }
  case ConvexHull:  // TODO: Here I just copy paste code from Primitive, not sure what everything is doing
  {
    Vector3 ptlocal;
    GetTransform().mulInverse(pt, ptlocal);
    res.elem1 = 0;
    res.hasDirections = true;
    CollisionConvexHull3D &g = this->ConvexHullCollisionData();
    res.d = g.ClosestPoint(ptlocal, res.cp1, res.dir1);
    res.dir2.setNegative(res.dir1);
    Transform1(res, GetTransform());
    Offset1(res, margin);
    return res;
  }
  case ImplicitSurface:
  {
    const CollisionImplicitSurface &vg = ImplicitSurfaceCollisionData();
    res.d = ::Distance(vg, pt, res.cp1, res.dir2);
    res.dir1.setNegative(res.dir2);
    res.hasPenetration = true;
    res.hasDirections = true;
    res.elem1 = PointIndex(vg, res.cp1);
    Offset1(res, margin);
    //cout<<"Doing ImplicitSurface - point collision detection, with direction "<<res.dir2<<endl;
    return res;
  }
  case TriangleMesh:
  {
    const CollisionMesh& cm = TriangleMeshCollisionData();
    int tri = ClosestPoint(cm, pt, res.cp1, settings.upperBound + margin);
    if(tri<0) res.d = settings.upperBound;
    else {
      res.cp1 = cm.currentTransform * res.cp1;
      res.elem2 = tri;
      res.d = pt.distance(res.cp1) - margin;
    }
    return res;
  }
  case PointCloud:
  {
    Vector3 ptlocal;
    GetTransform().mulInverse(pt, ptlocal);
    const CollisionPointCloud &pc = PointCloudCollisionData();
    if (!pc.octree->NearestNeighbor(ptlocal, res.cp1, res.elem1, settings.upperBound + margin)) {
      res.d = settings.upperBound;
      return res;
    }
    res.d = res.cp1.distance(ptlocal) - margin;
    Transform1(res, GetTransform());
    return res;
  }
  case Group:
  {
    vector<AnyCollisionGeometry3D> &items = GroupCollisionData();
    AnyDistanceQuerySettings modsettings = settings;
    modsettings.upperBound += margin;
    for (size_t i = 0; i < items.size(); i++)
    {
      AnyDistanceQueryResult ires = items[i].Distance(pt, modsettings);
      if (ires.d < res.d)
      {
        res = ires;
        PushGroup1(res, (int)i);
        modsettings.upperBound = res.d + margin;
      }
    }
    Offset1(res, margin);
    return res;
  }
  }
  return res;
}

bool Collides(const CollisionImplicitSurface &grid, const GeometricPrimitive3D &a, Real margin,
              vector<int> &gridelements, size_t maxContacts)
{
  if (a.type != GeometricPrimitive3D::Point && a.type != GeometricPrimitive3D::Sphere)
  {
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't collide an implicit surface and a non-sphere primitive yet");
    return false;
  }
  Vector3 gclosest, aclosest, grad;
  if (::Distance(grid, a, gclosest, aclosest, grad) <= margin)
  {
    gridelements.resize(1);
    gridelements[0] = PointIndex(grid, gclosest);
    return true;
  }
  return false;
}

bool Collides(const GeometricPrimitive3D &a, const GeometricPrimitive3D &b, Real margin)
{
  if (margin == 0)
    return a.Collides(b);
  return a.Distance(b) <= margin;
}

bool Collides(const GeometricPrimitive3D &a, const CollisionImplicitSurface &b, Real margin,
              vector<int> &gridelements, size_t maxContacts)
{
  return Collides(b, a, margin, gridelements, maxContacts);
}

bool Collides(const GeometricPrimitive3D &a, const CollisionMesh &c, Real margin,
              vector<int> &meshelements, size_t maxContacts)
{
  NearbyTriangles(c, a, margin, meshelements, (int)maxContacts);
  return !meshelements.empty();
}

bool Collides(const GeometricPrimitive3D &a, const CollisionPointCloud &b, Real margin, vector<int> &pcelements, size_t maxContacts)
{
  NearbyPoints(b, a, margin, pcelements, maxContacts);
  return !pcelements.empty();
}

bool Collides(const CollisionImplicitSurface &a, const CollisionImplicitSurface &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  LOG4CXX_ERROR(GET_LOGGER(Geometry), "Volume grid to volume grid collisions not done");
  return false;
}

bool Collides(const CollisionImplicitSurface &a, const CollisionMesh &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  LOG4CXX_ERROR(GET_LOGGER(Geometry), "Volume grid to triangle mesh collisions not done");
  return false;
}

bool Collides(const CollisionMesh &a, const CollisionMesh &b, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  if (maxContacts == 1)
  {
    CollisionMeshQueryEnhanced query(a, b);
    query.margin1 = 0;
    query.margin2 = margin;
    bool res = query.Collide();
    if (res)
    {
      query.CollisionPairs(elements1, elements2);
      assert(elements1.size() == 1);
      assert(elements2.size() == 1);
    }
    return res;
  }
  NearbyTriangles(a, b, margin, elements1, elements2, (int)maxContacts);
  return !elements1.empty();
}

//forward declarations
bool Collides(const GeometricPrimitive3D &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts);
bool Collides(const CollisionMesh &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts);
bool Collides(const CollisionPointCloud &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts);
bool Collides(const CollisionImplicitSurface &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts);

template <class T>
bool Collides(const T &a, vector<AnyCollisionGeometry3D> &bitems, Real margin,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  elements1.resize(0);
  elements2.resize(0);
  for (size_t i = 0; i < bitems.size(); i++)
  {
    assert(bitems[i].CollisionDataInitialized());
    vector<int> e1, e2;
    if (Collides(a, margin, bitems[i], e1, e2, maxContacts - elements2.size()))
    {
      for (size_t j = 0; j < e1.size(); j++)
      {
        elements1.push_back(e1[j]);
        elements2.push_back((int)i);
      }
    }
    if (elements2.size() >= maxContacts)
      return true;
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
    LOG4CXX_INFO(GET_LOGGER(Geometry),"Colliding point "<<pw.x<<" "<<pw.y<<" "<<pw.z<<" distance "<<gWithinDistanceGeom->Distance(pw));
    gWithinDistanceElements2->push_back(p-&gWithinDistancePC->points[0]);
    if(gWithinDistanceElements1->size() >= gWithinDistanceMaxContacts) 
      return false;
  }
  return true;
}
*/

inline void Copy(const PQP_REAL p[3], Vector3 &x)
{
  x.set(p[0], p[1], p[2]);
}

inline void Copy(const Vector3 &x, PQP_REAL p[3])
{
  p[0] = x.x;
  p[1] = x.y;
  p[2] = x.z;
}

inline void BVToBox(const BV &b, Box3D &box)
{
  Copy(b.d, box.dims);
  Copy(b.To, box.origin);
  //box.xbasis.set(b.R[0][0],b.R[0][1],b.R[0][2]);
  //box.ybasis.set(b.R[1][0],b.R[1][1],b.R[1][2]);
  //box.zbasis.set(b.R[2][0],b.R[2][1],b.R[2][2]);
  box.xbasis.set(b.R[0][0], b.R[1][0], b.R[2][0]);
  box.ybasis.set(b.R[0][1], b.R[1][1], b.R[2][1]);
  box.zbasis.set(b.R[0][2], b.R[1][2], b.R[2][2]);

  //move the box to have origin at the corner
  box.origin -= box.dims.x * box.xbasis;
  box.origin -= box.dims.y * box.ybasis;
  box.origin -= box.dims.z * box.zbasis;
  box.dims *= 2;
}

inline void BoxToBV(const Box3D &box, BV &b)
{
  Copy(box.dims * 0.5, b.d);
  Copy(box.center(), b.To);
  box.xbasis.get(b.R[0][0], b.R[1][0], b.R[2][0]);
  box.ybasis.get(b.R[0][1], b.R[1][1], b.R[2][1]);
  box.zbasis.get(b.R[0][2], b.R[1][2], b.R[2][2]);
}

inline bool Collide(const Triangle3D &tri, const Sphere3D &s)
{
  Vector3 pt = tri.closestPoint(s.center);
  return s.contains(pt);
}

inline Real Volume(const AABB3D &bb)
{
  Vector3 d = bb.bmax - bb.bmin;
  return d.x * d.y * d.z;
}

inline Real Volume(const OctreeNode &n)
{
  return Volume(n.bb);
}

inline Real Volume(const BV &b)
{
  return 8.0 * b.d[0] * b.d[1] * b.d[2];
}

class PointMeshCollider
{
public:
  const CollisionPointCloud &pc;
  const CollisionMesh &mesh;
  RigidTransform Tba, Twa, Tab;
  Real margin;
  size_t maxContacts;
  vector<int> pcpoints, meshtris;
  vector<Real> pairdistances2;
  PointMeshCollider(const CollisionPointCloud &a, const CollisionMesh &b, Real _margin)
      : pc(a), mesh(b), margin(_margin), maxContacts(1)
  {
    Twa.setInverse(a.currentTransform);
    Tba.mul(Twa, b.currentTransform);
    Tab.setInverse(Tba);
  }
  bool Recurse(size_t _maxContacts = 1)
  {
    maxContacts = _maxContacts;
    _Recurse(0, 0);
    return !pcpoints.empty();
  }
  bool Prune(const OctreeNode &pcnode, const BV &meshnode)
  {
    Box3D meshbox, meshbox_pc;
    BVToBox(meshnode, meshbox);
    meshbox_pc.setTransformed(meshbox, Tba);
    if (margin == 0)
      return !meshbox_pc.intersects(pcnode.bb);
    else
    {
      AABB3D expanded_bb = pcnode.bb;
      expanded_bb.bmin -= Vector3(margin);
      expanded_bb.bmax += Vector3(margin);
      return !meshbox_pc.intersects(expanded_bb);
    }
  }
  bool _Recurse(int pcOctreeNode, int meshBVHNode)
  {
    const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
    const BV &meshnode = mesh.pqpModel->b[meshBVHNode];
    //returns true to keep recursing
    if (Prune(pcnode, meshnode))
      return true;
    if (pc.octree->IsLeaf(pcnode))
    {
      if (pc.octree->NumPoints(pcnode) == 0)
        return true;
      if (meshnode.Leaf())
      {
        int t = -meshnode.first_child - 1;
        Triangle3D tri;
        Copy(mesh.pqpModel->tris[t].p1, tri.a);
        Copy(mesh.pqpModel->tris[t].p2, tri.b);
        Copy(mesh.pqpModel->tris[t].p3, tri.c);
        tri.a = Tba * tri.a;
        tri.b = Tba * tri.b;
        tri.c = Tba * tri.c;
        //collide the triangle and points
        vector<Vector3> pts;
        vector<int> pcids;
        pc.octree->GetPoints(pcOctreeNode, pts);
        pc.octree->GetPointIDs(pcOctreeNode, pcids);
        Real dmin2 = Sqr(margin);
        int closestpt = -1;
        for (size_t i = 0; i < pts.size(); i++)
        {
          Vector3 pt = tri.closestPoint(pts[i]);
          Real d2 = pts[i].distanceSquared(pt);
          if (d2 < dmin2)
          {
            dmin2 = d2;
            closestpt = pcids[i];
          }
        }
        if (closestpt >= 0)
        {
          pcpoints.push_back(closestpt);
          meshtris.push_back(mesh.pqpModel->tris[t].id);
          pairdistances2.push_back(dmin2);
        }
        //continue
        return true;
      }
      else
      {
        //split mesh BVH
        return _RecurseSplitMesh(pcOctreeNode, meshBVHNode);
      }
    }
    else
    {
      if (meshnode.Leaf())
      {
        //split octree node
        return _RecurseSplitOctree(pcOctreeNode, meshBVHNode);
      }
      else
      {
        //determine which BVH to split
        Real vpc = Volume(pcnode);
        Real vmesh = Volume(meshnode);
        if (vpc < vmesh)
          return _RecurseSplitMesh(pcOctreeNode, meshBVHNode);
        else
          return _RecurseSplitOctree(pcOctreeNode, meshBVHNode);
      }
    }
  }
  bool _RecurseSplitMesh(int pcOctreeNode, int meshBVHNode)
  {
    int c1 = mesh.pqpModel->b[meshBVHNode].first_child;
    int c2 = c1 + 1;
    if (!_Recurse(pcOctreeNode, c1))
      return false;
    if (!_Recurse(pcOctreeNode, c2))
      return false;
    return true;
  }
  bool _RecurseSplitOctree(int pcOctreeNode, int meshBVHNode)
  {
    const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
    for (int i = 0; i < 8; i++)
      if (!_Recurse(pcnode.childIndices[i], meshBVHNode))
        return false;
    return true;
  }
};

bool Collides(const CollisionPointCloud &a, Real margin, const CollisionMesh &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  PointMeshCollider collider(a, b, margin);
  bool res = collider.Recurse(maxContacts);
  if (res)
  {
    //do we want to filter by closest pairs?
    map<int, int> closest1, closest2;
    map<int, Real> dclosest1, dclosest2;
    for (size_t i = 0; i < collider.pcpoints.size(); i++)
    {
      if (closest1.count(collider.pcpoints[i]) == 0 || collider.pairdistances2[i] < dclosest1[collider.pcpoints[i]])
      {
        closest1[collider.pcpoints[i]] = collider.meshtris[i];
        dclosest1[collider.pcpoints[i]] = collider.pairdistances2[i];
      }
    }
    for (size_t i = 0; i < collider.meshtris.size(); i++)
    {
      if (closest2.count(collider.meshtris[i]) == 0 || collider.pairdistances2[i] < dclosest2[collider.meshtris[i]])
      {
        closest2[collider.meshtris[i]] = collider.pcpoints[i];
        dclosest2[collider.meshtris[i]] = collider.pairdistances2[i];
      }
    }
    elements1.resize(0);
    elements2.resize(0);
    for (const auto &c1 : closest1)
    {
      elements1.push_back(c1.first);
      elements2.push_back(c1.second);
    }
    for (const auto &c2 : closest2)
    {
      if (closest1[c2.second] != c2.first)
      {
        elements1.push_back(c2.second);
        elements2.push_back(c2.first);
      }
    }
    //Basic version
    //elements1=collider.pcpoints;
    //elements2=collider.meshtris;
    return true;
  }
  return false;
}

bool Collides(const CollisionPointCloud &a, Real margin, const CollisionPointCloud &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  return Geometry::Collides(a, b, margin, elements1, elements2, maxContacts);
}

bool Collides(const CollisionPointCloud &a, Real margin, const CollisionImplicitSurface &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  bool res = Geometry::Collides(b, a, margin, elements1, maxContacts);
  elements2.resize(elements1.size());
  for (size_t i = 0; i < elements1.size(); i++)
    elements2[i] = PointIndex(b, a.currentTransform * a.points[elements1[i]]);
  return res;
}

bool Collides(const GeometricPrimitive3D &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  Assert(b.CollisionDataInitialized());
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    if (::Collides(a, bw, margin + b.margin))
    {
      elements1.push_back(0);
      elements2.push_back(0);
      return true;
    }
    return false;
  }
  case AnyCollisionGeometry3D::ImplicitSurface:
    if (::Collides(b.ImplicitSurfaceCollisionData(), a, margin + b.margin, elements2, maxContacts))
    {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::TriangleMesh:
    if (::Collides(a, b.TriangleMeshCollisionData(), margin + b.margin, elements2, maxContacts))
    {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::PointCloud:
    if (::Collides(a, b.PointCloudCollisionData(), margin + b.margin, elements2, maxContacts))
    {
      elements1.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do primitive-convex hull collisions yet");
    break;
  case AnyCollisionGeometry3D::Group:
  {
    vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    return ::Collides(a, bitems, margin + b.margin, elements1, elements2, maxContacts);
  }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const GeometricPrimitive3D &a, const RigidTransform &Ta, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  if (a.type == GeometricPrimitive3D::Empty)
    return false;
  GeometricPrimitive3D aw = a;
  aw.Transform(Ta);
  return Collides(aw, margin, b, elements1, elements2, maxContacts);
}

bool Collides(const CollisionImplicitSurface &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    if (::Collides(a, bw, margin + b.margin, elements1, maxContacts))
    {
      elements2.push_back(0);
      return true;
    }
    return false;
  }
  case AnyCollisionGeometry3D::ImplicitSurface:
    return ::Collides(a, b.ImplicitSurfaceCollisionData(), margin + b.margin, elements1, elements2, maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(a, b.TriangleMeshCollisionData(), margin + b.margin, elements1, elements2, maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
  {
    const CollisionPointCloud &pc = b.PointCloudCollisionData();
    bool res = Geometry::Collides(a, pc, margin, elements2, maxContacts);
    elements1.resize(elements2.size());
    for (size_t i = 0; i < elements2.size(); i++)
      elements1[i] = PointIndex(a, b.currentTransform * pc.points[elements2[i]]);
    return res;
  }
  break;
  case AnyCollisionGeometry3D::Group:
  {
    vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    return ::Collides(a, bitems, margin + b.margin, elements1, elements2, maxContacts);
  }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const CollisionMesh &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    if (::Collides(bw, a, margin + b.margin, elements1, maxContacts))
    {
      elements2.push_back(0);
      return true;
    }
    return false;
  }
  case AnyCollisionGeometry3D::ImplicitSurface:
    return ::Collides(b.ImplicitSurfaceCollisionData(), a, margin + b.margin, elements2, elements1, maxContacts);
  case AnyCollisionGeometry3D::TriangleMesh:
    return ::Collides(a, b.TriangleMeshCollisionData(), margin + b.margin, elements1, elements2, maxContacts);
  case AnyCollisionGeometry3D::PointCloud:
    return ::Collides(b.PointCloudCollisionData(), margin + b.margin, a, elements2, elements1, maxContacts);
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do convex hull-anything collision yet");
    break;
  case AnyCollisionGeometry3D::Group:
  {
    vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    return ::Collides(a, bitems, margin + b.margin, elements1, elements2, maxContacts);
  }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const CollisionPointCloud &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    if (::Collides(bw, a, margin + b.margin, elements1, maxContacts))
    {
      elements2.push_back(0);
      return true;
    }
    return false;
  }
  case AnyCollisionGeometry3D::TriangleMesh:
  {
    bool res = ::Collides(a, margin, b.TriangleMeshCollisionData(), elements1, elements2, maxContacts);
    return res;
  }
  case AnyCollisionGeometry3D::PointCloud:
  {
    bool res = ::Collides(a, margin, b.PointCloudCollisionData(), elements1, elements2, maxContacts);
    return res;
  }
  case AnyCollisionGeometry3D::ImplicitSurface:
  {
    bool res = ::Collides(a, margin, b.ImplicitSurfaceCollisionData(), elements1, elements2, maxContacts);
    return res;
  }
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do point cloud-convex hull collisions yet");
    break;
  case AnyCollisionGeometry3D::Group:
  {
    vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    return ::Collides(a, bitems, margin + b.margin, elements1, elements2, maxContacts);
  }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(const CollisionConvexHull3D &a, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do convex hull-primitive collisions yet");
    return false;
  case AnyCollisionGeometry3D::TriangleMesh:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do convex hull-triangle mesh collisions yet");
    return false;
  case AnyCollisionGeometry3D::PointCloud:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do convex hull-point cloud collisions yet");
    return false;
  case AnyCollisionGeometry3D::ImplicitSurface:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do convex hull-implicit surface collisions yet");
    return false;
  case AnyCollisionGeometry3D::ConvexHull:
    if(a.Collides(b.ConvexHullCollisionData())) {
      elements1.push_back(0);
      elements2.push_back(0);
      return true;
    }
    return false;
  case AnyCollisionGeometry3D::Group:
  {
    vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    for(size_t i=0;i<bitems.size();i++) {
      vector<int> temp1,temp2;
      if(Collides(a,margin+b.margin,bitems[i],temp1,temp2,maxContacts)) {
        elements1.push_back(0);
        elements2.push_back(int(i));
      }
    }
    return !elements1.empty();
  }
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool Collides(vector<AnyCollisionGeometry3D> &group, Real margin, AnyCollisionGeometry3D &b,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  for (size_t i = 0; i < group.size(); i++)
  {
    vector<int> ei1, ei2;
    if (group[i].WithinDistance(b, margin, ei1, ei2, maxContacts - (int)elements1.size()))
    {
      for (size_t j = 0; j < ei1.size(); j++)
      {
        elements1.push_back((int)i);
        elements2.push_back((int)ei2[j]);
      }
      if (elements2.size() >= maxContacts)
        return true;
    }
  }
  return !elements2.empty();
}

bool AnyCollisionGeometry3D::Collides(AnyCollisionGeometry3D &geom)
{
  InitCollisionData();
  geom.InitCollisionData();
  vector<int> elem1, elem2;
  return Collides(geom, elem1, elem2, 1);
}

bool AnyCollisionGeometry3D::Collides(AnyCollisionGeometry3D &geom,
                                      vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  InitCollisionData();
  geom.InitCollisionData();
  //prioritize point cloud testing
  if (geom.type == PointCloud && type != PointCloud)
    return geom.Collides(*this, elements2, elements1, maxContacts);
  //otherwise...
  switch (type)
  {
  case Primitive:
    return ::Collides(AsPrimitive(), GetTransform(), margin, geom, elements1, elements2, maxContacts);
  case ImplicitSurface:
    return ::Collides(ImplicitSurfaceCollisionData(), margin, geom, elements1, elements2, maxContacts);
  case TriangleMesh:
    return ::Collides(TriangleMeshCollisionData(), margin, geom, elements1, elements2, maxContacts);
  case PointCloud:
    return ::Collides(PointCloudCollisionData(), margin, geom, elements1, elements2, maxContacts);
  case Group:
    return ::Collides(GroupCollisionData(), margin, geom, elements1, elements2, maxContacts);
  case ConvexHull:
    return ::Collides(ConvexHullCollisionData(), margin, geom, elements1, elements2, maxContacts);
    break;
  default:
    FatalError("Invalid type");
  }
  return false;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const GeometricPrimitive3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.elem1 = 0;
  res.elem2 = 0;
  res.hasPenetration = true;
  if (a.SupportsClosestPoints(b.type))
  {
    res.hasClosestPoints = true;
    res.hasDirections = true;
    res.d = a.ClosestPoints(b, res.cp1, res.dir1);
    SetCP2(res);
  }
  else
    res.d = a.Distance(b);
  return res;
}

AnyDistanceQueryResult Distance(const ConvexHull3D &a, const ConvexHull3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.elem1 = 0;
  res.elem2 = 0;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.d = a.ClosestPoints(b, res.cp1, res.dir1);
  SetCP2(res);
  return res;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const CollisionMesh &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.elem1 = 0;
  res.d = Geometry::Distance(b, a, res.elem2, res.cp2, res.dir2, settings.upperBound);
  res.dir1.setNegative(res.dir2);
  res.cp1 = res.cp2 + res.d * res.dir2;
  return res;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const CollisionImplicitSurface &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.elem1 = 0;
  res.d = Geometry::Distance(b, a, res.cp2, res.cp1, res.dir1);
  res.elem2 = PointIndex(b, res.cp2);
  res.dir2.setNegative(res.dir1);
  return res;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.elem1 = 0;
  res.d = Distance(b, a, res.elem2, settings.upperBound);
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.cp2 = b.currentTransform * b.points[res.elem2];
  a.ClosestPoints(res.cp2, res.cp1, res.dir1);
  res.dir2.setNegative(res.dir1);
  return res;
}

AnyDistanceQueryResult Distance(const CollisionImplicitSurface &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.hasElements = true;
  res.hasPenetration = true;
  res.hasClosestPoints = true;
  res.hasDirections = true;
  res.d = Geometry::Distance(a, b, res.elem2, settings.upperBound);
  res.cp2 = b.currentTransform * b.points[res.elem2];
  Geometry::Distance(a, res.cp2, res.cp1, res.dir1);
  res.dir2.setNegative(res.dir1);
  res.elem1 = PointIndex(a, res.cp1);
  return res;
}

AnyDistanceQueryResult Distance(const CollisionMesh &a, const CollisionMesh &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  CollisionMeshQuery q(a, b);
  res.d = q.Distance(settings.absErr, settings.relErr, settings.upperBound);
  q.ClosestPair(res.elem1, res.elem2);
  q.ClosestPoints(res.cp1, res.cp2);
  res.cp1 = a.currentTransform * res.cp1;
  res.cp2 = b.currentTransform * res.cp2;
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}

class PointMeshDistance
{
public:
  const CollisionPointCloud &pc;
  const CollisionMesh &mesh;
  RigidTransform Tba, Twa, Tab;
  PQP_REAL pqpRab[3][3], pqpTab[3];
  BV pcbv;
  Real absErr, relErr;
  Real upperBound;
  int pcpoint;
  int meshtri;
  Vector3 meshpoint; //note: local coordinates
  Heap<pair<int, int>, Real> queue;

  PointMeshDistance(const CollisionPointCloud &a, const CollisionMesh &b, Real _absErr, Real _relErr, Real _upperBound = Inf)
      : pc(a), mesh(b), absErr(_absErr), relErr(_relErr), upperBound(_upperBound), pcpoint(-1), meshtri(-1)
  {
    Twa.setInverse(a.currentTransform);
    Tba.mul(Twa, b.currentTransform);
    Tab.setInverse(Tba);
    //RigidTransformToPQP(Tab,pqpRab,pqpTab);
    RigidTransformToPQP(Tba, pqpRab, pqpTab);

    RigidTransform ident;
    ident.setIdentity();
    RigidTransformToPQP(ident, pcbv.R, pcbv.To);

    //start with initial upper bound by taking some arbitrary point
    Triangle3D tri;
    b.GetTriangle(b.tris.size() / 3, tri);
    Vector3 pt = Tab * pc.points[pc.points.size() / 3];
    Vector3 cp = tri.closestPoint(pt);
    Real d = cp.distance(pt);
    if (d < upperBound)
    {
      upperBound = d;
      pcpoint = int(pc.points.size() / 3);
      meshtri = int(b.tris.size() / 3);
      meshpoint = cp;
    }
  }
  Real Distance(const OctreeNode &pcnode, const BV &meshnode)
  {
    //Vector3 halfdims = (pcnode.bb.bmax-pcnode.bb.bmin)*0.5;
    //Vector3 center = (pcnode.bb.bmin+pcnode.bb.bmax)*0.5;
    //halfdims.get(pcbv.d);
    //center.get(pcbv.Tr);
    //PQP uses rectangle-swept-sphere to do distance calculation
    Vector3 d = pcnode.bb.bmax - pcnode.bb.bmin;
    pcbv.Tr[0] = pcnode.bb.bmin.x;
    pcbv.Tr[1] = pcnode.bb.bmin.y;
    pcbv.Tr[2] = (pcnode.bb.bmin.z + pcnode.bb.bmax.z) * 0.5;
    pcbv.l[0] = d.x;
    pcbv.l[1] = d.y;
    pcbv.r = d.z * 0.5;
    return BV_Distance2(pqpRab, pqpTab, &pcbv, &meshnode);
  }
  void UpdateLeaves(const OctreeNode &pcOctreeNode, const BV &meshnode)
  {
    int t = -meshnode.first_child - 1;
    Triangle3D tri;
    Copy(mesh.pqpModel->tris[t].p1, tri.a);
    Copy(mesh.pqpModel->tris[t].p2, tri.b);
    Copy(mesh.pqpModel->tris[t].p3, tri.c);
    tri.a = Tba * tri.a;
    tri.b = Tba * tri.b;
    tri.c = Tba * tri.c;
    //collide the triangle and points
    vector<Vector3> pts;
    vector<int> pcids;
    pc.octree->GetPoints(pcOctreeNode, pts);
    pc.octree->GetPointIDs(pcOctreeNode, pcids);
    Vector3 cp;
    for (size_t i = 0; i < pts.size(); i++)
    {
      cp = tri.closestPoint(pts[i]);
      Real d = pts[i].distance(cp);
      if (d < upperBound)
      {
        upperBound = d;
        pcpoint = pcids[i];
        meshtri = t;
        meshpoint = Tab * cp;
      }
    }
  }
  void Recurse()
  {
    int maxqueuesize = 100;
    queue.push(make_pair(0, 0), -Distance(pc.octree->Node(0), mesh.pqpModel->b[0]));
    while (!queue.empty())
    {
      if (-queue.topPriority() * (1 + relErr) + absErr >= upperBound)
        break;
      pair<int, int> next = queue.top();
      queue.pop();
      int pcOctreeNode = next.first;
      int meshBVHNode = next.second;

      if (queue.size() >= maxqueuesize)
      {
        Recurse(pcOctreeNode, meshBVHNode);
        continue;
      }

      const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
      const BV &meshnode = mesh.pqpModel->b[meshBVHNode];
      if (pc.octree->IsLeaf(pcnode))
      {
        if (pc.octree->NumPoints(pcnode) == 0)
          continue;
        if (meshnode.Leaf())
        {
          UpdateLeaves(pcnode, meshnode);
          continue;
        }
      }
      bool splitPC = true;
      if (pc.octree->IsLeaf(pcnode))
        splitPC = false;
      else if (!meshnode.Leaf())
      {
        //determine which BVH to split
        Real vpc = Volume(pcnode);
        Real vmesh = Volume(meshnode);
        if (vpc * 10 < vmesh)
          splitPC = false;
      }
      if (splitPC)
      {
        for (int i = 0; i < 8; i++)
        {
          const auto &child = pc.octree->Node(pcnode.childIndices[i]);
          if (child.bb.bmin.x > child.bb.bmax.x)
            continue;
          Real d = Distance(child, meshnode);
          if (d * (1.0 + relErr) + absErr < upperBound)
          {
            queue.push(pair<int, int>(pcnode.childIndices[i], meshBVHNode), -d);
          }
        }
      }
      else
      {
        int c1 = mesh.pqpModel->b[meshBVHNode].first_child;
        int c2 = c1 + 1;
        assert(c1 >= 0);
        Real d1 = Distance(pcnode, mesh.pqpModel->b[c1]);
        Real d2 = Distance(pcnode, mesh.pqpModel->b[c2]);
        if (d1 * (1.0 + relErr) + absErr < upperBound)
        {
          queue.push(pair<int, int>(pcOctreeNode, c1), -d1);
        }
        if (d2 * (1.0 + relErr) + absErr < upperBound)
        {
          queue.push(pair<int, int>(pcOctreeNode, c2), -d2);
        }
      }
    }
  }
  void Recurse(int pcOctreeNode, int meshBVHNode)
  {
    const OctreeNode &pcnode = pc.octree->Node(pcOctreeNode);
    const BV &meshnode = mesh.pqpModel->b[meshBVHNode];
    if (pc.octree->IsLeaf(pcnode))
    {
      if (pc.octree->NumPoints(pcnode) == 0)
        return;
      if (meshnode.Leaf())
      {
        UpdateLeaves(pcnode, meshnode);
        return;
      }
    }
    bool splitPC = true;
    if (pc.octree->IsLeaf(pcnode))
      splitPC = false;
    else if (!meshnode.Leaf())
    {
      //determine which BVH to split
      Real vpc = Volume(pcnode);
      Real vmesh = Volume(meshnode);
      if (vpc * 10 < vmesh)
        splitPC = false;
    }
    if (splitPC)
    {
      vector<pair<Real, int>> sorter;
      for (int i = 0; i < 8; i++)
      {
        const auto &child = pc.octree->Node(pcnode.childIndices[i]);
        if (child.bb.bmin.x > child.bb.bmax.x)
          continue;
        Real d = Distance(child, meshnode);
        if (d * (1.0 + relErr) + absErr < upperBound)
        {
          sorter.push_back(make_pair(d, pcnode.childIndices[i]));
        }
      }
      sort(sorter.begin(), sorter.end());
      for (const auto &order : sorter)
      {
        if (order.first * (1.0 + relErr) + absErr < upperBound)
          Recurse(order.second, meshBVHNode);
        else
          break;
      }
    }
    else
    {
      int c1 = mesh.pqpModel->b[meshBVHNode].first_child;
      int c2 = c1 + 1;
      assert(c1 >= 0);
      Real d1 = Distance(pcnode, mesh.pqpModel->b[c1]);
      Real d2 = Distance(pcnode, mesh.pqpModel->b[c2]);
      if (d1 < d2)
      {
        if (d1 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c1);
        }
        if (d2 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c2);
        }
      }
      else
      {
        if (d2 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c2);
        }
        if (d1 * (1.0 + relErr) + absErr < upperBound)
        {
          Recurse(pcOctreeNode, c1);
        }
      }
    }
  }
};

AnyDistanceQueryResult Distance(const CollisionPointCloud &a, const CollisionMesh &b, const AnyDistanceQuerySettings &settings)
{
  PointMeshDistance solver(a, b, settings.absErr, settings.relErr, settings.upperBound);
  solver.Recurse();
  AnyDistanceQueryResult res;
  res.d = solver.upperBound;
  res.elem1 = solver.pcpoint;
  res.elem2 = solver.meshtri;
  res.cp1 = a.currentTransform * a.points[solver.pcpoint];
  res.cp2 = b.currentTransform * solver.meshpoint;
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}

AnyDistanceQueryResult Distance(const CollisionPointCloud &a, const CollisionPointCloud &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  res.d = Geometry::Distance(a, b, res.elem1, res.elem2, settings.upperBound);
  res.cp1 = a.currentTransform * a.points[res.elem1];
  res.cp2 = b.currentTransform * b.points[res.elem2];
  res.hasElements = true;
  res.hasClosestPoints = true;
  return res;
}

//advance declarations
AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings);
AnyDistanceQueryResult Distance(const CollisionImplicitSurface &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings);
AnyDistanceQueryResult Distance(const CollisionMesh &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings);
AnyDistanceQueryResult Distance(const CollisionPointCloud &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings);

//note modifies settings
template <class T>
AnyDistanceQueryResult Distance_Group(const T &a, const vector<AnyCollisionGeometry3D> &bitems, AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  for (size_t i = 0; i < bitems.size(); i++)
  {
    AnyDistanceQueryResult ires = ::Distance(a, bitems[i], settings);
    if (ires.d < res.d)
    {
      res = ires;
      PushGroup2(res, (int)i);
      settings.upperBound = res.d;
    }
  }
  return res;
}

AnyDistanceQueryResult Distance(const ConvexHull3D &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  Assert(b.type == AnyCollisionGeometry3D::ConvexHull);
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  ConvexHull3D bw = b.AsConvexHull();
  bw.Transform(b.GetTransform());
  res = Distance(a, bw, modsettings);
  Offset2(res, b.margin);
  return res;
}

AnyDistanceQueryResult Distance(const GeometricPrimitive3D &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  if (a.type == GeometricPrimitive3D::Empty)
    return res;
  Assert(b.CollisionDataInitialized());
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    res = Distance(a, bw, modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ImplicitSurface:
  {
    res = Distance(a, b.ImplicitSurfaceCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::TriangleMesh:
  {
    res = Distance(a, b.TriangleMeshCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::PointCloud:
  {
    res = Distance(a, b.PointCloudCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do primitive-convex hull distance yet");
    break;
  case AnyCollisionGeometry3D::Group:
  {
    const vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    res = Distance_Group(a, bitems, modsettings);
    Offset2(res, b.margin);
  }
  break;
  default:
    FatalError("Invalid type");
  }
  return res;
}

AnyDistanceQueryResult Distance(const CollisionImplicitSurface &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    res = Distance(bw, a, modsettings);
    Flip(res);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ImplicitSurface:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Unable to do implicit surface/implicit surface distance yet");
    break;
  case AnyCollisionGeometry3D::TriangleMesh:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Unable to do implicit surface/triangle mesh distance yet");
    break;
  case AnyCollisionGeometry3D::PointCloud:
  {
    res = Distance(a, b.PointCloudCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do implicit surface-convex hull distance yet");
    break;
  case AnyCollisionGeometry3D::Group:
  {
    const vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    res = ::Distance_Group(a, bitems, modsettings);
    Offset2(res, b.margin);
  }
  break;
  default:
    FatalError("Invalid type");
  }
  return res;
}

AnyDistanceQueryResult Distance(const CollisionMesh &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    res = Distance(bw, a, modsettings);
    Flip(res);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ImplicitSurface:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Unable to do triangle mesh/implicit surface distance yet");
    break;
  case AnyCollisionGeometry3D::TriangleMesh:
  {
    res = Distance(a, b.TriangleMeshCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::PointCloud:
  {
    res = Distance(b.PointCloudCollisionData(), a, modsettings);
    Flip(res);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do mesh-convex hull distance yet");
    break;
  case AnyCollisionGeometry3D::Group:
  {
    const vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    res = ::Distance_Group(a, bitems, modsettings);
    Offset2(res, b.margin);
  }
  break;
  default:
    FatalError("Invalid type");
  }
  return res;
}

AnyDistanceQueryResult Distance(const CollisionPointCloud &a, const AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  modsettings.upperBound += b.margin;
  switch (b.type)
  {
  case AnyCollisionGeometry3D::Primitive:
  {
    GeometricPrimitive3D bw = b.AsPrimitive();
    bw.Transform(b.GetTransform());
    res = ::Distance(bw, a, modsettings);
    Flip(res);
    Offset2(res, b.margin);
    //return ::Distance(bw,a,elem1) - (margin+b.margin);
  }
  break;
  case AnyCollisionGeometry3D::TriangleMesh:
  {
    res = Distance(a, b.TriangleMeshCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::PointCloud:
  {
    res = ::Distance(a, b.PointCloudCollisionData(), modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ImplicitSurface:
  {
    res = ::Distance(b.ImplicitSurfaceCollisionData(), a, modsettings);
    Flip(res);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::Group:
  {
    const vector<AnyCollisionGeometry3D> &bitems = b.GroupCollisionData();
    res = ::Distance_Group(a, bitems, modsettings);
    Offset2(res, b.margin);
  }
  break;
  case AnyCollisionGeometry3D::ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do point cloud-convex hull distance yet");
    break;
  default:
    FatalError("Invalid type");
  }
  return res;
}

AnyDistanceQueryResult Distance(vector<AnyCollisionGeometry3D> &group, AnyCollisionGeometry3D &b, const AnyDistanceQuerySettings &settings)
{
  AnyDistanceQueryResult res;
  AnyDistanceQuerySettings modsettings = settings;
  for (size_t i = 0; i < group.size(); i++)
  {
    AnyDistanceQueryResult ires = group[i].Distance(b, modsettings);
    if (ires.d < res.d)
    {
      res = ires;
      PushGroup1(res, int(i));
      modsettings.upperBound = res.d;
    }
  }
  return res;
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
  modsettings.upperBound += margin;
  switch (type)
  {
  case Primitive:
  {
    GeometricPrimitive3D aw = AsPrimitive();
    aw.Transform(GetTransform());
    result = ::Distance(aw, geom, modsettings);
    Offset1(result, margin);
    return result;
  }
  case ImplicitSurface:
    result = ::Distance(ImplicitSurfaceCollisionData(), geom, modsettings);
    Offset1(result, margin);
    return result;
  case TriangleMesh:
    result = ::Distance(TriangleMeshCollisionData(), geom, modsettings);
    Offset1(result, margin);
    return result;
  case PointCloud:
    result = ::Distance(PointCloudCollisionData(), geom, modsettings);
    Offset1(result, margin);
    return result;
  case Group:
    result = ::Distance(GroupCollisionData(), geom, modsettings);
    Offset1(result, margin);
    return result;
  case ConvexHull: 
  {
    if(geom.type != ConvexHull) {
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't compute distance for convex hulls to anything else yet");
      return AnyDistanceQueryResult();
    }
    else {
      // use AABB to exclude them
      AABB3D aabb1 = this->GetAABB();
      AABB3D aabb2 = geom.GetAABB();
      // std::cout << "BBox1 " << aabb1.bmin.x << " " << aabb1.bmin.y << " " << aabb1.bmin.z << " and " << aabb1.bmax.x << " " << aabb1.bmax.y << " " << aabb1.bmax.z << "\n";
      // std::cout << "BBox2 " << aabb2.bmin.x << " " << aabb2.bmin.y << " " << aabb2.bmin.z << " and " << aabb2.bmax.x << " " << aabb2.bmax.y << " " << aabb2.bmax.z << "\n";
      double d = aabb1.distance(aabb2);
      // std::cout << "d = " << d << std::endl;
      if(d > modsettings.upperBound) {
        result.d = d;
        result.hasElements = false;
        result.hasClosestPoints = false;
        result.hasPenetration = false;
        result.hasDirections = false;
        return result;
      }
      CollisionConvexHull3D &hull1 = this->ConvexHullCollisionData();
      CollisionConvexHull3D &hull2 = geom.ConvexHullCollisionData();
      Vector3 cp, direction;
      Real dist = hull1.ClosestPoints(hull2, cp, direction);
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
      Offset1(result, margin);
      return result;
    }
  }
  default:
    FatalError("Invalid type");
  }
  return result;
}

bool AnyCollisionGeometry3D::WithinDistance(AnyCollisionGeometry3D &geom, Real tol)
{
  InitCollisionData();
  geom.InitCollisionData();
  vector<int> elem1, elem2;
  return WithinDistance(geom, tol, elem1, elem2, 1);
}

bool AnyCollisionGeometry3D::WithinDistance(AnyCollisionGeometry3D &geom, Real tol,
                                            vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
  InitCollisionData();
  geom.InitCollisionData();
  switch (type)
  {
  case Primitive:
    return ::Collides(AsPrimitive(), GetTransform(), margin + tol, geom, elements1, elements2, maxContacts);
  case ImplicitSurface:
    return ::Collides(ImplicitSurfaceCollisionData(), margin + tol, geom, elements1, elements2, maxContacts);
  case TriangleMesh:
    return ::Collides(TriangleMeshCollisionData(), margin + tol, geom, elements1, elements2, maxContacts);
  case PointCloud:
    return ::Collides(PointCloudCollisionData(), margin + tol, geom, elements1, elements2, maxContacts);
  case Group:
    return ::Collides(GroupCollisionData(), margin + tol, geom, elements1, elements2, maxContacts);
  case ConvexHull:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't do within-distance for convex hulls yet");
    break;
  default:
    FatalError("Invalid type");
  }
  return false;
}

bool AnyCollisionGeometry3D::RayCast(const Ray3D &r, Real *distance, int *element)
{
  InitCollisionData();
  switch (type)
  {
  case Primitive:
  {
    RigidTransform T = PrimitiveCollisionData(), Tinv;
    Tinv.setInverse(T);
    Ray3D rlocal;
    rlocal.setTransformed(r, Tinv);
    Vector3 localpt;
    if (AsPrimitive().RayCast(rlocal, localpt))
    {
      if (distance)
      {
        *distance = localpt.distance(rlocal.source);
        //TODO: this isn't perfect if the margin is > 0 -- will miss silouettes
        *distance -= margin;
      }
      if (element)
        *element = 0;
      return true;
    }
    return false;
  }
  case ImplicitSurface:
  {
    const auto& surf = ImplicitSurfaceCollisionData();
    Real dist = ::RayCast(surf,r,margin);
    if(!IsInf(dist)) {
      if(distance) *distance = dist;
      if(element) *element = PointIndex(surf,r.source+dist*r.direction);
      return true;
    }
    return false;
  }
  case TriangleMesh:
  {
    Vector3 worldpt;
    int tri = ::RayCast(TriangleMeshCollisionData(), r, worldpt);
    if (tri >= 0)
    {
      if (distance)
      {
        *distance = worldpt.distance(r.source);
        //TODO: this isn't perfect if the margin is > 0 -- will miss silhouettes
        *distance -= margin;
      }
      if (element)
        *element = tri;
      return true;
    }
    return false;
  }
  case PointCloud:
  {
    const CollisionPointCloud &pc = PointCloudCollisionData();
    Vector3 pt;
    int res = ::RayCast(pc, margin, r, pt);
    if (res < 0)
      return false;
    if (distance)
    {
      Vector3 temp;
      *distance = r.closestPoint(pt, temp);
    }
    if (element)
      *element = res;
    return true;
  }
  case Group:
  {
    vector<AnyCollisionGeometry3D> &items = GroupCollisionData();
    Real closest = Inf;
    for (size_t i = 0; i < items.size(); i++)
    {
      Real d;
      int elem;
      if (items[i].RayCast(r, &d, &elem))
      {
        if (d < closest)
        {
          closest = d;
          if (element)
            *element = (int)i;
        }
      }
    }
    if (distance)
      *distance = closest;
    return !IsInf(closest);
  }
  case ConvexHull:
  {
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "Can't ray-cast convex hull yet");
    break;
  }
  }
  return false;
}

typedef AnyContactsQueryResult::ContactPair ContactPair;

//if a normal has this length then it is ignored
const static Real gZeroNormalTolerance = 1e-4;

//if two contact points are closer than this threshold, will try to look
//at the local geometry to derive a contact normal
const static Real gNormalFromGeometryTolerance = 1e-5;
//const static Real gNormalFromGeometryTolerance = 1e-2;

//if a barycentric coordinate is within this tolerance of zero, it will be
//considered a zero
const static Real gBarycentricCoordZeroTolerance = 1e-3;

//if true, takes the ODE tolerance points and performs additional contact
//checking -- useful for flat contacts
const static bool gDoTriangleTriangleCollisionDetection = false;

//doesn't consider unique contact points if they are between this tolerance
const static Real cptol = 1e-5;

void ReverseContact(ContactPair &contact)
{
  std::swap(contact.p1, contact.p2);
  contact.n.inplaceNegative();
  std::swap(contact.elem1, contact.elem2);
}

//1 = pt, 2 = edge, 3 = face
inline int FeatureType(const Vector3 &b)
{
  int type = 0;
  if (FuzzyZero(b.x, gBarycentricCoordZeroTolerance))
    type++;
  if (FuzzyZero(b.y, gBarycentricCoordZeroTolerance))
    type++;
  if (FuzzyZero(b.z, gBarycentricCoordZeroTolerance))
    type++;
  return 3 - type;
}

int EdgeIndex(const Vector3 &b)
{
  if (FuzzyZero(b.x, gBarycentricCoordZeroTolerance))
    return 0;
  if (FuzzyZero(b.y, gBarycentricCoordZeroTolerance))
    return 1;
  if (FuzzyZero(b.z, gBarycentricCoordZeroTolerance))
    return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

int VertexIndex(const Vector3 &b)
{
  if (FuzzyEquals(b.x, One, gBarycentricCoordZeroTolerance))
    return 0;
  if (FuzzyEquals(b.y, One, gBarycentricCoordZeroTolerance))
    return 1;
  if (FuzzyEquals(b.z, One, gBarycentricCoordZeroTolerance))
    return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

Vector3 VertexNormal(CollisionMesh &m, int tri, int vnum)
{
  if (m.incidentTris.empty())
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "VertexNormal: mesh is not properly initialized with incidentTris array?");
    m.CalcIncidentTris();
    //return Vector3(0.0);
    //FatalError("VertexNormal: mesh is not properly initialized with incidentTris array?");
  }
  Assert(vnum >= 0 && vnum < 3);
  int v = m.tris[tri][vnum];
  Assert(v >= 0 && v < (int)m.incidentTris.size());
  if (m.incidentTris[v].empty())
    return Vector3(0.0);
  Vector3 n(Zero);
  for (size_t i = 0; i < m.incidentTris[v].size(); i++)
    n += m.TriangleNormal(m.incidentTris[v][i]);
  n.inplaceNormalize();
  return m.currentTransform.R * n;
}

Vector3 EdgeNormal(CollisionMesh &m, int tri, int e)
{
  if (m.triNeighbors.empty())
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "EdgeNormal: Warning, mesh is not properly initialized with triNeighbors");
    m.CalcTriNeighbors();
    //return Vector3(0.0);
  }
  Assert(!m.triNeighbors.empty());
  Vector3 n = m.TriangleNormal(tri);
  if (m.triNeighbors[tri][e] != -1)
  {
    n += m.TriangleNormal(m.triNeighbors[tri][e]);
    n.inplaceNormalize();
  }
  return m.currentTransform.R * n;
}

///Compute normal from mesh geometry: returns the local normal needed for
///triangle 1 on m1 to get out of triangle 2 on m2.
///p1 and p2 are given in local coordinates
Vector3 ContactNormal(CollisionMesh &m1, CollisionMesh &m2, const Vector3 &p1, const Vector3 &p2, int t1, int t2)
{
  Triangle3D tri1, tri2;
  m1.GetTriangle(t1, tri1);
  m2.GetTriangle(t2, tri2);
  Vector3 b1 = tri1.barycentricCoords(p1);
  Vector3 b2 = tri2.barycentricCoords(p2);
  int type1 = FeatureType(b1), type2 = FeatureType(b2);
  switch (type1)
  {
  case 1: //pt
    switch (type2)
    {
    case 1: //pt
      //get the triangle normals
      {
        //printf("ODECustomMesh: Point-point contact\n");
        Vector3 n1 = VertexNormal(m1, t1, VertexIndex(b1));
        Vector3 n2 = VertexNormal(m2, t2, VertexIndex(b2));
        n2 -= n1;
        n2.inplaceNormalize();
        return n2;
      }
      break;
    case 2: //edge
    {
      //printf("ODECustomMesh: Point-edge contact\n");
      Vector3 n1 = VertexNormal(m1, t1, VertexIndex(b1));
      int e = EdgeIndex(b2);
      Segment3D s = tri2.edge(e);
      Vector3 ev = m2.currentTransform.R * (s.b - s.a);
      Vector3 n2 = EdgeNormal(m2, t2, e);
      n2 -= (n1 - ev * ev.dot(n1) / ev.dot(ev)); //project onto normal
      n2.inplaceNormalize();
      return n2;
    }
    break;
    case 3: //face
      return m2.currentTransform.R * tri2.normal();
    }
    break;
  case 2: //edge
    switch (type2)
    {
    case 1: //pt
    {
      //printf("ODECustomMesh: Edge-point contact\n");
      Vector3 n2 = VertexNormal(m2, t2, VertexIndex(b2));
      int e = EdgeIndex(b1);
      Segment3D s = tri1.edge(e);
      Vector3 ev = m1.currentTransform.R * (s.b - s.a);
      Vector3 n1 = EdgeNormal(m1, t1, e);
      n2 = (n2 - ev * ev.dot(n2) / ev.dot(ev)) - n1; //project onto normal
      n2.inplaceNormalize();
      return n2;
    }
    break;
    case 2: //edge
    {
      //printf("ODECustomMesh: Edge-edge contact\n");
      int e = EdgeIndex(b1);
      Segment3D s1 = tri1.edge(e);
      Vector3 ev1 = m1.currentTransform.R * (s1.b - s1.a);
      ev1.inplaceNormalize();
      e = EdgeIndex(b2);
      Segment3D s2 = tri2.edge(e);
      Vector3 ev2 = m2.currentTransform.R * (s2.b - s2.a);
      ev2.inplaceNormalize();
      Vector3 n;
      n.setCross(ev1, ev2);
      Real len = n.length();
      if (len < gZeroNormalTolerance)
      {
        //hmm... edges are parallel?
      }
      n /= len;
      //make sure the normal direction points into m1 and out of m2
      if (n.dot(m1.currentTransform * s1.a) < n.dot(m2.currentTransform * s2.a))
        n.inplaceNegative();
      /*
        if(n.dot(m1.currentTransform.R*tri1.normal()) > 0.0) {
          if(n.dot(m2.currentTransform.R*tri2.normal()) > 0.0) {
            printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
          }
          n.inplaceNegative();
        }
        else {
          if(n.dot(m2.currentTransform.R*tri2.normal()) < 0.0) {
            printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
          }
        }
        */
      //cout<<"Edge vector 1 "<<ev1<<", vector 2" <<ev2<<", normal: "<<n<<endl;
      return n;
    }
    break;
    case 3: //face
      return m2.currentTransform.R * tri2.normal();
    }
    break;
  case 3: //face
    if (type2 == 3)
    {
      //printf("ODECustomMesh: Warning, face-face contact?\n");
    }
    return m1.currentTransform.R * (-tri1.normal());
  }
  static int warnedCount = 0;
  if (warnedCount % 10000 == 0)
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Warning, degenerate triangle, types " << type1 << " " << type2);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

//Returns a contact normal for the closest point to the triangle t.  p is the point on the triangle.
//The direction is the one in which triangle t can move to get away from closestpt
Vector3 ContactNormal(CollisionMesh &m, const Vector3 &p, int t, const Vector3 &closestPt)
{
  Triangle3D tri;
  m.GetTriangle(t, tri);
  Vector3 b = tri.barycentricCoords(p);
  int type = FeatureType(b);
  switch (type)
  {
  case 1: //pt
    //get the triangle normal
    {
      Vector3 n = VertexNormal(m, t, VertexIndex(b));
      n.inplaceNegative();
      return n;
    }
    break;
  case 2: //edge
  {
    int e = EdgeIndex(b);
    Vector3 n = EdgeNormal(m, t, e);
    n.inplaceNegative();
    return n;
  }
  break;
  case 3: //face
    return m.currentTransform.R * (-tri.normal());
  }
  static int warnedCount = 0;
  if (warnedCount % 10000 == 0)
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Warning, degenerate triangle, types " << type);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

void MeshMeshContacts(CollisionMesh &m1, Real outerMargin1, CollisionMesh &m2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  CollisionMeshQuery q(m1, m2);
  bool res = q.WithinDistanceAll(outerMargin1 + outerMargin2);
  if (!res)
  {
    return;
  }

  vector<int> t1, t2;
  vector<Vector3> cp1, cp2;
  q.TolerancePairs(t1, t2);
  q.TolerancePoints(cp1, cp2);
  vector<bool> unreliable(t1.size(), false);

  const RigidTransform &T1 = m1.currentTransform;
  const RigidTransform &T2 = m2.currentTransform;
  RigidTransform T21;
  T21.mulInverseA(T1, T2);
  RigidTransform T12;
  T12.mulInverseA(T2, T1);
  Real tol = outerMargin1 + outerMargin2;
  Real tol2 = Sqr(tol);

  size_t imax = t1.size();
  Triangle3D tri1, tri2, tri1loc, tri2loc;
  if (gDoTriangleTriangleCollisionDetection)
  {
    //test if more triangle vertices are closer than tolerance
    for (size_t i = 0; i < imax; i++)
    {
      m1.GetTriangle(t1[i], tri1);
      m2.GetTriangle(t2[i], tri2);

      tri1loc.a = T12 * tri1.a;
      tri1loc.b = T12 * tri1.b;
      tri1loc.c = T12 * tri1.c;
      tri2loc.a = T21 * tri2.a;
      tri2loc.b = T21 * tri2.b;
      tri2loc.c = T21 * tri2.c;
      bool usecpa, usecpb, usecpc, usecpa2, usecpb2, usecpc2;
      Vector3 cpa = tri1.closestPoint(tri2loc.a);
      Vector3 cpb = tri1.closestPoint(tri2loc.b);
      Vector3 cpc = tri1.closestPoint(tri2loc.c);
      Vector3 cpa2 = tri2.closestPoint(tri1loc.a);
      Vector3 cpb2 = tri2.closestPoint(tri1loc.b);
      Vector3 cpc2 = tri2.closestPoint(tri1loc.c);
      usecpa = (cpa.distanceSquared(tri2loc.a) < tol2);
      usecpb = (cpb.distanceSquared(tri2loc.b) < tol2);
      usecpc = (cpc.distanceSquared(tri2loc.c) < tol2);
      usecpa2 = (cpa2.distanceSquared(tri1loc.a) < tol2);
      usecpb2 = (cpb2.distanceSquared(tri1loc.b) < tol2);
      usecpc2 = (cpc2.distanceSquared(tri1loc.c) < tol2);
      //if already existing, disable it
      if (usecpa && cpa.isEqual(cp1[i], cptol))
        usecpa = false;
      if (usecpb && cpb.isEqual(cp1[i], cptol))
        usecpb = false;
      if (usecpc && cpc.isEqual(cp1[i], cptol))
        usecpc = false;
      if (usecpa2 && cpa2.isEqual(cp2[i], cptol))
        usecpa2 = false;
      if (usecpb2 && cpb2.isEqual(cp2[i], cptol))
        usecpb2 = false;
      if (usecpc2 && cpc2.isEqual(cp2[i], cptol))
        usecpc2 = false;

      if (usecpa)
      {
        if (usecpb && cpb.isEqual(cpa, cptol))
          usecpb = false;
        if (usecpc && cpc.isEqual(cpa, cptol))
          usecpc = false;
      }
      if (usecpb)
      {
        if (usecpc && cpc.isEqual(cpb, cptol))
          usecpc = false;
      }
      if (usecpa2)
      {
        if (usecpb2 && cpb2.isEqual(cpa2, cptol))
          usecpb2 = false;
        if (usecpc2 && cpc2.isEqual(cpa2, cptol))
          usecpc2 = false;
      }
      if (usecpb)
      {
        if (usecpc2 && cpc.isEqual(cpb2, cptol))
          usecpc2 = false;
      }

      if (usecpa)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpa);
        cp2.push_back(tri2.a);
      }
      if (usecpb)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpb);
        cp2.push_back(tri2.b);
      }
      if (usecpc)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpc);
        cp2.push_back(tri2.c);
      }
      if (usecpa2)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.a);
        cp2.push_back(cpa2);
      }
      if (usecpb2)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.b);
        cp2.push_back(cpb2);
      }
      if (usecpc2)
      {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.c);
        cp2.push_back(cpc2);
      }
    }
    /*
    if(t1.size() != imax)
      printf("ODECustomMesh: Triangle vert checking added %d points\n",t1.size()-imax);
    */
    //getchar();
  }

  imax = t1.size();
  for (size_t i = 0; i < imax; i++)
  {
    m1.GetTriangle(t1[i], tri1);
    m2.GetTriangle(t2[i], tri2);

    //tri1loc.a = T12*tri1.a;
    //tri1loc.b = T12*tri1.b;
    //tri1loc.c = T12*tri1.c;
    tri2loc.a = T21 * tri2.a;
    tri2loc.b = T21 * tri2.b;
    tri2loc.c = T21 * tri2.c;
    Segment3D s;
    //this is here to avoid degenerate triangles
    bool collides;
    Vector3 n1, n2;
    n1.setCross(tri1.b - tri1.a, tri1.c - tri1.a);
    n2.setCross(tri2.b - tri2.a, tri2.c - tri2.a);
    if (n2.normSquared() > n1.normSquared())
      collides = tri2loc.intersects(tri1, s);
    else
      collides = tri1.intersects(tri2loc, s);
    if (collides)
    {
      unreliable[i] = true;
      /*
      cout<<"Triangle 1"<<endl;
      cout<<"  "<<tri1.a<<endl;
      cout<<"  "<<tri1.b<<endl;
      cout<<"  "<<tri1.c<<endl;
      cout<<"intersects triangle 2"<<endl;
      cout<<"  "<<tri2loc.a<<endl;
      cout<<"  "<<tri2loc.b<<endl;
      cout<<"  "<<tri2loc.c<<endl;
      */
      /*
      //the two triangles intersect! can't trust results of PQP
      t1[i] = t1.back();
      t2[i] = t2.back();
      cp1[i] = cp1.back();
      cp2[i] = cp2.back();
      i--;
      imax--;
      */
    }
  }
  if (t1.size() != imax)
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), t1.size() - imax << " candidate points were removed due to mesh collision");
    t1.resize(imax);
    t2.resize(imax);
    cp1.resize(imax);
    cp2.resize(imax);
  }

  contacts.reserve(cp1.size());
  for (size_t i = 0; i < cp1.size(); i++)
  {
    Vector3 p1 = T1 * cp1[i];
    Vector3 p2 = T2 * cp2[i];
    Vector3 n = p2 - p1;
    Real d = n.norm();
    if (d < gNormalFromGeometryTolerance)
    { //compute normal from the geometry
      n = ContactNormal(m1, m2, cp1[i], cp2[i], t1[i], t2[i]);
      n.inplaceNegative();
    }
    else if (d > tol)
    { //some penetration -- we can't trust the result of PQP
      LOG4CXX_WARN(GET_LOGGER(Geometry), "Skipping contact due to irregular distance between points " << d);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  cp 1 " << p1);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  cp 2 " << p2);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  local cp 1 " << cp1[i]);
      LOG4CXX_WARN(GET_LOGGER(Geometry), "  local cp 2 " << cp2[i]);
      continue;
    }
    else
      n /= d;
    //check for invalid normals
    Real len = n.length();
    if (len < gZeroNormalTolerance || !IsFinite(len))
    {
      LOG4CXX_WARN(GET_LOGGER(Geometry), "Skipping contact due to irregular normal length " << len);
      continue;
    }
    //cout<<"Local Points "<<cp1[i]<<", "<<cp2[i]<<endl;
    //cout<<"Points "<<p1<<", "<<p2<<endl;
    //Real utol = (tol)*0.5/d + 0.5;
    //CopyVector(contact[k].pos,p1+utol*(p2-p1));
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = p1 + outerMargin1 * n;
    contacts[k].p2 = p2 - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    if (contacts[k].depth < 0)
      contacts[k].depth = 0;
    contacts[k].elem1 = t1[i];
    contacts[k].elem2 = t2[i];
    contacts[k].unreliable = unreliable[i];
    //cout<<"Normal "<<n<<", depth "<<contact[i].depth<<endl;
    //getchar();
  }
}

void MeshPointCloudContacts(CollisionMesh &m1, Real outerMargin1, CollisionPointCloud &pc2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  vector<int> tris;
  if (!Collides(pc2, tol, m1, points, tris, maxcontacts))
    return;
  Assert(points.size() == tris.size());
  Triangle3D tri, triw;
  contacts.reserve(points.size());
  for (size_t i = 0; i < points.size(); i++)
  {
    Vector3 pw = pc2.currentTransform * pc2.points[points[i]];
    m1.GetTriangle(tris[i], tri);
    triw.a = m1.currentTransform * tri.a;
    triw.b = m1.currentTransform * tri.b;
    triw.c = m1.currentTransform * tri.c;
    Vector3 cp = triw.closestPoint(pw);
    Vector3 n = pw - cp;
    Real d = n.length();
    if (d < gNormalFromGeometryTolerance)
    { //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp, plocal);
      n = ContactNormal(m1, plocal, tris[i], pw);
      n.inplaceNegative();
    }
    else if (d > tol)
    { //some penetration -- we can't trust the result of PQP
      continue;
    }
    else
      n /= d;
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = cp + outerMargin1 * n;
    contacts[k].p2 = pw - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = tris[i];
    contacts[k].elem2 = points[i];
    contacts[k].unreliable = false;
  }
  /*
  Real tol = outerMargin1 + outerMargin2;
  Box3D mbb,mbb_pclocal;
  GetBB(m1,mbb);
  RigidTransform Tw_pc;
  Tw_pc.setInverse(pc2.currentTransform);
  mbb_pclocal.setTransformed(mbb,Tw_pc);
  AABB3D maabb_pclocal;
  mbb_pclocal.getAABB(maabb_pclocal);
  maabb_pclocal.bmin -= Vector3(tol);
  maabb_pclocal.bmax += Vector3(tol);
  maabb_pclocal.setIntersection(pc2.bblocal);
  list<void*> nearpoints;
  pc2.grid.BoxItems(Vector(3,maabb_pclocal.bmin),Vector(3,maabb_pclocal.bmax),nearpoints);
  int k=0;
  vector<int> tris;
  Triangle3D tri,triw;
  for(list<void*>::iterator i=nearpoints.begin();i!=nearpoints.end();i++) {
    Vector3 pcpt = *reinterpret_cast<Vector3*>(*i);
    Vector3 pw = pc2.currentTransform*pcpt;
    NearbyTriangles(m1,pw,tol,tris,maxcontacts-k);
    for(size_t j=0;j<tris.size();j++) {   
      m1.GetTriangle(tris[j],tri);
      triw.a = m1.currentTransform*tri.a;
      triw.b = m1.currentTransform*tri.b;
      triw.c = m1.currentTransform*tri.c;
      Vector3 cp = triw.closestPoint(pw);
      Vector3 n = cp - pw;
      Real d = n.length();
      if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
        Vector3 plocal;
        m1.currentTransform.mulInverse(cp,plocal);
        n = ContactNormal(m1,plocal,tris[j],pw);
      }
      else if(d > tol) {  //some penetration -- we can't trust the result of PQP
        continue;
      }
      else n /= d;
      //migrate the contact point to the center of the overlap region
      CopyVector(contact[k].pos,0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n);
      CopyVector(contact[k].n,n);
      contact[k].depth = tol - d;
      k++;
      if(k == maxcontacts) break;
    }
  }
  */
}

void PointCloudMeshContacts(CollisionPointCloud &pc1, Real outerMargin1, CollisionMesh &m2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  MeshPointCloudContacts(m2, outerMargin2, pc1, outerMargin1, contacts, maxcontacts);
  for (auto &c : contacts)
    ReverseContact(c);
}

void MeshSphereContacts(CollisionMesh &m1, Real outerMargin1, const Sphere3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Triangle3D tri;
  vector<int> tris;
  NearbyTriangles(m1, s.center, s.radius + tol, tris, maxcontacts);
  for (size_t j = 0; j < tris.size(); j++)
  {
    m1.GetTriangle(tris[j], tri);
    tri.a = m1.currentTransform * tri.a;
    tri.b = m1.currentTransform * tri.b;
    tri.c = m1.currentTransform * tri.c;

    Vector3 cp = tri.closestPoint(s.center);
    Vector3 n = s.center - cp;
    Real nlen = n.length();
    Real d = nlen - s.radius;
    Vector3 pw = s.center;
    if (s.radius > 0)
      //adjust pw to the sphere surface
      pw -= n * (s.radius / nlen);
    if (d < gNormalFromGeometryTolerance)
    { //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp, plocal);
      n = ContactNormal(m1, plocal, tris[j], pw);
      n.inplaceNegative();
    }
    else if (d > tol)
    { //some penetration -- we can't trust the result of PQP
      continue;
    }
    else
      n /= nlen;
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = cp + outerMargin1 * n;
    contacts[k].p2 = pw - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = tris[j];
    contacts[k].elem2 = -1;
    contacts[k].unreliable = false;
  }
}

void MeshPrimitiveContacts(CollisionMesh &m1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  GeometricPrimitive3D gworld = g2;
  gworld.Transform(T2);

  if (gworld.type == GeometricPrimitive3D::Point)
  {
    Sphere3D s;
    s.center = *AnyCast<Point3D>(&gworld.data);
    s.radius = 0;
    MeshSphereContacts(m1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else if (gworld.type == GeometricPrimitive3D::Sphere)
  {
    const Sphere3D &s = *AnyCast<Sphere3D>(&gworld.data);
    MeshSphereContacts(m1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Distance computations between Triangles and " << gworld.TypeName() << " not supported");
    return;
  }
}

void PointCloudPrimitiveContacts(CollisionPointCloud &pc1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  if (g2.type == GeometricPrimitive3D::Empty)
    return;
  if (!g2.SupportsDistance(GeometricPrimitive3D::Point))
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Cannot do contact checking on point cloud vs primitive " << g2.TypeName() << " yet");
    return;
  }

  GeometricPrimitive3D gworld = g2;
  gworld.Transform(T2);

  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  NearbyPoints(pc1, gworld, tol, points, maxcontacts);
  contacts.reserve(points.size());
  for (size_t j = 0; j < points.size(); j++)
  {
    Vector3 pw = pc1.currentTransform * pc1.points[points[j]];
    Real dg = gworld.Distance(pw);
    if (dg > tol)
      continue;
    vector<double> u = gworld.ClosestPointParameters(pw);
    Vector3 cp = gworld.ParametersToPoint(u);
    Vector3 n = cp - pw;
    Real d = n.length();
    if (!FuzzyEquals(d, dg))
      LOG4CXX_WARN(GET_LOGGER(Geometry), "Hmm... point distance incorrect? " << dg << " vs " << d);
    if (d < gNormalFromGeometryTolerance)
    { //too close?
      continue;
    }
    else if (d > tol)
    { //why did this point get farther away?
      continue;
    }
    else
      n /= d;
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = pw + outerMargin1 * n;
    contacts[k].p2 = cp - outerMargin2 * n;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points[j];
    contacts[k].elem2 = -1;
    contacts[k].unreliable = false;
  }
}

void PointCloudPointCloudContacts(CollisionPointCloud &pc1, Real outerMargin1, CollisionPointCloud &pc2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points1, points2;
  Collides(pc1, pc2, tol, points1, points2, maxcontacts);

  //this is already done in Collides
  /*  
  map<int,int> closest1,closest2;
  map<int,Real> dclosest1,dclosest2;
  //filter by closest point 
  for(size_t i=0;i<points1.size();i++) {
    Vector3 p1w = pc1.currentTransform*pc1.points[points1[i]];
    Vector3 p2w = pc2.currentTransform*pc2.points[points2[i]];
    Real d=p1w.distance(p2w);
    if(closest1.count(points1[i])==0 || d < dclosest1[points1[i]]) {
      closest1[points1[i]] = points2[i];
      dclosest1[points1[i]] = d;
    }
    if(closest2.count(points2[i])==0 || d < dclosest2[points2[i]]) {
      closest2[points2[i]] = points1[i];
      dclosest2[points2[i]] = d;
    }
  }
  points1.resize(0);
  points2.resize(0);
  for(const auto& c1: closest1) {
    points1.push_back(c1.first);
    points2.push_back(c1.second);
  }
  for(const auto& c2: closest2) {
    if(closest1[c2.second] != c2.first) {
      points1.push_back(c2.second);
      points2.push_back(c2.first);
    }
  }
  */
  contacts.reserve(points1.size());
  for (size_t j = 0; j < points1.size(); j++)
  {
    Vector3 p1w = pc1.currentTransform * pc1.points[points1[j]];
    Vector3 p2w = pc2.currentTransform * pc2.points[points2[j]];
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].n = p2w - p1w;
    Real d = contacts[k].n.norm();
    if (!FuzzyZero(d))
    {
      contacts[k].n /= d;
      contacts[k].unreliable = false;
    }
    else
    {
      contacts[k].n.setZero();
      contacts[k].unreliable = true;
    }
    contacts[k].p1 = p1w + contacts[k].n * outerMargin1;
    contacts[k].p2 = p2w - contacts[k].n * outerMargin2;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points1[j];
    contacts[k].elem2 = points2[j];
  }
}

void PointCloudImplicitSurfaceContacts(CollisionPointCloud &pc1, Real outerMargin1, CollisionImplicitSurface &s2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  Geometry::Collides(s2, pc1, tol, points, maxcontacts);
  contacts.reserve(points.size());
  for (size_t j = 0; j < points.size(); j++)
  {
    Vector3 p1w = pc1.currentTransform * pc1.points[points[j]];
    Vector3 p2w;
    Vector3 n;
    Real d = Geometry::Distance(s2, p1w, p2w, n);
    size_t k = contacts.size();
    contacts.resize(k + 1);
    contacts[k].p1 = p1w + n * outerMargin1;
    contacts[k].p2 = p2w - n * outerMargin2;
    contacts[k].n = n;
    contacts[k].depth = tol - d;
    contacts[k].elem1 = points[j];
    contacts[k].elem2 = PointIndex(s2, p2w);
    contacts[k].unreliable = false;
  }
}

void ImplicitSurfaceSphereContacts(CollisionImplicitSurface &s1, Real outerMargin1, const Sphere3D &s, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  //NOTE: this does not allow for multiple points of contact, e.g., for non-convex implicit surfaces
  contacts.resize(0);
  Real tol = outerMargin1 + outerMargin2;
  Vector3 cp, dir;
  Real d = Distance(s1, s.center, cp, dir) - s.radius;
  if (d > tol)
    return;
  Vector3 pw = s.center + dir * s.radius;
  contacts.resize(1);
  Vector3 n;
  n.setNegative(dir);
  contacts[0].p1 = cp + outerMargin1 * n;
  contacts[0].p2 = pw - outerMargin2 * n;
  contacts[0].n = n;
  contacts[0].depth = tol - d;
  contacts[0].elem1 = PointIndex(s1, cp);
  contacts[0].elem2 = -1;
  contacts[0].unreliable = false;
}

void ImplicitSurfacePrimitiveContacts(CollisionImplicitSurface &s1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  GeometricPrimitive3D gworld = g2;
  gworld.Transform(T2);

  if (gworld.type == GeometricPrimitive3D::Point)
  {
    Sphere3D s;
    s.center = *AnyCast<Point3D>(&gworld.data);
    s.radius = 0;
    ImplicitSurfaceSphereContacts(s1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else if (gworld.type == GeometricPrimitive3D::Sphere)
  {
    const Sphere3D &s = *AnyCast<Sphere3D>(&gworld.data);
    ImplicitSurfaceSphereContacts(s1, outerMargin1, s, outerMargin2, contacts, maxcontacts);
  }
  else
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Distance computations between ImplicitSurface and " << gworld.TypeName() << " not supported");
    return;
  }
}

void PrimitivePrimitiveContacts(GeometricPrimitive3D &g1, const RigidTransform &T1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  if (maxcontacts == 0)
    return;
  if (!g1.SupportsDistance(g2.type))
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: primitive collisions of type " << g1.TypeName() << " to " << g2.TypeName());
    return;
  }
  if ((g1.type != GeometricPrimitive3D::Point && g1.type != GeometricPrimitive3D::Sphere) && (g2.type == GeometricPrimitive3D::Point || g2.type == GeometricPrimitive3D::Sphere))
  {
    //do this the other way around
    PrimitivePrimitiveContacts(g2, T2, outerMargin2, g1, T1, outerMargin1, contacts, maxcontacts);
    for (auto &c : contacts)
      ReverseContact(c);
    return;
  }
  GeometricPrimitive3D tg1 = g1, tg2 = g2;
  tg1.Transform(T1);
  tg2.Transform(T2);
  if (g1.type != GeometricPrimitive3D::Point && g1.type != GeometricPrimitive3D::Sphere)
  {
    //TODO: try copying into ODE data structures?
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Contact computations between primitives " << g1.TypeName() << " and " << g2.TypeName() << " not yet supported");
    return;
  }
  else
  {
    Sphere3D s;
    if (g1.type == GeometricPrimitive3D::Point)
    {
      s.center = *AnyCast<Point3D>(&tg1.data);
      s.radius = 0;
    }
    else
    {
      s = *AnyCast<Sphere3D>(&tg1.data);
    }
    if (tg2.Distance(s.center) > s.radius + outerMargin1 + outerMargin2)
      return;
    vector<double> params = tg2.ClosestPointParameters(s.center);
    Vector3 p2 = tg2.ParametersToPoint(params);
    //normal out from sphere to g2
    Vector3 n = p2 - s.center;
    Real d = n.norm();
    if (FuzzyZero(d))
    {
      //penetrating all the way to center?
      n = tg2.ParametersToNormal(params);
    }
    else
      n /= d;
    Vector3 p1 = s.center + n * s.radius;
    p2 -= outerMargin2 * n;
    p1 += outerMargin1 * n;
    contacts.resize(1);
    contacts[0].depth = p1.distance(p2);
    contacts[0].p1 = p1;
    contacts[0].p2 = p2;
    contacts[0].n = n;
    contacts[0].elem1 = contacts[0].elem2 = 0;
    contacts[0].unreliable = false;
  }
}

void PrimitiveMeshContacts(GeometricPrimitive3D &g1, const RigidTransform &T1, Real outerMargin1, CollisionMesh &m2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  MeshPrimitiveContacts(m2, outerMargin2, g1, T1, outerMargin1, contacts, maxcontacts);
  for (auto &c : contacts)
    ReverseContact(c);
}

void PrimitivePointCloudContacts(GeometricPrimitive3D &g1, const RigidTransform &T1, Real outerMargin1, CollisionPointCloud &pc2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  PointCloudPrimitiveContacts(pc2, outerMargin2, g1, T1, outerMargin1, contacts, maxcontacts);
  for (auto &c : contacts)
    ReverseContact(c);
}

void PrimitiveGeometryContacts(GeometricPrimitive3D &g1, const RigidTransform &T, Real outerMargin1, Geometry::AnyCollisionGeometry3D &g2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  switch (g2.type)
  {
  case AnyGeometry3D::Primitive:
    PrimitivePrimitiveContacts(g1, T, outerMargin1,
                               g2.AsPrimitive(), g2.PrimitiveCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
    return;
  case AnyGeometry3D::TriangleMesh:
    PrimitiveMeshContacts(g1, T, outerMargin1,
                          g2.TriangleMeshCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
    return;
  case AnyGeometry3D::PointCloud:
    PrimitivePointCloudContacts(g1, T, outerMargin1,
                                g2.PointCloudCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
    return;
  case AnyGeometry3D::ImplicitSurface:
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: primitive-implicit surface contacts");
    break;
  case AnyGeometry3D::ConvexHull:
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: primitive-convex hull contacts");
    break;
  case AnyGeometry3D::Group:
  {
    vector<Geometry::AnyCollisionGeometry3D> &items = g2.GroupCollisionData();
    int n = 0;
    for (size_t i = 0; i < items.size(); i++)
    {
      vector<ContactPair> subcontacts;
      PrimitiveGeometryContacts(g1, T, outerMargin1, items[i], g2.margin + outerMargin2, subcontacts, maxcontacts - n);
      for (auto &c : subcontacts)
        c.elem2 = (int)i;
      contacts.insert(contacts.end(), subcontacts.begin(), subcontacts.end());
      n += subcontacts.size();
      if (n >= (int)maxcontacts)
        return;
    }
  }
  break;
  }
}

void MeshGeometryContacts(CollisionMesh &m1, Real outerMargin1, Geometry::AnyCollisionGeometry3D &g2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  switch (g2.type)
  {
  case AnyGeometry3D::Primitive:
    MeshPrimitiveContacts(m1, outerMargin1,
                          g2.AsPrimitive(), g2.PrimitiveCollisionData(), g2.margin + outerMargin2,
                          contacts, maxcontacts);
    return;
  case AnyGeometry3D::TriangleMesh:
    MeshMeshContacts(m1, outerMargin1,
                     g2.TriangleMeshCollisionData(), g2.margin + outerMargin2,
                     contacts, maxcontacts);
    return;
  case AnyGeometry3D::PointCloud:
    MeshPointCloudContacts(m1, outerMargin1,
                           g2.PointCloudCollisionData(), g2.margin + outerMargin2,
                           contacts, maxcontacts);
    return;
  case AnyGeometry3D::ImplicitSurface:
    LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: triangle mesh-implicit surface contacts");
    break;
  case AnyGeometry3D::ConvexHull:
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: triangle mesh-convex hull contacts");
    break;
  case AnyGeometry3D::Group:
  {
    vector<Geometry::AnyCollisionGeometry3D> &items = g2.GroupCollisionData();
    int n = 0;
    for (size_t i = 0; i < items.size(); i++)
    {
      vector<ContactPair> subcontacts;
      MeshGeometryContacts(m1, outerMargin1, items[i], g2.margin + outerMargin2, subcontacts, maxcontacts - n);
      for (auto &c : subcontacts)
        c.elem2 = (int)i;
      contacts.insert(contacts.end(), subcontacts.begin(), subcontacts.end());
      n += subcontacts.size();
      if (n >= (int)maxcontacts)
        return;
    }
  }
  break;
  }
}

//m is max number of contacts
void GeometryGeometryContacts(Geometry::AnyCollisionGeometry3D &g1, Real outerMargin1,
                              Geometry::AnyCollisionGeometry3D &g2, Real outerMargin2,
                              vector<ContactPair> &contacts, size_t maxcontacts)
{
  g1.InitCollisionData();
  g2.InitCollisionData();
  switch (g1.type)
  {
  case AnyGeometry3D::Primitive:
    PrimitiveGeometryContacts(g1.AsPrimitive(), g1.PrimitiveCollisionData(), g1.margin + outerMargin1, g2, outerMargin2, contacts, maxcontacts);
    return;
  case AnyGeometry3D::TriangleMesh:
    MeshGeometryContacts(g1.TriangleMeshCollisionData(), g1.margin + outerMargin1, g2, outerMargin2, contacts, maxcontacts);
    return;
  case AnyGeometry3D::PointCloud:
    switch (g2.type)
    {
    case AnyGeometry3D::Primitive:
      PointCloudPrimitiveContacts(g1.PointCloudCollisionData(), g1.margin + outerMargin1, g2.AsPrimitive(), g2.PrimitiveCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
      return;
    case AnyGeometry3D::TriangleMesh:
      PointCloudMeshContacts(g1.PointCloudCollisionData(), g1.margin + outerMargin1, g2.TriangleMeshCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
      return;
    case AnyGeometry3D::PointCloud:
      PointCloudPointCloudContacts(g1.PointCloudCollisionData(), g1.margin + outerMargin1, g2.PointCloudCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
      break;
    case AnyGeometry3D::ImplicitSurface:
      PointCloudImplicitSurfaceContacts(g1.PointCloudCollisionData(), g1.margin + outerMargin1, g2.ImplicitSurfaceCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
      break;
    case AnyGeometry3D::Group:
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: point cloud-group contacts");
      break;
    case AnyGeometry3D::ConvexHull:
      LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: point cloud-convex hull contacts");
      break;
    }
    break;
  case AnyGeometry3D::ImplicitSurface:
    switch (g2.type)
    {
    case AnyGeometry3D::Primitive:
      ImplicitSurfacePrimitiveContacts(g1.ImplicitSurfaceCollisionData(), g1.margin + outerMargin1, g2.AsPrimitive(), g2.PrimitiveCollisionData(), g2.margin + outerMargin2, contacts, maxcontacts);
      break;
    case AnyGeometry3D::TriangleMesh:
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: implicit surface-triangle mesh contacts");
      break;
    case AnyGeometry3D::PointCloud:
      {
        PointCloudImplicitSurfaceContacts(g2.PointCloudCollisionData(), g2.margin + outerMargin2, g1.ImplicitSurfaceCollisionData(), g1.margin + outerMargin1, contacts, maxcontacts);
        for (auto &c : contacts)
          ReverseContact(c);
      }
      break;
    case AnyGeometry3D::ImplicitSurface:
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: implicit surface-implicit surface contacts");
      break;
    case AnyGeometry3D::Group:
      LOG4CXX_ERROR(GET_LOGGER(Geometry), "TODO: implicit surface-group contacts");
      break;
    case AnyGeometry3D::ConvexHull:
      LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: implicit surface-convex hull contacts");
      break;
    }
    break;
  case AnyGeometry3D::ConvexHull:
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: convex hull-anything contacts");
    break;
  case AnyGeometry3D::Group:
  {
    vector<Geometry::AnyCollisionGeometry3D> &items = g1.GroupCollisionData();
    int n = 0;
    for (size_t i = 0; i < items.size(); i++)
    {
      vector<ContactPair> subcontacts;
      GeometryGeometryContacts(items[i], g1.margin + outerMargin1, g2, outerMargin2, subcontacts, maxcontacts - n);
      for (auto &c : subcontacts)
        c.elem2 = (int)i;
      contacts.insert(contacts.end(), subcontacts.begin(), subcontacts.end());
      n += subcontacts.size();
      if (n >= (int)maxcontacts)
        return;
    }
  }
    return;
  }
  return;
}

AnyContactsQueryResult AnyCollisionGeometry3D::Contacts(AnyCollisionGeometry3D &other, const AnyContactsQuerySettings &settings)
{
  AnyContactsQueryResult res;
  if (settings.cluster)
    GeometryGeometryContacts(*this, settings.padding1, other, settings.padding2, res.contacts, std::numeric_limits<size_t>::max());
  else
    GeometryGeometryContacts(*this, settings.padding1, other, settings.padding2, res.contacts, settings.maxcontacts);
  res.clustered = false;
  return res;
}

AnyContactsQuerySettings::AnyContactsQuerySettings()
    : padding1(0), padding2(0), maxcontacts(std::numeric_limits<size_t>::max()), cluster(false)
{
}

AnyContactsQueryResult::AnyContactsQueryResult()
    : clustered(false)
{
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
  if (q->a->type == AnyGeometry3D::TriangleMesh && q->b->type == AnyGeometry3D::TriangleMesh)
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
