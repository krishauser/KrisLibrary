#include <KrisLibrary/Logger.h>
#include "GeometryAppearance.h"
#include "GLTexture1D.h"
#include "GLTexture2D.h"
#include <geometry/AnyGeometry.h>
#include "drawMesh.h"
#include "drawgeometry.h"
#include "drawextra.h"
#include <meshing/PointCloud.h>
#include <meshing/VolumeGrid.h>
#include <meshing/Expand.h>
#include <meshing/TriMeshOperators.h>
#include <geometry/Conversions.h>
#include <graph/UndirectedGraph.h>
#include <graph/ConnectedComponents.h>
#include <KrisLibrary/Timer.h>
#include <memory.h>
#include <unordered_map>

#ifdef GL_CLAMP_TO_EDGE
#define GL_CLAMP_MODE GL_CLAMP_TO_EDGE
#else
#define GL_CLAMP_MODE GL_CLAMP
#endif //GL_CLAMP_TO_EDGE

using namespace Geometry;
namespace GLDraw {

  void TransferTexture1D(GLTextureObject& obj,const Image& img)
  {
    GLTexture1D tex;
    tex.texObj = obj;
    int n=img.w*img.h;
    switch(img.format) {
    case Image::R8G8B8:
      tex.setRGB(img.data,n);
      break;
    case Image::R8G8B8A8:
      tex.setRGBA(img.data,n);
      break;
    case Image::B8G8R8:
      tex.setBGR(img.data,n);
      break;
    case Image::B8G8R8A8:
      tex.setBGRA(img.data,n);
      break;
    case Image::A8:
      tex.setLuminance(img.data,n);
      break;
    default:
      LOG4CXX_ERROR(KrisLibrary::logger(),"Texture image doesn't match a supported GL format");
      break;
    }
  }


  void TransferTexture2D(GLTextureObject& obj,const Image& img)
  {
    GLTexture2D tex;
    tex.texObj = obj;
    //OpenGL stores images from bottom left, Image has them from top left
    int pixelsize = img.pixelSize();
    int stride = pixelsize*img.w;
    unsigned char* buf = new unsigned char[img.w*img.h*pixelsize];
    for(int i=0;i<img.h;i++) {
      int iflip = img.h-1-i;
      memcpy(&buf[iflip*stride],&img.data[i*stride],stride);
    }
    switch(img.format) {
    case Image::R8G8B8:
      tex.setRGB(buf,img.w,img.h);
      break;
    case Image::R8G8B8A8:
      tex.setRGBA(buf,img.w,img.h);
      break;
    case Image::B8G8R8:
      tex.setBGR(buf,img.w,img.h);
      break;
    case Image::B8G8R8A8:
      tex.setBGRA(buf,img.w,img.h);
      break;
    case Image::A8:
      tex.setLuminance(buf,img.w,img.h);
      break;
    default:
      LOG4CXX_ERROR(KrisLibrary::logger(),"Texture image doesn't match a supported GL format");
      break;
    }
    delete [] buf;
  }

  void draw(const Geometry::AnyGeometry3D& geom)
  {
    if(geom.type == AnyGeometry3D::Type::PointCloud) 
      drawPoints(geom);
    else if(geom.type == AnyGeometry3D::Type::Group) {
      const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom.AsGroup();
      for(size_t i=0;i<subgeoms.size();i++)
        draw(subgeoms[i]);
    }
    else
      drawFaces(geom);
  }

  void drawPoints(const Geometry::AnyGeometry3D& geom)
  {
    const vector<Vector3>* verts = NULL;
    if(geom.type == AnyGeometry3D::Type::TriangleMesh) 
      verts = &geom.AsTriangleMesh().verts;
    else if(geom.type == AnyGeometry3D::Type::PointCloud) 
      verts = &geom.AsPointCloud().points;
    else if(geom.type == AnyGeometry3D::Type::Group) {
      const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom.AsGroup();
      for(size_t i=0;i<subgeoms.size();i++)
        drawPoints(subgeoms[i]);
    }
    if(verts) {
      glBegin(GL_POINTS);
      for(size_t i=0;i<verts->size();i++) {
        const Vector3& v=(*verts)[i];
        //glVertex3v();
        glVertex3f(v.x,v.y,v.z);
      }
      glEnd();
    }
  }

  void drawFaces(const Geometry::AnyGeometry3D& geom)
  {
    const Meshing::TriMesh* trimesh = NULL;
    if(geom.type == AnyGeometry3D::Type::TriangleMesh) 
      trimesh = &geom.AsTriangleMesh();
    else if(geom.type == AnyGeometry3D::Type::Group) {
      const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom.AsGroup();
      for(size_t i=0;i<subgeoms.size();i++)
        drawFaces(subgeoms[i]);
    }
    else if(geom.type == AnyGeometry3D::Type::Primitive) 
      draw(geom.AsPrimitive());  

    //draw the mesh
    if(trimesh) {
      DrawGLTris(*trimesh);
    }
  }

  void drawWorld(const Geometry::AnyCollisionGeometry3D& geom)
  {
    glPushMatrix();
    glMultMatrix(Matrix4(geom.GetTransform()));
    draw(geom);
    glPopMatrix();
  }

  void drawPointsWorld(const Geometry::AnyCollisionGeometry3D& geom)
  {
    glPushMatrix();
    glMultMatrix(Matrix4(geom.GetTransform()));
    drawPoints(geom);
    glPopMatrix();
  }

  void drawFacesWorld(const Geometry::AnyCollisionGeometry3D& geom)
  {
    glPushMatrix();
    glMultMatrix(Matrix4(geom.GetTransform()));
    drawFaces(geom);
    glPopMatrix();
  }

  void drawExpanded(Geometry::AnyCollisionGeometry3D& geom,Real p)
  {
    if(p < 0) p = geom.margin;
    if(geom.type == Geometry::AnyGeometry3D::Type::TriangleMesh) {
      Meshing::TriMesh m;
      geom.TriangleMeshCollisionData().CalcTriNeighbors();
      geom.TriangleMeshCollisionData().CalcIncidentTris();
      Meshing::Expand2Sided(geom.TriangleMeshCollisionData(),p,3,m);
      DrawGLTris(m);
    }
    else if(geom.type == Geometry::AnyGeometry3D::Type::PointCloud) {
      const Meshing::PointCloud3D& pc = geom.AsPointCloud();
      for(size_t i=0;i<pc.points.size();i++) {
        glPushMatrix();
        glTranslate(pc.points[i]);
        drawSphere(p,8,4);
        glPopMatrix();
      }
    }
    else if(geom.type == Geometry::AnyGeometry3D::Type::ImplicitSurface) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw implicit surface");
    }
    else if(geom.type == Geometry::AnyGeometry3D::Type::OccupancyGrid) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw occupancy grid");
    }
    else if(geom.type == Geometry::AnyGeometry3D::Type::Primitive) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw expanded primitive");
      draw(geom.AsPrimitive());
    }
    else if(geom.type == Geometry::AnyGeometry3D::Type::Group) {
      std::vector<Geometry::AnyCollisionGeometry3D>& subgeoms = geom.GroupCollisionData();
      for(size_t i=0;i<subgeoms.size();i++)
        drawExpanded(subgeoms[i],p);
    }
  }


void VertexNormals(const Meshing::TriMesh& mesh,vector<Vector3>& normals)
{
  normals.resize(mesh.verts.size());
  fill(normals.begin(),normals.end(),Vector3(0.0));
  for(size_t i=0;i<mesh.tris.size();i++) {
    const IntTriple&t=mesh.tris[i];
    Vector3 n = mesh.TriangleNormal(i);
    normals[t.a] += n;
    normals[t.b] += n;
    normals[t.c] += n;
  }
  for(size_t i=0;i<mesh.verts.size();i++) {
    Real l = normals[i].norm();
    if(!FuzzyZero(l))
      normals[i] /= l;
  }
}

void CreaseMesh(Meshing::TriMeshWithTopology& in,Meshing::TriMesh& out,Real creaseRads,bool dropDegenerate=true)
{
  //double graphTime = 0, ccTime = 0, overheadTime = 0;
  Timer timer;
  if(in.incidentTris.empty())
    in.CalcIncidentTris();
  if(in.triNeighbors.empty())
    in.CalcTriNeighbors();
  Real cosCreaseRads = Cos(creaseRads);

  vector<Vector3> triNormals(in.tris.size());
  for(size_t i=0;i<in.tris.size();i++)
    triNormals[i] = in.TriangleNormal(i);
  out.verts.resize(in.verts.size());
  out.tris.resize(in.tris.size());
  fill(out.tris.begin(),out.tris.end(),IntTriple(-1,-1,-1));
  vector<int> triGraphIndex(in.tris.size());  //temporary, used only for each vertex's incidient triangles
  for(size_t i=0;i<in.verts.size();i++) {
    //theres a potential graph of creases here
    //timer.Reset();
    const auto& tris = in.incidentTris[i];
    Graph::UndirectedGraph<int,int> tgraph;
    tgraph.Resize(tris.size());
    for(size_t s=0;s<tris.size();s++) {
      tgraph.nodes[s] = tris[s];
      triGraphIndex[tris[s]] = (int)s;
    }
    for(size_t s=0;s<tris.size();s++) {
      int n=tris[s];
      int iprev,inext;
      if(in.tris[n].a == int(i)) {
        iprev = in.triNeighbors[n].b;
        inext = in.triNeighbors[n].c;
      }
      else if(in.tris[n].b == int(i)) {
        iprev = in.triNeighbors[n].c;
        inext = in.triNeighbors[n].a;
      }
      else {
        iprev = in.triNeighbors[n].a;
        inext = in.triNeighbors[n].b;
      }
      if (iprev >= 0 && triNormals[n].dot(triNormals[iprev]) >= cosCreaseRads) {
        int previndex = triGraphIndex[iprev];
        if(!tgraph.HasEdge(s,previndex)) 
          tgraph.AddEdge(s,previndex);
      }
      if (inext >= 0 && triNormals[n].dot(triNormals[inext]) >= cosCreaseRads) {
        int nextindex = triGraphIndex[inext];
        if(!tgraph.HasEdge(s,nextindex))
          tgraph.AddEdge(s,nextindex);
      }
    }
    //graphTime += timer.ElapsedTime();
    //timer.Reset();
    Graph::ConnectedComponents ccs;
    ccs.Compute(tgraph);
    vector<int> reps;
    ccs.GetRepresentatives(reps);
    //ccTime += timer.ElapsedTime();
    //timer.Reset();
    //add a new vertex for each cc
    vector<int> cc;
    for(auto r:reps) {
      ccs.EnumerateComponent(r,cc);
      int nvout = (int)out.verts.size();
      for(auto n:cc) {
        int t=tgraph.nodes[n];
        Assert(in.tris[t].getIndex(i) >= 0);
        out.tris[t][in.tris[t].getIndex(i)] = nvout;
      }
      out.verts.push_back(in.verts[i]);
    }
    //overheadTime += timer.ElapsedTime();
  }
  timer.Reset();
  if(dropDegenerate) {
    size_t validTriangles = 0;
    for(size_t i=0;i<out.tris.size();i++) {
      const IntTriple& t = out.tris[i];
      if(t.a < 0 || t.b < 0 || t.c < 0) {
        const IntTriple& tin = in.tris[i];
        printf("CreaseMesh: Invalid triangle %d %d %d, input triangle %d %d %d\n",t.a,t.b,t.c,tin.a,tin.b,tin.c);
        continue;
      }
      //Assert(t.a >= 0 && t.b >= 0 && t.c >= 0);
      out.tris[validTriangles] = out.tris[i];
      validTriangles++;
    }
    //printf("CreaseMesh: dropping from %d to %d triangles\n",out.tris.size(),validTriangles);
    out.tris.resize(validTriangles);
  }
  else {
    for(size_t i=0;i<out.tris.size();i++) {
      IntTriple& t = out.tris[i];
      if(t.a < 0 || t.b < 0 || t.c < 0)
        t.a = t.b = t.c = 0;
    }
  }
  //overheadTime += timer.ElapsedTime();
  //printf("Time for graph %g, cc %g, overhead %g\n",graphTime,ccTime,overheadTime);
}


GeometryAppearance::GeometryAppearance()
  :geom(NULL),drawVertices(false),drawEdges(false),drawFaces(false),vertexSize(1.0),edgeSize(1.0),
   lightFaces(true),
   vertexColor(1,1,1),edgeColor(1,1,1),faceColor(0.5,0.5,0.5),emissiveColor(0,0,0),shininess(0),specularColor(0.1,0.1,0.1),
   texWrap(true),texFilterNearest(false),
   creaseAngle(0),silhouetteRadius(0),silhouetteColor(0,0,0),tintColor(0,0,0),tintStrength(0)
{}

void GeometryAppearance::CopyMaterial(const GeometryAppearance& rhs)
{
  if(this == &rhs) return;
  if(subAppearances.size() == rhs.subAppearances.size()) {
    for(size_t i=0;i<subAppearances.size();i++)
      subAppearances[i].CopyMaterial(rhs.subAppearances[i]);
  }
  else if(rhs.subAppearances.empty()) {
    for(size_t i=0;i<subAppearances.size();i++)
      subAppearances[i].CopyMaterial(rhs);
  }

  drawVertices=rhs.drawVertices;
  drawEdges=rhs.drawEdges;
  drawFaces=rhs.drawFaces;
  vertexSize=rhs.vertexSize;
  edgeSize=rhs.edgeSize;
  lightFaces=rhs.lightFaces;
  vertexColor=rhs.vertexColor;
  edgeColor=rhs.edgeColor;
  faceColor=rhs.faceColor;
  if(!rhs.vertexColors.empty() && !vertexColors.empty()) {
    if(rhs.vertexColors.size() != vertexColors.size())
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of per-vertex colors?"); 
    Refresh();
  }
  if(!rhs.faceColors.empty() && !faceColors.empty()) {
    if(rhs.faceColors.size() != faceColors.size())
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of per-face colors?"); 
    Refresh();
  }
  vertexColors=rhs.vertexColors;
  faceColors=rhs.faceColors;
  tex1D=rhs.tex1D;
  tex2D=rhs.tex2D;
  if(!rhs.texcoords.empty() && !texcoords.empty()) {
    if(rhs.texcoords.size() != texcoords.size())
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of texture coordinates?"); 
    Refresh();
  }
  texcoords=rhs.texcoords;
  texWrap=rhs.texWrap;
  texFilterNearest=rhs.texFilterNearest;
  texgen=rhs.texgen;
  texgenEyeTransform=rhs.texgenEyeTransform;
  emissiveColor=rhs.emissiveColor;
  shininess=rhs.shininess;
  specularColor=rhs.specularColor;
  if(silhouetteRadius != rhs.silhouetteRadius)
    silhouetteDisplayList.erase();
  silhouetteRadius=rhs.silhouetteRadius;
  silhouetteColor=rhs.silhouetteColor;
  if(creaseAngle != rhs.creaseAngle)
    faceDisplayList.erase();
  creaseAngle=rhs.creaseAngle;
  tintColor=rhs.tintColor;
  tintStrength=rhs.tintStrength;
  densityGradientKeypoints=rhs.densityGradientKeypoints;
  densityGradientColors=rhs.densityGradientColors;
}

void GeometryAppearance::CopyMaterialFlat(const GeometryAppearance& rhs)
{
  if(this == &rhs) return;
  if(subAppearances.size() == rhs.subAppearances.size()) {
    for(size_t i=0;i<subAppearances.size();i++) 
      subAppearances[i].CopyMaterialFlat(rhs.subAppearances[i]);
  }
  else if(rhs.subAppearances.empty()) {
    for(size_t i=0;i<subAppearances.size();i++) 
      subAppearances[i].CopyMaterialFlat(rhs);
  }

  drawVertices=rhs.drawVertices;
  drawEdges=rhs.drawEdges;
  drawFaces=rhs.drawFaces;
  vertexSize=rhs.vertexSize;
  edgeSize=rhs.edgeSize;
  lightFaces=rhs.lightFaces;
  vertexColor=rhs.vertexColor;
  edgeColor=rhs.edgeColor;
  faceColor=rhs.faceColor;
  tex1D=rhs.tex1D;
  tex2D=rhs.tex2D;
  texWrap=rhs.texWrap;
  texFilterNearest=rhs.texFilterNearest;
  texgen=rhs.texgen;
  texgenEyeTransform=rhs.texgenEyeTransform;
  emissiveColor=rhs.emissiveColor;
  shininess=rhs.shininess;
  specularColor=rhs.specularColor;
  if(silhouetteRadius != rhs.silhouetteRadius)
    silhouetteDisplayList.erase();
  silhouetteRadius=rhs.silhouetteRadius;
  silhouetteColor=rhs.silhouetteColor;
  if(creaseAngle != rhs.creaseAngle)
    faceDisplayList.erase();
  creaseAngle=rhs.creaseAngle;
  tintColor=rhs.tintColor;
  tintStrength=rhs.tintStrength;
  densityGradientKeypoints=rhs.densityGradientKeypoints;
  densityGradientColors=rhs.densityGradientColors;
}


void GeometryAppearance::CopyCache(const GeometryAppearance& rhs,bool if_cache_exists)
{
  if(this == &rhs) return;
  if((if_cache_exists || rhs.faceDisplayList) && rhs.faceDisplayList)
    faceDisplayList = rhs.faceDisplayList;
  if((if_cache_exists || rhs.edgeDisplayList) && rhs.edgeDisplayList)
    edgeDisplayList = rhs.edgeDisplayList;
  if((if_cache_exists || rhs.vertexDisplayList) && rhs.vertexDisplayList)
    vertexDisplayList = rhs.vertexDisplayList;
  if((if_cache_exists || rhs.silhouetteDisplayList) && rhs.silhouetteDisplayList)
    silhouetteDisplayList = rhs.silhouetteDisplayList;
  if((if_cache_exists || rhs.tempMesh) && rhs.tempMesh)
    tempMesh = rhs.tempMesh;
  if((if_cache_exists || rhs.tempMesh2) && rhs.tempMesh2)
    tempMesh2 = rhs.tempMesh2;
}

void GeometryAppearance::Refresh()
{
  vertexDisplayList.erase();
  edgeDisplayList.erase();
  faceDisplayList.erase();
  silhouetteDisplayList.erase();
  textureObject.cleanup();

  tempMesh.reset();
  tempMesh2.reset();
  if(geom->type == AnyGeometry3D::Type::ImplicitSurface) {
    const Meshing::VolumeGrid* g = &geom->AsImplicitSurface();
    tempMesh.reset(new Meshing::TriMesh);
    ImplicitSurfaceToMesh(*g,*tempMesh);
  }
  else if(geom->type == AnyGeometry3D::Type::OccupancyGrid) {
    //TODO: draw as blocks with density
    AnyGeometry3D gmesh;
    geom->Convert(AnyGeometry3D::Type::TriangleMesh,gmesh);
    tempMesh.reset(new Meshing::TriMesh);
    *tempMesh = gmesh.AsTriangleMesh();
  }
  else if(geom->type == AnyGeometry3D::Type::PointCloud) {
    const Meshing::PointCloud3D& pc = geom->AsPointCloud();
    if(pc.IsStructured()) {
      //draw mesh rather than points
      drawFaces = true;
      drawVertices = false;
      tempMesh.reset(new Meshing::TriMesh);
      //PointCloudToMesh(pc,*tempMesh,*this,0.02);
      PointCloudToMesh(pc,*tempMesh,0.02);
    }
  }
  else if(geom->type == AnyGeometry3D::Type::ConvexHull) {
    const Geometry::ConvexHull3D* g = &geom->AsConvexHull();
    tempMesh.reset(new Meshing::TriMesh);
    ConvexHullToMesh(*g,*tempMesh);
  }
  else if(geom->type == AnyGeometry3D::Type::Heightmap) {
    const Meshing::Heightmap* g = &geom->AsHeightmap();
    tempMesh.reset(new Meshing::TriMesh);
    HeightmapToMesh(*g,*tempMesh,*this);
  }
}

void GeometryAppearance::Set(const Geometry::AnyCollisionGeometry3D& _geom)
{
  geom = &_geom;
  if(geom->type == AnyGeometry3D::Type::Group) {
    drawFaces = true;
    drawEdges = true;
    drawVertices = true;
    if(!_geom.CollisionDataInitialized()) {
      const std::vector<Geometry::AnyGeometry3D>& subgeoms = _geom.AsGroup();
      subAppearances.resize(subgeoms.size());
      for(size_t i=0;i<subAppearances.size();i++) {
        subAppearances[i].CopyMaterialFlat(*this);
        subAppearances[i].Set(subgeoms[i]);
      }
    }
    else {
      const std::vector<Geometry::AnyCollisionGeometry3D>& subgeoms = _geom.GroupCollisionData();
      subAppearances.resize(subgeoms.size());
      for(size_t i=0;i<subAppearances.size();i++) {
        subAppearances[i].CopyMaterialFlat(*this);
        subAppearances[i].Set(subgeoms[i]);
      }
    }
    Refresh();
  }
  else {
    Set(*geom);
  }
  collisionGeom = &_geom;
}

void GeometryAppearance::Set(const AnyGeometry3D& _geom)
{
  geom = &_geom;
  collisionGeom = NULL;
  if(geom->type == AnyGeometry3D::Type::TriangleMesh) {
    auto& app = dynamic_cast<Geometry3DTriangleMesh*>(geom->data.get())->appearance;
    if(app) 
      CopyMaterial(*app);
  }
  if(geom->type == AnyGeometry3D::Type::Primitive) {
    const Math3D::GeometricPrimitive3D* g = &geom->AsPrimitive();
    drawFaces = true;
    drawEdges = false;
    drawVertices = false;
    if(g->type == Math3D::GeometricPrimitive3D::Empty) 
      drawFaces = false;
    else if(g->type == Math3D::GeometricPrimitive3D::Point) {
      drawFaces = false;
      drawVertices = true;
    }
    else if(g->type == Math3D::GeometricPrimitive3D::Segment) {
      drawFaces = false;
      drawEdges = true;
    }
    else if(g->type == Math3D::GeometricPrimitive3D::Triangle) {
      drawEdges = true;
    }
    else if(g->type == Math3D::GeometricPrimitive3D::Polygon) {
      drawEdges = true;
    }
  }
  else if(geom->type == AnyGeometry3D::Type::ImplicitSurface) {
    drawFaces = true;
    drawEdges = false;
    drawVertices = false;
  }
  else if(geom->type == AnyGeometry3D::Type::OccupancyGrid) {
    //TODO: draw as blocks with density
    drawFaces = true;
    drawEdges = false;
    drawVertices = false;
  }
  else if(geom->type == AnyGeometry3D::Type::PointCloud) {
    drawVertices = true;
    drawEdges = false;
    drawFaces = false;
    vector<Real> rgb;
    const Meshing::PointCloud3D& pc = geom->AsPointCloud();
    const static Real scale = 1.0/255.0;
    if(vertexColors.size() == pc.points.size()) {
      //already assigned color
    }
    else {
      if(pc.GetProperty("rgb",rgb)) {
        //convert real to hex to GLcolor
        vertexColors.resize(rgb.size());
        for(size_t i=0;i<rgb.size();i++) {
          unsigned int col = (unsigned int)rgb[i];
          vertexColors[i].set(((col&0xff0000)>>16) * scale,
          ((col&0xff00)>>8) * scale,
          (col&0xff) * scale);
        }
      }
      if(pc.GetProperty("rgba",rgb)) {
        //convert real to hex to GLcolor
        //following PCD, this is actuall A-RGB
        vertexColors.resize(rgb.size());
        for(size_t i=0;i<rgb.size();i++) {
          unsigned int col = (unsigned int)rgb[i];
          vertexColors[i].set(((col&0xff0000)>>16) * scale,
          ((col&0xff00)>>8) * scale,
          (col&0xff) * scale,
          ((col&0xff000000)>>24) * scale);
        }
      }
      if(pc.GetProperty("r",rgb)) {
        if(!vertexColors.empty()) 
          //already assigned color?
          LOG4CXX_WARN(KrisLibrary::logger(),"Point cloud has both r channel and either rgb or rgba channel");
        vertexColors.resize(rgb.size(),vertexColor.rgba);
        for(size_t i=0;i<rgb.size();i++) 
          vertexColors[i].rgba[0] = rgb[i];
      }
      if(pc.GetProperty("g",rgb)) {
        if(vertexColors.empty()) {
          //already assigned color?
          LOG4CXX_WARN(KrisLibrary::logger(),"Point cloud has g channel but not r channel?");
        }
        else {
          for(size_t i=0;i<rgb.size();i++) 
            vertexColors[i].rgba[1] = rgb[i];
        }
      }
      if(pc.GetProperty("b",rgb)) {
        if(vertexColors.empty()) {
          //already assigned color?
          LOG4CXX_WARN(KrisLibrary::logger(),"Point cloud has a channel but not r channel?");
        }
        else {
          for(size_t i=0;i<rgb.size();i++) 
            vertexColors[i].rgba[2] = rgb[i];
        }
      }
      if(pc.GetProperty("opacity",rgb) || pc.GetProperty("a",rgb)) {
        if(!vertexColors.empty()) {
          //already assigned color, just get opacity
          for(size_t i=0;i<rgb.size();i++) {
            vertexColors[i].rgba[3] = rgb[i];
          }
        }
        else {
          vertexColors.resize(rgb.size());
          for(size_t i=0;i<rgb.size();i++) {
            vertexColors[i] = vertexColor.rgba;
            vertexColors[i].rgba[3] = rgb[i];
          }
        }
      }
      if(pc.GetProperty("c",rgb)) {
        //this is a weird opacity in UINT byte format
        if(!vertexColors.empty()) {
          //already assigned color, just get opacity
          for(size_t i=0;i<rgb.size();i++) {
            vertexColors[i].rgba[3] = rgb[i] * scale;
          }
        }
        else {
          vertexColors.resize(rgb.size());
          for(size_t i=0;i<rgb.size();i++) {
            vertexColors[i] = vertexColor.rgba;
            vertexColors[i].rgba[3] = rgb[i] * scale;
          }
        }
      }
    }
    drawFaces = false;
    drawVertices = true;
    if(pc.IsStructured()) {
      //draw mesh rather than points
      drawFaces = true;
      drawVertices = false;
    }
  }
  else if(geom->type == AnyGeometry3D::Type::ConvexHull) {
    drawFaces = true;
    drawEdges = false;
    drawVertices = false;
  }
  else if(geom->type == AnyGeometry3D::Type::Heightmap) {
    const Meshing::Heightmap* g = &geom->AsHeightmap();
    if(g->HasColors() && !tex2D) {
      tex2D = make_shared<Image>(g->colors);
      Vector3 xb(g->viewport.pose.R.col1());
      Vector3 yb(g->viewport.pose.R.col2());
      Vector3 zb(g->viewport.pose.R.col3());
      if(g->viewport.perspective) {
        //x = e1^T(R^-1*(p - t)) = xb^T p - xb^T t
        //y = e2^T(R^-1*(p - t)) = yb^T p - yb^T t
        //z = e3^T(R^-1*(p - t)) = zb^T p - zb^T t
        //u = (x/z*fx + cx)/w = (x*fx/w + cx/w*z) / z 
        //v = (y/z*fy + cy)/h = (y*fy/h + cy/h*z) / z
        Vector3 ub = xb*g->viewport.fx/g->viewport.w + zb*g->viewport.cx/g->viewport.w;
        Vector3 vb = yb*g->viewport.fy/g->viewport.h + zb*g->viewport.cy/g->viewport.h;
        texgen.resize(4);
        texgen[0].set(ub.x,ub.y,ub.z -xb.dot(g->viewport.pose.t)*g->viewport.fx/g->viewport.w);
        texgen[1].set(vb.x,vb.y,vb.z -yb.dot(g->viewport.pose.t)*g->viewport.fy/g->viewport.h);
        texgen[2].set(zb.x,zb.y,zb.z,0);
        texgen[3].set(zb.x,zb.y,zb.z,0);
      }
      else {
        //u = (x*fx + cx)/w = (x*fx/w + cx/w) 
        //v = (y*fy + cy)/h = (y*fy/h + cy/h)
        Vector3 ub = xb*g->viewport.fx/g->viewport.w;
        Vector3 vb = yb*g->viewport.fy/g->viewport.h;
        texgen.resize(2);
        texgen[0].set(ub.x,ub.y,ub.z,g->viewport.cx/g->viewport.w-xb.dot(g->viewport.pose.t)*g->viewport.fx/g->viewport.w);
        texgen[1].set(vb.x,vb.y,vb.z,g->viewport.cx/g->viewport.w-yb.dot(g->viewport.pose.t)*g->viewport.fy/g->viewport.h);
      }
    }
    drawFaces = true;
    drawEdges = false;
    drawVertices = false;
  }
  else if(geom->type == AnyGeometry3D::Type::Group) {
    drawFaces = true;
    drawEdges = true;
    drawVertices = true;
    const std::vector<Geometry::AnyGeometry3D>& subgeoms = _geom.AsGroup();
    subAppearances.resize(subgeoms.size());
    for(size_t i=0;i<subAppearances.size();i++) {
      subAppearances[i].CopyMaterialFlat(*this);
      subAppearances[i].Set(subgeoms[i]);
    }
  }
  else 
    drawFaces = true;
  Refresh();
}

inline void SetTintedColor(const GLColor& col,const GLColor& tintColor,float tintStrength)
{
  if(tintStrength != 0) {
    GLColor temp;
    temp.blend(col,tintColor,tintStrength);
    temp.setCurrentGL();
  }
  else col.setCurrentGL();
}

inline void SetTintedMaterial(GLenum face,GLenum pname,const GLColor& col,const GLColor& tintColor,float tintStrength)
{
  if(tintStrength != 0) {
    GLColor temp;
    temp.blend(col,tintColor,tintStrength);
    glMaterialfv(face,pname,temp.rgba);
  }
  else glMaterialfv(face,pname,col.rgba);
}

void GeometryAppearance::DrawGL(Element e)
{
  bool doDrawVertices = false;
  bool doDrawEdges = false;
  bool doDrawFaces = false;
  bool doDrawSilhouette = false;
  if(e == ALL) {
    doDrawVertices = drawVertices;
    doDrawEdges = drawEdges;
    doDrawFaces = drawFaces;
    doDrawSilhouette = drawFaces && (silhouetteRadius > 0);
  }
  else if(e == FACES)
    doDrawFaces = drawFaces;
  else if(e == EDGES)
    doDrawEdges = drawEdges;
  else if(e == VERTICES)
    doDrawVertices = drawVertices;
  else if(e == ALL_TRANSPARENT) {
    doDrawVertices = drawVertices && (vertexColor.rgba[3] < 1.0);
    doDrawEdges = drawEdges && (edgeColor.rgba[3] < 1.0);
    doDrawFaces = drawFaces && (faceColor.rgba[3] < 1.0);
    doDrawSilhouette = drawFaces && (silhouetteRadius > 0) && (faceColor.rgba[3] < 1.0 || silhouetteColor.rgba[3] < 1.0);
  }
  else if(e == ALL_OPAQUE) {
    doDrawVertices = drawVertices && (vertexColor.rgba[3] >= 1.0);
    doDrawEdges = drawEdges && (edgeColor.rgba[3] >= 1.0);
    doDrawFaces = drawFaces && (faceColor.rgba[3] >= 1.0);
    doDrawSilhouette = drawFaces && (silhouetteRadius > 0) && (faceColor.rgba[3] >= 1.0 && silhouetteColor.rgba[3] >= 1.0);
  }
  else
    FatalError("Invalid Element specified");
  if(doDrawVertices) {   
    const vector<Vector3>* verts = NULL;  
    if(geom->type == AnyGeometry3D::Type::ImplicitSurface ||
      geom->type == AnyGeometry3D::Type::OccupancyGrid ||
      geom->type == AnyGeometry3D::Type::Heightmap ||
      geom->type == AnyGeometry3D::Type::ConvexHull) 
      verts = &tempMesh->verts;
    else if(geom->type == AnyGeometry3D::Type::TriangleMesh) 
      verts = &geom->AsTriangleMesh().verts;
    else if(geom->type == AnyGeometry3D::Type::PointCloud) 
      verts = &geom->AsPointCloud().points;
    if(verts) {
      //do the drawing
      glDisable(GL_LIGHTING);
      if(vertexColor.rgba[3] != 1.0 || !vertexColors.empty()) {
        glEnable(GL_BLEND); 
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      }
      if(vertexColors.size()!=verts->size()) {
        SetTintedColor(vertexColor,tintColor,tintStrength);
      }
      glPointSize(vertexSize);

      //compile the vertex display list
      if(!vertexDisplayList) {
        //LOG4CXX_INFO(KrisLibrary::logger(),"Compiling vertex display list "<<verts->size());
        vertexDisplayList.beginCompileAndExecute();
        if(!vertexColors.empty() && vertexColors.size() != verts->size())
          LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, vertexColors wrong size");
        if(vertexColors.size()==verts->size()) {
          glDisable(GL_LIGHTING);
          glBegin(GL_POINTS);
          for(size_t i=0;i<verts->size();i++) {
            const Vector3& v=(*verts)[i];
            SetTintedColor(vertexColors[i],tintColor,tintStrength);
            glVertex3f(v.x,v.y,v.z);
          }
          glEnd();
          glEnable(GL_LIGHTING);
        }
        else {
          glBegin(GL_POINTS);
          for(size_t i=0;i<verts->size();i++) {
            const Vector3& v=(*verts)[i];
            glVertex3f(v.x,v.y,v.z);
          }
          glEnd();
        }
        //if(silhouetteRadius > 0) glDepthFunc(GL_LESS);
        vertexDisplayList.endCompile();
      }
      else
        vertexDisplayList.call();

      if(vertexColor.rgba[3] != 1.0 || !vertexColors.empty()) {
        glDisable(GL_BLEND); 
      }
    }
  }

  if(doDrawFaces) {
    if(faceColor.rgba[3] != 1.0) {
      glEnable(GL_BLEND); 
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    }

    if(lightFaces) {
      glEnable(GL_LIGHTING);
      SetTintedMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,faceColor,tintColor,tintStrength);
      glMaterialfv(GL_FRONT,GL_EMISSION,emissiveColor.rgba);
      glMaterialf(GL_FRONT,GL_SHININESS,shininess);
      if(shininess != 0)
        glMaterialfv(GL_FRONT,GL_SPECULAR,specularColor.rgba);
      else {
        float zeros[4]={0,0,0,0};
        glMaterialfv(GL_FRONT,GL_SPECULAR,zeros);
      }
    }
    else {
      glDisable(GL_LIGHTING);
      SetTintedColor(faceColor,tintColor,tintStrength);
    }
    //set up the texture coordinates
    if(tex1D || tex2D) {
      if(!textureObject) {
        textureObject.generate();
        if(tex2D) {
          TransferTexture2D(textureObject,*tex2D);
          textureObject.bind(GL_TEXTURE_2D);
        }
        else {
          TransferTexture1D(textureObject,*tex1D);
          textureObject.bind(GL_TEXTURE_1D);
        }
      }
      glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
      if(tex1D) {
        glEnable(GL_TEXTURE_1D);
        textureObject.bind(GL_TEXTURE_1D);
        if(texWrap) {
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
        }
        else {
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_CLAMP_MODE);
        }
        if(texFilterNearest) {
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
        }
        else {
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        }
      }
      else if(tex2D) {
        glEnable(GL_TEXTURE_2D);
        textureObject.bind(GL_TEXTURE_2D);
        if(texWrap) {
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
        }
        else {
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_MODE);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_MODE);
        }
        if(texFilterNearest) {
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
        }
        else {
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        }
      }
    }

    if(!texgen.empty()) {
      GLint mode = GL_OBJECT_LINEAR; 
      GLenum plane = GL_OBJECT_PLANE; 
      if(texgenEyeTransform) {
        mode = GL_EYE_LINEAR;
        plane = GL_EYE_PLANE;
        RigidTransform Tinv;
        if(collisionGeom)
          Tinv.setInverse(collisionGeom->currentTransform);
        else
          Tinv.setIdentity();
        glPushMatrix();
        glMultMatrix(Matrix4(*texgenEyeTransform*Tinv));
      }
      
      glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,mode);
      float params[4];
      for(int i=0;i<4;i++)
        params[i] = texgen[0][i];
      glTexGenfv(GL_S,plane,params);
      glEnable(GL_TEXTURE_GEN_S);
      if(texgen.size() >= 2) {
        glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,mode);
        for(int i=0;i<4;i++)
          params[i] = texgen[1][i];
        glTexGenfv(GL_T,plane,params);
        glEnable(GL_TEXTURE_GEN_T);
      }
      if(texgen.size() >= 3) {
        glTexGeni(GL_R,GL_TEXTURE_GEN_MODE,mode);
        for(int i=0;i<4;i++)
          params[i] = texgen[2][i];
        glTexGenfv(GL_R,plane,params);
        glEnable(GL_TEXTURE_GEN_R);
      }
      if(texgen.size() >= 4) {
        glTexGeni(GL_Q,GL_TEXTURE_GEN_MODE,mode);
        for(int i=0;i<4;i++)
          params[i] = texgen[3][i];
        glTexGenfv(GL_Q,plane,params);
        glEnable(GL_TEXTURE_GEN_Q);
      }

      if(texgenEyeTransform) {
        glPopMatrix();
      }
    }
    
    if(!faceDisplayList) {
      faceDisplayList.beginCompileAndExecute();
      Timer timer;
      const Meshing::TriMesh* trimesh = NULL;
      const Meshing::TriMeshWithTopology* trimesh_topology = NULL;
      if(geom->type == AnyGeometry3D::Type::ImplicitSurface ||
        geom->type == AnyGeometry3D::Type::OccupancyGrid ||
        geom->type == AnyGeometry3D::Type::Heightmap ||
        geom->type == AnyGeometry3D::Type::PointCloud ||
        geom->type == AnyGeometry3D::Type::ConvexHull) {
        trimesh = tempMesh.get();
      }
      else if(geom->type == AnyGeometry3D::Type::TriangleMesh) {
        trimesh = &geom->AsTriangleMesh();
        if(collisionGeom && collisionGeom->CollisionDataInitialized())
          trimesh_topology = &collisionGeom->TriangleMeshCollisionData();
      }
      else if(geom->type == AnyGeometry3D::Type::Primitive) 
        draw(geom->AsPrimitive());

      //LOG4CXX_INFO(KrisLibrary::logger(),"Compiling face display list "<<trimesh->tris.size());
      //draw the mesh
      if(trimesh) {
        Meshing::TriMesh creaseMesh;
        vector<Vector3> vertexNormals;

        if(vertexColors.size() != trimesh->verts.size() && texcoords.size() != trimesh->verts.size() && (creaseAngle > 0 || silhouetteRadius > 0)) {
          shared_ptr<Meshing::TriMeshWithTopology> weldMesh = make_shared<Meshing::TriMeshWithTopology>();
          tempMesh2 = weldMesh;
          weldMesh->verts = trimesh->verts;
          weldMesh->tris = trimesh->tris;
          bool drop = faceColors.empty();
          Meshing::MergeVertices(*weldMesh,1e-5,drop);
          if((weldMesh->verts.size() == trimesh->verts.size()) && trimesh_topology) {
            //copy topology
            weldMesh->vertexNeighbors = trimesh_topology->vertexNeighbors;
            weldMesh->incidentTris = trimesh_topology->incidentTris;
            weldMesh->triNeighbors = trimesh_topology->triNeighbors;
          }
          if(creaseAngle > 0) {
            CreaseMesh(*weldMesh,creaseMesh,creaseAngle,drop);
            VertexNormals(creaseMesh,vertexNormals);
            if(!faceColors.empty())
              assert(trimesh->tris.size() == creaseMesh.tris.size());
            trimesh = &creaseMesh;
          }
          if(timer.ElapsedTime() > 0.5) {
            LOG4CXX_INFO(KrisLibrary::logger(),"GeometryAppearance: preprocessing mesh with "<<trimesh->tris.size()<<" tris took "<<timer.ElapsedTime()<<"s, try disabling creasing and silhouettes");
          }
          else if(timer.ElapsedTime() > 0.1) {
            LOG4CXX_INFO(KrisLibrary::logger(),"GeometryAppearance: preprocessing mesh with "<<trimesh->tris.size()<<" tris took "<<timer.ElapsedTime()<<"s");
          }
        }

        if(!texcoords.empty() && texcoords.size()!=trimesh->verts.size())
          LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, texcoords wrong size: "<<(int)texcoords.size()<<" vs "<<(int)trimesh->verts.size());
        if(!faceColors.empty() && faceColors.size()!=trimesh->tris.size())
          LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, faceColors wrong size: "<<(int)faceColors.size()<<" vs "<<(int)trimesh->tris.size());
        if(texcoords.size()!=trimesh->verts.size() && faceColors.size()!=trimesh->tris.size()) {
          if(vertexColors.size() != trimesh->verts.size()) {
            //flat shaded
            if(creaseAngle == 0)
              DrawGLTris(*trimesh);
            else {
              //smooth shaded
              glShadeModel(GL_SMOOTH);
              glBegin(GL_TRIANGLES);
              for(size_t i=0;i<trimesh->tris.size();i++) {
                const IntTriple&t=trimesh->tris[i];
                const Vector3& a=trimesh->verts[t.a];
                const Vector3& b=trimesh->verts[t.b];
                const Vector3& c=trimesh->verts[t.c];
                const Vector3& na = vertexNormals[t.a];
                const Vector3& nb = vertexNormals[t.b];
                const Vector3& nc = vertexNormals[t.c];
                glNormal3f(na.x,na.y,na.z);
                glVertex3f(a.x,a.y,a.z);
                glNormal3f(nb.x,nb.y,nb.z);
                glVertex3f(b.x,b.y,b.z);
                glNormal3f(nc.x,nc.y,nc.z);
                glVertex3f(c.x,c.y,c.z);
              }
              glEnd();
            }
            /*
            //testing vertex arrays -- not perceptibly faster.
            vector<float> vbuf(trimesh->verts.size()*3);
            vector<float> nbuf(trimesh->verts.size()*3,0.0);
            vector<unsigned int> ibuf(trimesh->tris.size()*3);
            for(size_t i=0;i<trimesh->tris.size();i++) {
              Math3D::Vector3 normal = trimesh->TriangleNormal(i);
              ibuf[i*3] = trimesh->tris[i].a;
              ibuf[i*3+1] = trimesh->tris[i].b;
              ibuf[i*3+2] = trimesh->tris[i].c;
              int aofs = trimesh->tris[i].a*3;
              int bofs = trimesh->tris[i].b*3;
              int cofs = trimesh->tris[i].c*3;
              nbuf[aofs] += normal.x;
              nbuf[aofs+1] += normal.y;
              nbuf[aofs+2] += normal.z;
              nbuf[bofs] += normal.x;
              nbuf[bofs+1] += normal.y;
              nbuf[bofs+2] += normal.z;
              nbuf[cofs] += normal.x;
              nbuf[cofs+1] += normal.y;
              nbuf[cofs+2] += normal.z;
            }
            for(size_t i=0;i<trimesh->verts.size();i++) {
              vbuf[i*3] = trimesh->verts[i].x;
              vbuf[i*3+1] = trimesh->verts[i].y;
              vbuf[i*3+2] = trimesh->verts[i].z;
              float scale = float(1.0/Vector3(nbuf[i*3],nbuf[i*3+1],nbuf[i*3+2]).norm());
              if(IsFinite(scale)) {
                nbuf[i*3] *= scale;
                nbuf[i*3+1] *= scale;
                nbuf[i*3+2] *= scale;
              }
            }
            */
            /*
            vector<float> vbuf(trimesh->tris.size()*9);
            vector<float> nbuf(trimesh->tris.size()*9);
            for(size_t i=0;i<trimesh->tris.size();i++) {
              Math3D::Vector3 normal = trimesh->TriangleNormal(i);
              const Math3D::Vector3& a=trimesh->verts[trimesh->tris[i].a];
              const Math3D::Vector3& b=trimesh->verts[trimesh->tris[i].b];
              const Math3D::Vector3& c=trimesh->verts[trimesh->tris[i].c];
              nbuf[i*9] = normal.x;
              nbuf[i*9+1] = normal.y;
              nbuf[i*9+2] = normal.z;
              nbuf[i*9+3] = normal.x;
              nbuf[i*9+4] = normal.y;
              nbuf[i*9+5] = normal.z;
              nbuf[i*9+6] = normal.x;
              nbuf[i*9+7] = normal.y;
              nbuf[i*9+8] = normal.z;
              vbuf[i*9+0] = a.x;
              vbuf[i*9+1] = a.y;
              vbuf[i*9+2] = a.z;
              vbuf[i*9+3] = b.x;
              vbuf[i*9+4] = b.y;
              vbuf[i*9+5] = b.z;
              vbuf[i*9+6] = c.x;
              vbuf[i*9+7] = c.y;
              vbuf[i*9+8] = c.z;
            }
            */
            /*
            glVertexPointer(3,GL_FLOAT,0,&vbuf[0]);
            glNormalPointer(GL_FLOAT,0,&nbuf[0]);
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_NORMAL_ARRAY);
            //glDrawArrays(GL_TRIANGLES,0,trimesh->tris.size()*3);
            glDrawElements(GL_TRIANGLES,trimesh->tris.size()*3,GL_UNSIGNED_INT,&ibuf[0]);
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glVertexPointer(3,GL_FLOAT,0,0);
            glNormalPointer(GL_FLOAT,0,0);
            */
          }
          else {
            //vertex colors given!
            glDisable(GL_LIGHTING);
            glShadeModel(GL_SMOOTH);
            glBegin(GL_TRIANGLES);
            for(size_t i=0;i<trimesh->tris.size();i++) {
              const IntTriple&t=trimesh->tris[i];
              const Vector3& a=trimesh->verts[t.a];
              const Vector3& b=trimesh->verts[t.b];
              const Vector3& c=trimesh->verts[t.c];
              //Vector3 n = trimesh->TriangleNormal(i);
              //glNormal3f(n.x,n.y,n.z);
              SetTintedColor(vertexColors[t.a],tintColor,tintStrength);
              glVertex3f(a.x,a.y,a.z);
              SetTintedColor(vertexColors[t.b],tintColor,tintStrength);
              glVertex3f(b.x,b.y,b.z);
              SetTintedColor(vertexColors[t.c],tintColor,tintStrength);
              glVertex3f(c.x,c.y,c.z);
            }
            glEnd();
            
            glEnable(GL_LIGHTING);
          }
        }
        else {
          bool useFaceColors = (faceColors.size()==trimesh->tris.size());
          bool useTexCoords = (texcoords.size()==trimesh->verts.size());
          if(useFaceColors)
            glDisable(GL_LIGHTING);
          glBegin(GL_TRIANGLES);
          for(size_t i=0;i<trimesh->tris.size();i++) {
            const IntTriple&t=trimesh->tris[i];
            const Vector3& a=trimesh->verts[t.a];
            const Vector3& b=trimesh->verts[t.b];
            const Vector3& c=trimesh->verts[t.c];
            Vector3 n = trimesh->TriangleNormal(i);
            if(useFaceColors)
              SetTintedColor(faceColors[i],tintColor,tintStrength);
            glNormal3f(n.x,n.y,n.z);
            if(useTexCoords) {
              glTexCoord2f(texcoords[t.a].x,texcoords[t.a].y);
              glVertex3f(a.x,a.y,a.z);
              glTexCoord2f(texcoords[t.b].x,texcoords[t.b].y);
              glVertex3f(b.x,b.y,b.z);
              glTexCoord2f(texcoords[t.c].x,texcoords[t.c].y);
              glVertex3f(c.x,c.y,c.z);
            }
            else {
              glVertex3f(a.x,a.y,a.z);
              glVertex3f(b.x,b.y,b.z);
              glVertex3f(c.x,c.y,c.z);
            }
          }
          glEnd();
          if(useFaceColors)
            glEnable(GL_LIGHTING);
        }
      }
      else if(geom->type == AnyGeometry3D::Type::Primitive) {
        draw(geom->AsPrimitive());
      }
      faceDisplayList.endCompile();
    }
    else {
      //Timer timer;
      faceDisplayList.call();
    }

    //cleanup, reset GL state
    if(tex1D) {
      glDisable(GL_TEXTURE_1D);
    }
    else if(tex2D) {
      glDisable(GL_TEXTURE_2D);
    }
    if(texgen.size() >= 1)
      glDisable(GL_TEXTURE_GEN_S);
    if(texgen.size() >= 2)
      glDisable(GL_TEXTURE_GEN_T);
    if(texgen.size() >= 3)
      glDisable(GL_TEXTURE_GEN_R);
    if(texgen.size() >= 4)
      glDisable(GL_TEXTURE_GEN_Q);

    if(faceColor.rgba[3] != 1.0) {
      glDisable(GL_BLEND); 
    }
  }

  if(doDrawEdges) {
    const Meshing::TriMesh* trimesh = NULL;
    if(geom->type == AnyGeometry3D::Type::ImplicitSurface ||
      geom->type == AnyGeometry3D::Type::OccupancyGrid || 
      geom->type == AnyGeometry3D::Type::Heightmap ||
      geom->type == AnyGeometry3D::Type::PointCloud ||
      geom->type == AnyGeometry3D::Type::ConvexHull) 
      trimesh = tempMesh.get();
    else if(geom->type == AnyGeometry3D::Type::TriangleMesh) 
      trimesh = &geom->AsTriangleMesh();
    else if(geom->type == AnyGeometry3D::Type::Primitive) {
      if(!edgeDisplayList) {
        edgeDisplayList.beginCompile();
        glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT);
          glDisable(GL_LIGHTING);
          glDepthFunc(GL_LEQUAL);
          draw(geom->AsPrimitive());
        glPopAttrib();
        edgeDisplayList.endCompile();
      }
    }
    if(trimesh) {
      if(!edgeDisplayList) {
        edgeDisplayList.beginCompile();
        glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT);
          glDepthFunc(GL_LEQUAL);
          glDisable(GL_LIGHTING);
          vector<set<int> > vneighbors(trimesh->verts.size());
          for(size_t i=0;i<trimesh->tris.size();i++) {
            vneighbors[trimesh->tris[i].a].insert(trimesh->tris[i].b);
            vneighbors[trimesh->tris[i].a].insert(trimesh->tris[i].c);
            vneighbors[trimesh->tris[i].b].insert(trimesh->tris[i].a);
            vneighbors[trimesh->tris[i].b].insert(trimesh->tris[i].c);
            vneighbors[trimesh->tris[i].c].insert(trimesh->tris[i].a);
            vneighbors[trimesh->tris[i].c].insert(trimesh->tris[i].b);
          }
          glBegin(GL_LINES);
          for(size_t i=0;i<vneighbors.size();i++)
            for(auto j:vneighbors[i]) {
              glVertex3v(trimesh->verts[i]);
              glVertex3v(trimesh->verts[j]);
            }
          glEnd();
        glPopAttrib();
        edgeDisplayList.endCompile();
      }
    }
    if(edgeColor.rgba[3] != 1.0) {
      glEnable(GL_BLEND); 
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    }
    SetTintedColor(edgeColor,tintColor,tintStrength);
    edgeDisplayList.call();
    if(edgeColor.rgba[3] != 1.0) {
      glDisable(GL_BLEND);
    }
  }

  if(doDrawSilhouette) {
    //TODO: do we want to try silhouettes around point clounds or occupancy grids?
    /*
      if(silhouetteRadius > 0) {
        glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_POINT_BIT);
        silhouetteColor.setCurrentGL();
        glDisable(GL_LIGHTING);
        glPointSize(vertexSize + Max(silhouetteRadius,2.0f));
        glBegin(GL_POINTS);
        for(size_t i=0;i<verts->size();i++) {
          const Vector3& v=(*verts)[i];
          glVertex3f(v.x,v.y,v.z);
        }
        glEnd();
        glPopAttrib();
      }
      if(silhouetteRadius > 0) glDepthFunc(GL_LEQUAL);
      */

    Meshing::TriMesh* weldMesh = tempMesh2.get();
    if(!weldMesh) {
      //make the weld mesh from the triangle mesh
      const Meshing::TriMesh* trimesh = NULL;
      if(geom->type == AnyGeometry3D::Type::ImplicitSurface ||
        geom->type == AnyGeometry3D::Type::Heightmap ||
        geom->type == AnyGeometry3D::Type::ConvexHull ) 
        trimesh = tempMesh.get();
      else if(geom->type == AnyGeometry3D::Type::PointCloud) 
        trimesh = tempMesh.get();
      else if(geom->type == AnyGeometry3D::Type::TriangleMesh) 
        trimesh = &geom->AsTriangleMesh();
      
      if(trimesh) {
        weldMesh = new Meshing::TriMeshWithTopology;
        tempMesh2.reset(weldMesh);
        weldMesh->verts = trimesh->verts;
        weldMesh->tris = trimesh->tris;
        Meshing::MergeVertices(*weldMesh,1e-5);
      }
    }

    if(weldMesh) {
      silhouetteColor.setCurrentGL();
      if(silhouetteColor[3] < 1.0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      }

      if(!silhouetteDisplayList) {
        silhouetteDisplayList.beginCompileAndExecute();
        vector<Vector3> weldVertexNormals;
        VertexNormals(*weldMesh,weldVertexNormals);

        glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        glDisable(GL_LIGHTING);
        glBegin(GL_TRIANGLES);
        for(size_t i=0;i<weldMesh->tris.size();i++) {
          const IntTriple&t=weldMesh->tris[i];
          const Vector3& na = weldVertexNormals[t.a];
          const Vector3& nb = weldVertexNormals[t.b];
          const Vector3& nc = weldVertexNormals[t.c];
          Vector3 a=weldMesh->verts[t.a] + silhouetteRadius*na;
          Vector3 b=weldMesh->verts[t.b] + silhouetteRadius*nb;
          Vector3 c=weldMesh->verts[t.c] + silhouetteRadius*nc;
          glVertex3f(a.x,a.y,a.z);
          glVertex3f(b.x,b.y,b.z);
          glVertex3f(c.x,c.y,c.z);
        }
        glEnd();
        glPopAttrib();
        silhouetteDisplayList.endCompile();
      }
      else {
        silhouetteDisplayList.call();
      }
    }
  }

  if(!subAppearances.empty()) {
    //for group geometries
    const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom->AsGroup();
    for(size_t i=0;i<subAppearances.size();i++) {
      bool iDrawFaces=subAppearances[i].drawFaces;
      bool iDrawEdges=subAppearances[i].drawEdges;
      bool iDrawVertices=subAppearances[i].drawVertices;
      bool customAppearance = false;
      if(subgeoms[i].type == AnyGeometry3D::Type::TriangleMesh) {
        auto* mesh = dynamic_cast<Geometry3DTriangleMesh*>(subgeoms[i].data.get());
        if(mesh->appearance) customAppearance = true;
      }
      if(!customAppearance)
        subAppearances[i].CopyMaterialFlat(*this);
      subAppearances[i].drawFaces = iDrawFaces && drawFaces;
      subAppearances[i].drawEdges = iDrawEdges && drawEdges;
      subAppearances[i].drawVertices = iDrawVertices && drawVertices;
      subAppearances[i].DrawGL(e);
      subAppearances[i].drawFaces = iDrawFaces;
      subAppearances[i].drawEdges = iDrawEdges;
      subAppearances[i].drawVertices = iDrawVertices;
    }
  }
}


void GeometryAppearance::SetColor(float r,float g, float b, float a)
{
  vertexColor.set(r,g,b,a);
  edgeColor.set(r,g,b,a);
  faceColor.set(r,g,b,a);
  if(!vertexColors.empty() || !faceColors.empty()) {
    vertexColors.clear();
    faceColors.clear();
    Refresh();
  }
  for(size_t i=0;i<subAppearances.size();i++)
    subAppearances[i].SetColor(r,g,b,a);
}

void GeometryAppearance::SetColor(const GLColor& color)
{
  SetColor(color.rgba[0],color.rgba[1],color.rgba[2],color.rgba[3]);
}

void GeometryAppearance::SetTintColor(const GLColor& color,float fraction)
{
  tintColor = color;
  tintStrength = fraction;
  for(size_t i=0;i<subAppearances.size();i++)
    subAppearances[i].SetTintColor(color,fraction);
}


} //namespace GLDraw
