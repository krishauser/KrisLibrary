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

using namespace Geometry;

namespace GLDraw {

  void TransferTexture1D(GLTextureObject& obj,const Image& img)
  {
    GLTexture1D tex;
    tex.texObj = obj;
    int n=img.w*img.h;
    switch(img.format) {
    case Image::R8G8B8:
      {
        unsigned char* buf = new unsigned char[img.num_bytes];
        for(int i=0;i<n;i++) {
          buf[i*3] = img.data[i*3+2];
          buf[i*3+1] = img.data[i*3+1];
          buf[i*3+2] = img.data[i*3];
        }
        tex.setRGB(buf,n);
        delete [] buf;
      }
      break;
    case Image::A8R8G8B8:
      {
        unsigned char* buf = new unsigned char[img.num_bytes];
        for(int i=0;i<n;i++) {
          buf[i*3] = img.data[i*3+3];
          buf[i*3+1] = img.data[i*3+2];
          buf[i*3+2] = img.data[i*3+1];
          buf[i*3+3] = img.data[i*3+0];
        }
        tex.setRGBA(buf,n);
        delete [] buf;
      }
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
    switch(img.format) {
    case Image::R8G8B8:
      {
        unsigned char* buf = new unsigned char[img.num_bytes];
        for(int i=0;i<img.w*img.h;i++) {
          buf[i*3] = img.data[i*3+2];
          buf[i*3+1] = img.data[i*3+1];
          buf[i*3+2] = img.data[i*3];
        }
        tex.setRGB(buf,img.w,img.h);
      }
      break;
    case Image::A8R8G8B8:
      {
        unsigned char* buf = new unsigned char[img.num_bytes];
        for(int i=0;i<img.w*img.h;i++) {
          buf[i*3] = img.data[i*3+3];
          buf[i*3+1] = img.data[i*3+2];
          buf[i*3+2] = img.data[i*3+1];
          buf[i*3+3] = img.data[i*3+0];
        }
        tex.setRGBA(buf,img.w,img.h);
      }
      break;
    case Image::A8:
      tex.setLuminance(img.data,img.w,img.h);
      break;
    default:
            LOG4CXX_ERROR(KrisLibrary::logger(),"Texture image doesn't match a supported GL format");
      break;
    }
  }

  void draw(const Geometry::AnyGeometry3D& geom)
  {
    if(geom.type == AnyGeometry3D::PointCloud) 
      drawPoints(geom);
    else if(geom.type == AnyGeometry3D::Group) {
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
    if(geom.type == AnyGeometry3D::TriangleMesh) 
      verts = &geom.AsTriangleMesh().verts;
    else if(geom.type == AnyGeometry3D::PointCloud) 
      verts = &geom.AsPointCloud().points;
    else if(geom.type == AnyGeometry3D::Group) {
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
    if(geom.type == AnyGeometry3D::TriangleMesh) 
      trimesh = &geom.AsTriangleMesh();
    else if(geom.type == AnyGeometry3D::Group) {
      const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom.AsGroup();
      for(size_t i=0;i<subgeoms.size();i++)
        drawFaces(subgeoms[i]);
    }
    else if(geom.type == AnyGeometry3D::Primitive) 
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
    if(geom.type == Geometry::AnyCollisionGeometry3D::TriangleMesh) {
      Meshing::TriMesh m;
      geom.TriangleMeshCollisionData().CalcTriNeighbors();
      geom.TriangleMeshCollisionData().CalcIncidentTris();
      Meshing::Expand2Sided(geom.TriangleMeshCollisionData(),p,3,m);
      DrawGLTris(m);
    }
    else if(geom.type == Geometry::AnyCollisionGeometry3D::PointCloud) {
      const Meshing::PointCloud3D& pc = geom.AsPointCloud();
      for(size_t i=0;i<pc.points.size();i++) {
        glPushMatrix();
        glTranslate(pc.points[i]);
        drawSphere(p,8,4);
        glPopMatrix();
      }
    }
    else if(geom.type == Geometry::AnyCollisionGeometry3D::ImplicitSurface) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw implicit surface");
    }
    else if(geom.type == Geometry::AnyCollisionGeometry3D::Primitive) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw expanded primitive");
      draw(geom.AsPrimitive());
    }
    else if(geom.type == Geometry::AnyCollisionGeometry3D::Group) {
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

void CreaseMesh(Meshing::TriMeshWithTopology& in,Meshing::TriMesh& out,Real creaseRads)
{
  if(in.incidentTris.empty())
    in.CalcIncidentTris();
  if(in.triNeighbors.empty())
    in.CalcTriNeighbors();
  Real cosCreaseRads = Cos(creaseRads);

  vector<Vector3> triNormals(in.tris.size());
  for(size_t i=0;i<in.tris.size();i++)
    triNormals[i] = in.TriangleNormal(i);
  out.tris.resize(in.tris.size());
  fill(out.tris.begin(),out.tris.end(),IntTriple(-1,-1,-1));
  vector<int> outVertToIn;
  for(size_t i=0;i<in.verts.size();i++) {
    //theres a potential graph of creases here
    auto tris = in.incidentTris[i];
    Graph::UndirectedGraph<int,int> tgraph;
    map<int,int> triGraphIndex;
    for(auto t: tris)
      triGraphIndex[t] = tgraph.AddNode(t);
    for(size_t s=0;s<tris.size();s++) {
      int iprev,inext;
      if(in.tris[tris[s]].a == int(i)) {
        iprev = in.triNeighbors[tris[s]].b;
        inext = in.triNeighbors[tris[s]].c;
      }
      else if(in.tris[tris[s]].b == int(i)) {
        iprev = in.triNeighbors[tris[s]].c;
        inext = in.triNeighbors[tris[s]].a;
      }
      else {
        iprev = in.triNeighbors[tris[s]].a;
        inext = in.triNeighbors[tris[s]].b;
      }
      if (iprev >= 0 && triNormals[tris[s]].dot(triNormals[iprev]) >= cosCreaseRads) {
        int previndex = triGraphIndex[iprev];
        if(!tgraph.HasEdge(s,previndex)) 
          tgraph.AddEdge(s,previndex);
      }
      if (inext >= 0 && triNormals[tris[s]].dot(triNormals[inext]) >= cosCreaseRads) {
        int nextindex = triGraphIndex[inext];
        if(!tgraph.HasEdge(s,nextindex))
          tgraph.AddEdge(s,nextindex);
      }
    }
    Graph::ConnectedComponents ccs;
    ccs.Compute(tgraph);
    vector<int> reps;
    ccs.GetRepresentatives(reps);
    //add a new vertex for each cc
    vector<int> cc;
    for(auto r:reps) {
      ccs.EnumerateComponent(r,cc);
      int nvout = (int)out.verts.size();
      outVertToIn.push_back(nvout);
      for(auto n:cc) {
        int t=tgraph.nodes[n];
        Assert(in.tris[t].getIndex(i) >= 0);
        out.tris[t][in.tris[t].getIndex(i)] = nvout;
      }
      out.verts.push_back(in.verts[i]);
    }
  }
  for(const auto& t:out.tris)
    Assert(t.a >= 0 && t.b >= 0 && t.c >= 0);
}


GeometryAppearance::GeometryAppearance()
  :geom(NULL),drawVertices(false),drawEdges(false),drawFaces(false),vertexSize(1.0),edgeSize(1.0),
   lightFaces(true),
   vertexColor(1,1,1),edgeColor(1,1,1),faceColor(0.5,0.5,0.5),texWrap(false),
   creaseAngle(0),silhouetteRadius(0),silhouetteColor(0,0,0)
{}

void GeometryAppearance::CopyMaterial(const GeometryAppearance& rhs)
{
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
  texWrap=rhs.texWrap;
  if(!rhs.texcoords.empty() && !texcoords.empty()) {
    if(rhs.texcoords.size() != texcoords.size())
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of texture coordinates?"); 
    Refresh();
  }
  texcoords=rhs.texcoords;
  texgen=rhs.texgen;
  if(silhouetteRadius != rhs.silhouetteRadius)
    silhouetteDisplayList.erase();
  silhouetteRadius=rhs.silhouetteRadius;
  silhouetteColor=rhs.silhouetteColor;
  creaseAngle=rhs.creaseAngle;
}

void GeometryAppearance::Refresh()
{
  vertexDisplayList.erase();
  edgeDisplayList.erase();
  faceDisplayList.erase();
  silhouetteDisplayList.erase();
  textureObject.cleanup();
}

void GeometryAppearance::Set(const Geometry::AnyCollisionGeometry3D& _geom)
{
  geom = &_geom;
  if(geom->type == AnyGeometry3D::ImplicitSurface) {
    Set(*geom);
  }
  else if(geom->type == AnyGeometry3D::PointCloud) {
    Set(*geom);
  }
  else if(geom->type == AnyGeometry3D::Group) {
    if(!_geom.CollisionDataInitialized()) {
      const std::vector<Geometry::AnyGeometry3D>& subgeoms = _geom.AsGroup();
      subAppearances.resize(subgeoms.size());
      for(size_t i=0;i<subAppearances.size();i++) {
        subAppearances[i].Set(subgeoms[i]);
        subAppearances[i].vertexSize = vertexSize;
        subAppearances[i].edgeSize = edgeSize;
        subAppearances[i].lightFaces = lightFaces;
        subAppearances[i].vertexColor = vertexColor;
        subAppearances[i].edgeColor = edgeColor;
        subAppearances[i].faceColor = faceColor;
      }
    }
    else {
      const std::vector<Geometry::AnyCollisionGeometry3D>& subgeoms = _geom.GroupCollisionData();
      subAppearances.resize(subgeoms.size());
      for(size_t i=0;i<subAppearances.size();i++) {
        subAppearances[i].Set(subgeoms[i]);
        subAppearances[i].vertexSize = vertexSize;
        subAppearances[i].edgeSize = edgeSize;
        subAppearances[i].lightFaces = lightFaces;
        subAppearances[i].vertexColor = vertexColor;
        subAppearances[i].edgeColor = edgeColor;
        subAppearances[i].faceColor = faceColor;
      }
    }
  }
  else
    drawFaces = true;
  Refresh();
}

void GeometryAppearance::Set(const AnyGeometry3D& _geom)
{
  geom = &_geom;
  if(geom->type == AnyGeometry3D::ImplicitSurface) {
    const Meshing::VolumeGrid* g = &geom->AsImplicitSurface();
    if(!tempMesh) tempMesh.reset(new Meshing::TriMesh);
    ImplicitSurfaceToMesh(*g,*tempMesh);
    drawFaces = true;
  }
  else if(geom->type == AnyGeometry3D::PointCloud) {
    Timer timer;
    drawVertices = true;
    vector<Real> rgb;
    const Meshing::PointCloud3D& pc = geom->AsPointCloud();
    const static Real scale = 1.0/255.0;
    if(pc.GetProperty("rgb",rgb)) {
      //convert real to hex to GLcolor
      vertexColors.resize(rgb.size());
      for(size_t i=0;i<rgb.size();i++) {
        if(pc.points[i].z <= 0) continue;
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
    if(pc.GetProperty("opacity",rgb)) {
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
    drawFaces = false;
    drawVertices = true;
    tempMesh = NULL;
    tempMesh2 = NULL;
    if(pc.IsStructured()) {
      //draw mesh rather than points
      drawFaces = true;
      drawVertices = false;
      if(!tempMesh) tempMesh.reset(new Meshing::TriMesh);
      //PointCloudToMesh(pc,*tempMesh,*this,0.02);
      PointCloudToMesh(pc,*tempMesh,0.02);
    }
  }
  else if(geom->type == AnyGeometry3D::Group) {
    const std::vector<Geometry::AnyGeometry3D>& subgeoms = _geom.AsGroup();
    subAppearances.resize(subgeoms.size());
    for(size_t i=0;i<subAppearances.size();i++) {
      subAppearances[i].Set(subgeoms[i]);
      subAppearances[i].vertexSize = vertexSize;
      subAppearances[i].edgeSize = edgeSize;
      subAppearances[i].lightFaces = lightFaces;
      subAppearances[i].vertexColor = vertexColor;
      subAppearances[i].edgeColor = edgeColor;
      subAppearances[i].faceColor = faceColor;
    }
  }
  else 
    drawFaces = true;
  Refresh();
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
  else if(e == TRANSPARENT) {
    doDrawVertices = drawVertices && (vertexColor.rgba[3] < 1.0);
    doDrawEdges = drawEdges && (edgeColor.rgba[3] < 1.0);
    doDrawFaces = drawFaces && (faceColor.rgba[3] < 1.0);
    doDrawSilhouette = drawFaces && (silhouetteRadius > 0) && (faceColor.rgba[3] < 1.0 || silhouetteColor.rgba[3] < 1.0);
  }
  else if(e == OPAQUE) {
    doDrawVertices = drawVertices && (vertexColor.rgba[3] >= 1.0);
    doDrawEdges = drawEdges && (edgeColor.rgba[3] >= 1.0);
    doDrawFaces = drawFaces && (faceColor.rgba[3] >= 1.0);
    doDrawSilhouette = drawFaces && (silhouetteRadius > 0) && (faceColor.rgba[3] >= 1.0 && silhouetteColor.rgba[3] >= 1.0);
  }
  else
    FatalError("Invalid Element specified");
  if(doDrawVertices) {   
    const vector<Vector3>* verts = NULL;  
    if(geom->type == AnyGeometry3D::ImplicitSurface) 
      verts = &tempMesh->verts;
    else if(geom->type == AnyGeometry3D::TriangleMesh) 
      verts = &geom->AsTriangleMesh().verts;
    else if(geom->type == AnyGeometry3D::PointCloud) 
      verts = &geom->AsPointCloud().points;
    if(verts) {
      //compile the vertex display list
      if(!vertexDisplayList) {
        //LOG4CXX_INFO(KrisLibrary::logger(),"Compiling vertex display list "<<verts->size());
        vertexDisplayList.beginCompile();
        if(!vertexColors.empty() && vertexColors.size() != verts->size())
                  LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, vertexColors wrong size");
        if(vertexColors.size()==verts->size()) {
          glBegin(GL_POINTS);
          for(size_t i=0;i<verts->size();i++) {
            const Vector3& v=(*verts)[i];
            vertexColors[i].setCurrentGL();
            glVertex3f(v.x,v.y,v.z);
          }
          glEnd();
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
      
      //do the drawing
      glDisable(GL_LIGHTING);
      if(vertexColor.rgba[3] != 1.0 || !vertexColors.empty()) {
        glEnable(GL_BLEND); 
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      }
      if(vertexColors.size()!=verts->size())
        vertexColor.setCurrentGL();
      glPointSize(vertexSize);

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
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,faceColor.rgba);
    }
    else {
      glDisable(GL_LIGHTING);
      glColor4fv(faceColor.rgba);
    }
    //set up the texture coordinates
    if(tex1D || tex2D) {
      if(!textureObject) {
        textureObject.generate();
        if(tex2D) {
          TransferTexture2D(textureObject,*tex2D);
          textureObject.bind(GL_TEXTURE_2D);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
          if(texWrap) {
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
          }
          else {
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
          }
        }
        else {
          TransferTexture1D(textureObject,*tex1D);
          textureObject.bind(GL_TEXTURE_1D);
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
          glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
          if(texWrap)
            glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_REPEAT);
          else
            glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_CLAMP);
        }
      }
      glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
      if(tex1D) {
        glEnable(GL_TEXTURE_1D);
        textureObject.bind(GL_TEXTURE_1D);
      }
      else if(tex2D) {
        glEnable(GL_TEXTURE_2D);
        textureObject.bind(GL_TEXTURE_2D);
      }
    }

    if(!texgen.empty()) {
      glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      float params[4];
      for(int i=0;i<4;i++)
        params[i] = texgen[0][i];
      glTexGenfv(GL_S,GL_OBJECT_PLANE,params);
      glEnable(GL_TEXTURE_GEN_S);
      if(texgen.size() >= 2) {
        glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
        for(int i=0;i<4;i++)
          params[i] = texgen[1][i];
        glTexGenfv(GL_T,GL_OBJECT_PLANE,params);
        glEnable(GL_TEXTURE_GEN_T);
      }
    }
    
    if(!faceDisplayList) {
      faceDisplayList.beginCompile();
      const Meshing::TriMesh* trimesh = NULL;
      if(geom->type == AnyGeometry3D::ImplicitSurface) 
        trimesh = tempMesh.get();
      else if(geom->type == AnyGeometry3D::PointCloud) 
        trimesh = tempMesh.get();
      else if(geom->type == AnyGeometry3D::TriangleMesh) 
        trimesh = &geom->AsTriangleMesh();
      else if(geom->type == AnyGeometry3D::Primitive) 
        draw(geom->AsPrimitive());

      //LOG4CXX_INFO(KrisLibrary::logger(),"Compiling face display list "<<trimesh->tris.size());

      //draw the mesh
      if(trimesh) {
        Meshing::TriMesh creaseMesh;
        vector<Vector3> vertexNormals;

        if(creaseAngle > 0 || silhouetteRadius > 0) {
          Meshing::TriMeshWithTopology* weldMesh = new Meshing::TriMeshWithTopology;
          tempMesh2.reset(weldMesh);
          weldMesh->verts = trimesh->verts;
          weldMesh->tris = trimesh->tris;
          Meshing::MergeVertices(*weldMesh,1e-5);
          if(creaseAngle > 0) {
            CreaseMesh(*weldMesh,creaseMesh,creaseAngle);
            trimesh = &creaseMesh;
            VertexNormals(creaseMesh,vertexNormals);
          }
        }

        if(!texcoords.empty() && texcoords.size()!=trimesh->verts.size())
          LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, texcoords wrong size: "<<(int)texcoords.size()<<" vs "<<(int)trimesh->verts.size());
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
            glBegin(GL_TRIANGLES);
            for(size_t i=0;i<trimesh->tris.size();i++) {
              const IntTriple&t=trimesh->tris[i];
              const Vector3& a=trimesh->verts[t.a];
              const Vector3& b=trimesh->verts[t.b];
              const Vector3& c=trimesh->verts[t.c];
              Vector3 n = trimesh->TriangleNormal(i);
              glNormal3f(n.x,n.y,n.z);
              vertexColors[t.a].setCurrentGL();
              glVertex3f(a.x,a.y,a.z);
              vertexColors[t.b].setCurrentGL();
              glVertex3f(b.x,b.y,b.z);
              vertexColors[t.c].setCurrentGL();
              glVertex3f(c.x,c.y,c.z);
            }
            glEnd();
            glEnable(GL_LIGHTING);
          }
        }
        else {
          glBegin(GL_TRIANGLES);
          for(size_t i=0;i<trimesh->tris.size();i++) {
            const IntTriple&t=trimesh->tris[i];
            const Vector3& a=trimesh->verts[t.a];
            const Vector3& b=trimesh->verts[t.b];
            const Vector3& c=trimesh->verts[t.c];
            Vector3 n = trimesh->TriangleNormal(i);
            if(faceColors.size()==trimesh->tris.size())
              faceColors[i].setCurrentGL();
            glNormal3f(n.x,n.y,n.z);
            glTexCoord2f(texcoords[t.a].x,texcoords[t.a].y);
            glVertex3f(a.x,a.y,a.z);
            glTexCoord2f(texcoords[t.b].x,texcoords[t.b].y);
            glVertex3f(b.x,b.y,b.z);
            glTexCoord2f(texcoords[t.c].x,texcoords[t.c].y);
            glVertex3f(c.x,c.y,c.z);
          }
          glEnd();
        }
      }
      else if(geom->type == AnyGeometry3D::Primitive) {
        draw(geom->AsPrimitive());
      }
      faceDisplayList.endCompile();
    }
    //Timer timer;
    faceDisplayList.call();

    //cleanup, reset GL state
    if(tex1D) {
      glDisable(GL_TEXTURE_1D);
      glDisable(GL_TEXTURE_GEN_S);
    }
    else if(tex2D) {
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
    }

    if(faceColor.rgba[3] != 1.0) {
      glDisable(GL_BLEND); 
    }
  }

  if(doDrawEdges) {
    const Meshing::TriMesh* trimesh = NULL;
    if(geom->type == AnyGeometry3D::ImplicitSurface) 
      trimesh = tempMesh.get();
    else if(geom->type == AnyGeometry3D::PointCloud) 
      trimesh = tempMesh.get();
    else if(geom->type == AnyGeometry3D::TriangleMesh) 
      trimesh = &geom->AsTriangleMesh();
    else if(geom->type == AnyGeometry3D::Primitive) 
      ;
    if(trimesh) {
      if(!edgeDisplayList) {
        edgeDisplayList.beginCompile();
        glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT);
          edgeColor.setCurrentGL();
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
      edgeDisplayList.call();
    }
  }

  if(doDrawSilhouette) {
    //TODO: do we want to try silhouettes around point clounds?
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
      //make the weld mesh from the trinagle mesh
      const Meshing::TriMesh* trimesh = NULL;
      if(geom->type == AnyGeometry3D::ImplicitSurface) 
        trimesh = tempMesh.get();
      else if(geom->type == AnyGeometry3D::PointCloud) 
        trimesh = tempMesh.get();
      else if(geom->type == AnyGeometry3D::TriangleMesh) 
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
      if(!silhouetteDisplayList) {
        silhouetteDisplayList.beginCompile();
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

      silhouetteColor.setCurrentGL();
      if(silhouetteColor[3] < 1.0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      }
      silhouetteDisplayList.call();
    }
  }

  
  //for group geometries
  for(size_t i=0;i<subAppearances.size();i++) {
    subAppearances[i].DrawGL(e);
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

void GeometryAppearance::ModulateColor(const GLColor& color,float fraction)
{
  faceColor.blend(GLColor(faceColor),color,fraction);
  vertexColor.blend(GLColor(vertexColor),color,fraction);
  edgeColor.blend(GLColor(edgeColor),color,fraction);

  if(!vertexColors.empty() || !faceColors.empty()) {
    for(size_t i=0;i<vertexColors.size();i++)
      vertexColors[i].blend(GLColor(vertexColors[i]),color,fraction);
    for(size_t i=0;i<faceColors.size();i++)
      faceColors[i].blend(GLColor(faceColors[i]),color,fraction);
    Refresh();
  }
  for(size_t i=0;i<subAppearances.size();i++)
    subAppearances[i].ModulateColor(color,fraction);
}


} //namespace GLDraw
