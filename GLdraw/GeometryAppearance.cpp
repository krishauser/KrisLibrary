#include <log4cxx/logger.h>
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
#include <geometry/Conversions.h>
#include "Timer.h"

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
            LOG4CXX_ERROR(KrisLibrary::logger(),"Texture image doesn't match a supported GL format\n");
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"Texture image doesn't match a supported GL format\n");
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw implicit surface\n");
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::Primitive) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: draw expanded primitive\n");
    draw(geom.AsPrimitive());
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::Group) {
    std::vector<Geometry::AnyCollisionGeometry3D>& subgeoms = geom.GroupCollisionData();
    for(size_t i=0;i<subgeoms.size();i++)
      drawExpanded(subgeoms[i],p);
  }
}	  



GeometryAppearance::GeometryAppearance()
  :geom(NULL),drawVertices(false),drawEdges(false),drawFaces(false),vertexSize(1.0),edgeSize(1.0),
   lightFaces(true),
   vertexColor(1,1,1),edgeColor(1,1,1),faceColor(0.5,0.5,0.5),texWrap(false)
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
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of per-vertex colors?\n"); 
    Refresh();
  }
  if(!rhs.faceColors.empty() && !faceColors.empty()) {
    if(rhs.faceColors.size() != faceColors.size())
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of per-face colors?\n"); 
    Refresh();
  }
  vertexColors=rhs.vertexColors;
  faceColors=rhs.faceColors;
  tex1D=rhs.tex1D;
  tex2D=rhs.tex2D;
  texWrap=rhs.texWrap;
  if(!rhs.texcoords.empty() && !texcoords.empty()) {
    if(rhs.texcoords.size() != texcoords.size())
      LOG4CXX_WARN(KrisLibrary::logger(),"GeometryAppearance::CopyMaterial(): Warning, erroneous size of texture coordinates?\n"); 
    Refresh();
  }
  texcoords=rhs.texcoords;
  texgen=rhs.texgen;
}

void GeometryAppearance::Refresh()
{
  vertexDisplayList.erase();
  faceDisplayList.erase();
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
    if(!implicitSurfaceMesh) implicitSurfaceMesh.reset(new Meshing::TriMesh);
    ImplicitSurfaceToMesh(*g,*implicitSurfaceMesh);
    drawFaces = true;
  }
  else if(geom->type == AnyGeometry3D::PointCloud) {
    drawVertices = true;
    vector<Real> rgb;
    const Meshing::PointCloud3D& pc = geom->AsPointCloud();
    if(pc.GetProperty("rgb",rgb)) {
      //convert real to hex to GLcolor
      vertexColors.resize(rgb.size());
      for(size_t i=0;i<rgb.size();i++) {
  unsigned int col = (unsigned int)rgb[i];
  vertexColors[i].set(((col&0xff0000)>>16) / 255.0,
          ((col&0xff00)>>8) / 255.0,
          (col&0xff) / 255.0);
      }
    }
    if(pc.GetProperty("rgba",rgb)) {
      //convert real to hex to GLcolor
      //following PCD, this is actuall A-RGB
      vertexColors.resize(rgb.size());
      for(size_t i=0;i<rgb.size();i++) {
  unsigned int col = (unsigned int)rgb[i];
  vertexColors[i].set(((col&0xff0000)>>16) / 255.0,
          ((col&0xff00)>>8) / 255.0,
          (col&0xff) / 255.0,
          ((col&0xff000000)>>24) / 255.0);
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
    vertexColors[i].rgba[3] = rgb[i]/255.0;
  }
      }
      else {
	vertexColors.resize(rgb.size());
	for(size_t i=0;i<rgb.size();i++) {
	  vertexColors[i] = vertexColor.rgba;
	  vertexColors[i].rgba[3] = rgb[i]/255.0;
	}
      }
    }
    if(pc.IsStructured()) {
      //draw mesh rather than points
      drawFaces = true;
      drawVertices = false;
      if(!implicitSurfaceMesh) implicitSurfaceMesh.reset(new Meshing::TriMesh);
      PointCloudToMesh(pc,*implicitSurfaceMesh,*this,0.02);
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

void GeometryAppearance::DrawGL()
{
  if(drawVertices) {   
    const vector<Vector3>* verts = NULL;
    if(geom->type == AnyGeometry3D::ImplicitSurface) 
      verts = &implicitSurfaceMesh->verts;
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
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, vertexColors wrong size\n");
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

  if(drawFaces) {
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
	trimesh = implicitSurfaceMesh.get();
      else if(geom->type == AnyGeometry3D::PointCloud) 
  trimesh = implicitSurfaceMesh.get();
      else if(geom->type == AnyGeometry3D::TriangleMesh) 
	trimesh = &geom->AsTriangleMesh();
      else if(geom->type == AnyGeometry3D::Primitive) 
  draw(geom->AsPrimitive());

      //LOG4CXX_INFO(KrisLibrary::logger(),"Compiling face display list "<<trimesh->tris.size());

      //draw the mesh
      if(trimesh) {
	if(!texcoords.empty() && texcoords.size()!=trimesh->verts.size())
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"GeometryAppearance: warning, texcoords wrong size: "<<(int)texcoords.size()<<" vs "<<(int)trimesh->verts.size());
	if(texcoords.size()!=trimesh->verts.size() && faceColors.size()!=trimesh->tris.size()) {
	  if(vertexColors.size() != trimesh->verts.size()) {
	    DrawGLTris(*trimesh);
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

  //TODO edges?
  
  //for group geometries
  for(size_t i=0;i<subAppearances.size();i++) {
    subAppearances[i].DrawGL();
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
