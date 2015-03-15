#include "GeometryAppearance.h"
#include "drawMesh.h"
#include "drawgeometry.h"
#include "drawextra.h"
#include <meshing/PointCloud.h>
#include <meshing/VolumeGrid.h>
#include <meshing/MarchingCubes.h>
#include <meshing/Expand.h>

using namespace Geometry;

namespace GLDraw {

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
    verts = &AnyCast<Meshing::TriMesh>(&geom.data)->verts;
  else if(geom.type == AnyGeometry3D::PointCloud) 
    verts = &AnyCast<Meshing::PointCloud3D>(&geom.data)->points;
  else if(geom.type == AnyGeometry3D::Group) {
    const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom.AsGroup();
    for(size_t i=0;i<subgeoms.size();i++)
      drawPoints(subgeoms[i]);
  }
  if(verts) {
    glBegin(GL_POINTS);
    for(size_t i=0;i<verts->size();i++) {
      glVertex3v((*verts)[i]);
    }
    glEnd();
  }
}

void drawFaces(const Geometry::AnyGeometry3D& geom)
{
  const Meshing::TriMesh* trimesh = NULL;
  if(geom.type == AnyGeometry3D::TriangleMesh) 
    trimesh = AnyCast<Meshing::TriMesh>(&geom.data);
  else if(geom.type == AnyGeometry3D::Group) {
    const std::vector<Geometry::AnyGeometry3D>& subgeoms = geom.AsGroup();
    for(size_t i=0;i<subgeoms.size();i++)
      drawFaces(subgeoms[i]);
  }
  else if(geom.type == AnyGeometry3D::Primitive) 
    draw(*AnyCast<GeometricPrimitive3D>(&geom.data));  

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
    fprintf(stderr,"TODO: draw implicit surface\n");
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::Primitive) {
    fprintf(stderr,"TODO: draw expanded primitive\n");
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
   vertexColor(1,1,1),edgeColor(1,1,1),faceColor(0.5,0.5,0.5)
{}

void GeometryAppearance::Refresh()
{
  vertexDisplayList.erase();
  faceDisplayList.erase();
}

void GeometryAppearance::Set(const Geometry::AnyCollisionGeometry3D& _geom)
{
  geom = &_geom;
  if(geom->type == AnyGeometry3D::ImplicitSurface) {
    const Meshing::VolumeGrid* g = AnyCast<Meshing::VolumeGrid>(&geom->data);
    MarchingCubes(g->value,0,g->bb,mesh);
    drawFaces = true;
  }
  else if(geom->type == AnyGeometry3D::PointCloud) {
    drawVertices = true;
    vector<Real> rgb;
    if(AnyCast<Meshing::PointCloud3D>(&geom->data)->GetProperty("rgb",rgb)) {
      //convert real to hex to GLcolor
      vertexColors.resize(rgb.size());
      for(size_t i=0;i<rgb.size();i++) {
	int col = (int)rgb[i];
	vertexColors[i].set(((col&0xff0000)>>16) / 255.0,
			    ((col&0xff00)>>8) / 255.0,
			    (col&0xff) / 255.0);
      }
    }
    if(AnyCast<Meshing::PointCloud3D>(&geom->data)->GetProperty("rgba",rgb)) {
      //convert real to hex to GLcolor
      //following PCD, this is actuall A-RGB
      vertexColors.resize(rgb.size());
      for(size_t i=0;i<rgb.size();i++) {
	int col = (int)rgb[i];
	vertexColors[i].set(((col&0xff0000)>>16) / 255.0,
			    ((col&0xff00)>>8) / 255.0,
			    (col&0xff) / 255.0,
			    ((col&0xff000000)>>24) / 255.0);
      }
    }
    if(AnyCast<Meshing::PointCloud3D>(&geom->data)->GetProperty("opacity",rgb)) {
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
    if(AnyCast<Meshing::PointCloud3D>(&geom->data)->GetProperty("c",rgb)) {
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
  }
  else if(geom->type == AnyGeometry3D::Group) {
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
  else
    drawFaces = true;
  Refresh();
}

void GeometryAppearance::Set(const AnyGeometry3D& _geom)
{
  geom = &_geom;
  if(geom->type == AnyGeometry3D::ImplicitSurface) {
    const Meshing::VolumeGrid* g = AnyCast<Meshing::VolumeGrid>(&geom->data);
    MarchingCubes(g->value,0,g->bb,mesh);
    drawFaces = true;
  }
  else if(geom->type == AnyGeometry3D::PointCloud) {
    drawVertices = true;
    vector<Real> rgb;
    if(AnyCast<Meshing::PointCloud3D>(&geom->data)->GetProperty("rgb",rgb)) {
      //convert real to hex to GLcolor
      vertexColors.resize(rgb.size());
      for(size_t i=0;i<rgb.size();i++) {
	int col = (int)rgb[i];
	vertexColors[i].set(((col&0xff0000)>>16) / 255.0,
			    ((col&0xff00)>>8) / 255.0,
			    (col&0xff) / 255.0);
      }
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
      verts = &mesh.verts;
    else if(geom->type == AnyGeometry3D::TriangleMesh) 
      verts = &AnyCast<Meshing::TriMesh>(&geom->data)->verts;
    else if(geom->type == AnyGeometry3D::PointCloud) 
      verts = &AnyCast<Meshing::PointCloud3D>(&geom->data)->points;
    if(verts) {
      //compile the vertex display list
      if(!vertexDisplayList) {
	vertexDisplayList.beginCompile();
	if(!vertexColors.empty() && vertexColors.size() != verts->size())
	  fprintf(stderr,"GeometryAppearance: warning, vertexColors wrong size\n");
	glBegin(GL_POINTS);
	for(size_t i=0;i<verts->size();i++) {
	  if(vertexColors.size()==verts->size()) vertexColors[i].setCurrentGL();
	  glVertex3v((*verts)[i]);
	}
	glEnd();
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
      glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);

      if(tex1D) {
	glEnable(GL_TEXTURE_1D);
	tex1D->setCurrentGL();
      }
      else if(tex2D) {
	glEnable(GL_TEXTURE_2D);
	tex2D->setCurrentGL();
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
	trimesh = &mesh;
      if(geom->type == AnyGeometry3D::TriangleMesh) 
	trimesh = AnyCast<Meshing::TriMesh>(&geom->data);

      //draw the mesh
      if(trimesh) {
	if(!texcoords.empty() && texcoords.size()!=trimesh->verts.size())
	  fprintf(stderr,"GeometryAppearance: warning, texcoords wrong size\n");
	if(texcoords.size()!=trimesh->verts.size() && faceColors.size()!=trimesh->tris.size()) {
	  DrawGLTris(*trimesh);
	}
	else {
	  glBegin(GL_TRIANGLES);
	  for(size_t i=0;i<trimesh->tris.size();i++) {
	    const IntTriple&t=trimesh->tris[i];
	    if(faceColors.size()==trimesh->tris.size())
	      faceColors[i].setCurrentGL();
	    glNormal3v(trimesh->TriangleNormal(i));
	    glTexCoord2v(texcoords[t.a]);
	    glVertex3v(trimesh->verts[t.a]);
	    glTexCoord2v(texcoords[t.b]);
	    glVertex3v(trimesh->verts[t.b]);
	    glTexCoord2v(texcoords[t.c]);
	    glVertex3v(trimesh->verts[t.c]);
	  }
	  glEnd();
	}
      }
      else if(geom->type == AnyGeometry3D::Primitive) {
	draw(*AnyCast<GeometricPrimitive3D>(&geom->data));
      }
      faceDisplayList.endCompile();
    }
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

} //namespace GLDraw
