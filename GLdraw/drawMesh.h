#ifndef GLDRAW_DRAW_MESH_H
#define GLDRAW_DRAW_MESH_H

#include <KrisLibrary/meshing/TriMesh.h>
#include "GL.h"
#include "drawextra.h"

namespace GLDraw {

inline void DrawGLTris(const Meshing::TriMesh& mesh)
{
  glBegin(GL_TRIANGLES);
  for(size_t i=0;i<mesh.tris.size();i++) {
	  Math3D::Vector3 normal = mesh.TriangleNormal(i);
    glNormal3v(normal);
    glVertex3v(mesh.verts[mesh.tris[i].a]);
    glVertex3v(mesh.verts[mesh.tris[i].b]);
    glVertex3v(mesh.verts[mesh.tris[i].c]);
  }
  glEnd();
}

inline void DrawGLEdges(const Meshing::TriMesh& mesh) 
{
  for(size_t i=0;i<mesh.tris.size();i++) {
    glBegin(GL_LINE_LOOP);
    glVertex3v(mesh.verts[mesh.tris[i].a]);
    glVertex3v(mesh.verts[mesh.tris[i].b]);
    glVertex3v(mesh.verts[mesh.tris[i].c]);
    glEnd();
  }
}

inline void DrawGLVerts(const Meshing::TriMesh& mesh) 
{
  glBegin(GL_POINTS);
  for(size_t i=0;i<mesh.verts.size();i++) 
    glVertex3v(mesh.verts[i]);
  glEnd();
}

} //namespace GLDraw

#endif
