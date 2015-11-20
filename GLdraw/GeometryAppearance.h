#ifndef GLDRAW_GEOMETRY_APPEARANCE_H
#define GLDRAW_GEOMETRY_APPEARANCE_H

#include <utils/SmartPointer.h>
#include <math3d/primitives.h>
#include <meshing/TriMesh.h>
#include "GLColor.h"
#include "GLTexture1D.h"
#include "GLTexture2D.h"
#include "GLDisplayList.h"

namespace Geometry {
  //forward declaration
  class AnyGeometry3D;
  class AnyCollisionGeometry3D;
}// namespace Geometry

namespace GLDraw {

void draw(const Geometry::AnyGeometry3D& geom);
void drawPoints(const Geometry::AnyGeometry3D& geom);
void drawFaces(const Geometry::AnyGeometry3D& geom);

///draw the collision geometry in its world coordinates
void drawWorld(const Geometry::AnyCollisionGeometry3D& geom);
///draw the collision geometry's points in its world coordinates
void drawPointsWorld(const Geometry::AnyCollisionGeometry3D& geom);
///draw the collision geometry's faces in its world coordinates
void drawFacesWorld(const Geometry::AnyCollisionGeometry3D& geom);

///draw the expansion of the collision geometry, in its local coordinates
void drawExpanded(Geometry::AnyCollisionGeometry3D& geom,Math::Real p=-1);

class GeometryAppearance
{
 public:
  GeometryAppearance();
  void Set(const Geometry::AnyGeometry3D& geom);
  void Set(const Geometry::AnyCollisionGeometry3D& geom);
  ///Call this if the underlying geometry, per-element colors, or texture
  ///coordinates change
  void Refresh();
  ///Draws the geometry using OpenGL
  void DrawGL();

  ///Geometry pointer
  const Geometry::AnyGeometry3D* geom;
  ///Mesh computed for implicit surfaces
  Meshing::TriMesh mesh;
  ///The display lists
  GLDisplayList vertexDisplayList,faceDisplayList;
  ///For group geometries
  std::vector<GeometryAppearance> subAppearances;

  bool drawVertices,drawEdges,drawFaces;
  float vertexSize,edgeSize;
  ///Use lighting or not (with lighting, will always do ambient and diffuse)
  bool lightFaces;
  ///Global color
  GLColor vertexColor,edgeColor,faceColor;
  ///Optional: per-element colors
  std::vector<GLColor> vertexColors,faceColors;
  ///Optional: set to non-null if you want to texture the object
  SmartPointer<GLTexture1D> tex1D;
  SmartPointer<GLTexture2D> tex2D;
  ///Optional: per-element texture mapping coordinates (up to 2D)
  std::vector<Math3D::Vector2> texcoords;
  ///Optional: linear texture generation coefficients for S and T coordinates
  ///S = c[0]^T p (p in homogeneous coordinates)
  ///T = c[1]^T p
  std::vector<Math3D::Vector4> texgen;
};

} //namespace GLDraw

#endif
