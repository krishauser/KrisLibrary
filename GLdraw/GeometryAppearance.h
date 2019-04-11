#ifndef GLDRAW_GEOMETRY_APPEARANCE_H
#define GLDRAW_GEOMETRY_APPEARANCE_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/meshing/TriMesh.h>
#include "GLColor.h"
#include "GLTextureObject.h"
#include "GLDisplayList.h"
#include <KrisLibrary/image/image.h>
#include <memory>

namespace Geometry {
  //forward declaration
  class AnyGeometry3D;
  class AnyCollisionGeometry3D;
}// namespace Geometry
//forward declaration
class Image;

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

/** @brief A class for coloring, texturing, and drawing meshes in OpenGL.
 *
 * GL display lists are created on first DrawGL call.  If the appearance
 * changes geometry or vertex/face colors, Refresh() must be called to
 * rebuild the display lists.
 */
class GeometryAppearance
{
 public:
  enum Element { ALL, VERTICES, EDGES, FACES, SILHOUETTE, TRANSPARENT, OPAQUE };

  GeometryAppearance();
  ///This copies over the "material" information but doesn't change the display lists (if possible)
  void CopyMaterial(const GeometryAppearance& rhs);
  void Set(const Geometry::AnyGeometry3D& geom);
  void Set(const Geometry::AnyCollisionGeometry3D& geom);
  ///Call this if the underlying geometry, per-element colors, texture
  ///coordinates, or silhouette radius change
  void Refresh();
  ///Draws the geometry using OpenGL
  void DrawGL(Element e=ALL);
  ///Sets flat colors, including sub-appearances, and deletes any per-vertex
  ///or per-face colors.  Refresh() does not need to be called.
  void SetColor(float r,float g, float b, float a);
  void SetColor(const GLColor& color);
  ///Modulates the current color, including sub-appearances, and per-vertex
  ///or per-face colors.  Refresh() does not need to be called.
  void ModulateColor(const GLColor& color,float fraction);

  ///Geometry pointer
  const Geometry::AnyGeometry3D* geom;
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
  std::shared_ptr<Image> tex1D,tex2D;
  ///If true, the texture will wrap.  Default false
  bool texWrap; 
  ///Optional: per-element texture mapping coordinates (up to 2D)
  std::vector<Math3D::Vector2> texcoords;
  ///Optional: linear texture generation coefficients for S and T coordinates
  ///S = c[0]^T p (p in homogeneous coordinates)
  ///T = c[1]^T p
  std::vector<Math3D::Vector4> texgen;
  ///Optional: draw meshes with a crease if the angle between triangles is
  ///greater than this threshold, in radians (default 0)
  float creaseAngle;
  ///Optional: draw meshes with a silhouette outline (default 0)
  float silhouetteRadius;
  GLColor silhouetteColor;
  
  ///Temporary: Mesh computed for implicit surfaces, point clouds, silhouettes
  std::shared_ptr<Meshing::TriMesh> tempMesh,tempMesh2;
  ///Temporary: The display lists and texture lists for vertices, edges, faces, and silhouette
  GLDisplayList vertexDisplayList,edgeDisplayList,faceDisplayList,silhouetteDisplayList;
  GLTextureObject textureObject;
};

} //namespace GLDraw

#endif
