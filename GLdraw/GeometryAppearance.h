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

/** @brief A class for coloring, texturing, and drawing geometries in OpenGL.
 *
 * GL display lists are created on first DrawGL call.  This makes subsequent
 * DrawGL calls very fast.  If you change the overall face color or vertex
 * color, the calls will still be fast.  However, if the geometry changes
 * completely, or you wish to use per-vertex/per-face colors, Refresh() must
 * be called to rebuild the display lists.
 *
 * Note: because display lists are cached, copying appearances can be a
 * bit weird.  If you use the assignment operator a = b where a is cached
 * and b is not, then a's display lists will be clobbered, leading to 
 * severe performance drops!  If this isn't what you intended, then
 * you should use the CopyMaterial method. 
 * 
 * Keep that in mind if you are, e.g., assigning vectors of
 * GeometryAppearances.
 * 
 * TriangleMeshes are drawn as faces, edges, and/or vertices depending on
 * which elements are enabled.
 * 
 * GeometricPrimitives are converted to triangle meshes and drawn as meshes.
 * 
 * PointClouds, if unstructured, are drawn as points.  If structured, they
 * are drawn as meshes.
 * 
 * ImplicitSurface geometries are extracted using marching cubes at the
 * 0 level set.
 * 
 * Occupancy grid geometries are drawn as ImplicitSurfaces for now. 
 * TODO: Draw as density maps / voxel grids.
 *
 */
class GeometryAppearance
{
 public:
  enum Element { ALL, VERTICES, EDGES, FACES, SILHOUETTE, ALL_TRANSPARENT, ALL_OPAQUE };
  
  GeometryAppearance();
  ///This copies over the "material" information but doesn't change the display lists (if possible)
  void CopyMaterial(const GeometryAppearance& rhs);
  ///This copies over the "material" information except for vertex and face-specific info (colors and texcoords).
  ///It also doesn't change the display lists.
  void CopyMaterialFlat(const GeometryAppearance& rhs);
  ///This copies over the display lists and other cached information, if rhs has it.
  ///If if_cache_exists=true, then any existing display lists / cached information is
  ///overridden.  If if_cache_exists=false (default), then any existing cached
  ///information is preserved
  void CopyCache(const GeometryAppearance& rhs,bool if_cache_exists=false);
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
  ///or per-face colors.  Refresh() does not need to be called.  Setting
  ///fraction=0 resets to the original color
  void SetTintColor(const GLColor& color,float fraction);
  //void ModulateColor(const GLColor& color,float fraction) { SetTintColor(color,fraction); }

  ///Geometry pointer
  const Geometry::AnyGeometry3D* geom;
  const Geometry::AnyCollisionGeometry3D* collisionGeom;
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
  ///Optional: emissive color; only used if lightFaces=true.  (default (0,0,0))
  GLColor emissiveColor;
  ///Optional: specular coefficient and color; only used if lightFaces=true.
  ///(default 0 indicating no shininess, and 10% white).
  ///Recommended values for metal or smooth plastic are 20 and (0.1,0.1,0.1).
  float shininess;
  GLColor specularColor;
  ///Optional: set to non-null if you want to texture the object
  std::shared_ptr<Image> tex1D,tex2D;
  bool texWrap;            ///Optional: if true, texture will wrap
  bool texFilterNearest;   ///Optional: if true, texture will not be interpolated (i.e., will look blocky)

  ///Optional: per-element texture mapping coordinates (up to 2D)
  std::vector<Math3D::Vector2> texcoords;
  ///Optional: linear texture generation coefficients.
  ///
  ///E.g., for 2D, set texgen.size()==2, and the texcoords (S,T) will be
  ///S = c[0]^T p (p in homogeneous coordinates)
  ///T = c[1]^T p
  ///
  ///For projection mapping, the R and Q elements should be set.  The Q
  ///coordinate determines the homogeneous coordinate, and should be 
  ///set to [vx vy vz -ofs] with (vx,vy,vz) the viewing direction and
  ///ofs the dot product of the viewing direction and origin.
  std::vector<Math3D::Vector4> texgen;
  ///Optional: if texture generation is used, you can specify world coordinates
  ///rather than object coordinates (default) by setting this to true.
  ///World coordinates are used for projection mapping.
  ///
  ///If an AnyCollisionGeometry is drawn, the object's inverse transform
  ///is performed before this one.  Otherwise, this transform is interpreted
  ///as being relative to the object coordinates. 
  std::shared_ptr<Math3D::RigidTransform> texgenEyeTransform;
  ///Optional: draw meshes with a crease if the angle between triangles is
  ///greater than this threshold, in radians (default 0, indicating disabled)
  float creaseAngle;
  ///Optional: draw meshes with a silhouette outline (default 0, indicating disabled)
  float silhouetteRadius;
  GLColor silhouetteColor;
  ///Tints faces / vertices etc with a modulation color
  GLColor tintColor;
  float tintStrength;
  ///Optional: for OccupancyGrids, the piecewise-linear map from density values to
  ///colors.  If not set, goes from clear to faceColor.
  std::vector<float> densityGradientKeypoints;
  std::vector<GLColor> densityGradientColors;
  
  ///Temporary: Mesh computed for implicit surfaces, point clouds, silhouettes
  std::shared_ptr<Meshing::TriMesh> tempMesh,tempMesh2;
  ///Temporary: The display lists and texture lists for vertices, edges, faces, and silhouette
  GLDisplayList vertexDisplayList,edgeDisplayList,faceDisplayList,silhouetteDisplayList;
  GLTextureObject textureObject;
};

} //namespace GLDraw

#endif
