#ifndef GEOMETRY_CONVERSIONS_H
#define GEOMETRY_CONVERSIONS_H

#include "AnyGeometry.h"

namespace GLDraw {
	class GeometryAppearance;
};

/** @file geometry/Conversions.h
 * @ingroup Geometry
 * @brief 3D geometry conversion routines.
 */


namespace Geometry {

/** @ingroup Geometry
 * @brief Places points on the vertices of a mesh. 
 * If any triangle has vertices farther away than maxDispersion, the triangle is subdivided until
 * all triangles are covered by balls of radius maxDispersion.
 */
void MeshToPointCloud(const Meshing::TriMesh& mesh,Meshing::PointCloud3D& pc,Real maxDispersion=Inf,bool wantNormals=false);

/** @ingroup Geometry
 * @brief If a point cloud is structured, this creates a uniform mesh out of it.  If depthDiscontinuity
 * is provided, the mesh is split at the points where the relative depth of adjacent points is greater than
 * depthDiscontinuity * average depth.  (A decent value is 0.02 for typical depth sensors.)
 */
void PointCloudToMesh(const Meshing::PointCloud3D& pc,Meshing::TriMesh& mesh,Real depthDiscontinuity=Inf);

/** @ingroup Geometry
 * @brief Same as normal PointCloudToMesh, but colors and UV coordinates are extracted
 */
void PointCloudToMesh(const Meshing::PointCloud3D& pc,Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& appearance,Real depthDiscontinuity=Inf);

/** @ingroup Geometry
 * @brief Creates a mesh for a geometric primitive.  If the primitive has curved surfaces, they
 * are split into numDivs strips.
 */
void PrimitiveToMesh(const GeometricPrimitive3D& primitive,Meshing::TriMesh& mesh,int numDivs);

/** @ingroup Geometry
 * @brief Creates an implicit surface for a geometric primitive.  The grid has
 * resolution no more than resolution on each axis. 
 */
void PrimitiveToImplicitSurface(const GeometricPrimitive3D& primitive,Meshing::VolumeGrid& grid,Real resolution,Real expansion=0);

/** @ingroup Geometry
 * @brief Creates an implicit surface for a mesh using a Fast Marching Method.
 *
 * Note: the mesh's current transform is NOT taken into account (i.e., the resulting grid
 * is in local coordinates)
 */
void MeshToImplicitSurface_FMM(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution);

/** @ingroup Geometry
 * @brief Creates an implicit surface for a mesh using a space-carving technique.
 * The grid has resolution no less than resolution on each axis.  numViews views
 * (6 orthographic, numViews-6 random) are taken of the mesh to create the space
 * carving.
 *
 * Note: the mesh's current transform is NOT taken into account (i.e., the resulting grid
 * is in local coordinates)
 */
void MeshToImplicitSurface_SpaceCarving(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution,int numViews=20);

/** @ingroup Geometry
 * @brief Creates a mesh from an implicit surface via Marching Cubes.  This assumes
 * the values of the grid are defined at the cell centers, unlike the methods in
 * MarchingCubes.h which assume the array defines values at the cell corners... this
 * performs the necessary correction.
 */
void ImplicitSurfaceToMesh(const Meshing::VolumeGrid& grid,Meshing::TriMesh& mesh);


} //namespace Geometry

#endif