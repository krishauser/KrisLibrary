#ifndef MESHING_VOXELIZE_H
#define MESHING_VOXELIZE_H

#include "TriMesh.h"
#include "TriMeshTopology.h"
#include <KrisLibrary/math3d/Segment3D.h>
#include <KrisLibrary/structs/array3d.h>

/** @file meshing/Rasterize.h
 * @ingroup Meshing
 * @brief 3D rasterization routines.
 */

namespace Meshing {

/** @ingroup Meshing
 * @brief Returns a list of cells that the segment overlaps, given
 * an infinite unit grid.
 */
void GetSegmentCells(const Segment3D& s,vector<IntTriple>& cells);

/** @ingroup Meshing
 * @brief Returns a list of cells that the segment overlaps, given
 * a grid of size m,n,p over range bb.
 */
void GetSegmentCells(const Segment3D& s,int m,int n,int p,const AABB3D& bb,vector<IntTriple>& cells);

/** @ingroup Meshing
 * @brief Returns a list of cells that the triangle overlaps, given
 * an infinite unit grid.
 */
void GetTriangleCells(const Triangle3D& tri,vector<IntTriple>& cells);

/** @ingroup Meshing
 * @brief Returns a list of cells that the triangle overlaps, given
 * a grid of size m,n,p over range bb.
 */
void GetTriangleCells(const Triangle3D& tri,int m,int n,int p,const AABB3D& bb,vector<IntTriple>& cells);

/** @ingroup Meshing
 * @brief Sets cells of a boolean 3D grid (occupied,bb) that overlap the
 * surface of m to true.
 *
 * If bb is empty, automatically fits the bounding box to contain the mesh
 * in the non-border cells of the grid.
 */
void SurfaceOccupancyGrid(const TriMesh& m,Array3D<bool>& occupied,AABB3D& bb);

/** @ingroup Meshing
 * @brief Sets cells of a boolean 3D grid (occupied,bb) to true
 * if the cell's center is in the interior of the mesh, and false otherwise.
 * Occupancy is determined by a flood-fill algorithm starting from the seed
 * point.
 */
void VolumeOccupancyGrid_FloodFill(const TriMesh& m,Array3D<bool>& occupied,AABB3D& bb,const IntTriple& seed,bool seedOccupied);

/** @ingroup Meshing
 * @brief Sets cells of a boolean 3D grid (occupied,bb) to true
 * if the cell's center is in the interior of the mesh, and false otherwise. 
 * Occupancy is determined by sweeping a ray through the mesh.
 */
void VolumeOccupancyGrid_CenterShooting(const TriMesh& m,Array3D<bool>& occupied,AABB3D& bb,int shootDirection=0);

/** @ingroup Meshing
 * @brief From one "visible" side of the grid, sweeps the visibility across
 * the volume until a mesh surface is hit.  After that, visible is marked
 * as false.
 *
 * direction can be one of 1,2,3,-1,-2, or -3, corresponding to the sweep
 * direction (+x,+y,+z,-x,-y, or -z, respectively).
 *
 * If singleSided=true, then a surface is "hit" only when approaching the 
 * front side (ccw winding) of the triangle.
 */
void SweepVisibilityGrid(const TriMesh& m,int direction,Array3D<bool>& visible,AABB3D& bb,bool singleSided=true);

/** @ingroup Meshing
 * @brief Fills in a distance field on a 3D grid (occupied,bb) using the fast
 * marching method initialized at the surface m.
 *
 * O(n^3 log n) for a grid of size n x n x n
 *
 * If bb is empty, automatically fits the bounding box to contain the mesh
 * in the non-border cells of the grid.
 */
void FastMarchingMethod(const TriMeshWithTopology& m,Array3D<Real>& distance,Array3D<Vector3>& gradient,AABB3D& bb,vector<IntTriple>& surfaceCells);


/** @ingroup Meshing
 * @brief Same as FastMarchingMethod but constrains start points to the exterior
 * of the mesh (sometimes avoids artifacts from internal structures)
 */
void FastMarchingMethod_Fill(const TriMeshWithTopology& m,Array3D<Real>& distance,Array3D<Vector3>& gradient,AABB3D& bb,vector<IntTriple>& surfaceCells);


/** @ingroup Meshing
 * @brief Estimates the object's density filling the grid using a shooting method.
 */
void DensityEstimate_CenterShooting(const TriMesh& m,Array3D<Real>& density,AABB3D& bb,int shootDirection=0);

/** @ingroup Meshing
 * @brief Estimates the object's density filling the grid using a shooting method.
 */
void DensityEstimate_RandomShooting(const TriMesh& m,Array3D<Real>& density,AABB3D& bb,int numSamples,int shootDirection=0);

/** @ingroup Meshing
 * @brief Fills in a density estimate on a 3D grid (occupied,bb) using a flood fill
 * to determine inside/outside, more accurate calculation for density.
 *
 * The seed point is assumed to be empty.
 */
 void DensityEstimate_FloodFill(const TriMeshWithTopology& m,Array3D<Real>& density,AABB3D& bb,const IntTriple& seed);

/** @ingroup Meshing
 * @brief Fills in a density estimate on a 3D grid (occupied,bb) using the
 * FMM distance measurement.
 */
void DensityEstimate_FMM(const TriMeshWithTopology& m,Array3D<Real>& density,AABB3D& bb);

/** @ingroup Meshing
 * @brief Fills in a density estimate on a 3D grid (occupied,bb) using the
 * density, gradient output from running FMM.
 */
void DensityEstimate_FMM(const Array3D<Real>& distance,const Array3D<Vector3>& gradient,const AABB3D& bb,Array3D<Real>& density);


} //namespace Meshing

#endif
