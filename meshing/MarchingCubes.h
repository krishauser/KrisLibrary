#ifndef MESHING_MARCHING_CUBES_H
#define MESHING_MARCHING_CUBES_H

#include "TriMesh.h"
#include <KrisLibrary/structs/array3d.h>

/** @file meshing/MarchingCubes.h
 * @ingroup Meshing
 * @brief The marching cubes algorithm for extracting an isosurface of a 3D
 * function.
 */

namespace Math {
  class ScalarFieldFunction;
} //namespace Math

namespace Meshing {

  /** @addtogroup Meshing */
  /*@{*/

/// Takes a 3D function as input, meshes the isosurface at f(x)=isoval
void MarchingCubes(ScalarFieldFunction& f,Real isoval,const AABB3D& bb,const int dims[3],TriMesh& m);

/// Takes a 3D function as input, meshes the isosurface at f(x)=isoval
void MarchingCubes(Real (*f)(Real,Real,Real),Real isoval,const AABB3D& bb,const int dims[3],TriMesh& m);

/// Takes a 3D grid as input, meshes the isosurface at f(x)=isoval.
/// Assumes the input values are defined at the vertices of a grid with
/// m-1 x n-1 x p-1 cells
///
/// Defined for T = char, int, float, and double
template <class T>
void MarchingCubes(const Array3D<T>& input,T isoval,const AABB3D& bb,TriMesh& m);

/// Takes values of a function f at a cube's vertices as input,
/// meshes the isosurface at f(x)=isoval
/// cube vertex indices are given by the index's last 3 bits in xyz order,
/// i.e., 0 = 000b = (0,0,0), 1 = 001b = (0,0,+x), 3 = 011b = (0,+y,+x), etc.
void CubeToMesh(const Real vals[8],Real isoval,const AABB3D& bb,TriMesh& m);

  /*@}*/

} //namespace Meshing

#endif
