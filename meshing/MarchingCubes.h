#ifndef MESHING_MARCHING_CUBES_H
#define MESHING_MARCHING_CUBES_H

#include "TriMesh.h"
#include <structs/array3d.h>

/** @file meshing/MarchingCubes.h
 * @ingroup Meshing
 * @brief The marching cubes algorithm for extracting an isosurface of a 3D
 * function.
 */

namespace Math {
  struct ScalarFieldFunction;
} //namespace Math

namespace Meshing {

  /** @addtogroup Meshing */
  /*@{*/

/// Takes a 3D function as input, meshes the isosurface at f(x)=isoval
void MarchingCubes(ScalarFieldFunction& f,Real isoval,const AABB3D& bb,const int dims[3],TriMesh& m);

/// Takes a 3D function as input, meshes the isosurface at f(x)=isoval
void MarchingCubes(Real (*f)(Real,Real,Real),Real isoval,const AABB3D& bb,const int dims[3],TriMesh& m);

/// Takes a 3D grid as input, meshes the isosurface at f(x)=isoval
void MarchingCubes(const Array3D<Real>& input,Real isoval,const AABB3D& bb,TriMesh& m);

/// Takes values of a function f at a cube's vertices as input,
/// meshes the isosurface at f(x)=isoval
/// cube vertex indices are given by the index's last 3 bits in xyz order
void CubeToMesh(const Real vals[8],Real isoval,const AABB3D& bb,TriMesh& m);

  /*@}*/

} //namespace Meshing

#endif
