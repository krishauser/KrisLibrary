#ifndef MESHING_PRIMITIVES_H
#define MESHING_PRIMITIVES_H

#include "TriMesh.h"

namespace Math3D {
  struct AABB3D; 
  struct Box3D;
  struct Sphere3D;
  struct Ellipsoid3D;
  struct Cylinder3D;
  struct Polygon3D;
  class GeometricPrimitive3D;
} //namespace Math3D

/** @file meshing/MeshPrimitives.h
 * @ingroup Meshing
 * @brief Constructors for basic mesh primitives.
 */

namespace Meshing {

  /** @addtogroup Meshing */
  /*@{*/

///makes a unit square on z=0 with m,n divisions in the x,y directions
void MakeTriPlane(int m,int n,TriMesh& mesh);
///makes a square of size [x,y] with m,n divisions in the x,y directions
void MakeTriPlane(int m,int n,Real x,Real y,TriMesh& mesh);

///makes a unit cube with m,n,p divisions in the x,y,z directions
void MakeTriCube(int m,int n,int p,TriMesh& mesh);
///makes a [x,y,z] sized box with m,n,p divisions in the x,y,z directions
void MakeTriBox(int m,int n,int p,Real x,Real y,Real z,TriMesh& mesh);

///makes a unit cube centered at 0 with m,n,p divisions in the x,y,z directions
void MakeTriCenteredCube(int m,int n,int p,TriMesh& mesh);
///makes a [x,y,z] sized cube centered at 0 with m,n,p divisions in the x,y,z directions
void MakeTriCenteredBox(int m,int n,int p,Real x,Real y,Real z,TriMesh& mesh);

///makes a unit sphere with the given stacks and slices (axis in z direction)
void MakeTriSphere(int numStacks,int numSlices,TriMesh& mesh);
///makes a radius r sphere with the given stacks and slices (axis in z direction)
void MakeTriSphere(int numStacks,int numSlices,Real r,TriMesh& mesh);

///makes a unit height cone with unit base radius (base at origin, tip pointing in z direction)
void MakeTriCone(int numSlices,TriMesh& mesh);
///makes a cone with height h, base radius rbase
void MakeTriCone(int numSlices,Real h,Real rbase,TriMesh& mesh);

///makes a unit height cylinder with unit base radius (centered at origin, extending in z direction)
void MakeTriCylinder(int numSlices,TriMesh& mesh);
///makes a cylinder with height h, base radius rbase
void MakeTriCylinder(int numSlices,Real h,Real rbase,TriMesh& mesh);

///makes a triangle mesh from a sphere
void MakeTriMesh(const Sphere3D& geom,int numStacks,int numSlices,TriMesh& mesh);
///makes a triangle mesh from a triangle
void MakeTriMesh(const Triangle3D& geom,TriMesh& mesh);
///makes a triangle mesh from an AABB
void MakeTriMesh(const AABB3D& geom,TriMesh& mesh);
///makes a triangle mesh from a box
void MakeTriMesh(const Box3D& geom,TriMesh& mesh);
///makes a triangle mesh from an ellipsoid
void MakeTriMesh(const Ellipsoid3D& geom,int numStacks,int numSlices,TriMesh& mesh);
///makes a triangle mesh from a cylinder
void MakeTriMesh(const Cylinder3D& geom,int numSlices,TriMesh& mesh);
///makes a triangle mesh from a convex polygon (one sided)
void MakeTriMesh(const Polygon3D& geom,TriMesh& mesh);

///makes a triangle mesh from a generic geometric primitive
void MakeTriMesh(const GeometricPrimitive3D& geom,TriMesh& mesh,int numDivs = 32);

  /*@}*/
} //namespace Meshing

#endif
