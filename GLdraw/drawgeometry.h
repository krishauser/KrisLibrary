#ifndef GL_DRAWGEOMETRY_H
#define GL_DRAWGEOMETRY_H


#include <KrisLibrary/math3d/geometry2d.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include "GL.h"

namespace GLDraw {

  using namespace Math3D;

  void draw(const GeometricPrimitive2D& geom);
  void draw(const GeometricPrimitive3D& geom);
  void draw(const ConvexPolygon2D& geom);

} //namespace GLDraw

#endif
