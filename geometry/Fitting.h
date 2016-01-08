#ifndef GEOMETRY_FITTING_H
#define GEOMETRY_FITTING_H

#include <KrisLibrary/math3d/primitives.h>
#include <vector>

namespace Math3D
{
  struct Line2D;
  struct Line3D;
  struct Plane3D;
  struct Circle2D;
} //namespace Math3D


namespace Geometry {

using namespace Math3D;

Vector2 GetMean(const std::vector<Vector2>& pts);
Vector3 GetMean(const std::vector<Vector3>& pts);
void GetCovariance(const std::vector<Vector2>& pts,Matrix2& C);
void GetCovariance(const std::vector<Vector3>& pts,Matrix3& C);

bool FitGaussian(const std::vector<Vector2>& pts,Vector2& center,Matrix2& R,Vector2& axes);
bool FitGaussian(const std::vector<Vector3>& pts,Vector3& center,Matrix3& R,Vector3& axes);

bool FitLine(const std::vector<Vector2>& pts,Line2D& l);
bool FitLine(const std::vector<Vector3>& pts,Line3D& l);
bool FitPlane(const std::vector<Vector3>& pts,Plane3D& p);

bool FitCircle(const std::vector<Vector2>& pts,Circle2D& c);
//bool FitEllipse(const std::vector<Vector2>& pts,Ellipse2D& c);

} //namespace Geometry

#endif
