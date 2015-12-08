#ifndef MESHING_POINT_CLOUD_H
#define MESHING_POINT_CLOUD_H

#include <math3d/primitives.h>
#include <math/vector.h>
#include <vector>
#include <map>
#include <iostream>
#include <string>

namespace Meshing {

using namespace Math3D;
using namespace std;

/** @brief A 3D point cloud class.
 *
 * Points may have optional associated floating point properties
 * like ID, color, normal, etc.  These are named in propertyNames and
 * each property is an entry of a property vector attached to each point.
 *
 * The point cloud itself may also have associated settings, as given by
 * the settings map.
 *
 * Standard properties (adopted from PCL) include:
 * - rgb: integer representation of the 8-bit RGB value of a color. 0x00ff0000
 *   is red, 0x0000ff00 is green, 0x000000ff is blue.
 * - rgba: integer representation of the 8-bit ARGB value of a color with
 *   alpha channel.  0xff000000 is the opacity.
 * - opacity: a floating point value indicating the opacity of a point, in
 *   the range [0,1].
 * - c: A byte-based opacity format used in PCL where opacity is in the range
 *   [0,255].
 * When drawn via the GeometryAppearance class, these properties will be
 * properly interpreted to color the point cloud display.
 */
class PointCloud3D
{
 public:
  void Clear();
  bool LoadPCL(const char* fn);
  bool SavePCL(const char* fn) const;
  bool LoadPCL(istream& in);
  bool SavePCL(ostream& out) const;
  void GetAABB(Vector3& bmin,Vector3& bmax) const;
  void Transform(const Matrix4& mat);
  int PropertyIndex(const string& name) const;
  bool HasProperty(const string& name) const { return PropertyIndex(name) >= 0; }
  bool GetProperty(const string& name,vector<Real>& items) const;
  void SetProperty(const string& name,const vector<Real>& items);
  void RemoveProperty(const string& name);
  void GetSubCloud(const Vector3& bmin,const Vector3& bmax,PointCloud3D& subcloud);
  void GetSubCloud(const string& property,Real value,PointCloud3D& subcloud);
  void GetSubCloud(const string& property,Real minValue,Real maxValue,PointCloud3D& subcloud);
  bool HasXYZAsProperties() const;
  void SetXYZAsProperties(bool);
  bool HasNormals() const;
  bool GetNormals(vector<Vector3>& normals) const;
  void SetNormals(const vector<Vector3>& normals);
  bool HasColor() const;
  bool HasOpacity() const;
  bool HasRGB() const;
  bool HasRGBA() const;
  bool UnpackColorChannels(bool alpha=false);
  bool PackColorChannels(const string& property,const char* fmt=NULL);
  bool GetColors(vector<Real>& r,vector<Real>& g,vector<Real>& b,vector<Real>& a) const;
  void SetColors(const vector<Real>& r,const vector<Real>& g,const vector<Real>& b,bool includeAlpha = false);
  void SetColors(const vector<Real>& r,const vector<Real>& g,const vector<Real>& b,const vector<Real>& a,bool includeAlpha = true);

  vector<Vector3> points;
  vector<string> propertyNames;
  vector<Vector> properties;
  map<string,string> settings;
};

} //namespace Meshing

#endif 

