#ifndef MESHING_POINT_CLOUD_H
#define MESHING_POINT_CLOUD_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/utils/PropertyMap.h>
#include <vector>
#include <iosfwd>
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
 * the settings map. Standard properties include:
 * - width, height: for structured point clouds, the width and height of the point cloud.
 *   If present, the points list will be interpreted in scan-line order
 *   (left to right, top to bottom)
 * - viewpoint: if present, the "x y z qw qx qy qz" camera frame.  [x,y,z] is the focal
 *   point (origin) and (qw,qx,qy,qz) is the quaternion representing its
 *   rotation from the standard camera reference frame (+z forward, +x right, +y down)
 * - id: if present, the id of the scan
 *
 * Standard properties (adopted from PCL) include:
 * - normal_x, normal_y, normal_z: the outward normal estimated at a point
 * - rgb: integer representation of the 8-bit RGB value of a color. 0x00ff0000
 *   is red, 0x0000ff00 is green, 0x000000ff is blue.
 * - rgba: integer representation of the 8-bit ARGB value of a color with
 *   alpha channel.  0xff000000 is the opacity.
 * - opacity: a floating point value indicating the opacity of a point, in
 *   the range [0,1].
 * - c: A byte-based opacity format used in PCL where opacity is in the range
 *   [0,255].
 * - r,g,b,a: color channels in the range [0,1]
 * - u,v: a (u,v) coordinate in range [0,1]^2 mapping into some other image (usually RGB color)
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
  bool IsStructured() const;
  int GetStructuredWidth() const;
  int GetStructuredHeight() const;
  void SetStructured(int w,int h);
  Vector3 GetOrigin() const;
  void SetOrigin(const Vector3& origin);
  RigidTransform GetViewpoint() const;
  void SetViewpoint(const RigidTransform& T);
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
  ///Converts rgb or rgba packed color channels to r,g,b / r,g,b,a channels. Removes old channels.
  bool UnpackColorChannels(bool alpha=false);
  ///Converts unpacked color channels into packed channel. fmt is the desired property, either "rgb" or "rgba". Removes old channels.
  bool PackColorChannels(const char* fmt="rgba");
  bool GetColors(vector<Real>& r,vector<Real>& g,vector<Real>& b,vector<Real>& a) const;
  bool GetColors(vector<Vector4>& rgba) const;
  void SetColors(const vector<Real>& r,const vector<Real>& g,const vector<Real>& b,bool includeAlpha = false);
  void SetColors(const vector<Real>& r,const vector<Real>& g,const vector<Real>& b,const vector<Real>& a,bool includeAlpha = true);
  void SetColors(const vector<Vector4>& rgba,bool includeAlpha=true);
  bool HasUV() const;
  bool GetUV(vector<Vector2>& uvs) const;
  void SetUV(const vector<Vector2>& uvs);

  vector<Vector3> points;
  vector<string> propertyNames;
  vector<Vector> properties;
  PropertyMap settings;
};

} //namespace Meshing

#endif 

