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
 * like color, normal, etc.  These are named in propertyNames and given
 * as a vector for each point.
 *
 * The point cloud itself may also have associated settings, as given by
 * the settings map.
 */
class PointCloud3D
{
 public:
  void Clear();
  bool LoadPCL(const char* fn);
  bool SavePCL(const char* fn) const;
  bool LoadPCL(istream& in);
  bool SavePCL(ostream& out) const;
  void Transform(const Matrix4& mat);
  int PropertyIndex(const string& name) const;
  bool GetProperty(const string& name,vector<Real>& items) const;
  void SetProperty(const string& name,const vector<Real>& items);
  void RemoveProperty(const string& name);
  void GetSubCloud(const Vector3& bmin,const Vector3& bmax,PointCloud3D& subcloud);
  void GetSubCloud(const string& property,Real value,PointCloud3D& subcloud);
  void GetSubCloud(const string& property,Real minValue,Real maxValue,PointCloud3D& subcloud);

  vector<Vector3> points;
  vector<string> propertyNames;
  vector<Vector> properties;
  map<string,string> settings;
};

} //namespace Meshing

#endif 

