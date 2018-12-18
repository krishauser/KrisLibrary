#ifndef MULTI_VOLUME_GRID_H
#define MULTI_VOLUME_GRID_H

#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/math/vector.h>
#include <iosfwd>
#include <string>

namespace Geometry {

  using namespace Math3D;

/** @ingroup Meshing
 * @brief A 3D array over an axis-aligned 3D volume, containing one or more channels.
 *
 * Basically, this contains a set of VolumeGrids all of which are similar (equal shape and size).

 * See the VolumeGrid class for more information about the basic subroutines.
 */
class MultiVolumeGrid
{
 public:
  MultiVolumeGrid();
  inline bool IsEmpty() const { return channels.empty() || channels[0].IsEmpty(); }
  void AddChannel(const std::string& name);
  void SetChannelName(int channel,const std::string& name);
  int GetChannel(const std::string& name) const;

  void Resize(int m,int n,int p);
  void ResizeByResolution(const Vector3& res);
  void MakeSimilar(const MultiVolumeGrid& grid);
  bool IsSimilar(const MultiVolumeGrid& grid) const;
  template <class T>
  inline void MakeSimilar(const Meshing::VolumeGridTemplate<T>& grid) { for(size_t i=0;i<channels.size();i++) channels[i].MakeSimilar(grid); }
  template <class T>
  inline bool IsSimilar(const Meshing::VolumeGridTemplate<T> & grid) const { return !channels.empty() && channels[0].IsSimilar(grid); }
  inline void GetCell(int i,int j,int k,AABB3D& cell) const { return channels[0].GetCell(i,j,k,cell); }
  inline void GetCellCenter(int i,int j,int k,Vector3& center) const { return channels[0].GetCellCenter(i,j,k,center); }
  inline Vector3 GetCellSize() const { return channels[0].GetCellSize(); }
  inline void GetIndex(const Vector3& pt,int& i,int& j,int& k) const { return channels[0].GetIndex(pt,i,j,k); }
  inline void GetIndexAndParams(const Vector3& pt,IntTriple& index,Vector3& params) const { return channels[0].GetIndexAndParams(pt,index,params); }
  inline void GetIndexRange(const AABB3D& range,IntTriple& imin,IntTriple& imax) const { return channels[0].GetIndexRange(range,imin,imax); }
  inline void GetCell(const IntTriple& index,AABB3D& cell) const { GetCell(index.a,index.b,index.c,cell); }
  inline void GetCenter(const IntTriple& index,Vector3& center) const { GetCellCenter(index.a,index.b,index.c,center); }
  inline void GetIndex(const Vector3& pt,IntTriple& index) const { GetIndex(pt,index.a,index.b,index.c); }

  ///Gets all values at the given cell for all channels
  void GetValue(const IntTriple& index,Vector& values) const;
  ///Gets all values at the given point for all channels
  void GetValue(const Vector3& pt,Vector& values) const;
  ///Gets all values at the given cell for all channels
  void SetValue(const IntTriple& index,const Vector& values);
  ///Gets all values at the given point for all channels
  void SetValue(const Vector3& pt,const Vector& values);
  ///Gets all values at the given point
  void TrilinearInterpolate(const Vector3& pt,Vector& values) const;
  ///Resamples the given volume grid onto the current grid, taking trilinear interpolation at cell centers
  void ResampleTrilinear(const MultiVolumeGrid& grid);
  ///Resamples the given volume grid onto the current grid, taking averages over grid cells
  void ResampleAverage(const MultiVolumeGrid& grid);

  std::vector<Meshing::VolumeGridTemplate<float> > channels;
  std::vector<std::string> channelNames;
};




} //namespace Geometry

#endif

