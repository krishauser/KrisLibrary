#ifndef TSDF_RECONSTRUCTION_H
#define TSDF_RECONSTRUCTION_H

#include "MultiVolumeGrid.h"
#include "SparseVolumeGrid.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/TriMesh.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/utils/threadutils.h>
#include <map>

namespace Geometry {

class ICPParameters
{
public:
  ICPParameters();
  
  //input parameters
  int maxIters;
  int subsample;
  bool pointToPlane;
  Real colorWeight;
  Real percentileOutliers;
  Real rmseThreshold,rmseChangeThreshold;

  //output parameters
  int numIters;
  RigidTransform Tcamera;
  Vector standardError;
  Real rmseDistance,rmseColor;
  int numInliers;
  std::vector<std::pair<Vector3,Vector3> > correspondences;
};

/** @ingroup Geometry
 * @brief Performs a kinect-fusion-like dense TSDF reconstruction.
 *
 * All camera transforms have the convention that z > 0 is forward.
 */
class DenseTSDFReconstruction
{
public:
  DenseTSDFReconstruction(const AABB3D& volume,const IntTriple& res,Real truncationDistance=0.1);
  ///Changes the truncation distance (this clears the TSDF)
  void SetTruncationDistance(Real truncationDistance);
  /** @brief Performs registration between the point cloud and the TSDF
   * 
   * - pc: the point cloud, in camera coordinates
   * - Tcamera_est: the estimated camera transform
   * - params: the parameters for ICP and the solution (in/out)
   */
  void Register(const Meshing::PointCloud3D& pc,const RigidTransform& Tcamera_est,ICPParameters& params);
  /// Fuses the point cloud (in camera coordinates) into the TSDF
  void Fuse(const RigidTransform& Tcamera,const Meshing::PointCloud3D& pc,Real weight=1.0);
  ///Builds the mesh using the marching cubes algorithm
  void ExtractMesh(Meshing::TriMesh& mesh);
  ///Builds a colored mesh using the marching cubes algorithm
  void ExtractMesh(Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app);
  ///Extracts the mesh at a region of interest (bounding box) 
  void ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh);
  ///Extracts a colored mesh at a region of interest (bounding box) 
  void ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app);
  ///Gets the 3 color channels of a point, if colored.  Each of r,g,b is in the range [0,1]
  void GetColor(const Vector3& point,float* color) const;
  ///Clears the TSDF at a given point
  void ClearPoint(const Vector3& p,Real dist=0,Real weight=1.0);
  ///Clears the TSDF in a given box
  void ClearBox(const Vector3& bmin,const Vector3& bmax,Real weight=1.0);
  ///Estimates memory usage, in bytes
  size_t MemoryUsage() const;
  
  ///The max truncation distance (default 0.1)
  Real truncationDistance;  
  ///The standard deviation of a depth value is assumed to be depthStddev0 + d*depthStddev1 (defaults 0.005, 0.01)
  Real depthStddev0,depthStddev1;
  ///The rate at which voxel data is forgotten due to age e^-(forgettingRate*age) (default 0)
  Real forgettingRate; 
  ///If true, the TSDF will be colored (as long as the first point cloud is colored) (default true)
  bool colored;
  ///These indices are added from a PointCloud's attributes to the auxilary volume grid (default empty)
  std::vector<int> auxiliaryAttributes;

  struct VoxelInfo
  {
    unsigned char rgb[3];
    float occupancy;
    float weight,surfaceWeight;
    int lastID;
  };

  Meshing::VolumeGridTemplate<float> tsdf;
  MultiVolumeGrid auxiliary;
  Array3D<VoxelInfo> info;
  int scanID;
  Meshing::TriMesh currentMesh;
  int currentMeshID;
};


/** @ingroup Geometry
 * @brief Performs a kinect-fusion-like sparse TSDF reconstruction.
 * A hash grid is used to build multiple TSDFs across an infinite domain.
 *
 * All camera transforms have the convention that z > 0 is forward.
 */
class SparseTSDFReconstruction
{
public:
  SparseTSDFReconstruction(const Vector3& cellSize,Real truncationDistance=0.1);
  ~SparseTSDFReconstruction();
  /** @brief Performs registration between the point cloud and the TSDF
   * 
   * - pc: the point cloud, in camera coordinates
   * - Tcamera_est: the estimated camera transform
   * - params: the parameters for ICP and the solution (in/out)
   */
  void Register(const Meshing::PointCloud3D& pc,const RigidTransform& Tcamera_est,ICPParameters& params);
  /// Fuses the point cloud (in camera coordinates) into the TSDF
  void Fuse(const RigidTransform& Tcamera,const Meshing::PointCloud3D& pc,Real weight=1.0);
  ///Builds the mesh using the marching cubes algorithm
  void ExtractMesh(Meshing::TriMesh& mesh);
  ///Builds a colored mesh using the marching cubes algorithm
  void ExtractMesh(Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app);
  ///Extracts the mesh at a region of interest (bounding box) 
  void ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh);
  ///Extracts a colored mesh at a region of interest (bounding box) 
  void ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app);
  ///Gets the 3 color channels of a point, if colored.  Each of r,g,b is in the range [0,1]
  void GetColor(const Vector3& point,float* color) const;
  ///Clears the TSDF at a given point
  void ClearPoint(const Vector3& p,Real dist=0,Real weight=1.0);
  ///Clears the TSDF in a given box
  void ClearBox(const Vector3& bmin,const Vector3& bmax,Real weight=1.0);
  ///Estimates memory usage, in bytes
  size_t MemoryUsage() const;

  void StartThreads();
  void StopThreads();
  
  ///The max truncation distance (default 0.1)
  Real truncationDistance;  
  ///The standard deviation of a depth value is assumed to be depthStddev0 + d*depthStddev1 (defaults 0.005, 0.01)
  Real depthStddev0,depthStddev1;
  ///The rate at which voxel data is forgotten due to age e^-(forgettingRate*age) (default 0)
  Real forgettingRate; 
  ///If true, the TSDF will be colored (as long as the first point cloud is colored) (default true)
  bool colored;
  ///These indices are added from a PointCloud's attributes to the auxilary volume grid (default empty)
  std::vector<int> auxiliaryAttributes;
  ///Number of threads to use (default 1)
  int numThreads;

  SparseVolumeGrid tsdf;
  //Indices into ths sparse tsdf's channels (automatically set up on first scan)
  int depthChannel,weightChannel,ageChannel,rgbChannel,surfaceWeightChannel,auxiliaryChannelStart;
  int scanID;
  std::map<int,int> blockLastTouched;

  //for multithreading
  std::vector<Thread> threads;
  std::vector<void*> threadData;
  Mutex lock;
};

} //namespace Geometry

#endif