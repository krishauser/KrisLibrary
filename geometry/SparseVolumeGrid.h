#ifndef SPARSE_VOLUME_GRID_H
#define SPARSE_VOLUME_GRID_H

#include "GridSubdivision.h"
#include "MultiVolumeGrid.h"
#include <KrisLibrary/meshing/TriMesh.h>

namespace Geometry {

  using namespace Math3D;

/** @ingroup Meshing
 * @brief A 3D array over a sparse, axis aligned 3D volume.  Has a similar interface as VolumeGrid,
 * but uses a hash grid data structure to achieve better performance in large grids with infinite
 * domain.
 *
 * A spatial hash is used to store blocks of dense volumes.  Each volume is by default 8x8x8, but this
 * can be configured using blockSize.
 */
class SparseVolumeGrid
{
 public:
  class Block {
  public:
    int id;
    IntTriple index;
    MultiVolumeGrid grid;
  };

  SparseVolumeGrid(Real blockRes);
  SparseVolumeGrid(const Vector3& blockRes);
  ~SparseVolumeGrid();
  inline bool IsEmpty() const { return hash.buckets.empty(); }
  void Clear();
  int AddChannel(const std::string& name);
  int GetChannel(const std::string& name) const;
  size_t GetNumChannels() const { return channelNames.size(); }
  inline void SetBlockSize(int m,int n,int p) { SetBlockSize(IntTriple(m,n,p)); }
  void SetBlockSize(const IntTriple& size);
  inline IntTriple GetBlockSize() const { return blockSize; }
  void SetBlockRes(const Vector3& blockRes);
  inline void SetBlockRes(Real blockRes) { SetBlockRes(Vector3(blockRes)); }
  Vector3 GetBlockRes() const;
  inline void SetDefaultValue(Real _defaultValue) { defaultValue.set(_defaultValue); }
  inline void SetDefaultValue(const Vector& _defaultValue) { defaultValue = _defaultValue; }
  ///Copies the block shape parameters from grid
  void MakeSimilar(const SparseVolumeGrid& grid);
  ///Returns true if this shares the same block shape parameters as grid
  bool IsSimilar(const SparseVolumeGrid& grid) const;
  void GetCell(int i,int j,int k,AABB3D& cell) const;
  void GetCellCenter(int i,int j,int k,Vector3& center) const;
  inline Vector3 GetCellSize() const { return cellRes; }
  void GetIndex(const Vector3& pt,int& i,int& j,int& k) const;
  void GetIndexAndParams(const Vector3& pt,IntTriple& index,Vector3& params) const;
  void GetIndexRange(const AABB3D& range,IntTriple& imin,IntTriple& imax) const;
  inline void GetCell(const IntTriple& index,AABB3D& cell) const { GetCell(index.a,index.b,index.c,cell); }
  inline void GetCenter(const IntTriple& index,Vector3& center) const { GetCellCenter(index.a,index.b,index.c,center); }
  inline void GetIndex(const Vector3& pt,IntTriple& index) const { GetIndex(pt,index.a,index.b,index.c); }
  ///Sets the value of the grid at a given cell, creating a new block if necessary
  void SetValue(int i,int j,int k,Real value,int channel=0);
  ///Sets the value of the grid at a given point, creating a new block if necessary
  void SetValue(const Vector3& pt,Real value,int channel=0);
  ///Sets all the values of the grid at a given cell, creating a new block if necessary
  void SetValue(int i,int j,int k,const Vector& values);
  ///Sets all the values of the grid at a given point, creating a new block if necessary
  void SetValue(const Vector3& pt,const Vector& values);
  ///Gets the value of the grid at a given cell
  Real GetValue(int i,int j,int k,int channel=0) const;
  ///Gets the value of the grid at a given point (no interpolation is performed)
  Real GetValue(const Vector3& pt,int channel=0) const;
  ///Gets all the values of the grid at a given cell
  void GetValue(int i,int j,int k,Vector& values) const;
  ///Gets all the values of the grid at a given point
  void GetValue(const Vector3& pt,Vector& values) const;
  ///Returns the index of the block for cell (i,j,k)
  void GetBlock(int i,int j,int k,IntTriple& blockIndex) const;
  ///Returns the index of the block at pt (unlike GetIndex, which gives a cell in the hypothetical infinite grid)
  void GetBlock(const Vector3& pt,IntTriple& blockIndex) const;
  ///Returns the min/max indices of blocks (unlike GetIndexRange, which gives a range of cell sin the hypothetical infinite grid)
  void GetBlockRange(const AABB3D& range,IntTriple& blockMin,IntTriple& blockMax) const;
  ///Retrieves the block given an index, or NULL if the block doesn't exist
  Block* BlockPtr(const IntTriple& blockIndex) const;
  ///If block exists at the given index, returns BlockPtr().  Otherwise, calls MakeBlock() on it and returns the pointer.
  ///Faster than "MakeBlock(b); return BlockPtr(b);".
  Block* GetMakeBlock(const IntTriple& blockIndex);
  ///Creates an empty block (filled with the default value) at the given hash index.  Returns true if a new block was made, false
  ///if the block already existed
  bool MakeBlock(const IntTriple& blockIndex);
  ///Erases a block block at the given hash index.  Returns true if a block was deleted, false if no block existed there.
  bool EraseBlock(const IntTriple& blockIndex);
  ///For a similar grid, makes default blocks with the same block pattern as grid
  void AddBlocks(const SparseVolumeGrid& grid);
  ///Subdivides a given block
  void SubdivideBlock(const IntTriple& block);

  ///Samples the grid along a dense volume grid (using basic extraction at cell centers).  The grid bbox and values must be
  ///sized appropriately.
  void GetSamples(MultiVolumeGrid& range) const;
  void GetSamples(Meshing::VolumeGrid& range,int channel=0) const;
  ///Samples the grid along a dense volume grid (using trilinear interpolation at cell centers)
  void GetSamples_Trilinear(MultiVolumeGrid& range) const;
  void GetSamples_Trilinear(Meshing::VolumeGrid& range,int channel=0) const;
  ///Samples the grid along a dense volume grid (using averaging over cells)
  void GetSamples_Average(MultiVolumeGrid& range) const;
  void GetSamples_Average(Meshing::VolumeGrid& range,int channel=0) const;
  ///Sets entries of the grid to be equal to those of a dense volume grid
  void SetSamples(const MultiVolumeGrid& range);
  void SetSamples(const Meshing::VolumeGrid& range,int channel=0);

  ///Computes the trilinear interpolation of the field at pt, assuming values are sampled exactly at cell centers
  Real TrilinearInterpolate(const Vector3& pt,int channel=0) const;
  void TrilinearInterpolate(const Vector3& pt,Vector& values) const;
  ///Average value of the range.  Each cell's value is weighted by the volume overlap with range
  Real Average(const AABB3D& range,int channel=0) const;
  void Average(const AABB3D& range,Vector& values) const;
  ///Returns the trilinear interpolated gradient.  If any element of pt is on a cell boundary the gradient is estimated
  ///using centered differencing
  void Gradient(const Vector3& pt,Vector3& grad,int channel=0) const;
  void Add(const SparseVolumeGrid& grid);
  void Subtract(const SparseVolumeGrid& grid);
  void Multiply(const SparseVolumeGrid& grid);
  void Max(const SparseVolumeGrid& grid);
  void Min(const SparseVolumeGrid& grid);
  void Add(Real val,int channel=0);
  void Multiply(Real val,int channel=0);
  void Max(Real val,int channel=0);
  void Min(Real val,int channel=0);

  ///Generates a mesh for the level set at the given level set (usually 0), using the marching cubes algorithm
  void ExtractMesh(float isosurface,Meshing::TriMesh& mesh);

  GridHash3D hash;
  int blockIDCounter;
  IntTriple blockSize;
  std::vector<std::string> channelNames;
  Vector3 cellRes;
  Vector defaultValue;
};


} //namespace Geometry

#endif

