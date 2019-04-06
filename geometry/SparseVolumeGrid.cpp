#include "SparseVolumeGrid.h"
#include <KrisLibrary/meshing/MarchingCubes.h>
#include <algorithm>
#include <iostream>
#include <limits>

using namespace std;
using namespace Geometry;
using namespace Meshing;

inline int modulo(int x,int N){
    return (x % N + N) %N;
}

inline int floordiv(int x,int N) {
  int res = x/N;
  if(x >= 0) return res;
  else {
    if(res*N == x) return res;
    return res-1;
  }
}

SparseVolumeGrid::SparseVolumeGrid(Real blockRes)
:hash(blockRes),blockIDCounter(0),blockSize(8,8,8),defaultValue(1,0.0)
{
  channelNames.push_back("value");
  cellRes.x = blockRes / blockSize.a;
  cellRes.y = blockRes / blockSize.b;
  cellRes.z = blockRes / blockSize.c;
}

SparseVolumeGrid::SparseVolumeGrid(const Vector3& blockRes)
:hash(blockRes),blockIDCounter(0),blockSize(8,8,8),defaultValue(1,0.0)
{
  channelNames.push_back("value");
  cellRes.x = blockRes.x / blockSize.a;
  cellRes.y = blockRes.y / blockSize.b;
  cellRes.z = blockRes.z / blockSize.c;
}


SparseVolumeGrid::~SparseVolumeGrid()
{
  Clear();
}

void SparseVolumeGrid::Clear()
{
  for(auto i:hash.buckets)
    delete reinterpret_cast<Block*>(i.second);
  hash.buckets.clear();
}

int SparseVolumeGrid::AddChannel(const std::string& name)
{
  channelNames.push_back(name);
  Vector oldDefault = defaultValue;
  defaultValue.resize(oldDefault.n+1);
  defaultValue.copySubVector(0,oldDefault);
  defaultValue[oldDefault.n] = oldDefault[oldDefault.n-1];
  for(auto i:hash.buckets) {
    Block* b = reinterpret_cast<Block*>(i.second);
    //TODO: copy the channel names? or just save some memory
    b->grid.channels.resize(b->grid.channels.size()+1);
    b->grid.channels.back().MakeSimilar(b->grid.channels[0]);
  }
  return oldDefault.n;
}

int SparseVolumeGrid::GetChannel(const std::string& name) const
{
  auto i=find(channelNames.begin(),channelNames.end(),name);
  if(i==channelNames.end()) return -1;
  return i-channelNames.begin();
}

void SparseVolumeGrid::SetBlockSize(const IntTriple& _blockSize)
{
  blockSize = _blockSize;
  Vector3 blockRes = GetBlockRes();
  cellRes.x = blockRes.x / blockSize.a;
  cellRes.y = blockRes.y / blockSize.b;
  cellRes.z = blockRes.z / blockSize.c;
}

void SparseVolumeGrid::SetBlockRes(const Vector3& blockRes)
{
  hash.SetResolution(blockRes);
  cellRes.x = blockRes.x / blockSize.a;
  cellRes.y = blockRes.y / blockSize.b;
  cellRes.z = blockRes.z / blockSize.c;
}

Vector3 SparseVolumeGrid::GetBlockRes() const
{
  return hash.GetResolution();
}

void SparseVolumeGrid::MakeSimilar(const SparseVolumeGrid& grid)
{
  hash.hinv = grid.hash.hinv;
  blockSize = grid.blockSize;
}

bool SparseVolumeGrid::IsSimilar(const SparseVolumeGrid& grid) const
{
  return hash.hinv == grid.hash.hinv && blockSize == grid.blockSize;
}

void SparseVolumeGrid::GetCell(int i,int j,int k,AABB3D& cell) const
{
  cell.bmin.x = i*cellRes.x;
  cell.bmin.y = j*cellRes.y;
  cell.bmin.z = k*cellRes.z;
  cell.bmax = cell.bmin + cellRes;
}

void SparseVolumeGrid::GetCellCenter(int i,int j,int k,Vector3& center) const
{
  center.x = (i+0.5)*cellRes.x;
  center.y = (j+0.5)*cellRes.y;
  center.z = (k+0.5)*cellRes.z;
}

void SparseVolumeGrid::GetIndex(const Vector3& pt,int& i,int& j,int& k) const
{
  i = (int)Floor(pt.x/cellRes.x);
  j = (int)Floor(pt.y/cellRes.y);
  k = (int)Floor(pt.z/cellRes.z);
}

void SparseVolumeGrid::GetIndexAndParams(const Vector3& pt,IntTriple& index,Vector3& params) const
{
  Vector3 temp,temp2;
  temp.x = pt.x/cellRes.x;
  temp.y = pt.y/cellRes.y;
  temp.z = pt.z/cellRes.z;
  temp2.x = Floor(temp.x);
  temp2.y = Floor(temp.y);
  temp2.z = Floor(temp.z);
  params = temp-temp2;
  index.set((int)temp2.x,(int)temp2.y,(int)temp2.z);
}

void SparseVolumeGrid::GetIndexRange(const AABB3D& range,IntTriple& imin,IntTriple& imax) const
{
  GetIndex(range.bmin,imin);
  GetIndex(range.bmax,imax);
}

SparseVolumeGrid::Block* SparseVolumeGrid::BlockPtr(const IntTriple& blockIndex) const
{
  return reinterpret_cast<Block*>(hash.Get(blockIndex));
}

void SparseVolumeGrid::SetValue(int i,int j,int k,Real value,int channel)
{
  IntTriple hashIndex;
  GetBlock(i,j,k,hashIndex);
  MakeBlock(hashIndex);
  Block* b = BlockPtr(hashIndex);
  b->grid.channels[channel].value(modulo(i,blockSize.a),modulo(j,blockSize.b),modulo(k,blockSize.c)) = (float)value;
}

void SparseVolumeGrid::SetValue(const Vector3& pt,Real value,int channel)
{
  int a,b,c;
  GetIndex(pt,a,b,c);
  SetValue(a,b,c,value,channel);
}

void SparseVolumeGrid::SetValue(int i,int j,int k,const Vector& values)
{
  Assert(values.size() == defaultValue.size());
  IntTriple hashIndex;
  GetBlock(i,j,k,hashIndex);
  MakeBlock(hashIndex);
  Block* b = BlockPtr(hashIndex);
  for(int c=0;c<values.n;c++) 
    b->grid.channels[c].value(modulo(i,blockSize.a),modulo(j,blockSize.b),modulo(k,blockSize.c)) = (float)values[c];
}

void SparseVolumeGrid::SetValue(const Vector3& pt,const Vector& values)
{
  int a,b,c;
  GetIndex(pt,a,b,c);
  SetValue(a,b,c,values);
}

Real SparseVolumeGrid::GetValue(int i,int j,int k,int channel) const
{
  IntTriple hashIndex;
  GetBlock(i,j,k,hashIndex);
  Block* b = BlockPtr(hashIndex);
  if(!b) return defaultValue[channel];
  return b->grid.channels[channel].value(modulo(i,blockSize.a),modulo(j,blockSize.b),modulo(k,blockSize.c));
}

Real SparseVolumeGrid::GetValue(const Vector3& pt,int channel) const
{
  int a,b,c;
  GetIndex(pt,a,b,c);
  return GetValue(a,b,c,channel);
}

void SparseVolumeGrid::GetValue(int i,int j,int k,Vector& values) const
{
  IntTriple hashIndex;
  GetBlock(i,j,k,hashIndex);
  Block* b = BlockPtr(hashIndex);
  if(!b) values = defaultValue;
  else {
    int p=modulo(i,blockSize.a),q=modulo(j,blockSize.b),r=modulo(k,blockSize.c);
    for(int d=0;d<values.n;d++)
      b->grid.channels[d].value(p,q,r) = (float)values[d];
  }
}

void SparseVolumeGrid::GetValue(const Vector3& pt,Vector& values) const
{
  int a,b,c;
  GetIndex(pt,a,b,c);
  GetValue(a,b,c,values);
}

void SparseVolumeGrid::GetBlock(int i,int j,int k,IntTriple& hashIndex) const
{
  hashIndex.a = floordiv(i,blockSize.a);
  hashIndex.b = floordiv(j,blockSize.b);
  hashIndex.c = floordiv(k,blockSize.c);
}

void SparseVolumeGrid::GetBlock(const Vector3& pt,IntTriple& hashIndex) const
{
  hash.PointToIndex(pt,hashIndex);
}

void SparseVolumeGrid::GetBlockRange(const AABB3D& range,IntTriple& hashMin,IntTriple& hashMax) const
{
  GetBlock(range.bmin,hashMin);
  GetBlock(range.bmax,hashMax);
}

SparseVolumeGrid::Block* SparseVolumeGrid::GetMakeBlock(const IntTriple& blockIndex)
{
  void* res = hash.Get(blockIndex);
  if (res != NULL) return reinterpret_cast<Block*>(res);
  Block* newblock = new Block();
  newblock->id = blockIDCounter++;
  newblock->index = blockIndex;
  //TODO: copy the channel names? or just save some memory
  //newblock->grid.channelNames = channelNames.size();
  newblock->grid.channelNames.clear();
  newblock->grid.channels.resize(channelNames.size());
  newblock->grid.Resize(blockSize.a,blockSize.b,blockSize.c);
  Vector3 bmin,bmax;
  hash.IndexBucketBounds(blockIndex,bmin,bmax);
  for(size_t i=0;i<channelNames.size();i++) {
    newblock->grid.channels[i].value.set((float)defaultValue[i]);
    newblock->grid.channels[i].bb.bmin = bmin;
    newblock->grid.channels[i].bb.bmax = bmax;
  }
  hash.Set(blockIndex,newblock);
  return newblock;
}

bool SparseVolumeGrid::MakeBlock(const IntTriple& hashIndex)
{
  void* res = hash.Get(hashIndex);
  if (res != NULL) return false;
  Block* newblock = new Block();
  newblock->id = blockIDCounter++;
  newblock->index = hashIndex;
  //TODO: copy the channel names? or just save some memory
  //newblock->grid.channelNames = channelNames.size();
  newblock->grid.channelNames.clear();
  newblock->grid.channels.resize(channelNames.size());
  newblock->grid.Resize(blockSize.a,blockSize.b,blockSize.c);
  Vector3 bmin,bmax;
  hash.IndexBucketBounds(hashIndex,bmin,bmax);
  for(size_t i=0;i<channelNames.size();i++) {
    newblock->grid.channels[i].value.set((float)defaultValue[i]);
    newblock->grid.channels[i].bb.bmin = bmin;
    newblock->grid.channels[i].bb.bmax = bmax;
  }
  hash.Set(hashIndex,newblock);
  return true;
}

bool SparseVolumeGrid::EraseBlock(const IntTriple& hashIndex)
{
  void* res = hash.Get(hashIndex);
  if (res == NULL) return false;
  Block* b = reinterpret_cast<Block*>(res);
  delete b;
  hash.Erase(hashIndex);
  return true;
}

void SparseVolumeGrid::AddBlocks(const SparseVolumeGrid& grid)
{
  Assert(IsSimilar(grid));
  for(auto it = grid.hash.buckets.begin();it != grid.hash.buckets.end();it++) {
    MakeBlock(IntTriple(it->first[0],it->first[1],it->first[2]));
  }
}

void SparseVolumeGrid::GetSamples(VolumeGrid& range,int channel) const
{
  IntTriple bmin,bmax;
  IntTriple hind;
  GetBlockRange(range.bb,bmin,bmax);
  for(int i=bmin.a;i<=bmax.a;i++)
    for(int j=bmin.b;j<=bmax.b;j++)
      for(int k=bmin.c;k<=bmax.c;k++) {
        int imin = i*blockSize.a;
        int imax = ::Min(range.value.m,imin+blockSize.a);
        int jmin = j*blockSize.b;
        int jmax = ::Min(range.value.n,jmin+blockSize.b);
        int kmin = k*blockSize.c;
        int kmax = ::Min(range.value.p,kmin+blockSize.c);
        Range3Indices rind(i,i+blockSize.a,j,j+blockSize.b,k,k+blockSize.c);
        hind.set(i,j,k);
        Block* b = reinterpret_cast<Block*>(hash.Get(hind));
        if(b) {
          for(int p=imin;p<imax;p++)
            for(int q=jmin;q<jmax;q++)
              for(int r=kmin;r<kmax;r++)
                range.value(p,q,r) = b->grid.channels[channel].value(p-imin,q-jmin,r-kmin);
        }
        else {
          for(int p=imin;p<imax;p++)
            for(int q=jmin;q<jmax;q++)
              for(int r=kmin;r<kmax;r++)
                range.value(p,q,r) = defaultValue[channel];
        }
      }
}

//void SparseVolumeGrid::GetSamples_Trilinear(VolumeGrid& range) const;
//void SparseVolumeGrid::GetSamples_Average(VolumeGrid& range) const;
void SparseVolumeGrid::SetSamples(const VolumeGrid& range,int channel)
{
  IntTriple bmin,bmax;
  IntTriple hind;
  GetBlockRange(range.bb,bmin,bmax);
  for(int i=bmin.a;i<=bmax.a;i++)
    for(int j=bmin.b;j<=bmax.b;j++)
      for(int k=bmin.c;k<=bmax.c;k++) {
        int imin = i*blockSize.a;
        int imax = ::Min(range.value.m,imin+blockSize.a);
        int jmin = j*blockSize.b;
        int jmax = ::Min(range.value.n,jmin+blockSize.b);
        int kmin = k*blockSize.c;
        int kmax = ::Min(range.value.p,kmin+blockSize.c);
        Range3Indices rind(i,i+blockSize.a,j,j+blockSize.b,k,k+blockSize.c);
        hind.set(i,j,k);
        MakeBlock(hind);
        Block* b = reinterpret_cast<Block*>(hash.Get(hind));
        for(int p=imin;p<imax;p++)
          for(int q=jmin;q<jmax;q++)
            for(int r=kmin;r<kmax;r++)
              b->grid.channels[channel].value(p-imin,q-jmin,r-kmin) = (float)range.value(p,q,r);
      }
}

Real SparseVolumeGrid::TrilinearInterpolate(const Vector3& pt,int channel) const
{
  IntTriple hind;
  GetBlock(pt,hind);
  Block* b = BlockPtr(hind);
  if(!b) return defaultValue[channel];
  return b->grid.channels[channel].TrilinearInterpolate(pt);
}

//Real SparseVolumeGrid::Average(const AABB3D& range) const;
void SparseVolumeGrid::Gradient(const Vector3& pt,Vector3& grad,int channel) const
{
  IntTriple hind;
  GetBlock(pt,hind);
  Block* b = BlockPtr(hind);
  if(!b) grad.setZero();
  else b->grid.channels[channel].Gradient(pt,grad);
}
void SparseVolumeGrid::Add(const SparseVolumeGrid& grid)
{
  assert(channelNames.size() == grid.channelNames.size());
  AddBlocks(grid);
  defaultValue += grid.defaultValue;
  for(auto it : grid.hash.buckets) {
    Block* b1=reinterpret_cast<Block*>(hash.buckets[it.first]);
    Block* b2=reinterpret_cast<Block*>(it.second);
    for(size_t c=0;c<channelNames.size();c++)
      b1->grid.channels[c].Add(b2->grid.channels[c]);
  }
}
void SparseVolumeGrid::Subtract(const SparseVolumeGrid& grid)
{
  assert(channelNames.size() == grid.channelNames.size());
  AddBlocks(grid);
  defaultValue -= grid.defaultValue;
  for(auto it : grid.hash.buckets) {
    Block* b1=reinterpret_cast<Block*>(hash.buckets[it.first]);
    Block* b2=reinterpret_cast<Block*>(it.second);
    for(size_t c=0;c<channelNames.size();c++)
      b1->grid.channels[c].Subtract(b2->grid.channels[c]);
  }
}
void SparseVolumeGrid::Multiply(const SparseVolumeGrid& grid)
{
  assert(channelNames.size() == grid.channelNames.size());
  AddBlocks(grid);
  defaultValue.inplaceComponentMul(grid.defaultValue);
  for(auto it : grid.hash.buckets) {
    Block* b1=reinterpret_cast<Block*>(hash.buckets[it.first]);
    Block* b2=reinterpret_cast<Block*>(it.second);
    for(size_t c=0;c<channelNames.size();c++)
      b1->grid.channels[c].Multiply(b2->grid.channels[c]);
  }
}
void SparseVolumeGrid::Max(const SparseVolumeGrid& grid)
{
  assert(channelNames.size() == grid.channelNames.size());
  AddBlocks(grid);
  defaultValue = ::Max(defaultValue,grid.defaultValue);
  for(auto it : grid.hash.buckets) {
    Block* b1=reinterpret_cast<Block*>(hash.buckets[it.first]);
    Block* b2=reinterpret_cast<Block*>(it.second);
    for(size_t c=0;c<channelNames.size();c++)
      b1->grid.channels[c].Max(b2->grid.channels[c]);
  }
}
void SparseVolumeGrid::Min(const SparseVolumeGrid& grid)
{
  assert(channelNames.size() == grid.channelNames.size());
  AddBlocks(grid);
  defaultValue = ::Min(defaultValue,grid.defaultValue);
  for(auto it : grid.hash.buckets) {
    Block* b1=reinterpret_cast<Block*>(hash.buckets[it.first]);
    Block* b2=reinterpret_cast<Block*>(it.second);
    for(size_t c=0;c<channelNames.size();c++)
      b1->grid.channels[c].Min(b2->grid.channels[c]);
  }
}
void SparseVolumeGrid::Add(Real val,int channel)
{
  defaultValue[channel] += val;
  for(auto i=hash.buckets.begin();i!=hash.buckets.end();i++)
    reinterpret_cast<Block*>(i->second)->grid.channels[channel].Add((float)val);
}
void SparseVolumeGrid::Multiply(Real val,int channel)
{
  defaultValue[channel] *= val;
  for(auto i=hash.buckets.begin();i!=hash.buckets.end();i++)
    reinterpret_cast<Block*>(i->second)->grid.channels[channel].Multiply((float)val);
}
void SparseVolumeGrid::Max(Real val,int channel)
{
  if(defaultValue[channel] < val)
    defaultValue[channel] = val;
  for(auto i=hash.buckets.begin();i!=hash.buckets.end();i++)
    reinterpret_cast<Block*>(i->second)->grid.channels[channel].Max((float)val);
}
void SparseVolumeGrid::Min(Real val,int channel)
{
  if(defaultValue[channel] > val)
    defaultValue[channel] = val;
  for(auto i=hash.buckets.begin();i!=hash.buckets.end();i++)
    reinterpret_cast<Block*>(i->second)->grid.channels[channel].Min((float)val);
}


void SparseVolumeGrid::ExtractMesh(float isosurface,Meshing::TriMesh& mesh)
{
  mesh.tris.resize(0);
  mesh.verts.resize(0);
  const static int m=8,n=8,p=8;
  const static float NaN = std::numeric_limits<float>::quiet_NaN();
  TriMesh tempMesh;
  Array3D<float> expandedBlock(m+1,n+1,p+1);
  for(auto b=hash.buckets.begin();b!=hash.buckets.end();b++) {
    VolumeGridTemplate<float>& depth = reinterpret_cast<Block*>(b->second)->grid.channels[0];
    AABB3D center_bb = depth.bb;
    Vector3 celldims = depth.GetCellSize();
    center_bb.bmin += celldims*0.5;
    center_bb.bmax += celldims*0.5;

    //copy 8x8x8 block
    for(auto i=depth.value.begin(),j=expandedBlock.begin(Range3Indices(0,m,0,n,0,p));i!=depth.value.end();++i,++j) 
      *j = *i;

    //now work on the seams:
    IntTriple c;
    void* ptr;
    c = b->first;
    c[0] += 1;
    if((ptr=hash.Get(c))) {
      Array3D<float> &xnext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
      for(int j=0;j<n;j++) 
        for(int k=0;k<p;k++) 
          expandedBlock(m,j,k) = xnext(0,j,k);
      c[1] += 1;
      if((ptr=hash.Get(c))) {
        Array3D<float> &xynext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
        for(int k=0;k<p;k++) 
          expandedBlock(m,n,k) = xynext(0,0,k);
        c[2] += 1;
        if((ptr=hash.Get(c))) {
          Array3D<float> &xyznext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
          expandedBlock(m,n,p) = xyznext(0,0,0);
        }
        else
          expandedBlock(m,n,p) = NaN;
        c[2] -= 1;
      }
      else {
        for(int k=0;k<=p;k++) 
          expandedBlock(m,n,k) = NaN;
      }
      c[1] -= 1;
      c[2] += 1;
      if((ptr=hash.Get(c))) {
        Array3D<float> &xznext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
        for(int j=0;j<n;j++) 
          expandedBlock(m,j,p) = xznext(0,j,0);
      }
      else {
        for(int j=0;j<n;j++) 
          expandedBlock(m,j,p) = NaN;
      }
      c[2] -= 1;
    }
    else {
      for(int j=0;j<=n;j++) 
        for(int k=0;k<=p;k++) 
          expandedBlock(m,j,k) = NaN;
    }
    c[0] -= 1;
    c[1] += 1;
    if((ptr=hash.Get(c))) {
      Array3D<float> &ynext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
      for(int i=0;i<m;i++)
        for(int k=0;k<p;k++) 
          expandedBlock(i,n,k) = ynext(i,0,k);
      c[2] += 1;
      if((ptr=hash.Get(c))) {
        Array3D<float> &yznext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
        for(int i=0;i<m;i++) 
          expandedBlock(i,n,p) = yznext(i,0,0);
      }
      else
        for(int i=0;i<m;i++) 
          expandedBlock(i,n,p) = NaN;
      c[2] -= 1;
    }
    else {
      for(int i=0;i<m;i++)
        for(int k=0;k<=p;k++) 
          expandedBlock(i,n,k) = NaN;
    }
    c[1] -= 1;
    c[2] += 1;
    if((ptr=hash.Get(c))) {
      Array3D<float> &znext = reinterpret_cast<Block*>(ptr)->grid.channels[0].value;
      for(int i=0;i<m;i++)
        for(int j=0;j<n;j++) 
          expandedBlock(i,j,p) = znext(i,j,0);
    }
    else {
      for(int i=0;i<m;i++)
        for(int j=0;j<n;j++) 
          expandedBlock(i,j,p) = NaN;
    }

    MarchingCubes(expandedBlock,isosurface,center_bb,tempMesh);
    mesh.MergeWith(tempMesh);
  }
}