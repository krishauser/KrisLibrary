#include "MultiVolumeGrid.h"
#include <algorithm>
using namespace std;
using namespace Meshing;
using namespace Geometry;

MultiVolumeGrid::MultiVolumeGrid()
{
  channelNames.push_back("value");
  channels.resize(1);
}

void MultiVolumeGrid::AddChannel(const std::string& name)
{
  channelNames.push_back(name);
  channels.resize(channels.size()+1);
  if(channels.size() > 1)
    channels.back().MakeSimilar(channels[0]);
}

void MultiVolumeGrid::SetChannelName(int channel,const std::string& name)
{
  channelNames[channel] = name;
}

int MultiVolumeGrid::GetChannel(const std::string& name) const
{
  auto i=find(channelNames.begin(),channelNames.end(),name);
  if(i==channelNames.end()) return -1;
  return i-channelNames.begin();
}

void MultiVolumeGrid::Resize(int m,int n,int p)
{
  for(size_t i=0;i<channels.size();i++)
    channels[i].Resize(m,n,p);
}

void MultiVolumeGrid::ResizeByResolution(const Vector3& res)
{
  for(size_t i=0;i<channels.size();i++)
    channels[i].ResizeByResolution(res);
}

void MultiVolumeGrid::MakeSimilar(const MultiVolumeGrid& grid)
{
  channelNames = grid.channelNames;
  channels.resize(grid.channels.size());
  for(size_t i=0;i<channels.size();i++)
    channels[i].MakeSimilar(grid.channels[i]);
}
bool MultiVolumeGrid::IsSimilar(const MultiVolumeGrid& grid) const
{
  if(channels.size() != grid.channels.size()) return false;
  for(size_t i=0;i<channels.size();i++)
    if(!channels[i].IsSimilar(grid.channels[i])) return false;
  return true;
}

void MultiVolumeGrid::GetValue(const IntTriple& index,Vector& values) const
{
  ///TODO: make this a little faster by precomputing array offset?
  values.resize(channels.size());
  for(size_t i=0;i<channels.size();i++)
    values[i] = channels[i].value(index);
}

void MultiVolumeGrid::GetValue(const Vector3& pt,Vector& values) const
{
  IntTriple index;
  channels[0].GetIndex(pt,index);
  GetValue(index,values);
}

void MultiVolumeGrid::SetValue(const IntTriple& index,const Vector& values)
{
  Assert(values.size()==(int)channels.size());
  for(size_t i=0;i<channels.size();i++)
    channels[i].value(index) = values[i];
}

void MultiVolumeGrid::SetValue(const Vector3& pt,const Vector& values)
{
  IntTriple index;
  channels[0].GetIndex(pt,index);
  SetValue(index,values);
}

void MultiVolumeGrid::TrilinearInterpolate(const Vector3& pt,Vector& values) const
{
  ///TODO: make this a little faster by precomputing interpolation parameters?
  values.resize(channels.size());
  for(size_t i=0;i<channels.size();i++)
    values[i] = channels[i].TrilinearInterpolate(pt);
}

void MultiVolumeGrid::ResampleTrilinear(const MultiVolumeGrid& grid)
{
  channelNames = grid.channelNames;
  channels.resize(grid.channels.size());
  for(size_t i=1;i<channels.size();i++)
    channels[i].MakeSimilar(channels[0]);
  for(size_t i=0;i<channels.size();i++)
    channels[i].ResampleTrilinear(grid.channels[i]);
}
void MultiVolumeGrid::ResampleAverage(const MultiVolumeGrid& grid)
{
  channelNames = grid.channelNames;
  channels.resize(grid.channels.size());
  for(size_t i=1;i<channels.size();i++)
    channels[i].MakeSimilar(channels[0]);
  for(size_t i=0;i<channels.size();i++)
    channels[i].ResampleAverage(grid.channels[i]);
}
