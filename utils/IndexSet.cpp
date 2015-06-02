#include "IndexSet.h"
#include <utils.h>
using namespace std;

IndexSet::IndexSet(int _imax)
  :imin(0),imax(_imax)
{}

IndexSet::IndexSet(int _imin,int _imax)
  :imin(_imin),imax(_imax)
{}

IndexSet::IndexSet(const std::vector<int>& _indices)
  :imin(0),imax(-1),indices(_indices)
{}

IndexSet::operator std::vector<int> () const
{
  if(IsRange()) {
    vector<int> res(Size());
    for(size_t i=0;i<res.size();i++)
      res[i] = (int)i+imin;
    return res;
  }
  else return indices;
}

size_t IndexSet::Size() const
{
  if(IsRange())
    return imax-imin;
  else
    return indices.size();
}

int IndexSet::operator [] (int i) const
{
  if(IsRange()) return imin+i;
  else return indices[i];
}

int IndexSet::Find(int i) const
{
  if(IsRange()) {
    if(i < imin || i >= imax) return -1;
    return i-imin;
  }
  else {
    for(size_t k=0;k<indices.size();k++)
      if(indices[k] == i) return (int)k;
    return -1;
  }
}

int IndexSet::MaxValue() const
{
  if(IsRange()) return imax;
  Assert(!indices.empty());
  int imax=indices[0];
  for(size_t i=1;i<indices.size();i++)
    imax = Max((int)imax,indices[i]);
  return imax;
}
