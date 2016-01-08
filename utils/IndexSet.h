#ifndef INDEX_SET_H
#define INDEX_SET_H

#include <vector>
#include <KrisLibrary/errors.h>

/**@brief A generic set of indices, either listed explicitly or in a range. 
 * If imax < imin, this indicates that the indices should be used instead.
 */
class IndexSet
{
 public:
  IndexSet(int imax=0);
  IndexSet(int imin,int imax);
  IndexSet(const std::vector<int>& indices);
  operator std::vector<int> () const;
  int operator [] (int i) const;
  inline bool IsRange() const { return imin<=imax; }
  size_t Size() const;
  int Find(int index) const;
  int MaxValue() const;
  template <class T> void GetElements(const T& x,T& xind) const;
  template <class T> void SetElements(const T& xind,T& x) const;

  int imin,imax;
  std::vector<int> indices;
};

template <class T>
void IndexSet::GetElements(const T& x,T& xind) const
{
  xind.resize(Size());
  if(IsRange()) 
    std::copy(x.begin()+imin,x.begin()+imax,xind.begin());
  else {
    for(size_t i=0;i<indices.size();i++)
      xind[i] = x[indices[i]];
  }
}

template <class T>
void IndexSet::SetElements(const T& xind,T& x) const
{
  Assert((size_t)xind.size()==Size());
  if(IsRange()) {
    Assert(imin >= 0);
    Assert(imax < (int)x.size());
    std::copy(xind.begin(),xind.end(),x.begin()+imin);
  }
  else {
    for(size_t i=0;i<indices.size();i++) {
      Assert(indices[i] >= 0 && indices[i] < (int)x.size());
      x[indices[i]] = xind[i];
    }
  }
}

#endif
