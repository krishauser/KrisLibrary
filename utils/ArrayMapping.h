#ifndef ARRAY_MAPPING_H
#define ARRAY_MAPPING_H

#include <vector>

/** @ingroup Utils
 * @brief An invertible mapping from indices in [0,imax), which operates in
 * two modes: offset mode adds an offset, and mapping mode stores an
 * explicit mapping.
 *
 * Maps array indices i in [0,imax) to either:
 *  - (i+offset) 
 *  - (mapping[i]) (if the mapping is defined)
 * Map method 1 maps an index to its target value.
 * Map method 2 maps values from x to mapx, that is, mapx[Map(i)] <- x[i]
 * InvMap method 1 maps a target value to an index, or < 0 if the value is
 *   not in the map's range.
 * InvMap method 2 maps values from mapx to x, that is, x[InvMap(i)] <- mapx[i]
 */
struct ArrayMapping
{
  inline ArrayMapping();

  inline bool IsOffset() const { return mapping.empty(); }
  inline int Size() const { return (IsOffset() ?  imax : (int)mapping.size()); } 
  inline void SetOffset(int offset,int imax);

  inline int Map(int i) const;
  inline int InvMap(int imap) const;
  template <class A>
  inline void Map(const A& x,A& mapx) const;
  template <class A>
  inline void InvMap(const A& mapx,A& x) const;  

  std::vector<int> mapping;    
  int imax;
  int offset;
};


ArrayMapping::ArrayMapping()
  :imax(0),offset(0)
{}

void ArrayMapping::SetOffset(int _offset,int _imax) 
{
  mapping.clear(); 
  offset=_offset; 
  imax=_imax; 
}

int ArrayMapping::Map(int i) const 
{
  return (IsOffset() ? i+offset : mapping[i]);
}

int ArrayMapping::InvMap(int imap) const 
{
  if(IsOffset()) 
    return imap-offset;
  else {
    for(size_t i=0;i<mapping.size();i++)
      if(mapping[i] == imap) return (int)i;
    abort();
    return -1;
  }
}

template <class A>
void ArrayMapping::Map(const A& x,A& mapx) const 
{
  if(mapping.empty()) {
    for(int i=0;i<imax;i++)
      mapx[i+offset] = x[i];
  }
  else {
    for(int i=0;i<(int)mapping.size();i++)
      mapx[mapping[i]] = x[i];
  }
}

template <class A>
void ArrayMapping::InvMap(const A& mapx,A& x) const 
{
  if(IsOffset()) {
    for(int i=0;i<imax;i++)
      x[i] = mapx[i+offset];
  }
  else {
    for(int i=0;i<(int)mapping.size();i++)
      x[i] = mapx[mapping[i]];
  }
}
  


#endif
