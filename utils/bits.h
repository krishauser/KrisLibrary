#ifndef UTILS_BITS_H
#define UTILS_BITS_H

#include <KrisLibrary/errors.h>

/// Gets the index'th bit in x (starting from least significant)
inline bool GetBit(unsigned int x,int index)
{
  return ((x & (1 << index)) != 0);
}

/// Sets the index'th bit in x (starting from least significant)
inline void SetBit(unsigned int& x,int index,bool value)
{
  if(value) x |= (1<<index);
  else x &= (~(1<<index));
}

/// Returns the number of nonzero bits in x
inline int NumBits(unsigned int x)
{
  int n=0;
  while(x)
  {
    ++n; 
    x &= x - 1;
  }
  return n;
}

/// Returns the number of bits where x and y differ
inline int HammingDistance(unsigned int x,unsigned int y)
{
  return NumBits(x ^ y);
}

inline int GetLeastBit4(unsigned int x)
{
  if(x & 0x1) return 0;
  else if(x & 0x2) return 1;
  else if(x & 0x4) return 2;
  else if(x & 0x8) return 3;
  else return -1;
}

inline int GetLeastBit8(unsigned int x)
{
  if(x & 0xf) 
    return GetLeastBit4(x);
  else
    return GetLeastBit4(x>>4)+4;
}

inline int GetLeastBit16(unsigned int x)
{
  if(x & 0xff) 
    return GetLeastBit8(x);
  else
    return GetLeastBit8(x>>8)+8;
}

/** @brief Returns the index of the least significant bit of x, or -1 if x=0.
 */
inline int GetLeastBit(unsigned int x)
{
  if(x & 0xffff) 
    return GetLeastBit16(x);
  else
    return GetLeastBit16(x>>16)+16;
}

inline int GetGreatestBit4(unsigned int x)
{
  if(x & 0x8) return 3;
  else if(x & 0x4) return 2;
  else if(x & 0x2) return 1;
  else if(x & 0x1) return 0;
  else return -1;
}

inline int GetGreatestBit8(unsigned int x)
{
  if(x & 0xf0) 
    return GetGreatestBit4(x>>4)+4;
  else
    return GetGreatestBit4(x);
}

inline int GetGreatestBit16(unsigned int x)
{
  if(x & 0xff00) 
    return GetGreatestBit8(x>>8)+8;
  else
    return GetGreatestBit8(x);
}

/** @brief Returns the index of the most significant bit of x, or -1 if x=0.
 */
inline int GetGreatestBit(unsigned int x)
{
  if(x & 0xffff0000) 
    return GetGreatestBit16(x>>16)+16;
  else
    return GetGreatestBit16(x);
}


/** @brief For integer iterators in the range [begin,end),
 * sets the bits corresponding to the indices.
 */
template <class IterT>
unsigned int IndicesToBits(IterT begin,IterT end)
{
  unsigned int c=0;
  while(begin != end) {
    Assert(*begin >= 0 && *begin < 32);
    c |= (1 << *begin);
    ++begin;
  }
  return c;
}

/** @brief For integer iterators starting from begin (with at least
 * NumBits(x) entries) gets the bits corresponding to the indices. 
 * Returns the resulting end iterator
 */
template <class IterT>
IterT BitsToIndices(unsigned int x,IterT begin)
{
  unsigned int c=1;
  for(int i=0;i<32;i++) {
    if(x & c) {
      *(begin) = i;
      ++begin;
    }
    c <<= 1;
  }
  return begin;
}

/** @brief For boolean iterators in the range [begin,end),
 * sets the bits corresponding to the nonzero entries.
 */
template <class IterT>
unsigned int IteratorToBits(IterT begin,IterT end)
{
  unsigned int c=0;
  int i=0;
  while(begin != end) {
    Assert(*begin >= 0 && *begin < 32);
    if(*begin) c |= (1 << i);
    ++begin;
    ++i;
  }
  return c;
}

/** @brief For boolean iterators in the range [begin,end),
 * gets the bits corresponding to the bits of x.
 * end-begin must be >= the most significant bit of x for the output to
 * be correct.
 */
template <class IterT>
void BitsToIterator(unsigned int x,IterT begin,IterT end)
{
  unsigned int c=1;
  for(int i=0;i<32;i++) {
    if(x & c) *(begin) = 1;
    else (*begin) = 0;
    c <<= 1;
    ++begin;
    if(begin == end) break;
  }
  while(begin != end) {
    *begin=0; 
    ++begin;
  } 
}

#endif
