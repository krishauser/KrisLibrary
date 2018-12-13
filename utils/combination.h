#ifndef UTILS_COMBINATION_H
#define UTILS_COMBINATION_H

#include <vector>
#include <KrisLibrary/errors.h>

/** @file utils/combination.h
 * @ingroup Utils
 * @brief Various combination utilities.
 *
 * A combination is defined by an array v (or a vector) of length k,
 * with entries from 0 to n.  Elements are sorted and unique.
 */

/** @addtogroup Utils */
/*@{*/

inline void FirstCombination(std::vector<int>& v,int n)
{
  int k=(int)v.size();
  Assert(k <= n);
  for(int i=0;i<k;i++) v[i]=i;
}

inline void LastCombination(std::vector<int>& v,int n)
{
  int k=(int)v.size();
  Assert(k <= n);
  for(int i=0;i<k;i++) v[i]=i+n-k;
}

inline int NextCombination(std::vector<int>& v,int n)
{
  int k=(int)v.size();
  Assert(k <= n);
  for(int i=k-1;i>=0;i--) {
    if(v[i] >= i+n-k) { //can't go any farther, loop the previous
      if(i == 0) {
	FirstCombination(v,n);
	return 1;
      }
      v[i] = v[i-1]+2;
      if(v[i] > i+n-k) {
	FirstCombination(v,n);
	return 1;
      }
    }
    else {
      v[i]++;
      return 0;
    }
  }
  //should never reach here
  FirstCombination(v,n);
  return 1;
}

inline int PrevCombination(std::vector<int>& v,int n)
{
  int k=(int)v.size();
  Assert(k <= n);
  for(int i=k-1;i>=0;i--) {
    if(i > 0) {
      if(v[i] <= v[i-1]+1)  //can't go any farther, loop the previous
	v[i] = i+n-k;
      else {
	v[i]--;
	return 0;
      }
    }
    else {
      if(v[i] > 0) {
	v[i]--;
	return 0;
      }
      else {
	LastCombination(v,n);
	return 1;
      }
    }
  }
  //should never get here
  LastCombination(v,n);
  return 1;
}

/** @brief A class that enumerates combinations.
 */
class Combination
{
 public:
  inline Combination(int _n) :value(_n,0),n(_n),numCycles(0) { FirstCombination(value,n); }
  inline Combination(int _k,int _n) :value(_k,0),n(_n),numCycles(0) { Assert(_k <= _n); FirstCombination(value,n); }
  inline const Combination& operator ++() {
    numCycles += NextCombination(value,n);
    return *this;
  }
  inline const Combination& operator ++(int) { return operator ++(); }
  inline const Combination& operator --() {
    numCycles -= PrevCombination(value,n);
    return *this;
  }
  inline const Combination& operator --(int) { return operator --(); }
  inline int cycleCount() const { return numCycles; }
  inline bool isDone() const { return numCycles>0; }
  inline const std::vector<int>& operator* () const { return value; }
  inline const std::vector<int>* operator ->() const { return &value; }
  inline const Combination& operator = (const Combination& c) {
    value = c.value;
    n = c.n;
    numCycles = c.numCycles;
    return *this;
  }
  inline bool operator == (const Combination& c) const { return n == c.n && numCycles == c.numCycles && value == c.value; }
  inline bool operator < (const Combination& c) const { 
    Assert(n == c.n);
    if(numCycles < c.numCycles) return true;
    else if(numCycles > c.numCycles) return false;
    return std::lexicographical_compare(value.begin(),value.end(),c.value.begin(),c.value.end());
  }

 private:
  std::vector<int> value;
  int n;
  int numCycles;
};

/*@}*/

#endif
