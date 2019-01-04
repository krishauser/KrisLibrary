#ifndef UTILS_PERMUTATION_H
#define UTILS_PERMUTATION_H

#include <vector>
#include <KrisLibrary/errors.h>

/** @file utils/permutation.h
 * @ingroup Utils
 * @brief Various permutation utilities.
 *
 * A permutation is defined by an array v (or a vector) of length k,
 * with entries from 0 to n.
 */

/** @addtogroup Utils */
/*@{*/

inline void IdentityPermutation(int v[],int n)
{
  for(int i=0;i<n;i++) v[i]=i;
}

template <class T>
inline void RandomlyPermute(T v[],int n)
{
  for(int i=0;i<n;i++) {
    int k=i+rand()%(n-i);
    std::swap(v[i],v[k]);
  }
}

inline void RandomPermutation(int v[],int n)
{
  IdentityPermutation(v,n);
  RandomlyPermute(v,n);
}

inline void IdentityPermutation(std::vector<int>& v)
{
  int n=(int)v.size();
  for(int i=0;i<n;i++) v[i]=i;
}

template <class T>
inline void RandomlyPermute(std::vector<T>& v)
{
  int n=(int)v.size();
  for(int i=0;i<n;i++) {
    int k=i+rand()%(n-i);
    std::swap(v[i],v[k]);
  }
}

inline void RandomPermutation(std::vector<int>& v)
{
  IdentityPermutation(v);
  RandomlyPermute(v);
}


inline void FirstPermutation(int v[],int k,int n)
{
  for(int i=0;i<k;i++) v[i]=0;
}

inline void FirstPermutation(std::vector<int>& v,int n)
{
  std::fill(v.begin(),v.end(),0);
}

inline void LastPermutation(int v[],int k,int n)
{
  for(int i=0;i<k;i++) v[i]=n-1;
}

inline void LastPermutation(std::vector<int>& v,int n)
{
  std::fill(v.begin(),v.end(),n-1);
}


///Gets the next lexicographically ordered combination.
///Returns 1 if it reaches the end.
inline int NextPermutation(int v[],int k,int n)
{
  for(int i=k-1;i>=0;i--) {
    v[i]++;
    if(v[i] >= n) v[i]=0;
    else return 0;
  }
  return 1;
}

///Gets the next lexicographically ordered combination.
///Returns 1 if it reaches the end.
inline int NextPermutation(std::vector<int>& v,int n)
{
  int k=(int)v.size();
  for(int i=k-1;i>=0;i--) {
    v[i]++;
    if(v[i] >= n) v[i]=0;
    else return 0;
  }
  return 1;
}

///Gets the previous lexicographically ordered combination.
///Returns 1 if it reaches the beginning.
inline int PrevPermutation(int v[],int k,int n)
{
  for(int i=k-1;i>=0;i--) {
    v[i]--;
    if(v[i] < 0) v[i]=n-1;
    else return 0;
  }
  return 1;
}

///Gets the previous lexicographically ordered combination.
///Returns 1 if it reaches the beginning.
inline int PrevPermutation(std::vector<int>& v,int n)
{
  int k=(int)v.size();
  for(int i=k-1;i>=0;i--) {
    v[i]--;
    if(v[i] < 0) v[i]=n-1;
    else return 0;
  }
  return 1;
}


/** @brief A class that enumerates permutations.
 */
class Permutation
{
 public:
  inline Permutation(int _n) :value(_n,0),n(_n),numCycles(0) {}
  inline Permutation(int _k,int _n) :value(_k,0),n(_n),numCycles(0) {}
  inline const Permutation& operator ++() {
    numCycles += NextPermutation(value,n);
    return *this;
  }
  inline const Permutation& operator ++(int) { return operator ++(); }
  inline const Permutation& operator --() {
    numCycles -= PrevPermutation(value,n);
    return *this;
  }
  inline const Permutation& operator --(int) { return operator --(); }
  inline int cycleCount() const { return numCycles; }
  inline bool isDone() const { return numCycles>0; }
  inline const std::vector<int>& operator*() const { return value; }
  inline const std::vector<int>* operator ->() const { return &value; }
  inline const Permutation& operator = (const Permutation& c) {
    value = c.value;
    n = c.n;
    numCycles = c.numCycles;
    return *this;
  }
  inline bool operator == (const Permutation& c) const { return n == c.n && numCycles == c.numCycles && value == c.value; }
  inline bool operator < (const Permutation& c) const { 
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
