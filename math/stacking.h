#ifndef MATH_STACKING_H
#define MATH_STACKING_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"
#include <vector>

namespace Math {

/** @ingroup Math
 * @file math/indexing.h
 * @brief Utilities for matrix/vector stacking
 */

template <class T>
void Stack(const std::vector<VectorTemplate<T> >& vs,VectorTemplate<T>& out)
{
  int n=0;
  for(size_t i=0;i<vs.size();i++) n += vs[i].n;
  out.resize(n);
  n=0;
  for(size_t i=0;i<vs.size();i++) {
    out.copySubVector(n,vs[i]);
    n += vs[i].n;
  }
}


template <class T>
void HStack(const std::vector<VectorTemplate<T> >& vs,MatrixTemplate<T>& out)
{
  Assert(!vs.empty());
  for(size_t i=0;i<vs.size();i++) Assert(vs[i].n==vs[0].n);
  out.resize(vs.size(),vs[0].n);
  for(size_t i=0;i<vs.size();i++) 
    out.copyRow(i,vs[i]);
}


template <class T>
void VStack(const std::vector<VectorTemplate<T> >& vs,MatrixTemplate<T>& out)
{
  Assert(!vs.empty());
  for(size_t i=0;i<vs.size();i++) Assert(vs[i].n==vs[0].n);
  out.resize(vs[0].n,vs.size());
  for(size_t i=0;i<vs.size();i++) 
    out.copyCol(i,vs[i]);
}

} //namespace Math

#endif
