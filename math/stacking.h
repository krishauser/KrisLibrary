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
void Stack(const VectorTemplate<T>& v1,const VectorTemplate<T>& v2,VectorTemplate<T>& out)
{
  out.resize(v1.n+v2.n);
  out.copySubVector(0,v1);
  out.copySubVector(v1.n,v2);
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

template <class T>
void HStack(const MatrixTemplate<T>& v1,const MatrixTemplate<T>& v2,MatrixTemplate<T>& out)
{
  Assert(v1.n == v2.n);
  out.resize(v1.m+v2.m,v1.n);
  out.copySubMatrix(0,0,v1);
  out.copySubMatrix(v1.m,0,v2);
}

template <class T>
void VStack(const MatrixTemplate<T>& v1,const MatrixTemplate<T>& v2,MatrixTemplate<T>& out)
{
  Assert(v1.m == v2.m);
  out.resize(v1.m,v1.n+v2.n);
  out.copySubMatrix(0,0,v1);
  out.copySubMatrix(0,v1.n,v2);
}

} //namespace Math

#endif
