#include "indexing.h"
#include <stdlib.h>
#include "errors.h"
using namespace std;

int IncrementIndex(vector<int>& i,const vector<int>& imin,const vector<int>& imax)
{
  Assert(i.size()==imin.size());
  Assert(i.size()==imax.size());
  for(size_t k=0;k<i.size();k++) {
    i[k]++;
    if(i[k] > imax[k]) {
      i[k]=imin[k];
    }
    else {
      return 0;
    }
  }
  return 1;
}

int IncrementIndex(vector<int>& i,const vector<int>& imax)
{
  Assert(i.size()==imax.size());
  for(size_t k=0;k<i.size();k++) {
    i[k]++;
    if(i[k] >= imax[k]) {
      i[k]=0;
    }
    else {
      return 0;
    }
  }
  return 1;
}


RangeIndices::RangeIndices()
  :start(0),size(0),stride(1)
{}

RangeIndices::RangeIndices(const RangeIndices& rhs)
  :start(rhs.start),size(rhs.size),stride(rhs.stride)
{}

RangeIndices::RangeIndices(int max)
  :start(0),size(max),stride(1)
{}

RangeIndices::RangeIndices(int min,int max)
  :start(min),size(max-min),stride(1)
{
  if(size < 0) {  //reversed range
    stride=-stride;
    size=-size;
  }
}

RangeIndices::RangeIndices(int min,int max,int _stride)
  :start(min),size(max-min),stride(_stride)
{
  Assert(stride >= 1);
  if(size < 0) {  //reversed range
    stride=-stride;
    size=-size;
  }
}

void RangeIndices::enumerate(vector<int>& indices)
{
  indices.resize(size);
  int k=start;
  for(int i=0;i<size;i++) {
    indices[i] = k;
    k+=stride;
  }
}

bool RangeIndices::contains(int index) const
{
  if(stride != 1) {
    div_t d=div(index-start,stride);
    if(d.quot < 0 || d.quot>=size) return false;
    if(d.rem != 0) return false;
  }
  else {
    if(index < start || index >= start+size) return false;
  }
  return true;
}

int RangeIndices::indexToElement(int index) const
{
  if(stride != 1) {
    div_t d=div(index-start,stride);
    if(d.rem != 0) return -1;
    return d.quot;
  }
  else {
    return index-start;
  }
  return true;  
}

bool RangeIndices::operator == (const RangeIndices& range) const
{
  if(this == &range) return true;
  return size==range.size && start==range.start && stride==range.stride;
}



RangeIndices::iterator::iterator()
  :range(NULL),i(0),index(0)
{}

RangeIndices::iterator::iterator(const RangeIndices* _range)
  :range(_range),i(0),index(_range->start)
{}

RangeIndices::iterator::iterator(const RangeIndices* _range, int invalid)
  :range(_range),i(_range->size),index(_range->start+_range->size*_range->stride)
{}

RangeIndices::iterator::iterator(const iterator& rhs)
  :range(rhs.range),i(rhs.i),index(rhs.index)
{}

void RangeIndices::iterator::setElement(int _i)
{
  i=_i;
  index=range->start+i*range->stride;
}

RangeIndices::iterator& RangeIndices::iterator::operator ++ ()
{
  i++;
  index+=range->stride;
  return *this;
}

RangeIndices::iterator& RangeIndices::iterator::operator -- ()
{
  i--;
  index-=range->stride;
  return *this;
}

RangeIndices::iterator& RangeIndices::iterator::operator += (int skip)
{
  Assert(skip >= 0);
  i+=skip;
  index+=skip*range->stride;
  return *this;
}

RangeIndices::iterator& RangeIndices::iterator::operator -= (int skip)
{
  Assert(skip >= 0);
  i-=skip;
  index-=skip*range->stride;
  return *this;
}

bool RangeIndices::iterator::isInvalid() const
{
  return (i<0 || i>=range->size);
}

bool RangeIndices::iterator::operator < (const iterator& rhs) const
{
  Assert(range == rhs.range);
  return i < rhs.i;
}

bool RangeIndices::iterator::operator == (const iterator& rhs) const
{
  if(*range!=*rhs.range) return false;
  if(isInvalid()) return rhs.isInvalid();
  return i==rhs.i;
}






Range2Indices::Range2Indices()
{}

Range2Indices::Range2Indices(const Range2Indices& rhs)
  :irange(rhs.irange),jrange(rhs.jrange)
{}

Range2Indices::Range2Indices(const RangeIndices& _irange,const RangeIndices& _jrange)
  :irange(_irange),jrange(_jrange)
{}

Range2Indices::Range2Indices(int imax,int jmax)
  :irange(imax),jrange(jmax)
{}

Range2Indices::Range2Indices(int imin,int imax,int jmin,int jmax)
  :irange(imin,imax),jrange(jmin,jmax)
{}

Range2Indices::Range2Indices(int imin,int imax,int istride,int jmin,int jmax,int jstride)
  :irange(imin,imax,istride),jrange(jmin,jmax,jstride)
{}

void Range2Indices::enumerate(std::vector<IntPair>& indices)
{
  indices.resize(irange.size*jrange.size);
  int count=0;
  int ki=irange.start;
  for(int i=0;i<irange.size;i++) {
    int kj=jrange.start;
    for(int j=0;j<jrange.size;j++) {
      indices[count].set(ki,kj);
      kj += jrange.stride;
    }
    ki += irange.stride;
  }
  /*
  indices.resize(irange.size*jrange.size);
  int k=0;
  for(iterator i=begin();i!=end();++i,++k)
    indices[k]=*i;
  */
}

Range2Indices::iterator::iterator()
{}

Range2Indices::iterator::iterator(const RangeIndices& irange,const RangeIndices& jrange)
  :i(irange.begin()),j(jrange.begin()),element(0)
{}

Range2Indices::iterator::iterator(const RangeIndices& irange,const RangeIndices& jrange,int invalid)
  :i(irange.end()),j(jrange.begin()),element(irange.size*jrange.size)
{}

Range2Indices::iterator::iterator(const iterator& rhs)
  :i(rhs.i),j(rhs.j),element(rhs.element)
{}

void Range2Indices::iterator::setElement(int k)
{
  element=k;
  div_t d=div(k,j.range->size);
  i.setElement(d.quot);
  j.setElement(d.rem);
}

void Range2Indices::iterator::setElement(int _i,int _j)
{
  element = _i*j.range->size+_j;
  i.setElement(_i);
  j.setElement(_j);
}

Range2Indices::iterator& Range2Indices::iterator::operator ++ ()
{
  ++j;
  element++;
  if(j.i >= j.range->size) {
    j.setElement(0);
    ++i;
  }
  return *this;
}

Range2Indices::iterator& Range2Indices::iterator::operator -- ()
{
  --j;
  element--;
  if(j.i < 0) {
    j.setElement(j.range->size-1);
    --i;
  }
  return *this;
}

Range2Indices::iterator& Range2Indices::iterator::operator += (int skip)
{
  Assert(skip >= 0);
  j+=skip;
  element+=skip;
  if(j.i >= j.range->size) {
    div_t d=div(j.i,j.range->size);
    j.setElement(d.rem);
    i+=d.quot;
  }
  return *this;
}

Range2Indices::iterator& Range2Indices::iterator::operator -= (int skip)
{
  Assert(skip >= 0);
  j-=skip;
  element-=skip;
  if(j.i < 0) {
    div_t d=div(j.i,j.range->size);  //rounds toward zero
    Assert(d.rem < 0);
    j.setElement(d.rem+j.range->size);
    i+=(d.quot-1);
  }
  return *this;
}

bool Range2Indices::iterator::operator < (const iterator& rhs) const
{
  if(*i.range != *rhs.i.range) return false;
  if(*j.range != *rhs.j.range) return false;
  return element < rhs.element;
}






Range3Indices::Range3Indices()
{}

Range3Indices::Range3Indices(const Range3Indices& rhs)
  :irange(rhs.irange),jrange(rhs.jrange),krange(rhs.krange)
{}

Range3Indices::Range3Indices(const RangeIndices& _irange,const RangeIndices& _jrange,const RangeIndices& _krange)
  :irange(_irange),jrange(_jrange),krange(_krange)
{}

Range3Indices::Range3Indices(int imax,int jmax,int kmax)
  :irange(imax),jrange(jmax),krange(kmax)
{}

Range3Indices::Range3Indices(int imin,int imax,int jmin,int jmax,int kmin,int kmax)
  :irange(imin,imax),jrange(jmin,jmax),krange(kmin,kmax)
{}


void Range3Indices::enumerate(std::vector<IntTriple>& indices)
{
  indices.resize(irange.size*jrange.size*krange.size);
  int count=0;
  int ki=irange.start;
  for(int i=0;i<irange.size;i++) {
    int kj=jrange.start;
    for(int j=0;j<jrange.size;j++) {
      int kk=krange.start;
      for(int k=0;k<krange.size;k++) {
	indices[count].set(ki,kj,kk);
	kk += krange.stride;
      }
      kj += jrange.stride;
    }
    ki += irange.stride;
  }
  /*
  indices.resize(irange.size*jrange.size*krange.size);
  int k=0;
  for(iterator i=begin();i!=end();++i,++k)
    indices[k]=*i;
  */
}

Range3Indices::iterator::iterator()
{}

Range3Indices::iterator::iterator(const RangeIndices& irange,const RangeIndices& jrange,const RangeIndices& krange)
  :i(irange.begin()),j(jrange.begin()),k(krange.begin()),element(0)
{}

Range3Indices::iterator::iterator(const RangeIndices& irange,const RangeIndices& jrange,const RangeIndices& krange,int invalid)
  :i(irange.end()),j(jrange.begin()),k(krange.begin()),element(irange.size*jrange.size*krange.size)
{}

Range3Indices::iterator::iterator(const iterator& rhs)
  :i(rhs.i),j(rhs.j),k(rhs.k),element(rhs.element)
{}

void Range3Indices::iterator::setElement(int m)
{
  element=m;
  div_t d=div(m,k.range->size);
  k.setElement(d.rem);
  m=d.rem;
  d=div(m,j.range->size);
  j.setElement(d.rem);
  i.setElement(d.quot);
}

void Range3Indices::iterator::setElement(int _i,int _j,int _k)
{
  element = (_i*j.range->size+_j)*k.range->size+_k;
  i.setElement(_i);
  j.setElement(_j);
  k.setElement(_k);
}

Range3Indices::iterator& Range3Indices::iterator::operator ++ ()
{
  --k;
  element++;
  if(k.i >= k.range->size) {
    k.setElement(0);
    ++j;
    if(j.i >= j.range->size) {
      j.setElement(0);
      ++i;
    }
  }
  return *this;
}

Range3Indices::iterator& Range3Indices::iterator::operator -- ()
{
  --k;
  element--;
  if(k.i < 0) {
    k.setElement(k.range->size-1);
    --j;
    if(j.i < 0) {
      j.setElement(j.range->size-1);
      --i;
    }
  }
  return *this;
}

Range3Indices::iterator& Range3Indices::iterator::operator += (int skip)
{
  Assert(skip >= 0);
  k+=skip;
  element+=skip;
  if(k.i >= k.range->size) {
    div_t d=div(k.i,k.range->size);
    k.setElement(d.rem);
    j+=d.quot;
    if(j.i >= j.range->size) {
      d=div(j.i,j.range->size);
      j.setElement(d.rem);
      i+=d.quot;
    }
  }
  return *this;
}

Range3Indices::iterator& Range3Indices::iterator::operator -= (int skip)
{
  Assert(skip >= 0);
  k-=skip;
  element-=skip;
  if(k.i < 0) {
    div_t d=div(k.i,k.range->size);  //rounds toward zero
    Assert(d.rem < 0);
    k.setElement(d.rem+k.range->size);  
    j+=(d.quot-1);
    if(j.i < 0) {
      d=div(j.i,j.range->size);  //rounds toward zero
      j.setElement(d.rem+k.range->size);
      i+=(d.quot-1);
    }
  }
  return *this;
}

bool Range3Indices::iterator::operator < (const iterator& rhs) const
{
  if(*i.range != *rhs.i.range) return false;
  if(*j.range != *rhs.j.range) return false;
  if(*k.range != *rhs.k.range) return false;
  return element < rhs.element;
}







Stripe2Indices::Stripe2Indices()
  :base(0),isize(0),jsize(0),istride(1),jstride(1)
{}

Stripe2Indices::Stripe2Indices(const Range2Indices& rhs)
{
  isize=rhs.irange.size;
  jsize=rhs.jrange.size;
  istride=rhs.irange.stride*rhs.jrange.size*rhs.jrange.stride;
  jstride=rhs.jrange.stride;
  //count from zero
  base=rhs.jrange.start*jstride+rhs.irange.start*istride;
}

Stripe2Indices::Stripe2Indices(const Stripe2Indices& rhs)
  :base(rhs.base),isize(rhs.isize),jsize(rhs.jsize),istride(rhs.istride),jstride(rhs.jstride)
{}

Stripe2Indices::Stripe2Indices(int _isize,int _jsize,const Range2Indices& subRange)
//:base(0),isize(_isize),jsize(_jsize),istride(_jsize),jstride(1)
{
  base = subRange.irange.start*_jsize + subRange.jrange.start;
  isize = subRange.irange.size;
  jsize = subRange.jrange.size;
  istride = subRange.irange.stride*_jsize;
  jstride = subRange.jrange.stride;
}

Stripe2Indices::Stripe2Indices(const RangeIndices& irange,const RangeIndices& jrange)
{
  isize=irange.size;
  jsize=jrange.size;
  istride=irange.stride*jrange.size*jrange.stride;
  jstride=jrange.stride;
  //count from zero
  base=jrange.start*jstride+irange.start*istride;
}

Stripe2Indices::Stripe2Indices(int _isize,int _jsize,int _base,int _istride,int _jstride)
  :base(_base),isize(_isize),jsize(_jsize),istride(_istride*_jsize*_jstride),jstride(_jstride)
{}

void Stripe2Indices::enumerate(std::vector<int>& indices)
{
  indices.resize(isize*jsize);
  int k=base;
  int count=0;
  for(int i=0;i<isize;i++) {
    int ki=k;
    for(int j=0;j<jsize;j++,count++) {
      indices[count] = k;
      k += jstride;
    }
    k = ki+istride;
  }
}

bool Stripe2Indices::contains(int index) const
{
  div_t di = div(index-base,istride);
  div_t dj = div(index-base,jstride);
  if(di.rem != 0 || dj.rem != 0) return false;
  if(di.quot < 0 || di.quot >= isize) return false;
  if(dj.quot < 0 || dj.quot >= jsize) return false;
  return true;
}

IntPair Stripe2Indices::indexToElement(int index) const
{
  div_t di = div(index-base,istride);
  div_t dj = div(index-base,jstride);
  return IntPair(di.quot,dj.quot);
}

void Stripe2Indices::setRange(const Stripe2Indices& stripe,const Range2Indices& inds)
{
  Assert(this != &stripe);
  base = stripe.base + inds.irange.start*stripe.istride + inds.jrange.start*stripe.jstride;
  isize = inds.irange.size;
  jsize = inds.jrange.size;
  istride = inds.irange.stride*stripe.istride;
  jstride = inds.jrange.stride*stripe.jstride;
}

bool Stripe2Indices::operator == (const Stripe2Indices& stripe) const
{
  if(this == &stripe) return true;
  return base==stripe.base && isize==stripe.isize && jsize==stripe.jsize && istride==stripe.istride && jstride==stripe.jstride;
}

Stripe2Indices::iterator::iterator(const Stripe2Indices* _stripe)
  :stripe(_stripe),i(0),j(0),index(_stripe->base),stripeIndex(_stripe->base)
{}

Stripe2Indices::iterator::iterator(const Stripe2Indices* _stripe,int invalid)
  :stripe(_stripe),i(_stripe->isize),j(0),index(stripe->base+_stripe->isize*_stripe->istride),stripeIndex(_stripe->base+_stripe->isize*_stripe->istride)
{}

Stripe2Indices::iterator::iterator(const iterator& it)
  :stripe(it.stripe),i(it.i),j(it.j),index(it.index),stripeIndex(it.stripeIndex)
{}

Stripe2Indices::iterator& Stripe2Indices::iterator::operator ++()
{
  j++;
  index+=stripe->jstride;
  if(j >= stripe->jsize) {
    j=0;
    i++;
    stripeIndex+=stripe->istride;
    index=stripeIndex;
  }
  return *this;
}

Stripe2Indices::iterator& Stripe2Indices::iterator::operator --()
{
  j--;
  index-=stripe->jstride;
  if(j < 0) {
    j=stripe->jsize-1;
    i--;
    stripeIndex-=stripe->istride;
    index=stripeIndex;
  }
  return *this;
}

Stripe2Indices::iterator& Stripe2Indices::iterator::operator +=(int skip)
{
  Assert(skip >= 0);
  j+=skip;
  index+=stripe->jstride*skip;
  if(j >= stripe->jsize) {
    div_t d=div(j,stripe->jsize);
    j=d.rem;
    i+=d.quot;
    stripeIndex+=stripe->istride*d.quot;
    index=stripeIndex+j*stripe->jstride;
  }
  return *this;
}

Stripe2Indices::iterator& Stripe2Indices::iterator::operator -=(int skip)
{
  Assert(skip >= 0);
  j-=skip;
  index-=stripe->jstride*skip;
  if(j < 0) {
    div_t d=div(j,stripe->jsize);  //rounds toward zero
    Assert(d.rem < 0);
    j=d.rem+stripe->jsize;
    i+=(d.quot-1);
    stripeIndex+=stripe->istride*(d.quot-1);
    index=stripeIndex+j*stripe->jstride;
  }
  return *this;
}

void Stripe2Indices::iterator::incFirst(int skip)
{
  i += skip;
  stripeIndex+=skip*stripe->istride;
  index+=skip*stripe->istride;
}

void Stripe2Indices::iterator::incSecond(int skip)
{
  j += skip;
  index += skip*stripe->jstride;
}

bool Stripe2Indices::iterator::isInvalid() const
{
  return i<0 || i>=stripe->isize || j<0 || j>=stripe->jsize; 
}

bool Stripe2Indices::iterator::operator == (const iterator& rhs) const
{
  if(*stripe != *rhs.stripe) return false;
  if(isInvalid()) return rhs.isInvalid();
  return index == rhs.index;
}


bool Stripe2Indices::iterator::operator < (const iterator& rhs) const
{
  if(stripe != rhs.stripe) return false;
  return index < rhs.index;
}






Stripe3Indices::Stripe3Indices()
  :base(0),isize(0),jsize(0),ksize(0),istride(1),jstride(1),kstride(1)
{}

Stripe3Indices::Stripe3Indices(int _isize,int _jsize,int _ksize,const Range3Indices& subRange)
//:base(0),isize(_isize),jsize(_jsize),ksize(_ksize),istride(_jsize*_ksize),jstride(_ksize),kstride()
{
  base = subRange.irange.start*_jsize*_ksize + subRange.jrange.start*_ksize + subRange.krange.start;
  isize = subRange.irange.size;
  jsize = subRange.jrange.size;
  ksize = subRange.krange.size;
  istride = subRange.irange.stride*_jsize*_ksize;
  jstride = subRange.jrange.stride*_ksize;
  kstride = subRange.krange.stride;
}

Stripe3Indices::Stripe3Indices(const Range3Indices& rhs)
{
  isize=rhs.irange.size;
  jsize=rhs.jrange.size;
  ksize=rhs.krange.size;
  kstride=rhs.krange.stride;
  jstride=kstride*rhs.krange.size*rhs.jrange.stride;
  istride=jstride*rhs.jrange.size*rhs.irange.stride;
  //count from zero
  base=rhs.krange.start*kstride+rhs.jrange.start*jstride+rhs.irange.start*istride;
}

Stripe3Indices::Stripe3Indices(const Stripe3Indices& rhs)
  :base(rhs.base),isize(rhs.isize),jsize(rhs.jsize),ksize(rhs.ksize),istride(rhs.istride),jstride(rhs.jstride),kstride(rhs.kstride)
{}

Stripe3Indices::Stripe3Indices(int _isize,int _jsize,int _ksize,int _base,int _istride,int _jstride,int _kstride)
  :base(_base),isize(_isize),jsize(_jsize),ksize(_ksize),istride(_istride*_jstride*_jsize*_ksize*_kstride),jstride(_jstride*_ksize*_kstride),kstride(_kstride)
{}

void Stripe3Indices::enumerate(std::vector<int>& indices)
{
  indices.resize(isize*jsize*ksize);
  int index=base;
  int count=0;
  for(int i=0;i<isize;i++) {
    int ki=index;
    for(int j=0;j<jsize;j++) {
      int kj=index;
      for(int k=0;k<ksize;k++,count++) {
	indices[count] = index;
	index += kstride;
      }
      index = kj+jstride;
    }
    index = ki+istride;
  }
}

bool Stripe3Indices::contains(int index) const
{
  div_t di = div(index-base,istride);
  div_t dj = div(index-base,jstride);
  div_t dk = div(index-base,kstride);
  if(di.rem != 0 || dj.rem != 0 || dk.rem != 0) return false;
  if(di.quot < 0 || di.quot >= isize) return false;
  if(dj.quot < 0 || dj.quot >= jsize) return false;
  if(dk.quot < 0 || dk.quot >= ksize) return false;
  return true;
}

IntTriple Stripe3Indices::indexToElement(int index) const
{
  div_t di = div(index-base,istride);
  div_t dj = div(index-base,jstride);
  div_t dk = div(index-base,kstride);
  return IntTriple(di.quot,dj.quot,dk.quot);
}

void Stripe3Indices::setRange(const Stripe3Indices& stripe,const Range3Indices& inds)
{
  Assert(this != &stripe);
  base = stripe.base + inds.irange.start*stripe.istride + inds.jrange.start*stripe.jstride + inds.krange.start*stripe.kstride;
  isize = inds.irange.size;
  jsize = inds.jrange.size;
  ksize = inds.krange.size;
  istride = inds.irange.stride*stripe.istride;
  jstride = inds.jrange.stride*stripe.jstride;
  kstride = inds.krange.stride*stripe.kstride;
}

bool Stripe3Indices::operator == (const Stripe3Indices& stripe) const
{
  if(this == &stripe) return true;
  return base==stripe.base &&
    isize==stripe.isize && jsize==stripe.jsize && ksize==stripe.ksize &&
    istride==stripe.istride && jstride==stripe.jstride && kstride==stripe.kstride;
}

Stripe3Indices::iterator::iterator(const Stripe3Indices* _stripe)
  :stripe(_stripe),i(0),j(0),k(0),index(_stripe->base),firstIndex(_stripe->base),secondIndex(_stripe->base)
{
}

Stripe3Indices::iterator::iterator(const Stripe3Indices* _stripe,int invalid)
  :stripe(_stripe),i(_stripe->isize),j(0),k(0),index(stripe->base+_stripe->isize*_stripe->istride),firstIndex(_stripe->base+_stripe->isize*_stripe->istride),secondIndex(_stripe->base+_stripe->isize*_stripe->istride)
{
}

Stripe3Indices::iterator::iterator(const iterator& it)
  :stripe(it.stripe),i(it.i),j(it.j),k(it.k),index(it.index),firstIndex(it.firstIndex),secondIndex(it.secondIndex)
{
}

Stripe3Indices::iterator& Stripe3Indices::iterator::operator ++()
{
  index+=stripe->kstride;
  k++;
  if(k >= stripe->ksize) {
    k=0;
    j++;
    secondIndex+=stripe->jstride;
    index=secondIndex;
    if(j >= stripe->jsize) {
      j=0;
      i++;
      firstIndex+=stripe->istride;
      index=secondIndex=firstIndex;
    }
  }
  return *this;
}

Stripe3Indices::iterator& Stripe3Indices::iterator::operator --()
{
  index-=stripe->kstride;
  k--;
  if(k < 0) {
    k=stripe->ksize-1;
    j--;
    secondIndex-=stripe->jstride;
    index=secondIndex;
    if(j < 0) {
      j=stripe->jsize-1;
      i--;
      firstIndex-=stripe->istride;
      index=secondIndex=firstIndex;
    }
  }
  return *this;
}

Stripe3Indices::iterator& Stripe3Indices::iterator::operator +=(int skip)
{
  Assert(skip >= 0);
  index+=stripe->kstride*skip;
  k+=skip;
  if(k >= stripe->ksize) {
    div_t d=div(k,stripe->ksize);
    k=d.rem;
    j+=d.quot;
    secondIndex+=stripe->jstride*d.quot;
    index=secondIndex+k*stripe->kstride;
    if(j >= stripe->jsize) {
      d=div(j,stripe->jsize);
      j=d.rem;
      i+=d.quot;
      firstIndex+=stripe->istride*d.quot;
      secondIndex=firstIndex+j*stripe->jstride;
      index=secondIndex+k*stripe->kstride;
    }
  }
  return *this;
}

Stripe3Indices::iterator& Stripe3Indices::iterator::operator -=(int skip)
{
  Assert(skip >= 0);
  index-=stripe->kstride*skip;
  k-=skip;
  if(k < 0) {
    div_t d=div(k,stripe->ksize);  //rounds twoard zero
    k=(d.rem+stripe->ksize);
    j+=(d.quot-1);
    secondIndex+=stripe->jstride*d.quot;
    index=secondIndex+k*stripe->kstride;
    if(j >= stripe->jsize) {
      d=div(j,stripe->jsize);  //rounds toward zero
      j=(d.rem+stripe->jsize);
      i+=(d.quot-1);
      firstIndex+=stripe->istride*d.quot;
      secondIndex=firstIndex+j*stripe->jstride;
      index=secondIndex+k*stripe->kstride;
    }
  }
  return *this;
}

void Stripe3Indices::iterator::incFirst(int skip)
{
  i += skip;
  firstIndex+=skip*stripe->istride;
  index+=skip*stripe->istride;
}

void Stripe3Indices::iterator::incSecond(int skip)
{
  j += skip;
  secondIndex += skip*stripe->jstride;
  index += skip*stripe->jstride;
}

void Stripe3Indices::iterator::incThird(int skip)
{
  k += skip;
  index += skip*stripe->kstride;
}

bool Stripe3Indices::iterator::isInvalid() const
{
  return i<0 || i>=stripe->isize ||
    j<0 || j>=stripe->jsize ||
    k<0 || k>=stripe->ksize;
}

bool Stripe3Indices::iterator::operator == (const iterator& rhs) const
{
  if(*stripe != *rhs.stripe) return false;
  if(isInvalid()) return rhs.isInvalid();
  return index == rhs.index;
}


bool Stripe3Indices::iterator::operator < (const iterator& rhs) const
{
  if(stripe != rhs.stripe) return false;
  return index < rhs.index;
}







Grid2Indices::iterator::iterator(const Grid2Indices* _grid)
  :grid(_grid),range(grid->iindices.size(),grid->jindices.size()),e(range.begin())
{}

Grid2Indices::iterator::iterator(const Grid2Indices* _grid,int invalid)
  :grid(_grid),range(grid->iindices.size(),grid->jindices.size()),e(range.end())
{}

Grid2Indices::iterator::iterator(const Grid2Indices* _grid,const Range2Indices& _range)
  :grid(_grid),range(_range),e(range.begin())
{
  Assert(range.irange.start >= 0 && range.irange.start+range.irange.stride*range.irange.size <= (int)grid->iindices.size());
  Assert(range.jrange.start >= 0 && range.jrange.start+range.jrange.stride*range.jrange.size <= (int)grid->jindices.size());
}

Grid2Indices::iterator::iterator(const Grid2Indices* _grid,const Range2Indices& _range,int invalid)
  :grid(_grid),range(_range),e(range.end())
{
  Assert(range.irange.start >= 0 && range.irange.start+range.irange.stride*range.irange.size <= (int)grid->iindices.size());
  Assert(range.jrange.start >= 0 && range.jrange.start+range.jrange.stride*range.jrange.size <= (int)grid->jindices.size());
}



