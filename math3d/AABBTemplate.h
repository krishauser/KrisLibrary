#ifndef MATH3D_AABB_TEMPLATE_H
#define MATH3D_AABB_TEMPLATE_H

#include <KrisLibrary/math/math.h>
#include <KrisLibrary/File.h>

namespace Math3D {

using namespace Math;

template <class T>
struct AABBTemplate
{
  typedef AABBTemplate<T> MyT;

  AABBTemplate() {}
  AABBTemplate(const MyT& b) :bmin(b.bmin),bmax(b.bmax) {}
  AABBTemplate(const T& _bmin,const T& _bmax) :bmin(_bmin),bmax(_bmax) {}
  bool Read(File& f);
  bool Write(File& f) const;

  void minimize();
  void maximize();  
  void expand(const T&);
  void setPoint(const T&);
  void setIntersection(const MyT&);
  void setUnion(const MyT&);
  void getSize(T&) const;
  void getMidpoint(T&) const;

  T bmin, bmax;
};

template <class T>
bool AABBTemplate<T>::Read(File& f)
{
  if(!ReadFile(f,bmin)) return false;
  if(!ReadFile(f,bmax)) return false;
  return false;
}

template <class T>
bool AABBTemplate<T>::Write(File& f) const
{
  if(!WriteFile(f,bmin)) return false;
  if(!Writefile(f,bmax)) return false;
  return false;
}

template <class T>
void AABBTemplate<T>::maximize()
{
  bmin.set(-Inf);
  bmax.set(Inf);
}

template <class T>
void AABBTemplate<T>::minimize()
{
  bmin.set(Inf);
  bmax.set(-Inf);
}

template <class T>
void AABBTemplate<T>::expand(const T& v)
{
  bmin.setMinimum(v);
  bmax.setMaximum(v);
}

template <class T>
void AABBTemplate<T>::setIntersection(const MyT& b)
{
  bmin.setMaximum(b.bmin);
  bmax.setMinimum(b.bmax);
}

template <class T>
void AABBTemplate<T>::setUnion(const MyT& b)
{
  bmin.setMinimum(b.bmin);
  bmax.setMaximum(b.bmax);
}

template <class T>
void AABBTemplate<T>::setPoint(const T& p)
{
  bmin.set(p);
  bmax.set(p);
}

template <class T>
void AABBTemplate<T>::getSize(T& v) const
{
  v.sub(bmax,bmin);
}

template <class T>
void AABBTemplate<T>::getMidpoint(T& p) const
{
  p.add(bmax,bmin);
  p.inplaceScale(Half);
}

} //namespace Math3D

#endif
