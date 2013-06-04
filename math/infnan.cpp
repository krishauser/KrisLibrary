#include "infnan.h"
#include "math.h"
#include <stdio.h>
#include <math.h>

namespace Math {

int IsNaN(double x)
{
#ifdef _MSC_VER
  return _isnan(x);
#elif HAVE_DECL_ISNAN
  return isnan(x);
#elif HAVE_IEEE_COMPARISONS
  return (x!=x?1:0);
#else
  #error "IsNaN: Neither Microsoft's _isnan, isnan, or IEEE comparisons defined"
  return 0;
#endif
}

int IsFinite(double x)
{
#ifdef _MSC_VER
  return _finite(x);
#elif HAVE_DECL_ISFINITE
  return isfinite(x);
#elif HAVE_DECL_FINITE
  return finite(x);
#elif HAVE_IEEE_COMPARISONS
  double y=x-x;
  return (y==y?1:0);
#else
  #error "IsFinite: Neither Microsoft's _isfinite, isfinite, or IEEE comparisons defined"
  return 0;
#endif
}

int IsInf(double x)
{
#ifdef _MSC_VER  //doesn't have isinf
  int cls = _fpclass(x);
  if(cls == _FPCLASS_PINF) return 1;
  else if(cls == _FPCLASS_NINF) return -1;
  else return 0;
#elif HAVE_DECL_ISINF
  if(isinf(x)) {
    if(x > 0) return 1;
    else return -1;
  }
  else return 0;
#elif HAVE_IEEE_COMPARISONS
  double y=x-x;
  if(IsNaN(y)) 
    return (x>0?1:-1);
  else return 0;
#else
  #error "IsInf: Neither Microsoft's _fpclass, isinf, or IEEE comparisons defined"
  return 0;
#endif
}



int IsNaN(float x)
{
#ifdef _MSC_VER
  return _isnan(x);
#elif HAVE_DECL_ISNAN
  //return isnanf(x);
  return isnan(x);
#elif HAVE_IEEE_COMPARISONS
  return (x!=x?1:0);
#else
  #error "IsNaN: Neither Microsoft's _isnan, isnan, or IEEE comparisons defined"
  return 0;
#endif
}

int IsFinite(float x)
{
#ifdef _MSC_VER
  return _finite(x);
#elif HAVE_DECL_FINITE
  return finitef(x);
#elif HAVE_DECL_ISFINITE
  return isfinitef(x);
#elif HAVE_IEEE_COMPARISONS
  float y=x-x;
  return (y==y?1:0);
#else
  #error "IsFinite: Neither Microsoft's _isfinite, isfinite, or IEEE comparisons defined"
  return 0;
#endif
}

int IsInf(float x)
{
#ifdef _MSC_VER  //doesn't have isinf
  int cls = _fpclass(x);
  if(cls == _FPCLASS_PINF) return 1;
  else if(cls == _FPCLASS_NINF) return -1;
  else return 0;
#elif HAVE_DECL_ISINF
  //if(isinff(x)) {
  if(isinf(x)) {
    if(x > 0) return 1;
    else return -1;
  }
  else return 0;
#elif HAVE_IEEE_COMPARISONS
  float y=x-x;
  if(IsNaN(y)) 
    return (x>0?1:-1);
  else return 0;
#else
  #error "IsInf: Neither Microsoft's _fpclass, isinf, or IEEE comparisons defined"
  return 0;
#endif
}

} //namespace Math
