#include "CSpace.h"
#include <math/random.h>
using namespace std;

void CSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(c.n);
  for(int i=0;i<c.n;i++)
    x(i) = c(i) + Rand(-r,r);
}

void CSpace::Interpolate(const Config& x, const Config& y, Real u, Config& out)
{
  out.mul(x,One-u);
  out.madd(y,u);
}

void CSpace::Midpoint(const Config& x, const Config& y, Config& out)
{
  out.add(x,y);
  out.inplaceMul(Half);
}


