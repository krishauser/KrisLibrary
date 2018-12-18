#include "random.h"

namespace Math3D {

/*
void RandRotation(Quaternion& q)
{
  const int maxIters = 20; //small chance of not being uniformly distributed
  int iters=0;
  Real n;
  do {
    q.w = Rand(-1,1);
    q.x = Rand(-1,1);
    q.y = Rand(-1,1);
    q.z = Rand(-1,1);
    n=q.normSquared();
    iters++; if(iters > maxIters) break;
  } while(n > One && n < Epsilon);
  q /= Sqrt(n);
}
*/

void RandRotation(Quaternion& q)
{
  const int maxIters = 20; //small chance of not being uniformly distributed
  int iters=0;
  Real x2y2,w2z2;
  do {
    q.w = Rand(-1,1);
    q.x = Rand(-1,1);
    q.y = Rand(-1,1);
    q.z = Rand(-1,1);
    x2y2=Sqr(q.x)+Sqr(q.y);
    w2z2=Sqr(q.w)+Sqr(q.z);
    iters++; if(iters > maxIters) break;
  } while(x2y2 > One || w2z2 > One);
  Real u=Sqrt((One-x2y2)/w2z2);
  q.z*=u;
  q.w*=u;
}


} //namespace Math3D
