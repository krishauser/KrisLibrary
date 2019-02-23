#include <KrisLibrary/Logger.h>
#include "Rasterize.h"
#include "Meshing.h"
#include <math/cast.h>
#include <geometry/primitives.h>
#include <utils/shift.h>
#include <errors.h>
using namespace Geometry;
using namespace std;

namespace Meshing { 

struct BarycentricCoords
{
  Vector3 a,b,c;
};

//crop to the positive side of plane p, adds the resulting triangles to res
void CropTriangles(vector<Triangle2D>& tris,const Plane2D& p)
{
  Vector2 newPts[2];
  IntTriple newTris[3];
  bool triPositive[3];
  size_t n=tris.size();
  for(size_t i=0;i<n;i++) {
    int nt=SplitTriangle(tris[i],p,newPts,newTris,triPositive,1e-5);
    const Vector2* p[5]={&tris[i].a,&tris[i].b,&tris[i].c,&newPts[0],&newPts[1]};
    for(int k=0;k<nt;k++)
      if(triPositive[k]) {
	Triangle2D temp;
	temp.a = *p[newTris[k][0]];
	temp.b = *p[newTris[k][1]];
	temp.c = *p[newTris[k][2]];
	tris.push_back(temp);
      }
  }
  tris.erase(tris.begin(),tris.begin()+n);
}


void Clip(const AABB2D& aabb,std::vector<Triangle2D>& tris,std::vector<BarycentricCoords>& bary)
{
  Plane2D p;
  p.normal.set(1,0); p.offset = aabb.bmin.x;
  CropTriangles(tris,p);
  p.normal.set(-1,0); p.offset = -aabb.bmax.x;
  CropTriangles(tris,p);
  p.normal.set(0,1); p.offset = aabb.bmin.y;
  CropTriangles(tris,p);
  p.normal.set(0,-1); p.offset = -aabb.bmax.y;
  CropTriangles(tris,p);
}

void GetSegmentCells(const Segment2D& s,vector<IntPair>& cells)
{
  cells.resize(0);
  Vector2 d=s.b-s.a;
  IntPair i((int)iFloor(s.a.x),(int)iFloor(s.a.y));

  Vector2 cellCorner(i.a,i.b);
  Real invdx = 1.0/d.x;
  Real invdy = 1.0/d.y;
  Real param=0;  //goes from 0 to 1
  while(param < 1.0) {
    cells.push_back(i);
    //see which cell face is hit next
    int closest=0;  //1=+x,-1=-x,2=+y,-2=-y,3=+z,-3=-z
    param=Inf;
    if(d.x > 0) { //check +x face
      param = ((cellCorner.x+1.0)-s.a.x)*invdx;
      closest = 1;
    }
    else if(d.x < 0) { //check -x face
      param = (cellCorner.x-s.a.x)*invdx;
      closest = -1;
    }
    if(d.y > 0) { //check +y face
      if((cellCorner.y+1.0)-s.a.y < param*d.y) {
	param = ((cellCorner.y+1.0)-s.a.y)*invdy;
	closest = 2;
      }
    }
    else if(d.y < 0) { //check -y face
      if(cellCorner.y-s.a.y > param*d.y) {
	param = (cellCorner.y-s.a.y)*invdy;
	closest = -2;
      }
    }
    switch(closest) {
    case 0: param = 1; break;
    case 1: i.a++; cellCorner.x+=1.0; break;
    case -1: i.a--; cellCorner.x-=1.0; break;
    case 2: i.b++; cellCorner.y+=1.0; break;
    case -2: i.b--; cellCorner.y-=1.0; break;
    }
  }
}

void GetTriangleCells_Clipped(const Triangle2D& torig,std::vector<IntPair>& cells,int imin,int jmin,int imax,int jmax)
{
  Vector2 a=torig.a,b=torig.b,c=torig.c;
  //rotate so that least lexicographic is in position 0
  if(Lexical2DOrder(b,a)) {
    if(Lexical2DOrder(c,b)) {  //c<b<a
      //set c to the front
      ShiftForward(a,b,c);
    }
    else {  //b<c, b<a
      ShiftBackward(a,b,c);
    }
  }
  else {
    if(Lexical2DOrder(c,a)) {
      ShiftForward(a,b,c);
    }
  }
  //find if the long edge of the triangle is up or down (because of ccw ordering, if vertex 1 < vertex 2, it's up)
  //if it's up, the long end is a--c, otherwise it's a--b
  bool long_end_up = Lexical2DOrder(b,c);
  if(long_end_up) {
    swap(b,c);
  }
  if(torig.orientation() < 0) long_end_up = !long_end_up;

  /********************************************************
   * now we have this situation (or a vertically mirrored image)
   *          o(c)
   *
   *  o(a)
   *              o(b)
   ********************************************************/
  /********************************************************
   * Step 1: go down the x direction of the long edge until you hit c.x,
   * filling edge from a--b up to a--c.
   *  |       o(c)
   *  |  .....|
   *  o(a)....|
   *  |       |   o(b)
   ********************************************************/
  /********************************************************
   * Step 2: go down the x direction of the long edge until you hit v[1].x,
   * filling edge from v[0]--v[1] up to v[2]--v[1].
   *          o(c)|
   *     .....|.  |
   *  o(a)....|.. |
   *          |...o(b)
   ********************************************************/
  //the y-coordinate of a segment x1,y1 to x2,y2, at an x coordinate x
  //is ((x2-x)*y1 + (x-x1)*y2)/(x2-x1) = x*(y2-y1)/(x2-x1)+(x2*y1-x1*y2)/(x2-x1) = mx+b
  Real den1 = b.x-a.x;
  Real den2 = c.x-a.x;
  Real den3 = b.x-c.x;
  Assert(den1 >= 0);
  Assert(den2 >= 0);
  Assert(den3 >= 0);
  Real m1,m2,m3,b1,b2,b3;
  //segments are y=m1*x+b1, y=m2*x+b2, y=m3*x+b3
  //as x sweeps down the triangle, the segment interpolation parameters are given as
  //  u1 = (x-v[0].x)/den1, u2 = (x-v[0].x)/den2, u3 = (x-v[2].x)/den3
  //so barycentric coordinates are
  //  b = (1-u1)*b[0] + u1*b[1], b = (1-u2)*b[0] + u2*b[2], b=(1-u3)*b[2]-u3*b[1]
  //which can be converted into mx+b representation like the above
  if(den1==0) { //the long segment is vertical! 
    Segment2D s;
    s.a.x = s.b.x = a.x;
    s.a.y = Min(Min(a.y,b.y),c.y);
    s.b.y = Max(Max(a.y,b.y),c.y);
    GetSegmentCells(s,cells);
    for(size_t i=0;i<cells.size();i++) {
      if(cells[i].a < imin || cells[i].a >= imax ||
	 cells[i].b < jmin || cells[i].b >= jmax) {
	cells.erase(cells.begin()+i);
	i--;
      }
    }
    return;
  }
  Real x = a.x;
  Real xnext = Ceil(x);
  int ix = (int)iFloor(x);
  m1=(b.y-a.y)/den1;
  b1=(b.x*a.y - a.x*b.y)/den1;
  if(ix < imin) {
    ix = imin;
    x = imin;
    xnext = x+1.0;
  }
  if(ix >= imax) return;
  if(den2>0) { //step 1
    m2=(c.y-a.y)/den2;
    b2=(c.x*a.y - a.x*c.y)/den2;
    Real y1 = a.y,y2=a.y;
    Real z1 = m1*xnext+b1,z2=m2*xnext+b2;
    while(x < c.x) {
      if(xnext > b.x) {
	z1 = b.y;
      }
      if(xnext > c.x) {
	z2 = c.y;
      }

      //fill the segment (x,y1)->(x,y2)
      int iymin,iymax;
      if(long_end_up) {
	if(y1 < y2-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<a<<"->"<<b<<", "<<a<<"->"<<c<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y1 = "<<y1<<", y2 = "<<y2);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y2,z2));
	iymax = (int)iFloor(Max(y1,z1));
      }
      else {
	if(y2 < y1-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<a<<"->"<<b<<", "<<a<<"->"<<c<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y2 = "<<y2<<", y1 = "<<y1);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y1,z1));
	iymax = (int)iFloor(Max(y2,z2));
      }
      iymin = Max(iymin,jmin);
      iymax = Min(iymax,jmax-1);
      for(int iy=iymin;iy<=iymax;iy++)
	cells.push_back(IntPair(ix,iy));

      x = xnext;
      xnext += 1.0;
      ix ++;
      y1 = z1;
      y2 = z2;
      z1 += m1;
      z2 += m2;
      if(ix == imax) return;
    }
  }
  //now x >= c.x
  if(den3>0) { //step 2
    m3=(b.y-c.y)/den3;
    b3=(b.x*c.y - c.x*b.y)/den3;
    Real y1 = m1*x+b1,y3=m3*x+b3;
    Real z1 = m1*xnext+b1,z3=m3*xnext+b3;
    while(x < b.x) {
      if(xnext > b.x) {
	z1 = z3 = b.y;
      }

      //fill the segment (x,y1)->(x,y3)
      int iymin,iymax;
      if(long_end_up) {
	if(y1 < y3-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<a<<"->"<<b<<", "<<c<<"->"<<b<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y1 = "<<y1<<", y3 = "<<y3);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y3,z3));
	iymax = (int)iFloor(Max(y1,z1));
      }
      else {
	if(y3 < y1-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<a<<"->"<<b<<", "<<c<<"->"<<b<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y3 = "<<y3<<", y1 = "<<y1);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y1,z1));
	iymax = (int)iFloor(Max(y3,z3));
      }
      iymin = Max(iymin,jmin);
      iymax = Min(iymax,jmax-1);
      for(int iy=iymin;iy<=iymax;iy++)
	cells.push_back(IntPair(ix,iy));
      x = xnext;
      xnext += 1.0;
      ix++;
      y1 = z1;
      y3 = z3;
      z1 += m1;
      z3 += m3;
      if(ix == imax) return;
    }
  }
  /*
  for(size_t i=0;i<cells.size();i++) {
    Assert(cells[i].a >= imin && cells[i].a < imax);
    Assert(cells[i].b >= jmin && cells[i].b < jmax);
  }
  */
}


void GetTriangleCells(const Triangle2D& t,vector<IntPair>& cells)
{
  const Vector2* v[3]={&t.a,&t.b,&t.c};
  //rotate so that least lexicographic is in position 0
  if(Lexical2DOrder(t.b,t.a)) {
    if(Lexical2DOrder(t.c,t.b)) {  //c<b<a
      //set c to the front
      ShiftForward(v[0],v[1],v[2]);
    }
    else {  //b<c, b<a
      ShiftBackward(v[0],v[1],v[2]);
    }
  }
  else {
    if(Lexical2DOrder(t.c,t.a)) {
      ShiftForward(v[0],v[1],v[2]);
    }
  }
  //find if the long edge of the triangle is up or down (because of ccw ordering, if vertex 1 < vertex 2, it's up)
  //if it's up, the long end is v[0]--v[2], otherwise it's v[0]--v[1]
  bool long_end_up = Lexical2DOrder(*v[1],*v[2]);
  if(long_end_up) {
    swap(v[1],v[2]);
  }
  if(t.orientation() < 0) long_end_up = !long_end_up;

  /********************************************************
   * now we have this situation (or a vertically mirrored image)
   *          o(2)
   *
   *  o(0)
   *              o(1)
   ********************************************************/
  /********************************************************
   * Step 1: go down the x direction of the long edge until you hit v[2].x,
   * filling edge from v[0]--v[1] up to v[0]--v[2].
   *  |       o(2)
   *  |  .....|
   *  o(0)....|
   *  |       |   o(1)
   ********************************************************/
  /********************************************************
   * Step 2: go down the x direction of the long edge until you hit v[1].x,
   * filling edge from v[0]--v[1] up to v[2]--v[1].
   *          o(2)|
   *     .....|.  |
   *  o(0)....|.. |
   *          |...o(1)
   ********************************************************/
  //the y-coordinate of a segment x1,y1 to x2,y2, at an x coordinate x
  //is ((x2-x)*y1 + (x-x1)*y2)/(x2-x1) = x*(y2-y1)/(x2-x1)+(x2*y1-x1*y2)/(x2-x1) = mx+b
  Real den1 = v[1]->x-v[0]->x;
  Real den2 = v[2]->x-v[0]->x;
  Real den3 = v[1]->x-v[2]->x;
  Assert(den1 >= 0);
  Assert(den2 >= 0);
  Assert(den3 >= 0);
  Real m1,m2,m3,b1,b2,b3;
  //segments are y=m1*x+b1, y=m2*x+b2, y=m3*x+b3
  //as x sweeps down the triangle, the segment interpolation parameters are given as
  //  u1 = (x-v[0].x)/den1, u2 = (x-v[0].x)/den2, u3 = (x-v[2].x)/den3
  //so barycentric coordinates are
  //  b = (1-u1)*b[0] + u1*b[1], b = (1-u2)*b[0] + u2*b[2], b=(1-u3)*b[2]-u3*b[1]
  //which can be converted into mx+b representation like the above
  if(den1==0) { //the long segment is vertical! 
    Segment2D s;
    s.a.x = s.b.x = t.a.x;
    s.a.y = Min(Min(t.a.y,t.b.y),t.c.y);
    s.b.y = Max(Max(t.a.y,t.b.y),t.c.y);
    GetSegmentCells(s,cells);
    return;
  }
  Real x = v[0]->x;
  Real xnext = Ceil(x);
  int ix = (int)iFloor(x);
  m1=(v[1]->y-v[0]->y)/den1;
  b1=(v[1]->x*v[0]->y - v[0]->x*v[1]->y)/den1;
  if(den2>0) { //step 1
    m2=(v[2]->y-v[0]->y)/den2;
    b2=(v[2]->x*v[0]->y - v[0]->x*v[2]->y)/den2;
    Real y1 = v[0]->y,y2=v[0]->y;
    Real z1 = m1*xnext+b1,z2=m2*xnext+b2;
    while(x < v[2]->x) {
      if(xnext > v[1]->x) {
	z1 = v[1]->y;
      }
      if(xnext > v[2]->x) {
	z2 = v[2]->y;
      }

      //fill the segment (x,y1)->(x,y2)
      int iymin,iymax;
      if(long_end_up) {
	if(y1 < y2-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[0]<<"->"<<*v[2]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y1 = "<<y1<<", y2 = "<<y2);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y2,z2));
	iymax = (int)iFloor(Max(y1,z1));
      }
      else {
	if(y2 < y1-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[0]<<"->"<<*v[2]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y2 = "<<y2<<", y1 = "<<y1);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y1,z1));
	iymax = (int)iFloor(Max(y2,z2));
      }
      for(int iy=iymin;iy<=iymax;iy++)
	cells.push_back(IntPair(ix,iy));

      x = xnext;
      xnext += 1.0;
      ix ++;
      y1 = z1;
      y2 = z2;
      z1 += m1;
      z2 += m2;
    }
  }
  //now x >= v[2]->x
  if(den3>0) { //step 2
    m3=(v[1]->y-v[2]->y)/den3;
    b3=(v[1]->x*v[2]->y - v[2]->x*v[1]->y)/den3;
    Real y1 = m1*x+b1,y3=m3*x+b3;
    Real z1 = m1*xnext+b1,z3=m3*xnext+b3;
    while(x < v[1]->x) {
      if(xnext > v[1]->x) {
	z1 = z3 = v[1]->y;
      }

      //fill the segment (x,y1)->(x,y3)
      int iymin,iymax;
      if(long_end_up) {
	if(y1 < y3-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[2]<<"->"<<*v[1]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y1 = "<<y1<<", y3 = "<<y3);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y3,z3));
	iymax = (int)iFloor(Max(y1,z1));
      }
      else {
	if(y3 < y1-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[2]<<"->"<<*v[1]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y3 = "<<y3<<", y1 = "<<y1);
	  KrisLibrary::loggerWait();
	}
	iymin = (int)iFloor(Min(y1,z1));
	iymax = (int)iFloor(Max(y3,z3));
      }
      for(int iy=iymin;iy<=iymax;iy++)
	cells.push_back(IntPair(ix,iy));
      x = xnext;
      xnext += 1.0;
      ix++;
      y1 = z1;
      y3 = z3;
      z1 += m1;
      z3 += m3;
    }
  }
}





void Rasterizer2D::Rasterize(const Triangle2D& t)
{
  Vector3 ba(1,0,0),bb(0,1,0),bc(0,0,1);
  Rasterize(t,ba,bb,bc);
}

void Rasterizer2D::Rasterize(const Triangle2D& t,const Vector3& ba,const Vector3& bb,const Vector3& bc)
{
  //LOG4CXX_INFO(KrisLibrary::logger(),"Rasterizing triangle "<<t.a<<", "<<t.b<<", "<<t.c<<"...");
  const Vector2* v[3]={&t.a,&t.b,&t.c};
  const Vector3* b[3]={&ba,&bb,&bc};
  //rotate so that least lexicographic is in position 0
  if(Lexical2DOrder(t.b,t.a)) {
    if(Lexical2DOrder(t.c,t.b)) {  //c<b<a
      //set c to the front
      ShiftForward(v[0],v[1],v[2]);
      ShiftForward(b[0],b[1],b[2]);
    }
    else {  //b<c, b<a
      ShiftBackward(v[0],v[1],v[2]);
      ShiftBackward(b[0],b[1],b[2]);
    }
  }
  else {
    if(Lexical2DOrder(t.c,t.a)) {
      ShiftForward(v[0],v[1],v[2]);
      ShiftForward(b[0],b[1],b[2]);
    }
  }
  //find if the long edge of the triangle is up or down (because of ccw ordering, if vertex 1 < vertex 2, it's up)
  //if it's up, the long end is v[0]--v[2], otherwise it's v[0]--v[1]
  bool long_end_up = Lexical2DOrder(*v[1],*v[2]);
  if(long_end_up) {
    swap(v[1],v[2]);
    swap(b[1],b[2]);
  }
  if(t.orientation() < 0) long_end_up = !long_end_up;

  /********************************************************
   * now we have this situation (or a vertically mirrored image)
   *          o(2)
   *
   *  o(0)
   *              o(1)
   ********************************************************/
  /********************************************************
   * Step 1: go down the x direction of the long edge until you hit v[2].x,
   * filling edge from v[0]--v[1] up to v[0]--v[2].
   *  |       o(2)
   *  |  .....|
   *  o(0)....|
   *  |       |   o(1)
   ********************************************************/
  /********************************************************
   * Step 2: go down the x direction of the long edge until you hit v[1].x,
   * filling edge from v[0]--v[1] up to v[2]--v[1].
   *          o(2)|
   *     .....|.  |
   *  o(0)....|.. |
   *          |...o(1)
   ********************************************************/
  //the y-coordinate of a segment x1,y1 to x2,y2, at an x coordinate x
  //is ((x2-x)*y1 + (x-x1)*y2)/(x2-x1) = x*(y2-y1)/(x2-x1)+(x2*y1-x1*y2)/(x2-x1) = mx+b
  Real den1 = v[1]->x-v[0]->x;
  Real den2 = v[2]->x-v[0]->x;
  Real den3 = v[1]->x-v[2]->x;
  Assert(den1 >= 0);
  Assert(den2 >= 0);
  Assert(den3 >= 0);
  Real m1,m2,m3,b1,b2,b3;
  //segments are y=m1*x+b1, y=m2*x+b2, y=m3*x+b3
  //as x sweeps down the triangle, the segment interpolation parameters are given as
  //  u1 = (x-v[0].x)/den1, u2 = (x-v[0].x)/den2, u3 = (x-v[2].x)/den3
  //so barycentric coordinates are
  //  b = (1-u1)*b[0] + u1*b[1], b = (1-u2)*b[0] + u2*b[2], b=(1-u3)*b[2]-u3*b[1]
  //which can be converted into mx+b representation like the above
  Vector3 bm1,bm2,bm3,bb1,bb2,bb3;
  if(den1==0) { //the long segment is vertical! 
    return;
  }
  Real x = Ceil(v[0]->x);
  int ix = (int)iRound(x);
  m1=(v[1]->y-v[0]->y)/den1;
  b1=(v[1]->x*v[0]->y - v[0]->x*v[1]->y)/den1;
  bm1=(*b[1]-*b[0])/den1;
  bb1=(v[1]->x*(*b[0]) - v[0]->x*(*b[1]))/den1;
  if(den2>Epsilon) { //step 1
    m2=(v[2]->y-v[0]->y)/den2;
    b2=(v[2]->x*v[0]->y - v[0]->x*v[2]->y)/den2;
    bm2=(*b[2]-*b[0])/den2;
    bb2=(v[2]->x*(*b[0]) - v[0]->x*(*b[2]))/den2;
    Real y1 = m1*x+b1,y2=m2*x+b2;
    Vector3 bary1 = bm1*x+bb1,bary2=bm2*x+bb2;
    while(x < v[2]->x) {
      //fill the segment (x,y1)->(x,y2)
      if(long_end_up) {
	if(y1 < y2-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[0]<<"->"<<*v[2]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y1 = "<<y1<<", y2 = "<<y2);
	  KrisLibrary::loggerWait();
	}
	RasterizeVerticalSegment(ix,y2,y1,bary2,bary1);
      }
      else {
	if(y2 < y1-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[0]<<"->"<<*v[2]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y2 = "<<y2<<", y1 = "<<y1);
	  KrisLibrary::loggerWait();
	}
	RasterizeVerticalSegment(ix,y1,y2,bary1,bary2);
      }
      x += One;
      ix++;
      y1 += m1;
      y2 += m2;
      bary1 += bm1;
      bary2 += bm2;
    }
  }
  //now x >= v[2]->x
  if(den3>Epsilon) { //step 2
    m3=(v[1]->y-v[2]->y)/den3;
    b3=(v[1]->x*v[2]->y - v[2]->x*v[1]->y)/den3;
    bm3=(*b[1]-*b[2])/den3;
    bb3=(v[1]->x*(*b[2]) - v[2]->x*(*b[1]))/den3;
    Real y1 = m1*x+b1,y3=m3*x+b3;
    Vector3 bary1 = bm1*x+bb1,bary3=bm3*x+bb3;
    while(x < v[1]->x) {
      //fill the segment (x,y1)->(x,y3)
      if(long_end_up) {
	if(y1 < y3-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[2]<<"->"<<*v[1]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y1 = "<<y1<<", y3 = "<<y3);
	  KrisLibrary::loggerWait();
	}
	RasterizeVerticalSegment(ix,y3,y1,bary3,bary1);
      }
      else {
	if(y3 < y1-1e-3) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Rasterizer2D: Warning, top segment passed below bottom segment?");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Segments "<<*v[0]<<"->"<<*v[1]<<", "<<*v[2]<<"->"<<*v[1]<<", x="<<x);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"y3 = "<<y3<<", y1 = "<<y1);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Den3 = "<<den3);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"m3 = "<<m3);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"b3 = "<<b3);
	  KrisLibrary::loggerWait();
	}
	RasterizeVerticalSegment(ix,y1,y3,bary1,bary3);
      }
      x += One;
      ix++;
      y1 += m1;
      y3 += m3;
      bary1 += bm1;
      bary3 += bm3;
    }
  }
}

void Rasterizer2D::RasterizeVerticalSegment(int x,Real y1,Real y2,const Vector3& baryA,const Vector3& baryB)
{
  if(y2 == y1) return;
  Real y=Ceil(y1);
  int iy=(int)iRound(y);
  Real u=(y-y1)/(y2-y1);
  Vector3 bary = (One-u)*baryA + u*baryB;
  Vector3 baryInc=baryB-baryA;
  baryInc /= y2-y1;
  while(y<y2) {
    VisitCell(bary,x,iy);
    bary += baryInc;
    y+=One;
    iy++;
  }
}


void Rasterizer2D::ClippedRasterize(const Triangle2D& t,const AABB2D& aabb)
{
  vector<Triangle2D> tris(1);
  vector<BarycentricCoords> bary(1);
  tris[0] = t;
  bary[0].a.set(1,0,0);
  bary[0].b.set(0,1,0);
  bary[0].c.set(0,0,1);
  Clip(aabb,tris,bary);
  for(size_t i=0;i<tris.size();i++) {
    Rasterize(tris[i],bary[i].a,bary[i].b,bary[i].c);
  }
}


void Rasterizer2D::Rasterize(const AABB2D& b)
{
  //TODO: fill correctly
  Real i1=Ceil(b.bmin.x);
  Real i2=Floor(b.bmax.x);
  Real j1=Ceil(b.bmin.y);
  Real j2=Floor(b.bmax.y);
  int imin = (int)iRound(i1);
  int imax = (int)iRound(i2);
  int jmin = (int)iRound(j1);
  int jmax = (int)iRound(j2);
  Real du,dv;
  du = 1.0/(b.bmax.x-b.bmin.x);
  dv = 1.0/(b.bmax.y-b.bmin.y);
  Vector3 params;
  params.z = 0;
  params.x = (i1 - b.bmin.x)*du;
  for(int i=imin;i<=imax;i++) {
    params.y = (j1 - b.bmin.y)*dv;
    for(int j=jmin;j<=jmax;j++) {
      VisitCell(params,i,j);
      params.y += du;
    }
    params.x += du;
  }
}

//void ClippedRasterize(const AABB2D& t,const AABB2D& aabb);

} //namespace Meshing
