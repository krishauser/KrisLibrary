#include <KrisLibrary/Logger.h>
#include "MonotoneChain.h"
#include <iostream>
#include <sstream>
using namespace Geometry;
using namespace std;

Real eval_y(const Vector2& a,const Vector2& b,Real x)
{
  assert(x >= a.x && x <= b.x);
  assert(a.x < b.x);
  Real u=(x-a.x)/(b.x - a.x);
  return (1-u)*a.y + u*b.y;
}

Real eval_y(const Segment2D& s,Real x)
{
  return eval_y(s.a,s.b,x);
}

Real XMonotoneChain::eval(Real x) const
{
  assert(!v.empty());
  assert(x >= v.front().x);
  assert(x <= v.back().x);
  if(v.size()==1) return v.front().y;
  assert(isValid());
  //should we do log n search?  eh whatever
  for(size_t i=0;i+1<v.size();i++) {
    if(v[i].x <= x && x <= v[i+1].x) {
      return eval_y(v[i],v[i+1],x);
    }
  }
  LOG4CXX_FATAL(KrisLibrary::logger(),"Shouldn't get here");
  stringstream ss;
  for(size_t i=0;i<v.size();i++)
    ss<<v[i]<<", ";
  LOG4CXX_FATAL(KrisLibrary::logger(),ss.str());
  LOG4CXX_FATAL(KrisLibrary::logger(),"x is "<<x);
  abort();
  return 0;
}

bool XMonotoneChain::isValid() const
{
  for(size_t i=0;i+1<v.size();i++) {
    if(IsNaN(v[i].x) || IsNaN(v[i].y)) { LOG4CXX_INFO(KrisLibrary::logger(),"NaN!"); return false; }
    if(!Lexical2DOrder(v[i],v[i+1])) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Not in lexical order!");
      LOG4CXX_INFO(KrisLibrary::logger(),v[i]<<" -> "<<v[i+1]);
      return false;
    }
  }
  if(IsNaN(v.back().x) || IsNaN(v.back().y)) { LOG4CXX_INFO(KrisLibrary::logger(),"NaN!"); return false; }
  return true;
}

void XMonotoneChain::upperEnvelope(const XMonotoneChain& e)
{
  const vector<Vector2>& w=e.v;
  if(v.empty()) {
    v=w;
    return;
  }
  if(w.empty()) {  //no change to upper envelope
    return;
  }
  assert(isValid() && e.isValid());
  assert(v.size()>=2);
  assert(w.size()>=2);
  //make sure x-ranges intersect
  assert(v.front().x < w.back().x);
  assert(w.front().x < v.back().x);

  //sweep line from 0 to 2pi
  //status is s1,s2, the segment that we're currently on
  //event points only at segment points
  Vector2 p;
  Segment2D s1,s2;
  Real y1,y2;

  //first point intersection with x=0
  s1.a=v[0];
  s2.a=w[0];
  s1.b=v[1];
  s2.b=w[1];
  int i1=1,i2=1;

  enum {Seg1,Seg2};
  int nextEventPoint;
  vector<Vector2> z; z.reserve(v.size()+w.size());

#define ADDPOINT(x) { \
if(z.empty()) z.push_back(x); \
else if(!z.back().isEqual(x,Epsilon)) { \
  if(!Lexical2DOrder(z.back(),x)) { \
    LOG4CXX_FATAL(KrisLibrary::logger(),"Out of order addition to z"); \
    LOG4CXX_FATAL(KrisLibrary::logger(),z.back()<<", "<<x); \
  } \
  assert(Lexical2DOrder(z.back(),x)); \
  z.push_back(x); \
}\
}

  //eat up first segs
  if(Lexical2DOrder(s1.a,s2.a)) {
    while(Lexical2DOrder(s1.b,s2.a)) {
      ADDPOINT(s1.a);
      i1++; s1.a=s1.b; s1.b=v[i1];
      assert(i1 < (int)v.size());
    }
    nextEventPoint=Seg2;
  }
  else {
    while(Lexical2DOrder(s2.b,s1.a)) {
      ADDPOINT(s2.a);
      i2++; s2.a=s2.b; s2.b=w[i2];
      assert(i2 < (int)w.size());
    }
    nextEventPoint=Seg1;
  }
  bool done=false;
  while(!done) {
    if(nextEventPoint == Seg1) {
      assert(!Lexical2DOrder(s1.a,s2.a) && !Lexical2DOrder(s2.b,s1.a));
      y1=s1.a.y;
      y2=eval_y(s2,s1.a.x);
      if(y1>=y2) {
	ADDPOINT(s1.a);
      }
    }
    else if(nextEventPoint == Seg2) {
      assert(!Lexical2DOrder(s2.a,s1.a) && !Lexical2DOrder(s1.b,s2.a));
      y1=eval_y(s1,s2.a.x);
      y2=s2.a.y;
      if(y2>=y1) {
	ADDPOINT(s2.a);
      }
    }

    if(s1.intersects(s2,p)) {
      if(!p.isEqual(s1.a,Epsilon) &&
	 !p.isEqual(s1.b,Epsilon) &&
	 !p.isEqual(s2.a,Epsilon) && 
	 !p.isEqual(s2.b,Epsilon)) {
	if(Lexical2DOrder(s1.a,p) &&
	   Lexical2DOrder(p,s1.b) &&
	   Lexical2DOrder(s2.a,p) &&
	   Lexical2DOrder(p,s2.b)) {
	  ADDPOINT(p);
	}
	else {
	  LOG4CXX_FATAL(KrisLibrary::logger(),"intersection point "<<p<<" violates the order: ");
	  LOG4CXX_FATAL(KrisLibrary::logger(),s1.a<<" -> "<<s1.b);
	  LOG4CXX_FATAL(KrisLibrary::logger(),s2.a<<" -> "<<s2.b);
	  abort();
	}
      }
    }


    //what's the next event point?  either s1.b or s2.b
    if(Lexical2DOrder(s1.b,s2.b)) {
      //increment seg1 
      i1++;
      if(i1 >= (int)v.size())
	done=true;
      else {
	s1.a=s1.b;   s1.b = v[i1];
      }
      nextEventPoint=Seg1;
    }
    else {
      //increment seg2 
      i2++;
      if(i2 >= (int)w.size())
	done=true;
      else {
	s2.a=s2.b;  s2.b = w[i2];
      }
      nextEventPoint=Seg2;
    }
  }
  assert(i1 == (int)v.size() || i2 == (int)w.size());
  //append remaining edges to edge list
  if(i1 == (int)v.size()) {
    //we still have the last point of v to take care of
    assert(Lexical2DOrder(s1.b,w[i2]));
    y1=s1.b.y;
    y2=eval_y(s2,s1.b.x);
    if(y1>=y2) {
      ADDPOINT(s1.b);
    }
    //fill out the rest of the chain with w
    while(i2 < (int)w.size()) {
      ADDPOINT(w[i2]);
      i2++;
    }
  }
  else {
    //we still have the last point of v to take care of
    assert(!Lexical2DOrder(v[i1],s2.b));
    y1=eval_y(s1,s2.b.x);
    y2=s2.b.y;
    if(y2>=y1) {
      ADDPOINT(s2.b);
    }
    //fill out the rest of the chain with v
    while(i1 < (int)v.size()) {
      ADDPOINT(v[i1]);
      i1++;
    }
  }
  /*
  //ERROR CHECKING
  XMonotoneChain f;
  f.v=z;
  assert(f.isValid());
  for(size_t i=0;i<z.size();i++) {
    Real x=z[i].x;
    if(x >= v.front().x && x <= v.back().x) {
      y1=f.eval(x); y2=eval(x);
      if(!(y1+0.001 >= y2)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error in MonotoneChain.upperEnvelope()!");
	LOG4CXX_INFO(KrisLibrary::logger(),y1<<" < "<<y2);
      }
      assert(y1+0.001 >= y2);
    }
    if(x >= w.front().x && x <= w.back().x) {
      y1=f.eval(x); y2=e.eval(x);
      if(!(y1+0.001 >= y2)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error in MonotoneChain.upperEnvelope()!");
	LOG4CXX_INFO(KrisLibrary::logger(),y1<<" < "<<y2);
      }
      assert(y1+0.001 >= y2);
    }
  }
  */
  v = z;
}


Real XMonotoneChain::minimum(Real a,Real b,Real* x)
{
  assert(a <= b);
  Real xmin=a;
  Real ymin=eval(a);
  //could do this in log(n)+k time, but who cares...
  for(size_t i=0;i<v.size();i++) {
    if(v[i].x >= a && v[i].x <= b) {
      if(v[i].y < ymin) {
	xmin = v[i].x;
	ymin = v[i].y;
      }
    }
  }
  Real ytemp=eval(b);
  if(ytemp < ymin) { xmin=b; ymin=ytemp; }
  if(x) *x=xmin;
  return ymin;
}

void XMonotoneChain::SelfTest()
{
  XMonotoneChain c1,c2;
  c1.v.resize(2);
  c2.v.resize(2);
  c1.v[0].set(0,-1);
  c1.v[1].set(2,1);
  c2.v[0].set(0,1);
  c2.v[1].set(2,-1);
  c1.upperEnvelope(c2);
  for(size_t i=0;i<c1.v.size();i++) {
    LOG4CXX_INFO(KrisLibrary::logger(),c1.v[i]<<", ");
  }
  KrisLibrary::loggerWait();
}
