#include <KrisLibrary/Logger.h>
#include "Geometric2DCSpace.h"
#include "EdgePlannerHelpers.h"
#include "CSetHelpers.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/math3d/Segment2D.h>
#include <KrisLibrary/math3d/Line2D.h>
#include <KrisLibrary/utils/arrayutils.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include "EdgePlanner.h"
using namespace GLDraw;

Geometric2DObstacleFreeSet::Geometric2DObstacleFreeSet(const GeometricPrimitive2D& _obstacle)
:obstacle(_obstacle),robot(NULL),translationOnly(true)
{
}

Geometric2DObstacleFreeSet::Geometric2DObstacleFreeSet(const GeometricPrimitive2D& _obstacle,const Geometric2DCollection& _robot,bool _translationOnly)
:obstacle(_obstacle),robot(&_robot),translationOnly(_translationOnly)
{}

bool Geometric2DObstacleFreeSet::Contains(const Config& x)
{
  if(robot==NULL) {
    return !obstacle.Collides(Vector2(x[0],x[1]));
  }
  Geometric2DCollection trobot = *robot;
  RigidTransform2D T;
  if(translationOnly) {
    T.R.setIdentity();
    T.t.set(x[0],x[1]);
  }
  else {
    T.t.set(x[0],x[1]);
    T.R.setRotate(x[2]);
  }
  trobot.Transform(T);
  return !trobot.Collides(obstacle);
}

Real Geometric2DObstacleFreeSet::ObstacleDistance(const Config& x)
{
  return Inf;
  if(robot==NULL) return obstacle.Distance(Vector2(x[0],x[1]));
  Geometric2DCollection trobot = *robot;
  RigidTransform2D T;
  if(translationOnly) {
    T.R.setIdentity();
    T.t.set(x[0],x[1]);
  }
  else {
    T.t.set(x[0],x[1]);
    T.R.setRotate(x[2]);
  }
  trobot.Transform(T);
  //return trobot.Distance(obstacle);
  return -Inf;
}


void Geometric2DCollection::Clear()
{
  triangles.clear();
  aabbs.clear();
  boxes.clear();
  circles.clear();
}
int Geometric2DCollection::NumObstacles() const
{
  return aabbs.size()+boxes.size()+triangles.size()+circles.size();
}
int Geometric2DCollection::ObstacleIndex(int obstacle) const
{
  if(obstacle < (int)aabbs.size()) 
    return obstacle;
  obstacle -= (int)aabbs.size();
  if(obstacle < (int)boxes.size()) 
    return (int)obstacle;
  obstacle -= boxes.size();
  if(obstacle < (int)triangles.size()) 
    return obstacle;
  obstacle -= (int)triangles.size();
  assert(obstacle < (int)circles.size());
  return obstacle;
}
Geometric2DCollection::Type Geometric2DCollection::ObstacleType(int obstacle) const
{
  if(obstacle < (int)aabbs.size()) 
    return AABB;
  obstacle -= (int)aabbs.size();
  if(obstacle < (int)boxes.size()) 
    return Box;
  obstacle -= (int)boxes.size();
  if(obstacle < (int)triangles.size())
    return Triangle;
  obstacle -= (int)triangles.size();
  assert(obstacle < (int)circles.size());
  return Circle;
}
const char* Geometric2DCollection::ObstacleTypeName(int obstacle) const
{
  if(obstacle < (int)aabbs.size()) 
    return "aabb";
  obstacle -= (int)aabbs.size();
  if(obstacle < (int)boxes.size()) 
    return "box";
  obstacle -= (int)boxes.size();
  if(obstacle < (int)triangles.size())
    return "triangle";
  obstacle -= (int)triangles.size();
  assert(obstacle < (int)circles.size());
  return "circle";
}
GeometricPrimitive2D Geometric2DCollection::Obstacle(int obstacle) const
{
  if(obstacle < (int)aabbs.size()) 
    return GeometricPrimitive2D(aabbs[obstacle]);
  obstacle -= (int)aabbs.size();
  if(obstacle < (int)boxes.size()) 
    return GeometricPrimitive2D(boxes[obstacle]);
  obstacle -= boxes.size();
  if(obstacle < (int)triangles.size()) 
    return GeometricPrimitive2D(triangles[obstacle]);
  obstacle -= (int)triangles.size();
  assert(obstacle < (int)circles.size());
  return GeometricPrimitive2D(circles[obstacle]);
}

void Geometric2DCollection::Add(const Triangle2D& tri)
{
  triangles.push_back(tri);
}
void Geometric2DCollection::Add(const Box2D& box)
{
  boxes.push_back(box);
}
void Geometric2DCollection::Add(const AABB2D& bbox)
{
  aabbs.push_back(bbox);
}
//void Add(const Polygon2D& poly);
void Geometric2DCollection::Add(const Circle2D& sphere)
{
  circles.push_back(sphere);
}

void Geometric2DCollection::Add(const Geometric2DCollection& geom)
{
  ArrayUtils::concat(aabbs,geom.aabbs);
  ArrayUtils::concat(boxes,geom.boxes);
  ArrayUtils::concat(triangles,geom.triangles);
  ArrayUtils::concat(circles,geom.circles);
}

void Geometric2DCollection::Transform(const RigidTransform2D& T)
{
  size_t oldBoxSize=boxes.size();
  boxes.resize(boxes.size()+aabbs.size());
  for(size_t i=0;i<oldBoxSize;i++)
    boxes[i].setTransformed(boxes[i],T);
  for(size_t i=0;i<aabbs.size();i++)
    boxes[i+oldBoxSize].setTransformed(aabbs[i],T);
  aabbs.clear();
  for(size_t i=0;i<triangles.size();i++)
    triangles[i].setTransformed(triangles[i],T);
  for(size_t i=0;i<circles.size();i++)
    circles[i].center = T*circles[i].center;
}

Real Geometric2DCollection::Distance(const Vector2& p) const
{
  Real dmin=Inf;
  //now we're looking at d^2
  for(size_t i=0;i<triangles.size();i++)
    dmin = Min(dmin,triangles[i].closestPoint(p).distanceSquared(p));
  for(size_t i=0;i<aabbs.size();i++) {
    Vector2 q;
    dmin = Min(dmin,aabbs[i].distanceSquared(p,q));
  }
  for(size_t i=0;i<boxes.size();i++) {
    Vector2 q;
    dmin = Min(dmin,boxes[i].distanceSquared(p,q));
  }
  //back to looking at d
  dmin = Sqrt(dmin);
  for(size_t i=0;i<circles.size();i++)
    dmin = Min(dmin,circles[i].distance(p));
  if(dmin < 0) return 0;
  return dmin;
}

Real Geometric2DCollection::Distance(const Circle2D& s) const
{
  Real d=Distance(s.center);
  if(d <= s.radius) return 0;
  return d-s.radius;
}

bool Geometric2DCollection::Collides(const Vector2& p) const
{
  for(size_t i=0;i<aabbs.size();i++) 
    if(aabbs[i].contains(p)) return true;
  for(size_t i=0;i<circles.size();i++)
    if(circles[i].contains(p)) return true;
  for(size_t i=0;i<boxes.size();i++)
    if(boxes[i].contains(p)) return true;
  for(size_t i=0;i<triangles.size();i++)
    if(triangles[i].contains(p)) return true;
  return false;
}

bool Geometric2DCollection::Collides(const Segment2D& s) const
{
  for(size_t i=0;i<aabbs.size();i++) {
    if(s.intersects(aabbs[i])) return true;
  }
  for(size_t i=0;i<triangles.size();i++) {
    if(triangles[i].intersects(s)) return true;
  }
  for(size_t i=0;i<aabbs.size();i++)
    if(s.intersects(aabbs[i])) return true;
  for(size_t i=0;i<boxes.size();i++) {
    if(boxes[i].intersects(s)) return true;
  }
  for(size_t i=0;i<circles.size();i++) {
    if(circles[i].intersects(s)) return true;
  }
  return false;
}

bool Geometric2DCollection::Collides(const Circle2D& circle) const
{
  return Distance(circle.center) <= circle.radius;
}

bool Geometric2DCollection::Collides(const Box2D& box) const
{
  for(size_t i=0;i<aabbs.size();i++)
    if(box.intersects(aabbs[i])) return true;
  for(size_t i=0;i<boxes.size();i++) 
    if(box.intersects(boxes[i])) return true;
  for(size_t i=0;i<circles.size();i++)
    if(box.intersects(circles[i])) return true;
  for(size_t i=0;i<triangles.size();i++)
    if(box.intersects(triangles[i])) return true;
  return false;
}

bool Geometric2DCollection::Collides(const Triangle2D& tri) const
{
  for(size_t i=0;i<aabbs.size();i++) {
    Box2D box; box.set(aabbs[i]);
    if(box.intersects(tri)) return true;
  }
  for(size_t i=0;i<boxes.size();i++)
    if(boxes[i].intersects(tri)) return true;
  for(size_t i=0;i<circles.size();i++)
    if(tri.closestPoint(circles[i].center).distanceSquared(circles[i].center) < Sqr(circles[i].radius)) return true;
  for(size_t i=0;i<triangles.size();i++)
    if(tri.intersects(triangles[i])) return true;
  return false;
}

bool Geometric2DCollection::Collides(const GeometricPrimitive2D& geom) const
{
  for(size_t i=0;i<aabbs.size();i++) 
    if(geom.Collides(aabbs[i])) return true;
  for(size_t i=0;i<boxes.size();i++) 
    if(geom.Collides(boxes[i])) return true;
  for(size_t i=0;i<circles.size();i++)
    if(geom.Collides(circles[i])) return true;
  for(size_t i=0;i<triangles.size();i++)
    if(geom.Collides(triangles[i])) return true;
  return false;
}

bool Geometric2DCollection::Collides(const Geometric2DCollection& geom) const
{
  for(size_t i=0;i<geom.aabbs.size();i++) {
    Box2D box; box.set(geom.aabbs[i]);
    if(Collides(box)) return true;
  }
  for(size_t i=0;i<geom.boxes.size();i++) 
    if(Collides(geom.boxes[i])) return true;
  for(size_t i=0;i<geom.circles.size();i++)
    if(Collides(geom.circles[i])) return true;
  for(size_t i=0;i<geom.triangles.size();i++)
    if(Collides(geom.triangles[i])) return true;
  return false;
}

Real Geometric2DCollection::Distance(const Vector2& p,int obstacle) const
{
  Type type=ObstacleType(obstacle);
  int index = ObstacleIndex(obstacle);
  switch(type) {
  case AABB:
    {
      Vector2 q;
      return aabbs[index].distance(p,q);
    }
  case Triangle:
    return triangles[index].closestPoint(p).distance(p);
  case Box:
    {
      Vector2 q;
      return Sqrt(boxes[index].distanceSquared(p,q));
    }
  case Circle:
    return circles[index].distance(p);
  }
  abort();
  return 0;
}

Real Geometric2DCollection::Distance(const Circle2D& s,int obstacle) const
{
  Real d=Distance(s.center,obstacle);
  if(d <= s.radius) return 0;
  return d-s.radius;
}

bool Geometric2DCollection::Collides(const Vector2& p,int obstacle) const
{
  Type type=ObstacleType(obstacle);
  int index = ObstacleIndex(obstacle);
  switch(type) {
  case AABB:
    return aabbs[index].contains(p);
  case Triangle:
    return triangles[index].contains(p);
  case Box:
    return boxes[index].contains(p);
  case Circle:
    return circles[index].contains(p);
  }
  abort();
  return false;
}

bool Geometric2DCollection::Collides(const Segment2D& s,int obstacle) const
{
  Type type=ObstacleType(obstacle);
  int index = ObstacleIndex(obstacle);
  switch(type) {
  case AABB:
    return s.intersects(aabbs[index]);
  case Triangle:
    return triangles[index].intersects(s);
  case Box:
    return boxes[index].intersects(s);
  case Circle:
    return circles[index].intersects(s);
  }
  abort();
  return false;
}

bool Geometric2DCollection::Collides(const Circle2D& circle,int obstacle) const
{
  return Distance(circle.center,obstacle) <= circle.radius;
}

bool Geometric2DCollection::Collides(const Box2D& box,int obstacle) const
{
  Type type=ObstacleType(obstacle);
  int index = ObstacleIndex(obstacle);
  switch(type) {
  case AABB:
    return box.intersects(aabbs[index]);
  case Triangle:
    return box.intersects(triangles[index]);
  case Box:
    return box.intersects(boxes[index]);
  case Circle:
    return box.intersects(circles[index]);
  }
  abort();
  return false;
}


bool Geometric2DCollection::Collides(const Triangle2D& t,int obstacle) const
{
  Type type=ObstacleType(obstacle);
  int index = ObstacleIndex(obstacle);
  switch(type) {
  case AABB:
    {
      Box2D box; box.set(aabbs[index]);
      return box.intersects(t);
    }
  case Triangle:
    return triangles[index].intersects(t);
  case Box:
    return boxes[index].intersects(t);
  case Circle:
    {
      return (t.closestPoint(circles[index].center).distanceSquared(circles[index].center) < Sqr(circles[index].radius));
    }
  }
  abort();
  return false;
}


bool Geometric2DCollection::Collides(const GeometricPrimitive2D& geom,int obstacle) const
{
  return geom.Collides(Obstacle(obstacle));
}


bool Geometric2DCollection::Collides(const Geometric2DCollection& geom,int obstacle) const
{
  for(size_t i=0;i<geom.aabbs.size();i++) {
    Box2D box; box.set(geom.aabbs[i]);
    if(Collides(box,obstacle)) return true;
  }
  for(size_t i=0;i<geom.boxes.size();i++)
    if(Collides(geom.boxes[i],obstacle)) return true;
  for(size_t i=0;i<geom.circles.size();i++)
    if(Collides(geom.circles[i],obstacle)) return true;
  for(size_t i=0;i<geom.triangles.size();i++)
    if(Collides(geom.triangles[i],obstacle)) return true;
  return false;
}


void Geometric2DCollection::DrawOutlinesGL() const
{
  for(size_t i=0;i<aabbs.size();i++) {
    glBegin(GL_LINE_LOOP);
    glVertex2d(aabbs[i].bmin.x,aabbs[i].bmin.y);
    glVertex2d(aabbs[i].bmax.x,aabbs[i].bmin.y);
    glVertex2d(aabbs[i].bmax.x,aabbs[i].bmax.y);
    glVertex2d(aabbs[i].bmin.x,aabbs[i].bmax.y);
    glEnd();
  }
  for(size_t i=0;i<boxes.size();i++) {
    glBegin(GL_LINE_LOOP);
    glVertex2v(boxes[i].origin);
    glVertex2v(boxes[i].origin+boxes[i].dims.x*boxes[i].xbasis);
    glVertex2v(boxes[i].origin+boxes[i].dims.x*boxes[i].xbasis+boxes[i].dims.y*boxes[i].ybasis);
    glVertex2v(boxes[i].origin+boxes[i].dims.y*boxes[i].ybasis);
    glEnd();
  }
  for(size_t i=0;i<triangles.size();i++) {
    glBegin(GL_LINE_LOOP);
    glVertex2v(triangles[i].a);
    glVertex2v(triangles[i].b);
    glVertex2v(triangles[i].c);
    glEnd();
  }
  for(size_t i=0;i<circles.size();i++) {
    drawWireCircle2D(circles[i].center,float(circles[i].radius));
  }
}

void Geometric2DCollection::DrawGL() const
{
  glBegin(GL_QUADS);
  for(size_t i=0;i<aabbs.size();i++) {
    glVertex2d(aabbs[i].bmin.x,aabbs[i].bmin.y);
    glVertex2d(aabbs[i].bmax.x,aabbs[i].bmin.y);
    glVertex2d(aabbs[i].bmax.x,aabbs[i].bmax.y);
    glVertex2d(aabbs[i].bmin.x,aabbs[i].bmax.y);
  }
  for(size_t i=0;i<boxes.size();i++) {
    glVertex2v(boxes[i].origin);
    glVertex2v(boxes[i].origin+boxes[i].dims.x*boxes[i].xbasis);
    glVertex2v(boxes[i].origin+boxes[i].dims.x*boxes[i].xbasis+boxes[i].dims.y*boxes[i].ybasis);
    glVertex2v(boxes[i].origin+boxes[i].dims.y*boxes[i].ybasis);
  }
  glEnd();
  glBegin(GL_TRIANGLES);
  for(size_t i=0;i<triangles.size();i++) {
    glVertex2v(triangles[i].a);
    glVertex2v(triangles[i].b);
    glVertex2v(triangles[i].c);
  }
  glEnd();
  for(size_t i=0;i<circles.size();i++) {
    drawCircle2D(circles[i].center,float(circles[i].radius));
  }
}

void Geometric2DCollection::ToPolygons(vector<vector<Vector2> >& polys) const
{
  polys.resize(0);
  for(size_t i=0;i<aabbs.size();i++) {
    polys.resize(polys.size()+1);
    polys.back().resize(4);
    polys.back()[0].set(aabbs[i].bmin.x,aabbs[i].bmin.y);
    polys.back()[1].set(aabbs[i].bmax.x,aabbs[i].bmin.y);
    polys.back()[2].set(aabbs[i].bmax.x,aabbs[i].bmax.y);
    polys.back()[3].set(aabbs[i].bmin.x,aabbs[i].bmax.y);
  }
  for(size_t i=0;i<boxes.size();i++) {
    polys.resize(polys.size()+1);
    polys.back().resize(4);
    polys.back()[0].set(boxes[i].origin);
    polys.back()[1].set(boxes[i].origin+boxes[i].dims.x*boxes[i].xbasis);
    polys.back()[2].set(boxes[i].origin+boxes[i].dims.x*boxes[i].xbasis+boxes[i].dims.y*boxes[i].ybasis);
    polys.back()[3].set(boxes[i].origin+boxes[i].dims.y*boxes[i].ybasis);
  }
  for(size_t i=0;i<triangles.size();i++) {
    polys.resize(polys.size()+1);
    polys.back().resize(3);
    polys.back()[0].set(triangles[i].a);
    polys.back()[1].set(triangles[i].b);
    polys.back()[2].set(triangles[i].c);
  }
  for(size_t i=0;i<circles.size();i++) {
    polys.resize(polys.size()+1);
    polys.back().resize(32);
    for(int k=0;k<32;k++) {
      Real u = Real(k)/Real(32);
      Vector2 ofs(Cos(TwoPi*u),Sin(TwoPi*u));
      polys.back()[k].set(circles[i].center + circles[i].radius*ofs);
    }
  }
}




Geometric2DCSpace::Geometric2DCSpace()
{
  euclideanSpace=true;
  visibilityEpsilon = 0.01;
  domain.bmin.set(0,0);
  domain.bmax.set(1,1);
}

void Geometric2DCSpace::InitConstraints()
{
  AddConstraint("x_bound",new AxisRangeSet(0,domain.bmin.x,domain.bmax.x));
  AddConstraint("y_bound",new AxisRangeSet(0,domain.bmin.y,domain.bmax.y));
  char buf[64];
  for(int i=0;i<Geometric2DCollection::NumObstacles();i++) {
    snprintf(buf,64,"%s[%d]",ObstacleTypeName(i),ObstacleIndex(i));
    AddConstraint(buf,new Geometric2DObstacleFreeSet(Obstacle(i)));
  }
}

Real Geometric2DCSpace::ObstacleDistance(const Vector2& p) const
{
  Real dmin = Inf;
  if(p.x < domain.bmin.x || p.x > domain.bmax.x) return 0;
  dmin = Min(dmin,p.x-domain.bmin.x);
  dmin = Min(dmin,domain.bmax.x-p.x);
  if(p.y < domain.bmin.y || p.y > domain.bmax.y) return 0;
  dmin = Min(dmin,p.y-domain.bmin.y);
  dmin = Min(dmin,domain.bmax.y-p.y);
  return Min(dmin,Geometric2DCollection::Distance(p));
}

Real Geometric2DCSpace::ObstacleDistance(const Circle2D& c) const
{
  Real dmin = Inf;
  const Vector2& p=c.center;
  Real r=c.radius;
  if(p.x-r < domain.bmin.x || p.x+r > domain.bmax.x) return 0;
  dmin = Min(dmin,p.x-r-domain.bmin.x);
  dmin = Min(dmin,domain.bmax.x-p.x-r);
  if(p.y-r < domain.bmin.y || p.y+r > domain.bmax.y) return 0;
  dmin = Min(dmin,p.y-r-domain.bmin.y);
  dmin = Min(dmin,domain.bmax.y-p.y-r);
  return Min(dmin,Geometric2DCollection::Distance(c));
}

bool Geometric2DCSpace::ObstacleOverlap(const Segment2D& s) const
{
  if(!domain.contains(s.a)) return true;
  if(!domain.contains(s.b)) return true;
  if(Geometric2DCollection::Collides(s)) return true;
  return false;
}

bool Geometric2DCSpace::ObstacleOverlap(const Circle2D& circle) const
{
  return ObstacleDistance(circle.center) <= circle.radius;
}

bool Geometric2DCSpace::ObstacleOverlap(const Box2D& box) const
{
  AABB2D bb;
  box.getAABB(bb);
  if(!domain.contains(bb)) return true;
  return Geometric2DCollection::Collides(box);
}


bool Geometric2DCSpace::ObstacleOverlap(const Triangle2D& tri) const
{
  AABB2D bb;
  tri.getAABB(bb);
  if(!domain.contains(bb)) return true;
  return Geometric2DCollection::Collides(tri);
}


void Geometric2DCSpace::DrawGL() const
{
  //blank out background (light yellow)
  //glColor3f(1.f,1.f,0.5f);
  //blank out background (white)
  glColor3f(1.f,1.f,1.f);
  glBegin(GL_QUADS);
  glVertex2d(domain.bmin.x,domain.bmin.y);
  glVertex2d(domain.bmax.x,domain.bmin.y);
  glVertex2d(domain.bmax.x,domain.bmax.y);
  glVertex2d(domain.bmin.x,domain.bmax.y);
  glEnd();

  //draw obstacles (dark grey)
  glColor3f(0.2f,0.2f,0.2f);
  Geometric2DCollection::DrawGL();
}

void Geometric2DCSpace::Sample(Config& x)
{
  x.resize(2);
  x(0)=Rand(domain.bmin.x,domain.bmax.x);
  x(1)=Rand(domain.bmin.y,domain.bmax.y);
}

void Geometric2DCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(2);
  if(euclideanSpace) SampleDisk(r,x(0),x(1));
  else { x(0)=Rand(-r,r); x(1)=Rand(-r,r); }
  x += c;
}



class Geometric2DEdgeChecker : public EdgeChecker
{
public:
  Geometric2DEdgeChecker(const Config& _a,const Config& _b,Geometric2DCSpace* _space,int _obstacle=-1)
    :EdgeChecker(_space,_a,_b),gspace(_space),obstacle(_obstacle),done(false),failed(false)
  {
    s.a.set(_a(0),_a(1));
    s.b.set(_b(0),_b(1));
  }
  virtual bool IsVisible() {
    if(obstacle < 0) {
      return !gspace->Collides(s);
    }
    else {
      return !gspace->Collides(s,obstacle);
    }
  }
  
  virtual CSpace* Space() const { return space; }
  virtual EdgePlannerPtr Copy() const { return make_shared<Geometric2DEdgeChecker>(path->Start(),path->End(),gspace,obstacle); }
  virtual EdgePlannerPtr ReverseCopy() const  { return make_shared<Geometric2DEdgeChecker>(path->End(),path->Start(),gspace,obstacle); }

  //for incremental planners
  virtual Real Priority() const { return s.a.distanceSquared(s.b); }
  virtual bool Plan() { failed = !IsVisible(); done = true; return !failed; }
  virtual bool Done() const {  return done; }
  virtual bool Failed() const {  return failed; }

  Geometric2DCSpace* gspace;
  int obstacle;
  Segment2D s;
  //used for incremental planner only
  bool done;
  bool failed;
};

EdgePlannerPtr Geometric2DCSpace::PathChecker(const Config& a,const Config& b)
{
  //return new EpsilonEdgeChecker(a,b,this,visibilityEpsilon);
  return make_shared<Geometric2DEdgeChecker>(a,b,this);
}

EdgePlannerPtr Geometric2DCSpace::PathChecker(const Config& a,const Config& b,int constraint)
{
  if(constraint <= 1) return make_shared<TrueEdgeChecker>(this,a,b);
  Segment2D s;
  s.a.set(a(0),a(1));
  s.b.set(b(0),b(1));
  if(Geometric2DCollection::Collides(s,constraint-1))
    return make_shared<FalseEdgeChecker>(this,a,b);
  else
    return make_shared<TrueEdgeChecker>(this,a,b);
}

Real Geometric2DCSpace::Distance(const Config& x, const Config& y)
{ 
  if(euclideanSpace) return Distance_L2(x,y);
  else return Distance_LInf(x,y);
}

void Geometric2DCSpace::Properties(PropertyMap& map) const
{
  map.set("cartesian",1);
  map.set("geodesic",1);
  if(euclideanSpace)
    map.set("metric","euclidean");
  else
    map.set("metric","Linf");
  Real v = (domain.bmax.x-domain.bmin.x)*(domain.bmax.y-domain.bmin.y);
  map.set("volume",v);
  map.set("diameter",domain.bmin.distance(domain.bmax));
  vector<Real> bmin(2),bmax(2);
  domain.bmin.get(bmin[0],bmin[1]);
  domain.bmax.get(bmax[0],bmax[1]);
  map.setArray("minimum",bmin);
  map.setArray("maximum",bmax);
}
