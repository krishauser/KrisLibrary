#include <KrisLibrary/Logger.h>
#include "PenetrationDepth.h"
#include <math3d/geometry3d.h>
#include <math3d/interpolate.h>
#include <map>
#include <utils.h>
#include <iostream>
using namespace std;
using namespace Geometry;

#define VERBOSE 0



ApproximatePenetrationDepth::ApproximatePenetrationDepth(const TriMeshWithTopology &_m1, const TriMesh &_m2)
  :m1(_m1),m2(_m2),maxDepth(0)
{
  Assert(m1.vertexNeighbors.size() == m1.verts.size());
}

void ApproximatePenetrationDepth::Reset()
{
  int nv=m1.verts.size();
  vertexClass.resize(nv);
  depth.resize(nv);
  normal.resize(nv);
  fringe.clear();
  std::fill(vertexClass.begin(),vertexClass.end(),Unvisited);
  std::fill(depth.begin(),depth.end(),Zero);
  std::fill(normal.begin(),normal.end(),Vector3(Zero));
  maxDepth = 0;
}


struct EdgeCollision
{
  Vector3 point,normal;
  int vinside,voutside;
};

void ApproximatePenetrationDepth::ComputeInitial(const RigidTransform& f1,const RigidTransform& f2,const int tc1[], const int tc2[],int n)
{
  /*
  for(int i=0;i<n;i++) {
    int t1 = tc1[i], t2 = tc2[i];
    Triangle3D T1,T2;
    m1.GetTriangle(t1,T1);
    m2.GetTriangle(t2,T2);
    T1.a = f1*T1.a;
    T1.b = f1*T1.b;
    T1.c = f1*T1.c;
    T2.a = f2*T2.a;
    T2.b = f2*T2.b;
    T2.c = f2*T2.c;
    if(!T1.intersect(T2)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error, T1 doesn't intersect T2!");
      LOG4CXX_INFO(KrisLibrary::logger(),"i="<<i);
      LOG4CXX_INFO(KrisLibrary::logger(),T1.a<<", "<<T1.b<<", "<<T1.c);
      LOG4CXX_INFO(KrisLibrary::logger(),T2.a<<", "<<T2.b<<", "<<T2.c);
    }
    //Assert(T1.intersect(T2) && T2.intersect(T1));
  }
  */
  RigidTransform fRel,f1inv;
  f1inv.setInverse(f1);
  fRel.compose(f1inv,f2);
  Matrix4 mRel(fRel);
  //map the inside vertex to the edgecollision structure
  multimap<int,EdgeCollision> collisions;
  EdgeCollision ec;
  Real da,db,dc,u;
  for(int i=0;i<n;i++) {
    const TriMesh::Tri& t1 = m1.tris[tc1[i]];
    Triangle3D T1;
    Triangle3D T2,T2_loc;
    m1.GetTriangle(tc1[i],T1);
    m2.GetTriangle(tc2[i],T2_loc);
    T2.setTransformed(T2_loc,mRel);
    const Vector3 &a=T1.a,&b=T1.b,&c=T1.c;
    /*
    Triangle3D T2_world, T2_1, T2_2;
    T2_world.setTransformed(T2_loc,f2);
    T2_1.setTransformed(T2_world,f1inv);
    T2_2.setTransformed(T2_loc,fRel);

    Assert((T2_1.a-T2_2.a).norm() < Epsilon);
    Assert((T2_1.b-T2_2.b).norm() < Epsilon);
    Assert((T2_1.c-T2_2.c).norm() < Epsilon);
    
    T2 = T2_1;
    Assert(T2.intersect(T1) && T1.intersect(T2));
    */

    ec.normal = T2.normal();
    //find all edges of T1 that collide with T2

    Plane3D P2;
    T2.getPlane(P2);
    da = P2.distance(a);
    db = P2.distance(b);
    dc = P2.distance(c);

    if(Max(da,db,dc) < Zero || Min(da,db,dc) > Zero) {
      //LOG4CXX_WARN(KrisLibrary::logger(),"Warning: triangle "<<tc1[i]<<" does not intersect "<<tc2[i]);
      continue;
    }
    
    u = SegmentZeroCrossing(da,db);
    if(u >= Zero && u <= One) {
      interpolate(a,b,u,ec.point);
      if(T2.contains(ec.point)) {
	int x=t1.a,y=t1.b;
	if(da > Zero) swap(x,y);
	ec.vinside = x; ec.voutside = y;
	collisions.insert(pair<int,EdgeCollision>(x,ec));
      }
    }

    u = SegmentZeroCrossing(db,dc);
    if(u >= Zero && u <= One) {
      interpolate(b,c,u,ec.point);
      if(T2.contains(ec.point)) {
	int x=t1.b,y=t1.c;
	if(db > Zero) swap(x,y);
	ec.vinside = x; ec.voutside = y;
	collisions.insert(pair<int,EdgeCollision>(x,ec));
      }
    }

    u = SegmentZeroCrossing(dc,da);
    if(u >= Zero && u <= One) {
      interpolate(c,a,u,ec.point);
      if(T2.contains(ec.point)) {
	int x=t1.c,y=t1.a;
	if(dc > Zero) swap(x,y);
	ec.vinside = x; ec.voutside = y;
	collisions.insert(pair<int,EdgeCollision>(x,ec));
      }
    }
  }
#if VERBOSE
  if(!collisions.empty())
    LOG4CXX_INFO(KrisLibrary::logger(),collisions.size()<<" edge collisions.");
#endif

  Real minEdgeDepth=Inf;
  maxDepth=0;
  typedef multimap<int,EdgeCollision>::iterator Iterator;
  for(size_t i=0;i<vertexClass.size();i++)
    Assert(vertexClass[i] == Unvisited);
  for(Iterator i=collisions.begin();i!=collisions.end();i++)
    vertexClass[i->second.vinside] = Fringe;
  for(Iterator i=collisions.begin();i!=collisions.end();i++) {
    if(vertexClass[i->second.voutside] == Fringe) {
      //get the opposing edge penetration
      pair<Iterator,Iterator> range=collisions.equal_range(i->second.voutside);
      Iterator j;
      for(j=range.first;j!=range.second;j++)
	if(j->second.voutside == i->second.vinside) break;
      if(j != range.second) {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Looks like an edge penetration...");
	Assert(j->second.vinside == i->second.voutside);
	Assert(i->second.vinside == j->second.voutside);
	const Vector3& a = m1.verts[i->second.vinside];
	const Vector3& b = m1.verts[i->second.voutside];
	Plane3D p1; p1.setPointNormal(i->second.point,i->second.normal);
	Plane3D p2; p2.setPointNormal(j->second.point,j->second.normal);
	if(p1.distance(a) > Epsilon) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: distance from triangle to inside point is greater than 0! "<<p1.distance(a));
	  KrisLibrary::loggerWait();
	}
	if(p2.distance(b) > Epsilon) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: distance from triangle to inside point is greater than 0! "<<p2.distance(b));
	  KrisLibrary::loggerWait();
	}
	Real da = -p1.distance(a);
	Real db = -p2.distance(b);
	//get the distance from ab to the intersection of p1,p2
	Real de = Inf;
	Line3D l;
	Vector3 pl,pab;
	int res=p1.allIntersections(p2,l);
	if(res == 1) {
	  //line distance
	  Line3D lab; lab.setPoints(a,b);
	  Real t,u;
	  lab.closestPoint(l,t,u);
	  lab.eval(t,pab);
	  l.eval(u,pl);
	  de = pl.distance(pab);
	}
	Real d=Min(da,db,de);
	if(d < minEdgeDepth) {
	  minEdgeDepth = d;
	  if(d == da) {
	    deepestPoint = a;
	    deepestNormal = p1.normal;
	  }
	  else if(d == db) {
	    deepestPoint = b;
	    deepestNormal = p2.normal;
	  }
	  else {
	    deepestPoint = pab;
	    deepestNormal = pl - pab;
	    if(de == 0) {  //lines intersect -- what do we do?
	      deepestNormal.setZero();
	      deepestNormal.z = 1;
	    }
	    else deepestNormal /= de;
	  }
	}
	vertexClass[i->second.vinside] = Outside;
      }
      else {
	//LOG4CXX_INFO(KrisLibrary::logger(),"Hmm... the outside vertex is a fringe vertex, what do we do?");
      }
    }
    vertexClass[i->second.voutside] = Outside;
  }
  if(!IsInf(minEdgeDepth)) maxDepth = minEdgeDepth;
  for(Iterator i=collisions.begin();i!=collisions.end();) {
    int x = i->first;
    pair<Iterator,Iterator> range=collisions.equal_range(x);
    Assert(i->second.vinside == x);
    Assert(range.first == i);
    Assert(range.second != range.first);
    if(range.second != collisions.end())
      Assert(x != range.second->first);

    if(vertexClass[x] == Fringe) {
      const vector<int>& n=m1.vertexNeighbors[x];  

      //range contains all the edges that touch x
      Vector3 dxy;
      Real sumW=0;
      Real sumDepth=0;
      Vector3 sumNormal(Zero);
      for(Iterator j=range.first;j!=range.second;j++) {
	const EdgeCollision& ec = j->second;
	//add its contribution
	dxy.sub(ec.point,m1.verts[x]);
	Real ndxy = dxy.norm();
	ndxy = Max(ndxy,Epsilon);
	Real w = Inv(ndxy);
	sumW += w;
	sumNormal.madd(ec.normal,w);
	sumDepth += w*dot(ec.normal,dxy);
      }
      depth[x] = sumDepth/sumW;
      normal[x] = sumNormal/sumW;
      normal[x].inplaceNormalize();
      vertexClass[x] = Computed;
      
      if(depth[x] >= 0) {  //not a local minimum
	for(size_t j=0;j<n.size();j++) {
	  int y=n[j];
	  if(vertexClass[y] == Unvisited) {
	    //add to next fringe
	    fringe.push_back(y);
	    vertexClass[y] = Fringe;
	  }
	}
      }
      else {
	LOG4CXX_WARN(KrisLibrary::logger(),"PenetrationDepth::ComputeInitial(): Warning, initial point has a negative depth");
	KrisLibrary::loggerWait();
      }
      
      if(depth[x] > maxDepth) {
	maxDepth = depth[x];
	deepestPoint = m1.verts[x];
	deepestNormal = normal[x];
      }
    }

    i = range.second;
  }
}

void ApproximatePenetrationDepth::ComputeDepth()
{
  int visitedNodes=0;
  vector<int> nextFringe;
  while(!fringe.empty()) {
    for(vector<int>::iterator i=fringe.begin();i!=fringe.end();i++) {
      visitedNodes++;
      int x=*i;
      Assert(vertexClass[x] == Fringe);
      const vector<int>& n=m1.vertexNeighbors[x];      

      //weights w(y) = 1/|d(x,y)| (normalized)
      int numAdj=0;
      Vector3 dxy;
      Real sumW=0;
      Real sumDepth=0;
      Real maxNeighborDepth=0;
      Vector3 sumNormal(Zero);
      for(size_t j=0;j<n.size();j++) {
	int y=n[j];
	if(vertexClass[y] == Computed) {
	  //add its contribution
	  numAdj++;
	  dxy.sub(m1.verts[y],m1.verts[x]);
	  Real ndxy = dxy.norm();
	  ndxy = Max(ndxy,Epsilon);
	  Real w = Inv(ndxy);
	  sumW += w;
	  sumNormal.madd(normal[y],w);
	  sumDepth += w*(depth[y]+dot(normal[y],dxy));
	  maxNeighborDepth = Max(maxNeighborDepth,depth[y]);
	}
      }
      depth[x] = sumDepth/sumW;
      normal[x] = sumNormal/sumW;
      normal[x].inplaceNormalize();
      vertexClass[x] = Computed;
      Assert(numAdj != 0);
      
      if(depth[x] >= maxNeighborDepth) {  //not a local minimum
	for(size_t j=0;j<n.size();j++) {
	  int y=n[j];
	  if(vertexClass[y] == Unvisited) {
	    //add to next fringe
	    nextFringe.push_back(y);
	    vertexClass[y] = Fringe;
	  }
	}
      }

      if(depth[x] > maxDepth) {
	maxDepth = depth[x];
      }
    }
    fringe = nextFringe;
    nextFringe.resize(0);
  }

#if VERBOSE
  if(maxDepth != Zero)
    LOG4CXX_INFO(KrisLibrary::logger(),"MaxDepth "<<maxDepth<<", "<<visitedNodes<<" of "<<m1.verts.size());
#endif
}
