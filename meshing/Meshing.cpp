#include <KrisLibrary/Logger.h>
#include "Meshing.h"
#include <math3d/interpolate.h>
#include <math3d/Plane3D.h>
#include <math3d/Plane2D.h>
#include <utils/shift.h>
#include <iostream>
#include <errors.h>
#include <queue>

namespace Meshing {

//if segment is x = a + u*(b-a), returns the value u s.t. x=0
inline Real SegmentInterpolation(Real a,Real b)
{
  if(a == b) return Zero;
  return a/(a-b);
}

//returns 0 if the best split is a-c, else b-d
inline int DelaunaySplit(const Vector3& a,const Vector3& b,const Vector3& c,const Vector3& d)
{
  Real d0=a.distanceSquared(c);
  Real d1=b.distanceSquared(d);
  if(d0<=d1) return 0;
  else return 1;
}

//returns 0 if the best split is a-c, else b-d
inline int DelaunaySplit(const Vector2& a,const Vector2& b,const Vector2& c,const Vector2& d)
{
  Real d0=a.distanceSquared(c);
  Real d1=b.distanceSquared(d);
  if(d0<=d1) return 0;
  else return 1;
}

inline int OddMan(Real a,Real b,Real c,Real tol)
{
  //cases:
  //--- = none
  //--0 = none
  //--+ = +
  //-00 = none
  //-0+ = either - or +
  //-++ = -
  //000 = none
  //00+ = none
  //0++ = none
  //+++ = none
  int sign[3];
  sign[0] = (a <= -tol? -1 : (a >= tol ? 1 : 0));
  sign[1] = (b <= -tol? -1 : (b >= tol ? 1 : 0));
  sign[2] = (c <= -tol? -1 : (c >= tol ? 1 : 0));
  int negcount=0;
  int poscount=0;
  for(int i=0;i<3;i++) {
    if(sign[i] < 0) negcount++;
    else if(sign[i] > 0) poscount++;
  }
  if(negcount && poscount) {
    if(negcount > poscount) {
      for(int i=0;i<3;i++) if(sign[i] > 0) return i;
    }
    else {
      for(int i=0;i<3;i++) if(sign[i] < 0) return i;
    }
  }
  return -1;
}

int SplitTriangle(const Triangle3D& _t,const Plane3D& p,Vector3 newPts[2],IntTriple newTris[3],bool triPositive[3],Real tol) 
{
  Triangle3D t;//=_t;
  int indices[3]; //={0,1,2};
  Real d[3];
  d[0] = p.distance(_t.a);
  d[1] = p.distance(_t.b);
  d[2] = p.distance(_t.c);
  //find the "odd man" on the opposite side of the plane as the other 2
  //rotate the points so index 0 is the odd man
  int oddman = OddMan(d[0],d[1],d[2],tol);
  if(oddman == -1) {
    newTris[0].set(0,1,2);
    triPositive[0]=(d[0]>=-tol); 
    return 1; 
  }
  else if(oddman == 2) {
    //oddMan=2, rotate a=c, b=a, c=b;
    ShiftForward(d[0],d[1],d[2]);
    //ShiftForward(t.a,t.b,t.c);
    //ShiftForward(indices[0],indices[1],indices[2]);
    indices[0]=2;  indices[1]=0;  indices[2]=1;
    t.a = _t.c;  t.b = _t.a;  t.c = _t.b;
  }
  else if(oddman == 1) {
    //oddMan=1, rotate a=b, b=c, c=a
    ShiftBackward(d[0],d[1],d[2]);
    //ShiftBackward(t.a,t.b,t.c);
    //ShiftBackward(indices[0],indices[1],indices[2]);
    indices[0]=1;  indices[1]=2;  indices[2]=0;
    t.a = _t.b;  t.b = _t.c;  t.c = _t.a;
  }
  else {
    //oddMan=0, no change
    //d stays the same
    indices[0]=0;  indices[1]=1;  indices[2]=2;
    t.a = _t.a;  t.b = _t.b;  t.c = _t.c;
  }
  triPositive[0] = (d[0]>=-tol);
  Assert(!FuzzyZero(d[0],tol));
  if(FuzzyZero(d[1],tol)) {
    Assert((d[2] >= tol) != (d[0] >= tol));  //other cases should be taken care of already
    Assert(!FuzzyZero(d[2],tol));
    //only split edge 0,2
    Real u=SegmentInterpolation(d[0],d[2]);
    interpolate(t.a,t.c,u,newPts[0]);
    newTris[0].set(indices[0],indices[1],3);
    newTris[1].set(indices[1],indices[2],3);
    triPositive[1] = !triPositive[0];
    return 2;
  }
  else if(FuzzyZero(d[2],tol)) {
    Assert((d[1] >= tol) != (d[0] >= tol));
    //only split edge 0,1
    Real u=SegmentInterpolation(d[0],d[1]);
    interpolate(t.a,t.b,u,newPts[0]);
    newTris[0].set(indices[0],3,indices[2]);
    newTris[1].set(indices[2],3,indices[1]);
    triPositive[1] = !triPositive[0];
    return 2;
  }
  Assert((d[1] >= tol) != (d[0] >= tol));
  Assert((d[2] >= tol) != (d[0] >= tol));
  //split both edge 1 and 2
  Real u1=SegmentInterpolation(d[0],d[1]);
  Real u2=SegmentInterpolation(d[0],d[2]);
  interpolate(t.a,t.b,u1,newPts[0]);
  interpolate(t.a,t.c,u2,newPts[1]);
  newTris[0].set(indices[0],3,4);
  //pick the delaunay triangulation of the remaining 4 points
  int choice=DelaunaySplit(newPts[0],t.b,t.c,newPts[1]);
  if(choice==0) { //3 to 2
    newTris[1].set(3,indices[1],indices[2]);
    newTris[2].set(3,indices[2],4);
  }
  else { //4 to 1
    newTris[1].set(4,3,indices[1]);
    newTris[2].set(4,indices[1],indices[2]);
  }
  triPositive[1] = !triPositive[0];
  triPositive[2] = !triPositive[0];
  return 3;
}


int SplitTriangle(const Triangle2D& _t,const Plane2D& p,Vector2 newPts[2],IntTriple newTris[3],bool triPositive[3],Real tol) 
{
  Triangle2D t; //=_t; 
  int indices[3]; //={0,1,2}
  Real d[3];
  d[0] = p.distance(t.a);
  d[1] = p.distance(t.b);
  d[2] = p.distance(t.c);
  //find the "odd man" on the opposite side of the plane as the other 2
  //rotate the points so index 0 is the odd man
  int oddman = OddMan(d[0],d[1],d[2],tol);
  if(oddman == -1) {
    newTris[0].set(0,1,2);
    triPositive[0]=(d[0]>=-tol); 
    return 1; 
  }
  else if(oddman == 2) {
    //oddMan=2, rotate a=c, b=a, c=b;
    ShiftForward(d[0],d[1],d[2]);
    //ShiftForward(t.a,t.b,t.c);
    //ShiftForward(indices[0],indices[1],indices[2]);
    indices[0]=2;  indices[1]=0;  indices[2]=1;
    t.a = _t.c;  t.b = _t.a;  t.c = _t.b;
  }
  else if(oddman == 1) {
    //oddMan=1, rotate a=b, b=c, c=a
    ShiftBackward(d[0],d[1],d[2]);
    //ShiftBackward(t.a,t.b,t.c);
    //ShiftBackward(indices[0],indices[1],indices[2]);
    indices[0]=1;  indices[1]=2;  indices[2]=0;
    t.a = _t.b;  t.b = _t.c;  t.c = _t.a;
  }
  else {
    //oddMan=0, no change
    //d stays the same
    indices[0]=0;  indices[1]=1;  indices[2]=2;
    t.a = _t.a;  t.b = _t.b;  t.c = _t.c;
  }
  triPositive[0] = (d[0]>=-tol);
  Assert(!FuzzyZero(d[0]));
  if(FuzzyZero(d[1],tol)) {
    Assert((d[2] >= tol) != (d[0] >= tol));  //other cases should be taken care of already
    Assert(!FuzzyZero(d[2],tol));
    //only split edge 0,2
    Real u=SegmentInterpolation(d[0],d[2]);
    interpolate(t.a,t.c,u,newPts[0]);
    newTris[0].set(indices[0],indices[1],3);
    newTris[1].set(indices[1],indices[2],3);
    triPositive[1] = !triPositive[0];
    return 2;
  }
  else if(FuzzyZero(d[2],tol)) {
    Assert((d[1] >= tol) != (d[0] >= tol));
    //only split edge 0,1
    Real u=SegmentInterpolation(d[0],d[1]);
    interpolate(t.a,t.b,u,newPts[0]);
    newTris[0].set(indices[0],3,indices[2]);
    newTris[1].set(3,indices[1],indices[2]);
    triPositive[1] = !triPositive[0];
    return 2;
  }
  Assert((d[1] >= tol) != (d[0] >= tol));
  Assert((d[2] >= tol) != (d[0] >= tol));
  //split both edge 1 and 2
  Real u1=SegmentInterpolation(d[0],d[1]);
  Real u2=SegmentInterpolation(d[0],d[2]);
  interpolate(t.a,t.b,u1,newPts[0]);
  interpolate(t.a,t.c,u2,newPts[1]);
  newTris[0].set(indices[0],3,4);
  //pick the delaunay triangulation of the remaining 4 points
  int choice=DelaunaySplit(newPts[0],t.b,t.c,newPts[1]);
  if(choice==0) { //3 to 2
    newTris[1].set(3,indices[1],indices[2]);
    newTris[2].set(3,indices[2],4);
  }
  else { //4 to 1
    newTris[1].set(4,3,indices[1]);
    newTris[2].set(4,indices[1],indices[2]);
  }
  triPositive[1] = !triPositive[0];
  triPositive[2] = !triPositive[0];
  return 3;
}

TriSplitter::TriSplitter(TriMeshWithTopology& _mesh)
  :mesh(_mesh),tol(1e-4), deleteNegative(false)
{
  positive.resize(mesh.tris.size(),true);
  origTri.resize(mesh.tris.size());
  for(size_t i=0;i<mesh.tris.size();i++)
    origTri[i] = (int)i;
}

struct SplitCallback : public TriMeshTraversalCallback
{
  SplitCallback(TriSplitter& _splitter,const Plane3D& _p)
    :splitter(_splitter),p(_p)
  {}

  virtual void TriArc(int _t,int _e) {
    t=_t;
    e=_e;
  }

  virtual void Edge(int v1,int v2) {
    Assert(v1 == splitter.mesh.tris[t][(e+1)%3]);
    Assert(v2 == splitter.mesh.tris[t][(e+2)%3]);
    Real d1=splitter.d[v1];
    Real d2=splitter.d[v2];
    if((d1 < -splitter.tol && d2 > splitter.tol) ||
       (d1 > splitter.tol && d2 <-splitter.tol)) {
      Vector3 x;
      interpolate(splitter.mesh.verts[v1],splitter.mesh.verts[v2],
		  SegmentInterpolation(d1,d2),x);
      int v=(int)splitter.mesh.verts.size();
      splitter.mesh.verts.push_back(x);
      splitter.newVerts[t].push_back(pair<int,int>(e,v));
      int n=splitter.mesh.triNeighbors[t][e];
      if(n != -1) {
	int ne=splitter.mesh.triNeighbors[n].getIndex(t);
	Assert(ne != -1);
	splitter.newVerts[n].push_back(pair<int,int>(ne,v));
      }
    }
  }

  TriSplitter& splitter;
  const Plane3D& p;
  int t,e;
};

void TriSplitter::Split(const Plane3D& p)
{
  Assert(positive.size()==mesh.tris.size());
  Assert(origTri.size()==mesh.tris.size());
  d.resize(mesh.verts.size());
  newVerts.resize(mesh.tris.size());
  for(size_t i=0;i<mesh.verts.size();i++)
    d[i] = p.distance(mesh.verts[i]);
  for(size_t i=0;i<mesh.tris.size();i++)
    newVerts[i].resize(0);

  SplitCallback callback(*this,p);
  mesh.TriBFS(callback);

  for(size_t i=0;i<newVerts.size();i++) {
    bool ipositive=positive[i];
    int itriorig=origTri[i];
    //remesh the triangle with the new vertices
    Assert(newVerts[i].size() <= 2);
    int e1,e2,v1,v2;
    int a,b,c;
    const TriMesh::Tri& t=mesh.tris[i];
    switch(newVerts[i].size()) {
    case 0:
      //set positive=false if it's on the negative side of the plane
      if(d[t.a] < -tol || d[t.b] < -tol || d[t.c] < -tol)
	positive[i]=false;
      break;
    case 1:
      //split triangle in half
      e1=newVerts[i][0].first;
      v1=newVerts[i][0].second;
      a=t[e1];
      if(d[a] <= -tol || d[a] >= tol) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error with the vertex split!");
	LOG4CXX_INFO(KrisLibrary::logger(),"Triangle "<<mesh.tris[i]);
	LOG4CXX_INFO(KrisLibrary::logger(),"Distances "<<d[mesh.tris[i][0]]<<", "<<d[mesh.tris[i][1]]<<", "<<d[mesh.tris[i][2]]);
	LOG4CXX_INFO(KrisLibrary::logger(),"Edge "<<e1);
	LOG4CXX_INFO(KrisLibrary::logger(),"Split vertex "<<a);
	KrisLibrary::loggerWait();
      }
      Assert(d[a] >= -tol && d[a] <= tol);
      t.getCompliment(e1,b,c);
      mesh.tris[i].set(a,b,v1);
      mesh.tris.push_back(IntTriple(a,v1,c));
      origTri.push_back(itriorig);

      if(d[b] <= -tol) {
	positive[i]=false;
	Assert(d[c] >= tol);
	positive.push_back(ipositive);
      }
      else {
	Assert(d[c] <= -tol);
	positive.push_back(false);
      }
      break;
    case 2:
      //split triangle in 2 parts
      e1=newVerts[i][0].first;
      v1=newVerts[i][0].second;
      e2=newVerts[i][1].first;
      v2=newVerts[i][1].second;
      Assert(e1 >= 0 && e1 < 3);
      Assert(e2 >= 0 && e2 < 3);
      //make sure a->b,a->c have the splits
      if((e1+1)%3 != e2) {
	Assert((e2+1)%3 == e1);
	swap(e1,e2);
	swap(v1,v2);
      }
      a=t.getCompliment(e1,e2);
      b=t[e1];
      c=t[e2];
      mesh.tris[i].set(a,v2,v1);
      if(DelaunaySplit(mesh.verts[v1],mesh.verts[v2],mesh.verts[b],mesh.verts[c])==0) {
	mesh.tris.push_back(IntTriple(v1,v2,b));
	mesh.tris.push_back(IntTriple(v1,b,c));
      }
      else {
	mesh.tris.push_back(IntTriple(v2,b,c));
	mesh.tris.push_back(IntTriple(v2,c,v1));
      }
      origTri.push_back(itriorig);
      origTri.push_back(itriorig);
      if(d[a] <= -tol) {
	positive[i]=false;
	Assert(d[c] >= tol);
	Assert(d[b] >= tol);
	positive.push_back(ipositive); positive.push_back(ipositive);
      }
      else {
	Assert(d[c] <= -tol);
	Assert(d[b] <= -tol);
	positive.push_back(false); positive.push_back(false);
      }
      break;
    }
  }
  if(deleteNegative) {
    vector<IntTriple> newTris;
    vector<int> newOrigTri;
    newTris.reserve(mesh.tris.size()/2);
    newOrigTri.reserve(mesh.tris.size()/2);
    for(size_t i=0;i<mesh.tris.size();i++)
      if(positive[i]) {
	newTris.push_back(mesh.tris[i]);
	newOrigTri.push_back(origTri[i]);
      }
    swap(mesh.tris,newTris);
    swap(origTri,newOrigTri);
    positive.resize(mesh.tris.size());
    fill(positive.begin(),positive.end(),true);
  }
  //hmm... could preserve topology, but might be easier just to do this
  if(!mesh.vertexNeighbors.empty()) mesh.CalcVertexNeighbors();
  if(!mesh.incidentTris.empty()) mesh.CalcIncidentTris();
  if(!mesh.triNeighbors.empty()) mesh.CalcTriNeighbors();
}

void GetCoplanarTris(const TriMesh& mesh,int t,Real tol,vector<int>& tris)
{
  tris.resize(0);
  Plane3D p;
  Triangle3D tri;
  mesh.GetTriangle(t,tri);
  tri.getPlane(p);
  vector<bool> withinTol(mesh.verts.size());
  for(size_t i=0;i<mesh.verts.size();i++)
    withinTol[i] = FuzzyZero(p.distance(mesh.verts[i]),tol);
  for(size_t i=0;i<mesh.tris.size();i++) {
    bool coplanar=(withinTol[mesh.tris[i].a]&&
		   withinTol[mesh.tris[i].b]&&
		   withinTol[mesh.tris[i].c]);
    if(coplanar && p.normal.dot(mesh.TriangleNormal(i)) > 0) {
      tris.push_back(i);
    }
  }
}

void GetConnectedCoplanarTris(TriMeshWithTopology& mesh,int t,Real tol,vector<int>& tris)
{
  Assert(mesh.triNeighbors.size()==mesh.tris.size());
  tris.resize(0);
  Plane3D p;
  Triangle3D tri;
  mesh.GetTriangle(t,tri);
  tri.getPlane(p);
  mesh.BeginTriWalk();
  
  queue<int> q;
  q.push(t);
  while(!q.empty()) {
    int t=q.front(); q.pop();
    mesh.visited[t]=2;
    tris.push_back(t);

    for(int e=0;e<3;e++) {
      int n=mesh.triNeighbors[t][e];
      if(mesh.visited[n] != 0) continue;

      int ne=mesh.triNeighbors[n].getIndex(t);
      Assert(ne != -1);
      if(FuzzyZero(p.distance(mesh.verts[mesh.tris[n][ne]]),tol)) {
	mesh.visited[n]=1;
	q.push(n);
      }
    }
  }
}

/*
void GetConnectedCoplanarTris(TriMeshWithTopology& mesh,int t,Real tol,PolygonWithHoles2D& poly)
{
  vector<int> tris;
  GetConnectedCoplanarTris(mesh,t,tol,tris);

  vector<pair<int,int> > edges;
  //use the mesh.visited structure from the previous call
  for(size_t i=0;i<tris.size();i++) {
    Assert(mesh.visited[tris[i]]==2);
    for(int e=0;e<3;e++) {
      if(mesh.visited[mesh.triNeighbors[i][e]] != 2) {
	int v1,v2;
	mesh.tris[t].getCompliment(e,v1,v2);
	edges.push_back(pair<int,int>(v1,v2));
      }
    }
  }

  //extract cycles from the edge list
  LOG4CXX_INFO(KrisLibrary::logger(),"TODO: extract cycles from edge list");
  abort();
}
*/

} //namespace Meshing
