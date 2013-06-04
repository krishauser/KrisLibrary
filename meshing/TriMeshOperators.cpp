#include "TriMeshOperators.h"
#include <set>

namespace Meshing {

///Returns true if the vertex is a boundary vertex
bool IncidentTriangleOrdering(const TriMeshWithTopology& mesh,int v,vector<list<int> >& triStrips)
{
  Assert(!mesh.incidentTris.empty());
  Assert(!mesh.triNeighbors.empty());
  set<int> incidentTris;
  for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
    int t=mesh.incidentTris[v][i];
    incidentTris.insert(t);
  }
  triStrips.resize(0);
  while(!incidentTris.empty()) {
    int t=*incidentTris.begin();
    triStrips.resize(triStrips.size()+1);
    triStrips.back().push_back(t);
    incidentTris.erase(incidentTris.begin());
    //go forward (CCW)
    int t0=t;
    for(;;) {  //iterate on t
      int n=CCWNeighbor(mesh,t,v);
      if(n == t0) return false;
      if(n>=0) {
	triStrips.back().push_back(n);
	set<int>::iterator i=incidentTris.find(n);
	Assert(i!=incidentTris.end());
	incidentTris.erase(i);
	t=n;
      }
      else break;
    }
    t=t0;  //set t back to the start
    //go backward (CW)
    for(;;) {  //iterate on t
      int n=CWNeighbor(mesh,t,v);
      if(n == t0) return false;
      if(n>=0) {
	triStrips.back().push_front(n);
	set<int>::iterator i=incidentTris.find(n);
	Assert(i!=incidentTris.end());
	incidentTris.erase(i);
	t=n;
      }
      else break;
    }
  }
  return true;
}

Real VertexGaussianCurvature(const TriMeshWithTopology& mesh,int v)
{
  Assert(!mesh.incidentTris.empty());
  vector<list<int> > strips;
  bool res=IncidentTriangleOrdering(mesh,v,strips);
  if(!res) { //non-boundary vertex
    Assert(strips.size()==1);
    Real sumAngles=0;
    for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
      //add the angle of the outgoing edges about v
      int t=mesh.incidentTris[v][i];
      const IntTriple& tri=mesh.tris[t];
      int vindex=tri.getIndex(v);
      Assert(vindex!=-1);
      int v1,v2;
      tri.getCompliment(vindex,v1,v2);
      Vector3 e1=mesh.verts[v1]-mesh.verts[v];
      Vector3 e2=mesh.verts[v2]-mesh.verts[v];
      Real a = Angle(e1,e2);
      sumAngles += a;
    }
    return 3.0*(TwoPi-sumAngles)/IncidentTriangleArea(mesh,v);
  }
  else {
    Real sum=0;
    for(size_t s=0;s<strips.size();s++) {
      Real subtendedAngle;
      int t0=*strips[s].begin();
      int tn=*(--strips[s].end());
      Assert(CWNeighbor(mesh,t0,v) <0);
      Assert(CCWNeighbor(mesh,tn,v) <0);
      int v0=CWAdjacentVertex(mesh,t0,v);
      int vn=CCWAdjacentVertex(mesh,tn,v);
      subtendedAngle = Angle(mesh.verts[vn]-mesh.verts[v],mesh.verts[v0]-mesh.verts[v]);

      Real sumAngles=0;
      for(list<int>::const_iterator i=strips[s].begin();i!=strips[s].end();i++) {
	int t=*i;
	//get vertices on t opposite v
	const IntTriple& tri=mesh.tris[t];
	int vindex=tri.getIndex(v);
	Assert(vindex!=-1);
	int v1,v2;
	tri.getCompliment(vindex,v1,v2);
	Vector3 e1=mesh.verts[v1]-mesh.verts[v];
	Vector3 e2=mesh.verts[v2]-mesh.verts[v];
	Real a = Angle(e1,e2);
	sumAngles += a;
      }
      sum += (subtendedAngle-sumAngles)*subtendedAngle/TwoPi;
    }
    return 3.0*sum/IncidentTriangleArea(mesh,v);
  }
}

Real VertexAbsMeanCurvature(const TriMeshWithTopology& mesh,int v)
{
  if(mesh.incidentTris[v].empty()) return 0;
  Real sum=0;
  //go around triangles in CCW order
  for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
    int t1=mesh.incidentTris[v][i];
    int t2=CCWNeighbor(mesh,t1,v);
    if(t2==-1) 
      continue;
    Vector3 n1=mesh.TriangleNormal(t1);
    Vector3 n2=mesh.TriangleNormal(t2);
    int e=mesh.triNeighbors[t1].getIndex(t2);  //edge index
    Assert(e!=-1);
    int v1,v2;
    mesh.tris[t1].getCompliment(e,v1,v2);
    Assert(v1 == v || v2 == v);
    if(v2 == v) swap(v1,v2);
    Real edgeLen = mesh.verts[v].distance(mesh.verts[v2]);
    //dihedral angle
    Real dihedral=Angle(n1,n2);
    sum += edgeLen*Abs(dihedral);
  }
  return 3.0*sum*0.25/IncidentTriangleArea(mesh,v);
}


} //namespace Meshing
