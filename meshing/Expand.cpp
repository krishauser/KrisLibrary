#include "Expand.h"
#include <utils/arrayutils.h>
#include <errors.h>

namespace Meshing {

struct TriMeshEdge
{
  int v1,v2;  //vertices from begin to end of edge
  int t1,t2;  //triangles on left and right side of edge (looking from outside)
  int e1,e2;  //edge of that triangle
};

void GetEdges(const TriMeshWithTopology& in,vector<TriMeshEdge>& edges)
{
  Assert(in.tris.size()==in.triNeighbors.size());
  edges.resize(0);
  edges.reserve(in.tris.size()*3/2);
  TriMeshEdge temp;
  for(int i=0;i<(int)in.tris.size();i++) {
    temp.t1 = i;
    temp.t2 = in.triNeighbors[i][0];
    if(temp.t2 > i || temp.t2 < 0) {
      temp.v1 = in.tris[i][1];
      temp.v2 = in.tris[i][2];
      temp.e1 = 0;
      if(temp.t2 >= 0) temp.e2 = in.triNeighbors[temp.t2].getIndex(i);
      edges.push_back(temp);
    }
    temp.t2 = in.triNeighbors[i][1];
    if(temp.t2 > i || temp.t2 < 0) {
      temp.v1 = in.tris[i][2];
      temp.v2 = in.tris[i][0];
      temp.e1 = 1;
      if(temp.t2 >= 0) temp.e2 = in.triNeighbors[temp.t2].getIndex(i);
      edges.push_back(temp);
    }
    temp.t2 = in.triNeighbors[i][2];
    if(temp.t2 > i || temp.t2 < 0) {
      temp.v1 = in.tris[i][0];
      temp.v2 = in.tris[i][1];
      temp.e1 = 2;
      if(temp.t2 >= 0) temp.e2 = in.triNeighbors[temp.t2].getIndex(i);
      edges.push_back(temp);
    }
  }  
}

void GetPairedEdges(const TriMeshWithTopology& in,vector<TriMeshEdge>& edges)
{
  GetEdges(in,edges);
  for(size_t i=0;i<edges.size();i++) {
    if(edges[i].t1 < 0 || edges[i].t2 < 0) {
      edges[i] = edges.back();
      i--;
      edges.resize(edges.size()-1);
    }
  } 
}

void Expand(const TriMeshWithTopology& in,Real distance,int divs,TriMesh& m)
{
  Assert(in.tris.size()==in.triNeighbors.size());
  Assert(in.verts.size()==in.incidentTris.size());
  if(divs > 1) {
    FatalError("Only support 1 division yet");
  }
  Assert(divs <= 1);
  vector<TriMeshEdge> edges;
  vector<IntTriple> edgeIndices(in.tris.size());
  GetPairedEdges(in,edges);
  for(size_t i=0;i<edges.size();i++) {
    edgeIndices[edges[i].t1][edges[i].e1] = i;
    edgeIndices[edges[i].t2][edges[i].e2] = i;
  }

  vector<Vector3> tripts(in.tris.size()*3);
  vector<Vector3> vertpts(in.verts.size());
  vector<Vector3> edgepts(edges.size()*2);
  for(size_t i=0;i<in.tris.size();i++) {
    Vector3 n=in.TriangleNormal(i)*distance;
    tripts[i*3] = in.TriangleVertex(i,0) + n;
    tripts[i*3+1] = in.TriangleVertex(i,1) + n;
    tripts[i*3+2] = in.TriangleVertex(i,2) + n;
  }
  for(size_t i=0;i<in.verts.size();i++) {
    Vector3 n(Zero);
    for(size_t j=0;j<in.incidentTris[i].size();j++) 
      n+=in.TriangleNormal(in.incidentTris[i][j]);
    n.inplaceNormalize();
    vertpts[i] = in.verts[i] + n * distance;
  }
  if(divs >= 1) {
    for(size_t i=0;i<edges.size();i++) {
      Vector3 n=in.TriangleNormal(edges[i].t1)+in.TriangleNormal(edges[i].t2);
      n.inplaceNormalize();
      edgepts[i*2] = in.verts[edges[i].v1] + n * distance;
      edgepts[i*2+1] = in.verts[edges[i].v2] + n * distance;
    }
  }
  m.verts = tripts;
  ArrayUtils::concat(m.verts,vertpts);
  ArrayUtils::concat(m.verts,edgepts);
  m.tris.resize(0);
  m.tris.reserve(in.tris.size() + 2*(1+divs)*edges.size() + in.verts.size());
  //triangle faces
  for(size_t i=0;i<in.tris.size();i++) 
    m.tris.push_back(IntTriple(i*3,i*3+1,i*3+2));
  //vertex faces
  int k1=(int)tripts.size();
  int k2=(int)tripts.size()+(int)vertpts.size();
  for(size_t i=0;i<in.verts.size();i++) {
    if(divs==1) {
      for(size_t j=0;j<in.incidentTris[i].size();j++) {
	int t=in.incidentTris[i][j];
	int vi=in.tris[t].getIndex(i);
	Assert(vi >= 0);
	int e1,e2;
	edgeIndices[in.incidentTris[i][j]].getCompliment(vi,e1,e2);
	int flip1 = (edges[e1].v1 == (int)i? 0 : 1);
	int flip2 = (edges[e2].v1 == (int)i? 0 : 1);
	m.tris.push_back(IntTriple(k1+i,t*3+vi,k2+e1*2+flip1));
	m.tris.push_back(IntTriple(k1+i,k2+e2*2+flip2,t*3+vi));
      }
    }
    else {
      vector<int> ring; ring.reserve(in.incidentTris[i].size());
      int t0=in.incidentTris[i][0];
      while(ring.size() < in.incidentTris[i].size()) {
	ring.push_back(t0);
	int vi=in.tris[t0].getIndex(i);
	t0=in.triNeighbors[t0][(vi+1)%3];
	if(ring.size() >= 2)
	  Assert(t0 != ring[ring.size()-2]);
      }
      vector<int> v(ring.size());
      for(size_t j=0;j<ring.size();j++) {
	int t=ring[j];
	int vi=in.tris[t].getIndex(i);
	Assert(vi >= 0);
	v[j] = t*3+vi;
      }
      for(size_t j=0;j<ring.size();j++) {
	int vi=v[j];
	int vn=v[(j+1)%ring.size()];
	m.tris.push_back(IntTriple(k1+i,vi,vn));
      }
    }
  }
  //edge faces
  for(size_t i=0;i<edges.size();i++) {
    int v11=(edges[i].e1+1)%3,v12=(edges[i].e1+2)%3;
    int v21=(edges[i].e2+1)%3,v22=(edges[i].e2+2)%3;
    int t1=edges[i].t1;
    int t2=edges[i].t2;
    int a=t1*3+v11,b=t1*3+v12,c=t2*3+v21,d=t2*3+v22;
    if(divs==0) {
      m.tris.push_back(IntTriple(a,c,b));
      m.tris.push_back(IntTriple(a,d,c));
    }
    else {
      int e=k2+i*2;
      int f=k2+i*2+1;
      m.tris.push_back(IntTriple(a,f,b));
      m.tris.push_back(IntTriple(a,e,f));
      m.tris.push_back(IntTriple(e,c,f));
      m.tris.push_back(IntTriple(e,d,c));
    }
  }
}

} //namespace Meshing
