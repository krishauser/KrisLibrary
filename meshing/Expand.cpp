#include "Expand.h"
#include <math3d/geometry3d.h>
#include "MeshPrimitives.h"
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
  vector<IntTriple> edgeIndices(in.tris.size(),IntTriple(-1,-1,-1));
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
    vertpts[i] = in.verts[i] + n *distance;
  }
  if(divs >= 1) {
    for(size_t i=0;i<edges.size();i++) {
      Vector3 n=in.TriangleNormal(edges[i].t1)+in.TriangleNormal(edges[i].t2);
      n.inplaceNormalize();
      edgepts[i*2] = in.verts[edges[i].v1] + n * distance;
      edgepts[i*2+1] = in.verts[edges[i].v2] + n * distance;
    }
  }
  //vertex order: triangle points, vertex points, edge points
  m.verts = tripts;
  ArrayUtils::concat(m.verts,vertpts);
  ArrayUtils::concat(m.verts,edgepts);
  m.tris.resize(0);
  m.tris.reserve(in.tris.size() + 2*(1+divs)*edges.size() + in.verts.size());
  //triangle faces
  for(size_t i=0;i<in.tris.size();i++) 
    m.tris.push_back(IntTriple(i*3,i*3+1,i*3+2));
  //vertex faces
  int k1=(int)tripts.size();  //k1 is the start index of the vertex points
  int k2=(int)tripts.size()+(int)vertpts.size();  //k2 is the start index of the edge points
  for(size_t i=0;i<in.verts.size();i++) {
    if(divs==1) {
      for(size_t j=0;j<in.incidentTris[i].size();j++) {
	int t=in.incidentTris[i][j];
	int vi=in.tris[t].getIndex(i);
	Assert(vi >= 0);
	int e1,e2;
	edgeIndices[t].getCompliment(vi,e1,e2);
	if(e1 >= 0) {
	  int flip1 = (edges[e1].v1 == (int)i? 0 : 1);
	  m.tris.push_back(IntTriple(k1+i,t*3+vi,k2+e1*2+flip1));
	}
	if(e2 >= 0) {
	  int flip2 = (edges[e2].v1 == (int)i? 0 : 1);
	  m.tris.push_back(IntTriple(k1+i,k2+e2*2+flip2,t*3+vi));
	}
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

void Expand2Sided(const TriMeshWithTopology& in,Real distance,int divs,TriMesh& m)
{
  vector<TriMesh> expandedTris;
  expandedTris.reserve(in.tris.size());
  for(size_t i=0;i<in.tris.size();i++) {
    Vector3 n=in.TriangleNormal(i);
    Vector3 e1=in.verts[in.tris[i].b]-in.verts[in.tris[i].a];
    Vector3 e2=in.verts[in.tris[i].c]-in.verts[in.tris[i].b];
    Vector3 e3=in.verts[in.tris[i].a]-in.verts[in.tris[i].c];
    e1.inplaceNormalize();
    e2.inplaceNormalize();
    e3.inplaceNormalize();
    TriMesh tris;
    Triangle3D t;
    in.GetTriangle(i,t);
    tris.verts.resize(6);
    tris.verts[0] = t.a-n*distance;
    tris.verts[1] = t.b-n*distance;
    tris.verts[2] = t.c-n*distance;
    tris.verts[3] = t.a+n*distance;
    tris.verts[4] = t.b+n*distance;
    tris.verts[5] = t.c+n*distance;
    tris.tris.resize(2);
    tris.tris[0].set(0,2,1);
    tris.tris[1].set(3,4,5);

    //cylinders
    Vector3 d1,d2,d3;
    d1.setCross(e1,n);
    d2.setCross(e2,n);
    d3.setCross(e3,n);
    vector<vector<Vector3> > rows(divs);
    vector<vector<int> > rowsindices(divs);
    for(int k=0;k<divs;k++) {
      Real theta = Pi*Real(k+1)/Real(divs+1)-Pi*0.5;
      Real x=Cos(theta)*distance;
      Real y=Sin(theta)*distance;
      //edge a->b
      rows[k].push_back(t.a+d1*x+n*y);
      rowsindices[k].push_back(0);
      rows[k].push_back(t.b+d1*x+n*y);
      rowsindices[k].push_back(1);
      //transition from a->b to b->c
      Vector3 d12 = d1+d2; d12.inplaceNormalize();
      rows[k].push_back(t.b+d12*x+n*y);
      rowsindices[k].push_back(1);
      //edge b->a
      rows[k].push_back(t.b+d2*x+n*y);
      rowsindices[k].push_back(1);
      rows[k].push_back(t.c+d2*x+n*y);
      rowsindices[k].push_back(2);
      //transition from b->c to c->a 
      Vector3 d23 = d2+d3; d23.inplaceNormalize();
      rows[k].push_back(t.c+d23*x+n*y);
      rowsindices[k].push_back(2);
      //edge c->a
      rows[k].push_back(t.c+d3*x+n*y);
      rowsindices[k].push_back(2);
      rows[k].push_back(t.a+d3*x+n*y);
      rowsindices[k].push_back(0);
      //transition from c->a to a->b
      Vector3 d31 = d3+d1; d31.inplaceNormalize();
      rows[k].push_back(t.a+d31*x+n*y);
      rowsindices[k].push_back(0);
      Assert(rows.size()==rowsindices.size());

      /*
      //edge a->b
      tris.verts.push_back(t.a+d1*x+n*y);
      tris.verts.push_back(t.b+d1*x+n*y);
      int vn=(int)tris.verts.size();
      int vlast = vn-6;
      if(k==0) { //bottom row
	tris.tris.push_back(IntTriple(0,1,vn-1));
	tris.tris.push_back(IntTriple(0,vn-1,vn-2));
      }
      else {
	tris.tris.push_back(IntTriple(vlast-2,vlast-1,vn-1));
	tris.tris.push_back(IntTriple(vlast-2,vn-1,vn-2));
      }
      if(k+1==divs) { //top row
	tris.tris.push_back(IntTriple(vn-2,vn-1,4));
	tris.tris.push_back(IntTriple(vn-2,4,3));
      }

      //edge b->c
      tris.verts.push_back(t.b+d2*x+n*y);
      tris.verts.push_back(t.c+d2*x+n*y);
      vn=(int)tris.verts.size();
      int vlast = vn-6;
      if(k==0) { //bottom row
	tris.tris.push_back(IntTriple(1,2,vn-1));
	tris.tris.push_back(IntTriple(1,vn-1,vn-2));
      }
      else {
	tris.tris.push_back(IntTriple(vlast-2,vn-1-6,vn-1));
	tris.tris.push_back(IntTriple(vlast-2,vn-1,vn-2));
      }
      if(k+1==divs) { //top row
	tris.tris.push_back(IntTriple(vn-2,vn-1,5));
	tris.tris.push_back(IntTriple(vn-2,5,4));
      }

      //edge c->a
      tris.verts.push_back(t.c+d3*x+n*y);
      tris.verts.push_back(t.a+d3*x+n*y);
      vn=(int)tris.verts.size();
      int vlast = vn-6;
      if(k==0) { //bottom row
	tris.tris.push_back(IntTriple(2,0,vn-1));
	tris.tris.push_back(IntTriple(2,vn-1,vn-2));
      }
      else {
	tris.tris.push_back(IntTriple(vlast-2,vlast-1,vn-1));
	tris.tris.push_back(IntTriple(vlast-2,vn-1,vn-2));
      }
      if(k+1==divs) { //top row
	tris.tris.push_back(IntTriple(vn-2,vn-1,3));
	tris.tris.push_back(IntTriple(vn-2,3,5));
      }

      //temp: flat corners
      if(k==0) {
	//bottom row
	tris.tris.push_back(IntTriple(0,vn-6,vn-1));
	tris.tris.push_back(IntTriple(1,vn-4,vn-5));
	tris.tris.push_back(IntTriple(2,vn-2,vn-3));
      }
      else {
	tris.tris.push_back(IntTriple(vn-6-1,vn-6-6,vn-1));
	tris.tris.push_back(IntTriple(vn-1,vn-6-6,vn-6));
	tris.tris.push_back(IntTriple(vn-6-5,vn-6-4,vn-5));
	tris.tris.push_back(IntTriple(vn-5,vn-6-4,vn-4));
	tris.tris.push_back(IntTriple(vn-6-3,vn-6-2,vn-3));
	tris.tris.push_back(IntTriple(vn-3,vn-6-2,vn-2));
      }
      if(k+1 == divs) { //top row
	tris.tris.push_back(IntTriple(vn-1,vn-6,3));
	tris.tris.push_back(IntTriple(vn-5,vn-4,4));
	tris.tris.push_back(IntTriple(vn-3,vn-2,5));
      }
      */
    }
    //stitch together rows with triangles
    vector<size_t> rowstart(rows.size());
    for(size_t i=0;i<rows.size();i++) {
      rowstart[i] = tris.verts.size();
      tris.verts.insert(tris.verts.end(),rows[i].begin(),rows[i].end());
    }
    //bottom row
    for(size_t j=0;j<rows[0].size();j++) {
      size_t v = rowstart[0]+j;
      size_t vnext = rowstart[0]+(j+1)%rows[0].size();
      int vbottom = rowsindices[0][j];
      int vbottomnext = rowsindices[0][(j+1)%rows[0].size()];
      if(vbottom == vbottomnext) //single triangle
	tris.tris.push_back(IntTriple(vbottom,vnext,v));
      else {
	tris.tris.push_back(IntTriple(vbottom,vbottomnext,vnext));
	tris.tris.push_back(IntTriple(vbottom,vnext,v));
      }
    }
    for(size_t i=0;i+1<rows.size();i++) {
      for(size_t j=0;j<rows[i].size();j++) {
	size_t v = rowstart[i]+j;
	size_t vnext = rowstart[i]+(j+1)%rows[i].size();
	size_t vup = rowstart[i+1]+j;
	size_t vupnext = rowstart[i+1]+(j+1)%rows[i+1].size();
	tris.tris.push_back(IntTriple(v,vnext,vupnext));
	tris.tris.push_back(IntTriple(v,vupnext,vup));
      }
    }
    //upper row
    for(size_t j=0;j<rows.back().size();j++) {
      size_t v = rowstart.back()+j;
      size_t vnext = rowstart.back()+(j+1)%rows.back().size();
      int vup = rowsindices.back()[j]+3;
      int vupnext = rowsindices.back()[(j+1)%rows.back().size()]+3;
      if(vup == vupnext) { //single triangle
	tris.tris.push_back(IntTriple(vup,v,vnext));
      }
      else {
	tris.tris.push_back(IntTriple(v,vnext,vupnext));
	tris.tris.push_back(IntTriple(v,vupnext,vup));
      }
    }

    //spheres
    Real theta1 = Acos(d1.dot(d2));
    Real theta2 = Acos(d2.dot(d3));
    Real theta3 = Acos(d3.dot(d1));
    int slices1 = (int)Floor(theta1/Pi*divs);
    int slices2 = (int)Floor(theta2/Pi*divs);
    int slices3 = (int)Floor(theta3/Pi*divs);

    //TODO: round corners

    expandedTris.push_back(tris);
  }
  m.Merge(expandedTris);
}


} //namespace Meshing
