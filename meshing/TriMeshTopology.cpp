#include <KrisLibrary/Logger.h>
#include "TriMeshTopology.h"
#include <set>
#include <queue>
#include <algorithm>
#include <errors.h>
#include <iostream>
using namespace Meshing;

void TriMeshWithTopology::ClearTopology()
{
  vertexNeighbors.clear();
  incidentTris.clear();
  triNeighbors.clear();
}

void TriMeshWithTopology::CalcVertexNeighbors()
{
  vector<set<int> > neighborSets(verts.size());
  for(size_t i=0;i<tris.size();i++) {
    int a=tris[i].a,b=tris[i].b,c=tris[i].c;
    neighborSets[a].insert(b);
    neighborSets[a].insert(c);
    neighborSets[b].insert(a);
    neighborSets[b].insert(c);
    neighborSets[c].insert(a);
    neighborSets[c].insert(b);
  }
  vertexNeighbors.resize(verts.size());
  for(size_t i=0;i<verts.size();i++) {
    vertexNeighbors[i].resize(neighborSets[i].size());
    std::copy(neighborSets[i].begin(),neighborSets[i].end(),vertexNeighbors[i].begin());
  }
}


void TriMeshWithTopology::CalcIncidentTris()
{
  incidentTris.resize(verts.size());
  for(size_t i=0;i<verts.size();i++)
    incidentTris[i].resize(0);
  for(size_t i=0;i<tris.size();i++) {
    incidentTris[tris[i].a].push_back((int)i);
    incidentTris[tris[i].b].push_back((int)i);
    incidentTris[tris[i].c].push_back((int)i);
  }
}

void TriMeshWithTopology::CalcTriNeighbors()
{
  if(incidentTris.size()!=verts.size())
    CalcIncidentTris();

  size_t numDuplicateNeighbors = 0;
  size_t duplicateNeighborMin = tris.size();
  size_t duplicateNeighborMax = 0;
  triNeighbors.resize(tris.size());
  for(size_t i=0;i<tris.size();i++) {
    const TriMesh::Tri& t=tris[i];
    //search for all neighbors of t
    triNeighbors[i].set(-1,-1,-1);
    //use the incident tri list as a speedup
    //vertex a => edge ca, ab
    for(size_t j=0;j<incidentTris[t.a].size();j++) {
      int k=incidentTris[t.a][j];
      if(k==(int)i) continue; //skip this triangle
      const TriMesh::Tri& t2=tris[k];
      Assert(t2.contains(t.a));
      if(t2.contains(t.b)) {
	//edge ba
	if(triNeighbors[i].c!=-1) {
    numDuplicateNeighbors += 1;
    duplicateNeighborMin = Min(duplicateNeighborMin,i);
    duplicateNeighborMax = Max(duplicateNeighborMax,i);
	}
	triNeighbors[i].c=k;
      }
      if(t2.contains(t.c)) {
	//edge ac
	if(triNeighbors[i].b!=-1) {
	  numDuplicateNeighbors += 1;
    duplicateNeighborMin = Min(duplicateNeighborMin,i);
    duplicateNeighborMax = Max(duplicateNeighborMax,i);
	}
	triNeighbors[i].b=k;
      }
    }
    //vertex b => edge bc
    for(size_t j=0;j<incidentTris[t.b].size();j++) {
      int k=incidentTris[t.b][j];
      if(k==(int)i) continue; //skip this triangle
      const TriMesh::Tri& t2=tris[k];
      Assert(t2.contains(t.b));
      if(t2.contains(t.c)) {
	//edge bc
	if(triNeighbors[i].a!=-1) {
	  numDuplicateNeighbors += 1;
    duplicateNeighborMin = Min(duplicateNeighborMin,i);
    duplicateNeighborMax = Max(duplicateNeighborMax,i);
	}
	triNeighbors[i].a=k;
      }
    }
  }
  if(numDuplicateNeighbors>0) {
    LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: mesh has "<<numDuplicateNeighbors<<" triangles with duplicate neighbors!");
    LOG4CXX_WARN(KrisLibrary::logger(),"  Triangle range "<<duplicateNeighborMin<<" to "<<duplicateNeighborMax);
    LOG4CXX_WARN(KrisLibrary::logger(),"  May see strange results for some triangle mesh operations");
    //KrisLibrary::loggerWait();
  }
}

bool TriMeshWithTopology::IsConsistent()
{
  if(!vertexNeighbors.empty()) {
    vector<vector<int> > tempVertexNeighbors;
    swap(vertexNeighbors,tempVertexNeighbors);
    CalcVertexNeighbors();
    if(vertexNeighbors != tempVertexNeighbors)
      return false;
  }
  if(!incidentTris.empty()) {
    vector<vector<int> > tempIncidentTris;
    swap(incidentTris,tempIncidentTris);
    CalcIncidentTris();
    if(incidentTris != tempIncidentTris)
      return false;
  }
  if(!triNeighbors.empty()) {
    vector<TriNeighbors> tempTriNeighbors;
    swap(triNeighbors,tempTriNeighbors);
    CalcTriNeighbors();
    if(triNeighbors != tempTriNeighbors)
      return false;
  }
  return true;
}

template <class T>
void replace(std::vector<T>& v,const T& from,const T& to)
{
  typename std::vector<T>::iterator i=std::find(v.begin(),v.end(),from);
  Assert(i!=v.end());
  *i=to;
}

void TriMeshWithTopology::SplitEdge(int tri,int e,const Vector3& newPt)
{
  Assert(triNeighbors.size()==tris.size());
  int v=(int)verts.size();
  verts.push_back(newPt);
  
  int adj=triNeighbors[tri][e];
  int a,b,c,d;
  a=tris[tri].a;
  tris[tri].getCompliment(e,b,c);
  int ind1,ind2;
  if(!tris[adj].contains(b,ind1)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Internal inconsistency!");
    abort();
  }
  if(!tris[adj].contains(c,ind2)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Internal inconsistency!");
    abort();
  }
  int ea=3-ind1-ind2;
  d=tris[adj][ea];
  int t1 = triNeighbors[tri][(e+1)%3];
  int t2 = triNeighbors[tri][(e+2)%3];
  int t3 = triNeighbors[adj][(ea+1)%3];
  int t4 = triNeighbors[adj][(ea+2)%3];
  
  int r1=tri,r2=tris.size(),r3=tris.size(),r4=tris.size()+1;
  tris[tri].set(v,c,a);
  tris.push_back(IntTriple(v,a,b));
  tris[adj].set(v,b,d);
  tris.push_back(IntTriple(v,d,c));

  //set the adjacencies
  triNeighbors[r1].set(t1,r2,r4);
  triNeighbors[r2].set(t2,r3,r1);
  triNeighbors[r3].set(t3,r4,r2);
  triNeighbors[r4].set(t4,r1,r3);

  if(!incidentTris.empty()) {
    //indcidence changes
    //a: -= tri, += r1,r2
    //b: -= tri,adj, += r2,r3
    //c: -= tri,adj, += r4,r1
    //d: -= adj, += r3,r4
    //since r1=tri and r3=adj, we just have to set
    //a: += r2
    //b: replace tri with r2
    //c: replace adj with r4
    //d: += r4
    incidentTris[a].push_back(r2);
    replace(incidentTris[b],tri,r2);
    replace(incidentTris[c],adj,r4);
    incidentTris[d].push_back(r4);
    vector<int> vn(4);
    vn[0]=r1; vn[1]=r2; vn[2]=r3; vn[3]=r4;
    incidentTris.push_back(vn);
  }

  if(!vertexNeighbors.empty()) {
    vertexNeighbors[a].push_back(v);
    vertexNeighbors[b].push_back(v);
    vertexNeighbors[c].push_back(v);
    vertexNeighbors[d].push_back(v);
    vector<int> vn(4);
    vn[0]=a; vn[1]=b; vn[2]=c; vn[3]=d;
    vertexNeighbors.push_back(vn);
  }

  Assert(IsConsistent());
}

//flags for tri walk
//white is always 0
#define TriGray 0x1
#define TriBlack 0x2
#define TriMask 0x3
#define Edge1Black 0x4
#define Edge2Black 0x8
#define Edge3Black 0x10
#define EdgeBlack(e) (Edge1Black<<e)

#define IsTriWhite(x) ((x&TriMask) == 0)
#define IsTriGray(x) ((x&TriMask) == TriGray)
#define IsTriBlack(x) (x&TriBlack)
#define SetTriWhite(x) { x &= ~TriMask; }
#define SetTriGray(x) { x = (x & (~TriMask)) | TriGray; }
#define SetTriBlack(x) { x |= TriBlack; }
#define IsEdgeBlack(e,x) (x&EdgeBlack(e))
#define IsEdgeWhite(e,x) ((x&EdgeBlack(e)) == 0)
#define SetEdgeBlack(e,x) { x |= EdgeBlack(e); }
#define SetEdgeWhite(e,x) { x &= ~EdgeBlack(e); }




void TriMeshWithTopology::TriBFS(TriMeshTraversalCallback& callback)
{
  BeginTriWalk();
  int numComponents=0;
  for(size_t i=0;i<tris.size();i++) {
    //must be different component
    if(IsTriWhite(visited[i])) Assert(visited[i]==0);
    if(visited[i]==0) {
      callback.NewComponent(numComponents);
      numComponents++;
      _TriBFS((int)i,callback);
    }
  }
}

void TriMeshWithTopology::VertexBFS(TriMeshTraversalCallback& callback)
{
  BeginVertexWalk();
  int numComponents=0;
  for(size_t i=0;i<verts.size();i++) {
    if(visited[i]==0) {
      callback.NewComponent(numComponents);
      numComponents++;
      _VertexBFS((int)i,callback);
    }
  }
}

void TriMeshWithTopology::BeginTriWalk()
{
  if(triNeighbors.empty()) CalcTriNeighbors();
  Assert(triNeighbors.size()==tris.size());
  visited.resize(tris.size());  
  fill(visited.begin(),visited.end(),0);
}

void TriMeshWithTopology::BeginVertexWalk()
{
  if(vertexNeighbors.empty()) CalcVertexNeighbors();
  Assert(vertexNeighbors.size()==verts.size());
  visited.resize(verts.size());  
  fill(verts.begin(),verts.end(),0);
}

void TriMeshWithTopology::_TriBFS(int start,TriMeshTraversalCallback& callback)
{
  queue<int> q;
  q.push(start);

  //bfs
  int n;
  while(!q.empty()) {
    int t=q.front(); q.pop();
    SetTriBlack(visited[t]);
    callback.Tri(t);
    for(int i=0;i<3;i++) {
      n=triNeighbors[t][i];
      int v1,v2;
      tris[t].getCompliment(i,v1,v2);
      if(!IsEdgeBlack(i,visited[t])) {
	callback.TriArc(t,i);
	callback.Edge(v1,v2);
	SetEdgeBlack(i,visited[t]);
      }
      if(n!=-1) {
	//mark the edge black for n
	int nedgeindex=triNeighbors[n].getIndex(t);
	Assert(nedgeindex >= 0);
	SetEdgeBlack(nedgeindex,visited[n]);
	if(IsTriWhite(visited[n])) {
	  SetTriGray(visited[n]);
	  q.push(n);
	}
      }
    }
  }
}

void TriMeshWithTopology::_VertexBFS(int start,TriMeshTraversalCallback& callback)
{
  queue<int> q;
  q.push(start);

  //bfs
  int n;
  while(!q.empty()) {
    int v=q.front(); q.pop();
    visited[v]=2;
    callback.Vertex(v);
    for(size_t i=0;i<vertexNeighbors[v].size();i++) {
      n=vertexNeighbors[v][i];
      if(visited[n]==0) {
	visited[n]=1;
	callback.Edge(v,n);
	q.push(n);
      }
    }
  }
}
