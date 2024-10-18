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
        int t2a = t2.getIndex(t.a);
        int t2b = t2.getIndex(t.b);
        if((t2a + 1)%3 != t2b) { //forward ordering, invalid
          triNeighbors[i].c=k;
        }
      }
      if(t2.contains(t.c)) {
        //edge ac
        if(triNeighbors[i].b!=-1) {
          numDuplicateNeighbors += 1;
          duplicateNeighborMin = Min(duplicateNeighborMin,i);
          duplicateNeighborMax = Max(duplicateNeighborMax,i);
        }
        int t2a = t2.getIndex(t.a);
        int t2c = t2.getIndex(t.c);
        if((t2c + 1)%3 != t2a) { //forward ordering, invalid
          triNeighbors[i].b=k;
        }
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
        int t2c = t2.getIndex(t.c);
        int t2b = t2.getIndex(t.b);
        if((t2b + 1)%3 != t2c) { //forward ordering, invalid
          triNeighbors[i].a=k;
        }
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
    if(vertexNeighbors != tempVertexNeighbors) {
      LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: consistency error in vertex neighbors");
      return false;
    }
  }
  if(!incidentTris.empty()) {
    vector<vector<int> > tempIncidentTris;
    swap(incidentTris,tempIncidentTris);
    CalcIncidentTris();
    if(incidentTris.size() != tempIncidentTris.size()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: consistency error in incident tris size "<<incidentTris.size()<<" should be "<<tempIncidentTris.size());
      return false;
    }
    for(size_t i=0;i<incidentTris.size();i++) {
      sort(tempIncidentTris[i].begin(),tempIncidentTris[i].end());
      if(incidentTris[i] != tempIncidentTris[i]) {
        printf("Incident tris: ");
        for(auto j:tempIncidentTris[i])
          printf("%d ",j);
        printf("Should be: ");
        for(auto j:incidentTris[i])
          printf("%d ",j);
        printf("\n");
        LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: consistency error in incident tris of vertex "<<i);
        return false;
      }
    }
    for(size_t i=0;i<tris.size();i++) {
      const auto& a=incidentTris[tris[i].a];
      const auto& b=incidentTris[tris[i].b];
      const auto& c=incidentTris[tris[i].c];
      if(find(a.begin(),a.end(),int(i))==a.end()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: incident triangles of vertex "<<tris[i].a<<" doesn't contain "<<i);
      }
      if(find(b.begin(),b.end(),int(i))==b.end()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: incident triangles of vertex "<<tris[i].b<<" doesn't contain "<<i);
      }
      if(find(c.begin(),c.end(),int(i))==c.end()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: incident triangles of vertex "<<tris[i].c<<" doesn't contain "<<i);
      }
    }
  }
  if(!triNeighbors.empty()) {
    vector<TriNeighbors> tempTriNeighbors;
    swap(triNeighbors,tempTriNeighbors);
    CalcTriNeighbors();
    if(triNeighbors != tempTriNeighbors) {
      LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: consistency error in tri neighbors");
      if(triNeighbors.size() != tempTriNeighbors.size())
        printf("  Not equal size\n");
      else
        for(size_t i=0;i<triNeighbors.size();i++)
          if(triNeighbors[i] != tempTriNeighbors[i])
            cout<<"  Mismatch for tri "<<i<<": "<<triNeighbors[i]<<" vs "<<tempTriNeighbors[i]<<endl;
      return false;
    }
    for(size_t i=0;i<triNeighbors.size();i++) {
      for(int j=0;j<3;j++) {
        int a = triNeighbors[i][j];
        if(a >= 0) {
          if(triNeighbors[a].getIndex((int)i) < 0) {
            LOG4CXX_WARN(KrisLibrary::logger(),"TriMeshTopology: tri neighbor mismatch "<<i<<" "<<a<<" "<<triNeighbors[a][1]);
            return false;
          }
        }
      }
    }
  }
  return true;
}

int TriMeshWithTopology::MakeConsistent()
{
  ClearTopology();
  int numAdded = 0;
  vertexNeighbors.clear();
  CalcIncidentTris();
  triNeighbors.resize(tris.size());
  fill(triNeighbors.begin(),triNeighbors.end(),IntTriple(-1,-1,-1));
  bool manifold = true;
  set<int> suspectTris;
  for(size_t i=0;i<tris.size();i++) {
    TriMesh::Tri& t=tris[i];
    //use the incident tri list as a speedup
    //vertex a => edge ca, ab
    for(size_t j=0;j<incidentTris[t.a].size();j++) {
      int k=incidentTris[t.a][j];
      if(k==(int)i) continue; //skip this triangle
      TriMesh::Tri& t2=tris[k];
      Assert(t2.contains(t.a));
      if(t2.contains(t.b)) {
        //edge ba
        if(triNeighbors[i].c!=-1) {
          manifold = false;
          suspectTris.insert(k);
          //suspectTris.insert(i);
          //suspectTris.insert(triNeighbors[i].c);
        }
        else {
          int t2a = t2.getIndex(t.a);
          int t2b = t2.getIndex(t.b);
          if(t2a >= 0 && (t2a + 1)%3 == t2b) { //forward ordering, invalid
            triNeighbors[i].c=-1;
            suspectTris.insert(k);
            //suspectTris.insert(i);
          }
          else
            triNeighbors[i].c=k;
        }
      }
      if(t2.contains(t.c)) {
        //edge ac
        if(triNeighbors[i].b!=-1) {
          suspectTris.insert(k);
          //suspectTris.insert(i);
          //suspectTris.insert(triNeighbors[i].b);
          manifold = false;
        }
        else {
          int t2a = t2.getIndex(t.a);
          int t2c = t2.getIndex(t.b);
          if(t2c >= 0 && (t2c + 1)%3 == t2a) { //forward ordering, invalid
            suspectTris.insert(k);
            //suspectTris.insert(i);
          }
          else
            triNeighbors[i].b=k;
        }
      }
    }
    //vertex b => edge bc
    for(size_t j=0;j<incidentTris[t.b].size();j++) {
      int k=incidentTris[t.b][j];
      if(k==(int)i) continue; //skip this triangle
      TriMesh::Tri& t2=tris[k];
      Assert(t2.contains(t.b));
      if(t2.contains(t.c)) {
        //edge bc
        if(triNeighbors[i].a!=-1) {
          manifold = false;
          suspectTris.insert(k);
          //suspectTris.insert(i);
          //suspectTris.insert(triNeighbors[i].a);
        }
        else {
          int t2b = t2.getIndex(t.b);
          int t2c = t2.getIndex(t.c);
          if(t2b >= 0 && (t2b + 1)%3 == t2c) { //forward ordering, invalid
            suspectTris.insert(k);
            //suspectTris.insert(i);
          }
          else
            triNeighbors[i].a=k;
        }
      }
    }
    if(triNeighbors[i].a >= 0 && triNeighbors[i].a == triNeighbors[i].b && triNeighbors[i].a == triNeighbors[i].c) {
      manifold = false;
      suspectTris.insert(i);
      suspectTris.insert(triNeighbors[i].a);
    }
  }
  if(manifold) return 0;

  for(auto t: suspectTris) {
    IntTriple& tri = tris[t];
    int tstart = int(verts.size());
    verts.push_back(verts[tri.a]);
    verts.push_back(verts[tri.b]);
    verts.push_back(verts[tri.c]);
    tri.a = tstart;
    tri.b = tstart+1;
    tri.c = tstart+2;
  }
  triNeighbors.resize(0);
  ClearTopology();
  CalcIncidentTris();
  CalcTriNeighbors();
  return suspectTris.size()*3;
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
  
  // printf("Splitting edge %d of tri %d=(%d,%d,%d)\n",e,tri,tris[tri].a,tris[tri].b,tris[tri].c);
  int adj=triNeighbors[tri][e];
  int a,b,c,d;
  a=tris[tri][e];
  tris[tri].getCompliment(e,b,c);
  // printf("Apex of triangle %d, split edge %d %d\n",a,b,c);
  int ea=-1; //vertex index of apex of adjacent triangle
  if(adj >= 0) {
    int ind1,ind2;
    if(!tris[adj].contains(b,ind1)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TriMeshWithTopology: internal inconsistency!");
      abort();
    }
    if(!tris[adj].contains(c,ind2)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TriMeshWithTopology: internal inconsistency!");
      abort();
    }
    ea=3-ind1-ind2;
    d=tris[adj][ea];
    // printf("Apex of adjacent triangle %d\n",d);
  }
  else
    d = -1;
  int t1 = triNeighbors[tri][(e+1)%3];
  int t2 = triNeighbors[tri][(e+2)%3];
  // printf("Non-edge neighbors of triangle: %d %d\n",t1,t2);
  
  int r1=tri,r2=tris.size(),r3=-1,r4=-1;
  if(adj >= 0) {
    r3=adj;
    r4=tris.size()+1;
  }
  tris[tri].set(v,c,a);
  tris.push_back(IntTriple(v,a,b));
  triNeighbors.push_back(IntTriple(-1,-1,-1));
  //set the adjacencies
  triNeighbors[r1].set(t1,r2,r4);
  triNeighbors[r2].set(t2,r3,r1);
  if(t2 >= 0) {
    int t2ind=triNeighbors[t2].getIndex(r1);
    Assert(t2ind >= 0);
    triNeighbors[t2][t2ind] = r2; // replace neighbor with new triangle
  }
  // printf("New vertex %d\n",v);
  // printf("New triangle 1 (%d): %d %d %d\n",r1,tris[tri].a,tris[tri].b,tris[tri].c);
  // printf("New triangle 2 (%d): %d %d %d\n",r2,tris[r2].a,tris[r2].b,tris[r2].c);
  // printf("New triangle 1 neighbors: %d %d %d\n",triNeighbors[r1].a,triNeighbors[r1].b,triNeighbors[r1].c);
  // printf("New triangle 2 neighbors: %d %d %d\n",triNeighbors[r2].a,triNeighbors[r2].b,triNeighbors[r2].c);
  
  if(adj >= 0) {
    int t3 = triNeighbors[adj][(ea+1)%3];
    int t4 = triNeighbors[adj][(ea+2)%3];
    Assert(triNeighbors[t3].getIndex(adj) >= 0);
    Assert(triNeighbors[t4].getIndex(adj) >= 0);
    // printf("Non-edge neighbors of adjacent: %d %d\n",t3,t4);
    tris[adj].set(v,b,d);
    tris.push_back(IntTriple(v,d,c));
    // printf("New triangle 3 (%d): %d %d %d\n",r3,tris[r3].a,tris[r3].b,tris[r3].c);
    // printf("New triangle 4 (%d): %d %d %d\n",r4,tris[r4].a,tris[r4].b,tris[r4].c);
    triNeighbors.push_back(IntTriple(-1,-1,-1));
    triNeighbors[r3].set(t3,r4,r2);
    triNeighbors[r4].set(t4,r1,r3);
    if(t4 >= 0) {
      int t4ind=triNeighbors[t4].getIndex(r3);
      Assert(t4ind >= 0);
      triNeighbors[t4][t4ind] = r4; // replace neighbor with new triangle
    }
    // printf("New triangle 3 neighbors: %d %d %d\n",triNeighbors[r3].a,triNeighbors[r3].b,triNeighbors[r3].c);
    // printf("New triangle 4 neighbors: %d %d %d\n",triNeighbors[r4].a,triNeighbors[r4].b,triNeighbors[r4].c);
  }

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
    if(adj >= 0) {
      replace(incidentTris[c],adj,r4);
      incidentTris[d].push_back(r4);
    }
    vector<int> vn(2);
    vn.reserve(4);
    vn[0]=r1; vn[1]=r2;
    if(r3 >= 0) {
      vn.push_back(r3);
      vn.push_back(r4);
    }
    incidentTris.push_back(vn);
  }

  if(!vertexNeighbors.empty()) {
    vertexNeighbors[a].push_back(v);
    vertexNeighbors[b].push_back(v);
    vertexNeighbors[c].push_back(v);
    if(d >= 0)
      vertexNeighbors[d].push_back(v);
    vector<int> vn(4);
    vn[0]=a; vn[1]=b; vn[2]=c; vn[3]=d;
    vertexNeighbors.push_back(vn);
  }

  //Assert(IsConsistent());
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
