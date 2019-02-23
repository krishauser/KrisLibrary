#include <KrisLibrary/Logger.h>
#include "TriMesh.h"
#include <locale.h>
#include <iostream>
#include <utils/stringutils.h>
#include <math3d/geometry3d.h>
#include <math3d/misc.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <fstream>
#include <algorithm>
#include <errors.h>
namespace Meshing {

Vector3& TriMesh::TriangleVertex(int tri,int v)
{
  Assert(v>=0&&v<3);
  return verts[tris[tri][v]];
}

const Vector3& TriMesh::TriangleVertex(int tri,int v) const
{
  Assert(v>=0&&v<3);
  return verts[tris[tri][v]];
}

Vector3 TriMesh::TriangleNormal(int tri) const
{
  const Vector3& a = verts[tris[tri].a];
  const Vector3& b = verts[tris[tri].b];
  const Vector3& c = verts[tris[tri].c];
  return Triangle3D::normal(a,b,c);
}

void TriMesh::GetTriangle(int tri,Triangle3D& t) const
{
  t.a = verts[tris[tri].a];
  t.b = verts[tris[tri].b];
  t.c = verts[tris[tri].c];
}

void TriMesh::GetIncidentTris(int v,vector<int>& t) const
{
  t.resize(0);
  AppendIncidentTris(v,t);
}

void TriMesh::AppendIncidentTris(int v,vector<int>& t) const
{
  for(size_t i=0;i<tris.size();i++) {
    if(tris[i].contains(v)) t.push_back(i);
  }
}

void TriMesh::GetEdge(int tri,int e,int& v1,int& v2) const
{
  Assert(e>=0&&e<3);
  tris[tri].getCompliment(e,v1,v2);
}

int TriMesh::GetAdjacentTri(int tri,int e) const
{
  int a,b;
  GetEdge(tri,e,a,b);
  for(size_t i=0;i<tris.size();i++) {
    if((int)i==tri) continue;
    if(tris[i].contains(a) && tris[i].contains(b))
      return i;
  }
  return -1;
}

bool TriMesh::IsValid() const
{
  int numverts=(int)verts.size();
  bool res=true;
  for(size_t i=0;i<tris.size();i++) {
    for(int k=0;k<3;k++) {
      if(tris[i][k] < 0 || tris[i][k] >= numverts) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Invalid triangle "<<i<<" vertex "<<k<<": "<<tris[i][k]);
    res=false;
      }
    }
    if(tris[i].a == tris[i].b || 
       tris[i].a == tris[i].c ||
       tris[i].b == tris[i].c) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Degenerate triangle "<<i<<": "<<tris[i]);
      res=false;
    }
    /*
    Triangle3D t;
    GetTriangle(i,t);
    if(t.area() == Zero) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Degenerate triangle "<<tris[i]);
    }
    */
  }
  return res;
}

void TriMesh::GetAABB(Vector3& bmin, Vector3& bmax) const
{
  bmin.set(Inf);
  bmax.set(-Inf);
  for(size_t i=0;i<verts.size();i++) {
    bmin.setMinimum(verts[i]);
    bmax.setMaximum(verts[i]);
  } 
}

int TriMesh::ClosestPoint(const Vector3& pt,Vector3& cp) const
{
  int tmin=-1;
  Real dmin=Inf;
  Vector3 temp;
  Triangle3D tri;
  for(size_t i=0;i<tris.size();i++) {
    GetTriangle(i,tri);
    temp = tri.closestPoint(pt);
    Real d2=temp.distanceSquared(pt);
    if(d2 < dmin) {
      tmin = (int)i;
      dmin = d2;
      cp = temp;
    }
  }
  return tmin;
}

int TriMesh::RayCast(const Ray3D& r,Vector3& pt) const
{
  int tmin=-1;
  Real dmin=Inf;
  Real d;
  Vector2 uv;
  Triangle3D tri;
  for(size_t i=0;i<tris.size();i++) {
    GetTriangle(i,tri);
    if(tri.rayIntersects(r,&d,&uv.x,&uv.y)) {
      if(d < dmin) {
    tmin = (int)i;
    dmin = d;
    pt = tri.planeCoordsToPoint(uv);
      }
    }
  }
  return tmin;
}

bool TriMesh::Intersects(const Plane3D& p) const
{
  vector<Real> d(verts.size());
  for(size_t i=0;i<verts.size();i++)
    d[i] = p.distance(verts[i]);
  Real da,db,dc;
  for(size_t i=0;i<tris.size();i++) {
    da=d[tris[i].a];
    db=d[tris[i].b];
    dc=d[tris[i].c];
    if(da > Zero) {
      if(db <= Zero || dc <= Zero) return true;
    }
    else if(da < Zero) {
      if(db >= Zero || dc >= Zero) return true;
    }
    else //da==Zero
      return true;
  }
  return false;
}

bool TriMesh::PlaneSplits(const Plane3D& p,Real& dmin,Real& dmax) const
{
  dmin=Inf;
  dmax=-Inf;
  Real d;
  for(size_t i=0;i<verts.size();i++) {
    d = p.distance(verts[i]);
    if(d < dmin) dmin=d;
    if(d > dmax) dmax=d;
  }
  return (dmin <= Zero && dmax >= Zero);
}

void TriMesh::Transform(const Matrix4& mat) {
  Vector3 tmp;
  for(size_t i=0;i<verts.size();i++) {
    tmp = verts[i];
    mat.mulPoint(tmp,verts[i]);
  }
}

void TriMesh::FlipFaces() {
  for(size_t i=0;i<tris.size();i++) {
    std::swap(tris[i].b,tris[i].c);
  }
}

struct AddTriOffset
{
  AddTriOffset(int _ofs) :ofs(_ofs) {}
  TriMesh::Tri operator() (const TriMesh::Tri& t) const {
    TriMesh::Tri s;
    s.a = t.a+ofs;
    s.b = t.b+ofs;
    s.c = t.c+ofs;
    return s;
  }
  int ofs;
};

void TriMesh::Merge(const vector<TriMesh>& files)
{
  if(files.size() == 0) {
    verts.clear();
    tris.clear();
    return;
  }
  else if(files.size() == 1) {
    *this = files[0];
    return;
  }

  vector<int> vertoffsets(files.size());
  vector<int> trioffsets(files.size());
  int numverts = 0;
  int numtris = 0;
  //count stats
  for(size_t i=0;i<files.size();i++) {
    vertoffsets[i] = numverts;
    trioffsets[i] = numtris;
    numverts += files[i].verts.size();
    numtris += files[i].tris.size();
  }
  //copy data
  verts.resize(numverts);
  tris.resize(numtris);
  for(size_t i=0;i<files.size();i++) {
    copy(files[i].verts.begin(),files[i].verts.end(),
     verts.begin()+vertoffsets[i]);
    std::transform(files[i].tris.begin(),files[i].tris.end(),
     tris.begin()+trioffsets[i],AddTriOffset(vertoffsets[i]));
  }
}

void TriMesh::MergeWith(const TriMesh& mesh)
{
  size_t vertoffset = verts.size();
  size_t trioffset = tris.size();
  verts.insert(verts.end(),mesh.verts.begin(),mesh.verts.end());
  tris.insert(tris.end(),mesh.tris.begin(),mesh.tris.end());
  for(size_t t=trioffset;t<tris.size();t++) {
    tris[t].a += vertoffset;
    tris[t].b += vertoffset;
    tris[t].c += vertoffset;
  }
}

void TriMesh::RemoveUnusedVerts()
{
  vector<int> vertexMap(verts.size(),-1);
  vector<Vector3> newVerts(verts.size());
  int numVerts=0;
  for(size_t i=0;i<tris.size();i++) {
    for(int k=0;k<3;k++) {
      int v=tris[i][k];
      if(vertexMap[v]==-1) {
    vertexMap[v]=numVerts;
    newVerts[numVerts]=verts[v];
    numVerts++;
      }
      tris[i][k]=vertexMap[v];
    }
  }
  newVerts.resize(numVerts);
  swap(verts,newVerts);
}

bool LoadTriMesh(FILE* f,TriMesh& tri)
{
  int numverts,numtris;
  setlocale(LC_NUMERIC, "en_US.UTF-8");
  if(fscanf(f,"%d",&numverts) <= 0) return false;
  if(numverts <= 0 || numverts>10000000) {
    LOG4CXX_ERROR(KrisLibrary::logger(), "LoadTriMesh: Invalid number of vertices: "<<numverts);
    return false;
  }
      
  tri.verts.resize(numverts);
  for(int i=0;i<numverts;i++) {
    if(fscanf(f,"%lg %lg %lg",&tri.verts[i].x,&tri.verts[i].y,&tri.verts[i].z)<=0) {
      LOG4CXX_ERROR(KrisLibrary::logger(), "LoadTriMesh: Invalid vertex: "<<i);
      return false;
    }
  }
      
  if(fscanf(f,"%d",&numtris)<=0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"LoadTriMesh: Couldn't read num triangles");
    return false;
  }
  if(numtris <= 0 || numtris>10000000) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"LoadTriMesh: Invalid number of triangles: "<<numtris);
    return false;
  }
  tri.tris.resize(numtris);
  for(int i=0;i<numtris;i++) {
    if(fscanf(f,"%d %d %d",&tri.tris[i].a,&tri.tris[i].b,&tri.tris[i].c)<=0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR: Couldn't read triangle # "<<i);
      return false;
    }
  }
  if(!tri.IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: the triangle mesh is invalid or has degenerate triangles.");
    //LOG4CXX_ERROR(KrisLibrary::logger(),"Continuing may have unexpected results.");
    //KrisLibrary::loggerWait();
  }
  return true;
}

bool TriMesh::Load(const char* fn)
{
  /*
  ifstream in;
  in.open(fn);
  if(!in) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't open tri file "<<fn);
    return false;
  }
  in >> *this;
  if(!in) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't read tri file"<<fn);
    return false;
  }
  */
  FILE* f = fopen(fn,"r");
  if(!f) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't open tri file "<<fn);
    return false;
  }
  if(!LoadTriMesh(f,*this)) {
    fclose(f);
    return false;
  }
  fclose(f);
  return true;
}

bool TriMesh::Save(const char* fn) const
{
  ofstream out;
  out.open(fn);
  if(!out) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't open file for writing "<<fn);
    return false;
  }
  out << *this;
  if(!out) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't write tri file"<<fn);
    return false;
  }
  return true;
}

bool LoadMultipleTriMeshes(const char* fn,TriMesh& tri)
{
  /*
  ifstream in;
  in.open(fn);
  if(!in) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't open tri file "<<fn);
    return false;
  }
  */
  FILE* f = fopen(fn, "r");
  if (!f) {
      LOG4CXX_INFO(KrisLibrary::logger(), "Couldn't open tri file " << fn << "\n");
      return false;
  }
  vector<TriMesh> models;
  bool res;
  do {
    models.push_back(TriMesh());
    res = LoadTriMesh(f, models[models.size() - 1]);
    //in >> models[models.size()-1];
  //} while(in);
  } while (res);
  fclose(f);

  //TODO: detect errors with FILE
  if(true /* in.eof() */) {
    tri.Merge(models);
    return true;
  }
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error while loading tri file "<<fn);
    return false;
  }
  return true;
}

istream& operator >> (istream& in,TriMesh& tri) 
{
  int numverts=0,numtris=0;

  in >> numverts;      
  if(!in) return in;
  if(numverts <= 0 || numverts>10000000) {
    LOG4CXX_ERROR(KrisLibrary::logger(), "ERROR: Invalid number of vertices: "<<numverts);
    in.clear(ios::badbit);
    return in;
  }
      
  tri.verts.resize(numverts);
  for(int i=0;i<numverts;i++) {
    in >> tri.verts[i];
    if(!in)  return in;
  }
      
  in >> numtris;
  if(!in) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR: Couldn't read num triangles");
    in.clear(ios::badbit);
    return in;
  }
  if(numtris <= 0 || numtris>10000000) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR: Invalid number of triangles: "<<numtris);
    in.clear(ios::badbit);
    return in;
  }
  tri.tris.resize(numtris);
  for(int i=0;i<numtris;i++) {
    in >> tri.tris[i];
    if(!in) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR: Couldn't read triangle # "<<i);
      in.clear(ios::badbit);  
      return in;
    }
  }
  if(!tri.IsValid()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: the triangle mesh is invalid or has degenerate triangles.");
    LOG4CXX_ERROR(KrisLibrary::logger(),"Continuing may have unexpected results.");
    //KrisLibrary::loggerWait();
  }
  return in;
}

ostream& operator << (ostream& out,const TriMesh& tri)
{
  out << tri.verts.size() << "\n";
  for(size_t i=0;i<tri.verts.size();i++)
    out << tri.verts[i] << "\n";
  out << tri.tris.size() << "\n";
  for(size_t i=0;i<tri.tris.size();i++)
    out << tri.tris[i] << "\n";
  return out;
}

} //namespace Meshing
