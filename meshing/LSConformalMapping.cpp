#include <KrisLibrary/Logger.h>
#include "LSConformalMapping.h"
#include <optimization/LSQRInterface.h>
#include <math/MatrixPrinter.h>
#include <iostream>
using namespace Meshing;

LSConformalMapping::LSConformalMapping(TriMeshWithTopology& _mesh,TriMeshChart& _chart)
  :mesh(_mesh),chart(_chart)
{}

bool LSConformalMapping::Calculate()
{
  if(mesh.tris.empty()) return false;
  LOG4CXX_INFO(KrisLibrary::logger(),"LSConformalMapping: building...");
  SparseMatrix Mre,Mim;
  Mre.resize(mesh.tris.size(),mesh.verts.size());
  Mim.resize(mesh.tris.size(),mesh.verts.size());
  assert(mesh.tris.size()+2 >= mesh.verts.size());
  Triangle3D tri;
  for(size_t i=0;i<mesh.tris.size();i++) {
    const IntTriple& T=mesh.tris[i];
    mesh.GetTriangle(i,tri);
    Real area = tri.area();
    if(area == Zero) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error: degenerate triangle!");
      return false;
    }
    Real invsqrtarea = One/Sqrt(Two*area);
    Vector3 n=tri.normal(),xb,yb;
    n.getOrthogonalBasis(xb,yb);
    Vector2 p2,p3;  //local coordinates of p2,p3
    p2.x = xb.dot(mesh.verts[T.b]-mesh.verts[T.a]);
    p2.y = yb.dot(mesh.verts[T.b]-mesh.verts[T.a]);
    p3.x = xb.dot(mesh.verts[T.c]-mesh.verts[T.a]);
    p3.y = yb.dot(mesh.verts[T.c]-mesh.verts[T.a]);
    Mre(i,T.a) = (p2.x-p3.x)*invsqrtarea;
    Mim(i,T.a) = (p2.y-p3.y)*invsqrtarea;
    Mre(i,T.b) = (p3.x)*invsqrtarea;
    Mim(i,T.b) = (p3.y)*invsqrtarea;
    Mre(i,T.c) = (-p2.x)*invsqrtarea;
    Mim(i,T.c) = (-p2.y)*invsqrtarea;
  }
  int npinned=2;
  int vpinned[2];
  Vector2 Upinned[2];
  Upinned[0].setZero();

  //pin two vertices -- pick the approximately most distant
  Vector3 bmin(Inf),bmax(-Inf);
  size_t vmin[3],vmax[3];
  for(size_t i=0;i<mesh.verts.size();i++) {
    if(mesh.verts[i].x < bmin.x) {
      bmin.x = mesh.verts[i].x;
      vmin[0] = i;
    }
    if(mesh.verts[i].x > bmax.x) {
      bmax.x = mesh.verts[i].x;
      vmax[0] = i;
    }
    if(mesh.verts[i].y < bmin.y) {
      bmin.y = mesh.verts[i].y;
      vmin[1] = i;
    }
    if(mesh.verts[i].y > bmax.y) {
      bmax.y = mesh.verts[i].y;
      vmax[1] = i;
    }
    if(mesh.verts[i].z < bmin.z) {
      bmin.z = mesh.verts[i].z;
      vmin[2] = i;
    }
    if(mesh.verts[i].z > bmax.z) {
      bmax.z = mesh.verts[i].z;
      vmax[2] = i;
    }
  }
  Vector3 dims=bmax-bmin;
  int maxdim=0;
  if(dims.x > dims.y) {
    if(dims.x > dims.z) maxdim=0;
    else maxdim=2;
  }
  else {
    if(dims.y > dims.z) maxdim=1;
    else maxdim=2;
  }  
  vpinned[0] = vmin[maxdim];   vpinned[1] = vmax[maxdim];
  Upinned[1].set(0,dims[maxdim]);

  vector<int> verttox(mesh.verts.size(),-1);  //mapping of vertices to unpinned dims of x
  vector<int> xtovert(Mre.n-npinned);         //mapping of unpinned dims of x to vertices
  int k=0;
  for(int j=0;j<Mre.n;j++) {
    assert(npinned==2);
    if(j != vpinned[0] && j != vpinned[1]) {
      verttox[j] = k;
      xtovert[k] = j;
      k++;
    }
  }
  assert(k == Mre.n-npinned);

  LOG4CXX_INFO(KrisLibrary::logger(),"LSConformalMapping: creating A,b...");
  //create the A,b matrices
  SparseMatrix A;
  Vector b;
  A.resize(Mre.m*2,(Mre.n-npinned)*2);
  b.resize(Mre.m*2);
  for(int i=0;i<Mre.m;i++) {
    SparseMatrix::ConstRowIterator ire,iim;
    for(ire=Mre.rows[i].begin(),iim=Mim.rows[i].begin();
	ire!=Mre.rows[i].end();
	ire++,iim++) {
      int j=ire->first;
      assert(j == iim->first);
      int v=verttox[j];
      if(v >= 0) {
	A(i,v) = ire->second;
	A(i+Mre.m,v+Mre.n-npinned) = ire->second;
	A(i+Mre.m,v) = iim->second;
	A(i,v+Mre.n-npinned) = -iim->second;
      }
    }
  }
  /*
    k=0;
  for(int j=0;j<Mre.n;j++) {
    assert(npinned==2);
    if(j != vpinned[0] && j != vpinned[1]) {
      for(int i=0;i<Mre.m;i++) {
	//TODO: taking advantage of sparseness
	if(Mre.getEntry(i,j)) {
	  assert(Mim.getEntry(i,j)!=NULL);
	  A(i,k) = Mre(i,j);
	  A(i+Mre.m,k+Mre.n-npinned) = Mre(i,j);
	  A(i+Mre.m,k) = Mim(i,j);
	  A(i,k+Mre.n-npinned) = -Mim(i,j);
	}
      }
      k++;
    }
  }
  */
  b.setZero();
  for(int j=0;j<npinned;j++) {
    for(int i=0;i<Mre.m;i++) {
      b(i) -= Mre(i,vpinned[j])*Upinned[j].x - Mim(i,vpinned[j])*Upinned[j].y;
      b(i+Mre.m) -= Mre(i,vpinned[j])*Upinned[j].y + Mim(i,vpinned[j])*Upinned[j].x;
    }
  }
  Vector x(2*(Mre.n-npinned));

  LOG4CXX_INFO(KrisLibrary::logger(),"LSConformalMapping: solving...");
  Optimization::LSQRInterface lsqr;
  //do we have any ideas for a start vector? planar projection maybe?
  bool res=lsqr.Solve(A,b);
  if(!res) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Sparse system couldn't be solved!");
    return false;
  }
  x = lsqr.x;

  LOG4CXX_INFO(KrisLibrary::logger(),"LSConformalMapping: done!");

  /*
  Matrix Adense;
  A.get(Adense);
  LOG4CXX_INFO(KrisLibrary::logger(),"A: "<<MatrixPrinter(Adense));
  LOG4CXX_INFO(KrisLibrary::logger(),"b: "<<VectorPrinter(b));
  LOG4CXX_INFO(KrisLibrary::logger(),"x: "<<VectorPrinter(x));
  */
  chart.coordinates.resize(mesh.verts.size());
  for(size_t i=0;i<mesh.verts.size();i++) {
    //get the mapped vertex in v
    int v=verttox[i];
    if(v == -1) continue;  //pinned
    chart.coordinates[i].x = x(v);
    chart.coordinates[i].y = x(v+Mre.n-npinned);
  }
  for(int i=0;i<npinned;i++) {
    chart.coordinates[vpinned[i]] = Upinned[i];
  }
  return true;
}
