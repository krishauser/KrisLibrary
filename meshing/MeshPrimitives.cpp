#include "MeshPrimitives.h"
#include <math3d/geometry3d.h>
#include <math3d/basis.h>
#include <assert.h>
#include <math/complex.h>

namespace Meshing {

void MakeTriPlane(int m,int n,TriMesh& mesh)
{
  MakeTriPlane(m,n,1,1,mesh);
}

void MakeTriPlane(int m,int n,Real sx,Real sy,TriMesh& mesh)
{
  m=Max(m,1);
  n=Max(n,1);
  mesh.verts.resize((m+1)*(n+1));
  mesh.tris.resize(m*n*2);

  //set verts
  Real hx,hy;
  hx = sx/(Real)m;
  hy = sy/(Real)n;
  Real x,y;
  int k=0;
  x=0;
  for(int i=0;i<=m;i++,x+=hx) {
    y=0;
    for(int j=0;j<=n;j++,k++,y+=hy) {
      mesh.verts[k].set(x,y,0);
    }
  }

  //tris
  k=0;
  int v=0;  //v jumps by n+1 every column
  for(int i=0;i<m;i++) {
    for(int j=0;j<n;j++,v++,k+=2) {
      mesh.tris[k].set(v,v+(n+1),v+1);
      mesh.tris[k+1].set(v+1,v+(n+1),v+n+2);
    }
    v++;
  }
  assert(mesh.IsValid());
}

void MakeTriCube(int m,int n,int p,TriMesh& mesh)
{
  MakeTriBox(m,n,p,1,1,1,mesh);
}

void MakeTriBox(int m,int n,int p,Real x,Real y,Real z,TriMesh& mesh)
{
	if(m <= 1 && n <= 1 && p <= 1) {
	  mesh.verts.resize(8);
	  mesh.tris.resize(12);
	  mesh.verts[0].set(0,0,0);
	  mesh.verts[1].set(0,0,z);
	  mesh.verts[2].set(0,y,0);
	  mesh.verts[3].set(0,y,z);
	  mesh.verts[4].set(x,0,0);
	  mesh.verts[5].set(x,0,z);
	  mesh.verts[6].set(x,y,0);
	  mesh.verts[7].set(x,y,z);

	  //-x face
	  mesh.tris[0].set(0,1,3);
	  mesh.tris[1].set(0,3,2);
	  //+x face
	  mesh.tris[2].set(4,6,7);
	  mesh.tris[3].set(4,7,5);
	  //-y face
	  mesh.tris[4].set(0,4,5);
	  mesh.tris[5].set(0,5,1);
	  //+y face
	  mesh.tris[6].set(2,3,7);
	  mesh.tris[7].set(2,7,6);
	  //-z face
	  mesh.tris[8].set(0,2,6);
	  mesh.tris[9].set(0,6,4);
	  //+z face
	  mesh.tris[10].set(1,5,7);
	  mesh.tris[11].set(1,7,3);
	}
	else {
		if(m < 1) m = 1;
		if(n < 1) n = 1;
		if(p < 1) p = 1;
		TriMesh faceTemp;
		vector<TriMesh> faces(6);
		MakeTriPlane(m,n,x,y,faceTemp);
		faces[0] = faceTemp;
		faces[1] = faceTemp;
		for(size_t i=0;i<faces[1].verts.size();i++) faces[1].verts[i].z = z;
		MakeTriPlane(m,p,x,z,faceTemp);
		for(size_t i=0;i<faceTemp.verts.size();i++) {
			faceTemp.verts[i].z=faceTemp.verts[i].y;
			faceTemp.verts[i].y=0;
		}
		faces[2] = faceTemp;
		faces[3] = faceTemp;
		for(size_t i=0;i<faces[3].verts.size();i++) faces[3].verts[i].y = y;
		MakeTriPlane(n,p,y,z,faceTemp);
		for(size_t i=0;i<faceTemp.verts.size();i++) {
			faceTemp.verts[i].z=faceTemp.verts[i].y;
			faceTemp.verts[i].y=faceTemp.verts[i].x;
			faceTemp.verts[i].x=0;
		}
		faces[4] = faceTemp;
		faces[5] = faceTemp;
		for(size_t i=0;i<faces[5].verts.size();i++) faces[5].verts[i].x = x;
		faces[0].FlipFaces();
		faces[3].FlipFaces();
		faces[4].FlipFaces();
		mesh.Merge(faces);
		//TODO: weld vertices?
	}
}

void MakeTriCenteredCube(int m,int n,int p,TriMesh& mesh)
{
  MakeTriCenteredBox(m,n,p,1,1,1,mesh);
}

void MakeTriCenteredBox(int m,int n,int p,Real x,Real y,Real z,TriMesh& mesh)
{
  MakeTriBox(m,n,p,x,y,z,mesh);
  Vector3 shift(0.5*x,0.5*y,0.5*z);
  for(size_t i=0;i<mesh.verts.size();i++) mesh.verts[i] -= shift;
}


void MakeTriSphere(int numStacks,int numSlices,TriMesh& mesh)
{
  MakeTriSphere(numStacks,numSlices,1,mesh);
}

void MakeTriSphere(int numStacks,int numSlices,Real r,TriMesh& mesh)
{
  numStacks=Max(numStacks,2);
  numSlices=Max(numSlices,3);
  mesh.verts.resize(numSlices*(numStacks-1)+2);
  mesh.tris.resize(2*numSlices*(numStacks-2) + 2*numSlices);

  //verts
  Real dtheta=TwoPi/(Real)numSlices;
  Real dphi=Pi/(Real)numStacks;
  Real phi;
  Complex x,dx;
  dx.setPolar(One,dtheta);
  mesh.verts[0].set(0,0,-r);
  int k=1;
  phi=-Pi*Half+dphi;
  for(int i=0;i<numStacks-1;i++,phi+=dphi) {
    Real z=Sin(phi)*r;
    x.set(Cos(phi)*r,0);
    for(int j=0;j<numSlices;j++,k++) {
      mesh.verts[k].set(x.x,x.y,z);
      x *= dx;
    }
  }
  mesh.verts[k].set(0,0,r);
  assert(k+1 == (int)mesh.verts.size());

  //tris
  k=0;
  //bottom stack
  int v=1;
  for(int j=0;j<numSlices;j++,k++) {
    int n=(j+1 == numSlices ? 0 : j+1);
    mesh.tris[k].set(0,v+n,v+j);
  }

  //middle stacks
  for(int i=1;i+1<numStacks;i++) {
    //wrap a strip around starting from v
    for(int j=0;j<numSlices;j++,k+=2) {
      int n=(j+1 == numSlices ? 0 : j+1);
      mesh.tris[k].set(v+j+numSlices,v+j,v+n);
      mesh.tris[k+1].set(v+j+numSlices,v+n,v+n+numSlices);
    }
    v += numSlices;
  }

  //top stack
  for(int j=0;j<numSlices;j++,k++) {
    int n=(j+1 == numSlices ? 0 : j+1);
    mesh.tris[k].set(v+numSlices,v+j,v+n);
  }
  assert(k == (int)mesh.tris.size());
  assert(v+numSlices+1 == (int)mesh.verts.size());
  assert(mesh.IsValid());
}

void MakeTriCone(int numSlices,TriMesh& mesh)
{
  MakeTriCone(numSlices,1,1,mesh);
}

void MakeTriCone(int numSlices,Real h,Real rbase,TriMesh& mesh)
{
  numSlices=Max(numSlices,3);
  mesh.verts.resize(numSlices+2);
  mesh.tris.resize(numSlices*2);
  Real dtheta=TwoPi/(Real)numSlices;

  //verts
  Complex x,dx;
  dx.setPolar(1,dtheta);
  x.set(rbase,0);
  mesh.verts[0].setZero();
  for(int i=0;i<numSlices;i++) {
    mesh.verts[i+1].set(x.x,x.y,0);
    x *= dx;
  }
  mesh.verts[numSlices+1].set(0,0,h);
  
  //tris
  for(int i=0;i<numSlices;i++) {
    int n=(i+1==numSlices? 0 : i+1);
    mesh.tris[i].set(0,1+n,1+i);
    mesh.tris[i+numSlices].set(numSlices+1,1+i,1+n);
  }
  assert(mesh.IsValid());
}

void MakeTriCylinder(int numSlices,TriMesh& mesh)
{
  MakeTriCylinder(numSlices,1,1,mesh);
}

void MakeTriCylinder(int numSlices,Real h,Real rbase,TriMesh& mesh)
{
  mesh.verts.resize(numSlices*2+2);
  mesh.tris.resize(numSlices*4);

  Real dtheta=TwoPi/(Real)numSlices;

  //verts
  Complex x,dx;
  dx.setPolar(1,dtheta);
  x.set(rbase,0);
  mesh.verts[0].setZero();
  mesh.verts[numSlices*2+1].set(0,0,h);
  for(int i=0;i<numSlices;i++) {
    mesh.verts[i+1].set(x.x,x.y,0);
    mesh.verts[i+1+numSlices].set(x.x,x.y,h);
    x *= dx;
  }

  //tris
  for(int i=0;i<numSlices;i++) {
    int n=(i+1==numSlices? 0 : i+1);
    //bottom
    mesh.tris[i].set(0,1+n,1+i);
    //top
    mesh.tris[i+numSlices].set(numSlices*2+1,1+i+numSlices,1+n+numSlices);
    //sides
    mesh.tris[i*2+numSlices*2].set(1+i,1+n,1+i+numSlices);
    mesh.tris[i*2+numSlices*2+1].set(1+n,1+n+numSlices,1+i+numSlices);
  }
  assert(mesh.IsValid());
}





void MakeTriMesh(const Sphere3D& geom,int numStacks,int numSlices,TriMesh& mesh)
{
  MakeTriSphere(numStacks,numSlices,mesh);
  Matrix4 mat;
  mat.setIdentity();
  mat(0,0) = mat(1,1) = mat(2,2) = geom.radius;
  geom.center.get(mat(0,3),mat(1,3),mat(2,3));
  mesh.Transform(mat);
}

void MakeTriMesh(const Triangle3D& geom,TriMesh& mesh)
{
  mesh.verts.resize(3);
  mesh.tris.resize(1);
  mesh.verts[0] = geom.a;
  mesh.verts[1] = geom.b;
  mesh.verts[2] = geom.c;
  mesh.tris[0].a = 0;
  mesh.tris[0].b = 1;
  mesh.tris[0].c = 2;
}

void MakeTriMesh(const AABB3D& geom,TriMesh& mesh)
{
  MakeTriCube(1,1,1,mesh);
  Matrix4 mat;
  mat.setIdentity();
  mat(0,0) = geom.bmax.x-geom.bmin.x;
  mat(1,1) = geom.bmax.y-geom.bmin.y;
  mat(2,2) = geom.bmax.z-geom.bmin.z;
  geom.bmin.get(mat(0,3),mat(1,3),mat(2,3));
  mesh.Transform(mat);
}

void MakeTriMesh(const Box3D& geom,TriMesh& mesh)
{
  MakeTriCube(1,1,1,mesh);
  Matrix4 mat;
  geom.getBasisScaled(mat);
  mesh.Transform(mat);
}

void MakeTriMesh(const Ellipsoid3D& geom,int numStacks,int numSlices,TriMesh& mesh)
{
  MakeTriSphere(numStacks,numSlices,mesh);
  Matrix4 mat;
  geom.getBasisScaled(mat);
  mesh.Transform(mat);
}

void MakeTriMesh(const Cylinder3D& geom,int numSlices,TriMesh& mesh)
{
  MakeTriCylinder(numSlices,geom.height,geom.radius,mesh);
  Vector3 x,y;
  GetCanonicalBasis(geom.axis,x,y);
  mesh.Transform(Matrix4(x,y,geom.axis,geom.center));
}

///makes a triangle mesh from a polygon (one sided)
void MakeTriMesh(const Polygon3D& geom,TriMesh& mesh)
{
  mesh.verts = geom.vertices;
  mesh.tris.resize(geom.vertices.size()-2);
  for(size_t i=0;i+2<geom.vertices.size();i++)
    mesh.tris[i].set(0,i+1,i+2);
}

///makes a triangle mesh from a generic geometric primitive
void MakeTriMesh(const GeometricPrimitive3D& geom,TriMesh& mesh,int numDivs)
{
  switch(geom.type) {
  case GeometricPrimitive3D::Point:
    mesh.verts.resize(1);
    mesh.verts[0] = *AnyCast_Raw<Point3D>(&geom.data);
    mesh.tris.resize(0);
    break;
  case GeometricPrimitive3D::Segment:
    //make a degenerate mesh
    mesh.verts.resize(2);
    mesh.verts[0] = AnyCast_Raw<Segment3D>(&geom.data)->a;
    mesh.verts[1] = AnyCast_Raw<Segment3D>(&geom.data)->b;
    mesh.tris.resize(1);
    mesh.tris[0].set(0,1,1);
    break;
  case GeometricPrimitive3D::Triangle:
    MakeTriMesh(*AnyCast_Raw<Triangle3D>(&geom.data),mesh);
    break;
  case GeometricPrimitive3D::Polygon:
    MakeTriMesh(*AnyCast_Raw<Polygon3D>(&geom.data),mesh);
    break;
  case GeometricPrimitive3D::Box:
    MakeTriMesh(*AnyCast_Raw<Box3D>(&geom.data),mesh);
    break;
  case GeometricPrimitive3D::Cylinder:
    MakeTriMesh(*AnyCast_Raw<Cylinder3D>(&geom.data),numDivs,mesh);
    break;
  case GeometricPrimitive3D::Sphere:
    MakeTriMesh(*AnyCast_Raw<Sphere3D>(&geom.data),numDivs/2,numDivs,mesh);
    break;
  case GeometricPrimitive3D::Ellipsoid:
    MakeTriMesh(*AnyCast_Raw<Ellipsoid3D>(&geom.data),numDivs,numDivs,mesh);
    break;
  case GeometricPrimitive3D::AABB:
    MakeTriMesh(*AnyCast_Raw<AABB3D>(&geom.data),mesh);
    break;
  default:
    FatalError("Invalid primitive type %d for MakeTriMesh",geom.type);
  }
}

} //namespace Meshing
