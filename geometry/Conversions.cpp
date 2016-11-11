#include "Conversions.h"
#include "CollisionMesh.h"
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/MarchingCubes.h>
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/math3d/random.h>
#include <KrisLibrary/math3d/basis.h>

namespace Geometry {
	
void SubdivideAdd(const Triangle3D& t,Meshing::PointCloud3D& pc,Real maxDispersion)
{
	Vector3 c = (t.a+t.b+t.c)/3.0;
	if(c.distanceSquared(t.a) > maxDispersion || c.distanceSquared(t.b) > maxDispersion || c.distanceSquared(t.c) > maxDispersion) {
		Real ab = t.a.distanceSquared(t.b);
		Real bc = t.b.distanceSquared(t.c);
		Real ca = t.c.distanceSquared(t.a);
		int maxEdge = 0;
		Real maxLen = bc;
		if(ab > maxLen) {
			maxEdge = 2;
			maxLen = bc;
		}
		if(ca > maxLen) {
			maxEdge = 1;
			maxLen = ca;
		}
		Segment3D s = t.edge(maxEdge);
		Vector3 center = (s.a+s.b)*0.5;
		Triangle3D tc1,tc2;
		tc1.a = center;
		tc1.b = t.vertex(maxEdge);
		tc1.c = s.a;
		tc2.a = center;
		tc2.b = s.b;
		tc2.c = t.vertex(maxEdge);
		SubdivideAdd(tc1,pc,maxDispersion);
		SubdivideAdd(tc2,pc,maxDispersion);
	}
}

void MeshToPointCloud(const Meshing::TriMesh& mesh,Meshing::PointCloud3D& pc,Real maxDispersion)
{
	pc.Clear();
	pc.points = mesh.verts;
	if(IsInf(maxDispersion)) return;

	for(size_t i=0;i<mesh.tris.size();i++) {
		Triangle3D t;
		mesh.GetTriangle(i,t);
		SubdivideAdd(t,pc,maxDispersion);
	}
}

void PointCloudToMesh(const Meshing::PointCloud3D& pc,Meshing::TriMesh& mesh,Real depthDiscontinuity)
{
	if(!pc.IsStructured()) {
		fprintf(stderr,"PointCloudToMesh: TODO: convert unstructured point clouds to meshes\n");
		return;
	}
	mesh.verts = pc.points;
	mesh.tris.resize(0);
	int w = pc.GetStructuredWidth();
	int h = pc.GetStructuredHeight();
	int k=0;
	for(int i=0;i<h;i++) {
		for(int j=0;j<w;j++,k++) {
			if(i+1 ==h || j+1 == w) continue;
			int v11=k;
			int v12=k+1;
			int v21=k+w;
			int v22=k+w+1;
			const Vector3& p11=pc.points[v11];
			const Vector3& p12=pc.points[v12];
			const Vector3& p21=pc.points[v21];
			const Vector3& p22=pc.points[v22];
			//TODO: use origin / viewport
			Real z11 = p11.z;
			Real z12 = p12.z;
			Real z21 = p21.z;
			Real z22 = p22.z;
			bool d1x = (Abs(z11 - z12) > depthDiscontinuity*(z11+z12));
			bool d1y = (Abs(z11 - z21) > depthDiscontinuity*(z11+z21));
			bool d2x = (Abs(z22 - z21) > depthDiscontinuity*(z22+z21));
			bool d2y = (Abs(z22 - z12) > depthDiscontinuity*(z22+z12));
			bool dupperleft = (d1x || d1y);
			bool dupperright = (d1x || d2y);
			bool dlowerleft = (d2x || d1y);
			bool dlowerright = (d2x || d2y);


			if(dupperleft && !dlowerright) 
			  //only draw lower right corner
			  mesh.tris.push_back(IntTriple(v12,v21,v22));
			else if(!dupperleft && dlowerright) 
			  //only draw upper left corner
			  mesh.tris.push_back(IntTriple(v11,v21,v12));
			else if(!dupperright && dlowerleft) 
			  //only draw upper right corner
			  mesh.tris.push_back(IntTriple(v11,v22,v12));
			else if(dupperright && !dlowerleft) 
			  //only draw lower left corner
			  mesh.tris.push_back(IntTriple(v11,v21,v22));
			else if (!dupperleft && !dlowerright) {
			  //fully connected -- should draw better conditioned edge, but whatever
			  mesh.tris.push_back(IntTriple(v12,v21,v22));
			  mesh.tris.push_back(IntTriple(v11,v21,v12));
			}

		}
	}
}

void PrimitiveToMesh(const GeometricPrimitive3D& primitive,Meshing::TriMesh& mesh,int numDivs)
{
	Meshing::MakeTriMesh(primitive,mesh,numDivs);
}

void FitGridToBB(const AABB3D& bb,Meshing::VolumeGrid& grid,Real resolution)
{
	Vector3 size=bb.bmax-bb.bmin;
	int m = (int)Ceil(size.x/resolution)+2;
	int n = (int)Ceil(size.y/resolution)+2;
	int p = (int)Ceil(size.z/resolution)+2;
	
	Vector3 center = (bb.bmax+bb.bmin)*0.5;
	size.x = m*resolution;
	size.y = n*resolution;
	size.z = p*resolution;
	grid.bb.bmin = center - 0.5*size;
	grid.bb.bmax = center + 0.5*size;
	if(m*n*p > 100000000) {
		fprintf(stderr,"FitGridToBB: Warning, creating a volume grid of resolution %g will create %d cells\n",resolution,m*n*p);
		fprintf(stderr,"  Press enter to continue\n");
		getchar();
	}
	grid.Resize(m,n,p);
}

void PrimitiveToImplicitSurface(const GeometricPrimitive3D& primitive,Meshing::VolumeGrid& grid,Real resolution,Real expansion)
{
	AABB3D aabb = primitive.GetAABB();
	FitGridToBB(aabb,grid,resolution);
	Meshing::VolumeGrid::iterator it = grid.getIterator();
	Vector3 c;
	while(!it.isDone()) {
		it.getCellCenter(c);
		if(primitive.Collides(c)) *it = -expansion;
		else *it = primitive.Distance(c)-expansion;
		++it;
	}
}

void Extrema(const AABB3D& bb,const Vector3& dir,Real& a,Real& b)
{
	Vector3 p,q;
	p.x = dir.x*bb.bmin.x;
	p.y = dir.y*bb.bmin.y;
	p.z = dir.z*bb.bmin.z;
	q.x = dir.x*bb.bmax.x;
	q.y = dir.y*bb.bmax.y;
	q.z = dir.z*bb.bmax.z;
	a = Min(p.x,q.x)+Min(p.y,q.y)+Min(p.z,q.z);
	b = Max(p.x,q.x)+Max(p.y,q.y)+Max(p.z,q.z);
}

void SpaceCarving(const CollisionMesh& mesh,const Vector3& d,const Vector3& x,const Vector3& y,Meshing::VolumeGrid& grid,Real resolution)
{
	Real rmax = grid.bb.size().maxAbsElement();
	Real dmin,dmax;
	Real xmin,xmax,ymin,ymax;
	Extrema(grid.bb,d,dmin,dmax);
	Extrema(grid.bb,x,xmin,xmax);
	Extrema(grid.bb,y,ymin,ymax);
	printf("Carving over range [%g,%g]x[%g,%g] with resolution %g\n",xmin,xmax,ymin,ymax,resolution);
	double truncationCellCount = 10;
	Ray3D r;
	Segment3D s;
	Vector3 pt;
	Vector3 c;
	r.direction = d;
	Real u,v;
	vector<IntTriple> cells;
	//get the distance to surface to along the ray
	//for all cells along the ray
	//if the distance in the cell is positive, then take the minimum of that and the distance from the center to the usrface
	int ncast = 0;
	int nhit = 0;
	u = xmin + 0.5*resolution;
	while(u <= xmax) {
		v = ymin + 0.5*resolution;
		while(v <= ymax) {
			ncast ++;
			r.source = (dmin-resolution)*d + u*x + v*y;
			int tri = RayCast(mesh,r,pt);
			if(tri >= 0 && mesh.TriangleNormal(tri).dot(d) < 0) {
				nhit ++;
				s.a = r.source;
				s.b = pt + (truncationCellCount*resolution)*d;
				//cout<<"source "<<r.source<<" direction "<<r.direction<<endl;
				//cout<<"Hit point "<<pt<<endl;
				Meshing::GetSegmentCells(s,grid.value.m,grid.value.n,grid.value.p,grid.bb,cells);
				//cout<<"segment "<<s.a<<" -> "<<s.b<<endl;
				//cout<<"Bbox "<<grid.bb<<endl;
				//printf("Segment up to %g depth hits %d cells\n",5*resolution,cells.size());
				for(size_t i=0;i<cells.size();i++) {
					grid.GetCellCenter(cells[i].a,cells[i].b,cells[i].c,c);
					//distance from c to pt 
					Real distance = pt.distance(c)*Sign(d.dot(pt-c));
					Real v = grid.value(cells[i]);
					//printf("Cell %g %g %g, point %g %g %g, old distance %g new distance %g\n",c.x,c.y,c.z,pt.x,pt.y,pt.z,v,distance);
					assert(s.distance(c) < resolution);
					//printf("Distance to segment: %g\n",s.distance(c));
					if(v > 0) { //think we're outside, update if the ray gives a closer point
						if(distance < 0) {
							//previous ray says we're outside, but measurement is inside.  probably occluded.
						}
						else {
							if(v > distance)
								grid.value(cells[i]) = distance;
						}
					}
					else {
						//think we're inside
						if(distance > 0) {
							//actually we're outside
							grid.value(cells[i]) = distance;
						}
						else if(distance > v)
							//both measurement and previous are inside -- is the measurement a closer point to the surface?
							grid.value(cells[i]) = distance;
					}
				}
				//getchar();
			}
			else {
				//carve through free space
				s.a = r.source;
				s.b = r.source + d*(dmax+resolution);
				Meshing::GetSegmentCells(s,grid.value.m,grid.value.n,grid.value.p,grid.bb,cells);
				for(size_t i=0;i<cells.size();i++) {
					Real v = grid.value(cells[i]);
					if(v < 0) grid.value(cells[i]) = rmax;
					if(v > rmax) grid.value(cells[i]) = rmax;
				}
			}
			v += resolution;
		}
		u += resolution;
	}
	printf("%d / %d rays hit the mesh\n",nhit,ncast);
}

void MeshToImplicitSurface_SpaceCarving(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution,int numViews)
{
	numViews = 6;
	AABB3D aabb;
	Box3D b;
	GetBB(mesh,b);
	b.getAABB(aabb);
	FitGridToBB(aabb,grid,resolution);
	grid.value.set(-aabb.size().maxAbsElement());
	for(int i=0;i<6;i++) {
		Vector3 d(Zero);
		if(i%2 == 0)
			d[i/2] = 1;
		else
			d[i/2] = -1;
		Vector3 x(0.0),y(0.0);
		x[(i/2+1)%3] = 1;
		y[(i/2+2)%3] = 1;
		SpaceCarving(mesh,d,x,y,grid,resolution);
	}
	for(int i=6;i<numViews;i++) {
		Vector3 d;
		SampleSphere(1,d);
		Vector3 x,y;
		GetCanonicalBasis(d,x,y);
		SpaceCarving(mesh,d,x,y,grid,resolution);
	}
	int inside = 0;
	for(int i=0;i<grid.value.m;i++)
		for(int j=0;j<grid.value.n;j++)
			for(int k=0;k<grid.value.p;k++)
				if(grid.value(i,j,k) >= 0)
					inside += 1;
	printf("Volume grid has %d / %d cells occupied\n",inside,grid.value.m*grid.value.n*grid.value.p);
}

void ImplicitSurfaceToMesh(const Meshing::VolumeGrid& grid,Meshing::TriMesh& mesh)
{
	AABB3D center_bb = grid.bb;
	Vector3 celldims = grid.GetCellSize();
	//make correction for grid cell centers
	center_bb.bmin += celldims*0.5;
	center_bb.bmax -= celldims*0.5;
    MarchingCubes(grid.value,0,center_bb,mesh);
}

} //namespace Geometry