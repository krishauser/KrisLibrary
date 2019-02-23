#include <KrisLibrary/Logger.h>
#include "Conversions.h"
#include "CollisionMesh.h"
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/MarchingCubes.h>
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/math3d/random.h>
#include <KrisLibrary/math3d/basis.h>
#include <KrisLibrary/Timer.h>

namespace Geometry {
	
void SubdivideAdd(const Triangle3D& t,Meshing::PointCloud3D& pc,Real maxDispersion2)
{
	Vector3 c = (t.a+t.b+t.c)/3.0;
	if(c.distanceSquared(t.a) > maxDispersion2 || c.distanceSquared(t.b) > maxDispersion2 || c.distanceSquared(t.c) > maxDispersion2) {
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
		pc.points.push_back(center);
		Triangle3D tc1,tc2;
		tc1.a = center;
		tc1.b = t.vertex(maxEdge);
		tc1.c = s.a;
		tc2.a = center;
		tc2.b = s.b;
		tc2.c = t.vertex(maxEdge);
		SubdivideAdd(tc1,pc,maxDispersion2);
		SubdivideAdd(tc2,pc,maxDispersion2);
	}
}

void MeshToPointCloud(const Meshing::TriMesh& mesh,Meshing::PointCloud3D& pc,Real maxDispersion,bool wantNormals)
{
	if(wantNormals) FatalError("Sampling normals not done yet");
	pc.Clear();
	pc.points = mesh.verts;
	if(IsInf(maxDispersion)) return;

	Real maxDispersion2 = Sqr(maxDispersion);
	for(size_t i=0;i<mesh.tris.size();i++) {
		Triangle3D t;
		mesh.GetTriangle(i,t);
		SubdivideAdd(t,pc,maxDispersion2);
	}
}

void PointCloudToMesh(const Meshing::PointCloud3D& pc,Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& appearance,Real depthDiscontinuity)
{
	Timer timer;
	PointCloudToMesh(pc,mesh,depthDiscontinuity);
	vector<Vector4> colors;
	if(pc.GetColors(colors)) {
		appearance.vertexColors.resize(colors.size());
		for(size_t i=0;i<appearance.vertexColors.size();i++)
			appearance.vertexColors[i].set(colors[i].x,colors[i].y,colors[i].z,colors[i].w);
	}
	vector<Vector2> uv;
	if(pc.GetUV(uv)) {
		appearance.texcoords = uv;
	}
}

void PointCloudToMesh(const Meshing::PointCloud3D& pc,Meshing::TriMesh& mesh,Real depthDiscontinuity)
{
	if(!pc.IsStructured()) {
				LOG4CXX_ERROR(KrisLibrary::logger(),"PointCloudToMesh: TODO: convert unstructured point clouds to meshes\n");
		return;
	}
	mesh.verts = pc.points;
	mesh.tris.reserve(pc.points.size()*2);
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
			//sometimes NANs are used
			if(!IsFinite(z11)) z11=0;
			if(!IsFinite(z12)) z12=0;
			if(!IsFinite(z21)) z21=0;
			if(!IsFinite(z22)) z22=0;
			bool d1x = (z11 == 0 || z12 == 0) || (Abs(z11 - z12) > depthDiscontinuity*Abs(z11+z12));
			bool d1y = (z11 == 0 || z21 == 0) || (Abs(z11 - z21) > depthDiscontinuity*Abs(z11+z21));
			bool d2x = (z22 == 0 || z21 == 0) || (Abs(z22 - z21) > depthDiscontinuity*Abs(z22+z21));
			bool d2y = (z22 == 0 || z12 == 0) || (Abs(z22 - z12) > depthDiscontinuity*Abs(z22+z12));
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

void FitGridToBB(const AABB3D& bb,Meshing::VolumeGrid& grid,Real resolution,Real expansion=0.5)
{
	Vector3 size=bb.bmax-bb.bmin;
	size += expansion*2*resolution;
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
		LOG4CXX_ERROR(KrisLibrary::logger(),"FitGridToBB: Warning, creating a volume grid of resolution "<<resolution<<" will create "<<m*n*p);
		KrisLibrary::loggerWait();
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
		//if(primitive.Collides(c)) *it = -expansion;
		//else
		*it = primitive.Distance(c)-expansion;
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

void SpaceCarving(const CollisionMesh& mesh,const Vector3& d,const Vector3& x,const Vector3& y,Meshing::VolumeGrid& grid,Array3D<Real>& occupancy,Real resolution,bool firstPass=true)
{
	Real rmax = grid.bb.size().maxAbsElement();
	Real dmin,dmax;
	Real xmin,xmax,ymin,ymax;
	Extrema(grid.bb,d,dmin,dmax);
	Extrema(grid.bb,x,xmin,xmax);
	Extrema(grid.bb,y,ymin,ymax);
	Vector3 cellSize = grid.GetCellSize();
	Real cellResolution = (cellSize.x + cellSize.y + cellSize.z)/3.0;
	LOG4CXX_INFO(KrisLibrary::logger(),"Carving over range ["<<xmin<<","<<xmax<<"]x["<<ymin<<","<<ymax<<"] with resolution "<<resolution);
	double truncationCellCount = 10;
	Ray3D r;
	Segment3D s;
	Vector3 pt;
	Vector3 c;
	AABB3D cell;
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
			if(tri >= 0) {
				if(mesh.TriangleNormal(tri).dot(d) > 0) { //hit a backfacing edge
					v += resolution;
					continue;
				}
				nhit ++;
				s.a = r.source;
				s.b = pt + (truncationCellCount*resolution)*d;
				//LOG4CXX_INFO(KrisLibrary::logger(),"source "<<r.source<<" direction "<<r.direction);
				//LOG4CXX_INFO(KrisLibrary::logger(),"Hit point "<<pt);
				Meshing::GetSegmentCells(s,grid.value.m,grid.value.n,grid.value.p,grid.bb,cells);
				//LOG4CXX_INFO(KrisLibrary::logger(),"segment "<<s.a<<" -> "<<s.b);
				//LOG4CXX_INFO(KrisLibrary::logger(),"Bbox "<<grid.bb);
				//LOG4CXX_INFO(KrisLibrary::logger(),"Segment up to "<<5*resolution<<" depth hits "<<cells.size());
				for(size_t i=0;i<cells.size();i++) {
					//grid.GetCellCenter(cells[i].a,cells[i].b,cells[i].c,c);
					grid.GetCell(cells[i].a,cells[i].b,cells[i].c,cell);
					Real tmin,tmax;
					Real v;
					Real weight = 1.0;
					if(!s.intersects(cell,tmin,tmax))  {
						c = (cell.bmin + cell.bmax)*0.5;
						v = grid.value(cells[i]);
					}
					else {
						//s.eval((tmin + tmax)*0.5,c);
						c = (cell.bmin + cell.bmax)*0.5;
						//LOG4CXX_INFO(KrisLibrary::logger(),"Intersection "<<tmin<<" "<<tmax);
						weight = Min(tmax-tmin,cellResolution) / cellResolution;
						//weight = 1.0;
						v = grid.value(cells[i]);
						if(!firstPass && Abs(v) < cellResolution)
							v = grid.TrilinearInterpolate(c);
						//LOG4CXX_INFO(KrisLibrary::logger(),"Interpolated value "<<v<<", cell value "<<grid.value(cells[i]));
					}
					//distance from c to pt 
					Real measured = pt.distance(c)*Sign(d.dot(pt-c));
					//Real distance = d.dot(pt-c);
					//weight is now the fraction of the segment in the cell
					//modulate weight by distance - measurement difference so that far distances have high weight, low distances / close by have low weight
					Real occ = occupancy(cells[i]);
					if(measured <= cellResolution*0.5) {
						if(measured >= -cellResolution*0.5) 
							occupancy(cells[i]) += 1;
					}

					//LOG4CXX_INFO(KrisLibrary::logger(),cell.bmin<<" -> "<<cell.bmax);
					//LOG4CXX_INFO(KrisLibrary::logger(),"Cell "<<c.x<<" "<<c.y<<" "<<c.z<<", point "<<pt.x<<" "<<pt.y<<" "<<pt.z<<", old distance "<<v<<" new distance "<<distance);
					//LOG4CXX_INFO(KrisLibrary::logger(),"Weight "<<weight);
					//KrisLibrary::loggerWait();
					//assert(s.distance(c) < cellResolution);
					
					//LOG4CXX_INFO(KrisLibrary::logger(),"Distance to segment: "<<s.distance(c));

					//method 1: just average
					grid.value(cells[i]) += (measured-v)/(1+occ);
					//method 2: check inside vs outside
					/*
					if(v > 0) { //think we're outside, update if the ray gives a closer point
						if(measured < -resolution) {
							//previous ray says we're outside, but measurement is inside.  probably occluded.
						}
						else {
							if(v > measured)
								grid.value(cells[i]) += (measured-v)/(1.0+occ);
						}
					}
					else {
						//think we're inside
						if(measured > 0) {
							//actually we're outside
							grid.value(cells[i]) += (measured-v)/(1.0+occ);
						}
						else if(measured > v)
							//both measurement and previous are inside -- is the measurement a closer point to the surface?
							grid.value(cells[i]) += (measured-v)/(1.0+occ);
					}
					*/
				}
				//KrisLibrary::loggerWait();
			}
			else {
				//carve through free space
				s.a = r.source;
				s.b = r.source + d*(dmax+2*resolution-dmin);
				Meshing::GetSegmentCells(s,grid.value.m,grid.value.n,grid.value.p,grid.bb,cells);
				for(size_t i=0;i<cells.size();i++) {
					Real v = grid.value(cells[i]);
					Real occ = occupancy(cells[i]);
					if(occ == 0) {
						if(v > rmax) grid.value(cells[i]) = rmax;
					}
					if(v < resolution) grid.value(cells[i]) += (resolution-v)/(1.0+occ);
				}
			}
			v += resolution;
		}
		u += resolution;
	}
	LOG4CXX_INFO(KrisLibrary::logger(),""<<nhit<<" / "<<ncast);
}

void SaveSliceCSV(const Array3D<Real>& values,const char* fn)
{
	FILE* f = fopen(fn,"w");
	int k = values.p/2;
	for(int i=0;i<values.m;i++) {
		for(int j=0;j<values.n;j++)
			fprintf(f,"%g,",values(i,j,k));
		fprintf(f,"\n");
	}
	fclose(f);
}

void MeshToImplicitSurface_FMM(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution)
{
	AABB3D aabb;
	mesh.GetAABB(aabb.bmin,aabb.bmax);
	FitGridToBB(aabb,grid,resolution);
	Array3D<Vector3> gradient(grid.value.m,grid.value.n,grid.value.p);
	vector<IntTriple> surfaceCells;
	//Meshing::FastMarchingMethod(mesh,grid.value,gradient,grid.bb,surfaceCells);
	Meshing::FastMarchingMethod_Fill(mesh,grid.value,gradient,grid.bb,surfaceCells);
}

void MeshToImplicitSurface_SpaceCarving(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution,int numViews)
{
	AABB3D aabb;
	mesh.GetAABB(aabb.bmin,aabb.bmax);
	FitGridToBB(aabb,grid,resolution);
	Array3D<Real> occupancy(grid.value.m,grid.value.n,grid.value.p);
	double defaultValue = -aabb.size().maxAbsElement();
	grid.value.set(defaultValue);
	occupancy.set(0.0);
	//carve along the 6 canonical directions
	for(int i=0;i<6;i++) {
		Vector3 d(Zero);
		if(i%2 == 0)
			d[i/2] = 1;
		else
			d[i/2] = -1;
		Vector3 x(0.0),y(0.0);
		x[(i/2+1)%3] = 1;
		y[(i/2+2)%3] = 1;
		SpaceCarving(mesh,d,x,y,grid,occupancy,resolution*0.5,false);
	}
	//carve along random directions
	for(int i=6;i<numViews;i++) {
		Vector3 d;
		SampleSphere(1,d);
		Vector3 x,y;
		GetCanonicalBasis(d,x,y);
		SpaceCarving(mesh,d,x,y,grid,occupancy,resolution*0.5,false);
	}
	//count the number of inside cells, set all far cells to -resolution*0.5
	int inside = 0;
	for(int i=0;i<grid.value.m;i++)
		for(int j=0;j<grid.value.n;j++)
			for(int k=0;k<grid.value.p;k++) {
				if(grid.value(i,j,k) <= 0)
					inside += 1;
				if(grid.value(i,j,k) == defaultValue)
					grid.value(i,j,k) = -resolution*0.5;
			}

	for(size_t i=0;i<mesh.verts.size();i++) {
		Real depth = grid.TrilinearInterpolate(mesh.verts[i]);
		if(depth > 0) {
			//outside, reduce the grid values
			IntTriple cell;
			Vector3 params;
			grid.GetIndexAndParams(mesh.verts[i],cell,params);
			//get trilinear interpolation parameters
			Real u=params.x;
			Real v=params.y;
			Real w=params.z;
			int i1=cell.a;
			int j1=cell.b;
			int k1=cell.c;

			//get the alternate cell indices, interpolation parameters
			//(u interpolates between i1,i2, etc)
			int i2,j2,k2;
			if(u > 0.5) { i2=i1+1; u = u-0.5; }
			else { i2=i1; i1--; u = 0.5+u; }
			if(v > 0.5) { j2=j1+1; v = v-0.5; }
			else { j2=j1; j1--; v = 0.5+v; }
			if(w > 0.5) { k2=k1+1; w = w-0.5; }
			else { k2=k1; k1--; w = 0.5+w; }

			if(i1 < 0) i1=0; if(i1 >= grid.value.m) i1=grid.value.m-1;
			if(i2 < 0) i2=0; if(i2 >= grid.value.m) i2=grid.value.m-1;
			if(j1 < 0) j1=0; if(j1 >= grid.value.n) j1=grid.value.n-1;
			if(j2 < 0) j2=0; if(j2 >= grid.value.n) j2=grid.value.n-1;
			if(k1 < 0) k1=0; if(k1 >= grid.value.p) k1=grid.value.p-1;
			if(k2 < 0) k2=0; if(k2 >= grid.value.p) k2=grid.value.p-1;
			Real c111 = (1-u)*(1-v)*(1-w);
			Real c112 = (1-u)*(1-v)*(w);
			Real c121 = (1-u)*(v)*(1-w);
			Real c122 = (1-u)*(v)*(w);
			Real c211 = (u)*(1-v)*(1-w);
			Real c212 = (u)*(1-v)*(w);
			Real c221 = (u)*(v)*(1-w);
			Real c222 = (u)*(v)*(w);
			Real denom = Sqr(c111)+Sqr(c112)+Sqr(c121)+Sqr(c122)+Sqr(c211)+Sqr(c212)+Sqr(c221)+Sqr(c222);
			
			Real delta = -depth/denom;
			grid.value(i1,j1,k1) += delta*c111;
			grid.value(i1,j1,k2) += delta*c112;
			grid.value(i1,j2,k1) += delta*c121;
			grid.value(i1,j2,k2) += delta*c122;
			grid.value(i2,j1,k1) += delta*c211;
			grid.value(i2,j1,k2) += delta*c212;
			grid.value(i2,j2,k1) += delta*c221;
			grid.value(i2,j2,k2) += delta*c222;
			depth = grid.TrilinearInterpolate(mesh.verts[i]);
		}
	}
	LOG4CXX_INFO(KrisLibrary::logger(),"Volume grid has "<<inside<<" / "<<grid.value.m*grid.value.n*grid.value.p);
}

void ImplicitSurfaceToMesh(const Meshing::VolumeGrid& grid,Meshing::TriMesh& mesh)
{
	AABB3D center_bb = grid.bb;
	Vector3 celldims = grid.GetCellSize();
	//make correction for grid cell centers
	center_bb.bmin += celldims*0.5;
	center_bb.bmax -= celldims*0.5;
    MarchingCubes(grid.value,0.0,center_bb,mesh);
}

} //namespace Geometry