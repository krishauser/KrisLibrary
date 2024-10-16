#include <KrisLibrary/Logger.h>
#include "Conversions.h"
#include "CollisionMesh.h"
#include "ConvexHull3D.h"
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/MarchingCubes.h>
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/meshing/TriMeshOperators.h>
#include <KrisLibrary/math3d/random.h>
#include <KrisLibrary/math3d/basis.h>
#include <KrisLibrary/structs/FixedSizeHeap.h>
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/Timer.h>

DECLARE_LOGGER(Geometry)

#include "SOLID.h"
#include <hacdHACD.h>


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
	pc.Clear();
	pc.points = mesh.verts;
	vector<Vector3> normals;
	if(wantNormals) {
		normals.resize(mesh.verts.size());
		std::fill(normals.begin(),normals.end(),Vector3(0.0));
		for(size_t i=0;i<mesh.tris.size();i++) {
			Vector3 n = mesh.TriangleNormal(i);
			normals[mesh.tris[i].a] += n;
			normals[mesh.tris[i].b] += n;
			normals[mesh.tris[i].c] += n;
		}
		for(size_t i=0;i<normals.size();i++)
			normals[i].inplaceNormalize();
	}
	if(!IsInf(maxDispersion)) {
		Real maxDispersion2 = Sqr(maxDispersion);
		for(size_t i=0;i<mesh.tris.size();i++) {
			Triangle3D t;
			mesh.GetTriangle(i,t);
			size_t np = pc.points.size();
			SubdivideAdd(t,pc,maxDispersion2);
			if(wantNormals) {
				Vector3 n = t.normal();
				size_t np2 = pc.points.size();
				for(size_t j=np;j<np2;j++)
					normals.push_back(n);
			}
		}
	}
	if(wantNormals)
		pc.SetNormals(normals);
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
		LOG4CXX_ERROR(KrisLibrary::logger(),"PointCloudToMesh: TODO: convert unstructured point clouds to meshes");
		return;
	}
	mesh.verts = pc.points;
	mesh.tris.reserve(pc.points.size()*2);
	mesh.tris.resize(0);
	RigidTransform T = pc.GetViewpoint();
	Vector3 fwd(T.R.col3());
	Real zofs = fwd.dot(T.t);
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
			//get depth from origin / viewport
			Real z11 = fwd.dot(p11)-zofs;
			Real z12 = fwd.dot(p12)-zofs;
			Real z21 = fwd.dot(p21)-zofs;
			Real z22 = fwd.dot(p22)-zofs;
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

//expansion pads the box by expansion*resolution
void FitGridToBB(const AABB3D& bb,Meshing::VolumeGrid& grid,Real resolution,Real expansion=0.5)
{
	Vector3 size=bb.bmax-bb.bmin;
	size += (expansion*resolution)*2;
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

void PrimitiveOccupancyGridFill(const GeometricPrimitive3D& primitive,Meshing::VolumeGrid& grid,Real value,Real expansion)
{
	AABB3D bb=primitive.GetAABB();
	bb.bmin -= Vector3(expansion);
	bb.bmax += Vector3(expansion);
	Meshing::VolumeGridIterator<Real> it = grid.getIterator(bb);
	Real cellRadius = grid.GetCellSize().norm()*0.5;
	Vector3 c;
	for(;!it.isDone();++it) {
		it.getCellCenter(c);
		if(primitive.Distance(c) <= expansion + cellRadius) {
			*it = value;
		}
		++it;
	}
}

void PrimitiveImplicitSurfaceFill(const GeometricPrimitive3D& primitive,Meshing::VolumeGrid& grid,Real truncation)
{
	Vector3 c;
	if(truncation == 0 || !IsFinite(truncation)) {
		Meshing::VolumeGrid::iterator it = grid.getIterator();
		while(!it.isDone()) {
			it.getCellCenter(c);
			*it = primitive.Distance(c);
			++it;
		}
	}
	else {
		AABB3D bb=primitive.GetAABB();
		bb.bmin -= Vector3(truncation);
		bb.bmax += Vector3(truncation);
		Meshing::VolumeGridIterator<Real> it=grid.getIterator(bb);
		for(;!it.isDone();++it) {
			it.getCellCenter(c);
			*it = primitive.Distance(c);
			++it;
		}
	}
}

void PrimitiveToImplicitSurface(const GeometricPrimitive3D& primitive,Meshing::VolumeGrid& grid,Real resolution,Real expansion)
{
	AABB3D aabb = primitive.GetAABB();
	aabb.bmin -= Vector3(expansion);
	aabb.bmax += Vector3(expansion);
	FitGridToBB(aabb,grid,resolution);
	PrimitiveImplicitSurfaceFill(primitive,grid);
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


void MeshOccupancyGridFill(const Meshing::TriMesh& mesh,Meshing::VolumeGrid& grid,Real value,Real expansion)
{
	Vector3 expansionv(expansion);
	Triangle3D tri;
	AABB3D query,cell;
	Real cellradius = grid.GetCellSize().norm()*0.5;
	for(size_t i=0;i<mesh.tris.size();i++) {
		mesh.GetTriangle(i,tri);
		query.setPoint(tri.a);
		query.expand(tri.b);
		query.expand(tri.c);
		query.bmin -= expansionv;
		query.bmax -= expansionv;
		Meshing::VolumeGrid::iterator it=grid.getIterator(query);
		for(;!it.isDone();++it) {
			if(grid.value(it.index)>=value) continue;
			it.getCell(cell);
			if(tri.intersects(cell))
				grid.value(it.index)=value;
			else if(expansion > 0) {
				Vector3 c = (cell.bmin+cell.bmax)*0.5;
				Vector3 cp = tri.closestPoint(c);
				if(c.distance(cp) < expansion + cellradius)
					grid.value(it.index)=1;
			}
		}
	}
}

void MeshToOccupancyGrid(const Meshing::TriMesh& mesh,Meshing::VolumeGrid& grid,Real resolution,Real expansion)
{
	AABB3D aabb;
	mesh.GetAABB(aabb.bmin,aabb.bmax);
	Vector3 expansionv(expansion);
	aabb.bmin -= expansionv;
	aabb.bmax += expansionv;
	FitGridToBB(aabb,grid,resolution,0.5);
	grid.value.set(0.0);
	MeshOccupancyGridFill(mesh,grid,1.0,expansion);
}

void MeshImplicitSurfaceFill_FMM(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real truncation)
{
	if(!(truncation == 0 || !IsFinite(truncation))) LOG4CXX_WARN(KrisLibrary::logger(),"MeshImplicitSurfaceFill_FMM: truncation not implemented yet");
	Array3D<Vector3> gradient;
	vector<IntTriple> surfaceCells;
	//Meshing::FastMarchingMethod(mesh,grid.value,gradient,grid.bb,surfaceCells);
	Meshing::FastMarchingMethod_Fill(mesh,grid.value,gradient,grid.bb,surfaceCells);
}

void MeshToImplicitSurface_FMM(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution,Real expansion)
{
	AABB3D aabb;
	mesh.GetAABB(aabb.bmin,aabb.bmax);
	aabb.bmin -= Vector3(expansion);
	aabb.bmax += Vector3(expansion);
	FitGridToBB(aabb,grid,resolution,0.5);
	MeshImplicitSurfaceFill_FMM(mesh,grid);
}

void MeshToImplicitSurface_SpaceCarving(const CollisionMesh& mesh,Meshing::VolumeGrid& grid,Real resolution,int numViews,Real expansion)
{
	AABB3D aabb;
	mesh.GetAABB(aabb.bmin,aabb.bmax);
	aabb.bmin -= Vector3(expansion);
	aabb.bmax += Vector3(expansion);
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

void ImplicitSurfaceToMesh(const Meshing::VolumeGrid& grid,Meshing::TriMesh& mesh,Real levelSet)
{
	AABB3D center_bb = grid.bb;
	Vector3 celldims = grid.GetCellSize();
	//make correction for grid cell centers
	center_bb.bmin += celldims*0.5;
	center_bb.bmax -= celldims*0.5;
    MarchingCubes(grid.value,levelSet,center_bb,mesh);
    MergeVertices(mesh,0,true);
}

void HeightmapToMesh(const Meshing::Heightmap& hm, Meshing::TriMesh& mesh)
{
    if(hm.heights.m < 2 || hm.heights.n < 2) {
		mesh.verts.clear();
		mesh.tris.clear();
		return;
	}
	Meshing::MakeTriPlane(hm.heights.m-1,hm.heights.n-1,mesh);
	hm.GetVertices(mesh.verts);

	vector<pair<int,int> > invalidIndices;
	for(int i=0;i<hm.heights.m;i++) {
		for(int j=0;j<hm.heights.n;j++) {
			if(hm.viewport.perspective) {
		        if(hm.heights(i,j) == 0 || !IsFinite(hm.heights(i,j))) {
					invalidIndices.push_back(pair<int,int>(i,j));	
				}
			}
			else {
				if(!IsFinite(hm.heights(i,j))) {
					invalidIndices.push_back(pair<int,int>(i,j));
				}
			}
		}
	}
	if(invalidIndices.empty()) return;
	printf("HeightmapToMesh: Dropping %d invalid vertices\n",invalidIndices.size());

	vector<int> invalidVertices;
	invalidVertices.reserve(invalidIndices.size());
	for(const auto& p : invalidIndices) {
		invalidVertices.push_back(p.first*hm.heights.n+p.second);
	}
	vector<int> newVertMap, newTriMap;
	Meshing::DropVertices(mesh,invalidVertices,newVertMap,newTriMap);
	for(size_t i=0;i<mesh.verts.size();i++)
		Assert(IsFinite(mesh.verts[i].x) && IsFinite(mesh.verts[i].y) && IsFinite(mesh.verts[i].z));
}

void HeightmapToMesh(const Meshing::Heightmap& hm, Meshing::TriMesh& mesh, GLDraw::GeometryAppearance& appearance)
{
	if(hm.heights.m < 2 || hm.heights.n < 2) {
		mesh.verts.clear();
		mesh.tris.clear();
		return;
	}

	Meshing::MakeTriPlane(hm.heights.m-1,hm.heights.n-1,mesh);
	if(hm.viewport.perspective) {
		//bottom-up view, so need to flip triangles
		for(size_t i=0;i<mesh.tris.size();i++) {
			swap(mesh.tris[i].b,mesh.tris[i].c);
		}
	}
	hm.GetVertices(mesh.verts);
	if(hm.HasColors()) {
		vector<Vector3> rgb(mesh.verts.size());
		hm.GetVertexColors(rgb);
		appearance.vertexColors.resize(rgb.size());
		for(size_t i=0;i<rgb.size();i++) {
			appearance.vertexColors[i].set(rgb[i].x,rgb[i].y,rgb[i].z);
		}
	}
	else
		appearance.vertexColors.resize(0);

	vector<pair<int,int> > invalidIndices;
	for(int i=0;i<hm.heights.m;i++) {
		for(int j=0;j<hm.heights.n;j++) {
			if(!hm.ValidHeight(hm.heights(i,j))) {
				invalidIndices.push_back(pair<int,int>(i,j));	
			}
		}
	}
	if(invalidIndices.empty()) return;
	printf("HeightmapToMesh: Dropping %d invalid vertices\n",invalidIndices.size());

	vector<int> invalidVertices;
	invalidVertices.reserve(invalidIndices.size());
	for(const auto& p : invalidIndices) {
		invalidVertices.push_back(p.first*hm.heights.n+p.second);
	}
	vector<int> newVertMap, newTriMap;
	Meshing::DropVertices(mesh,invalidVertices,newVertMap,newTriMap);
	if(!appearance.vertexColors.empty()) {
		vector<GLDraw::GLColor> newColors(mesh.verts.size());
		for(size_t i=0;i<newVertMap.size();i++) 
			if(newVertMap[i] >= 0)
				newColors[newVertMap[i]] = appearance.vertexColors[(int)i];
		swap(appearance.vertexColors,newColors);
	}
	for(size_t i=0;i<mesh.verts.size();i++)
		Assert(IsFinite(mesh.verts[i].x) && IsFinite(mesh.verts[i].y) && IsFinite(mesh.verts[i].z));
}

void MeshToConvexHull(const Meshing::TriMesh &mesh, ConvexHull3D& ch) { ch.SetPoints(mesh.verts); }

void PointCloudToConvexHull(const Meshing::PointCloud3D &pc, ConvexHull3D& ch) {
	bool allvalid = true;
	for(const auto& pt:pc.points) {
		if(!IsFinite(pt.x) || !IsFinite(pt.y) || !IsFinite(pt.z)) {
			allvalid =false;
			break;
		}
	}
	if(allvalid) 
		ch.SetPoints(pc.points);
	else {
		vector<Vector3> temp;
		temp.reserve(pc.points.size());
		for(const auto& pt:pc.points) {
			if(IsFinite(pt.x) && IsFinite(pt.y) & IsFinite(pt.z)) {
				temp.push_back(pt);
			}
		}
		ch.SetPoints(temp);
	}
}

void _HACD_CallBack(const char * msg, double progress, double concavity, size_t nVertices)
{
    LOG4CXX_INFO(KrisLibrary::logger(),msg);
}

void MeshConvexDecomposition(const Meshing::TriMesh& mesh, ConvexHull3D& ch, Real concavity)
{
    if(concavity <= 0) {
        MeshToConvexHull(mesh,ch);
        return;
    }
  int minClusters = 1;  // so I allow conversion directly from convex objects
//  bool invert = false;
  bool addExtraDistPoints = true;
  bool addFacesPoints = true;
  float ccConnectDist = 30;
  int targetNTrianglesDecimatedMesh = 3000;

  vector<HACD::Vec3<Real> > points;
  vector<HACD::Vec3<long> > triangles;
  // convert mesh into points and triangles
  int n_point = mesh.verts.size();
  points.resize(n_point);
  for (size_t i=0; i < points.size(); ++i) {
    points[i].X() = mesh.verts[i].x;
    points[i].Y() = mesh.verts[i].y;
    points[i].Z() = mesh.verts[i].z;
  }
  triangles.resize(mesh.tris.size());
  for (size_t i=0; i < triangles.size(); ++i) {
    triangles[i][0] = mesh.tris[i].a;
    triangles[i][1] = mesh.tris[i].b;
    triangles[i][2] = mesh.tris[i].c;
  }

  HACD::HeapManager * heapManager = HACD::createHeapManager(65536*(1000));

  HACD::HACD * const myHACD = HACD::CreateHACD(heapManager);
  myHACD->SetPoints(&points[0]);
  myHACD->SetNPoints(points.size());
  myHACD->SetTriangles(&triangles[0]);
  myHACD->SetNTriangles(triangles.size());
  myHACD->SetCompacityWeight(0.0001);
  myHACD->SetVolumeWeight(0.0);
  myHACD->SetConnectDist(ccConnectDist);               // if two connected components are seperated by distance < ccConnectDist
                                                      // then create a virtual edge between them so the can be merged during
                                                      // the simplification process

  myHACD->SetNClusters(minClusters);                     // minimum number of clusters
  myHACD->SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
  myHACD->SetConcavity(concavity);                     // maximum concavity
  myHACD->SetSmallClusterThreshold(0.25);              // threshold to detect small clusters
  myHACD->SetNTargetTrianglesDecimatedMesh(targetNTrianglesDecimatedMesh); // # triangles in the decimated mesh
  myHACD->SetCallBack(&_HACD_CallBack);
  myHACD->SetAddExtraDistPoints(addExtraDistPoints);
  myHACD->SetAddFacesPoints(addFacesPoints);

  myHACD->Compute();
  int nClusters = myHACD->GetNClusters();

  auto get_hull_i = [myHACD](int i) {
    size_t nPoints = myHACD->GetNPointsCH(i);
    size_t nTriangles = myHACD->GetNTrianglesCH(i);
    vector <HACD::Vec3<Real> > hullpoints(nPoints);
    vector <HACD::Vec3<long> > hulltriangles(nTriangles);
    myHACD->GetCH(i, &hullpoints[0], &hulltriangles[0]);
    ConvexHull3D hull;
    vector<Vector3> points;
    for(size_t j = 0; j < nPoints; j++) {
      Vector3 pnt(hullpoints[j].X(), hullpoints[j].Y(), hullpoints[j].Z());
      points.push_back(pnt);
    }
    hull.SetPoints(points);
    return hull;
  };

  // we use hull if only one cluster exists
  if(nClusters == 1) {
    size_t nPoints = myHACD->GetNPointsCH(0);
    size_t nTriangles = myHACD->GetNTrianglesCH(0);
    vector <HACD::Vec3<Real> > hullpoints(nPoints);
    vector <HACD::Vec3<long> > hulltriangles(nTriangles);
    myHACD->GetCH(0, &hullpoints[0], &hulltriangles[0]);
    // I construct a ConvexHull using hullpoints
    ch = get_hull_i(0);
  }
  else{
    std::vector<ConvexHull3D> grp_hulls;
    for (int i=0; i < nClusters; ++i) {
      grp_hulls.push_back(get_hull_i(i));
    }
    //ch.type = ConvexHull3D::Composite;
    ch.data = grp_hulls;
	LOG4CXX_FATAL(KrisLibrary::logger(),"MeshConvexDecomposition: multiple convex hulls not supported yet");
	FatalError("MeshConvexDecomposition: multiple convex hulls not supported yet");
  }
  HACD::DestroyHACD(myHACD);
  HACD::releaseHeapManager(heapManager);
}

void AppendPoints(const ConvexHull3D& ch,vector<Vector3>& points)
{
	if(ch.type==ConvexHull3D::Polytope) {
		const auto& pts = ch.AsPolytope();
		for(size_t i=0;i<pts.size();i+=3) {
			points.push_back(Vector3(pts[i],pts[i+1],pts[i+2]));
		}
	}
	else if(ch.type == ConvexHull3D::Point)
		points.push_back(ch.AsPoint());
	else if(ch.type == ConvexHull3D::Trans) {
		const auto& data = ch.AsTrans();
		vector<Vector3> temp;
		AppendPoints(data.first,temp);
		for(const auto& p:temp) {
			points.push_back(data.second*p);
		}
	}
	else if(ch.type == ConvexHull3D::Hull) {
		const auto& data = ch.AsHull();
		AppendPoints(data.first,points);
		AppendPoints(data.second,points);
	}
	else {
		FatalError("Can't do that type of ConvexHull3D yet");
	}
}

void ConvexHullToMesh(const ConvexHull3D& ch, Meshing::TriMesh &mesh)
{
	vector<Vector3> points;
	vector<vector<int> > facets;
	if(ch.type == ConvexHull3D::Point) {
		mesh.verts.resize(1);
		mesh.tris.resize(0);
		mesh.verts[0] = ch.AsPoint();
		return;
	}
	else if(ch.type == ConvexHull3D::Trans) {
		const auto& data = ch.AsTrans();
		ConvexHullToMesh(data.first,mesh);
		mesh.Transform(data.second);
		return;
	}
	else {
		AppendPoints(ch,points);
	}
	if(!ConvexHull3D_Qhull(points,facets)) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"ConvexHullToMesh: QHull failed");
		mesh.verts.resize(0);
		mesh.tris.resize(0);
		return;
	}
	/*
	mesh.verts = points;
	for(const auto& f:facets) {
		for(size_t i=1;i+1<f.size();i++)
			mesh.tris.push_back(IntTriple(f[0],f[i],f[i+1]));
	}
	return;
	*/
	map<int,int> vertex_map;
	for(const auto& f:facets) {
		//printf("Facet: ");
		for(auto i:f) {
			//printf("%d ",i);
			assert(i >= 0 && i < (int)points.size());
			if(vertex_map.count(i) == 0) {
				int cnt = (int)vertex_map.size();
				vertex_map[i] = cnt;
			}
		}
		//printf("\n");
	}
	/*
	printf("ConvexHullToMesh: reducing from %d to %d vertices\n",points.size(),vertex_map.size());
	for(auto i:vertex_map) {
		printf("  %d -> %d\n",i.first,i.second);
	}
	*/
	mesh.verts.resize(vertex_map.size());
	mesh.tris.resize(0);
	for(auto i:vertex_map) {
		assert(i.first >= 0 && i.first < (int)points.size());
		assert(i.second >= 0 && i.second < (int)vertex_map.size());
		mesh.verts[i.second] = points[i.first];
	}
	assert(mesh.verts.size() == vertex_map.size());
	for(const auto& f:facets) {
		for(size_t i=1;i+1<f.size();i++)
			mesh.tris.push_back(IntTriple(vertex_map[f[0]],vertex_map[f[i+1]],vertex_map[f[i]]));
	}
	for(size_t i=0;i<mesh.tris.size();i++) {
		assert(mesh.tris[i].a >= 0 && mesh.tris[i].a < (int)mesh.verts.size());
		assert(mesh.tris[i].b >= 0 && mesh.tris[i].b < (int)mesh.verts.size());
		assert(mesh.tris[i].c >= 0 && mesh.tris[i].c < (int)mesh.verts.size());
	}
}

void ConvexHullOccupancyGridFill(const ConvexHull3D& ch,Meshing::VolumeGrid& grid,Real value,Real expansion)
{
	AABB3D bb=ch.GetAABB();
	bb.bmin -= Vector3(expansion);
	bb.bmax += Vector3(expansion);
	Meshing::VolumeGrid::iterator it=grid.getIterator(bb);
	Real cellRadius = it.cellSize.norm()*0.5;
	Vector3 c;
	for(;!it.isDone();++it) {
		it.getCellCenter(c);
		if(ch.Distance(c) <= expansion + cellRadius) {
			*it = value;
		}
		++it;
	}
}

void ConvexHullImplicitSurfaceFill(const ConvexHull3D& ch, Meshing::VolumeGrid& grid,Real truncation)
{
	Vector3 c;
	if(truncation == 0 || !IsFinite(truncation)) {
		Meshing::VolumeGrid::iterator it = grid.getIterator();
		while(!it.isDone()) {
			it.getCellCenter(c);
			*it = ch.Distance(c);
			++it;
		}
	}
	else {
		AABB3D bb=ch.GetAABB();
		bb.bmin -= Vector3(truncation);
		bb.bmax += Vector3(truncation);
		Meshing::VolumeGrid::iterator it=grid.getIterator(bb);
		for(;!it.isDone();++it) {
			it.getCellCenter(c);
			*it = ch.Distance(c);
			++it;
		}
	}
}

void ConvexHullToImplicitSurface(const ConvexHull3D& ch, Meshing::VolumeGrid& grid,Real resolution,Real expansion)
{
	AABB3D aabb = ch.GetAABB();
	aabb.bmin -= Vector3(expansion);
	aabb.bmax += Vector3(expansion);
	FitGridToBB(aabb,grid,resolution,0.5);
	ConvexHullImplicitSurfaceFill(ch,grid);
}

void PointCloudOccupancyGridFill(const Meshing::PointCloud3D& pc,Meshing::VolumeGrid& grid,Real value,Real expansion)
{
	if(expansion == 0) {
		for(size_t i=0;i<pc.points.size();i++) {
			IntTriple idx;
			if(!grid.GetIndexChecked(pc.points[i],idx.a,idx.b,idx.c)) continue;
			grid.value(idx) = value;
		}
	}
	else {
		Real cellRadius = grid.GetCellSize().norm()*0.5;
		Vector3 c;
		Vector3 expansionv(expansion);
		for(size_t i=0;i<pc.points.size();i++) {
			AABB3D bb(pc.points[i],pc.points[i]);
			bb.bmin -= expansionv;
			bb.bmax += expansionv;
			Meshing::VolumeGrid::iterator it=grid.getIterator(bb);
			for(;!it.isDone();++it) {
				it.getCellCenter(c);
				if(c.distanceSquared(pc.points[i]) <= Sqr(expansion+cellRadius))
					grid.value(it.index) = value;
				++it;
			}
		}
	}
}

void PointCloudImplicitSurfaceFill_FMM(const Meshing::PointCloud3D& pc,Meshing::VolumeGrid& grid,Real truncation)
{
	if(truncation == 0 || !IsFinite(truncation)) grid.value.set(Inf);
	if(truncation == 0) truncation = Inf;
	
	int M = grid.value.m;
	int N = grid.value.n;
	int P = grid.value.p;
	Array3D<int> closestPoint(M,N,P,-1);
	//encode an index i,j,k as (i*N+j)*P+k
	FixedSizeHeap<Real> queue(M*N*P);
	vector<IntTriple> perturbations(6);
	perturbations[0].set(1,0,0);
	perturbations[1].set(-1,0,0);
	perturbations[2].set(0,1,0);
	perturbations[3].set(0,-1,0);
	perturbations[4].set(0,0,1);
	perturbations[5].set(0,0,-1);
	Vector3 c;
	for(size_t i=0;i<pc.points.size();i++) {
		IntTriple idx;
		if(!grid.GetIndexChecked(pc.points[i],idx.a,idx.b,idx.c)) continue;
		grid.GetCellCenter(idx.a,idx.b,idx.c,c);
		Real d = pc.points[i].distance(c);
		if(d < grid.value(idx) && d <= truncation) {
			grid.value(idx) = d;
			closestPoint(idx) = i;
			queue.adjust((idx.a*N+idx.b)*P+idx.c,-d);
		}
		for(const auto& p:perturbations) {
			IntTriple nidx(idx.a+p.a,idx.b+p.b,idx.c+p.c);
			if(nidx.a < 0 || nidx.a >= M || nidx.b < 0 || nidx.b >= N || nidx.c < 0 || nidx.c >= P) continue;
			grid.GetCellCenter(nidx.a,nidx.b,nidx.c,c);
			Real d = pc.points[i].distance(c);
			if(d < grid.value(nidx) && d <= truncation) {
				grid.value(nidx) = d;
				closestPoint(nidx) = i;
				queue.adjust((nidx.a*N+nidx.b)*P+nidx.c,-d);
			}
		}
	}

	IntTriple index;
	vector<IntTriple> adj;
	while(!queue.empty()) {
		int heapIndex = queue.top();
		queue.pop();
		
		//decode heap index
		Assert(heapIndex >= 0);
		index.c = heapIndex % P;
		index.b = (heapIndex / P)%N;
		index.a = (heapIndex / (P*N));
		Assert(index.a < M);

		int cp=closestPoint(index.a,index.b,index.c);
		assert(cp >= 0);

		//add all potentially adjacent neighbors
		adj.resize(0);
		if(index.a+1 < M) { adj.push_back(index); adj.back().a++; }
		if(index.a-1 >= 0) { adj.push_back(index); adj.back().a--; }
		if(index.b+1 < N) { adj.push_back(index); adj.back().b++; }
		if(index.b-1 >= 0) { adj.push_back(index); adj.back().b--; }
		if(index.c+1 < P) { adj.push_back(index); adj.back().c++; }
		if(index.c-1 >= 0) { adj.push_back(index); adj.back().c--; }
		//overheadTime += timer.ElapsedTime();
		for(const auto& neighbor:adj) {
			//timer.Reset();
			Vector3 cellCenter;
			grid.GetCellCenter(neighbor.a,neighbor.b,neighbor.c,cellCenter);
			Real d = pc.points[cp].distance(cellCenter);
			if(d < grid.value(neighbor) && d <= truncation) {
				grid.value(neighbor) = d;
				closestPoint(neighbor) = cp;
				int heapIndex = (neighbor.a*N+neighbor.b)*P+neighbor.c;
				queue.adjust(heapIndex,-d);
			}
		}
	}
}


} //namespace Geometry