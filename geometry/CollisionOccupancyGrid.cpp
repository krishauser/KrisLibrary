#include "CollisionOccupancyGrid.h"
#include "CollisionImplicitSurface.h"
#include "Conversions.h"
#include "GridSubdivision.h"
#include "CollisionPrimitive.h"
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/utils/stl_tr1.h>
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/math3d/clip.h>
#include <KrisLibrary/math3d/Segment3D.h>

DECLARE_LOGGER(Geometry)

namespace Geometry {

    bool CollidesBruteForce(const Collider3DOccupancyGrid& a, Collider3D* b, Real d, vector<int>& elements1, vector<int>& elements2, size_t maxcollisions)
    {
        ///brute force method -- collide geom with boxes of the occupancy grid
        RigidTransform Tinv;
        Tinv.setInverse(a.currentTransform);
        Box3D gb = b->GetBB();
        Box3D gblocal;
        gblocal.setTransformed(gb,Tinv);
        AABB3D gbb;
        gblocal.getAABB(gbb);
        if(d != 0) {
            gbb.bmin -= Vector3(d);
            gbb.bmax += Vector3(d);
        }
        AABB3D bb;  //temp bb for occupied cells
        shared_ptr<Geometry3DPrimitive> prim = make_shared<Geometry3DPrimitive>(GeometricPrimitive3D(bb));
        shared_ptr<Collider3DPrimitive> cprim = make_shared<Collider3DPrimitive>(prim);
        cprim->SetTransform(a.currentTransform);
        vector<int> elems_g,elems_p;

        //either use the BB overlap to determine which cells to check, or use occupiedCells
        Meshing::VolumeGridIterator<Real> it = a.data->data.getIterator(gbb);
        if(it.isDone()) return false;
        int ncells = (it.hi.a - it.lo.a + 1) * (it.hi.b - it.lo.b + 1) * (it.hi.c - it.lo.c + 1);
        size_t nchecked = 0;
        if(ncells < (int)a.occupiedCells.size()) {
            //use BB overlap
            for(;!it.isDone();++it) {
                if(*it > a.data->occupancyThreshold) {
                    it.getCell(bb);
                    prim->data = GeometricPrimitive3D(bb);
                    elems_g.resize(0);
                    elems_p.resize(0);
                    nchecked += 1;
                    if(d > 0) {
                        if(!b->WithinDistance(cprim.get(),d,elems_g,elems_p,maxcollisions)) return false;
                    }
                    else {
                        if(!b->Collides(cprim.get(),elems_g,elems_p,maxcollisions)) return false;
                    }
                    if(!elems_g.empty()) {
                    elements2.insert(elements2.end(),elems_g.begin(),elems_g.end());
                    int gridelem = a.data->IndexToElement(it.index);
                    for(auto e:elems_g) 
                        elements1.push_back(gridelem);
                    if(elements1.size() >= maxcollisions) return true;
                    }
                }
            }
        }
        else {
            //check all occupied cells
            for(const auto &cell : a.occupiedCells) {
                a.data->data.GetCell(cell,bb);
                prim->data = GeometricPrimitive3D(bb);
                elems_g.resize(0);
                elems_p.resize(0);
                nchecked += 1;
                if(d > 0) {
                    if(!b->WithinDistance(cprim.get(),d,elems_g,elems_p,maxcollisions)) return false;
                }
                else {
                    if(!b->Collides(cprim.get(),elems_g,elems_p,maxcollisions)) return false;
                }
                if(!elems_g.empty()) {
                    elements2.insert(elements2.end(),elems_g.begin(),elems_g.end());
                    int gridelem = a.data->IndexToElement(cell);
                    for(auto e:elems_g) 
                        elements1.push_back(gridelem);
                    if(elements1.size() >= maxcollisions) return true;
                }
            }
        }
        //printf("OccupancyGrid: Checked %d elements of volume with %d cells in range, %d occupied\n",(int)nchecked,ncells,(int)a.occupiedCells.size());
        return true;
    }

    bool Collides(Collider3DOccupancyGrid& a, Collider3DOccupancyGrid& b, Real d, vector<int>& elements1, vector<int>& elements2, size_t maxCollisions)
    {
        if(d != 0) {
            LOG4CXX_ERROR(GET_LOGGER(Geometry),"Occupancy grid Collides with nonzero d not supported yet");
            return false;
        }
        Vector3 asize = a.data->data.GetCellSize();
        Vector3 bsize = b.data->data.GetCellSize();
        Assert(IsFinite(asize.maxAbsElement()) && IsFinite(bsize.maxAbsElement()));
        if(asize.maxAbsElement() > bsize.maxAbsElement())
            return Collides(b,a,d,elements2,elements1,maxCollisions);
        Box3D bba=a.GetBB(),bbb = b.GetBB();
        if(!bba.intersects(bbb)) return false;
        AABB3D bb;
        Vector3 centerLocal,center;
        for(const auto &cell : a.occupiedCells) {
            a.data->data.GetCell(cell,bb);
            centerLocal = (bb.bmin + bb.bmax)*0.5;
            a.currentTransform.mul(centerLocal,center);
            if(b.Contains(center)) {
                int aelem = a.data->IndexToElement(cell);
                int belem = b.PointToElement(center);
                elements1.push_back(aelem);
                elements2.push_back(belem);
                if(elements1.size() >= maxCollisions) return true;
            }
        }
        return !elements1.empty();
    }

    bool Collides(Collider3DImplicitSurface& a, Collider3DOccupancyGrid& b, Real dthresh, vector<int>& elements1, vector<int>& elements2, size_t maxCollisions)
    {
        Box3D bba=a.GetBB(),bbb = b.GetBB();
        if(!bba.intersects(bbb)) return false;
        Vector3 asize = a.data->data.GetCellSize();
        Vector3 bsize = b.data->data.GetCellSize();
        Assert(IsFinite(asize.maxAbsElement()) && IsFinite(bsize.maxAbsElement()));
        if(asize.maxAbsElement() > bsize.maxAbsElement()) {
            //occupancy grid is finer
            AABB3D bb;
            Vector3 centerLocal,center;
            for(const auto &cell :b.occupiedCells) {
                b.data->data.GetCell(cell,bb);
                centerLocal = (bb.bmin + bb.bmax)*0.5;
                b.currentTransform.mul(centerLocal,center);
                Real d = a.Distance(center);
                if(d <= dthresh) {
                    int belem = b.data->IndexToElement(cell);
                    int aelem = a.PointToElement(center);
                    elements1.push_back(aelem);
                    elements2.push_back(belem);
                    if(elements1.size() >= maxCollisions) return true;
                }
            }
            return !elements1.empty();
        }
        else {
            //implicit surface is finer
            Vector3 centerLocal,center;
            for(Meshing::VolumeGrid::iterator i=a.data->data.getIterator();!i.isDone();++i) {
                if(*i <= dthresh) {
                    i.getCellCenter(centerLocal);
                    a.currentTransform.mul(centerLocal,center);
                    if(b.Contains(center)) {
                        int aelem = a.data->IndexToElement(i.getIndex());
                        int belem = b.PointToElement(center);
                        elements1.push_back(aelem);
                        elements2.push_back(belem);
                        if(elements1.size() >= maxCollisions) return true;
                    }
                }   
            }
            return !elements1.empty();
        }
    }

} //namespace Geometry


using namespace Geometry;

Geometry3DOccupancyGrid::Geometry3DOccupancyGrid(Real _threshold)
: occupancyThreshold(_threshold)
{}

Geometry3DOccupancyGrid::Geometry3DOccupancyGrid(const Meshing::VolumeGrid& _data,Real _threshold)
: Geometry3DVolume(_data), occupancyThreshold(_threshold)
{}

Geometry3DOccupancyGrid::Geometry3DOccupancyGrid(Meshing::VolumeGrid&& _data,Real _threshold)
: Geometry3DVolume(_data), occupancyThreshold(_threshold)
{}


Geometry3D* Geometry3DOccupancyGrid::ConvertTo(Type restype,Real param,Real domainExpansion) const
{
    switch(restype) {
    case Type::PointCloud:
        {
        auto* pc = new Geometry3DPointCloud();
        Meshing::VolumeGridIterator<Real> it=data.getIterator();
        Vector3 c;
        while(!it.isDone()) {
            if(*it > param) {
                it.getCellCenter(c);
                pc->data.points.push_back(c);
            }
            ++it;
        }
        return pc;
        }
    case Type::TriangleMesh:
        {
        auto* gmesh = new Geometry3DTriangleMesh();
        auto& mesh = gmesh->data;
        Meshing::VolumeGridIterator<Real> it=data.getIterator();
        AABB3D bb;
        const static int face_vertices[7][12] = {
            {0,0,0,  0,1,0,   1,1,0,  1,0,0}, //#-z
            {0,0,0,  1,0,0,   1,0,1,  0,0,1}, //#-y
            {0,0,0,  0,0,1,   0,1,1,  0,1,0}, //#-x
            {0,0,0,  0,0,0,   0,0,0,  0,0,0}, //#null
            {1,0,0,  1,1,0,   1,1,1,  1,0,1}, //#+x
            {0,1,0,  0,1,1,   1,1,1,  1,1,0}, //#+y
            {0,0,1,  1,0,1,   1,1,1,  0,1,1}, //#+z
        };
        UNORDERED_MAP_TEMPLATE<IntTriple,int,IndexHash> vertexMap;
        while(!it.isDone()) {
            if(*it > 0) {
                vector<int> faces;
                IntTriple ind = it.getIndex();
                ind.a += 1;
                if(ind.a >= data.value.m || data.value(ind) <= 0) faces.push_back(1);
                ind.a -= 1;
                ind.a -= 1;
                if(ind.a < 0 || data.value(ind) <= 0) faces.push_back(-1);
                ind.a += 1;
                ind.b += 1;
                if(ind.b >= data.value.n || data.value(ind) <= 0) faces.push_back(2);
                ind.b -= 1;
                ind.b -= 1;
                if(ind.b < 0 || data.value(ind) <= 0) faces.push_back(-2);
                ind.b += 1;
                ind.c += 1;
                if(ind.c >= data.value.p || data.value(ind) <= 0) faces.push_back(3);
                ind.c -= 1;
                ind.c -= 1;
                if(ind.c < 0 || data.value(ind) <= 0) faces.push_back(-3);
                ind.c += 1;
                it.getCell(bb);
                for(size_t i=0;i<faces.size();i++) {
                    int f=faces[i]+3;
                    IntTriple a = IntTriple(face_vertices[f][0]+ind.a,face_vertices[f][1]+ind.b,face_vertices[f][2]+ind.c);
                    IntTriple b = IntTriple(face_vertices[f][3]+ind.a,face_vertices[f][4]+ind.b,face_vertices[f][5]+ind.c);
                    IntTriple c = IntTriple(face_vertices[f][6]+ind.a,face_vertices[f][7]+ind.b,face_vertices[f][8]+ind.c);
                    IntTriple d = IntTriple(face_vertices[f][9]+ind.a,face_vertices[f][10]+ind.b,face_vertices[f][11]+ind.c);
                    int ia,ib,ic,id;
                    if(vertexMap.count(a)) ia=vertexMap[a];
                    else {
                        ia=(int)mesh.verts.size();
                        Vector3 v((a.a==ind.a ? bb.bmin.x : bb.bmax.x),(a.b==ind.b ? bb.bmin.y : bb.bmax.y),(a.c==ind.c ? bb.bmin.z : bb.bmax.z));
                        mesh.verts.push_back(v);
                    }
                    if(vertexMap.count(b)) ib=vertexMap[b];
                    else {
                        ib=(int)mesh.verts.size();
                        Vector3 v((b.a==ind.a ? bb.bmin.x : bb.bmax.x),(b.b==ind.b ? bb.bmin.y : bb.bmax.y),(b.c==ind.c ? bb.bmin.z : bb.bmax.z));
                        mesh.verts.push_back(v);
                    }
                    if(vertexMap.count(c)) ic=vertexMap[c];
                    else {
                        ic=(int)mesh.verts.size();
                        Vector3 v((c.a==ind.a ? bb.bmin.x : bb.bmax.x),(c.b==ind.b ? bb.bmin.y : bb.bmax.y),(c.c==ind.c ? bb.bmin.z : bb.bmax.z));
                        mesh.verts.push_back(v);
                    }
                    if(vertexMap.count(d)) id=vertexMap[d];
                    else {
                        id=(int)mesh.verts.size();
                        Vector3 v((d.a==ind.a ? bb.bmin.x : bb.bmax.x),(d.b==ind.b ? bb.bmin.y : bb.bmax.y),(d.c==ind.c ? bb.bmin.z : bb.bmax.z));
                        mesh.verts.push_back(v);
                    }
                    mesh.tris.push_back(IntTriple(ia,ib,ic));
                    mesh.tris.push_back(IntTriple(ic,id,ia));
                }
            }
            ++it;
        }
        return gmesh;
        }
    default:
        return NULL;
    }
}
    
bool Geometry3DOccupancyGrid::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion) 
{
    switch(geom->GetType()) {
    case Type::Primitive:
    case Type::ConvexHull:
        {
        shared_ptr<Geometry3D> g2(geom->Convert(Type::ImplicitSurface, param, domainExpansion));
        if(g2) {
            return ConvertFrom(g2.get());
        }
        LOG4CXX_WARN(GET_LOGGER(Geometry),"Geometry "<<Geometry3D::TypeName(geom->GetType())<<" could not be converted to ImplicitSurface");
        return false;
        }
    case Type::PointCloud:
        {
        const auto* pc = dynamic_cast<const Geometry3DPointCloud*>(geom);
        if(param==0) {
            //auto-determine resolution to approximately have 1 point per cell
            AABB3D bb = geom->GetAABB();
            size_t n = pc->data.points.size();
            Vector3 dims = bb.bmax-bb.bmin;
            Real volume = dims.x*dims.y*dims.z;
            param = Pow(Pow(volume,2.0/3.0) / n,1.0/2.0);
        }
        ResizeTo(geom,param,domainExpansion);
        data.value.set(0.0);
        for(size_t i=0;i<pc->data.points.size();i++) 
            data.SetValue(pc->data.points[i],1.0);
        return true;
        }
    case Type::TriangleMesh:
        {
        const auto& mesh = dynamic_cast<const Geometry3DTriangleMesh*>(geom)->data;
        if (param == 0) {
            if (mesh.tris.empty()) return NULL;
            Real sumlengths = 0;
            for (size_t i = 0; i < mesh.tris.size(); i++) {
                sumlengths += mesh.verts[mesh.tris[i].a].distance(mesh.verts[mesh.tris[i].b]);
                sumlengths += mesh.verts[mesh.tris[i].b].distance(mesh.verts[mesh.tris[i].c]);
                sumlengths += mesh.verts[mesh.tris[i].c].distance(mesh.verts[mesh.tris[i].a]);
            }
            Real avglength = sumlengths / (3 * mesh.tris.size());
            param = avglength / 2;
            Vector3 bmin, bmax;
            mesh.GetAABB(bmin, bmax);
            param = Min(param, 0.25 * (bmax.x - bmin.x));
            param = Min(param, 0.25 * (bmax.y - bmin.y));
            param = Min(param, 0.25 * (bmax.z - bmin.z));
            LOG4CXX_INFO(GET_LOGGER(Geometry), "AnyGeometry::Convert: Auto-determined grid resolution " << param);
        }
        //TODO: this expands the occupied geometry by a margin -- including an additional margin will not be correct
        MeshToOccupancyGrid(mesh,data,param,domainExpansion);
        return true;
        }
    case Type::ImplicitSurface:
        {
        const Meshing::VolumeGrid& gdata = dynamic_cast<const Geometry3DImplicitSurface*>(geom)->data;
        data.MakeSimilar(gdata);
        for(int i=0;i<gdata.value.m;i++)
            for(int j=0;j<gdata.value.n;j++)
                for(int k=0;k<gdata.value.p;k++)
                    if(gdata.value(i,j,k) < 0)
                        data.value(i,j,k) = 1.0;
                    else
                        data.value(i,j,k) = 0.0;
        return true;
        }
    default:
        return false;
    }
}

bool Geometry3DOccupancyGrid::Merge(const Geometry3D* geom,const RigidTransform* Tgeom)
{
    if(Tgeom) {
        Geometry3D* temp = geom->Copy();
        if(!temp->Transform(*Tgeom)) return false; //TODO: transformed occupancy grids / implicit surfaces
        bool res = Merge(temp);
        delete temp;
        return res;
    }
    if(geom->GetType() == Type::OccupancyGrid) {
        auto* g = dynamic_cast<const Geometry3DOccupancyGrid*>(geom);
        if(g->data.value.empty()) return true;
        if(data.value.empty()) {
            data = g->data;
            return true;
        }
        if(data.IsSimilar(g->data)) {
            data.Max(g->data);
        }
        else {
            Meshing::VolumeGrid temp;
            temp.MakeSimilar(data);
            temp.ResampleAverage(g->data);
            data.Max(temp);
        }
        return true;
    }
    else if(geom->GetType() == Type::ImplicitSurface) {
        auto* temp = geom->Convert(Type::OccupancyGrid);
        if(!temp) {
            LOG4CXX_WARN(GET_LOGGER(Geometry),"Geometry "<<Geometry3D::TypeName(geom->GetType())<<" could not be converted to OccupancyGrid");
            return false;
        }
        bool res=Merge(temp,Tgeom);
        delete temp;
        return res;
    }
    else if(geom->GetType() == Type::ConvexHull) {
        auto* g = dynamic_cast<const Geometry3DConvexHull*>(geom);
        ConvexHullOccupancyGridFill(g->data, data);
        return true;
    }
    else if(geom->GetType() == Type::Primitive) {
        auto* g = dynamic_cast<const Geometry3DPrimitive*>(geom);
        PrimitiveOccupancyGridFill(g->data, data);
        return true;
    }
    else if(geom->GetType() == Type::TriangleMesh) {
        auto* g = dynamic_cast<const Geometry3DTriangleMesh*>(geom);
        MeshOccupancyGridFill(g->data, data);
        return true;
    }
    else if(geom->GetType() == Type::PointCloud) {
        auto* g = dynamic_cast<const Geometry3DPointCloud*>(geom);
        PointCloudOccupancyGridFill(g->data, data);
        return true;
    }
    else {
        return false;
    }
}

Geometry3D* Geometry3DOccupancyGrid::Remesh(Real resolution,bool refine,bool coarsen) const
{
    const Meshing::VolumeGrid& grid = data;
    auto* res = new Geometry3DOccupancyGrid;
    Vector3 size=grid.GetCellSize();
    if((resolution < size.x && refine) || (resolution > size.x && coarsen) ||
    (resolution < size.y && refine) || (resolution > size.y && coarsen) ||
    (resolution < size.z && refine) || (resolution > size.z && coarsen)) {
        int m = (int)Ceil((grid.bb.bmax.x-grid.bb.bmin.x) / resolution);
        int n = (int)Ceil((grid.bb.bmax.y-grid.bb.bmin.y) / resolution);
        int p = (int)Ceil((grid.bb.bmax.z-grid.bb.bmin.z) / resolution);
        Meshing::VolumeGrid& output = res->data;
        output.Resize(m,n,p);
        output.bb = grid.bb;
        output.ResampleAverage(grid);
    }
    else {
        res->data = grid;
    }
    return res;
}

Collider3DOccupancyGrid::Collider3DOccupancyGrid(shared_ptr<Geometry3DOccupancyGrid> _data)
:data(_data)
{
    currentTransform.setIdentity();
    Reset();
}

void Collider3DOccupancyGrid::Reset()
{
    //recompute occupied cells and surface cells
    occupiedCells.resize(0);
    surfaceCells.resize(0);
    Meshing::VolumeGrid::iterator it = data->data.getIterator();
    for(;!it.isDone();++it) {
        if(*it > data->occupancyThreshold) {
            occupiedCells.push_back(it.getIndex());
            IntTriple temp = it.getIndex();
            if(temp.a <= 0 || temp.a+1 >= data->data.value.m) surfaceCells.push_back(it.getIndex());
            else if(temp.b <= 0 || temp.b+1 >= data->data.value.n) surfaceCells.push_back(it.getIndex());
            else if(temp.c <= 0 || temp.c+1 >= data->data.value.p) surfaceCells.push_back(it.getIndex());
            else {
                temp.a--;
                if(data->data.value(temp) <= data->occupancyThreshold) surfaceCells.push_back(it.getIndex());
                temp.a+=2;
                if(data->data.value(temp) <= data->occupancyThreshold) surfaceCells.push_back(it.getIndex());
                temp.a--;
                temp.b--;
                if(data->data.value(temp) <= data->occupancyThreshold) surfaceCells.push_back(it.getIndex());
                temp.b+=2;
                if(data->data.value(temp) <= data->occupancyThreshold) surfaceCells.push_back(it.getIndex());
                temp.b--;
                temp.c--;
                if(data->data.value(temp) <= data->occupancyThreshold) surfaceCells.push_back(it.getIndex());
                temp.c+=2;
                if(data->data.value(temp) <= data->occupancyThreshold) surfaceCells.push_back(it.getIndex());
            }
        }
    }
}


int Collider3DOccupancyGrid::PointToElement(const Vector3 &ptworld) const
{
  Vector3 plocal;
  currentTransform.mulInverse(ptworld, plocal);
  return data->PointToElement(plocal);
}

bool Collider3DOccupancyGrid::Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{
    elements1.resize(0);
    elements2.resize(0);

    if(geom->GetType() == AnyGeometry3D::Type::Primitive ||
       geom->GetType() == AnyGeometry3D::Type::ConvexHull ||
       geom->GetType() == AnyGeometry3D::Type::PointCloud || 
       geom->GetType() == AnyGeometry3D::Type::TriangleMesh) {
        return ::CollidesBruteForce(*this,geom,0.0,elements1,elements2,maxcollisions);
    }
    else if(geom->GetType() == AnyGeometry3D::Type::OccupancyGrid) {
        Collider3DOccupancyGrid* grid = dynamic_cast<Collider3DOccupancyGrid*>(geom);
        bool res=Geometry::Collides(*this,*grid,0.0,elements1,elements2,maxcollisions);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size()==elements2.size());
        return true;
    }
    else if(geom->GetType() == AnyGeometry3D::Type::ImplicitSurface) {
        Collider3DImplicitSurface* grid = dynamic_cast<Collider3DImplicitSurface*>(geom);
        bool res=Geometry::Collides(*grid, *this, 0, elements2, elements1, maxcollisions);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size()==elements2.size());
        return true;   
    }
    else {
        LOG4CXX_ERROR(GET_LOGGER(Geometry),"OccupancyGrid::Collides: Unsupported geometry type "<<Geometry3D::TypeName(geom->GetType()));
        return false;
    }
}

bool Collider3DOccupancyGrid::Contains(const Vector3& pt) const
{ 
    Vector3 ptlocal;
    currentTransform.mulInverse(pt,ptlocal);
    IntTriple ind;
    Vector3 u;
    if(!data->data.GetIndexAndParamsChecked(ptlocal,ind,u)) return false;
    else 
        return (data->data.value(ind) > data->occupancyThreshold);
}

bool Collider3DOccupancyGrid::WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{ 
    if(geom->GetType() == AnyGeometry3D::Type::Primitive ||
       geom->GetType() == AnyGeometry3D::Type::ConvexHull ||
       geom->GetType() == AnyGeometry3D::Type::PointCloud || 
       geom->GetType() == AnyGeometry3D::Type::TriangleMesh) {
        return ::CollidesBruteForce(*this,geom,d,elements1,elements2,maxcollisions);
    }
    else if(geom->GetType() == AnyGeometry3D::Type::ImplicitSurface) {
        Collider3DImplicitSurface* grid = dynamic_cast<Collider3DImplicitSurface*>(geom);
        bool res=::Collides(*grid, *this, d, elements2, elements1, maxcollisions);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size()==elements2.size());
        return true;   
    }
    return false;
}

bool Collider3DOccupancyGrid::RayCast(const Ray3D& r,Real margin,Real& distance,int& element)
{
    Ray3D rlocal;
    RigidTransform Tinv;
    Tinv.setInverse(currentTransform);
    rlocal.setTransformed(r,Tinv);
    if(margin > 0) {
        printf("TODO: ray cast with margin\n");
        return false;
    }

    Real u1=0,u2=Inf;
    if(!r.intersects(data->data.bb,u1,u2)) {
        distance = Inf;
        element = -1;
        return true;
    }

    Segment3D s;
    rlocal.eval(u1,s.a);
    rlocal.eval(u2,s.b);
    vector<IntTriple> cells;
    vector<Real> params;
    Meshing::GetSegmentCells(s,data->data.value.m,data->data.value.n,data->data.value.p,data->data.bb,cells,&params);
    for(size_t i=0;i<cells.size();i++) {
        const auto& c=cells[i];
        if(data->data.value(c) > data->occupancyThreshold) {
            Vector3 pt = s.a + params[i]*(s.b-s.a);
            distance = (pt-rlocal.source).dot(rlocal.direction);
            element = PointToElement(currentTransform*pt);
            return true;
        }
    }
    distance = Inf;
    element = -1;
    return true;
}


Geometry3D* Geometry3DOccupancyGrid::ExtractROI(const AABB3D& bb,int flag) const
{
  if(flag & ExtractROIFlagInvert) return NULL;
  const Meshing::VolumeGrid& grid = data;
  auto* res = new Geometry3DOccupancyGrid;
  IntTriple imin,imax;
  Vector3 umin,umax;
  grid.GetIndexAndParamsClamped(bb.bmin,imin,umin);
  grid.GetIndexAndParamsClamped(bb.bmax,imax,umax);
  if(flag & ExtractROIFlagWithin || flag & ExtractROIFlagTouching) {
    //get only cells entirely within or touching the bbox
    if(umax.x == 1) imax.a += 1;
    if(umax.y == 1) imax.b += 1;
    if(umax.z == 1) imax.c += 1;
    if(umin.x == 0) imin.a -= 1;
    if(umin.y == 0) imin.b -= 1;
    if(umin.z == 0) imin.c -= 1;
    if(flag & ExtractROIFlagWithin) {
      imax.a -= 1;
      imax.b -= 1;
      imax.c -= 1;
      imin.a += 1;
      imin.b += 1;
      imin.c += 1;
    }
    AABB3D cellmin,cellmax;
    grid.GetCell(imin,cellmin);
    grid.GetCell(imax,cellmax);
    res->data.bb.bmin = cellmin.bmin;
    res->data.bb.bmax = cellmax.bmax;
    if(imin.a > imax.a || imin.b > imax.b || imin.c > imax.c) {
      res->data.value.resize(0,0,0);
      return res;
    }
    res->data.Resize(imax.a-imin.a+1,imax.b-imin.b+1,imax.c-imin.c+1);
    for(int i=imin.a;i<=imax.a;i++)
      for(int j=imin.b;j<=imax.b;j++)
        for(int k=imin.c;k<=imax.c;k++)
          res->data.value(i-imin.a,j-imin.b,k-imin.c) = grid.value(i,j,k);
    return res;
  }
  else if(flag & ExtractROIFlagIntersection) {
    //need to resample
    AABB3D cellmin,cellmax;
    grid.GetCell(imin,cellmin);
    grid.GetCell(imax,cellmax);
    Vector3 bmin = cellmin.bmin;
    bmin.x += umin.x*(cellmax.bmin.x-cellmin.bmin.x); 
    bmin.y += umin.y*(cellmax.bmin.y-cellmin.bmin.y);
    bmin.z += umin.z*(cellmax.bmin.z-cellmin.bmin.z);
    Vector3 bmax = cellmax.bmax;
    bmax.x += umax.x*(cellmax.bmax.x-cellmin.bmax.x);
    bmax.y += umax.y*(cellmax.bmax.y-cellmin.bmax.y);
    bmax.z += umax.z*(cellmax.bmax.z-cellmin.bmax.z);
    res->data.bb.bmin = bmin;
    res->data.bb.bmax = bmax;
    Vector3 size=grid.GetCellSize();
    res->data.ResizeByResolution(size);
    res->data.ResampleAverage(grid);
    return res;
  }
  delete res;
  return NULL;
}