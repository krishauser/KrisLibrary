#include "CollisionOccupancyGrid.h"
#include "Conversions.h"
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/utils/stl_tr1.h>
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/math3d/clip.h>
#include <KrisLibrary/math3d/Segment3D.h>

using namespace Geometry;

DECLARE_LOGGER(Geometry)

int PointIndex(const Collider3DOccupancyGrid &s, const Vector3 &ptworld)
{
  Vector3 plocal;
  s.currentTransform.mulInverse(ptworld, plocal);
  IntTriple cell;
  s.data->data.GetIndex(plocal, cell);
  if (cell.a < 0)
    cell.a = 0;
  if (cell.a >= s.data->data.value.m)
    cell.a = s.data->data.value.m - 1;
  if (cell.b < 0)
    cell.b = 0;
  if (cell.b >= s.data->data.value.n)
    cell.b = s.data->data.value.n - 1;
  if (cell.c < 0)
    cell.c = 0;
  if (cell.c >= s.data->data.value.p)
    cell.c = s.data->data.value.p - 1;
  return cell.a * s.data->data.value.n * s.data->data.value.p + cell.b * s.data->data.value.p + cell.c;
}

Geometry3D* Geometry3DOccupancyGrid::ConvertTo(Type restype,Real param,Real domainExpansion) const
{
    switch(restype) {
    case Type::PointCloud:
        {
        auto* pc = new Geometry3DPointCloud();
        Meshing::VolumeGridIterator<Real> it=data.getIterator();
        Vector3 c;
        while(!it.isDone()) {
            if(*it > 0) {
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
            {1,0,0,  0,1,0,   1,1,1,  1,0,1}, //#+x
            {0,1,0,  0,1,1,   1,1,1,  1,1,0}, //#+y
            {0,0,1,  1,0,1,   1,1,1,  0,1,1}, //#+z
        };
        UNORDERED_MAP_TEMPLATE<IntTriple,int> vertexMap;
        while(!it.isDone()) {
            if(*it > 0) {
                vector<int> faces;
                IntTriple ind = it.getIndex();
                ind.a += 1;
                if(ind.a < data.value.m && data.value(ind) <= 0) faces.push_back(1);
                ind.a -= 1;
                ind.a -= 1;
                if(ind.a >= 0 && data.value(ind) <= 0) faces.push_back(-1);
                ind.a -= 1;
                ind.b += 1;
                if(ind.b < data.value.m && data.value(ind) <= 0) faces.push_back(2);
                ind.b -= 1;
                ind.b -= 1;
                if(ind.b >= 0 && data.value(ind) <= 0) faces.push_back(-2);
                ind.b -= 1;
                ind.c += 1;
                if(ind.c < data.value.m && data.value(ind) <= 0) faces.push_back(3);
                ind.c -= 1;
                ind.c -= 1;
                if(ind.c >= 0 && data.value(ind) <= 0) faces.push_back(-3);
                ind.c -= 1;
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
        {
        auto* isurf = geom->ConvertTo(Type::ImplicitSurface, param, domainExpansion);
        if(!isurf) return false;
        ConvertFrom(isurf);
        delete isurf;
        return true;
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
    default:
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
}

void Collider3DOccupancyGrid::Reset()
{}

bool Collider3DOccupancyGrid::Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{ return false; }

bool Collider3DOccupancyGrid::Contains(const Vector3& pt,bool& result)
{ 
    Vector3 ptlocal;
    currentTransform.mulInverse(pt,ptlocal);
    result = data->data.GetValue(ptlocal) > 0;
    return true;
}

bool Collider3DOccupancyGrid::WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{ return false; }

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
    Real u1,u2;
    if(!ClipLine(rlocal.source,rlocal.direction,data->data.bb,u1,u2)) {
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
        if(data->data.value(c) > 0) {
            Vector3 pt = s.a + params[i]*(s.b-s.a);
            distance = (pt-rlocal.source).dot(rlocal.direction);
            element = PointIndex(*this,currentTransform*pt);
            return true;
        }
    }
    distance = Inf;
    element = -1;
    return true;
}

Collider3D* Collider3DOccupancyGrid::Slice(const RigidTransform& T,Real tol) const
{ return NULL; }
Collider3D* Collider3DOccupancyGrid::ExtractROI(const AABB3D& bb,int flag) const
{ return NULL; }
Collider3D* Collider3DOccupancyGrid::ExtractROI(const Box3D& bb,int flag) const
{ return NULL; }