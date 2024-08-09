#include "CollisionHeightmap.h"
#include "GeometryTypeImpl.h"
#include "Conversions.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/Rasterize.h>
#include <KrisLibrary/math3d/Triangle2D.h>
#include <KrisLibrary/math3d/Ray2D.h>
#include "AnyGeometry.h"

DECLARE_LOGGER(Geometry)

using namespace Geometry;

bool Geometry3DHeightmap::Load(const char* fn)
{
    return data.Load(fn);
}

bool Geometry3DHeightmap::Save(const char* fn) const
{
    return data.Save(fn);
}

bool Geometry3DHeightmap::Load(istream& in)
{
    return data.Load(in);
}

bool Geometry3DHeightmap::Save(ostream& out) const
{
    LOG4CXX_WARN(GET_LOGGER(Geometry),"Geometry3DHeightmap::Save: saving to ostream, using default height / color file names height.ppm / color.ppm");
    return data.Save(out,"height.ppm","color.ppm");
}

bool Geometry3DHeightmap::Empty() const
{
    return data.heights.empty();
}

size_t Geometry3DHeightmap::NumElements() const
{
    return data.heights.m*data.heights.n;
}

shared_ptr<Geometry3D> Geometry3DHeightmap::GetElement(int elem) const
{
    return shared_ptr<Geometry3D>();
}

AABB3D Geometry3DHeightmap::GetAABB() const
{
    return data.GetAABB();
}

bool Geometry3DHeightmap::Transform(const Matrix4& mat) 
{
    if(mat(1,0) != 0.0 || mat(2,0) != 0.0 || mat(0,1) != 0.0 || mat(2,1) != 0.0 || mat(0,2) != 0.0 || mat(1,2) != 0.0) {
        return false;
    }
    data.xysize.x *= mat(0,0);
    data.xysize.y *= mat(1,1);
    data.offset.x *= mat(0,0);
    data.offset.y *= mat(1,1);
    data.offset.z *= mat(2,2);
    data.Scale(mat(2,2));
    data.offset += Vector3(mat(0,3),mat(1,3),mat(2,3));
    return true;
}

Geometry3D* Geometry3DHeightmap::ConvertTo(Type restype,Real param,Real domainExpansion) const
{
    if(restype == Type::ImplicitSurface || restype == Type::OccupancyGrid) {
        if(data.perspective) {
            LOG4CXX_WARN(GET_LOGGER(Geometry),"Cannot convert perspective heightmap to implicit surface");
            return NULL;
        }
        if(param <= 0) {
            //heightmap's height range / 256
            auto bb=data.GetAABB();
            Real res = (bb.bmax.z-bb.bmin.z)/256;
            param = res;
        }
        Meshing::VolumeGrid grid;
        grid.bb = data.GetAABB();
        grid.value.resize(data.heights.m,data.heights.n,int(grid.bb.bmax.z - grid.bb.bmin.z)/param);
        Vector3 c;
        for(int i=0;i<data.heights.m;i++)
            for(int j=0;j<data.heights.n;j++)
                for(int k=0;k<grid.value.p;k++) {
                    grid.GetCellCenter(i,j,k,c);
                    if(restype == Type::ImplicitSurface)
                        grid.value(i,j,k) = c.z - data.heights(i,j);
                    else
                        grid.value(i,j,k) = (c.z > data.heights(i,j) ? 0.0 : 1.0);
                }
        if(restype == Type::ImplicitSurface)
            return new Geometry3DImplicitSurface(grid);
        else
            return new Geometry3DOccupancyGrid(grid);
    }
    else if(restype == Type::PointCloud) {
        Meshing::PointCloud3D pc;
        data.GetVertices(pc.points);
        vector<Vector3> rgb(pc.points.size());
        data.GetVertexColors(rgb);
        if(!rgb.empty()) {
            vector<Real> r,g,b;
            r.resize(rgb.size());
            g.resize(rgb.size());
            b.resize(rgb.size());
            for(size_t i=0;i<rgb.size();i++) {
                rgb[i].get(r[i],g[i],b[i]);
            }
            pc.SetColors(r,g,b);
        }
        return new Geometry3DPointCloud(pc);
    }
    else if(restype == Type::TriangleMesh) {
        Meshing::TriMesh mesh;
        shared_ptr<GLDraw::GeometryAppearance> appearance;
        if(data.HasColors()) {
            appearance = make_shared<GLDraw::GeometryAppearance>();
            HeightmapToMesh(data,mesh,*appearance);
        }
        else {
            HeightmapToMesh(data,mesh);
        }
        return new Geometry3DTriangleMesh(mesh,appearance);
    }
    return NULL;
}

bool Geometry3DHeightmap::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion)
{
    data.perspective = false;
    auto bb=geom->GetAABB();
    bb.bmin -= Vector3(domainExpansion);
    bb.bmax += Vector3(domainExpansion);
    data.offset = (bb.bmin+bb.bmax)*0.5;
    data.offset.z = bb.bmin.z;
    data.xysize.set(bb.bmax.x-bb.bmin.x,bb.bmax.y-bb.bmin.y);
    int m,n;
    if(param == 0) {
        m=256;
        n=256;
    }
    else {
        m = int(data.xysize.x / param) + 1;
        n = int(data.xysize.y / param) + 1;
    }
    data.heights.resize(m,n);
    fill(data.heights.begin(),data.heights.end(),0);
    if(!Merge(geom)) return false;
    return true;
}

Geometry3D* Geometry3DHeightmap::Remesh(Real resolution,bool refine,bool coarsen) const
{
    if(resolution <= 0) return NULL;
    Meshing::Heightmap res = data;
    res.heights.resize(int(data.xysize.x/resolution)+1,int(data.xysize.y/resolution)+1);
    if(data.HasColors()) {
        res.colors.initialize(res.heights.m,res.heights.n,data.colors.format);
    }
    res.Remesh(data);
    return new Geometry3DHeightmap(std::move(res));
}


bool Geometry3DHeightmap::Merge(const Geometry3D* geom,const RigidTransform* Tgeom)
{
    switch(geom->GetType()) {
    case Type::PointCloud:
        {
        //TODO: colors
        const Geometry3DPointCloud* pc = dynamic_cast<const Geometry3DPointCloud*>(geom);
        data.FusePointCloud(pc->data,Tgeom);
        return true;
        }
    case Type::TriangleMesh:
        {
        //TODO: colors
        const Geometry3DTriangleMesh* mesh = dynamic_cast<const Geometry3DTriangleMesh*>(geom);
        data.FuseMesh(mesh->data,Tgeom);
        return true;
        }
    case Type::OccupancyGrid:
        {
        shared_ptr<Geometry3D> g2(geom->Convert(Type::PointCloud,0.5));
        if(g2) {
            return Merge(g2.get(),Tgeom);
        }
        return false;
        }
    case Type::ImplicitSurface:
        {
        shared_ptr<Geometry3D> g2(geom->Convert(Type::PointCloud,0.0));
        if(g2) {
            return Merge(g2.get(),Tgeom);
        }
        return false;
        }
    case Type::ConvexHull:
        {
        const Geometry3DConvexHull* ch = dynamic_cast<const Geometry3DConvexHull*>(geom);
        ConvexHull3D hull = ch->data;
        if(Tgeom)
            hull.Transform(*Tgeom);
        auto bb = hull.GetAABB();
        //get range of bb to minimize # of rays cast
        IntPair lo = data.GetIndex(bb.bmin);
        IntPair hi = data.GetIndex(bb.bmax);
        if(hi.a < 0 || hi.b < 0 || lo.a >= data.heights.m || lo.b >= data.heights.n) return true;
        hi.a = Min(hi.a,data.heights.m-1);
        hi.b = Min(hi.b,data.heights.n-1);
        lo.a = Max(lo.a,0);
        lo.b = Max(lo.b,0);
        Ray3D ray;
        for(int i=lo.a;i<=hi.a;i++) {
            for(int j=lo.b;j<hi.b;j++) {
                data.GetVertexRay(i,j,ray.source,ray.direction);
                if(!data.perspective)  {
                    //ray needs to go from top down
                    ray.source.z = bb.bmax.z + 1.0;
                    ray.direction.set(0,0,-1);
                }
                Real dist;
                if(hull.RayCast(ray,&dist,data.heights(i,j))) {
                    if(data.perspective)
                        data.heights(i,j) = Min(float(dist),data.heights(i,j));
                    else
                        data.heights(i,j) = Max(float(bb.bmax.z + 1.0 - dist - data.offset.z),data.heights(i,j));
                }
            }
        }
        return true;
        }
    case Type::Primitive:
        {
        const Geometry3DPrimitive* p = dynamic_cast<const Geometry3DPrimitive*>(geom);
        GeometricPrimitive3D prim = p->data;
        if(Tgeom)
            prim.Transform(*Tgeom);
        auto bb = prim.GetAABB();
        //get range of bb to minimize # of rays cast
        IntPair lo = data.GetIndex(bb.bmin);
        IntPair hi = data.GetIndex(bb.bmax);
        if(hi.a < 0 || hi.b < 0 || lo.a >= data.heights.m || lo.b >= data.heights.n) return true;
        hi.a = Min(hi.a,data.heights.m-1);
        hi.b = Min(hi.b,data.heights.n-1);
        lo.a = Max(lo.a,0);
        lo.b = Max(lo.b,0);
        int nhits = 0;
        Ray3D ray;
        for(int i=lo.a;i<=hi.a;i++) {
            for(int j=lo.b;j<hi.b;j++) {
                data.GetVertexRay(i,j,ray.source,ray.direction);
                if(!data.perspective)  {
                    //ray needs to go from top down
                    ray.source.z = bb.bmax.z + 1.0;
                    ray.direction.set(0,0,-1);
                }
                Vector3 pt;
                if(prim.RayCast(ray,pt)) {
                    nhits ++;
                    Real dist = ray.closestPointParameter(pt);
                    if(data.perspective)
                        data.heights(i,j) = Min(float(dist),data.heights(i,j));
                    else
                        data.heights(i,j) = Max(float(bb.bmax.z + 1.0 - dist - data.offset.z),data.heights(i,j));
                }
            }
        }
        return true;
        }
    case Type::Group:
        {
        const Geometry3DGroup* g = dynamic_cast<const Geometry3DGroup*>(geom);
        for(size_t i=0;i<g->data.size();i++) {
            if(!Merge(g->data[i].data.get(),Tgeom)) return false;
        }
        return true;
        }
    default:
        return false;
    }
}


Collider3DHeightmap::Collider3DHeightmap(shared_ptr<Geometry3DHeightmap> _data)
{
    data = _data;
    currentTransform.setIdentity();
}

void Collider3DHeightmap::Reset()
{}

bool Collider3DHeightmap::Contains(const Vector3& pt,bool& result)
{
    if(data->data.GetHeight(pt) > pt.z) {
        result = !data->data.perspective;
    }
    else {
        result = data->data.perspective;
    }
    return true;
}

bool Collider3DHeightmap::Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{
    //TODO
    return false;
}
bool Collider3DHeightmap::WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{
    //TODO
    return false;
}
bool Collider3DHeightmap::RayCast(const Ray3D& r,Real margin,Real& distance,int& element)
{
    element = -1;
    vector<IntPair> cells;
    Ray3D rlocal,rimg;
    currentTransform.mulInverse(r.source,rlocal.source);
    currentTransform.mulVectorInverse(r.direction,rlocal.direction);
    rimg.source = data->data.Project(rlocal.source);
    Vector3 temp = data->data.Project(rlocal.source+1.0*rlocal.direction);
    rimg.direction = temp - rimg.source;
    Ray2D ray2;
    ray2.source.set(rimg.source);
    ray2.direction.set(rimg.direction);
    Meshing::GetRayCells(ray2,cells,0,0,data->data.heights.m,data->data.heights.n);
    for(const auto& c:cells) {
        Real z = data->data.heights(c.a,c.b);
        AABB3D cell;
        cell.bmin.x = c.a;
        cell.bmin.y = c.b;
        cell.bmin.z = -Inf;
        cell.bmax.x = c.a+1;
        cell.bmax.y = c.b+1;
        cell.bmax.z = Inf;
        Real tmin,tmax;
        if(rimg.intersects(cell,tmin,tmax)) {
            Real zray1 = rimg.source.z + tmin*rimg.direction.z;
            Real zray2 = rimg.source.z + tmax*rimg.direction.z;
            if(data->data.perspective) {
                if(zray1 >= z || zray2 >= z) {
                    distance = tmin;
                    element = c.a + c.b*data->data.heights.m;
                    return true;
                }
            }
            else {
                if(zray1 <= z || zray2 <= z) {
                    distance = tmin;
                    element = c.a + c.b*data->data.heights.m;
                    return true;
                }
            }
        }
    }
    return true;
}