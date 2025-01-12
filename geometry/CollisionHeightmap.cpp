#include "CollisionHeightmap.h"
#include "GeometryTypeImpl.h"
#include "Conversions.h"
#include "CollisionPointCloud.h"
#include "CollisionConvexHull.h"
#include "CollisionOccupancyGrid.h"
#include "CollisionImplicitSurface.h"
#include "CollisionPrimitive.h"
#include "SpiralIterator.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/Rasterize.h>
#include <KrisLibrary/math3d/Triangle2D.h>
#include <KrisLibrary/math3d/Ray2D.h>
#include "AnyGeometry.h"
#include <numeric>

DECLARE_LOGGER(Geometry)

namespace Geometry {

int Collider3DHeightmap::PointToElement(const Vector3& ptworld) const
{
    Vector3 ptlocal;
    currentTransform.mulInverse(ptworld,ptlocal);
    IntPair i = data->data.GetIndex(ptlocal,true);
    return i.a + i.b*data->data.heights.m;
}

bool Collides(Collider3DOccupancyGrid& a, Collider3DHeightmap& b, Real d, vector<int>& elements1, vector<int>& elements2, size_t maxCollisions)
{
    //the method checks all the surface cells of the occupancy grid against the heightmap
    AABB3D bb;
    Vector3 extremumLocal,extremum;
    Vector3 fwd = b.data->data.viewport.forward();
    Vector3 fwda;
    a.currentTransform.R.mulTranspose(b.currentTransform.R * fwd, fwda);
    for(const auto &cell : a.surfaceCells) {
        a.data->data.GetCell(cell,bb);
        if(fwda.x < 0) extremumLocal.x = bb.bmin.x;
        else extremumLocal.x = bb.bmax.x;
        if(fwda.y < 0) extremumLocal.y = bb.bmin.y;
        else extremumLocal.y = bb.bmax.y;
        if(fwda.z < 0) extremumLocal.z = bb.bmin.z;
        else extremumLocal.z = bb.bmax.z;
        a.currentTransform.mul(extremumLocal,extremum);
        bool res;
        if(!b.Contains(extremum,res)) return false;
        if(res) {
            int aelem = a.data->IndexToElement(cell);
            int belem = b.PointToElement(extremum);
            elements1.push_back(aelem);
            elements2.push_back(belem);
            if(elements1.size() >= maxCollisions) return true;
        }
    }
    return !elements1.empty();
}



} //namespace Geometry

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
    //(i,j) => i+j*m
    return data.heights.m*data.heights.n;
}

shared_ptr<Geometry3D> Geometry3DHeightmap::GetElement(int elem) const
{
    int i=elem % data.heights.m;
    int j=elem / data.heights.m;
    GeometricPrimitive3D g(Vector3(data.GetVertex(i,j)));
    return make_shared<Geometry3DPrimitive>(g);
}

AABB3D Geometry3DHeightmap::GetAABB() const
{
    return data.GetAABB();
}

bool Geometry3DHeightmap::Transform(const RigidTransform& T)
{
    data.viewport.pose = T*data.viewport.pose;
    return true;
}

bool Geometry3DHeightmap::Transform(const Matrix4& mat) 
{
    if(mat(1,0) != 0.0 || mat(2,0) != 0.0 || mat(0,1) != 0.0 || mat(2,1) != 0.0 || mat(0,2) != 0.0 || mat(1,2) != 0.0) {
        Matrix3 R;
        mat.get(R);
        Matrix3 RRt, I;
        RRt.mulTransposeB(R,R);
        I.setIdentity();
        if(!RRt.isEqual(I,1e-5)) {
            LOG4CXX_ERROR(GET_LOGGER(Geometry),"Geometry3DHeightmap::Transform: matrix is not a pure scaling or pure rotation matrix");
            return false;
        }
        Vector3 t = mat.getTranslation();
        return Transform(RigidTransform(R,t));
    }
    data.viewport.fx /= mat(0,0);
    data.viewport.fy /= mat(1,1);
    data.viewport.pose.t.x *= mat(0,0);
    data.viewport.pose.t.y *= mat(1,1);
    data.viewport.pose.t.z *= mat(2,2);
    if(mat(2,2) != 1.0) data.Scale(mat(2,2));
    data.viewport.pose.t += Vector3(mat(0,3),mat(1,3),mat(2,3));
    return true;
}

Geometry3D* Geometry3DHeightmap::ConvertTo(Type restype,Real param,Real domainExpansion) const
{
    if(restype == Type::ImplicitSurface || restype == Type::OccupancyGrid) {
        if(param <= 0) {
            //heightmap's height range / 128
            Vector2 hrange = data.ValidHeightRange();
            Real res = (hrange.y-hrange.x)/128;
            if(res==0) res=1;
            param = res;
        }
        Meshing::VolumeGrid grid;
        grid.bb = data.GetAABB();
        grid.bb.bmax.z += Max(domainExpansion,0.5*param);
        Real large = grid.bb.bmax.distance(grid.bb.bmin);
        Matrix3 I;
        I.setIdentity();
        if(!data.viewport.perspective && data.viewport.pose.R == I) {  //fast method
            int zdivs = int((grid.bb.bmax.z - grid.bb.bmin.z)/param);
            if(zdivs <= 0) zdivs = 1;
            grid.value.resize(data.heights.m,data.heights.n,zdivs);
            Vector3 c;
            for(int i=0;i<data.heights.m;i++)
                for(int j=0;j<data.heights.n;j++) {
                    for(int k=0;k<grid.value.p;k++) {
                        Real z= grid.bb.bmin.z + (Real(k)+0.5)*(grid.bb.bmax.z-grid.bb.bmin.z)/grid.value.p;
                        if(restype == Type::ImplicitSurface) {
                            if(!IsFinite(data.heights(i,j))) {
                                grid.value(i,j,k) = large;
                            }
                            else {
                                grid.value(i,j,k) = z - data.viewport.pose.t.z - data.heights(i,j);
                            }
                        }
                        else {
                            if(!IsFinite(data.heights(i,j))) {
                                grid.value(i,j,k) = 0.0;
                            }
                            else {
                                grid.value(i,j,k) = (z - data.viewport.pose.t.z > data.heights(i,j) ? 0.0 : 1.0);
                            }
                        }
                    }
                }
        }
        else {
            int xdivs = int((grid.bb.bmax.x - grid.bb.bmin.x)/param);
            int ydivs = int((grid.bb.bmax.y - grid.bb.bmin.y)/param);
            int zdivs = int((grid.bb.bmax.z - grid.bb.bmin.z)/param);
            if(xdivs <= 0) xdivs = 1;
            if(ydivs <= 0) ydivs = 1;
            if(zdivs <= 0) zdivs = 1;
            grid.value.resize(xdivs,ydivs,zdivs);
            Vector3 c;
            for(int i=0;i<grid.value.m;i++)
                for(int j=0;j<grid.value.n;j++)
                    for(int k=0;k<grid.value.p;k++) {
                        grid.GetCellCenter(i,j,k,c);
                        Real dh = data.GetHeightDifference(c);
                        if(IsFinite(dh)) {
                            if(data.viewport.perspective) {  //higher values are "behind"
                                if(restype == Type::ImplicitSurface)
                                    grid.value(i,j,k) = -dh;
                                else
                                    grid.value(i,j,k) = (dh < 0 ? 0.0 : 1.0);
                            }
                            else {
                                if(restype == Type::ImplicitSurface)
                                    grid.value(i,j,k) = dh;
                                else
                                    grid.value(i,j,k) = (dh > 0 ? 0.0 : 1.0);
                            }
                        }
                        else {
                            if(restype == Type::ImplicitSurface)
                                grid.value(i,j,k) = large;
                            else
                                grid.value(i,j,k) = 0.0;
                        }
                    }
        }
        // if(restype == Type::ImplicitSurface) {
        //     printf("IMPLICIT SURFACE RANGE %f %f\n",*std::min_element(grid.value.begin(),grid.value.end()),*std::max_element(grid.value.begin(),grid.value.end()));
        //     int nocc = 0;
        //     for(int i=0;i<grid.value.m;i++)
        //         for(int j=0;j<grid.value.n;j++)
        //             for(int k=0;k<grid.value.p;k++) 
        //                 nocc += (grid.value(i,j,k) < 0 ? 1 : 0);
        //     printf("NUMBER OF OCCUPIED CELLS: %d / %d\n",nocc,grid.value.m*grid.value.n*grid.value.p);
        // }
        // else {
        //     printf("NUMBER OF OCCUPIED CELLS: %f / %d\n",std::accumulate(grid.value.begin(),grid.value.end(),0.0),grid.value.m*grid.value.n*grid.value.p);
        // }
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
    auto bb=geom->GetAABB();
    bb.bmin -= Vector3(domainExpansion);
    bb.bmax += Vector3(domainExpansion);
    data.viewport.perspective = false;
    data.viewport.pose.R.setIdentity();
    data.viewport.pose.t = (bb.bmin+bb.bmax)*0.5;
    data.viewport.pose.t.z = bb.bmin.z;
    int m,n;
    if(geom->GetType() ==  Type::OccupancyGrid || geom->GetType() ==  Type::ImplicitSurface) {
        const Geometry3DVolume* vol = dynamic_cast<const Geometry3DVolume*>(geom);
        m = vol->data.value.m;
        n = vol->data.value.n;
    }
    else {
        if(param == 0) {
            m=256;
            n=256;
        }
        else {
            m = int((bb.bmax.x-bb.bmin.x) / param) + 1;
            n = int((bb.bmax.y-bb.bmin.y) / param) + 1;
        }
    }
    data.viewport.w = m;
    data.viewport.h = n;
    data.SetSize(bb.bmax.x-bb.bmin.x,bb.bmax.y-bb.bmin.y);
    data.heights.resize(m,n);
    fill(data.heights.begin(),data.heights.end(),0);
    if(!Merge(geom)) return false;
    return true;
}

Geometry3D* Geometry3DHeightmap::Remesh(Real resolution,bool refine,bool coarsen) const
{
    if(resolution <= 0) return NULL;
    Camera::Viewport newvp = data.viewport;
    Vector2 xysize = data.GetSize();
    newvp.resize(int(xysize.x/resolution)+1,int(xysize.y/resolution)+1);
    Meshing::Heightmap res;
    res.Remesh(data,newvp);
    return new Geometry3DHeightmap(std::move(res));
}


bool Geometry3DHeightmap::Merge(const Geometry3D* geom,const RigidTransform* Tgeom)
{
    switch(geom->GetType()) {
    case Type::PointCloud:
        {
        //TODO: do we add colors to an existing non-colored heightmap?
        const Geometry3DPointCloud* pc = dynamic_cast<const Geometry3DPointCloud*>(geom);
        data.FusePointCloud(pc->data,Tgeom);
        return true;
        }
    case Type::TriangleMesh:
        {
        //TODO: if mesh has colors, add these to the heightmap
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
        Ray3D ray;
        Real dscale = 1.0;
        for(int i=lo.a;i<=hi.a;i++) {
            for(int j=lo.b;j<hi.b;j++) {
                data.GetVertexRay(i,j,ray.source,ray.direction);
                if(!data.viewport.perspective)  {
                    //ray needs to go from top down
                    ray.source.z = bb.bmax.z + 1.0;
                    ray.direction.inplaceNegative();
                }
                else {
                    //ray is unnormalized, normalize it
                    dscale = ray.direction.norm();
                    ray.direction /= dscale;
                }
                Vector3 pt;
                if(prim.RayCast(ray,pt)) {
                    Real dist = ray.closestPointParameter(pt);
                    if(data.viewport.perspective) {
                        dist *= dscale;
                        data.heights(i,j) = Min(float(dist),data.heights(i,j));
                    }
                    else
                        data.heights(i,j) = Max(float(bb.bmax.z + 1.0 - dist - data.viewport.pose.t.z),data.heights(i,j));
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

Geometry3D* Geometry3DHeightmap::ExtractROI(const AABB3D& bb,int flag) const
{
    if(flag & ExtractROIFlagInvert) return NULL;
    Matrix3 I; I.setIdentity();
    if(data.viewport.pose.R != I) {
        LOG4CXX_INFO(GET_LOGGER(Geometry),"Geometry3DHeightmap::ExtractROI: nontrivial rotation not supported");
        return NULL;
    }
    const Meshing::Heightmap& grid = data;
    if(grid.viewport.perspective) {
        LOG4CXX_INFO(GET_LOGGER(Geometry),"Geometry3DHeightmap::ExtractROI: perspective view not supported");
        return NULL;
    }
    auto* res = new Geometry3DHeightmap;
    IntPair imin,imax;
    Vector2 umin,umax;
    grid.GetIndexAndParams(bb.bmin,imin,umin,/*clamp*/true);
    grid.GetIndexAndParams(bb.bmax,imax,umax,/*clamp*/true);
    if(flag & ExtractROIFlagWithin || flag & ExtractROIFlagTouching) {
        //get only cells entirely within or touching the bbox
        if(umin.x == 0) imin.a -= 1;
        if(umin.y == 0) imin.b -= 1;
        if(umax.x == 1) imax.a += 1;
        if(umax.y == 1) imax.b += 1;
        if(flag & ExtractROIFlagWithin) {
            imax.a -= 1;
            imax.b -= 1;
            imin.a += 1;
            imin.b += 1;
        }
        if(imin.a > imax.a || imin.b > imax.b) {
            //empty
            return res;
        }

        res->data.viewport = grid.viewport;
        res->data.viewport.w = imax.a-imin.a+1;
        res->data.viewport.h = imax.b-imin.b+1;
        res->data.viewport.cx = res->data.viewport.w*0.5;
        res->data.viewport.cy = res->data.viewport.h*0.5;
        //translate to the center of the range
        Vector3 bmin,bmax;
        bmin = grid.GetVertex(imin.a,imin.b,umin.x,umin.y);
        bmax = grid.GetVertex(imax.a,imax.b,umax.x,umax.y);
        res->data.viewport.pose.t = (bmin+bmax)*0.5;
        res->data.viewport.pose.t.z = grid.viewport.pose.t.z;
        res->data.propertyNames = grid.propertyNames;
        res->data.properties.resize(res->data.propertyNames.size());
        res->data.Resize(imax.a-imin.a+1,imax.b-imin.b+1);
        for(int i=imin.a;i<=imax.a;i++)
            for(int j=imin.b;j<=imax.b;j++) {
                res->data.heights(i-imin.a,j-imin.b) = grid.heights(i,j);
                Real zabs = grid.heights(i,j) + grid.viewport.pose.t.z;
                if(zabs > bb.bmax.z || zabs < bb.bmin.z) {
                    res->data.heights(i-imin.a,j-imin.b) = Inf; //invalid value
                }
            }
        if(grid.HasColors()) {
            res->data.AddColors(Vector3(0.0));
            for(int i=imin.a;i<=imax.a;i++)
                for(int j=imin.b;j<=imax.b;j++)
                    res->data.SetVertexColor(i-imin.a,j-imin.b,grid.GetVertexColor(i,j));
        }
        for(size_t p=0;p<res->data.properties.size();p++) {
            for(int i=imin.a;i<=imax.a;i++)
                for(int j=imin.b;j<=imax.b;j++)
                    res->data.properties[p](i-imin.a,j-imin.b) = grid.properties[p](i,j);
        }
        return res;
    }
    else if(flag & ExtractROIFlagIntersection) {
        //need to resample
        Vector3 bmin,bmax;
        bmin = grid.GetVertex(imin.a,imin.b,umin.x,umin.y);
        bmax = grid.GetVertex(imax.a,imax.b,umax.x,umax.y);
        if(bmin.x >= bmax.x || bmin.y >= bmax.y) {
            //empty
            return res;
        }
        Vector2 size = grid.GetCellSize();
        res->data.viewport = grid.viewport;
        res->data.viewport.w = (int)Ceil((bmax.x-bmin.x)/size.x);
        res->data.viewport.h = (int)Ceil((bmax.y-bmin.y)/size.y);
        res->data.viewport.cx = res->data.viewport.w*0.5;
        res->data.viewport.cy = res->data.viewport.h*0.5;
        res->data.viewport.pose.t = (bmin+bmax)*0.5;
        res->data.viewport.pose.t.z = grid.viewport.pose.t.z;
        res->data.Remesh(grid,res->data.viewport);
        return res;
    }
    delete res;
    return NULL;
}



Collider3DHeightmap::Collider3DHeightmap(shared_ptr<Geometry3DHeightmap> _data)
{
    data = _data;
    currentTransform.setIdentity();
    Reset();
}

void Collider3DHeightmap::Reset()
{
    Vector2 hrange = data->data.ValidHeightRange();
    hmin = hrange.x;
    hmax = hrange.y;
}

Box3D Collider3DHeightmap::GetBB() const
{
    ///faster than min/maxing over heights
    AABB3D localBB;
    if(data->data.viewport.perspective) {
        AABB2D bb2d = data->data.viewport.getViewRectangle(hmin,true);
        localBB.bmin.set(bb2d.bmin.x,bb2d.bmin.y,hmin);
        bb2d = data->data.viewport.getViewRectangle(hmax,true);
        localBB.bmax.set(bb2d.bmax.x,bb2d.bmax.y,hmax);
    }
    else {
        AABB2D bb2d = data->data.viewport.getViewRectangle(0,true);
        localBB.bmin.set(bb2d.bmin.x,bb2d.bmin.y,hmin);
        localBB.bmax.set(bb2d.bmax.x,bb2d.bmax.y,hmax);
    }
    Box3D res;
    res.setTransformed(localBB,currentTransform*data->data.viewport.pose);
    return res;
}

bool Collider3DHeightmap::Contains(const Vector3& pt,bool& result)
{
    Vector3 ptLocal;
    currentTransform.mulInverse(pt,ptLocal);
    Real h = data->data.GetHeight(ptLocal);
    Vector3 phm;
    data->data.viewport.pose.mulInverse(ptLocal,phm);
    result = false;
    if(data->data.viewport.perspective) {
        if(IsFinite(h) && h > 0)
            result = (phm.z >= h);
    }
    else {
        if(IsFinite(h))
            result = (phm.z <= h);
    }
    return true;
}

bool LowerHeight(int i, int j, const Meshing::Heightmap& hm, Real hmax, const GeometricPrimitive3D& prim, Real& dist) 
{
    Ray3D ray;
    hm.GetVertexRay(i,j,ray.source,ray.direction);
    //Real dmax = hm.heights(i,j);
    if(hm.viewport.perspective)  {
        //ray needs to go from back to origin
        ray.source = hm.viewport.pose.t + ray.direction*(hmax + 1.0);
        ray.direction *= -1;
        Real dscale = ray.direction.norm();
        //dmax = dscale*(hmax  + 1 - dmax);  //limit is for unnormalized ray
        ray.direction /= dscale;
    }
    Vector3 pt;
    if(prim.RayCast(ray,pt)) {
        dist = ray.closestPointParameter(pt);
        if(hm.viewport.perspective) {
            dist = hm.Project(ray.source + dist*ray.direction).z;
            return true;
        }
        else {
            return true;
        }
    }
    return false;
}

bool LowerHeight(int i, int j, const Meshing::Heightmap& hm, Real hmax, const ConvexHull3D& hull, Real& dist) 
{
    Ray3D ray;
    hm.GetVertexRay(i,j,ray.source,ray.direction);
    Real dmax = hm.heights(i,j);
    if(hm.viewport.perspective)  {
        //ray needs to go from back to origin
        ray.source = hm.viewport.pose.t + ray.direction*(hmax + 1.0);
        ray.direction *= -1;
        Real dscale = ray.direction.norm();
        dmax = dscale*(hmax  + 1 - dmax);  //limit is for unnormalized ray
        ray.direction /= dscale;
    }
    if(hull.RayCast(ray,&dist,dmax)) {
        if(hm.viewport.perspective) {
            dist = hm.Project(ray.source + dist*ray.direction).z;
            return true;
        }
        else {
            return true;
        }
    }
    return false;
}

bool Collider3DHeightmap::Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions)
{
    RigidTransform Tlocal;
    Tlocal.mulInverseA(currentTransform,geom->GetTransform());
    switch(geom->GetType()) {
    case Type::PointCloud:
        {
        //check each point against the heightmap
        const Collider3DPointCloud* pc = dynamic_cast<const Collider3DPointCloud*>(geom);
        Box3D bb = pc->GetBB();
        Box3D bblocal;
        RigidTransform Tworld_local;
        Tworld_local.setInverse(currentTransform);
        bblocal.setTransformed(bb,Tworld_local);
        IntPair lo,hi;
        data->data.GetIndexRange(bblocal,lo,hi);
        if(lo.a > hi.a || lo.b > hi.b) { //out of range
            return true;
        }
        for(size_t i=0;i<pc->data->data.points.size();i++) {
            Vector3 pt;
            Tlocal.mul(pc->data->data.points[i],pt);
            Vector3 phm  = data->data.Project(pt);
            IntPair idx(int(Floor(phm.x)),int(Floor(phm.y)));
            if(idx.a < 0 || idx.b < 0 || idx.a >= data->data.heights.m || idx.b >= data->data.heights.n) continue;
            if(data->data.viewport.perspective) {
                Real hidx = data->data.heights(idx);
                if(hidx > 0 && hidx <= phm.z) {
                    elements2.push_back(i);
                    elements1.push_back(idx.a + idx.b*data->data.heights.m);
                    if(elements1.size() >= maxcollisions) return true;
                }
            }
            else {
                if(data->data.heights(idx) >= phm.z) {
                    elements2.push_back(i);
                    elements1.push_back(idx.a + idx.b*data->data.heights.m);
                    if(elements1.size() >= maxcollisions) return true;
                }
            }
        }
        return true;
        }
    case Type::TriangleMesh:
        {
        //check each triangle against the heightmap
        const Collider3DTriangleMesh* mesh = dynamic_cast<const Collider3DTriangleMesh*>(geom);
        Box3D bb = mesh->GetBB();
        Box3D bblocal;
        RigidTransform Tworld_local;
        Tworld_local.setInverse(currentTransform);
        bblocal.setTransformed(bb,Tworld_local);
        IntPair lo,hi;
        data->data.GetIndexRange(bblocal,lo,hi);
        if(lo.a > hi.a || lo.b > hi.b) { //out of range
            return true;
        }
        Math3D::Triangle3D tri;
        vector<IntPair> tcells;
        vector<Real> theights;
        int num_overlapping = 0;
        for (size_t i=0;i<mesh->data->data.tris.size();i++) {
            mesh->data->data.GetTriangle(i,tri);
            tri.a = data->data.Project(Tlocal*tri.a);
            tri.b = data->data.Project(Tlocal*tri.b);
            tri.c = data->data.Project(Tlocal*tri.c);
            tcells.resize(0);
            theights.resize(0);
            Meshing::GetTriangleHeights_Clipped(tri,tcells,theights,0,0,data->data.heights.m,data->data.heights.n);
            if(!tcells.empty()) num_overlapping ++;
            for(size_t k=0;k<tcells.size();k++) {
                float& cell = data->data.heights(tcells[k]);
                const auto& idx = tcells[k];
                if(data->data.viewport.perspective) {
                    if(cell <= theights[k]) {
                        elements2.push_back(i);
                        elements1.push_back(idx.a + idx.b*data->data.heights.m);
                        if(elements1.size() >= maxcollisions) return true;
                    }
                }
                else {
                    if(cell >= theights[k]) {
                        elements2.push_back(i);
                        elements1.push_back(idx.a + idx.b*data->data.heights.m);
                        if(elements1.size() >= maxcollisions) {
                            return true;
                        }
                    }
                }
            }
        }
        return true;
        }
    case Type::OccupancyGrid:
        {
        Collider3DOccupancyGrid* occ = dynamic_cast<Collider3DOccupancyGrid*>(geom);
        bool res=Geometry::Collides(*occ, *this, 0, elements2, elements1, maxcollisions);
        Assert(res == (!elements1.empty()));
        Assert(elements1.size()==elements2.size());
        return true;   
        }
    case Type::ImplicitSurface:
        {
        Collider3DImplicitSurface* surf = dynamic_cast<Collider3DImplicitSurface*>(geom);
        int nhm = data->data.heights.m*data->data.heights.n;
        int nsurf = surf->data->data.value.m*surf->data->data.value.n*surf->data->data.value.p;
        if(nhm > nsurf) { //heightmap is more fine-grained than implicit surface
            Meshing::VolumeGridIterator<Real> it=surf->data->data.getIterator();
            Vector3 c,clocal;
            Vector2 params;
            while(!it.isDone()) {
                if(*it <= 0) {
                    it.getCellCenter(c);
                    c = surf->currentTransform * c;
                    bool result;
                    Contains(c,result);
                    if(result) {
                        IntPair idx;
                        currentTransform.mulInverse(c,clocal);
                        data->data.GetIndexAndParams(clocal,idx,params,true);
                        elements1.push_back(idx.a + idx.b*data->data.heights.m);
                        elements2.push_back(surf->data->IndexToElement(it.getIndex()));
                        if(elements1.size() >= maxcollisions) return true;
                    }
                }
                ++it;
            }
            return true;
        }
        else {
            //implicit surface is more fine grained.  Rasterize heightmap into implicit surface
            //determine a bounding box to iterate over
            Box3D surfbb = surf->GetBB();
            Box3D surfbb_hm;
            RigidTransform Thm_inv; Thm_inv.setInverse(currentTransform);
            surfbb_hm.setTransformed(surfbb,Thm_inv);
            IntPair lo,hi;
            data->data.GetIndexRange(surfbb_hm,lo,hi);
            
            RigidTransform Tab;
            Tab.mulInverseA(surf->currentTransform,currentTransform);
            for(int i=lo.a;i<=hi.a;i++) {
                for(int j=lo.b;j<hi.b;j++) {
                    Vector3 c=data->data.GetVertex(i,j);
                    Vector3 clocal;
                    Tab.mul(c,clocal);
                    if(surf->DistanceLocal(clocal) <= 0) {
                        elements2.push_back(surf->data->PointToElement(clocal));
                        elements1.push_back(i+j*data->data.heights.m);
                        if(elements1.size() >= maxcollisions) return true;
                    }
                }
            }
            return true;
        }
        }
    case Type::ConvexHull:
        {
        Collider3DConvexHull* ch = dynamic_cast<Collider3DConvexHull*>(geom);
        ConvexHull3D hull_local = ch->data->data;
        hull_local.Transform(Tlocal);
        auto bb = hull_local.GetAABB();
        IntPair lo,hi;
        data->data.GetIndexRange(bb,lo,hi);
        if(lo.a > hi.a || lo.b > hi.b) return true;
        Real geomdiam = (bb.bmin.distance(bb.bmax));
        Real hgeom = data->data.Project((bb.bmax + bb.bmin)*0.5).z - geomdiam*0.5;
        if(data->data.viewport.perspective) {
            if(hgeom < hmin) return true;
        }
        else {
            if(hgeom > hmax) return true;
        }
        Vector3 s,d;
        Segment3D seg;
        if(maxcollisions <= 1) {
            //check center
            IntPair c = data->data.GetIndex((bb.bmax + bb.bmin)*0.5);
            c.a = Min(Max(c.a,0),data->data.heights.m-1);
            c.b = Min(Max(c.b,0),data->data.heights.n-1);
            if(data->data.ValidHeight(c.a,c.b)) {
                data->data.GetVertexRay(c.a,c.b,s,d); // result is in local coordinates
                seg.a = s + d * data->data.heights(c);
                if(data->data.viewport.perspective)
                    seg.b = s + d * (data->data.heights(c) + geomdiam);
                else
                    seg.b = s + d * (data->data.heights(c) - geomdiam);
                ConvexHull3D seghull; seghull.Set(seg);
                if(hull_local.Collides(seghull)) {
                    elements1.push_back(c.a + c.b*data->data.heights.m);
                    elements2.push_back(-1);
                    return true;
                }
            }
            // //check midpoint of range
            // int i=(lo.a+hi.a)/2;
            // int j=(lo.b+hi.b)/2;
            // if(data->data.ValidHeight(i,j) && data->data.heights(i,j) >= hgeom) { 
            //     data->data.GetVertexRay(i,j,s,d);  // result is in world coordinates
            //     seg.a = s + d * data->data.heights(i,j);
            //     if(data->data.viewport.perspective)
            //         seg.b = s + d * (data->data.heights(i,j) + geomdiam);
            //     else
            //         seg.b = s + d * (data->data.heights(i,j) - geomdiam);
            //     ConvexHull3D seghull; seghull.Set(seg);
            //     if(hull_local.Collides(seghull)) {
            //         elements1.push_back(i+j*data->data.heights.m);
            //         elements2.push_back(-1);
            //         return true;
            //     }
            // }
        }
        //check all rays
        // for(int i=lo.a;i<=hi.a;i++) {
        //     for(int j=lo.b;j<hi.b;j++) {
        {
            for(SpiralIterator it(data->data.GetIndex((bb.bmax + bb.bmin)*0.5),lo,hi); !it.isDone(); ++it) {
                int i=it->a;
                int j=it->b;
                if(data->data.ValidHeight(i,j) && data->data.heights(i,j) >= hgeom) { 
                    data->data.GetVertexRay(i,j,s,d);  // result is in world coordinates
                    seg.a = s + d * data->data.heights(i,j);
                    if(data->data.viewport.perspective)
                        seg.b = s + d * (data->data.heights(i,j) + geomdiam);
                    else
                        seg.b = s + d * (data->data.heights(i,j) - geomdiam);
                    ConvexHull3D seghull; seghull.Set(seg);
                    if(hull_local.Collides(seghull)) {
                        elements1.push_back(i+j*data->data.heights.m);
                        elements2.push_back(-1);
                        if(elements1.size() >= maxcollisions) return true;
                    }
                }
            }
        }
        return true;
        }
    case Type::Primitive:
        {
        Collider3DPrimitive* p = dynamic_cast<Collider3DPrimitive*>(geom);
        //GeometricPrimitive3D prim_world = p->data->data;
        //prim_world.Transform(p->GetTransform());
        GeometricPrimitive3D prim_local = p->data->data;
        prim_local.Transform(Tlocal);
        auto bb = prim_local.GetAABB();
        IntPair lo,hi;
        data->data.GetIndexRange(bb,lo,hi);
        if(lo.a > hi.a || lo.b > hi.b) {
            //printf("Primitive bbox out of range of grid\n");
            return true;
        }
        Real geomdiam = (bb.bmin.distance(bb.bmax));
        Real hgeom = data->data.Project((bb.bmax + bb.bmin)*0.5).z - geomdiam*0.5;
        //printf("Minimum height of geometry: %f\n",hgeom);
        if(data->data.viewport.perspective) {
            if(hgeom < hmin) return true;
        }
        else {
            if(hgeom > hmax) 
                return true;
        }
        Real dist;
        if(maxcollisions <= 1) {
            //check center
            IntPair c = data->data.GetIndex((bb.bmax + bb.bmin)*0.5);
            c.a = Min(Max(c.a,0),data->data.heights.m-1);
            c.b = Min(Max(c.b,0),data->data.heights.n-1);
            if(data->data.ValidHeight(c.a,c.b) && LowerHeight(c.a,c.b,data->data,this->hmax,prim_local,dist)) {
                if(data->data.viewport.perspective) {
                    if(float(dist) >= data->data.heights(c)) {
                        elements1.push_back(c.a + c.b*data->data.heights.m);
                        elements2.push_back(-1);
                        return true;
                    }
                }
                else {
                    if(float(dist) <= data->data.heights(c)) {
                        elements1.push_back(c.a + c.b*data->data.heights.m);
                        elements2.push_back(-1);
                        return true;
                    }
                }
            }
            //check midpoint of range
            // int i=(lo.a+hi.a)/2;
            // int j=(lo.b+hi.b)/2;
            // if(data->data.ValidHeight(i,j) && data->data.heights(i,j) >= hgeom) { 
            //     if(LowerHeight(i,j,data->data,this->hmax,prim_local,dist)) {
            //         if(data->data.viewport.perspective) {
            //             if(float(dist) >= data->data.heights(i,j)) {
            //                 elements1.push_back(i+j*data->data.heights.m);
            //                 elements2.push_back(-1);
            //                 return true;
            //             }
            //         }
            //         else {
            //             if(float(dist) <= data->data.heights(i,j)) {
            //                 elements1.push_back(i+j*data->data.heights.m);
            //                 elements2.push_back(-1);
            //                 return true;
            //             }
            //         }
            //     }
            // }
        }
        //check whole range
        // for(int i=lo.a;i<=hi.a;i++) {
        //     for(int j=lo.b;j<hi.b;j++) {
        {
            for(SpiralIterator it(data->data.GetIndex((bb.bmax + bb.bmin)*0.5),lo,hi); !it.isDone(); ++it) {
                int i=it->a;
                int j=it->b;
                if(!data->data.ValidHeight(i,j) || data->data.heights(i,j) < hgeom) continue; 
                if(LowerHeight(i,j,data->data,this->hmax,prim_local,dist)) {
                    if(data->data.viewport.perspective) {
                        if(float(dist) >= data->data.heights(i,j)) {
                            elements1.push_back(i+j*data->data.heights.m);
                            elements2.push_back(-1);
                            if(elements1.size() >= maxcollisions) return true;
                        }
                    }
                    else {
                        if(float(dist) <= data->data.heights(i,j)) {
                            elements1.push_back(i+j*data->data.heights.m);
                            elements2.push_back(-1);
                            if(elements1.size() >= maxcollisions) return true;
                        }
                    }
                }
            }
        }
        return true;
        }
    default:
        return false;
    }
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
        Real tmin=0,tmax=Inf;
        if(rimg.intersects(cell,tmin,tmax)) {
            Real zray1 = rimg.source.z + tmin*rimg.direction.z;
            Real zray2 = rimg.source.z + tmax*rimg.direction.z;
            if(data->data.viewport.perspective) {
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