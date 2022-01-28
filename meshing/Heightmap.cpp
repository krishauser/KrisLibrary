#include "Heightmap.h"
#include "Rasterize.h"
#include <KrisLibrary/GLdraw/GeometryAppearance.h>

using namespace Meshing;

Heightmap::Heightmap()
:xysize(1,1),offset(Zero),perspective(false)
{}

Heightmap::Heightmap(int xdivs,int ydivs,const Vector2& xydims)
:heights(xdivs,ydivs),xysize(xydims),offset(Zero),perspective(false)
{}

Heightmap::Heightmap(const Array2D<float>& _heights,const Vector2& xydims)
:heights(_heights),xysize(xydims),offset(Zero),perspective(false)
{}

Heightmap::Heightmap(const Array2D<double>& _heights,const Vector2& xydims)
:heights(_heights),xysize(xydims),offset(Zero),perspective(false)
{}

Heightmap::Heightmap(const Image& _heights,const Vector2& xydims,Real zscale,bool bottom_row_first)
:xysize(xydims),offset(Zero),perspective(false)
{
    SetImage(_heights,zscale,bottom_row_first);
}

Heightmap::Heightmap(const Image& _heights,const Image& colors,const Vector2& xydims,Real zscale,bool bottom_row_first)
:xysize(xydims),offset(Zero),perspective(false)
{
    SetImage(_heights,colors,zscale,bottom_row_first);
}

void Heightmap::GetCell(int i,int j,AABB2D& bb) const
{
    Real di = 1.0/(heights.m-1);
    Real dj = 1.0/(heights.n-1);
    Real u = Real(i)*di - 0.5;
    Real v = Real(j)*dj - 0.5;
    bb.bmin.x = u*xysize.x+offset.x;
    bb.bmin.y = v*xysize.y+offset.y;
    bb.bmax.x = bb.bmin.x + di*xysize.x;
    bb.bmax.y = bb.bmin.y + dj*xysize.y;
}

Vector2 Heightmap::GetCellCenter(int i,int j) const
{
    Real di = 1.0/(heights.m-1);
    Real dj = 1.0/(heights.n-1);
    Real u = Real(i+0.5)*di - 0.5;
    Real v = Real(j+0.5)*dj - 0.5;
    return Vector2(u*xysize.x+offset.x,v*xysize.y+offset.y);
}

Vector2 Heightmap::GetCellSize() const
{
    if(heights.m <= 1 || heights.n <= 1) return Vector2(0,0);
    return Vector2(xysize.x/(heights.m-1),xysize.y/(heights.n-1));
}

void Heightmap::GetGrid(vector<Real>& xgrid,vector<Real>& ygrid) const
{
    Vector2 cellSize = GetCellSize();
    xgrid.resize(heights.m);
    ygrid.resize(heights.n);
    Real xshift = -Real(heights.m-1)*0.5*xysize.x;
    Real yshift = -Real(heights.n-1)*0.5*xysize.y;
    xgrid[0] = xshift + offset.x;
    for(int i=1;i<heights.m;i++)
        xgrid[i] = xgrid[i-1] + cellSize.x;
    ygrid[1] = yshift + offset.y;
    for(int j=1;j<heights.n;j++)
        ygrid[j] = ygrid[j-1] + cellSize.y;
}

AABB3D Heightmap::GetAABB() const
{
    if(perspective) FatalError("TODO perspective projection");
    AABB3D res;
    Real xscale = Real(heights.m-1)*0.5*xysize.x;
    Real yscale = Real(heights.n-1)*0.5*xysize.y;
    res.bmin.x = -xscale + offset.x;
    res.bmax.x = -xscale + offset.x;
    res.bmin.y = -yscale + offset.y;
    res.bmax.y = -yscale + offset.y;
    if(heights.empty()) {
        res.bmin.z = Inf;
        res.bmax.z = -Inf;
    }
    else {
        res.bmin.z = *std::min_element(heights.begin(),heights.end()) + offset.z;
        res.bmax.z = *std::max_element(heights.begin(),heights.end()) + offset.z;
    }
    return res;
}

void Heightmap::SetViewport(const Camera::Viewport& vp)
{
    perspective = vp.perspective;
    offset = vp.xform.t;
    if(vp.x != 0 || vp.y != 0) {
        FatalError("TODO: non-centered viewports");
    }
    heights.resize(vp.w,vp.h);
    xysize.set(vp.scale,vp.scale);
}

void Heightmap::GetViewport(Camera::Viewport & vp) const
{
    vp.perspective = perspective;
    vp.xform.R.setIdentity();
    vp.xform.t = offset;
    if(xysize.x != xysize.y)
        printf("Warning: Heightmap has non-square pixels, can't map exactly onto viewport");
    vp.scale = (xysize.x+xysize.y)*0.5;
    vp.x = 0;
    vp.y = 0;
    vp.w = heights.m;
    vp.h = heights.n;
}

Vector3 Heightmap::ToNormalized(const Vector3& pt) const
{
    Vector3 res;
    res = pt - offset;
    res.x /= xysize.x;
    res.y /= xysize.y;
    if(perspective) {
        res.x /= res.z;
        res.y /= res.z;
    }
    return res;
}

Vector3 Heightmap::FromNormalized(const Vector3& params) const
{
    Vector3 res;
    res.z = params.z;
    res.x = params.x*xysize.x;
    res.y = params.y*xysize.y;
    if(perspective) {
        res.x *= res.z;
        res.y *= res.z;
    }
    return res + offset;
}

IntPair Heightmap::GetIndex(const Vector3& pt,bool clamp) const
{
    IntPair index;
    Vector2 params;
    GetIndexAndParams(pt,index,params,clamp);
    return index;
}

void Heightmap::GetIndexAndParams(const Vector3& pt,IntPair& index,Vector2& params,bool clamp) const
{
    if(perspective) FatalError("TODO perspective projection");
    params.x = (heights.m-1)*((pt.x-offset.x)/xysize.x + 0.5);
    params.y = (heights.n-1)*((pt.y-offset.y)/xysize.y + 0.5);
    index.a = int(Floor(params.x));
    index.b = int(Floor(params.y));
    params.x -= Real(index.a);
    params.y -= Real(index.b);
    if(clamp) {
        if(index.a < 0) {
            index.a = 0;
            params.x = 0;
        }
        else if(index.a >= heights.m-1) {
            index.a = heights.m-1;
            params.x = 0;
        }
        if(index.b < 0) {
            index.b = 0;
            params.y = 0;
        }
        else if(index.b >= heights.n-1) {
            index.b = heights.n-1;
            params.y = 0;
        }
    }
}

float Heightmap::GetHeight(const Vector3& pt,int interpolation) const
{
    IntPair index;
    Vector2 params;
    GetIndexAndParams(pt,index,params);
    if(interpolation == InterpNearest) {
        if(params.x > 0.5) index.a += 1;
        if(params.y > 0.5) index.b += 1;
        index.a = ::Min(::Max(0,index.a),heights.m-1);
        index.b = ::Min(::Max(0,index.b),heights.n-1);
        return heights(index.a,index.b);
    }
    FatalError("TODO: bilinear and bicubic interpolation");
}

Vector3 Heightmap::GetColor(const Vector3& pt,int interpolation) const
{
    if(colors.num_bytes==0) return Vector3(0,0,0);
    Assert(colors.w == heights.m);
    Assert(colors.h == heights.n);
    IntPair index;
    Vector2 params;
    GetIndexAndParams(pt,index,params);
    if(interpolation == InterpNearest) {
        if(params.x > 0.5) index.a += 1;
        if(params.y > 0.5) index.b += 1;
        index.a = ::Min(::Max(0,index.a),heights.m-1);
        index.b = ::Min(::Max(0,index.b),heights.n-1);
        float col[4];
        colors.getNormalizedColor(index.a,heights.n-1-index.b,col);
        if(colors.pixelChannels()==1) 
            return Vector3(col[0]);
        else
            return Vector3(col[0],col[1],col[2]);
    }
    FatalError("TODO: bilinear and bicubic interpolation");
}


Vector3 Heightmap::GetVertex(int i,int j) const
{ 
    Vector3 res;
    Real di = 1.0/(heights.m-1);
    Real dj = 1.0/(heights.n-1);
    Real u = Real(i)*di - 0.5;
    Real v = Real(j)*dj - 0.5;
    res.x = u*xysize.x;
    res.y = v*xysize.y;
    if(heights.empty()) {
        res.z = 0;
        return res + offset;
    }
    i = ::Min(::Max(i,0),heights.m-1);
    j = ::Min(::Max(j,0),heights.n-1);
    res.z = heights(i,j);
    if(perspective) {
        res.x *= res.z;
        res.y *= res.z;
    }
    return res + offset;
}

Vector3 Heightmap::GetVertex(int i,int j,Real vu,Real vv,int interpolation) const
{
    Vector3 res;
    Real di = 1.0/(heights.m-1);
    Real dj = 1.0/(heights.n-1);
    Real u = Real(i+vu)*di - 0.5;
    Real v = Real(j+vv)*dj - 0.5;
    res.x = u*xysize.x;
    res.y = v*xysize.y;
    if(heights.empty()) {
        res.z = 0;
        return res + offset;
    }
    if(interpolation == InterpNearest) {
        if(u>0.5) i++;
        if(v>0.5) j++;
        i = ::Min(::Max(i,0),heights.m-1);
        j = ::Min(::Max(j,0),heights.n-1);
        res.z = heights(i,j);
    }
    else {
        FatalError("TODO: bilinear and bicubic interpolation");
    }
    
    if(perspective) {
        res.x *= res.z;
        res.y *= res.z;
    }
    return res + offset;
}

void Heightmap::GetVertices(vector<Vector3>& verts) const
{
    verts.resize(heights.m*heights.n);

    Real di = 1.0/(heights.m-1);
    Real dj = 1.0/(heights.n-1);
    Real fx = di*xysize.x;
    Real fy = dj*xysize.y;
    Vector3 res;
    Real u = -0.5*xysize.x;
    int k=0;
    for(int i=0;i<heights.m;i++,u+=fx) {
        Real v = -0.5*xysize.y;
        for(int j=0;j<heights.n;j++,k++,v+=fy) {
            auto& res = verts[k];
            res.x = u;
            res.y = v;
            res.z = heights(i,j);
            if(perspective) {
                res.x *= res.z;
                res.y *= res.z;
            }
        }
    }
}

void Heightmap::GetVertices(Array2D<Vector3>& verts) const
{
    verts.resize(heights.m,heights.n);
    Real di = 1.0/(heights.m-1);
    Real dj = 1.0/(heights.n-1);
    Real fx = di*xysize.x;
    Real fy = dj*xysize.y;
    Vector3 res;
    Real u = -0.5*xysize.x;
    for(int i=0;i<heights.m;i++,u+=fx) {
        Real v = -0.5*xysize.y;
        for(int j=0;j<heights.n;j++,v+=fy) {
            res.x = u;
            res.y = v;
            res.z = heights(i,j);
            if(perspective) {
                res.x *= res.z;
                res.y *= res.z;
            }
            verts(i,j) = res + offset;
        }
    }
}
   
void Heightmap::GetImage(Image& out,float hmin,float hmax,bool bottom_row_first)
{      
    if(IsInf(hmin)) {
        float hmin_ = *std::min_element(heights.begin(),heights.end());
        if(hmin_ > 0)
            hmin = 0;
        else
            hmin = hmin_;
    }
    if(IsInf(hmax)) {
        hmax = *std::max_element(heights.begin(),heights.end());
    }
    if(hmax == hmin)
        hmax = hmin+1;
    out.initialize(heights.n,heights.m,Image::FloatA);
    float* fdata = (float*)out.data;
    float hscale = 1.0/(hmax-hmin);
    if(bottom_row_first) {
        for(int i=0;i<heights.n;i++) {
            for(int j=0;j<heights.m;j++) {
                *fdata = (heights(j,i)-hmin)*hscale;
                fdata += 1;
            }
        }
    }
    else {
        for(int i=heights.n-1;i>=0;i--) {
            for(int j=0;j<heights.m;j++) {
                *fdata = (heights(j,i)-hmin)*hscale;
                fdata += 1;
            }
        }
    }
}


void Heightmap::Shift(float value)
{
    for(auto& i:heights) i+=value;
}

void Heightmap::Scale(float value)
{
    for(auto& i:heights) i*=value;
}

void Heightmap::Min(float value)
{
    for(auto& i:heights) i = ::Min(i,value);
}

void Heightmap::Max(float value)
{
    for(auto& i:heights) i = ::Max(i,value);
}

void Heightmap::Shift(const Array2D<float>& values)
{
    Assert(values.m == heights.m && values.n == heights.n);
    auto b = values.begin();
    for(auto a = heights.begin();a != heights.end();++a,++b) {
        *a += *b;
    }
}

void Heightmap::Scale(const Array2D<float>& values)
{
    Assert(values.m == heights.m && values.n == heights.n);
    auto b = values.begin();
    for(auto a = heights.begin();a != heights.end();++a,++b) {
        *a *= *b;
    }
}

void Heightmap::Min(const Array2D<float>& values)
{
    Assert(values.m == heights.m && values.n == heights.n);
    auto b = values.begin();
    for(auto a = heights.begin();a != heights.end();++a,++b) {
        *a = ::Min(*a,*b);
    }
}

void Heightmap::Max(const Array2D<float>& values)
{
    Assert(values.m == heights.m && values.n == heights.n);
    auto b = values.begin();
    for(auto a = heights.begin();a != heights.end();++a,++b) {
        *a = ::Max(*a,*b);
    }
}

void Heightmap::Remesh(const Heightmap& hm)
{
    if(hm.perspective) FatalError("TODO: perspective remeshing");
    Array2D<Vector3> verts;
    hm.GetVertices(verts);
    Array2D<float> newheights(verts.m,verts.n);
    Image newcolors;
    if(colors.num_bytes != 0)
        newcolors.initialize(verts.m,verts.n,colors.format);
    for(int i=0;i<verts.m;i++)
        for(int j=0;j<verts.n;j++) {
            newheights(i,j) = GetHeight(verts(i,j));
            if(colors.num_bytes != 0) {
                Vector3 c = GetColor(verts(i,j));
                float col[4] = {(float)c.x,(float)c.y,(float)c.z,1.0f};
                newcolors.setNormalizedColor(i,verts.n-1-j,col);
            }
        }
    offset = hm.offset;
    xysize = hm.xysize;
    heights = newheights;
}

void Heightmap::Remesh(const Camera::Viewport& vp)
{
    Heightmap temp;
    temp.SetViewport(vp);
    Remesh(temp);
}

void Heightmap::SetMesh(const TriMesh& mesh,Real resolution,bool topdown)
{
    Vector3 bmin,bmax;
    mesh.GetAABB(bmin,bmax);
    Vector3 center = (bmin+bmax)*0.5;
    Vector3 dims = bmax-bmin;
    Vector3 dims_padded = dims + Vector3(2*resolution);
    IntPair size((int)Ceil(dims_padded.x/resolution),(int)Ceil(dims_padded.y/resolution));

    heights.resize(size.a,size.b);
    heights.set(0.0);
    colors.clear();
    xysize.set(dims_padded.x,dims_padded.y);
    offset = center;
    if(topdown)
        offset.z = bmin.z;
    else
        offset.z = bmax.z;
    FuseMesh(mesh,topdown,OpReplace);
}

void Heightmap::FuseMesh(const TriMesh& mesh,bool topdown,int op)
{
    if(perspective) FatalError("TODO: render perspective");
    if(op != OpReplace) FatalError("TODO: raster operations except for OpReplace");
    SmoothFillRasterizer2D<float> rasterizer;
    rasterizer.grid = &heights;
    Triangle2D tri;
    vector<IntPair> cells;
    Vector3 lower = offset;
    Real fx = (heights.m-1)/xysize.x;
    Real fy = (heights.n-1)/xysize.y;
    lower.x -= xysize.x*0.5;
    lower.y -= xysize.y*0.5;
    for(size_t i=0;i<mesh.tris.size();i++) {
        tri.a.set(mesh.verts[mesh.tris[i].a]-lower);
        tri.b.set(mesh.verts[mesh.tris[i].b]-lower);
        tri.c.set(mesh.verts[mesh.tris[i].c]-lower);
        tri.a.x *= fx;
        tri.a.y *= fy;
        tri.b.x *= fx;
        tri.b.y *= fy;
        tri.c.x *= fx;
        tri.c.y *= fy;
        float ha = mesh.verts[mesh.tris[i].a].z-offset.z;
        float hb = mesh.verts[mesh.tris[i].b].z-offset.z;
        float hc = mesh.verts[mesh.tris[i].c].z-offset.z;
        rasterizer.Rasterize(tri,ha,hb,hc);
    }
}

void Heightmap::GetMesh(TriMesh& mesh) const
{
    if(heights.m < 2 || heights.n < 2) return;
    GetVertices(mesh.verts);
    mesh.tris.resize(2*(heights.m-1)*(heights.n-1));
    int k=0;
    for(int i=0;i<heights.m-1;i++) {
        int k1=i*heights.n;
        int k2=(i+1)*heights.n;
        for(int j=0;i<heights.n-1;j++) {
            mesh.tris[k].set(k1+j,k2+j,k1+j+1);
            k++;
            mesh.tris[k].set(k1+j+1,k2+j,k2+j+1);
            k++;
        }
    }
}

void Heightmap::GetMesh(TriMesh& mesh,GLDraw::GeometryAppearance& app) const
{
    GetMesh(mesh);
    if(colors.num_bytes != 0) {
        app.vertexColors.resize(heights.m*heights.n);
        int k=0;
        for(int i=0;i<heights.m;i++) {
            for(int j=0;i<heights.n;j++,k++) {
                
            }
        }
    }
}

void Heightmap::SetPointCloud(const PointCloud3D& pc,Real resolution,bool topdown)
{
    Vector3 bmin,bmax;
    pc.GetAABB(bmin,bmax);
    Vector3 center = (bmin+bmax)*0.5;
    Vector3 dims = bmax-bmin;
    Vector3 dims_padded = dims + Vector3(2*resolution);
    IntPair size((int)Ceil(dims_padded.x/resolution),(int)Ceil(dims_padded.y/resolution));

    heights.resize(size.a,size.b);
    heights.set(0.0);
    if(pc.HasColor())
        colors.initialize(size.b,size.a,Image::R8G8B8);
    else
        colors.clear();
    xysize.set(dims_padded.x,dims_padded.y);
    offset = center;
    if(topdown)
        offset.z = bmin.z;
    else
        offset.z = bmax.z;
    FusePointCloud(pc,topdown,OpReplace);
}

void Heightmap::FusePointCloud(const PointCloud3D& pc,bool topdown,int op)
{
    if(op == OpReplace) heights.set(0.0);
    bool useColor = (pc.HasColor() && colors.num_bytes != 0);
    vector<Vector4> pointColors;
    if(useColor) pc.GetColors(pointColors);
    for(size_t i=0;i<pc.points.size();i++) {
        IntPair ind = GetIndex(pc.points[i]);
        if(ind.a < 0 || ind.a >= heights.m) continue;
        if(ind.b < 0 || ind.b >= heights.n) continue;
        float z = pc.points[i].z-offset.z;
        if((op == OpReplace && topdown) || op == OpMax)
            heights(ind.a,ind.b) = ::Max(z,heights(ind.a,ind.b));
        else
            heights(ind.a,ind.b) = ::Min(z,heights(ind.a,ind.b));
        if(useColor && heights(ind.a,ind.b) == z) {
            float col[4];
            for(int k=0;k<4;k++)
                col[k] = pointColors[i][k];
            colors.setNormalizedColor(ind.a,heights.n-1-ind.b,col);
        }
    }
}

void Heightmap::GetPointCloud(PointCloud3D& pc) const
{
    GetVertices(pc.points);
    if(colors.num_bytes != 0) {
        vector<Vector4> rgba(pc.points.size());
        int k=0;
        for(int i=0;i<heights.m;i++) {
            for(int j=0;j<heights.n;j++,k++) {
                float col[4];
                colors.getNormalizedColor(i,heights.n-1-j,col);
                rgba[k].set(col[0],col[1],col[2],col[3]);
            }
        }
        pc.SetColors(rgba,(colors.pixelChannels()==4));
    }
}
