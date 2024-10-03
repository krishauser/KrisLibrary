#include "Heightmap.h"
#include "Rasterize.h"
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/image/io.h>
#include <memory.h>
#include <fstream>

using namespace Meshing;




Heightmap::Heightmap()
:viewport(1,1)
{
    viewport.perspective = false;
    viewport.setScale(2.0);
}

Heightmap::Heightmap(int xdivs,int ydivs,const Vector2& xydims)
:heights(xdivs,ydivs),viewport(xdivs,ydivs)
{
    viewport.perspective = false;
    viewport.setScale(2.0);
}

Heightmap::Heightmap(const Array2D<float>& _heights,const Vector2& xydims)
:heights(_heights),viewport(_heights.m,heights.n)
{
    SetSize(xydims.x,xydims.y);
}

Heightmap::Heightmap(const Array2D<double>& _heights,const Vector2& xydims)
:heights(_heights),viewport(_heights.m,heights.n)
{
    SetSize(xydims.x,xydims.y);
}

Heightmap::Heightmap(const Image& _heights,const Vector2& xydims,float hscale,float hoffset,bool bottom_row_first)
:viewport(_heights.w,_heights.h)
{
    SetSize(xydims.x,xydims.y);
    SetImage(_heights,hscale,hoffset,bottom_row_first);
}

Heightmap::Heightmap(const Image& _heights,const Image& colors,const Vector2& xydims,float hscale,float hoffset,bool bottom_row_first)
:viewport(_heights.w,_heights.h)
{
    SetSize(xydims.x,xydims.y);
    SetImage(_heights,colors,hscale,hoffset,bottom_row_first);
}

bool Heightmap::Load(const char* fn)
{
    ifstream in(fn,ios::in);
    if(!in) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Error opening file "<<fn<<" for reading");
        return false;
    }
    //may need to load images from the same folder as fn
    string path = GetFilePath(string(fn));
    const char* folder = NULL;
    if(!path.empty()) {
        folder = path.c_str();
    }
    if(!Load(in,folder)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Error reading heightmap file from "<<fn);
        return false;
    }
    return true;
}

bool Heightmap::Save(const char* fn) const
{
    ofstream out(fn,ios::out);
    if(!out) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Save: Error opening file "<<fn<<" for writing");
        return false;
    }
    string hfn = fn;
    string cfn = fn;
    auto pos = hfn.rfind('.');
    #if HAVE_FREE_IMAGE || defined(_WIN32)
    cfn = hfn.substr(0,pos) + "_color.png";
    hfn = hfn.substr(0,pos) + "_height.png";
    #else
    cfn = hfn.substr(0,pos) + "_color.ppm";
    hfn = hfn.substr(0,pos) + "_height.ppm";
    #endif //HAVE_FREE_IMAGE || defined(_WIN32)
    return Save(out,hfn.c_str(),cfn.c_str());
}

bool Heightmap::Load(std::istream& in, const char* folder)
{
    AnyCollection items;
    if(!items.read(in)) {
        return false;
    }
    if(!items.find("type")) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: No type field in heightmap file");
        return false;
    }
    string type;
    if(!items["type"].as(type)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Error reading type field in heightmap file");
        return false;
    }
    if(type != "Heightmap") {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Not a heightmap file, type is "<<type);
        return false;
    }
    if(items.find("xsize")) {
        Vector2 xysize;
        if(!items["xsize"].as(xysize.x)) return false;
        if(!items["ysize"].as(xysize.y)) return false;
        SetSize(xysize.x,xysize.y);
    }
    else {
        Vector2 xyfov;
        if(!items["xfov"].as(xyfov.x)) return false;
        if(!items["yfov"].as(xyfov.y)) return false;
        SetFOV(xyfov.x,xyfov.y);
    }
    if(items.find("offset")) {
        if(!items["offset"][0].as(viewport.pose.t.x)) return false;
        if(!items["offset"][1].as(viewport.pose.t.y)) return false;
        if(!items["offset"][2].as(viewport.pose.t.z)) return false;
    }
    else {
        viewport.pose.t.setZero();
    }
    if(items.find("orientation")) {
        auto o1 = items["orientation"][0];
        auto o2 = items["orientation"][1];
        auto o3 = items["orientation"][2];
        if(!o1[0].as(viewport.pose.R(0,0))) return false;
        if(!o1[1].as(viewport.pose.R(0,1))) return false;
        if(!o1[2].as(viewport.pose.R(0,2))) return false;
        if(!o2[0].as(viewport.pose.R(1,0))) return false;
        if(!o2[1].as(viewport.pose.R(1,1))) return false;
        if(!o2[2].as(viewport.pose.R(1,2))) return false;
        if(!o3[0].as(viewport.pose.R(2,0))) return false;
        if(!o3[1].as(viewport.pose.R(2,1))) return false;
        if(!o3[2].as(viewport.pose.R(2,2))) return false;
    }
    else {
        viewport.pose.R.setIdentity();
    }
    float hmin,hmax;
    if(!items["height_range"].isarray()) return false;
    if(!items["height_range"][0].as(hmin)) return false;
    if(!items["height_range"][1].as(hmax)) return false;

    string heightFn;
    if(!items["heights"].as(heightFn)) return false;
    Image himg;
    if(folder) {
        heightFn = JoinPath(folder,heightFn);
    }
    if(!ImportImage(heightFn.c_str(),himg)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Error importing height image from "<<heightFn);
        return false;
    }
    //unsigned char imgmin = *std::min_element(himg.data,himg.data+himg.w*himg.h);
    //unsigned char imgmax = *std::max_element(himg.data,himg.data+himg.w*himg.h);
    SetImage(himg,hmax-hmin,hmin);
    if(items.find("colors")) {
        string colorFn;
        if(!items["colors"].as(colorFn)) return false;
        if(folder) {
            colorFn = JoinPath(folder,colorFn);
        }
        Image cimg;
        if(!ImportImage(colorFn.c_str(),cimg)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Error importing color image from "<<colorFn);
            return false;
        }
        if(cimg.w != himg.w || cimg.h != himg.h) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Load: Color image has different dimensions from height image");
            return false;
        }
        colors = cimg;
        //colors.initialize(cimg.w,cimg.h,Image::R8G8B8);
        //cimg.blit(colors);
    }
    return true;
}

bool Heightmap::Save(std::ostream& out, const char* heightFn, const char* colorFn) const
{
    AnyCollection items;
    items["type"] = "Heightmap";
    if(!viewport.perspective) {
        AABB2D bb = viewport.getViewRectangle(0,true);
        items["xsize"] = bb.bmax.x-bb.bmin.x;
        items["ysize"] = bb.bmax.y-bb.bmin.y;
    }
    else {
        float xfov = viewport.getFOV();
        float yfov = viewport.getVerticalFOV();
        items["xfov"] = xfov;
        items["yfov"] = yfov;
    }
    if(viewport.pose.t.maxAbsElement() > 0) {
        AnyCollection joffset;
        joffset.resize(3);
        joffset[0] = viewport.pose.t.x;
        joffset[1] = viewport.pose.t.y;
        joffset[2] = viewport.pose.t.z;
        items["offset"] = joffset;
    }
    //TODO: orientation
    Image himg;
    float hmin = *std::min_element(heights.begin(),heights.end());
    float hmax = *std::max_element(heights.begin(),heights.end());
    items["height_range"].resize(2);
    items["height_range"][0] = hmin;
    items["height_range"][1] = hmax;

    GetImage(himg,hmin,hmax);
    //this is a float image.  Only certain image formats support floating point images.
    if(0==strcmp(FileExtension(heightFn),"tif") || 0==strcmp(FileExtension(heightFn),"tiff")) {
        //TIF/TIFF can output 32-bit float  
    }
    else if(0==strcmp(FileExtension(heightFn),"png")) {
        //TODO: Output 16-bit grayscale
        Image h8;
        h8.initialize(himg.w,himg.h,Image::A8);
        himg.blit(h8);
        if(!ExportImage(heightFn,h8)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Save: Error exporting height image to "<<heightFn);
            return false;
        }
    }
    else {
        Image h8;
        h8.initialize(himg.w,himg.h,Image::A8);
        himg.blit(h8);
        if(!ExportImage(heightFn,h8)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Save: Error exporting height image to "<<heightFn);
            return false;
        }
    }
    items["heights"] = GetFileName(heightFn);  //assumes these are the same directory as the heightmap json file
    if(HasColors()) {
        if(colorFn == NULL) {
            FatalError("Heightmap::Save: must provide a color filename to save a color image");
        }
        if(!ExportImage(colorFn,colors)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Heightmap::Save: Error exporting color image to "<<colorFn);
            return false;
        }
        items["colors"] = GetFileName(colorFn);
    }
    out<<items<<endl;
    return true;
}

void Heightmap::SetImage(const Image& _heights,float hscale,float hoffset,bool bottom_row_first)
{
    if(!(_heights.format == Image::FloatA || _heights.format == Image::A8)) {
        FatalError("Heightmap::SetImage: heights must be a single channel float image or an 8-bit grayscale image");
    }
    heights.resize(_heights.w,_heights.h);
    viewport.w = _heights.w;
    viewport.h = _heights.h;
    viewport.cx = _heights.w*0.5;
    viewport.cy = _heights.h*0.5;
    float h;
    const static float bytescale = 1.0/255.0;
    for(int j=0;j<_heights.h;j++) {
        for(int i=0;i<_heights.w;i++) {
            unsigned char* pix = _heights.getData(i,j);
            if(_heights.format == Image::A8) {
                h = float(*pix)*bytescale;
            }
            else {
                float* fpix = (float*)pix;
                h = *fpix;
            }
            h = h*hscale + hoffset;
            if(bottom_row_first)
                heights(i,j) = h;
            else
                heights(i,_heights.h - 1 - j) = h;
        }
    }
}

void Heightmap::SetImage(const Image& _heights,const Image& _colors,float hscale,float hoffset,bool bottom_row_first)
{
    if(_heights.w != _colors.w || _heights.h != _colors.h) {
        FatalError("Heightmap::SetImage: heights and colors must have the same dimensions");
    }
    SetImage(_heights,hscale,hoffset,bottom_row_first);
    colors.initialize(_colors.w,_colors.h,Image::R8G8B8);
    _colors.blit(colors);

    if(bottom_row_first) {
        //flip color image -- color image is expected to go from top down
        unsigned int rowbytes = colors.pitch();
        unsigned char* tmp = new unsigned char[rowbytes];
        for(int j=0;j<colors.h/2;j++) {
            unsigned char* d1 = colors.getData(0,j);
            unsigned char* d2 = colors.getData(0,colors.h-1-j);
            memcpy(tmp,d1,rowbytes);
            memcpy(d1,d2,rowbytes);
            memcpy(d2,tmp,rowbytes);
        }
        delete [] tmp;
    }
}

   
void Heightmap::GetImage(Image& out,float hmin,float hmax,bool bottom_row_first) const
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
    out.initialize(heights.m,heights.n,Image::FloatA);
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

void Heightmap::SetSize(Real width, Real height)
{
    if(height == 0) {
        height = width/(viewport.w-1)*(viewport.h-1);
    }
    viewport.perspective = false;
    viewport.fx = (viewport.w-1)/width;  //point 0.5*width maps to w / 2 when multiplied by fx
    viewport.fy = (viewport.h-1)/height;
    viewport.cx = 0.5 * viewport.w;
    viewport.cy = 0.5 * viewport.h;
}

void Heightmap::SetFOV(Real xfov,Real yfov)
{
    Real xsize_2 = Tan(xfov*0.5);
    Real ysize_2 = Tan(yfov*0.5);
    if(yfov==0) {
        ysize_2 = xsize_2/(viewport.w-1)*(viewport.h-1);
    }
    viewport.perspective = true;
    viewport.fx = 0.5*(viewport.w-1)/xsize_2;
    viewport.fy = 0.5*(viewport.h-1)/ysize_2;
    viewport.cx = 0.5 * viewport.w;
    viewport.cy = 0.5 * viewport.h;
}

Vector2 Heightmap::GetSize() const 
{
    if(viewport.perspective) {
        float xscale = 2.0*viewport.fx/(viewport.w-1);
        float yscale = 2.0*viewport.fy/(viewport.h-1);
        Real xfov = (Atan(1.0/xscale)*2);
        Real yfov = (Atan(1.0/yscale)*2);
        return Vector2(xfov,yfov);
    }
    else {
        AABB2D bb = viewport.getViewRectangle(0,true);    
        return bb.bmax-bb.bmin;
    }
}

void Heightmap::GetCell(int i,int j,AABB2D& bb) const
{
    Real invfx = 1.0/viewport.fx;
    Real invfy = 1.0/viewport.fy;
    Real u = (Real(i)-viewport.cx)*invfx;
    Real v = (Real(j)-viewport.cy)*invfy;
    bb.bmin.x = u;
    bb.bmin.y = v;
    bb.bmax.x = bb.bmin.x + invfx;
    bb.bmax.y = bb.bmin.y + invfx;
}

Vector2 Heightmap::GetCellCenter(int i,int j) const
{
    Real invfx = 1.0/viewport.fx;
    Real invfy = 1.0/viewport.fy;
    Real u = (Real(i)+0.5-viewport.cx)*invfx;
    Real v = (Real(j)+0.5-viewport.cy)*invfy;
    return Vector2(u,v);
}

Vector2 Heightmap::GetCellSize() const
{
    if(heights.m <= 1 || heights.n <= 1) return Vector2(0,0);
    return Vector2(1.0/viewport.fx,1.0/viewport.fy);
}

void Heightmap::GetGrid(vector<Real>& xgrid,vector<Real>& ygrid) const
{
    Vector2 cellSize = GetCellSize();
    xgrid.resize(heights.m);
    ygrid.resize(heights.n);
    Real xshift = (0.5-viewport.cx)*cellSize.x;
    Real yshift = (0.5-viewport.cy)*cellSize.y;
    xgrid[0] = xshift;
    for(int i=1;i<heights.m;i++)
        xgrid[i] = xgrid[i-1] + cellSize.x;
    ygrid[1] = yshift;
    for(int j=1;j<heights.n;j++)
        ygrid[j] = ygrid[j-1] + cellSize.y;
}

AABB3D Heightmap::GetAABB() const
{
    AABB3D res;
    AABB2D bb2d;
    if(heights.empty()) {
        res.bmin.z = Inf;
        res.bmax.z = -Inf;
    }
    else {
        res.bmin.z = *std::min_element(heights.begin(),heights.end(),[] (float x, float y) { return x < y ? true : !isnan(x); });
        res.bmax.z = *std::max_element(heights.begin(),heights.end(),[] (float x, float y) { return x < y ? true : isnan(x); });
    }
    if(viewport.perspective) {
        bb2d = viewport.getViewRectangle(res.bmin.z,true);
        vector<Vector3> corners(8);
        corners[0].set(bb2d.bmin.x,bb2d.bmin.y,res.bmin.z);
        corners[1].set(bb2d.bmax.x,bb2d.bmin.y,res.bmin.z);
        corners[2].set(bb2d.bmax.x,bb2d.bmax.y,res.bmin.z);
        corners[3].set(bb2d.bmin.x,bb2d.bmax.y,res.bmin.z);
        bb2d = viewport.getViewRectangle(res.bmax.z,true);
        corners[0].set(bb2d.bmin.x,bb2d.bmin.y,res.bmax.z);
        corners[1].set(bb2d.bmax.x,bb2d.bmin.y,res.bmax.z);
        corners[2].set(bb2d.bmax.x,bb2d.bmax.y,res.bmax.z);
        corners[3].set(bb2d.bmin.x,bb2d.bmax.y,res.bmax.z);
        res.minimize();
        for(size_t i=0;i<8;i++)
            res.expand(viewport.pose*corners[i]);
        return res;
    }
    else {
        bb2d = viewport.getViewRectangle(true);
        res.bmin.x = bb2d.bmin.x;
        res.bmax.x = bb2d.bmax.x;
        res.bmin.y = bb2d.bmin.y;
        res.bmax.y = bb2d.bmax.y;

        AABB3D bb;
        bb.setTransform(res,Matrix4(viewport.pose));
        return bb;
    }
}

Vector3 Heightmap::Project(const Vector3& pt) const
{
    float mx,my,mz;
    viewport.project(pt,mx,my,mz);
    return Vector3(mx,my,mz);
}

Vector3 Heightmap::Deproject(const Vector3& params) const
{
    Vector3 src,dir;
    viewport.deproject(params.x,params.y,src,dir);
    return src + dir*params.z;
}

IntPair Heightmap::GetIndex(const Vector3& pt,bool clamp) const
{
    Vector3 proj = Project(pt);
    IntPair index;
    index.a = int(Floor(proj.x));
    index.b = int(Floor(proj.y));
    if(clamp) {
        index.a = (index.a < 0 ? 0 : (index.a > heights.m-1 ? heights.m-1 : index.a));
        index.b = (index.b < 0 ? 0 : (index.b > heights.n-1 ? heights.n-1 : index.b));
    }
    return index;
}

void Heightmap::GetIndexAndParams(const Vector3& pt,IntPair& index,Vector2& params,bool clamp) const
{
    Vector3 proj = Project(pt);
    index.a = int(Floor(proj.x));
    index.b = int(Floor(proj.y));
    params.x = proj.x-Real(index.a);
    params.y = proj.y-Real(index.b);
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

bool Heightmap::HasHeight(const Vector3& pt) const
{
    IntPair index;
    Vector2 params;
    GetIndexAndParams(pt,index,params);
    if(params.x > 0.5) index.a += 1;
    if(params.y > 0.5) index.b += 1;
    index.a = ::Min(::Max(0,index.a),heights.m-1);
    index.b = ::Min(::Max(0,index.b),heights.n-1);
    Real v = heights(index.a,index.b);
    if(viewport.perspective)
        return v != 0 && IsFinite(v);
    return IsFinite(v);
}

Real Heightmap::GetHeightDifference(const Vector3& pt,int interpolation) const
{
    Vector3 ptlocal = Project(pt);
    Real xf = Floor(ptlocal.x);
    Real yf = Floor(ptlocal.y);
    IntPair index((int)xf,(int)yf);
    Vector2 params(ptlocal.x-xf,ptlocal.y-yf);
    float v;
    if(interpolation == InterpNearest) {
        if(params.x > 0.5) index.a += 1;
        if(params.y > 0.5) index.b += 1;
        index.a = ::Min(::Max(0,index.a),heights.m-1);
        index.b = ::Min(::Max(0,index.b),heights.n-1);
        v=heights(index.a,index.b);
    }
    else
        FatalError("TODO: bilinear and bicubic interpolation");
    if(viewport.perspective) {
        if(v==0 || !IsFinite(v)) return NAN;
    }
    else {
        if(!IsFinite(v)) return NAN;
    }
    return ptlocal.z - v;
}


Vector3 Heightmap::GetColor(const Vector3& pt,int interpolation) const
{
    if(colors.num_bytes==0) return Vector3(0,0,0);
    Assert(colors.w == heights.m);
    Assert(colors.h == heights.n);
    IntPair index;
    Vector2 params;
    GetIndexAndParams(pt,index,params);
    return GetVertexColor(index.a,index.b,params.x,params.y,interpolation);
}


Vector3 Heightmap::GetVertex(int i,int j) const
{ 
    Vector2 uv = GetCellCenter(i,j);
    Vector3 res;
    res.x = uv.x;
    res.y = uv.y;
    if(heights.empty()) {
        res.z = 0;
        return viewport.pose*res;
    }
    i = ::Min(::Max(i,0),heights.m-1);
    j = ::Min(::Max(j,0),heights.n-1);
    res.z = heights(i,j);
    if(viewport.perspective) {
        res.x *= res.z;
        res.y *= res.z;
    }
    return viewport.pose*res;
}

Vector3 Heightmap::GetVertex(int i,int j,Real vu,Real vv,int interpolation) const
{
    Real invfx = 1.0/viewport.fx;
    Real invfy = 1.0/viewport.fy;
    Real u = (Real(i)+0.5+vu-viewport.cx)*invfx;
    Real v = (Real(j)+0.5+vv-viewport.cy)*invfy;
    Vector3 res;
    res.x = u;
    res.y = v;
    if(heights.empty()) {
        res.z = 0;
        return viewport.pose*res;
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
    
    if(viewport.perspective) {
        res.x *= res.z;
        res.y *= res.z;
    }
    return viewport.pose*res;
}

Vector3 Heightmap::GetVertexColor(int i,int j) const
{
    i = ::Min(::Max(0,i),heights.m-1);
    j = ::Min(::Max(0,j),heights.n-1);
    float col[4];
    colors.getNormalizedColor(i,heights.n-1-j,col);
    if(colors.pixelChannels()==1) 
        return Vector3(col[0]);
    else
        return Vector3(col[0],col[1],col[2]);
}

Vector3 Heightmap::GetVertexColor(int i,int j,Real u,Real v,int interpolation) const
{
    if(interpolation == InterpNearest) {
        if(u > 0.5) i += 1;
        if(v > 0.5) j += 1;
        return GetVertexColor(i,j);
    }
    FatalError("TODO: bilinear and bicubic interpolation");
}

void Heightmap::GetVertices(vector<Vector3>& verts) const
{
    verts.resize(heights.m*heights.n);
    vector<Vector3> src,dir;
    viewport.getAllRays(src,dir,true,false);
    int k=0;
    for(int i=0;i<heights.m;i++) {
        for(int j=0;j<heights.n;j++,k++) {
            verts[k] = src[k] + heights(i,j)*dir[k];
        }
    }
}

void Heightmap::GetVertices(Array2D<Vector3>& verts) const
{
    verts.resize(heights.m,heights.n);
    vector<Vector3> src,dir;
    viewport.getAllRays(src,dir,true,false);
    int k=0;
    for(int i=0;i<heights.m;i++) {
        for(int j=0;j<heights.n;j++,k++) {
            verts(i,j) = src[k] + heights(i,j)*dir[k];
        }
    }

}

void Heightmap::GetVertexColors(vector<Vector3>& colors_out) const
{
    if(colors.num_bytes==0) {
        colors_out.resize(0);
        return;
    }
    colors_out.resize(heights.m*heights.n);
    int k=0;
    for(int i=0;i<heights.m;i++) {
        for(int j=0;j<heights.n;j++,k++) {
            colors_out[k]=GetVertexColor(i,j);
        }
    }
}

void Heightmap::GetVertexRay(int i,int j,Vector3& source,Vector3& dir) const
{
    viewport.deproject(i+0.5,j+0.5,source,dir);
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
    if(hm.viewport.perspective || viewport.perspective) {
        if(hm.viewport != viewport)
            FatalError("TODO: perspective remeshing");
    }
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
    viewport = hm.viewport;
    heights = newheights;
}

void Heightmap::Remesh(const Camera::Viewport& vp)
{
    Heightmap temp;
    temp.viewport = vp;
    Remesh(temp);
}

void Heightmap::SetMesh(const TriMesh& mesh,Real resolution,const RigidTransform* Tmesh,bool topdown)
{
    AABB3D bb;
    if(Tmesh) {
        bb.minimize();
        for(size_t i=0;i<mesh.verts.size();i++)
            bb.expand((*Tmesh)*mesh.verts[i]);
    }
    mesh.GetAABB(bb.bmin,bb.bmax);
    Vector3 center = (bb.bmin+bb.bmax)*0.5;
    Vector3 dims = bb.bmax-bb.bmin;
    Vector3 dims_padded = dims + Vector3(2*resolution);
    IntPair size((int)Ceil(dims_padded.x/resolution),(int)Ceil(dims_padded.y/resolution));

    heights.resize(size.a,size.b);
    heights.set(0.0);
    colors.clear();
    SetSize(dims_padded.x,dims_padded.y);
    viewport.pose.R.setIdentity();
    viewport.pose.t = center;
    if(topdown)
        viewport.pose.t.z = bb.bmin.z;
    else
        viewport.pose.t.z = bb.bmax.z;
    FuseMesh(mesh,Tmesh,topdown);
}

void Heightmap::FuseMesh(const TriMesh& mesh,const RigidTransform* Tmesh,bool topdown)
{
    Math3D::Triangle3D tri;
    bool maximize = (viewport.perspective != topdown);
    vector<IntPair> tcells;
    vector<Real> theights;
    for (size_t i=0;i<mesh.tris.size();i++) {
        mesh.GetTriangle(i,tri);
        if(Tmesh) {
            tri.a = (*Tmesh)*tri.a;
            tri.b = (*Tmesh)*tri.b;
            tri.c = (*Tmesh)*tri.c;
        }
        tri.a = Project(tri.a);
        tri.b = Project(tri.b);
        tri.c = Project(tri.c);
        tcells.resize(0);
        theights.resize(0);
        GetTriangleHeights_Clipped(tri,tcells,theights,0,0,heights.m,heights.n);
        for(size_t k=0;k<tcells.size();k++) {
            float& cell = heights(tcells[k]);
            if(maximize)
                cell = ::Max(float(theights[k]),cell);
            else
                cell = ::Min(float(theights[k]),cell);
        }
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
                auto rgb = GetVertexColor(i,j);
                app.vertexColors[k].set(rgb.x,rgb.y,rgb.z);
            }
        }
    }
}

void Heightmap::SetPointCloud(const PointCloud3D& pc,Real resolution,const RigidTransform* Tpc,bool topdown)
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
    SetSize(dims_padded.x,dims_padded.y);
    viewport.pose.R.setIdentity();
    viewport.pose.t = center;
    if(topdown)
        viewport.pose.t.z = bmin.z;
    else
        viewport.pose.t.z = bmax.z;
    FusePointCloud(pc,Tpc,topdown);
}

void Heightmap::FusePointCloud(const PointCloud3D& pc,const RigidTransform* Tpc,bool topdown)
{
    bool useColor = (pc.HasColor() && colors.num_bytes != 0);
    vector<Vector4> pointColors;
    if(useColor) pc.GetColors(pointColors);
    IntPair ind;
    Vector3 ptemp;
    for(size_t i=0;i<pc.points.size();i++) {
        if(Tpc) {
            Tpc->mulInverse(pc.points[i],ptemp);
            ind = GetIndex(ptemp);
        }
        else
            ind = GetIndex(pc.points[i]);
        if(ind.a < 0 || ind.a >= heights.m) continue;
        if(ind.b < 0 || ind.b >= heights.n) continue;
        viewport.pose.mulInverse(pc.points[i],ptemp);
        float z = ptemp.z;
        if(viewport.perspective != topdown)
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
