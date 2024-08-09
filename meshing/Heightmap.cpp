#include "Heightmap.h"
#include "Rasterize.h"
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/image/io.h>
#include <memory.h>
#include <fstream>

using namespace Meshing;

/// Used for rendering meshes into heightmaps 
template <class T>
struct HeightmapRasterizer2D : public Meshing::SmoothFillRasterizer2D<T>
{
    HeightmapRasterizer2D(bool _maximize)
        : maximize(_maximize)
        {}
    
    inline void Rasterize(const Triangle3D& tri) {
        Triangle2D tri2;
        this->fillA = tri.a.z;
        this->fillB = tri.b.z; 
        this->fillC = tri.c.z;
        tri2.a.set(tri.a);
        tri2.b.set(tri.b);
        tri2.c.set(tri.c);
        Meshing::SmoothFillRasterizer2D<T>::Rasterize(tri2);
    }

    virtual void Fill(const Vector3& bary,T& cell) {
        T val = bary.x*this->fillA+bary.y*this->fillB+bary.z*this->fillC;
        if(maximize)
            cell = Max(val,cell);
        else
            cell = Min(val,cell);
    }

    bool maximize;
};



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

Heightmap::Heightmap(const Image& _heights,const Vector2& xydims,float hscale,float hoffset,bool bottom_row_first)
:xysize(xydims),offset(Zero),perspective(false)
{
    SetImage(_heights,hscale,hoffset,bottom_row_first);
}

Heightmap::Heightmap(const Image& _heights,const Image& colors,const Vector2& xydims,float hscale,float hoffset,bool bottom_row_first)
:xysize(xydims),offset(Zero),perspective(false)
{
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
        if(!items["xsize"].as(xysize.x)) return false;
        if(!items["ysize"].as(xysize.y)) return false;
        perspective = false;
    }
    else {
        if(!items["xfov"].as(xysize.x)) return false;
        if(!items["yfov"].as(xysize.y)) return false;
        perspective = true;
    }
    if(items.find("offset")) {
        if(!items["offset"][0].as(offset.x)) return false;
        if(!items["offset"][1].as(offset.y)) return false;
        if(!items["offset"][2].as(offset.z)) return false;
    }
    else {
        offset.setZero();
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
    unsigned char imgmin = *std::min_element(himg.data,himg.data+himg.w*himg.h);
    unsigned char imgmax = *std::max_element(himg.data,himg.data+himg.w*himg.h);
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
    if(!perspective) {
        items["xsize"] = xysize.x;
        items["ysize"] = xysize.y;
    }
    else {
        items["xfov"] = 2.0*Atan(xysize.x);
        items["yfov"] = 2.0*Atan(xysize.y);
    }
    if(offset.maxAbsElement() > 0) {
        AnyCollection joffset;
        joffset.resize(3);
        joffset[0] = offset.x;
        joffset[1] = offset.y;
        joffset[2] = offset.z;
        items["offset"] = joffset;
    }
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
        height = width/(heights.m-1)*(heights.n-1);
    }
    xysize.set(width,height);
    perspective = false;
}

void Heightmap::SetFOV(Real xfov,Real yfov)
{
    Real xscale = Tan(xfov*0.5);
    Real yscale = Tan(yfov*0.5);
    if(yfov==0) {
        yscale = xscale/(heights.m-1)*(heights.n-1);
    }
    xysize.set(xscale,yscale);
    perspective = true;
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
    Real xshift = -0.5*xysize.x;
    Real yshift = -0.5*xysize.y;
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
    Real xscale = 0.5*xysize.x;
    Real yscale = 0.5*xysize.y;
    res.bmin.x = -xscale + offset.x;
    res.bmax.x = xscale + offset.x;
    res.bmin.y = -yscale + offset.y;
    res.bmax.y = yscale + offset.y;
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

Vector3 Heightmap::Project(const Vector3& pt) const
{
    Vector3 res=ToNormalized(pt);
    res.x  = (res.x+0.5)*(heights.m-1);
    res.y  = (res.y+0.5)*(heights.n-1);
    return res;
}

Vector3 Heightmap::Deproject(const Vector3& params) const
{
    Vector3 temp=params;
    temp.x = temp.x/(heights.m-1) - 0.5;
    temp.y = temp.y/(heights.n-1) - 0.5;
    return FromNormalized(temp);
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
    if(offset.maxAbsElement() > 0) 
        for(auto& v:verts) v += offset;
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
    Real u = Real(i)/(Real)(heights.m-1) - 0.5;
    Real v = Real(j)/(Real)(heights.n-1) - 0.5;
    if(perspective) {
        source = offset;
        dir.z = 1.0;
        dir.x = xysize[0]*u;
        dir.y = xysize[1]*v;
    }
    else {
        source.x = offset.x + u*xysize[0];
        source.y = offset.y + v*xysize[1];
        source.z = offset.z;
        dir.set(0,0,1);
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
    if(hm.perspective || perspective) {
        if(hm.offset != offset || hm.xysize != xysize)
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

void Heightmap::SetMesh(const TriMesh& mesh,Real resolution,const RigidTransform* Tmesh,bool topdown)
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
    FuseMesh(mesh,NULL,topdown);
}

void Heightmap::FuseMesh(const TriMesh& mesh,const RigidTransform* Tmesh,bool topdown)
{
    Math3D::Triangle3D tri;
    HeightmapRasterizer2D<float> rasterizer(perspective != topdown);
    rasterizer.grid = &heights;
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
        rasterizer.Rasterize(tri);
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
    xysize.set(dims_padded.x,dims_padded.y);
    offset = center;
    if(topdown)
        offset.z = bmin.z;
    else
        offset.z = bmax.z;
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
        float z = pc.points[i].z-offset.z;
        if(perspective != topdown)
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
