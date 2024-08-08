#ifndef MESHING_HEIGHTMAP_H
#define MESHING_HEIGHTMAP_H

#include "TriMesh.h"
#include "PointCloud.h"
#include <KrisLibrary/math3d/AABB2D.h>
#include <KrisLibrary/structs/array2d.h>
#include <KrisLibrary/image/image.h>
#include <KrisLibrary/camera/viewport.h>
#include <iostream>
namespace GLDraw { class GeometryAppearance;  } //forward declaration

namespace Meshing {

/** @brief Represents a height map or depth map, in either orthographic
 * or projective coordinates.
 * 
 * Array points specify heights at cell *corners*. (Note that this differs
 * from cell-based convention in VolumeGrid.)
 * 
 * The canonical (non-perspective) frame has the image centered 
 * at the origin with (u,v) coordinates [-0.5,-0.5]x[-0.5,0.5].  An 
 * array index (i,j) gives coordinates (u,v)=(i/m-0.5,j/n-0.5). Note 
 * that this differs from standard matrix and image coordinates (row,col)!
 * 
 * offset is set to 0 upon initialization.  If nonzero, the output mesh is translated 
 * uniformly by this amount. 
 * 
 * If perspective = false, the output geometry (x,y,z) has the range
 * [-xysize[0]/2,xysize[0]/2] x [-xysize[1]/2,xysize[1]/2] x [zmin,zmax].
 * 
 * If perspective = true, then this is assumed to take the form of a depth map
 * from a perspective projection camera at the origin, pointing in the +z direction.
 * xysize[0] gives the scaling from horizontal coordinate u to the x coordinate
 * xysize[0]*u*z and xysize[1] gives the scaling from vertical coordinate v to
 * the y coordinate xysize[1]*v*z where z = height[i,j]*zscale.
 * 
 * Vertex colors can be provided in the `colors` attribute.  The `colors` image
 * must have the same dimensions as the `heights` array.  As in standard image
 * convention, the rows in the colors image are assumed to go from top to bottom
 * whereas the heights array goes from bottom to top.
 */ 
class Heightmap
{
public:
    enum { 
        InterpNearest,
        InterpBilinear,
        InterpBicubic
    };

    Heightmap();
    Heightmap(int xdivs,int ydivs,const Vector2& xydims=Vector2(1,1));
    Heightmap(const Array2D<float>& heights,const Vector2& xydims=Vector2(1,1));
    Heightmap(const Array2D<double>& heights,const Vector2& xydims=Vector2(1,1));
    ///See SetImage for description of parameters
    Heightmap(const Image& heights,const Vector2& xydims=Vector2(1,1),float hscale=1,float hoffset=0,bool bottom_row_first=false);
    Heightmap(const Image& heights,const Image& colors,const Vector2& xydims=Vector2(1,1),float hscale=1,float hoffset=0,bool bottom_row_first=false);
    Heightmap(const Heightmap& rhs) = default;
    Heightmap(Heightmap&& rhs) = default;

    bool Load(const char* fn);
    bool Save(const char* fn) const;
    bool Load(std::istream& in,const char* folder=NULL);
    bool Save(std::ostream& out,const char* heightsFn,const char* colorsFn=NULL) const;
    
    /** @The height image must be of format Float or A8. In the former case the heights
     * are just multiplied by hscale, and in the A8 case they are divided by 255 and
     * multiplied by hscale.  They are then offset by hoffset.
     * 
     * If bottom_row_first=false, the image data is interpreted as going from left-to-right,
     * top-to-bottom.  Otherwise, image data is interpreted as going left-to-right, bottom-
     * to-top (Windows BMP style).
     */
    void SetImage(const Image& heights,float hscale=1,float hoffset=0,bool bottom_row_first=false);
    /** @The height image must be of format Float or A8. In the former case the heights
     * are just multiplied by hscale, and in the A8 case they are divided by 255 and
     * multiplied by hscale.  They are then offset by hoffset.
     * 
     * If bottom_row_first=false, the image data is interpreted as going from left-to-right,
     * top-to-bottom.  Otherwise, image data is interpreted as going left-to-right, bottom-
     * to-top (Windows BMP style).
     */
    void SetImage(const Image& heights,const Image& colors,float hscale=1,float hoffset=0,bool bottom_row_first=false);

    /** @brief Exports only the height component as an image.  The image format is
     * FloatA, which usually needs to be converted to A8 format.
     * 
     * To save to a standard 8-bit grayscale file:
     * 
     *     Image im1,im2;
     *     heightmap.GetImage(im1);
     *     im2.initialize(im1.w,im1.h,Image::A8);
     *     im1.blit(im2);
     *     // On Windows
     *     ExportImageGDIPlus("heightmap.png",im2); 
     *     // OR with OpenCV
     *     cv::Mat mat = toMat(im2);
     *     mat.save("heightmap.png"); 
     * 
     * If hmin and hmax are not finite, then the height range is determined automatically
     * and heights are normalized to the range [0,1].
     * 
     * To preserve the current heights, set hmin=0 and hmax=1.
     */
    void GetImage(Image& out,float hmin=Inf,float hmax=Inf,bool bottom_row_first=false) const;

    /** Sets this heightmap to an orthographic projection with the given width and
     * height, with origin at the center of the height map.  height=0 sets the vertical
     * scale for square pixels. */ 
    void SetSize(Real width, Real height=0);
    /** Sets this heightmap to a perspective projection with the given field of view
     * in radians, with origin at the center.  yfov=0 sets the vertical FOV for square
     * pixels. */ 
    void SetFOV(Real xfov,Real yfov=0);

    IntPair NumPoints() const { return IntPair(heights.m,heights.n); }
    IntPair NumCells() const { return IntPair(heights.m-1,heights.n-1); }
    bool HasColors() const { return colors.num_bytes > 0; }
    /** Converts from a grid index to a cell.  Only meaningful if perspective = false. */
    void GetCell(int i,int j,AABB2D& bb) const;
    /** Converts from a grid index to a cell xy center.  Only meaningful if perspective = false. */
    Vector2 GetCellCenter(int i,int j) const;
    /** Gets the dimensions of each cell. Only meaningful if perspective = false. */
    Vector2 GetCellSize() const;
    ///Returns the x,y coordinates of the grid.  Only meaningful if perspective = false
    void GetGrid(vector<Real>& xgrid,vector<Real>& ygrid) const;

    /** Retrieves overall bounding box, including heights. */
    AABB3D GetAABB() const;

    /** Sets projection according to a viewport. */
    void SetViewport(const Camera::Viewport& vp);
    /** Gets viewport according to this projection. */
    void GetViewport(Camera::Viewport & vp) const;

    /** Converts from a point to normalized coordinates [-0.5,0.5]x[-0.5,0.5],z */
    Vector3 ToNormalized(const Vector3& pt) const;
    /** Converts normalized coordinates [-0.5,0.5]x[-0.5,0.5],z to a point */
    Vector3 FromNormalized(const Vector3& params) const;

    /** Converts from a point to image coordinates [0,w-1]x[0,h-1],z */
    Vector3 Project(const Vector3& pt) const;
    /** Converts image coordinates [0,w-1]x[0,h-1],z to a point */
    Vector3 Deproject(const Vector3& params) const;

    /** Projects a point to a grid index.
     *
     * If the coordinates of the point are outside of the range, and clamp
     * =true, then the closest valid grid index will be returned.  Otherwise,
     * an index less than 0 or greater than width-1/height-1 may be returned.
     *
     * Returns:
     *    Indices (i,j) giving the array indices of the lower-left corner
     *    of the grid cell in which pt is located.
     */
    IntPair GetIndex(const Vector3& pt,bool clamp=false) const;

    /** Converts from a point to a grid index.
     *
     * If the coordinates of the point are outside of the range, and clamp
     * =true, then the closest valid grid index will be returned.  Otherwise,
     * an index less than 0 or greater than width-1/height-1 may be returned.
     *
     * Returns:
     *    Indices (i,j) and parameters (u,v).  (i,j) are the array indices of
     *    the lower-left corner of the grid cell in which pt is located. 
     *    (u,v) are the coordinates within the cell, and are each in the range [0,1].
     */
    void GetIndexAndParams(const Vector3& pt,IntPair& index,Vector2& params,bool clamp=false) const;

    /** @brief Reads the height of a point.
     * 
     * The value is clamped to the boundaries.
     *
     * Args:
     *      pt (list or tuple): a 2D or 3D point
     *      local (bool): whether the point is in local coordinates or world
     *          coordinates.
     *      interpolation (str): either 'nearest' or 'bilinear' describing how
     *          the interpolation is done between nearby heightmap values.
     */
    float GetHeight(const Vector3& pt,int interpolation=InterpNearest) const;

    /** @brief Reads the color of a point projected to the heightmap. Arguments
     * are the same as in GetHeight()
     *
     * Returns:
     *      (r,g,b) color, with each channel in the range [0,1].
     */
    Vector3 GetColor(const Vector3& pt,int interpolation=InterpNearest) const;

    /** Returns the coordinates of vertex (i,j) including height. */
    Vector3 GetVertex(int i,int j) const;
    /** Returns the coordinates of vertex (i,j) with height interpolated over cell coordinates (u,v). */
    Vector3 GetVertex(int i,int j,Real u,Real v,int interpolation=InterpNearest) const;
    /** Returns the color of vertex (i,j) as an (R,G,B) tuple. */
    Vector3 GetVertexColor(int i,int j) const;
    /** Returns the color of vertex (i,j) as an (R,G,B) tuple interpolated over cell coordinates (u,v). */
    Vector3 GetVertexColor(int i,int j,Real u,Real v,int interpolation=InterpNearest) const;

    ///Returns w*h vertices
    void GetVertices(vector<Vector3>& verts) const;
    ///Returns a w x h array of vertices
    void GetVertices(Array2D<Vector3>& verts) const;
    ///Returns the colors of w*h vertices.  If no colors are available, this returns an empty list.
    void GetVertexColors(vector<Vector3>& colors) const;
   
    /** Returns the source / direction of a free space ray leading to vertex i,j.
     * The heightmap vertex will be equal to source + dir * height[i,j].
     * 
     * For a perspective heightmap the source is constant but the direction changes.
     * For an orthographic heightmap the source changes (and is below the vertex)
     * but the direction is constant (0,0,1).
     */
    void GetVertexRay(int i,int j,Vector3& source,Vector3& dir) const;

    /** Shifts all heights by this value */
    void Shift(float value);
    /** Scals all heights by this value */
    void Scale(float value);
    /** Takes the min of all heights and this value */
    void Min(float value);
    /** Takes the max of all heights and this value */
    void Max(float value);
    /** Shifts all heights by these values */
    void Shift(const Array2D<float>& values);
    /** Scals all heights by these values */
    void Scale(const Array2D<float>& values);
    /** Takes the min of all heights and these values */
    void Min(const Array2D<float>& values);
    /** Takes the max of all heights and these values */
    void Max(const Array2D<float>& values);
    /** Remeshes the heightmap domain to match the given heightmap's domain. May lose information. */
    void Remesh(const Heightmap& hm);
    /** Remeshes the heightmap domain to match the given viewport's domain. May lose information. */
    void Remesh(const Camera::Viewport& vp);

    /** Resizes and rasterizes a triangle mesh into the height array */
    void SetMesh(const TriMesh& mesh,Real resolution,const RigidTransform* Tmesh=NULL,bool topdown=true);
    void FuseMesh(const TriMesh& mesh,const RigidTransform* Tmesh=NULL,bool topdown=true);
    /** Resizes and rasterizes a point cloud into the height array, and 
     * the colors image if the point cloud is colored.
     */
    void SetPointCloud(const PointCloud3D& pc,Real resolution,const RigidTransform* Tpc=NULL,bool topdown=true);
    void FusePointCloud(const PointCloud3D& pc,const RigidTransform* Tpc=NULL,bool topdown=true);
    void GetMesh(TriMesh& mesh) const;
    void GetMesh(TriMesh& mesh,GLDraw::GeometryAppearance& app) const;
    void GetPointCloud(PointCloud3D& pc) const;
   
    Array2D<float> heights;
    Image colors;
    Vector2 xysize;
    Vector3 offset;
    bool perspective;
};

} //namespace Meshing

#endif