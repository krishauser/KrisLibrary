#include "ROI.h"
#include <KrisLibrary/meshing/Meshing.h>
namespace Geometry {

template <class Obj>
void _ExtractROI(const Meshing::PointCloud3D& pc,const Obj& obj,Meshing::PointCloud3D& pc_roi,int flag)
{
    bool truthValue = (flag & ExtractROIFlagInvert ? false : true);
    vector<int> selection;
    selection.reserve(pc.points.size()/4);
    for(size_t i=0;i<pc.points.size();i++) {
        if(obj.contains(pc.points[i])==truthValue) 
            selection.push_back(i);
    }
    pc.GetSubCloud(selection,pc_roi);
}

///Returns the points in pc within bb.  O(V) time, where V is the number of points.
void ExtractROI(const Meshing::PointCloud3D& pc,const AABB3D& bb,Meshing::PointCloud3D& pc_roi,int flag)
{
    _ExtractROI(pc,bb,pc_roi,flag);
}

///Returns the points in pc within bb.  O(V) time, where V is the number of points.
void ExtractROI(const Meshing::PointCloud3D& pc,const Box3D& bb,Meshing::PointCloud3D& pc_roi,int flag)
{
    _ExtractROI(pc,bb,pc_roi,flag);
}

///Returns the points in pc within s.  O(V) time, where V is the number of points.
void ExtractROI(const Meshing::PointCloud3D& pc,const Sphere3D& s,Meshing::PointCloud3D& pc_roi,int flag)
{
    _ExtractROI(pc,s,pc_roi,flag);
}

///Returns the points in pc within bb.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
void ExtractROI(const CollisionPointCloud& pc,const AABB3D& bb,CollisionPointCloud& pc_roi,int flag)
{
    bool truthValue = (flag & ExtractROIFlagInvert ? false : true);
    pc_roi.currentTransform = pc.currentTransform;
    Matrix3 ident; ident.setIdentity();
    if(pc.currentTransform.R == ident) {
        AABB3D bblocal;
        bblocal.bmin = bb.bmin - pc.currentTransform.t;
        bblocal.bmax = bb.bmax - pc.currentTransform.t;
        if(!bblocal.intersects(pc.bblocal)) {
            if(truthValue) {
                pc_roi.points.resize(0);
                pc_roi.properties.resize(0,0);
                pc_roi.bblocal.minimize();
            }
            else {
                pc_roi = pc;
            }
            return;
        }
        _ExtractROI(pc,bblocal,pc_roi,flag);
        //pc_roi.InitCollisions();
    }
    else {
        RigidTransform Tinv;
        pc.currentTransform.getInverse(Tinv);
        Box3D bb_local;
        bb_local.setTransformed(bb,Matrix4(Tinv));
        if(!bb_local.intersects(pc.bblocal)) {
            if(truthValue) {
                pc_roi.points.resize(0);
                pc_roi.properties.resize(0,0);
                pc_roi.bblocal.minimize();
            }
            else {
                pc_roi = pc;
            }
        }
        _ExtractROI(pc,bb_local,pc_roi,flag);
        //pc_roi.InitCollisions();
    }
}

///Returns the points in pc within bb.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
void ExtractROI(const CollisionPointCloud& pc,const Box3D& bb,CollisionPointCloud& pc_roi,int flag)
{
    bool truthValue = (flag & ExtractROIFlagInvert ? false : true);
    pc_roi.currentTransform = pc.currentTransform;
    RigidTransform Tinv;
    pc.currentTransform.getInverse(Tinv);
    Box3D bb_local;
    bb_local.setTransformed(bb,Matrix4(Tinv));
    if(!bb_local.intersects(pc.bblocal)) {
        if(truthValue) {
            pc_roi.points.resize(0);
            pc_roi.properties.resize(0,0);
            pc_roi.bblocal.minimize();
        }
        else {
            pc_roi = pc;
        }
        return;
    }
    _ExtractROI(pc,bb_local,pc_roi,flag);
    //pc_roi.InitCollisions();
}

///Returns the points in pc within s.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
void ExtractROI(const CollisionPointCloud& pc,const Sphere3D& s,CollisionPointCloud& pc_roi,int flag)
{
    bool truthValue = (flag & ExtractROIFlagInvert ? false : true);
    pc_roi.currentTransform = pc.currentTransform;
    Sphere3D slocal;
    pc.currentTransform.mulInverse(s.center,slocal.center);
    slocal.radius = s.radius;
    if(!slocal.intersects(pc.bblocal)) {
        if(truthValue) {
            pc_roi.points.resize(0);
            pc_roi.properties.resize(0,0);
            pc_roi.bblocal.minimize();
        }
        else {
            pc_roi = pc;
        }
        return;
    }

    _ExtractROI(pc,slocal,pc_roi,flag);
    //pc_roi.InitCollisions();
}

void GetTriangles(const Meshing::TriMesh& m,const vector<int>& tris,Meshing::TriMesh& m_roi)
{
    map<int,int> vertmap;
    for(size_t i=0;i<tris.size();i++) {
        int a=m.tris[tris[i]].a,b=m.tris[tris[i]].b,c=m.tris[tris[i]].c;
        int aroi,broi,croi;
        if(vertmap.count(a)) aroi=vertmap[a];
        else {
            aroi = (int)m_roi.verts.size();
            vertmap[a] = aroi;
            m_roi.verts.push_back(m.verts[a]);
        }
        if(vertmap.count(b)) broi=vertmap[b];
        else {
            broi = (int)m_roi.verts.size();
            vertmap[b] = broi;
            m_roi.verts.push_back(m.verts[b]);
        }
        if(vertmap.count(c)) croi=vertmap[c];
        else {
            croi = (int)m_roi.verts.size();
            vertmap[c] = croi;
            m_roi.verts.push_back(m.verts[c]);
        }
        m_roi.tris.push_back(IntTriple(aroi,broi,croi));
    }
}

///Returns the triangles of a mesh intersected/touching/within bb.
///O(T) time, where T is the number of triangles.  Within queries are the fastest.
void ExtractROI(const Meshing::TriMesh& m,const AABB3D& bb,Meshing::TriMesh& m_roi,int flag)
{
    bool truthValue = (flag & ExtractROIFlagInvert ? false : true);
    bool triCheck = (flag & ExtractROIFlagTouching && truthValue) || (flag & ExtractROIFlagWithin && !truthValue) || flag & ExtractROIFlagIntersection;
    m_roi.verts.resize(0);
    m_roi.tris.resize(0);
    m_roi.verts.reserve(m.verts.size()/4);
    m_roi.tris.reserve(m.tris.size()/4);
    vector<bool> valid(m.verts.size());
    map<int,int> vertmap;
    for(size_t i=0;i<m.verts.size();i++)
        valid[i] = (bb.contains(m.verts[i])==truthValue);
    Triangle3D tri;
    for(size_t i=0;i<m.tris.size();i++) {
        int a=m.tris[i].a;
        int b=m.tris[i].b;
        int c=m.tris[i].c;
        bool add=false;
        if(valid[a] && valid[b] && valid[c]) {
            add = true;
        }
        else if(triCheck) {
            m.GetTriangle(i,tri);
            if(!tri.intersects(bb)) {
                if(flag&ExtractROIFlagWithin) add=true;
                continue;
            }
            //triangle intersects the box
            if(flag & ExtractROIFlagTouching) add=true;  //just add the whole triangle
            else {
                //need to clip the triangle to the box's boundaries
                Meshing::TriMeshWithTopology tmesh;
                tmesh.verts.push_back(m.verts[a]);
                tmesh.verts.push_back(m.verts[b]);
                tmesh.verts.push_back(m.verts[c]);
                tmesh.tris.push_back(IntTriple(0,1,2));
                tmesh.CalcTriNeighbors();
                Meshing::TriSplitter splitter(tmesh);
                splitter.deleteNegative = truthValue;
                Plane3D p;
                if(!IsInf(bb.bmin.x)) {
                    p.normal.set(1,0,0);
                    p.offset = bb.bmin.x;
                    splitter.Split(p);
                }
                if(!IsInf(bb.bmax.x)) {
                    p.normal.set(-1,0,0);
                    p.offset = -bb.bmax.x;
                    splitter.Split(p);
                }
                if(!IsInf(bb.bmin.y)) {
                    p.normal.set(0,1,0);
                    p.offset = bb.bmin.y;
                    splitter.Split(p);
                }
                if(!IsInf(bb.bmax.y)) {
                    p.normal.set(0,-1,0);
                    p.offset = -bb.bmax.y;
                    splitter.Split(p);
                }
                if(!IsInf(bb.bmin.z)) {
                    p.normal.set(0,0,1);
                    p.offset = bb.bmin.z;
                    splitter.Split(p);
                }
                if(!IsInf(bb.bmax.z)) {
                    p.normal.set(0,0,-1);
                    p.offset = -bb.bmax.z;
                    splitter.Split(p);
                }
                if(truthValue) {
                    int vlen = (int)m_roi.verts.size();
                    for(size_t j=0;j<tmesh.verts.size();j++) 
                        m_roi.verts.push_back(tmesh.verts[j]);
                    for(size_t j=0;j<tmesh.tris.size();j++) {
                        const auto& t=tmesh.tris[j];
                        m_roi.tris.push_back(IntTriple(t.a+vlen,t.b+vlen,t.c+vlen));
                    }
                }
                else {
                    int vlen = (int)m_roi.verts.size();
                    for(size_t j=0;j<tmesh.verts.size();j++) 
                        m_roi.verts.push_back(tmesh.verts[j]);
                    for(size_t j=0;j<tmesh.tris.size();j++) {
                        tmesh.GetTriangle(j,tri);
                        if(bb.contains(tri.centroid())==truthValue) {
                            const auto& t=tmesh.tris[j];
                            m_roi.tris.push_back(IntTriple(t.a+vlen,t.b+vlen,t.c+vlen));
                        }
                    }
                }
            }
        }
        if(add) {
            int aroi,broi,croi;
            if(vertmap.count(a)) aroi=vertmap[a];
            else {
                aroi = (int)m_roi.verts.size();
                vertmap[a] = aroi;
                m_roi.verts.push_back(m.verts[a]);
            }
            if(vertmap.count(b)) broi=vertmap[b];
            else {
                broi = (int)m_roi.verts.size();
                vertmap[b] = broi;
                m_roi.verts.push_back(m.verts[b]);
            }
            if(vertmap.count(c)) croi=vertmap[c];
            else {
                croi = (int)m_roi.verts.size();
                vertmap[c] = croi;
                m_roi.verts.push_back(m.verts[c]);
            }
            m_roi.tris.push_back(IntTriple(aroi,broi,croi));
        }
    }
}

///Returns the triangles of a mesh intersected/touching/within bb.
///O(T) time, where T is the number of triangles.  Within queries are the fastest.
void ExtractROI(const Meshing::TriMesh& m,const Box3D& bb,Meshing::TriMesh& m_roi,int flag)
{
    Meshing::TriMesh mtemp;
    RigidTransform T,Tinv;
    bb.getTransform(T);
    bb.getTransformInv(Tinv);
    mtemp.verts.resize(m.verts.size());
    mtemp.tris = m.tris;
    for(size_t i=0;i<m.verts.size();i++)
        Tinv.mul(m.verts[i],mtemp.verts[i]);
    AABB3D aabb;
    aabb.bmin.setZero();
    aabb.bmax = bb.dims;
    ExtractROI(mtemp,aabb,m_roi,flag);
    for(size_t i=0;i<m_roi.verts.size();i++) {
        m_roi.verts[i] = T*m_roi.verts[i];
    }
}

///Returns the triangles of a mesh intersected/touching/within bb.
///O(T) time, where T is the number of triangles.  Within queries are the fastest.
void ExtractROI(const CollisionMesh& m,const AABB3D& bb,CollisionMesh& m_roi,int flag)
{
    if(flag & ExtractROIFlagInvert) {
        //TODO: do this faster?
        Meshing::TriMesh mtemp;
        mtemp.verts.resize(m.verts.size());
        for(size_t i=0;i<mtemp.verts.size();i++)
            mtemp.verts[i] = m.currentTransform*m.verts[i];
        mtemp.tris = m.tris;
        ExtractROI(mtemp,bb,m_roi,flag);
        m_roi.currentTransform = m.currentTransform;
        for(size_t i=0;i<m_roi.verts.size();i++) 
            m.currentTransform.mulInverse(Vector3(m_roi.verts[i]),m_roi.verts[i]);
    }
    else {
        vector<int> tris;
        CollideAll(m,bb,tris);
        if(flag == ExtractROIFlagTouching) {
            GetTriangles(m,tris,m_roi);
            m_roi.currentTransform = m.currentTransform;
        }
        else {
            Meshing::TriMesh mtemp;
            GetTriangles(m,tris,mtemp);
            RigidTransform Tm_inv; Tm_inv.setInverse(m.currentTransform);
            Box3D bblocal;
            bblocal.setTransformed(bb,Tm_inv);
            ExtractROI(mtemp,bblocal,m_roi,flag);
            m_roi.currentTransform = m.currentTransform;
        }
    }
}

///Returns the triangles of a mesh intersected/touching/within bb.
///O(T) time, where T is the number of triangles.  Within queries are the fastest.
void ExtractROI(const CollisionMesh& m,const Box3D& bb,CollisionMesh& m_roi,int flag)
{
    if(flag & ExtractROIFlagInvert) {
        //TODO: do this faster?
        Meshing::TriMesh mtemp;
        mtemp.verts.resize(m.verts.size());
        for(size_t i=0;i<mtemp.verts.size();i++)
            mtemp.verts[i] = m.currentTransform*m.verts[i];
        mtemp.tris = m.tris;
        ExtractROI(mtemp,bb,m_roi,flag);
        m_roi.currentTransform = m.currentTransform;
        for(size_t i=0;i<m_roi.verts.size();i++) 
            m.currentTransform.mulInverse(Vector3(m_roi.verts[i]),m_roi.verts[i]);
    }
    else {
        vector<int> tris;
        CollideAll(m,bb,tris);
        if(flag == ExtractROIFlagTouching) {
            GetTriangles(m,tris,m_roi);
            m_roi.currentTransform = m.currentTransform;
        }
        else {
            Meshing::TriMesh mtemp;
            GetTriangles(m,tris,mtemp);
            RigidTransform Tm_inv; Tm_inv.setInverse(m.currentTransform);
            Box3D bblocal;
            bblocal.setTransformed(bb,Tm_inv);
            ExtractROI(mtemp,bblocal,m_roi,flag);
            m_roi.currentTransform = m.currentTransform;
        }
    }
}

} //namespace Geometry