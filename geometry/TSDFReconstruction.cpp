#include "TSDFReconstruction.h"
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/math3d/rotationfit.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/math3d/clip.h>
#include <KrisLibrary/math/misc.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/LDL.h>
using namespace Geometry;
using namespace Meshing;
using namespace std;

const static bool REGISTER_RASTERIZE = false;
#define DEBUG 1

namespace Meshing { 

//defined in MarchingCubes.cpp
template <class T>
void TSDFMarchingCubes(const Array3D<T>& input,T isoLevel,T truncationDistance,const AABB3D& bb,TriMesh& m);

} //namespace Meshing

ICPParameters::ICPParameters()
:maxIters(50),subsample(47),pointToPlane(true),colorWeight(0.01),percentileOutliers(0.2),rmseThreshold(1e-3),rmseChangeThreshold(1e-6),
numIters(0),stderr(6,0.0),rmseDistance(0),rmseColor(0),numInliers(0)
{
  Tcamera.setIdentity();
}


DenseTSDFReconstruction::DenseTSDFReconstruction(const AABB3D& volume,const IntTriple& res,Real _truncationDistance)
{
  truncationDistance = _truncationDistance;
  depthStddev0 = 0.005;
  depthStddev1 = 0.01;
  forgettingRate = 0;
  colored = true;

  tsdf.bb = volume;
  tsdf.Resize(res.a,res.b,res.c);
  tsdf.value.set(truncationDistance);
  info.resize(res.a,res.b,res.c);
  VoxelInfo blank;
  blank.rgb[0] = blank.rgb[1] = blank.rgb[2] = 0;
  blank.weight = 0;
  blank.occupancy = 0.5;
  blank.lastID = -1;
  info.set(blank);
  scanID = -1;
  currentMeshID = -1;
}

void DenseTSDFReconstruction::SetTruncationDistance(Real _truncationDistance)
{
  truncationDistance = _truncationDistance;
  tsdf.value.set(truncationDistance);
}

void DenseTSDFReconstruction::Register(const PointCloud3D& pc,const RigidTransform& Tcamera,ICPParameters& params)
{
  //should we try the rasterization approach?
  if(REGISTER_RASTERIZE) FatalError("TODO: use rasterization approach for registration");

  Vector3 p;
  vector<size_t> correspondences_raw,correspondences;
  vector<Real> distances_raw,distances;
  vector<Real> colorDistances;
  vector<Vector4> pc_colors;
  if(colored && params.colorWeight > 0)
    pc.GetColors(pc_colors);
  vector<Vector3> normals_raw,normals;
  vector<Vector3> a,b;
  params.Tcamera = Tcamera;
  params.rmseDistance = 0;
  params.rmseColor = 0;  //ignored
  for(params.numIters=0;params.numIters<params.maxIters;params.numIters++) {
    correspondences_raw.resize(0);
    distances_raw.resize(0);
    normals_raw.resize(0);
    for(size_t i=0;i<pc.points.size();i++) {
      if(i % params.subsample != 0) continue;
      p = params.Tcamera*pc.points[i];
      Real d = tsdf.TrilinearInterpolate(p);
      if(d >= truncationDistance) continue; //no correspondence
      correspondences_raw.push_back(i);
      distances_raw.push_back(d);
      Vector3 g;
      tsdf.Gradient(p,g);
      g.inplaceNormalize();
      normals_raw.push_back(g);
      if(colored && params.colorWeight > 0) {
        float rgb[3];
        GetColor(p-g*d,rgb);
        colorDistances.push_back(Sqr(pc_colors[i].x-rgb[0])+Sqr(pc_colors[i].y-rgb[1])+Sqr(pc_colors[i].z-rgb[2]));
      }
    }

    //outlier rejection
    if(params.percentileOutliers == 0) {
      swap(correspondences,correspondences_raw);
      swap(distances,distances_raw);
      swap(normals,normals_raw);
    }
    else {
      vector<pair<Real,size_t> > sorter(correspondences_raw.size());
      for(size_t i=0;i<correspondences_raw.size();i++) {
        sorter[i].first = Abs(distances_raw[i]) + colorDistances[i]*params.colorWeight;
        sorter[i].second = i;
      }
      sort(sorter.begin(),sorter.end());
      size_t percentile = size_t((1.0-params.percentileOutliers)*sorter.size());
      Real dthresh = Min(sorter[percentile].first,sorter.back().first + params.percentileOutliers * (sorter.front().first-sorter.back().first));
      correspondences.resize(0);
      distances.resize(0);
      normals.resize(0);
      for(size_t i=0;i<correspondences_raw.size();i++) {
        if(sorter[i].first < dthresh) {
          correspondences.push_back(correspondences_raw[sorter[i].second]);
          distances.push_back(distances_raw[sorter[i].second]);
          normals.push_back(normals_raw[sorter[i].second]);
        }
      }
    }
    if(DEBUG) printf("ICP Iteration %d: rmse %g, %d within distance, kept %d\n",params.numIters,params.rmseDistance,(int)correspondences_raw.size(),(int)correspondences.size());
    params.correspondences.resize(correspondences.size());
    for(size_t i=0;i<correspondences.size();i++) {
      params.correspondences[i].first = pc.points[correspondences[i]];
      params.correspondences[i].second = params.Tcamera*pc.points[correspondences[i]] - distances[i]*normals[i];
    }
    params.numInliers = (int)correspondences.size();

    if(correspondences.size() == 0) {
      params.rmseDistance = Inf;
      params.stderr.resize(6,Inf);
      return;
    }

    //determine convergence 
    Real old_rmse = params.rmseDistance;
    Real rmse_distance = 0;
    for(size_t i=0;i<distances.size();i++)
      rmse_distance += Sqr(distances[i]);
    rmse_distance = Sqrt(rmse_distance/distances.size());
    params.rmseDistance = rmse_distance;
    if(rmse_distance < params.rmseThreshold || Abs(rmse_distance - old_rmse) < params.rmseChangeThreshold) {
      //TODO: estimate stderr
      printf("Terminated due to convergence\n");
      printf("  Old RMSE %f, new RMSE %f\n",old_rmse,rmse_distance);
      return;
    }

    //not enough correspondences
    if(correspondences.size() <= 6) {
      params.stderr.resize(6,Inf);
      return;
    }
    
    //minimize error over Tcamera 
    if(params.pointToPlane) {
      //b[i] = a[i] - n[i]*d[i]
      //offset n[i]^T b[i] = n[i]^T a[i] - di
      //minimize sum_i (n[i]^T(R*a[i]+t-b[i]))^2
      //derivative w.r.t t is 
      //  2 sum_i (n[i]^T(R*a[i]+t-b[i])) n[i]
      //If R = exp[wc] R then derivative w.r.t c at c=0 is 
      //  2 sum_i (n[i]^T(R*a[i]+t-b[i])) n[i]^T [w] R a[i]
      LDLDecomposition<Real> ldl;
      Matrix C(6,6);
      Vector b(6);
      Vector cn(6),twist(6);
      vector<Real> origOffsets(distances.size());
      for(size_t i=0;i<correspondences.size();i++) {
        const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
        Real d = dot(a,normals[i]);
        origOffsets[i] = d - distances[i];
      }
      for(int inner=0;inner<10;inner++) {
        C.setZero();
        b.setZero();
        Real sse = 0.0;
        for(size_t i=0;i<correspondences.size();i++) {
          const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
          Vector3 c;
          c.setCross(a,normals[i]);
          c.get(cn[0],cn[1],cn[2]);
          normals[i].get(cn[3],cn[4],cn[5]);
          Real d = dot(a,normals[i]);
          for(int g=0;g<6;g++)
            for(int h=0;h<6;h++)
              C(g,h) += cn[g]*cn[h];
          b.madd(cn,-(d-origOffsets[i]));
          sse += Sqr((d-origOffsets[i]));
        }
        ldl.set(C);
        if(!ldl.backSub(b,twist)) {
          printf("    Terminated due to numerical error\n");
          return;
        }
        if(twist.normSquared() < 1e-8)
          break;
        MomentRotation m;
        m.set(twist[0],twist[1],twist[2]);
        Vector3 d(twist[3],twist[4],twist[5]);
        Matrix3 R;
        m.getMatrix(R);
        //cout<<"    Change in rotation "<<R<<endl;
        //cout<<"    Change in translation "<<d<<endl;
        params.Tcamera.R = R*params.Tcamera.R;
        params.Tcamera.t += d;
        Real sse2 = 0.0;
        for(size_t i=0;i<correspondences.size();i++) {
          const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
          Real d = dot(a,normals[i]) - origOffsets[i];
          sse2 += Sqr(d);
        }
        //printf("    SSE changed from %g to %g\n",sse,sse2);
      }
      Matrix Cinv;
      ldl.getInverse(Cinv);
      for(int i=0;i<6;i++)
        params.stderr(i) = Sqrt(Cinv(i,i));
      //getchar();
        
    }
    else {
      a.resize(correspondences.size());
      b.resize(correspondences.size());
      Real sse = 0.0;
      for(size_t i=0;i<correspondences.size();i++) {
        a[i] = pc.points[correspondences[i]];
        b[i] = params.Tcamera*a[i] - normals[i]*distances[i];
        sse += (params.Tcamera*a[i] - b[i]).normSquared();
      }
      //cout<<"  Original transform"<<endl;
      //cout<<params.Tcamera<<endl;
      RigidTransform Tlast = params.Tcamera;
      Real sse2 = TransformFit(a,b,params.Tcamera.R,params.Tcamera.t);
      //cout<<"  New transform"<<endl;
      //cout<<params.Tcamera<<endl;
      //printf("SSE changed from %f to %f\n",sse,sse2);
      if(IsInf(sse2)) {
        //some numerical error
        params.Tcamera = Tlast;
        params.stderr.resize(6,Inf);
        return;
      }
    }
  }
}

void DenseTSDFReconstruction::Fuse(const RigidTransform& Tcamera,const PointCloud3D& pc,Real weight)
{
  scanID++;
  if(scanID == 0) {
    //first scan, do some setup

    //see if the point cloud is colored
    if(colored)
      if(!pc.HasColor()) {
        colored = false;
      }
    //add auxiliary attributes
    for(size_t i=0;i<auxiliaryAttributes.size();i++) {
      if(auxiliaryAttributes[i] < 0 || auxiliaryAttributes[i] >= (int)pc.propertyNames.size())
        FatalError("Invalid attribute %d",auxiliaryAttributes[i]);
    }
    if(!auxiliaryAttributes.empty()) {
      auxiliary.channels[0].MakeSimilar(tsdf);
      auxiliary.channelNames[0] = pc.propertyNames[auxiliaryAttributes[0]];
      for(size_t i=1;i<auxiliaryAttributes.size();i++)
        auxiliary.AddChannel(pc.propertyNames[auxiliaryAttributes[i]]);
    }
  }
  Timer timer;
  Segment3D s;
  Vector3 cc;
  Vector3 fwd = Tcamera.R*Vector3(0,0,1);
  vector<IntTriple> cells;
  Real oldWeightScale = Exp(-forgettingRate);
  vector<Vector4> colors;
  if(colored)
    pc.GetColors(colors);
  Real rgb[3];
  int numChanged = 0;
  int numValidPoints = 0;

  //center position of cell c is center0 + center1 .* c
  Vector3 center0,center1;
  center1 = tsdf.GetCellSize();
  center0 = tsdf.bb.bmin + 0.5*center1;
  double setupTime = timer.ElapsedTime();
  double pointTime = 0.0;
  double cellTime = 0.0;

  for(size_t i=0;i<pc.points.size();i++) {
    if(DEBUG) timer.Reset();
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    numValidPoints ++;
    Real zmin = p.z - truncationDistance;
    Real zmax = p.z + truncationDistance;
    Real zinv = 1.0/p.z;
    s.a.x = p.x * zmin*zinv;
    s.a.y = p.y * zmin*zinv;
    s.a.z = zmin;
    s.b.x = p.x * zmax*zinv;
    s.b.y = p.y * zmax*zinv;
    s.b.z = zmax;
    s.a = Tcamera*s.a;
    s.b = Tcamera*s.b;
    if(colored) {
      for(int c=0;c<3;c++)
        rgb[c] = (unsigned char)(colors[i][c]*255.0);
    }
    //clamp segment to bbox
    Real u1=0,u2=1;
    if(!ClipLine(s.a,s.b-s.a,tsdf.bb,u1,u2)) continue;
    if(u1 > 0 || u2 < 1) {
      Vector3 a = s.a + Max(0.0,u1)*(s.b-s.a);
      Vector3 b = s.a + Min(1.0,u2)*(s.b-s.a);
      s.a = a;
      s.b = b;
    }
    Vector3 wp = Tcamera*p;
    Vector3 ray = wp - Tcamera.t;
    ray.inplaceNormalize();

    Real certainty = 1.0/(depthStddev0+p.z*depthStddev1);
    Real sweight = weight*certainty;

    if(DEBUG) {
      pointTime += timer.ElapsedTime();
      timer.Reset();
    }
    Meshing::GetSegmentCells(s,tsdf.value.m,tsdf.value.n,tsdf.value.p,tsdf.bb,cells);
    if(oldWeightScale != 1.0) {
      for(size_t j=0;j<cells.size();j++) {
        const IntTriple& c=cells[j];
        VoxelInfo& vox=info(c);
        if(vox.weight > 1e-3)
          vox.weight *= Pow(oldWeightScale,scanID-vox.lastID);
        if(vox.surfaceWeight > 1e-3)
          vox.surfaceWeight *= Pow(oldWeightScale,scanID-vox.lastID);
      }
    }
    numChanged += (int)cells.size();
    for(size_t j=0;j<cells.size();j++) {
      const IntTriple& c=cells[j];
      VoxelInfo& vox=info(c);
      float& dval = tsdf.value(c);
      //tsdf.GetCenter(c,cc);
      cc.x = c.a*center1.x;
      cc.y = c.b*center1.y;
      cc.z = c.c*center1.z;
      cc += center0;
      Real dcell = fwd.dot(cc-Tcamera.t);
      //Real dsurf = cc.distance(wp);
      //if(dcell < p.z)
      //  dsurf = -dsurf;
      Vector3 perp = cc-Tcamera.t - (ray*ray.dot(cc-Tcamera.t));
      Real dperp = perp.norm();
      Real dsurf = dcell - p.z;
      //define a simple falloff
      Real wscale = 1.0-Abs(dsurf+dperp)/truncationDistance;
      if(wscale <= 0) continue;
      Real u = wscale*sweight/(vox.weight+wscale*sweight);
      dval += u*(dsurf-dval);
      if(Abs(dsurf+dperp)*certainty < 3) {
        Real wsurf = Exp(-0.5*Sqr(dsurf+dperp)*certainty);
        Real usurf = wsurf / (vox.surfaceWeight+wsurf);
        //TODO: figure out occupancy estimate
        //Real pfree = 0.5*(Erf(dsurf*certainty)+1);
        //vox.occupancy += pfree*(-vox.occupancy);
        if(colored) {
          for(int c=0;c<3;c++)
            vox.rgb[c] = (unsigned char)(vox.rgb[c] + usurf*(rgb[c] - vox.rgb[c]));
        }
        for(size_t k=0;k<auxiliaryAttributes.size();k++) {
          Real vnew = pc.properties[i][auxiliaryAttributes[k]];
          float& v = auxiliary.channels[k].value(c);
          v += usurf*(vnew - v);
        }
        vox.surfaceWeight += wsurf;
      }
      vox.lastID = scanID;
      vox.weight += wscale*sweight;
    }
    if(DEBUG) 
      cellTime += timer.ElapsedTime();
  }
  if(DEBUG) {
    printf("DenseReconstruction::Fuse: Changed %d cells with %d points\n",numChanged,numValidPoints);
    printf("  Time %f setup %f points %f cells\n",setupTime,pointTime,cellTime);
  }
}


void DenseTSDFReconstruction::ExtractMesh(Meshing::TriMesh& mesh)
{
  if(currentMeshID == scanID) {
    printf("USING CACHED MESH???\n");
    mesh = currentMesh;
    return;
  }
  //make correction for grid cell centers
  AABB3D center_bb = tsdf.bb;
  Vector3 celldims = tsdf.GetCellSize();
  center_bb.bmin += celldims*0.5;
  center_bb.bmax -= celldims*0.5;
  TSDFMarchingCubes(tsdf.value,float(0.0),float(truncationDistance),center_bb,mesh);
  if(REGISTER_RASTERIZE) {
    currentMesh = mesh;
    currentMeshID = scanID;
  }
}

void DenseTSDFReconstruction::ExtractMesh(Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app)
{
  ExtractMesh(mesh);
  if(colored) {
    app.vertexColors.resize(mesh.verts.size());
    //correct for values defined at cell centers
    for(size_t i=0;i<mesh.verts.size();i++) {
      GLDraw::GLColor& color = app.vertexColors[i];
      GetColor(mesh.verts[i],color.rgba);
      color.rgba[3] = 1.0;
    }
  }
}

void DenseTSDFReconstruction::GetColor(const Vector3& point,float* color) const
{
  const static float cscale = 1.0/255;
  IntTriple c;
  Vector3 u;
  Vector3 celldims = tsdf.GetCellSize();
  tsdf.GetIndexAndParams(point-celldims*0.5,c,u);
  if(c.a < 0) { c.a = 0; u.x = 0; }
  if(c.b < 0) { c.b = 0; u.y = 0; }
  if(c.c < 0) { c.c = 0; u.z = 0; }
  if(c.a >= tsdf.value.m-1) { c.a = tsdf.value.m-2; u.x = 1; }
  if(c.b >= tsdf.value.n-1) { c.b = tsdf.value.n-2; u.y = 1; }
  if(c.c >= tsdf.value.p-1) { c.c = tsdf.value.p-2; u.z = 1; }
  vector<const VoxelInfo*> vs;
  vector<Real> ws;
  vs.reserve(8);
  ws.reserve(8);
  for(int p=0;p<=1;p++)
    for(int q=0;q<=1;q++)
      for(int r=0;r<=1;r++) {
        vs.push_back(&info(c.a+p,c.b+q,c.c+r));
        ws.push_back((1-p + (p*2-1)*u.x)*(1-q + (q*2-1)*u.y)*(1-r + (r*2-1)*u.z));
      }
  for(int j=0;j<3;j++) {
    color[j] = 0.0; 
    for(int k=0;k<8;k++)
      color[j] += vs[k]->rgb[j]*ws[k];
    color[j] *= cscale;
  }
}

void DenseTSDFReconstruction::ClearPoint(const Vector3& p,Real dist,Real weight)
{
  FatalError("Not done yet");
}

size_t DenseTSDFReconstruction::MemoryUsage() const
{
  size_t gridsize = tsdf.value.m*tsdf.value.n*tsdf.value.p;
  return (sizeof(float) + sizeof(VoxelInfo) + sizeof(float)*auxiliaryAttributes.size())*gridsize;
}




SparseTSDFReconstruction::SparseTSDFReconstruction(const Vector3& cellSize,Real _truncationDistance)
:tsdf(cellSize*8)
{
  truncationDistance = _truncationDistance;
  depthStddev0 = 0.005;
  depthStddev1 = 0.01;
  forgettingRate = 0;
  colored = true;

  tsdf.channelNames[0] = "depth";
  depthChannel = 0;
  weightChannel = rgbChannel = surfaceWeightChannel = ageChannel = auxiliaryChannelStart = -1;
  scanID = 0;
}

void SparseTSDFReconstruction::Fuse(const RigidTransform& Tcamera,const Meshing::PointCloud3D& pc,Real weight)
{
  if(weightChannel < 0) {
    weightChannel = tsdf.AddChannel("weight");
    ageChannel = tsdf.AddChannel("age");
    if(colored)
      if(!pc.HasColor()) {
        colored = false;
      }
    if(colored || !auxiliaryAttributes.empty()) {
      surfaceWeightChannel = tsdf.AddChannel("surfaceWeight");
    }
    if(colored)
      rgbChannel = tsdf.AddChannel("rgb");
    if(!auxiliaryAttributes.empty()) {
      auxiliaryChannelStart = tsdf.GetNumChannels();
      for(size_t i=0;i<auxiliaryAttributes.size();i++)
        tsdf.AddChannel(pc.propertyNames[auxiliaryAttributes[i]]);
    }

    //initialize truncation distance
    tsdf.defaultValue[0] = truncationDistance;
  }
  scanID++;
  float scanFloat = float(scanID);

  Timer timer;
  Segment3D s;
  Vector3 cc;
  Vector3 fwd = Tcamera.R*Vector3(0,0,1);
  vector<IntTriple> cells;
  Real oldWeightScale = Exp(-forgettingRate);
  vector<Vector4> colors;
  if(colored)
    pc.GetColors(colors);
  Real pointRgb[3];
  int numChanged = 0;
  int numValidPoints = 0;

  //center position of cell c is center0 + center1 .* c
  double setupTime = timer.ElapsedTime();
  double pointTime = 0.0;
  double cellTime = 0.0;

  map<IntTriple,vector<int> > blocks;
  for(size_t i=0;i<pc.points.size();i++) {
    if(DEBUG) timer.Reset();
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    numValidPoints ++;
    Vector3 pw = Tcamera*p;
    IntTriple b;
    set<IntTriple> iblocks;
    tsdf.GetBlock(pw,b);
    tsdf.MakeBlock(b);
    blocks[b].push_back(i);
    iblocks.insert(b);

    Real zmin = p.z - truncationDistance;
    Real zmax = p.z + truncationDistance;
    Real zinv = 1.0/p.z;
    s.a.x = p.x * zmin*zinv;
    s.a.y = p.y * zmin*zinv;
    s.a.z = zmin;
    s.b.x = p.x * zmax*zinv;
    s.b.y = p.y * zmax*zinv;
    s.b.z = zmax;
    s.a = Tcamera*s.a;
    s.b = Tcamera*s.b;
    for(int j=0;j<=4;j++) {
      Real u = j*0.25;
      tsdf.GetBlock((1-u)*s.a + u*s.b, b);
      if(iblocks.count(b)==0) {
        tsdf.MakeBlock(b);
        blocks[b].push_back(i);
        iblocks.insert(b);
      }
    }
  }
  for(auto bindex : blocks) {
    SparseVolumeGrid::Block* b = tsdf.BlockPtr(bindex.first);
    Assert(b != NULL);
    Assert(b->grid.channels.size() == tsdf.channelNames.size());
    blockLastTouched[b->id] = scanID;

    Array3D<float>& depthGrid = b->grid.channels[0].value;
    Array3D<float>& weightGrid = b->grid.channels[weightChannel].value;
    Array3D<float>& ageGrid = b->grid.channels[ageChannel].value;
    Array3D<float> *surfaceWeightGrid = NULL, *rgbGrid = NULL;
    if(surfaceWeightChannel >= 0) surfaceWeightGrid = &b->grid.channels[surfaceWeightChannel].value;
    if(rgbChannel >= 0) rgbGrid = &b->grid.channels[rgbChannel].value;
    Vector3 center0,center1;
    center1 = b->grid.GetCellSize();
    center0 = b->grid.channels[0].bb.bmin + 0.5*center1;
    
    for(auto i : bindex.second) {
      const Vector3& p = pc.points[i];
      Real zmin = p.z - truncationDistance;
      Real zmax = p.z + truncationDistance;
      Real zinv = 1.0/p.z;
      s.a.x = p.x * zmin*zinv;
      s.a.y = p.y * zmin*zinv;
      s.a.z = zmin;
      s.b.x = p.x * zmax*zinv;
      s.b.y = p.y * zmax*zinv;
      s.b.z = zmax;
      s.a = Tcamera*s.a;
      s.b = Tcamera*s.b;
      if(colored) {
        for(int c=0;c<3;c++)
          pointRgb[c] = (unsigned char)(colors[i][c]*255.0);
      }
      //clamp segment to bbox
      Real u1=0,u2=1;
      if(!ClipLine(s.a,s.b-s.a,b->grid.channels[0].bb,u1,u2)) {
        //cout<<"Skipping point "<<i<<" bbox "<<b->grid.channels[0].bb<<endl;;
        continue;
      }
      if(u1 > 0 || u2 < 1) {
        Vector3 va = s.a + Max(0.0,u1)*(s.b-s.a);
        Vector3 vb = s.a + Min(1.0,u2)*(s.b-s.a);
        s.a = va;
        s.b = vb;
      }
      Vector3 wp = Tcamera*p;
      Vector3 ray = wp - Tcamera.t;
      ray.inplaceNormalize();

      Real certainty = 1.0/(depthStddev0+p.z*depthStddev1);
      Real sweight = weight*certainty;

      if(DEBUG) {
        pointTime += timer.ElapsedTime();
        timer.Reset();
      }
      //cout<<"Segment "<<s.a<<" -- "<<s.b<<endl;
      Meshing::GetSegmentCells(s,depthGrid.m,depthGrid.n,depthGrid.p,b->grid.channels[0].bb,cells);
      if(oldWeightScale != 1.0) {
        for(auto c : cells) {
          if(c.a >= weightGrid.m) continue;
          if(c.b >= weightGrid.n) continue;
          if(c.c >= weightGrid.p) continue;
          float& weight = weightGrid(c);
          float& age = ageGrid(c);
          if(weight > 1e-3)
            weight *= Pow(float(oldWeightScale),scanFloat-age);
          if(surfaceWeightGrid) {
            float& surfaceWeight = (*surfaceWeightGrid)(c);
            if(surfaceWeight > 1e-3)
              surfaceWeight *= Pow(float(oldWeightScale),scanFloat-age);
          }
        }
      }
      numChanged += (int)cells.size();
      /*
      cout<<"Block "<<bindex.first<<endl;
      for(auto c : cells)
        cout<<"  "<<c<<endl;
      if(cells.empty()) {
        cout<<"( no cells )"<<endl;
        cout<<"  bbox "<<b->grid.channels[0].bb<<endl;
      }
      getchar();
      break;
      */
      for(auto c : cells) {
        if(c.a >= weightGrid.m) continue;
        if(c.b >= weightGrid.n) continue;
        if(c.c >= weightGrid.p) continue;
        float& dval = depthGrid(c);
        float& weight = weightGrid(c);
        float& age = ageGrid(c);
        //tsdf.GetCenter(c,cc);
        cc.x = c.a*center1.x;
        cc.y = c.b*center1.y;
        cc.z = c.c*center1.z;
        cc += center0;
        Real dcell = fwd.dot(cc-Tcamera.t);
        //Real dsurf = cc.distance(wp);
        //if(dcell < p.z)
        //  dsurf = -dsurf;
        Vector3 perp = cc-Tcamera.t - (ray*ray.dot(cc-Tcamera.t));
        Real dperp = perp.norm();
        Real dsurf = dcell - p.z;
        //define a simple falloff
        Real wscale = 1.0-Abs(dsurf+dperp)/truncationDistance;
        if(wscale <= 0) continue;
        Real u = wscale*sweight/(weight+wscale*sweight);
        dval += u*(dsurf-dval);
        if(surfaceWeightChannel>=0 && Abs(dsurf+dperp)*certainty < 3) {
          float& surfaceWeight = (*surfaceWeightGrid)(c);
          Real wsurf = Exp(-0.5*Sqr(dsurf+dperp)*certainty);
          Real usurf = wsurf / (surfaceWeight+wsurf);
          //TODO: figure out occupancy estimate
          //Real pfree = 0.5*(Erf(dsurf*certainty)+1);
          //vox.occupancy += pfree*(-vox.occupancy);
          if(colored) {
            unsigned char* rgb = reinterpret_cast<unsigned char*>(&(*rgbGrid)(c));
            for(int c=0;c<3;c++)
              rgb[c] = (unsigned char)(rgb[c] + usurf*(pointRgb[c] - rgb[c]));
          }
          for(size_t k=0;k<auxiliaryAttributes.size();k++) {
            Real vnew = pc.properties[i][auxiliaryAttributes[k]];
            float& v = b->grid.channels[auxiliaryChannelStart+k].value(c);
            v += usurf*(vnew - v);
          }
          surfaceWeight += wsurf;
        }
        age = scanFloat;
        weight += wscale*sweight;
      }
      if(DEBUG) 
        cellTime += timer.ElapsedTime();
    }
  }
  if(DEBUG) {
    printf("SparseReconstruction::Fuse: Changed %d cells in %d blocks with %d points\n",numChanged,(int)blocks.size(),numValidPoints);
    printf("  Time %f setup %f points %f cells\n",setupTime,pointTime,cellTime);
  }

}

void SparseTSDFReconstruction::Register(const PointCloud3D& pc,const RigidTransform& Tcamera,ICPParameters& params)
{
  //should we try the rasterization approach?
  if(REGISTER_RASTERIZE) FatalError("TODO: use rasterization approach for registration");

  Vector3 p;
  vector<size_t> correspondences_raw,correspondences;
  vector<Real> distances_raw,distances;
  vector<Real> colorDistances;
  vector<Vector4> pc_colors;
  if(colored && params.colorWeight > 0)
    pc.GetColors(pc_colors);
  vector<Vector3> normals_raw,normals;
  vector<Vector3> a,b;
  params.Tcamera = Tcamera;
  params.rmseDistance = 0;
  params.rmseColor = 0;  //ignored
  for(params.numIters=0;params.numIters<params.maxIters;params.numIters++) {
    correspondences_raw.resize(0);
    distances_raw.resize(0);
    normals_raw.resize(0);
    for(size_t i=0;i<pc.points.size();i++) {
      if(i % params.subsample != 0) continue;
      p = params.Tcamera*pc.points[i];
      Real d = tsdf.TrilinearInterpolate(p);
      if(d >= truncationDistance) continue; //no correspondence
      correspondences_raw.push_back(i);
      distances_raw.push_back(d);
      Vector3 g;
      tsdf.Gradient(p,g);
      g.inplaceNormalize();
      normals_raw.push_back(g);
      if(colored && params.colorWeight > 0) {
        float rgb[3];
        GetColor(p-g*d,rgb);
        colorDistances.push_back(Sqr(pc_colors[i].x-rgb[0])+Sqr(pc_colors[i].y-rgb[1])+Sqr(pc_colors[i].z-rgb[2]));
      }
    }
    //if(DEBUG) printf("ICP Iteration %d: %d within distance\n",params.numIters,(int)correspondences_raw.size());

    //outlier rejection
    if(params.percentileOutliers == 0) {
      swap(correspondences,correspondences_raw);
      swap(distances,distances_raw);
      swap(normals,normals_raw);
    }
    else {
      vector<pair<Real,size_t> > sorter(correspondences_raw.size());
      for(size_t i=0;i<correspondences_raw.size();i++) {
        sorter[i].first = Abs(distances_raw[i]) + colorDistances[i]*params.colorWeight;
        sorter[i].second = i;
      }
      sort(sorter.begin(),sorter.end());
      size_t percentile = size_t((1.0-params.percentileOutliers)*sorter.size());
      Real dthresh = Min(sorter[percentile].first,sorter.back().first + params.percentileOutliers * (sorter.front().first-sorter.back().first));
      correspondences.resize(0);
      distances.resize(0);
      normals.resize(0);
      for(size_t i=0;i<correspondences_raw.size();i++) {
        if(sorter[i].first < dthresh) {
          correspondences.push_back(correspondences_raw[sorter[i].second]);
          distances.push_back(distances_raw[sorter[i].second]);
          normals.push_back(normals_raw[sorter[i].second]);
        }
      }
    }
    //if(DEBUG) printf("  Keeping %d correspondences\n",(int)correspondences.size());
    if(DEBUG) printf("ICP Iteration %d: rmse %g, %d within distance, kept %d\n",params.numIters,params.rmseDistance,(int)correspondences_raw.size(),(int)correspondences.size());
    params.correspondences.resize(correspondences.size());
    for(size_t i=0;i<correspondences.size();i++) {
      params.correspondences[i].first = pc.points[correspondences[i]];
      params.correspondences[i].second = params.Tcamera*pc.points[correspondences[i]] - distances[i]*normals[i];
    }
    params.numInliers = (int)correspondences.size();

    if(correspondences.size() == 0) {
      params.rmseDistance = Inf;
      params.stderr.resize(6,Inf);
      return;
    }

    //determine convergence 
    Real old_rmse = params.rmseDistance;
    Real rmse_distance = 0;
    for(size_t i=0;i<distances.size();i++)
      rmse_distance += Sqr(distances[i]);
    rmse_distance = Sqrt(rmse_distance/distances.size());
    params.rmseDistance = rmse_distance;
    if(rmse_distance < params.rmseThreshold || Abs(rmse_distance - old_rmse) < params.rmseChangeThreshold) {
      //TODO: estimate stderr
      printf("Terminated due to convergence\n");
      printf("  Old RMSE %f, new RMSE %f\n",old_rmse,rmse_distance);
      return;
    }

    //not enough correspondences
    if(correspondences.size() <= 6) {
      params.stderr.resize(6,Inf);
      return;
    }
    
    //minimize error over Tcamera 
    if(params.pointToPlane) {
      //b[i] = a[i] - n[i]*d[i]
      //offset n[i]^T b[i] = n[i]^T a[i] - di
      //minimize sum_i (n[i]^T(R*a[i]+t-b[i]))^2
      //derivative w.r.t t is 
      //  2 sum_i (n[i]^T(R*a[i]+t-b[i])) n[i]
      //If R = exp[wc] R then derivative w.r.t c at c=0 is 
      //  2 sum_i (n[i]^T(R*a[i]+t-b[i])) n[i]^T [w] R a[i]
      LDLDecomposition<Real> ldl;
      Matrix C(6,6);
      Vector b(6);
      Vector cn(6),twist(6);
      vector<Real> origOffsets(distances.size());
      for(size_t i=0;i<correspondences.size();i++) {
        const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
        Real d = dot(a,normals[i]);
        origOffsets[i] = d - distances[i];
      }
      for(int inner=0;inner<10;inner++) {
        C.setZero();
        b.setZero();
        Real sse = 0.0;
        for(size_t i=0;i<correspondences.size();i++) {
          const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
          Vector3 c;
          c.setCross(a,normals[i]);
          c.get(cn[0],cn[1],cn[2]);
          normals[i].get(cn[3],cn[4],cn[5]);
          Real d = dot(a,normals[i]);
          for(int g=0;g<6;g++)
            for(int h=0;h<6;h++)
              C(g,h) += cn[g]*cn[h];
          b.madd(cn,-(d-origOffsets[i]));
          sse += Sqr((d-origOffsets[i]));
        }
        ldl.set(C);
        if(!ldl.backSub(b,twist)) {
          printf("    Terminated due to numerical error\n");
          return;
        }
        if(twist.normSquared() < 1e-8)
          break;
        MomentRotation m;
        m.set(twist[0],twist[1],twist[2]);
        Vector3 d(twist[3],twist[4],twist[5]);
        Matrix3 R;
        m.getMatrix(R);
        //cout<<"    Change in rotation "<<R<<endl;
        //cout<<"    Change in translation "<<d<<endl;
        params.Tcamera.R = R*params.Tcamera.R;
        params.Tcamera.t += d;
        Real sse2 = 0.0;
        for(size_t i=0;i<correspondences.size();i++) {
          const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
          Real d = dot(a,normals[i]) - origOffsets[i];
          sse2 += Sqr(d);
        }
        //printf("    SSE changed from %g to %g\n",sse,sse2);
      }
      Matrix Cinv;
      ldl.getInverse(Cinv);
      for(int i=0;i<6;i++)
        params.stderr(i) = Sqrt(Cinv(i,i));
      //getchar();
        
    }
    else {
      a.resize(correspondences.size());
      b.resize(correspondences.size());
      Real sse = 0.0;
      for(size_t i=0;i<correspondences.size();i++) {
        a[i] = pc.points[correspondences[i]];
        b[i] = params.Tcamera*a[i] - normals[i]*distances[i];
        sse += (params.Tcamera*a[i] - b[i]).normSquared();
      }
      //cout<<"  Original transform"<<endl;
      //cout<<params.Tcamera<<endl;
      RigidTransform Tlast = params.Tcamera;
      Real sse2 = TransformFit(a,b,params.Tcamera.R,params.Tcamera.t);
      //cout<<"  New transform"<<endl;
      //cout<<params.Tcamera<<endl;
      //printf("SSE changed from %f to %f\n",sse,sse2);
      if(IsInf(sse2)) {
        //some numerical error
        params.Tcamera = Tlast;
        params.stderr.resize(6,Inf);
        return;
      }
    }
  }
}


void SparseTSDFReconstruction::ExtractMesh(Meshing::TriMesh& mesh)
{
  mesh.tris.resize(0);
  mesh.verts.resize(0);
  const static int m=8,n=8,p=8;
  float NaN = truncationDistance;
  TriMesh tempMesh;
  Array3D<float> expandedBlock(m+1,n+1,p+1);
  for(auto b : tsdf.hash.buckets) {
    VolumeGridTemplate<float>& depth = reinterpret_cast<SparseVolumeGrid::Block*>(b.second)->grid.channels[0];
    AABB3D center_bb = depth.bb;
    Vector3 celldims = depth.GetCellSize();
    center_bb.bmin += celldims*0.5;
    center_bb.bmax += celldims*0.5; //one extra cell in each dimension

    //copy 8x8x8 block
    for(auto i=depth.value.begin(),j=expandedBlock.begin(Range3Indices(0,m,0,n,0,p));i!=depth.value.end();++i,++j) 
      *j = *i;

    //now work on the seams:
    IntTuple c;
    void* ptr;
    c = b.first;
    c[0] += 1;
    if((ptr=tsdf.hash.Get(c))) {
      Array3D<float> &xnext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
      for(int j=0;j<n;j++) 
        for(int k=0;k<p;k++) 
          expandedBlock(m,j,k) = xnext(0,j,k);
      c[1] += 1;
      if((ptr=tsdf.hash.Get(c))) {
        Array3D<float> &xynext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
        for(int k=0;k<p;k++) 
          expandedBlock(m,n,k) = xynext(0,0,k);
        c[2] += 1;
        if((ptr=tsdf.hash.Get(c))) {
          Array3D<float> &xyznext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
          expandedBlock(m,n,p) = xyznext(0,0,0);
        }
        else
          expandedBlock(m,n,p) = NaN;
        c[2] -= 1;
      }
      else {
        for(int k=0;k<=p;k++) 
          expandedBlock(m,n,k) = NaN;
      }
      c[1] -= 1;
      c[2] += 1;
      if((ptr=tsdf.hash.Get(c))) {
        Array3D<float> &xznext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
        for(int j=0;j<n;j++) 
          expandedBlock(m,j,p) = xznext(0,j,0);
      }
      else {
        for(int j=0;j<n;j++) 
          expandedBlock(m,j,p) = NaN;
      }
      c[2] -= 1;
    }
    else {
      for(int j=0;j<=n;j++) 
        for(int k=0;k<=p;k++) 
          expandedBlock(m,j,k) = NaN;
    }
    c[0] -= 1;
    c[1] += 1;
    if((ptr=tsdf.hash.Get(c))) {
      Array3D<float> &ynext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
      for(int i=0;i<m;i++)
        for(int k=0;k<p;k++) 
          expandedBlock(i,n,k) = ynext(i,0,k);
      c[2] += 1;
      if((ptr=tsdf.hash.Get(c))) {
        Array3D<float> &yznext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
        for(int i=0;i<m;i++) 
          expandedBlock(i,n,p) = yznext(i,0,0);
      }
      else
        for(int i=0;i<m;i++) 
          expandedBlock(i,n,p) = NaN;
      c[2] -= 1;
    }
    else {
      for(int i=0;i<m;i++)
        for(int k=0;k<=p;k++) 
          expandedBlock(i,n,k) = NaN;
    }
    c[1] -= 1;
    c[2] += 1;
    if((ptr=tsdf.hash.Get(c))) {
      Array3D<float> &znext = reinterpret_cast<SparseVolumeGrid::Block*>(ptr)->grid.channels[0].value;
      for(int i=0;i<m;i++)
        for(int j=0;j<n;j++) {
          expandedBlock(i,j,p) = znext(i,j,0);
        }
    }
    else {
      for(int i=0;i<m;i++)
        for(int j=0;j<n;j++) 
          expandedBlock(i,j,p) = NaN;
    }

    TSDFMarchingCubes(expandedBlock,float(0.0),float(truncationDistance),center_bb,tempMesh);
    mesh.MergeWith(tempMesh);
  }
}

void SparseTSDFReconstruction::ExtractMesh(Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app)
{
  //TODO: do this faster without repeated block lookup
  ExtractMesh(mesh);
  if(colored) {
    app.vertexColors.resize(mesh.verts.size());
    //correct for values defined at cell centers
    for(size_t i=0;i<mesh.verts.size();i++) {
      GLDraw::GLColor& color = app.vertexColors[i];
      GetColor(mesh.verts[i],color.rgba);
      color.rgba[3] = 1.0;
    }
  }
}

void SparseTSDFReconstruction::GetColor(const Vector3& point,float* color) const
{
  Assert(rgbChannel >= 0);
  const static float one_over_255 = 1.0/255;
  float rgb = tsdf.GetValue(point,rgbChannel);
  unsigned char* rgbchar = reinterpret_cast<unsigned char*>(&rgb);
  color[0] = rgbchar[0]*one_over_255;
  color[1] = rgbchar[1]*one_over_255;
  color[2] = rgbchar[2]*one_over_255;
}

size_t SparseTSDFReconstruction::MemoryUsage() const
{
  if(tsdf.hash.buckets.empty()) return 0;
  auto i=tsdf.hash.buckets.begin();
  SparseVolumeGrid::Block* b = reinterpret_cast<SparseVolumeGrid::Block*>(i->second);
  size_t blocksize = b->grid.channels[0].value.m*b->grid.channels[0].value.n*b->grid.channels[0].value.p;
  blocksize *= b->grid.channels.size();
  return blocksize*tsdf.hash.buckets.size();
}
