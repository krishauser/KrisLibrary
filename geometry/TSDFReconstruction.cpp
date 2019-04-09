#include "TSDFReconstruction.h"
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/math3d/rotationfit.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/math3d/clip.h>
#include <KrisLibrary/math/misc.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/LDL.h>
#include <KrisLibrary/utils/threadutils.h>
#include <KrisLibrary/utils/arrayutils.h>
#include <list>
#include <unordered_map>
using namespace Geometry;
using namespace Meshing;
using namespace std;

/** A single, unidirectional message between threads.
 *
 * Note: T must be copyable
 */
template <class T>
class InterprocessMessage
{
public:
  InterprocessMessage();
  //waits until something is available, if this has not been set yet
  T get();
  //sets and wakes anyone listening to this
  void set(const T& msg);
  //returns true if already set and not yet read
  bool isSet() const { return available; }
  //un-sets a message that was already set
  void clear() { available = false; }

  T value;
  bool available;
  std::mutex mutex;
  std::condition_variable condition;
};

/** A queued messaging system for communication between threads.
 * Note: T must be copyable
 */
template <class T>
class InterprocessQueue
{
public:
  InterprocessQueue();
  //waits until something is available, if this has not been set yet
  T get();
  //sets and wakes anyone listening to this
  void set(const T& msg);
  //returns true a message already set and not yet read
  size_t unread() const;
  //clears all messages that have been
  void clear();

  std::list<T> values;
  std::mutex mutex;
  std::condition_variable condition;
};


template <class T>
InterprocessMessage<T>::InterprocessMessage()
:available(false)
{}

template <class T>
T InterprocessMessage<T>::get()
{
  std::unique_lock<std::mutex> lock(mutex);
  try {
    condition.wait(lock,[&](){return this->available;});
  }
  catch (const std::system_error& e) {
        std::cout << "Caught system_error with code " << e.code() 
                  << " meaning " << e.what() << '\n';
    }
  T res = value;
  available = false;
  return res;
}

template <class T>
void InterprocessMessage<T>::set(const T& msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  value = msg;
  available = true;
  condition.notify_all();
}

template <class T>
InterprocessQueue<T>::InterprocessQueue()
{}

template <class T>
T InterprocessQueue<T>::get()
{
  std::unique_lock<std::mutex> lock(mutex);
  try {
    condition.wait(lock,[&](){return !this->values.empty();});
  }
  catch (const std::system_error& e) {
        std::cout << "Caught system_error with code " << e.code() 
                  << " meaning " << e.what() << '\n';
    }
  T res = values.front();
  values.pop_front();
  return res;
}

template <class T>
void InterprocessQueue<T>::set(const T& msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  values.push_back(msg);
  condition.notify_all();
}


const static bool REGISTER_RASTERIZE = false;
#define DEBUG 1
#define DEBUG_THREADING 0

namespace Meshing { 

//defined in MarchingCubes.cpp
template <class T>
void TSDFMarchingCubes(const Array3D<T>& input,T isoLevel,T truncationDistance,const AABB3D& bb,TriMesh& m);

} //namespace Meshing

ICPParameters::ICPParameters()
:maxIters(10),subsample(47),pointToPlane(true),colorWeight(0.01),percentileOutliers(0.2),rmseThreshold(1e-3),rmseChangeThreshold(1e-6),
numIters(0),standardError(6,0.0),rmseDistance(0),rmseColor(0),numInliers(0)
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
  tsdf.value.set((float)truncationDistance);
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
  tsdf.value.set((float)truncationDistance);
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
    for(size_t i=0;i<pc.points.size();i+=params.subsample) {
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
      params.standardError.resize(6,Inf);
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
      //TODO: estimate standardError
      printf("Terminated due to convergence\n");
      printf("  Old RMSE %f, new RMSE %f\n",old_rmse,rmse_distance);
      return;
    }

    //not enough correspondences
    if(correspondences.size() <= 6) {
      params.standardError.resize(6,Inf);
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
        params.standardError(i) = Sqrt(Cinv(i,i));
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
        params.standardError.resize(6,Inf);
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
          vox.weight *= (float)Pow(oldWeightScale,scanID-vox.lastID);
        if(vox.surfaceWeight > 1e-3)
          vox.surfaceWeight *= (float)Pow(oldWeightScale,scanID-vox.lastID);
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
      dval += (float)(u*(dsurf-dval));
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
          float vnew = (float)pc.properties[i][auxiliaryAttributes[k]];
          float& v = auxiliary.channels[k].value(c);
          v += (float)(usurf*(vnew - v));
        }
        vox.surfaceWeight += (float)wsurf;
      }
      vox.lastID = scanID;
      vox.weight += (float)(wscale*sweight);
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
  AABB3D roi = tsdf.bb;
  ExtractMesh(roi,mesh);
}

void DenseTSDFReconstruction::ExtractMesh(Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app)
{
  AABB3D roi = tsdf.bb;
  ExtractMesh(roi,mesh,app);
}

void DenseTSDFReconstruction::ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh)
{
  if(currentMeshID == scanID) {
    printf("USING CACHED MESH???\n");
    mesh = currentMesh;
    return;
  }
  IntTriple imin,imax;
  tsdf.GetIndex(roi.bmin,imin);
  tsdf.GetIndex(roi.bmax,imax);
  int amin = Max(0,imin.a);
  int amax = Min(tsdf.value.m,imax.a+1);
  int bmin = Max(0,imin.b);
  int bmax = Min(tsdf.value.n,imax.b+1);
  int cmin = Max(0,imin.c);
  int cmax = Min(tsdf.value.p,imax.c+1);
  int size = (amax-amin)*(bmax-bmin)*(cmax-cmin);
  if(size*4 > tsdf.value.m*tsdf.value.n*tsdf.value.p) {
    //big enough ROI just to take entire mesh
    //make correction for grid cell centers
    AABB3D center_bb = tsdf.bb;
    Vector3 celldims = tsdf.GetCellSize();
    center_bb.bmin += celldims*0.5;
    center_bb.bmax -= celldims*0.5;
    TSDFMarchingCubes(tsdf.value,float(0.0),float(truncationDistance),center_bb,mesh);
  }
  else {
    //extract the ROI
    Array3D<float> subvolume(amax-amin,bmax-bmin,cmax-cmin);
    auto range = Range3Indices(amin,amax,bmin,bmax,cmin,cmax);
    for(auto i=tsdf.value.begin(range), j=subvolume.begin();i!=tsdf.value.end(range);++i) 
      *j = *i;
    AABB3D center_bb;
    tsdf.GetCenter(IntTriple(amin,bmin,cmin),center_bb.bmin);
    tsdf.GetCenter(IntTriple(amax,bmax,cmax),center_bb.bmax);
    Vector3 celldims = tsdf.GetCellSize();
    center_bb.bmin += celldims*0.5;
    center_bb.bmax -= celldims*0.5;
    TSDFMarchingCubes(subvolume,float(0.0),float(truncationDistance),center_bb,mesh);
  }

  if(REGISTER_RASTERIZE) {
    currentMesh = mesh;
    currentMeshID = scanID;
  }
}

void DenseTSDFReconstruction::ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app)
{
  ExtractMesh(roi,mesh);
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
  const static float cscale = 1.0f/255;
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
  vector<float> ws;
  vs.reserve(8);
  ws.reserve(8);
  for(int p=0;p<=1;p++)
    for(int q=0;q<=1;q++)
      for(int r=0;r<=1;r++) {
        vs.push_back(&info(c.a+p,c.b+q,c.c+r));
        ws.push_back(float((1-p + (p*2-1)*u.x)*(1-q + (q*2-1)*u.y)*(1-r + (r*2-1)*u.z)));
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

enum { JOB_QUIT, JOB_FIND_BLOCKS, JOB_FUSE, JOB_CORRESPONDENCES, JOB_THRESHOLD, JOB_COVARIANCE };

struct FuseThreadJob
{
  int type;
  vector<size_t> indices;
};

//typedef list<size_t> POINTINDEXLIST;
typedef vector<size_t> POINTINDEXLIST;

struct FuseThreadData
{
  int id;
  int numThreads;
  Mutex* tsdfLock;

  InterprocessMessage<FuseThreadJob> job;
  InterprocessQueue<int>* finished;

  //in
  const RigidTransform* Tcamera;
  const Meshing::PointCloud3D* pc;
  SparseTSDFReconstruction* reconstruction;
  Real weight;
  vector<Vector4>* colors;
  vector<SparseVolumeGrid::Block*>* blocks;
  vector<vector<POINTINDEXLIST*> >* blockpoints;
 
  //out -- this is computed in JOB_FIND_BLOCKS and must be accessible for other threads to use in JOB_FUSE
  unordered_map<IntTriple,POINTINDEXLIST,IndexHash> blockToPoint;
  //map<IntTriple,POINTINDEXLIST> blockToPoint;

  //out -- for JOB_FUSE
  double cellTime,pointTime;
  int numChangedCells;

  //out -- for JOB_CORRESPONDENCES / JOB_THRESHOLD.  must be accessible for JOB_COVARIANCE
  vector<size_t> correspondences_raw,correspondences;
  vector<Real> distances_raw,distances;
  vector<Vector3> normals_raw,normals;
  vector<Real> colorDistances;
  vector<Real> origOffsets;
  //in
  Real colorWeight,dthresh;

  //out -- for JOB_COVARIANCE
  //for point to plane distances
  Matrix C;
  Vector d;
  Vector cn;
};


SparseTSDFReconstruction::SparseTSDFReconstruction(const Vector3& cellSize,Real _truncationDistance)
:tsdf(cellSize*8)
{
  truncationDistance = _truncationDistance;
  depthStddev0 = 0.005;
  depthStddev1 = 0.01;
  forgettingRate = 0;
  colored = true;
  numThreads = 1;

  tsdf.channelNames[0] = "depth";
  depthChannel = 0;
  weightChannel = rgbChannel = surfaceWeightChannel = ageChannel = auxiliaryChannelStart = -1;
  scanID = 0;
}

SparseTSDFReconstruction::~SparseTSDFReconstruction()
{
  StopThreads();
}

void DoFindBlocks(FuseThreadData* data,const vector<size_t>& pindices)
{
  //SparseTSDFReconstruction* self = data->reconstruction;
  SparseVolumeGrid& tsdf = data->reconstruction->tsdf;
  Real truncationDistance = data->reconstruction->truncationDistance;
  Mutex* tsdfLock = data->tsdfLock;
  const Meshing::PointCloud3D& pc = *data->pc;
  const RigidTransform& Tcamera = *data->Tcamera;
  data->blockToPoint.clear();
  Segment3D s;
  for(size_t i : pindices) {
    const Vector3& p = pc.points[i];
    //if(p.z == 0 || !IsFinite(p.z)) continue;
    Vector3 pw = Tcamera*p;
    IntTriple b;
    tsdf.GetBlock(pw,b);
    set<IntTriple> bindex;
    bindex.insert(b);

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
    int n=4;
    Real dt = 1.0/n;
    for(int j=0;j<=n;j++) {
      if(j*2==n) continue;
      Real u = j*dt;
      tsdf.GetBlock((1-u)*s.a + u*s.b, b);
      bindex.insert(b);
    }
    for(const auto& b : bindex) {
      data->blockToPoint[b].push_back(i);
    }
  }
}

void DoFuse(FuseThreadData* data,const vector<size_t>& bindices)
{
  SparseTSDFReconstruction* self = data->reconstruction;
  SparseVolumeGrid& tsdf = data->reconstruction->tsdf;
  Real truncationDistance = data->reconstruction->truncationDistance;
  Mutex* tsdfLock = data->tsdfLock;
  vector<SparseVolumeGrid::Block*>& blocks = *data->blocks;
  vector<vector<POINTINDEXLIST*> >& blockpoints = *data->blockpoints;
  const RigidTransform& Tcamera = *data->Tcamera;
  const Meshing::PointCloud3D& pc = *data->pc;
  Real weight = data->weight;
  
  Timer timer;
  Timer overallTimer;
  Segment3D s;
  Vector3 cc;
  Vector3 fwd = Tcamera.R*Vector3(0,0,1);
  vector<IntTriple> cells;
  float oldWeightScale = (float)Exp(-self->forgettingRate);
  unsigned char pointRgb[3];
  data->pointTime = 0;
  data->cellTime = 0;
  data->numChangedCells = 0;
  float scanFloat = float(self->scanID);

  for(size_t j : bindices) {
    SparseVolumeGrid::Block* b = blocks[j];
    Assert(b != NULL);
    Assert(b->grid.channels.size() == tsdf.channelNames.size());
    if(tsdfLock) tsdfLock->lock();
    self->blockLastTouched[b->id] = self->scanID;
    if(tsdfLock) tsdfLock->unlock();

    Array3D<float>& depthGrid = b->grid.channels[0].value;
    Array3D<float>& weightGrid = b->grid.channels[self->weightChannel].value;
    Array3D<float>& ageGrid = b->grid.channels[self->ageChannel].value;
    Array3D<float> *surfaceWeightGrid = NULL, *rgbGrid = NULL;
    if(self->surfaceWeightChannel >= 0) surfaceWeightGrid = &b->grid.channels[self->surfaceWeightChannel].value;
    if(self->rgbChannel >= 0) rgbGrid = &b->grid.channels[self->rgbChannel].value;
    Vector3 center0,center1;
    center1 = b->grid.GetCellSize();
    center0 = b->grid.channels[0].bb.bmin + 0.5*center1;
    
    //for(auto i : blockpoints[j]) {
    for(auto& listptr : blockpoints[j]) {
      POINTINDEXLIST& ptlist = *listptr;
      for(auto i : ptlist) {
        assert(i >= 0 && i < pc.points.size());
      }
      for(auto i : ptlist) {
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
        if(self->colored) {
          for(int c=0;c<3;c++)
            pointRgb[c] = (unsigned char)((*data->colors)[i][c]*255.0);
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

        Real certainty = 1.0/(self->depthStddev0+p.z*self->depthStddev1);
        Real sweight = weight*certainty;

        if(DEBUG) {
          data->pointTime += timer.ElapsedTime();
          timer.Reset();
        }
        //cout<<"Segment "<<s.a<<" -- "<<s.b<<endl;
        Meshing::GetSegmentCells(s,depthGrid.m,depthGrid.n,depthGrid.p,b->grid.channels[0].bb,cells);
        if(oldWeightScale != 1.0f) {
          for(const auto& c : cells) {
            if(c.a >= weightGrid.m) continue;
            if(c.b >= weightGrid.n) continue;
            if(c.c >= weightGrid.p) continue;
            float& weight = weightGrid(c);
            float& age = ageGrid(c);
            if(weight > 1e-3)
              weight *= Pow(oldWeightScale,scanFloat-age);
            if(surfaceWeightGrid) {
              float& surfaceWeight = (*surfaceWeightGrid)(c);
              if(surfaceWeight > 1e-3)
                surfaceWeight *= Pow(oldWeightScale,scanFloat-age);
            }
          }
        }
        data->numChangedCells += (int)cells.size();
        for(const auto& c : cells) {
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
          float wscale = float(1.0-Abs(dsurf+dperp)/truncationDistance);
          if(wscale <= 0) continue;
          float u = wscale*sweight/(weight+wscale*sweight);
          dval += u*(dsurf-dval);
          if(self->surfaceWeightChannel>=0 && Abs(dsurf+dperp)*certainty < 3) {
            float& surfaceWeight = (*surfaceWeightGrid)(c);
            float wsurf = (float)Exp(-0.5*Sqr(dsurf+dperp)*certainty);
            float usurf = wsurf / (surfaceWeight+wsurf);
            //TODO: figure out occupancy estimate
            //Real pfree = 0.5*(Erf(dsurf*certainty)+1);
            //vox.occupancy += pfree*(-vox.occupancy);
            if(self->colored) {
              unsigned char* rgb = reinterpret_cast<unsigned char*>(&(*rgbGrid)(c));
              for(int c=0;c<3;c++)
                rgb[c] = (unsigned char)(rgb[c] + usurf*(pointRgb[c] - rgb[c]));
            }
            for(size_t k=0;k<self->auxiliaryAttributes.size();k++) {
              float vnew = (float)pc.properties[i][self->auxiliaryAttributes[k]];
              float& v = b->grid.channels[self->auxiliaryChannelStart+k].value(c);
              v += usurf*(vnew - v);
            }
            surfaceWeight += wsurf;
          }
          age = scanFloat;
          weight += wscale*sweight;
        }
        if(DEBUG) {
          data->cellTime += timer.ElapsedTime();
          timer.Reset();
        }
      }
    }
  }
}

void DoCorrespondences(FuseThreadData* data,const vector<size_t>& indices)
{
  SparseTSDFReconstruction* self=data->reconstruction;
  const RigidTransform& Tcamera = *data->Tcamera;
  const Meshing::PointCloud3D& pc = *data->pc;
  vector<Vector4>& pc_colors = *data->colors;

  data->correspondences_raw.resize(0);
  data->distances_raw.resize(0);
  data->normals_raw.resize(0);
  for(size_t i: indices) {
    Vector3 p = Tcamera*pc.points[i];
    Real d = self->tsdf.TrilinearInterpolate(p);
    if(d >= self->truncationDistance) continue; //no correspondence
    data->correspondences_raw.push_back(i);
    data->distances_raw.push_back(d);
    Vector3 g;
    self->tsdf.Gradient(p,g);
    g.inplaceNormalize();
    data->normals_raw.push_back(g);
    if(self->colored) {
      float rgb[3];
      self->GetColor(p-g*d,rgb);
      data->colorDistances.push_back(Sqr(pc_colors[i].x-rgb[0])+Sqr(pc_colors[i].y-rgb[1])+Sqr(pc_colors[i].z-rgb[2]));
    }
  }
}

void DoThreshold(FuseThreadData* data)
{
  SparseTSDFReconstruction* self=data->reconstruction;
  const RigidTransform& Tcamera = *data->Tcamera;
  const Meshing::PointCloud3D& pc = *data->pc;
  data->correspondences.resize(0);
  data->distances.resize(0);
  data->normals.resize(0);
  for(size_t j=0;j<data->distances_raw.size();j++) {
    Real v = data->distances_raw[j];
    if(self->colored)
      v += data->colorDistances[j]*data->colorWeight;
    if(v < data->dthresh) {
      data->correspondences.push_back(data->correspondences_raw[j]);
      data->distances.push_back(data->distances_raw[j]);
      data->normals.push_back(data->normals_raw[j]);
    }
  }
  data->origOffsets.resize(data->correspondences.size());
  for(size_t j=0;j<data->correspondences.size();j++) {
    const Vector3& a=Tcamera*pc.points[data->correspondences[j]];
    Real d = dot(a,data->normals[j]);
    data->origOffsets[j] = d - data->distances[j];
  }
}

void DoCovariance(FuseThreadData* data)
{
  if(data->C.isEmpty()) {
    data->C.resize(6,6);
    data->d.resize(6);
    data->cn.resize(6);
  }
  data->C.setZero();
  data->d.setZero();
  const RigidTransform& Tcamera = *data->Tcamera;
  Vector& cn = data->cn;
  for(size_t i=0;i<data->correspondences.size();i++) {
    const Vector3& a=Tcamera*data->pc->points[data->correspondences[i]];
    Vector3 c;
    c.setCross(a,data->normals[i]);
    c.get(cn[0],cn[1],cn[2]);
    data->normals[i].get(cn[3],cn[4],cn[5]);
    Real dist = dot(a,data->normals[i]);
    for(int g=0;g<6;g++)
      for(int h=0;h<6;h++)
        data->C(g,h) += cn[g]*cn[h];
    data->d.madd(cn,-(dist-data->origOffsets[i]));
    //sse += Sqr((dist-origOffsets[i]));
  }
}

void* FuseThread(void* vdata)
{
  auto data=reinterpret_cast<FuseThreadData*>(vdata);
  if(DEBUG_THREADING) printf("Thread %d started\n",data->id);
  while(true) {
    //printf("Thread %d waiting for next job\n",data->id);
    FuseThreadJob job = data->job.get();
    if(job.type == JOB_QUIT) {
      if(DEBUG_THREADING) printf("Thread %d terminating\n",data->id);
      return vdata;
    }
    else if(job.type == JOB_FIND_BLOCKS) {
      if(DEBUG_THREADING) printf("Thread %d working on JOB_FIND_BLOCKS, points %d - %d\n",data->id,job.indices.front(),job.indices.back());
      Timer timer;
      DoFindBlocks(data,job.indices);
      if(DEBUG_THREADING) printf("Thread %d finished JOB_FIND_BLOCKS in time %g\n",data->id,timer.ElapsedTime());
      if(data->finished)
        data->finished->set(data->id);
    }
    else if(job.type == JOB_FUSE) {
      Timer timer;
      if(DEBUG_THREADING) printf("Thread %d working on JOB_FUSE, blocks %d - %d\n",data->id,job.indices.front(),job.indices.back());
      DoFuse(data,job.indices);
      if(DEBUG_THREADING) printf("Thread %d finished JOB_FUSE in time %g\n",data->id,timer.ElapsedTime());
      if(data->finished)
        data->finished->set(data->id);
    }
    else if(job.type == JOB_CORRESPONDENCES) {
      Timer timer;
      if(DEBUG_THREADING) printf("Thread %d working on JOB_CORRESPONDENCES, blocks %d - %d\n",data->id,job.indices.front(),job.indices.back());
      DoCorrespondences(data,job.indices);
      if(DEBUG_THREADING) printf("Thread %d finished JOB_CORRESPONDENCES in time %g\n",data->id,timer.ElapsedTime());
      if(data->finished)
        data->finished->set(data->id);
    }
    else if(job.type == JOB_THRESHOLD) {
      Timer timer;
      if(DEBUG_THREADING) printf("Thread %d working on JOB_THRESHOLD\n",data->id);
      DoThreshold(data);
      if(DEBUG_THREADING) printf("Thread %d finished JOB_THRESHOLD\n", data->id);
      if(data->finished)
        data->finished->set(data->id);
    }
    else if(job.type == JOB_COVARIANCE) {
      Timer timer;
      if(DEBUG_THREADING) printf("Thread %d working on JOB_COVARIANCE\n", data->id);
      DoCovariance(data);
      if(DEBUG_THREADING) printf("Thread %d finished JOB_COVARIANCE\n", data->id);
      if(data->finished)
        data->finished->set(data->id);
    }
    else
      FatalError("Invalid job type");
  }
  
  return vdata;
}


void SparseTSDFReconstruction::StartThreads()
{
  Assert(threads.empty());
  //begin threads
  threadData.resize(numThreads);
  for(int i=0;i<numThreads;i++) {
    FuseThreadData* idata = new FuseThreadData;
    idata->reconstruction = this;
    if(numThreads > 0)
      idata->tsdfLock = &lock;
    else
      idata->tsdfLock = NULL;
    if(numThreads > 1) 
      idata->tsdfLock = &lock;
    else
      idata->numThreads = 1;
    idata->id = i;
    threadData[i] = idata;
  }

  if(DEBUG_THREADING) printf("Firing up %d threads\n",numThreads);
  threads.resize(numThreads);
  for(int i=0;i<numThreads;i++) 
    threads[i] = ThreadStart(FuseThread,threadData[i]);
}

void SparseTSDFReconstruction::StopThreads()
{
  for(size_t i=0;i<threads.size();i++) {
    FuseThreadData* tdata = reinterpret_cast<FuseThreadData*>(threadData[i]);
    FuseThreadJob job;
    job.type = JOB_QUIT;
    tdata->job.set(job);
    ThreadJoin(threads[i]);
    delete tdata;
  }
  threads.resize(0);
  threadData.resize(0);
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
  //Segment3D s;
  //Vector3 cc;
  //Vector3 fwd = Tcamera.R*Vector3(0,0,1);
  //vector<IntTriple> cells;
  //Real oldWeightScale = Exp(-forgettingRate);
  vector<Vector4> colors;
  if(colored)
    pc.GetColors(colors);
  //Real pointRgb[3];
  int numChanged = 0;
  int numValidPoints = 0;

  double setupTime = timer.ElapsedTime();
  double pointTime = 0.0;
  double cellTime = 0.0;
  size_t origNumBlocks = tsdf.hash.buckets.size();

  Timer jtimer;

  vector<SparseVolumeGrid::Block*> blocks;
  vector<vector<POINTINDEXLIST*> > blockpoints;

  InterprocessQueue<int> finished;
  vector<FuseThreadData*> idata(numThreads);    
  if(threads.empty()) {
    StartThreads();
  }
  //set up jobs
  for(size_t i=0;i<threadData.size();i++) {
    idata[i] = reinterpret_cast<FuseThreadData*>(threadData[i]);
    idata[i]->pc = &pc;
    idata[i]->Tcamera = &Tcamera;
    idata[i]->weight = weight;
    idata[i]->colors = &colors;
    idata[i]->blocks = &blocks;
    idata[i]->blockpoints = &blockpoints;
    idata[i]->finished = &finished;
  }

  for(size_t i=0;i<pc.points.size();i++) {
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    numValidPoints ++;
  }

  //distribute work evenly
  vector<FuseThreadJob> jobs(numThreads);
  int pointCounter = 0;
  for(size_t i=0;i<pc.points.size();i++) {
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    int threadIndex = pointCounter * numThreads / numValidPoints;
    jobs[threadIndex].indices.push_back(i);
    pointCounter ++;
  }
  //send out jobs
  for(int i=0;i<numThreads;i++) {
    if(DEBUG_THREADING) printf("Sending out JOB_FIND_BLOCKS to thread %d\n",i);
    jobs[i].type = JOB_FIND_BLOCKS;
    idata[i]->job.set(jobs[i]);
  }
  if(DEBUG_THREADING) printf("Time to distribute and send out jobs %g\n",jtimer.ElapsedTime());

  //wait for jobs to complete
  vector<bool> done(numThreads,false);
  for(int i=0;i<numThreads;i++) {
    int thread = finished.get();
    assert(done[thread] == false);
    done[thread] = true;
    if(DEBUG_THREADING)  printf("Thread %d done with JOB_FIND_BLOCKS\n",thread);
  }

  //gather data 
  jtimer.Reset();
  //blocks.reserve(tsdf.hash.buckets.size());
  //blockpoints.reserve(tsdf.hash.buckets.size());
  unordered_map<IntTriple,size_t,IndexHash> blockToIndex;
  //map<IntTriple,size_t> blockToIndex;
  double findTime = 0,newTime = 0,insertTime = 0;
  for(int i=0;i<numThreads;i++) {
    FuseThreadData* data = idata[i];
    for(const auto& bmap:data->blockToPoint) {
      const IntTriple& b=bmap.first;
      SparseVolumeGrid::Block* bptr;
      size_t bindex;
      auto bind = blockToIndex.find(b);
      if(bind != blockToIndex.end())
        bindex = bind->second;
      else {
        bptr = tsdf.GetMakeBlock(b); //this may or may not create a block
        blockToIndex[b] = blocks.size();
        bindex = blocks.size();
        blocks.push_back(bptr);
        blockpoints.push_back(vector<POINTINDEXLIST*>());
      }
      blockpoints[bindex].push_back(&data->blockToPoint[bmap.first]);
      /*
      for(i : bmap.second)
        blockpoints[bindex].push_back(i);
      */
    }
  }
  if(DEBUG_THREADING) printf("Time to gather data %g\n",jtimer.ElapsedTime());

  /*
  for(size_t i=0;i<pc.points.size();i++) {
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    Vector3 pw = Tcamera*p;
    IntTriple b;
    tsdf.GetBlock(pw,b);
    set<IntTriple> bindex;
    bindex.insert(b);

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
    int n=4;
    Real dt = 1.0/n;
    for(int j=0;j<=n;j++) {
      if(j*2==n) continue;
      Real u = j*dt;
      tsdf.GetBlock((1-u)*s.a + u*s.b, b);
      bindex.insert(b);
    }
    for(const auto& b : bindex) {
      SparseVolumeGrid::Block* bptr;
      if(tsdf.MakeBlock(b)) {
        bptr = tsdf.BlockPtr(b);
        blocks.resize(bptr->id+1);
        blockpoints.resize(bptr->id+1);
      }
      else
        bptr = tsdf.BlockPtr(b);
      blocks[bptr->id] = bptr;
      blockpoints[bptr->id].push_back(i);
    }
  }
  */
  if(DEBUG) {
    pointTime += timer.ElapsedTime();
    timer.Reset();
  }
  printf("Block identification time %g, created %d blocks\n",pointTime,(int)(tsdf.hash.buckets.size()-origNumBlocks));

  //distribute work evenly and adaptively
  jtimer.Reset();
  size_t batchSize = blocks.size();
  if(numThreads > 1)
    batchSize /= (numThreads*numThreads);
  fill(done.begin(),done.end(),false);
  list<int> freeThreads;
  for(int i=0;i<numThreads;i++)
    freeThreads.push_back(i);
  size_t ctr = 0;
  while(ctr < blocks.size()) {
    FuseThreadJob job;
    job.type = JOB_FUSE;
    job.indices.reserve(batchSize);
    for(size_t i=0;i<batchSize;i++) {
      if(ctr == blocks.size()) break;
      job.indices.push_back(ctr);
      ctr++;
    }
    if(job.indices.empty()) break; //done
    if(freeThreads.empty()) {
      //printf("Waiting for free thread on block %d\n",ctr);
      int id = finished.get();
      done[id] = true;
      freeThreads.push_back(id);
    }
    int t = freeThreads.front();
    freeThreads.pop_front();
    done[t] = false;
    idata[t]->job.set(job);
  }
  if(DEBUG_THREADING) printf("Time to distribute and send out jobs %g\n",jtimer.ElapsedTime());

  //wait for jobs to complete
  while((int)freeThreads.size() != numThreads) {
    int id = finished.get();
    done[id] = true;
    freeThreads.push_back(id);
  }
  printf("Parallelized filling time: %g\n",timer.ElapsedTime());

  //gather data
  for(int i=0;i<numThreads;i++) {
    pointTime += idata[i]->pointTime;
    cellTime += idata[i]->cellTime;
    numChanged += idata[i]->numChangedCells;
  }

  /*
  for(size_t j=0;j<blocks.size();j++) {
    if(!blocks[j]) continue;
    if(DEBUG) timer.Reset();
    SparseVolumeGrid::Block* b = blocks[j];
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
    
    for(const auto& i : blockpoints[j]) {
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
        for(const auto& c : cells) {
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
      for(const auto& c : cells) {
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
      if(DEBUG) {
        cellTime += timer.ElapsedTime();
        timer.Reset();
      }
    }
  }
  */
  if(DEBUG) {
    printf("SparseReconstruction::Fuse: Changed %d cells in %d blocks with %d points\n",numChanged,(int)blocks.size(),numValidPoints);
    printf("  Time %f setup, sum of thread times %f points %f cells\n",setupTime,pointTime,cellTime);
  }
}


void SparseTSDFReconstruction::Register(const PointCloud3D& pc,const RigidTransform& Tcamera,ICPParameters& params)
{
  //should we try the rasterization approach?
  if(REGISTER_RASTERIZE) FatalError("TODO: use rasterization approach for registration");

  vector<Vector4> pc_colors;
  if(colored && params.colorWeight > 0)
    pc.GetColors(pc_colors);

  InterprocessQueue<int> finished;
  vector<FuseThreadData*> idata(numThreads);    
  if(threads.empty()) {
    StartThreads();
  }
  //set up jobs
  for(size_t i=0;i<threadData.size();i++) {
    idata[i] = reinterpret_cast<FuseThreadData*>(threadData[i]);
    idata[i]->finished = &finished;
    idata[i]->pc = &pc;
    idata[i]->Tcamera = &params.Tcamera;
    idata[i]->colors = &pc_colors;
  }

  Vector3 p;
  vector<size_t> correspondences_raw,correspondences;
  vector<Real> distances_raw,distances;
  vector<Real> colorDistances;
  vector<Real> origOffsets;
  vector<Vector3> normals_raw,normals;
  vector<Vector3> a,b;

  //for point to plan distances
  Matrix C(6,6);
  Vector d(6);
  Vector cn(6),twist(6);
  LDLDecomposition<Real> ldl;

  int numValidPoints = 0;
  for(size_t i=0;i<pc.points.size();i+=params.subsample) {
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    numValidPoints ++;
  }

  //distribute work evenly
  vector<FuseThreadJob> jobs(numThreads);
  int pointCounter = 0;
  for(size_t i=0;i<pc.points.size();i+=params.subsample) {
    const Vector3& p = pc.points[i];
    if(p.z == 0 || !IsFinite(p.z)) continue;
    int threadIndex = pointCounter * numThreads / numValidPoints;
    jobs[threadIndex].indices.push_back(i);
    pointCounter ++;
  }

  params.Tcamera = Tcamera;
  params.rmseDistance = 0;
  params.rmseColor = 0;  //ignored
  double correspondenceTime=0,outlierTime=0,fitTime=0;
  Timer timer;
  for(params.numIters=0;params.numIters<params.maxIters;params.numIters++) {
    timer.Reset();
    /*
    correspondences_raw.resize(0);
    distances_raw.resize(0);
    normals_raw.resize(0);
    colorDistances.resize(0);
    for(size_t i=0;i<pc.points.size();i+=params.subsample) {
      if(pc.points[i].z <= 0) continue;
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
    */
     //send out jobs
    for(int i=0;i<numThreads;i++) {
      if(DEBUG_THREADING) printf("Sending out JOB_CORRESPONDENCES to thread %d\n",i);
      jobs[i].type = JOB_CORRESPONDENCES;
      idata[i]->job.set(jobs[i]);
    }

    //wait for jobs to complete
    vector<bool> done(numThreads,false);
    for(int i=0;i<numThreads;i++) {
      int thread = finished.get();
      assert(done[thread] == false);
      done[thread] = true;
      if(DEBUG_THREADING)  printf("Thread %d done with JOB_CORRESPONDENCES\n",thread);
    } 

    //gather results
    correspondences_raw.resize(0);
    distances_raw.resize(0);
    normals_raw.resize(0);
    colorDistances.resize(0);
    for(int i=0;i<numThreads;i++) {
      correspondences_raw.insert(correspondences_raw.end(),idata[i]->correspondences_raw.begin(),idata[i]->correspondences_raw.end());
      distances_raw.insert(distances_raw.end(),idata[i]->distances_raw.begin(),idata[i]->distances_raw.end());
      normals_raw.insert(normals_raw.end(),idata[i]->normals_raw.begin(),idata[i]->normals_raw.end());
      colorDistances.insert(colorDistances.end(),idata[i]->colorDistances.begin(),idata[i]->colorDistances.end());
    }

    correspondenceTime += timer.ElapsedTime();
    timer.Reset();
    //if(DEBUG) printf("ICP Iteration %d: %d within distance\n",params.numIters,(int)correspondences_raw.size());

    //outlier rejection
    if(params.percentileOutliers == 0) {
      if(!params.pointToPlane) {
        swap(correspondences,correspondences_raw);
        swap(distances,distances_raw);
        swap(normals,normals_raw);
      }
    }
    else {
      vector<Real> scores(correspondences_raw.size());
      for(size_t i=0;i<correspondences_raw.size();i++) {
        scores[i] = Abs(distances_raw[i]);
        if(colored)
          scores[i] += colorDistances[i]*params.colorWeight;
      }
      //sort(sorter.begin(),sorter.end());
      size_t percentile = size_t((1.0-params.percentileOutliers)*scores.size());
      Real smin = *std::min_element(scores.begin(),scores.end());
      Real smax = *std::max_element(scores.begin(),scores.end());
      Real cutoff = ArrayUtils::nth_element(scores,percentile);
      Real dthresh = Min(cutoff,smax + params.percentileOutliers * (smin-smax));
      correspondences.resize(0);
      distances.resize(0);
      normals.resize(0);
      for(size_t i=0;i<correspondences_raw.size();i++) {
        if(scores[i] < dthresh) {
          correspondences.push_back(correspondences_raw[i]);
          distances.push_back(distances_raw[i]);
          normals.push_back(normals_raw[i]);
        }
      }
      //do the thresholding multithreaded
      for(int i=0;i<numThreads;i++) {
        if(DEBUG_THREADING) printf("Sending out JOB_THRESHOLD to thread %d\n",i);
        idata[i]->colorWeight = params.colorWeight;
        idata[i]->dthresh = dthresh;
        FuseThreadJob job;
        job.type = JOB_THRESHOLD;
        idata[i]->job.set(job);
      }
      //wait for jobs to complete
      vector<bool> done(numThreads,false);
      for(int i=0;i<numThreads;i++) {
        int thread = finished.get();
        assert(done[thread] == false);
        done[thread] = true;
        if(DEBUG_THREADING)  printf("Thread %d done with JOB_THRESHOLD\n",thread);
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

    outlierTime += timer.ElapsedTime();
    timer.Reset();

    if(correspondences.size() == 0) {
      params.rmseDistance = Inf;
      params.standardError.resize(6,Inf);
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
      //TODO: estimate standardError
      printf("Terminated due to convergence\n");
      printf("  Old RMSE %f, new RMSE %f\n",old_rmse,rmse_distance);
      Matrix Cinv;
      ldl.getInverse(Cinv);
      for(int i=0;i<6;i++)
        params.standardError(i) = Sqrt(Cinv(i,i));
      return;
    }

    //not enough correspondences
    if(correspondences.size() <= 6) {
      params.standardError.resize(6,Inf);
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
      for(int inner=0;inner<10;inner++) {
        C.setZero();
        d.setZero();
        /*
        Real sse = 0.0;
        for(size_t i=0;i<correspondences.size();i++) {
          const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
          Vector3 c;
          c.setCross(a,normals[i]);
          c.get(cn[0],cn[1],cn[2]);
          normals[i].get(cn[3],cn[4],cn[5]);
          Real dist = dot(a,normals[i]);
          for(int g=0;g<6;g++)
            for(int h=0;h<6;h++)
              C(g,h) += cn[g]*cn[h];
          d.madd(cn,-(dist-origOffsets[i]));
          sse += Sqr((dist-origOffsets[i]));
        }
        */
        //send out jobs
        for(int i=0;i<numThreads;i++) {
          if(DEBUG_THREADING) printf("Sending out JOB_COVARIANCE to thread %d\n",i);
          FuseThreadJob job;
          job.type = JOB_COVARIANCE;
          idata[i]->job.set(job);
        }

        //wait for jobs to complete
        vector<bool> done(numThreads,false);
        for(int i=0;i<numThreads;i++) {
          int thread = finished.get();
          assert(done[thread] == false);
          done[thread] = true;
          if(DEBUG_THREADING)  printf("Thread %d done with JOB_COVARIANCE\n",thread);
        } 
        for(int i=0;i<numThreads;i++) {
          C += idata[i]->C;
          d += idata[i]->d;
        }
        ldl.set(C);
        if(!ldl.backSub(d,twist)) {
          printf("    Terminated due to numerical error\n");
          Matrix Cinv;
          ldl.getInverse(Cinv);
          for(int i=0;i<6;i++)
            params.standardError(i) = Sqrt(Cinv(i,i));
          return;
        }
        if(twist.normSquared() < 1e-6)
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
        /*
        Real sse2 = 0.0;
        for(size_t i=0;i<correspondences.size();i++) {
          const Vector3& a=params.Tcamera*pc.points[correspondences[i]];
          Real dist = dot(a,normals[i]) - origOffsets[i];
          sse2 += Sqr(dist);
        }
        */
        //printf("    SSE changed from %g to %g\n",sse,sse2);
      }
        
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
        params.standardError.resize(6,Inf);
        return;
      }
    }
    fitTime += timer.ElapsedTime();
    timer.Reset();
  }
  printf("Timing for correspondences: %gs, oulier rejection %gs, fitting %gs\n",correspondenceTime,outlierTime,fitTime);
}

void SparseTSDFReconstruction::ExtractMesh(Meshing::TriMesh& mesh)
{
  AABB3D roi;
  roi.maximize();
  ExtractMesh(roi,mesh);
}

void SparseTSDFReconstruction::ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh)
{
  mesh.tris.resize(0);
  mesh.verts.resize(0);
  const static int m=8,n=8,p=8;
  float NaN = (float)truncationDistance;
  TriMesh tempMesh;
  Array3D<float> expandedBlock(m+1,n+1,p+1);
  for(const auto& b : tsdf.hash.buckets) {
    VolumeGridTemplate<float>& depth = reinterpret_cast<SparseVolumeGrid::Block*>(b.second)->grid.channels[0];
    if(!depth.bb.intersects(roi)) continue;
    AABB3D center_bb = depth.bb;
    Vector3 celldims = depth.GetCellSize();
    center_bb.bmin += celldims*0.5;
    center_bb.bmax += celldims*0.5; //one extra cell in each dimension

    //copy 8x8x8 block
    for(auto i=depth.value.begin(),j=expandedBlock.begin(Range3Indices(0,m,0,n,0,p));i!=depth.value.end();++i,++j) 
      *j = *i;

    //now work on the seams:
    IntTriple c;
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
  AABB3D roi;
  roi.maximize();
  ExtractMesh(roi,mesh,app);
}

void SparseTSDFReconstruction::ExtractMesh(const AABB3D& roi,Meshing::TriMesh& mesh,GLDraw::GeometryAppearance& app)
{
  //TODO: do this faster without repeated block lookup
  ExtractMesh(roi,mesh);
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
  const static float one_over_255 = 1.0f/255;
  float rgb = (float)tsdf.GetValue(point,rgbChannel);
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
  blocksize += sizeof(int)*4; //other fields in Block
  blocksize += sizeof(int)*4; //index in GridSubdivision
  return blocksize*tsdf.hash.buckets.size();
}
