#include "VolumeGrid.h"
using namespace std;

namespace Meshing {

void VolumeGrid::ResizeByResolution(const Vector3& res)
{
  Assert(res.x > 0 && res.y > 0 && res.z > 0);
  int m=(int)Ceil((bb.bmax.x-bb.bmin.x)/res.x);
  int n=(int)Ceil((bb.bmax.y-bb.bmin.y)/res.y);
  int p=(int)Ceil((bb.bmax.z-bb.bmin.z)/res.z);
  value.resize(m,n,p);
}

void VolumeGrid::MakeSimilar(const VolumeGrid& grid)
{
  bb=grid.bb;
  value.resize(grid.value.m,grid.value.n,grid.value.p);
}

bool VolumeGrid::IsSimilar(const VolumeGrid& grid) const
{
  return (grid.value.m == value.m && grid.value.n == value.n && grid.value.p == value.p) && (bb.bmin == grid.bb.bmin && bb.bmax == grid.bb.bmax);
}

Vector3 VolumeGrid::GetCellSize() const
{
  Vector3 size=bb.bmax-bb.bmin;
  size.x /= Real(value.m);
  size.y /= Real(value.n);
  size.z /= Real(value.p);
  return size;
}

void VolumeGrid::GetCell(int i,int j,int k,AABB3D& cell) const
{
  Real u=Real(i)/Real(value.m);
  Real v=Real(j)/Real(value.n);
  Real w=Real(k)/Real(value.p);
  cell.bmin.x = bb.bmin.x + u*(bb.bmax.x-bb.bmin.x);
  cell.bmin.y = bb.bmin.y + v*(bb.bmax.y-bb.bmin.y);
  cell.bmin.z = bb.bmin.z + w*(bb.bmax.z-bb.bmin.z);
  Real u2=Real(i+1)/Real(value.m);
  Real v2=Real(j+1)/Real(value.n);
  Real w2=Real(k+1)/Real(value.p);
  cell.bmax.x = bb.bmin.x + u2*(bb.bmax.x-bb.bmin.x);
  cell.bmax.y = bb.bmin.y + v2*(bb.bmax.y-bb.bmin.y);
  cell.bmax.z = bb.bmin.z + w2*(bb.bmax.z-bb.bmin.z);
}

void VolumeGrid::GetCellCenter(int i,int j,int k,Vector3& center) const
{
  Real u=(Real(i)+0.5)/Real(value.m);
  Real v=(Real(j)+0.5)/Real(value.n);
  Real w=(Real(k)+0.5)/Real(value.p);
  center.x = bb.bmin.x + u*(bb.bmax.x-bb.bmin.x);
  center.y = bb.bmin.y + v*(bb.bmax.y-bb.bmin.y);
  center.z = bb.bmin.z + w*(bb.bmax.z-bb.bmin.z);
}

void VolumeGrid::GetIndex(const Vector3& pt,int& i,int& j,int& k) const
{
  Real u=(pt.x - bb.bmin.x)/(bb.bmax.x-bb.bmin.x);
  Real v=(pt.y - bb.bmin.y)/(bb.bmax.y-bb.bmin.y);
  Real w=(pt.z - bb.bmin.z)/(bb.bmax.z-bb.bmin.z);

  i = (int)Floor(u*value.m);
  j = (int)Floor(v*value.n);
  k = (int)Floor(w*value.p);
}

void VolumeGrid::GetIndexAndParams(const Vector3& pt,IntTriple& index,Vector3& params) const
{
  Real u=(pt.x - bb.bmin.x)/(bb.bmax.x-bb.bmin.x)*value.m;
  Real v=(pt.y - bb.bmin.y)/(bb.bmax.y-bb.bmin.y)*value.n;
  Real w=(pt.z - bb.bmin.z)/(bb.bmax.z-bb.bmin.z)*value.p;

  Real ri = Floor(u);
  Real rj = Floor(v);
  Real rk = Floor(w);
  
  //set params u,v,w to their fractional component
  params.x = u - ri;
  params.y = v - rj;
  params.z = w - rk;
  //set the index to the integer component
  index.a = (int)ri;
  index.b = (int)rj;
  index.c = (int)rk;
}

void VolumeGrid::GetIndexRange(const AABB3D& range,IntTriple& imin,IntTriple& imax) const
{
  GetIndex(range.bmin,imin);
  GetIndex(range.bmax,imax);
}

Real VolumeGrid::TrilinearInterpolate(const Vector3& pt) const
{
  Real u=(pt.x - bb.bmin.x)/(bb.bmax.x-bb.bmin.x)*value.m;
  Real v=(pt.y - bb.bmin.y)/(bb.bmax.y-bb.bmin.y)*value.n;
  Real w=(pt.z - bb.bmin.z)/(bb.bmax.z-bb.bmin.z)*value.p;

  Real ri = Floor(u);
  Real rj = Floor(v);
  Real rk = Floor(w);
 
  //set u,v,w to their fractional component
  u = u - ri;
  v = v - rj;
  w = w - rk;

  //get the base cell index
  int i1=(int)ri;
  int j1=(int)rj;
  int k1=(int)rk;

  //get the alternate cell indices, interpolation parameters
  //(u interpolates between i1,i2, etc)
  int i2,j2,k2;
  if(u > 0.5) { i2=i1+1; u = u-0.5; }
  else { i2=i1; i1--; u = 0.5+u; }
  if(v > 0.5) { j2=j1+1; v = v-0.5; }
  else { j2=j1; j1--; v = 0.5+v; }
  if(w > 0.5) { k2=k1+1; w = w-0.5; }
  else { k2=k1; k1--; w = 0.5+w; }

  if(i1 < 0) i1=0; if(i1 >= value.m) i1=value.m-1;
  if(i2 < 0) i2=0; if(i2 >= value.m) i2=value.m-1;
  if(j1 < 0) j1=0; if(j1 >= value.n) j1=value.n-1;
  if(j2 < 0) j2=0; if(j2 >= value.n) j2=value.n-1;
  if(k1 < 0) k1=0; if(k1 >= value.p) k1=value.p-1;
  if(k2 < 0) k2=0; if(k2 >= value.p) k2=value.p-1;
  Real v11 = (1-w)*value(i1,j1,k1) + w*value(i1,j1,k2);
  Real v12 = (1-w)*value(i1,j2,k1) + w*value(i1,j2,k2);
  Real v21 = (1-w)*value(i2,j1,k1) + w*value(i2,j1,k2);
  Real v22 = (1-w)*value(i2,j2,k1) + w*value(i2,j2,k2);
  Real w1 = (1-v)*v11+v*v12;
  Real w2 = (1-v)*v21+v*v22;
  return (1-u)*w1 + u*w2;
}

Real VolumeGrid::MinimumFreeInterpolate(const Vector3& pt) const
{
  IntTriple i1,i2;
  Vector3 u;
  GetIndexAndParams(pt,i1,u);
  IntTriple size=value.size();

  //get the alternate cell indices, interpolation parameters
  for(int i=0;i<3;i++) {
    if(u[i] > 0.5) { i2[i]=i1[i]+1; u[i]-=0.5; }
    else { i2[i]=i1[i]; i1[i]--; u[i]+=0.5; }
    if(i1[i]<0) i1[i]=0;
    if(i1[i]>=size[i]) i1[i]=size[i]-1;
    if(i2[i]<0) i2[i]=0;
    if(i2[i]>=size[i]) i2[i]=size[i]-1;
  }  

  //pick a minimal center value
  Real v111=value(i1);
  Real v112=value(i1.a,i1.b,i2.c);
  Real v121=value(i1.a,i2.b,i1.c);
  Real v122=value(i1.a,i2.b,i2.c);
  Real v211=value(i2.a,i1.b,i1.c);
  Real v212=value(i2.a,i1.b,i2.c);
  Real v221=value(i2.a,i2.b,i1.c);
  Real v222=value(i2);
  //minimum center value is on the diagonals
  Real val;
  Real centerValue = 0.5*(v111+v222);
  val = 0.5*(v211+v122);
  if(val < centerValue) centerValue = val;
  val = 0.5*(v121+v212);
  if(val < centerValue) centerValue = val;
  val = 0.5*(v112+v221);
  if(val < centerValue) centerValue = val;
  //decompose cube into pyramids
  Real mainAxisParam = ::Max(::Max(Abs(u.x-0.5),Abs(u.y-0.5)),Abs(u.z-0.5));
  int mainAxis=0;
  for(int i=1;i<3;i++) 
    if(Abs(u[i]-0.5) == mainAxisParam) 
      mainAxis=i; 
  mainAxisParam = u[mainAxis];
  Real faceCenterValue;
  //minimum face-center value is on diagonal of given face
  if(mainAxis == 0) {
    if(mainAxisParam < 0.5) {  //-x
      faceCenterValue = ::Min(0.5*(v111+v122),0.5*(v112+v121));
    }
    else {  //+x
      faceCenterValue = ::Min(0.5*(v211+v222),0.5*(v212+v221));
    }
  }
  else if(mainAxis == 1) {
    if(mainAxisParam < 0.5) {  //-y
      faceCenterValue = ::Min(0.5*(v111+v212),0.5*(v112+v211));
    }
    else {  //+y
      faceCenterValue = ::Min(0.5*(v121+v222),0.5*(v122+v221));
    }
  }
  else {
    if(mainAxisParam < 0.5) {  //-z
      faceCenterValue = ::Min(0.5*(v111+v221),0.5*(v121+v211));
    }
    else {  //+z
      faceCenterValue = ::Min(0.5*(v112+v222),0.5*(v122+v212));
    }
  }
  //decompose pyramid into tetrahedra
  int secondaryAxis=(mainAxis+1)%3;
  int tertiaryAxis=(mainAxis+2)%3;
  if(Abs(u[secondaryAxis]-0.5) < Abs(u[tertiaryAxis]-0.5))
    swap(secondaryAxis,tertiaryAxis);
  Real ve1,ve2;  //values at endpoints at tertiary edge
  IntTriple ie1,ie2;  //endpoints at tertiary edge
  ie1[mainAxis] = ie2[mainAxis] = (u[mainAxis] < 0.5 ? i1[mainAxis] : i2[mainAxis]);
  ie1[secondaryAxis] = ie2[secondaryAxis] = (u[secondaryAxis] < 0.5 ? i1[secondaryAxis] : i2[secondaryAxis]);
  ie1[tertiaryAxis] = i1[tertiaryAxis];
  ie2[tertiaryAxis] = i2[tertiaryAxis];
  Assert(ie1.a >= 0 && ie1.a < value.m);
  Assert(ie1.b >= 0 && ie1.b < value.n);
  Assert(ie1.c >= 0 && ie1.c < value.p);
  Assert(ie2.a >= 0 && ie2.a < value.m);
  Assert(ie2.b >= 0 && ie2.b < value.n);
  Assert(ie2.c >= 0 && ie2.c < value.p);
  ve1 = value(ie1);
  ve2 = value(ie2);
  //interpolate by simplex
  Real bary[4];
  Real up = 0.5-Abs(u[mainAxis]-0.5);
  Real back = 0.5-Abs(u[secondaryAxis]-0.5);
  Real across = u[tertiaryAxis];
  bary[0] = 2.0*up;
  bary[1] = -2.0*up + 2.0*back;
  bary[2] = 1.0-across-back;
  bary[3] = across-back;
  for(int i=0;i<4;i++)
    Assert(-Epsilon <=bary[i] && bary[i]<=1+Epsilon );
  Assert(FuzzyEquals(bary[0]+bary[1]+bary[2]+bary[3],One));
  return bary[0]*centerValue + bary[1]*faceCenterValue + bary[2]*ve1 + bary[3]*ve2;
  /*
  Real vmin = ::Min(::Min(::Min(v111,v112),::Min(v121,v122)),
		    ::Min(::Min(v211,v212),::Min(v221,v222)));
  Real vmax = ::Max(::Max(::Max(v111,v112),::Max(v121,v122)),
		    ::Max(::Max(v211,v212),::Max(v221,v222)));
  if(val < vmin-Epsilon || val > vmax+Epsilon) {
    cerr<<"Error in MinimumFreeInterpolate!"<<endl;
    cerr<<"Value "<<val<<" out of bounds "<<vmin<<", "<<vmax<<endl;
    cerr<<"Params "<<u<<endl;
    cerr<<"corners "<<v111<<" "<<v112<<" "<<v121<<" "<<v122<<endl;
    cerr<<"        "<<v211<<" "<<v212<<" "<<v221<<" "<<v222<<endl;
    cerr<<"barycentric coordinates "<<bary[0]<<" "<<bary[1]<<" "<<bary[2]<<" "<<bary[3]<<endl;
    cerr<<"values "<<centerValue<<" "<<faceCenterValue<<" "<<ve1<<" "<<ve2<<endl;
    cerr<<"Tetrahedron Params "<<up<<" "<<back<<" "<<across<<endl;
    getchar();
  }
  Assert(centerValue >= vmin-Epsilon && centerValue <= vmax+Epsilon);
  Assert(faceCenterValue >= vmin-Epsilon && faceCenterValue <= vmax+Epsilon);
  //Assert(val >= vmin && val <= vmax);
  return val;
  */
}

Real VolumeGrid::Average(const AABB3D& range) const
{
  IntTriple imin,imax;
  GetIndexRange(range,imin,imax);
  //check range
  if(imax.a < 0 || imax.b < 0 || imax.c < 0) {
    return 0;
  }
  if(imin.a >= value.m || imin.b >= value.n || imax.c >= value.p) {
    return 0;
  }
  //limit range
  if(imin.a < 0) imin.a=0;
  if(imin.b < 0) imin.b=0;
  if(imin.c < 0) imin.c=0;
  if(imax.a >= value.m) imax.a=value.m-1;
  if(imax.b >= value.n) imax.b=value.n-1;
  if(imax.c >= value.p) imax.c=value.p-1;

  bool ignoreX=(range.bmin.x==range.bmax.x),ignoreY=(range.bmin.y==range.bmax.y),ignoreZ=(range.bmin.z==range.bmax.z);

  Vector3 cellcorner;
  Vector3 cellsize;
  cellsize.x = (bb.bmax.x-bb.bmin.x)/Real(value.m);
  cellsize.y = (bb.bmax.y-bb.bmin.y)/Real(value.n);
  cellsize.z = (bb.bmax.z-bb.bmin.z)/Real(value.p);
  Real sumValue=0;
  Real sumVolume=0;
  cellcorner.x = bb.bmin.x + imin.a*cellsize.x;
  for(int i=imin.a;i<=imax.a;i++,cellcorner.x += cellsize.x) {
    cellcorner.y = bb.bmin.y + imin.b*cellsize.y;
    for(int j=imin.b;j<=imax.b;j++,cellcorner.y += cellsize.y) {
      cellcorner.z = bb.bmin.z + imin.c*cellsize.z;
      for(int k=imin.c;k<=imax.c;k++,cellcorner.z += cellsize.z) {
	AABB3D intersect;
	intersect.bmin=cellcorner;
	intersect.bmax=cellcorner+cellsize;
	intersect.setIntersection(range);
	Vector3 isectsize=intersect.bmax-intersect.bmin;
	//due to rounding errors, may have negative sizes
	if(isectsize.x < 0 || isectsize.y < 0 || isectsize.z < 0) continue;
	Real volume=1;
	if(!ignoreX) volume*=isectsize.x;
	if(!ignoreY) volume*=isectsize.y;
	if(!ignoreZ) volume*=isectsize.z;
	sumValue += volume*value(i,j,k);
	sumVolume += volume;
      }
    }
  }
  Vector3 rangesize=range.bmax-range.bmin;
  Real rangeVolume = 1;
  if(!ignoreX) rangeVolume*=rangesize.x;
  if(!ignoreY) rangeVolume*=rangesize.y;
  if(!ignoreZ) rangeVolume*=rangesize.z;
  Assert(sumVolume < rangeVolume + Epsilon);
  return sumValue / rangeVolume;
}

void VolumeGrid::Resample(const VolumeGrid& grid)
{
  if(IsSimilar(grid)) {
    value = grid.value;
    return;
  }

  AABB3D cell;
  for(iterator it=getIterator();!it.isDone();++it) {
    it.getCell(cell);
    *it = grid.Average(cell);
  }
  /*
  for(int i=0;i<value.m;i++) {
    for(int j=0;j<value.n;j++) {
      for(int k=0;k<value.p;k++) {
	GetCell(i,j,k,cell);
	value(i,j,k) = grid.Average(cell);
      }
    }
  }
  */
}

void VolumeGrid::Add(const VolumeGrid& grid)
{
  if(!IsSimilar(grid)) {
    VolumeGrid resample;
    resample.value.resize(value.m,value.n,value.p);
    resample.bb = bb;
    resample.Resample(grid);
    Add(resample);
    return;
  }

  for(Array3D<Real>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i += *j;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) += grid.value(i,j,k);
  */
}

void VolumeGrid::Subtract(const VolumeGrid& grid)
{
  if(!IsSimilar(grid)) {
    VolumeGrid resample;
    resample.value.resize(value.m,value.n,value.p);
    resample.bb = bb;
    resample.Resample(grid);
    Subtract(resample);
    return;
  }

  for(Array3D<Real>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i -= *j;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) -= grid.value(i,j,k);
  */
}

void VolumeGrid::Multiply(const VolumeGrid& grid)
{
  if(!IsSimilar(grid)) {
    VolumeGrid resample;
    resample.value.resize(value.m,value.n,value.p);
    resample.bb = bb;
    resample.Resample(grid);
    Multiply(resample);
    return;
  }

  for(Array3D<Real>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i *= *j;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) *= grid.value(i,j,k);
  */
}

void VolumeGrid::Max(const VolumeGrid& grid)
{
  if(!IsSimilar(grid)) {
    VolumeGrid resample;
    resample.value.resize(value.m,value.n,value.p);
    resample.bb = bb;
    resample.Resample(grid);
    Max(resample);
    return;
  }

  for(Array3D<Real>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i = ::Max(*i,*j);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) = ::Max(value(i,j,k),grid.value(i,j,k));
  */
}

void VolumeGrid::Min(const VolumeGrid& grid)
{
  if(!IsSimilar(grid)) {
    VolumeGrid resample;
    resample.value.resize(value.m,value.n,value.p);
    resample.bb = bb;
    resample.Resample(grid);
    Min(resample);
    return;
  }

  for(Array3D<Real>::iterator i=value.begin(),j=grid.value.begin();i!=value.end();++i,++j) {
    *i = ::Min(*i,*j);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) = ::Min(value(i,j,k),grid.value(i,j,k));
  */
}

void VolumeGrid::Add(Real val)
{
  for(Array3D<Real>::iterator i=value.begin();i!=value.end();++i) {
    *i += val;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) += val;
  */
}

void VolumeGrid::Multiply(Real val)
{
  for(Array3D<Real>::iterator i=value.begin();i!=value.end();++i) {
    *i *= val;
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) *= val;
  */
}

void VolumeGrid::Max(Real val)
{
  for(Array3D<Real>::iterator i=value.begin();i!=value.end();++i) {
    *i = ::Max(*i,val);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) = ::Max(value(i,j,k),val);
  */
}

void VolumeGrid::Min(Real val)
{
  for(Array3D<Real>::iterator i=value.begin();i!=value.end();++i) {
    *i = ::Min(*i,val);
  }
  /*
  for(int i=0;i<value.m;i++)
    for(int j=0;j<value.n;j++)
      for(int k=0;k<value.p;k++)
	value(i,j,k) = ::Min(value(i,j,k),val);
  */
}




template <class T>
istream& operator >> (istream& in,Array3D<T>& a)
{
  int m,n,p;
  in>>m>>n>>p;
  if(!in) return in;
  Assert(m >= 0 && n >= 0 && p >= 0);
  a.resize(m,n,p);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      for(int k=0;k<p;k++)
	in>>a(i,j,k);
  return in;
}

template <class T>
ostream& operator << (ostream& out,const Array3D<T>& a)
{
  out<<a.m<<" "<<a.n<<" "<<" "<<a.p<<endl;
  for(int i=0;i<a.m;i++) {
    for(int j=0;j<a.n;j++) {
      for(int k=0;k<a.p;k++)
	out<<a(i,j,k)<<" ";
      out<<endl;
    }
  }
  return out;
}

istream& operator >> (istream& in,VolumeGrid& grid)
{
  in>>grid.bb.bmin>>grid.bb.bmax;
  in>>grid.value;
  return in;
}

ostream& operator << (ostream& out,const VolumeGrid& grid)
{
  out<<grid.bb.bmin<<"    "<<grid.bb.bmax<<endl;
  out<<grid.value<<endl;
  return out;
}


} //namespace Meshing
