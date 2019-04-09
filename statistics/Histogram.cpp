#include <KrisLibrary/Logger.h>
#include "Histogram.h"
#include "Histogram2D.h"
#include "Histogram3D.h"
#include <math3d/primitives.h>
#include <algorithm>
using namespace Statistics;
using namespace Math3D;
using namespace std;

Histogram::Histogram()
{
  Clear();
}

void Histogram::Clear()
{
  divs.clear();
  buckets.resize(1); //infinite
}

void Histogram::Resize(Real splitVal)
{
  divs.resize(1);
  buckets.resize(2);
  divs[0] = splitVal;
}

void Histogram::Resize(size_t n,Real a,Real b)
{
  if(n == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, resizing histogram to have 0 divisions -- was this intended?");
    Clear();
  }
  else {
    divs.resize(n+1);
    buckets.resize(n+2);
    Real x=a, h=(b-a)/n;
    for(size_t i=0;i<=n;i++) {
      divs[i] = x;
      x+=h;
    }
  }
}

void Histogram::ResizeToFit(const vector<Real>& data,size_t n)
{
  Real min=Inf,max=-Inf;
  for(size_t i=0;i<data.size();i++) {
    min = Min(min,data[i]);
    max = Max(max,data[i]);
  }
  if(min == max) max += 1;
  Resize(n,min,max);
}

void Histogram::Fill(Real val)
{
  fill(buckets.begin(),buckets.end(),val);
}

void Histogram::Calculate(const vector<Real>& data)
{
  assert(divs.size()+1 == buckets.size());
  fill(buckets.begin(),buckets.end(),0);
  for(size_t i=0;i<data.size();i++) {
    int b = GetBucket(data[i]);
    buckets[b]++;
  }
}

void Histogram::GetRange(int bucket,Real& min,Real& max) const
{
  assert(divs.size()+1 == buckets.size());
  assert(bucket >= 0 && bucket < (int)buckets.size());
  if(bucket == 0) min=-Inf; 
  else min = divs[bucket-1];
  if(bucket+1 == (int)buckets.size()) max=Inf;
  else max = divs[bucket];
}

int Histogram::GetBucket(Real val) const
{
  if(buckets.empty()) return -1;
  if(val < divs.front()) return 0;
  else if(val >= divs.back()) return buckets.size()-1;
  else {
    vector<Real>::const_iterator it = upper_bound(divs.begin(),divs.end(),val);
    //*lower_bound < val, *(lower_bound++) >= val
    //val >= *upper_bound, val < *(upper_bound++)
    int bucket=(it-divs.begin());
    Real a,b;
    GetRange(bucket,a,b);
    assert(a <= val && val < b);
    return bucket;
  }
}
void Histogram::AddBucket(Real val,Real num)
{
  int b=GetBucket(val);
  buckets[b] += num;
}

Real Histogram::NumObservations() const
{
  Real sum=0;
  for(size_t i=0;i<buckets.size();i++) sum+=buckets[i];
  return sum;
}



Histogram2D::Histogram2D()
{
  Clear();
}

void Histogram2D::Clear()
{
  div1.clear();
  div2.clear();
  buckets.resize(1,1);
}

void Histogram2D::Resize(size_t m,size_t n,Real a1,Real b1,Real a2,Real b2)
{
  if(m == 0 || n == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, resizing histogram to have 0 divisions -- was this intended?");
    Clear();
  }
  else {
    div1.resize(m+1);
    div2.resize(n+1);
    buckets.resize(m+2,n+2);
    Real x=a1, dx=(b1-a1)/m;
    for(size_t i=0;i<=m;i++) {
      div1[i] = x;
      x+=dx;
    }  
    Real y=a2, dy=(b2-a2)/n;
    for(size_t i=0;i<=n;i++) {
      div2[i] = y;
      y+=dy;
    }
  }
}

void Histogram2D::Resize(const Size dims,const Point min,const Point max)
{
  Resize(dims[0],dims[1],min[0],max[0],min[1],max[1]);
}

void Histogram2D::ResizeToFit(const std::vector<Real>& data1,const std::vector<Real>& data2,size_t m,size_t n)
{
  Real a1=Inf,a2=Inf,b1=-Inf,b2=-Inf;
  for(size_t i=0;i<data1.size();i++) {
    a1 = min(a1,data1[i]);
    b1 = max(b1,data1[i]);
  }
  for(size_t i=0;i<data2.size();i++) {
    a2 = min(a2,data2[i]);
    b2 = max(b2,data2[i]);
  }
  if(a1 == b1) b1 += 1;
  if(a2 == b2) b2 += 1;
  Resize(m,n,a1,b1,a2,b2);
}

void Histogram2D::ResizeToFit(const std::vector<Point>& data,const Size dims)
{
  Point bmin,bmax;
  bmin[0]=bmin[1]=Inf;
  bmax[0]=bmax[1]=-Inf;
  for(size_t i=0;i<data.size();i++) {
    for(int k=0;k<2;k++) {
      bmin[k] = min(bmin[k],data[i][k]);
      bmax[k] = max(bmax[k],data[i][k]);
    }
  }
  for(int k=0;k<2;k++)
    if(bmin[k]==bmax[k]) bmax[k] += 1;
  Resize(dims,bmin,bmax);
}

void Histogram2D::Fill(Real val)
{
  buckets.set(val);
}

void Histogram2D::Calculate(const std::vector<Real>& data1,const std::vector<Real>& data2)
{
  assert(div1.size()+1 == (size_t)buckets.m);
  assert(div2.size()+1 == (size_t)buckets.n);
  assert(data1.size()==data2.size());
  Fill(0);
  for(size_t i=0;i<data1.size();i++) {
    AddBucket(data1[i],data2[i],1);
  }
}

void Histogram2D::Calculate(const std::vector<Point>& data)
{
  assert(div1.size()+1 == (size_t)buckets.m);
  assert(div2.size()+1 == (size_t)buckets.n);
  Fill(0);
  for(size_t i=0;i<data.size();i++) {
    AddBucket(data[i][0],data[i][1],1);
  }
}

void Histogram2D::GetRange(const Index bucket,Point min,Point max) const
{
  assert(div1.size()+1 == (size_t)buckets.m);
  assert(div2.size()+1 == (size_t)buckets.n);
  assert(bucket[0] >= 0 && bucket[0] < buckets.m);
  assert(bucket[1] >= 0 && bucket[1] < buckets.n);
  min[0]=(bucket[0] == 0? -Inf: div1[bucket[0]-1]);
  max[0]=(bucket[0]+1 == buckets.m? Inf : div1[bucket[0]]);
  min[1]=(bucket[1] == 0? -Inf: div2[bucket[1]-1]);
  max[1]=(bucket[1]+1 == buckets.n? Inf : div2[bucket[1]]);
}

void Histogram2D::GetBucket(const Point val,Index bucket) const
{
  GetBucket(val[0],val[1],bucket);
}

void Histogram2D::GetBucket(Real v1,Real v2,Index bucket) const
{
  if(buckets.empty()) { bucket[0]=bucket[1]=-1; return; }
  if(v1 < div1.front()) bucket[0]=0;
  else if(v1 >= div1.back()) bucket[0]=buckets.m-1;
  else {
    vector<Real>::const_iterator it = upper_bound(div1.begin(),div1.end(),v1);
    bucket[0]=(it-div1.begin());
  }
  if(v2 < div2.front()) bucket[1]=0;
  else if(v2 >= div2.back()) bucket[1]=buckets.n-1;
  else {
    vector<Real>::const_iterator it = upper_bound(div2.begin(),div2.end(),v2);
    bucket[1]=(it-div2.begin());
  }
}

void Histogram2D::AddBucket(Real v1,Real v2,Real num)
{
  Index i;
  GetBucket(v1,v2,i);
  buckets(i[0],i[1]) += num;
}

Real Histogram2D::NumObservations() const
{
  Real sum=0;
  for(int i=0;i<buckets.m;i++)
    for(int j=0;j<buckets.n;j++)
      sum += buckets(i,j);
  return sum;
}




Histogram3D::Histogram3D()
{
  Clear();
}

void Histogram3D::Clear()
{
  div1.clear();
  div2.clear();
  div3.clear();
  buckets.resize(1,1,1);
}

void Histogram3D::Resize(const Size dims,const Point min,const Point max)
{
  if(dims[0] == 0 || dims[1] == 0 || dims[2] == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, resizing histogram to have 0 divisions -- was this intended?");
    Clear();
  }
  else {
    size_t m=dims[0],n=dims[1],p=dims[2];
    div1.resize(m+1);
    div2.resize(n+1);
    div3.resize(p+1);
    buckets.resize(m+2,n+2,p+2);

    Real x=min[0], dx=(max[0]-min[0])/m;
    for(size_t i=0;i<=m;i++) {
      div1[i] = x;
      x+=dx;
    }  
    Real y=min[1], dy=(max[1]-min[1])/n;
    for(size_t i=0;i<=n;i++) {
      div2[i] = y;
      y+=dy;
    }
    Real z=min[2], dz=(max[2]-min[2])/p;
    for(size_t i=0;i<=p;i++) {
      div3[i] = z;
      z+=dz;
    }
  }
}

void Histogram3D::ResizeToFit(const std::vector<Point>& data,const Size dims)
{
  Point bmin,bmax;
  bmin[0]=bmin[1]=bmin[2]=Inf;
  bmax[0]=bmax[1]=bmax[2]=-Inf;
  for(size_t i=0;i<data.size();i++) {
    for(int k=0;k<3;k++) {
      bmin[k] = min(bmin[k],data[i][k]);
      bmax[k] = max(bmax[k],data[i][k]);
    }
  }
  for(int k=0;k<3;k++)
    if(bmin[k]==bmax[k]) bmax[k] += 1;
  Resize(dims,bmin,bmax);
}

void Histogram3D::Fill(Real val)
{
  buckets.set(val);
}

void Histogram3D::Calculate(const std::vector<Point>& data)
{
  assert(div1.size()+1 == (size_t)buckets.m);
  assert(div2.size()+1 == (size_t)buckets.n);
  assert(div3.size()+1 == (size_t)buckets.p);
  Fill(0);
  for(size_t i=0;i<data.size();i++) {
    AddBucket(data[i],1);
  }
}

void Histogram3D::Calculate(const std::vector<Real>& data1,const std::vector<Real>& data2,const std::vector<Real>& data3)
{
  assert(div1.size()+1 == (size_t)buckets.m);
  assert(div2.size()+1 == (size_t)buckets.n);
  assert(div3.size()+1 == (size_t)buckets.p);
  assert(data1.size()==data2.size());
  assert(data1.size()==data3.size());
  Fill(0);
  Point p;
  for(size_t i=0;i<data1.size();i++) {
    p[0]=data1[i];
    p[1]=data2[i];
    p[2]=data3[i];
    AddBucket(p,1);
  }
}

void Histogram3D::GetRange(const Index bucket,Point min,Point max) const
{
  assert(div1.size()+1 == (size_t)buckets.m);
  assert(div2.size()+1 == (size_t)buckets.n);
  assert(div3.size()+1 == (size_t)buckets.p);
  assert(bucket[0] >= 0 && bucket[0] < buckets.m);
  assert(bucket[1] >= 0 && bucket[1] < buckets.n);
  assert(bucket[2] >= 0 && bucket[2] < buckets.p);
  min[0]=(bucket[0] == 0? -Inf: div1[bucket[0]-1]);
  max[0]=(bucket[0]+1 == buckets.m? Inf : div1[bucket[0]]);
  min[1]=(bucket[1] == 0? -Inf: div2[bucket[1]-1]);
  max[1]=(bucket[1]+1 == buckets.n? Inf : div2[bucket[1]]);
  min[2]=(bucket[2] == 0? -Inf: div3[bucket[2]-1]);
  max[2]=(bucket[2]+1 == buckets.n? Inf : div3[bucket[2]]);
}

void Histogram3D::GetBucket(const Point val,Index index) const
{
  if(buckets.empty()) {
    index[0]=index[1]=index[2]=-1;
    return;
  }
  if(val[0] < div1.front()) index[0]=0;
  else if(val[0] >= div1.back()) index[0]=buckets.m-1;
  else {
    vector<Real>::const_iterator it = std::upper_bound(div1.begin(),div1.end(),val[0]);
    index[0]=(it-div1.begin());
  }
  if(val[1] < div2.front()) index[1]=0;
  else if(val[1] >= div2.back()) index[1]=buckets.n-1;
  else {
    vector<Real>::const_iterator it = std::upper_bound(div2.begin(),div2.end(),val[1]);
    index[1]=(it-div2.begin());
  }
  if(val[2] < div3.front()) index[2]=0;
  else if(val[2] >= div3.back()) index[2]=buckets.p-1;
  else {
    vector<Real>::const_iterator it = upper_bound(div3.begin(),div3.end(),val[2]);
    index[2]=(it-div3.begin());
  }
}

void Histogram3D::AddBucket(const Point p,Real num)
{
  Index i;
  GetBucket(p,i);
  buckets(i[0],i[1],i[2]) += num;
}

void Histogram3D::AddBucket(Real v1,Real v2,Real v3,Real num)
{
  Point temp={v1,v2,v3};
  AddBucket(temp,num);
}

Real Histogram3D::NumObservations() const
{
  Real sum=0;
  for(int i=0;i<buckets.m;i++)
    for(int j=0;j<buckets.n;j++)
      for(int k=0;k<buckets.p;k++)
	sum += buckets(i,j,k);
  return sum;
}

