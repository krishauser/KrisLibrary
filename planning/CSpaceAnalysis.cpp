#include <KrisLibrary/Logger.h>
#include "CSpaceAnalysis.h"
#include <math/matrix.h>
#include <math/LAPACKInterface.h>
#include "errors.h"
#include <algorithm>
#include <set>
using namespace std;

template <class T1,class T2>
void SortAuxiliary(std::vector<T1>& value,std::vector<T2>& aux)
{
  Assert(aux.size()==value.size());
  //sort (value,index) pairs in increasing value
  std::vector<std::pair<T1,size_t> > pairs(value.size());
  for(size_t i=0;i<pairs.size();i++) {
    pairs[i].first = value[i];
    pairs[i].second = i;
  }
  std::sort(pairs.begin(),pairs.end());
  std::vector<T2> temp(aux.size());
  for(size_t i=0;i<pairs.size();i++) {
    value[i] = pairs[i].first;
    temp[i] = aux[pairs[i].second];
  }
  std::swap(temp,aux);
}

//returns the nearest neighbors and distances in unsorted order
void KNearestNeighbors(const Config& x,const vector<Config>& pts,int k,CSpace* cspace,vector<int>& nn,vector<Real>& dist)
{
  Assert(k > 0);
  nn.resize(k);
  dist.resize(k);
  fill(nn.begin(),nn.end(),-1);
  fill(dist.begin(),dist.end(),Inf);
  int maxdist = 0;
  for(size_t i=0;i<pts.size();i++) {
    Real d = cspace->Distance(x,pts[i]);
    if(d < dist[maxdist]) {
      dist[maxdist] = d;
      nn[maxdist] = (int)i;

      for(int j=0;j<k;j++) { //what's the furthest now?
	if(dist[j] > dist[maxdist]) 
	  maxdist=j;
      }
    }
  }
}

void KNearestNeighbors(int query_index,const vector<Config>& pts,int k,CSpace* cspace,vector<int>& nn,vector<Real>& dist)
{
  Assert(k > 0);
  nn.resize(k);
  dist.resize(k);
  fill(nn.begin(),nn.end(),-1);
  fill(dist.begin(),dist.end(),Inf);
  int maxdist = 0;
  for(size_t i=0;i<pts.size();i++) {
    if((int)i == query_index) continue;
    Real d = cspace->Distance(pts[query_index],pts[i]);
    if(d < dist[maxdist]) {
      dist[maxdist] = d;
      nn[maxdist] = (int)i;

      for(int j=0;j<k;j++) { //what's the furthest now?
	if(dist[j] > dist[maxdist]) 
	  maxdist=j;
      }
    }
  }
}

void KGoodNeighbors(int query_index,const vector<Config>& pts,int k,CSpace* cspace,vector<int>& nn,vector<Real>& dist,Real factor)
{
  Assert(k > 0);
  nn.resize(k);
  dist.resize(k);
  fill(nn.begin(),nn.end(),-1);
  fill(dist.begin(),dist.end(),Inf);
  vector<Real> allDist(pts.size());
  vector<int> allPts(pts.size());
  for(size_t i=0;i<pts.size();i++) {
    allDist[i] = cspace->Distance(pts[query_index],pts[i]);
    allPts[i] = i;
  }
  SortAuxiliary(allDist,allPts);
  int maxdist = 0;
  //for(size_t i=0;i<pts.size();i++) {
  for(size_t m=0;m<pts.size();m++) {
    int i=allPts[m];
    if((int)i == query_index) continue;
    Real d=allDist[m];
    //Real d = cspace->Distance(pts[query_index],pts[i]);
    bool include = true;
    for(size_t n=0;n<nn.size();n++) {
      int j=nn[n];
      if(j < 0) continue;
      Real dij = cspace->Distance(pts[i],pts[j]);
      Real d0j = dist[n];
      Real cosAngle = Half*(Sqr(d0j) + Sqr(d) - Sqr(dij))/(d0j*d);
      if(cosAngle < 0) continue;  //more than 180 degrees
      //if(d > factor*(dij+d0j)) {
      Real y=Pow(cosAngle,d/d0j);
      //LOG4CXX_INFO(KrisLibrary::logger(),"Cos angle "<<cosAngle<<", distance factor "<<d/d0j<<", result "<<y);
      if(y > factor) {
	include = false;
	break;
      }
    }
    if(include) {
      dist[maxdist] = d;
      nn[maxdist] = (int)i;
      
      maxdist++;
      if(maxdist == k) return;
    }
  }
}

void AnalyzeSamples(CSpace* cspace,CSpaceAnalysis::SetCharacteristics& s)
{
  if(s.points.size() <= 1) {  //can't analyize
    s.intrinsicDims = 0;
    return;
  }

  //maximum likelihood of dimensions
  //source: see MacKay, Ghahramani's comments to "Maximum Likelihood Estimation of Intrinsic Dimension" by E. Levina and P. Bickel (2004)

  //average over the central point
  vector<Real> dist(s.points.size());
  for(size_t i=0;i<s.points.size();i++) 
    dist[i] = cspace->Distance(s.center,s.points[i]);
  std::sort(dist.begin(),dist.end());
  s.localScales.resize(dist.size());
  s.localIntrinsicDims.resize(dist.size());
  for(size_t i=1;i<dist.size();i++) {
    s.localScales[i-1] = dist[i];
    Real dimInverse=0;
    for(size_t j=0;j<i;j++) 
      dimInverse += Log(dist[i]/dist[j]);
    dimInverse /= Real(i-1);
    s.localIntrinsicDims[i-1] = 1.0/dimInverse;
  }

  if(dist.back() > s.sampleRadius) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, neighborhood sampler does not correspond with distance metric: "<<dist.back()<<" > "<<s.sampleRadius);
    Real dimCenterInverse=0;
    for(size_t i=0;i<s.points.size();i++) 
      dimCenterInverse += Log(dist.back()/dist[i]);
    dimCenterInverse /= Real(s.points.size());
    s.intrinsicDims = 1.0/dimCenterInverse;
    
    s.localScales.back() = s.sampleRadius;
    s.localIntrinsicDims.back() = s.intrinsicDims;
  }
  else {
    Real dimCenterInverse=0;
    for(size_t i=0;i<s.points.size();i++) 
      dimCenterInverse += Log(s.sampleRadius/dist[i]);
    dimCenterInverse /= Real(s.points.size());
    s.intrinsicDims = 1.0/dimCenterInverse;
    
    s.localScales.back() = s.sampleRadius;
    s.localIntrinsicDims.back() = s.intrinsicDims;
  }

  /*
THIS CODE ESTIMATES THE DIMENSION OF AN ENTIRE SET OF POINTS
  int k=std::min(5,(int)s.points.size());
  vector<int> knn;
  vector<Real> dist;

  //average over all points sampled
  Real dimInverse=0;
  for(size_t i=0;i<s.points.size();i++) {
    KNearestNeighbors(i,s.points,k,cspace,knn,dist);
    SortAuxiliary(dist,knn);

    //local estimate^-1 = 1/(k-1) * sum[j=1...k-1] log (dk/dj)
    //where dk is the distance from x(i) to it's kth nearest neighbor
    for(int j=0;j<k-1;j++) {
      dimInverse += Log(dist.back()/dist[j]);
    }
  }
  dimInverse /= Real(s.points.size()*(k-1));

  LOG4CXX_ERROR(KrisLibrary::logger(),"Intrinsic dims, centered: "<<1.0/dimCenterInverse<<", averaged: "<<1.0/dimInverse);
  s.intrinsicDims = 1.0/dimInverse;
  */

  //cube of dimension d, radius r has volume (2r)^d
  //ball of dimension d, radius r has volume pi^(d/2)r^d/gamma(d/2+1)
  //where gamma is the gamma function
  //so, coefficients are 2^d in case 1, pi^(d/2)/gamma(d/2+1) in case 2
  //
};

void CSpaceAnalysis::AnalyzeNeighborhood(CSpace* cspace,const Config& x,Real radius,int numSamples)
{
  space.points.resize(0);
  feasible.points.resize(0);
  infeasible.points.resize(0);
  space.center = feasible.center = infeasible.center = x;
  space.sampleRadius = feasible.sampleRadius = infeasible.sampleRadius = radius;

  vector<bool> isFeasible(numSamples);
  Config y;
  for(int i=0;i<numSamples;i++) {
    cspace->SampleNeighborhood(x,radius,y);
    //Real d=cspace->Distance(x,y);

    space.points.push_back(y);
    isFeasible[i] = cspace->IsFeasible(y);
    if(isFeasible[i]) {
      feasible.points.push_back(y);
    }
    else {
      infeasible.points.push_back(y);
    }
  }

  AnalyzeSamples(cspace,space);
  AnalyzeSamples(cspace,feasible);
  AnalyzeSamples(cspace,infeasible);
  feasibleVolume = Real(feasible.points.size()) / Real(space.points.size());

  if(feasible.points.size() <= 1) {
    feasibleEigenvector.resize(0);
    return;
  }

  //build the knn network
  int k = 4;
  Matrix laplacian(feasible.points.size(),feasible.points.size(),Zero);

  vector<int> spaceToFeasible(space.points.size(),-1);
  int nf=0;
  for(size_t i=0;i<space.points.size();i++) 
    if(isFeasible[i]) spaceToFeasible[i] = nf++;

  LOG4CXX_INFO(KrisLibrary::logger(),"Computing K-nearest neighbors, k="<<k);
  vector<int> knn;
  vector<Real> dist;
  for(size_t i=0;i<space.points.size();i++) {
    if(!isFeasible[i]) continue;
    //KNearestNeighbors(i,space.points,k,cspace,knn,dist);
    KGoodNeighbors(i,space.points,k,cspace,knn,dist,0.7);

    int iindex = spaceToFeasible[i];
    for(size_t j=0;j<knn.size();j++) {
      if(!isFeasible[knn[j]]) continue;
      int jindex = spaceToFeasible[knn[j]];
      laplacian(iindex,jindex) = 1.0/dist[j];
    }
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Done.");

  //make symmetric
  for(int i=0;i<laplacian.m;i++) {
    for(int j=0;j<i;j++) {
      Real avg = Half*(laplacian(i,j)+laplacian(j,i));
      laplacian(i,j) = laplacian(j,i) = avg;
    }
  }

  //now they form the weighted adjacency matrix, form the laplacian now
  for(int i=0;i<laplacian.m;i++) {
    Assert(laplacian(i,i) == Zero);
    Real sum=0;
    for(int j=0;j<laplacian.n;j++)
      sum += laplacian(i,j);
    for(int j=0;j<i;j++)
      laplacian(i,j) = -laplacian(i,j);
    laplacian(i,i) = sum;
    for(int j=i+1;j<laplacian.n;j++)
      laplacian(i,j) = -laplacian(i,j);
  }
  /*
  //scale by inverse sqrt(diagonal) (pre and postmultiply)
  for(int i=0;i<laplacian.m;i++) {
    if(FuzzyZero(laplacian(i,i),1e-4)) continue;
    Real scale = 1.0/Sqrt(laplacian(i,i));
    for(int j=0;j<laplacian.n;j++) {
      laplacian(i,j) *= scale;
      laplacian(j,i) *= scale;
    }
  }
*/
  Vector lambda;
  Matrix Q;
  LOG4CXX_INFO(KrisLibrary::logger(),"Computing eigenvectors...");
  bool res=LAPACKInterface::Eigenvectors_Symmetric(laplacian,lambda,Q);
  Assert(res == true);
  LOG4CXX_INFO(KrisLibrary::logger(),"Done");
  //LOG4CXX_INFO(KrisLibrary::logger(),"Eigenvalues: "<<lambda);
  Assert(FuzzyEquals(lambda(0),Zero,1e-4));
  int col = 1;
  //while(FuzzyEquals(lambda(col),1e-4))
  //col++;
  Vector partition1;
  Q.getColRef(col,partition1);

  //mean threshold
  Real threshold = Zero;
  for(int i=0;i<partition1.n;i++)
    threshold += partition1(i);
  threshold /= partition1.n;
  /*
  //median threshold
  feasibleEigenvector.resize(partition1.n);
  for(int i=0;i<partition1.n;i++)
    feasibleEigenvector[i] = partition1(i);
  sort(feasibleEigenvector.begin(),feasibleEigenvector.end());
  Real threshold = feasibleEigenvector[feasibleEigenvector.size()/2];
  */

  feasibleEigenvector.resize(partition1.n);
  for(int i=0;i<partition1.n;i++)
    feasibleEigenvector[i] = partition1(i)-threshold;
}
