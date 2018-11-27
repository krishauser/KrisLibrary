#include <KrisLibrary/Logger.h>
#include "GaussianMixtureModel.h"
#include "statistics.h"
#include <math/matrix.h>
#include <math/LDL.h>
#include <math/SVDecomposition.h>
#include <math/sample.h>
#include <math/indexing.h>
#include <iostream>
#include <fstream>
using namespace std;

namespace Statistics {

  const static Real covarianceRegularizationFactor = 1e-3;
  const static Real p_outlier = 0.00001;

///Given input of nonnegative p's, perform normalization in a numerically
///stable way. Returns the sum of the p's.
///Specifically, if all p's are 0, then it returns a uniform distribution.
///If any p is infinite, then all non-infinite p's are set to 0 while
///the remaining weight is distributed evenly among all infinite p's
Real Normalize(vector<Real>& w)
{
  Real sumw = 0.0;
  for(size_t i=0;i<w.size();i++) sumw += w[i];
  if(sumw == 0.0) {
    fill(w.begin(),w.end(),1.0/w.size());
    return sumw;
  }
  else if(IsInf(sumw)) {
    assert(IsInf(sumw)==1);
    int numinf = 0;
    for(size_t i=0;i<w.size();i++)
      if(IsInf(sumw)==1) numinf ++;
    for(size_t i=0;i<w.size();i++)
      if(IsInf(sumw)==1) w[i] = 1.0/Real(numinf);
      else w[i] = 0.0;
    return sumw;
  }
  else {
    if(Abs(sumw) < 1e-5)
      for(size_t i=0;i<w.size();i++) w[i]/=sumw;
    else {
      Real scale=1.0/sumw;
      for(size_t i=0;i<w.size();i++) w[i]*=scale;
    }
    return sumw;
  }
}

Real NormalizeProbability(Vector& w)
{
  Real sumw = 0.0;
  for(int i=0;i<w.size();i++) sumw += w[i];
  if(sumw == 0.0) {
    fill(w.begin(),w.end(),1.0/w.size());
    return sumw;
  }
  else if(IsInf(sumw)) {
    assert(IsInf(sumw)==1);
    int numinf = 0;
    for(int i=0;i<w.size();i++)
      if(IsInf(sumw)==1) numinf ++;
    for(int i=0;i<w.size();i++)
      if(IsInf(sumw)==1) w[i] = 1.0/Real(numinf);
      else w[i] = 0.0;
    return sumw;
  }
  else {
    if(Abs(sumw) < 1e-5)
      for(int i=0;i<w.size();i++) w[i]/=sumw;
    else {
      Real scale=1.0/sumw;
      for(int i=0;i<w.size();i++) w[i]*=scale;
    }
    return sumw;
  }
}


  ///Given input of log(p)'s, perform exponentiation and normalization
  ///in a numerically stable way.  Return the sum probability of the
  ///p[i]'s
  Real ExpNormalize(vector<Real>& p)
  {
    //perform conditioning
    Real max_p = -Inf;
    for(size_t j=0;j<p.size();j++) max_p=Max(max_p,p[j]);
    if(!IsFinite(max_p)) {
      fill(p.begin(),p.end(),1.0/p.size());
      return 0.0;
    }
    else {
      for(size_t j=0;j<p.size();j++)
	p[j] = Exp(p[j]-max_p);
      
      //re-normalize
      Real sum_p = 0.0;
      for(size_t j=0;j<p.size();j++) sum_p += p[j];
      assert(sum_p != 0.0);
      if(Abs(sum_p) < 1e-5) 
	for(size_t j=0;j<p.size();j++) p[j] /= sum_p;
      else {
	Real scale = 1.0/sum_p;
	for(size_t j=0;j<p.size();j++) p[j] *= scale;
      }
      return sum_p;
    }
  }

  Real ExpNormalize(Vector& p)
  {
    //perform conditioning
    Real max_p = -Inf;
    for(int j=0;j<p.size();j++) max_p=Max(max_p,p[j]);
    if(!IsFinite(max_p)) {
      fill(p.begin(),p.end(),1.0/p.size());
      return 0.0;
    }
    else {
      for(int j=0;j<p.size();j++)
	p[j] = Exp(p[j]-max_p);
      
      //re-normalize
      Real sum_p = 0.0;
      for(int j=0;j<p.size();j++) sum_p += p[j];
      assert(sum_p != 0.0);
      if(Abs(sum_p) < 1e-5) 
	for(int j=0;j<p.size();j++) p[j] /= sum_p;
      else {
	Real scale = 1.0/sum_p;
	for(int j=0;j<p.size();j++) p[j] *= scale;
      }
      return sum_p;
    }
  }


GaussianMixtureModel::GaussianMixtureModel()
{}

GaussianMixtureModel::GaussianMixtureModel(int k,int d)
{
  Resize(k,d);
}

GaussianMixtureModel::GaussianMixtureModel(const GaussianMixtureModel& m)
{
  gaussians = m.gaussians;
  phi = m.phi;
}

int GaussianMixtureModel::NumDims() const 
{ 
  if(gaussians.empty()) return 0;
  return gaussians[0].numDims();
}

void GaussianMixtureModel::Resize(int k,int d)
{
  gaussians.resize(k);
  phi.resize(k,One/Real(k));
  for(size_t i=0;i<gaussians.size();i++) gaussians[i].resize(d);
}

void GaussianMixtureModel::SetLinearTransform(const GaussianMixtureModel& gmm,const Matrix& A,const Vector& b)
{
  if(this == &gmm) {
    GaussianMixtureModel temp=gmm;
    SetLinearTransform(temp,A,b);
    return;
  }
  phi = gmm.phi;
  gaussians.resize(gmm.gaussians.size());
  for(size_t i=0;i<gaussians.size();i++) {
    A.mul(gmm.gaussians[i].mu,gaussians[i].mu);
    gaussians[i].mu += b;
    gaussians[i].L.mul(A,gmm.gaussians[i].L);
    Matrix cov;
    gaussians[i].getCovariance(cov);
    gaussians[i].setCovariance(cov);
  }
}

Real GaussianMixtureModel::LogLikelihood(const std::vector<Vector>& data)
{
  Real ll = 0.0;
  for(size_t i=0;i<data.size();i++) {
    Real p=Probability(data[i]);
    if(p != 0.0)
      ll += Log(p);
    else
      ll += Log(p_outlier);
  }
  return ll;
}

bool GaussianMixtureModel::TrainEM(const std::vector<Vector>& examples,Real& tol,int maxIters,int verbose)
{
  int m=(int)examples.size();
  int n=(int)gaussians.size();
  int d=NumDims();

  //extract a structural covariance pattern
  Vector mean;
  Mean(examples,mean);
  Matrix allcov(mean.n,mean.n,Zero);
  Vector temp;
  for(size_t i=0;i<examples.size();i++) {
    temp.sub(examples[i],mean);
    for(int p=0;p<mean.n;p++)
      for(int q=0;q<mean.n;q++)
	allcov(p,q) += temp(p)*temp(q);
  }
  allcov /= examples.size();
  Real structuralZeroTolerance = 1e-7;
  int numStructural = 0;
  /*
  TEMP: do we want to infer structural constraints?
  for(int p=0;p<mean.n;p++)
    for(int q=p;q<mean.n;q++)
      if(Abs(allcov(p,q)) < structuralZeroTolerance) {
	numStructural++; 
	LOG4CXX_INFO(KrisLibrary::logger(),"Structural zero at "<<p<<" "<<q);
      }
  */

  if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Training GMM with "<< m<<" examples, "<<n);
  assert(m > 0);
  assert(n > 0);
  Real bestlikelihood, likelihood=LogLikelihood(examples);
  bestlikelihood = likelihood;
  if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Initial likelihood: "<< likelihood);
  Matrix w(m,n),lp(m,n);
  //initialize log probabilities
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++) 
      if(phi[j] > 0) 
	lp(i,j) = gaussians[j].logProbability(examples[i])+Log(phi[j]);
      else
	lp(i,j) = -Inf;
  for(int s=0;s<maxIters;s++) {
    if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Iteration "<< s);
    if((s+1) % 50 == 0) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Saving progress to temp.gmm\n");
      ofstream out("temp.gmm");
      out<<*this<<endl;
      out.close();
    }

    //if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"E...\n");
    //E step - calculate expectation of predictions
    int ndropped = 0;
    w = lp;
    for(int i=0;i<m;i++) {
      //exponentiate and normalize columns of w 
      Real wmax = -Inf;
      for(int j=0;j<n;j++) {
	wmax = Max(w(i,j),wmax);
      }
      Real wtot = Zero;
      if(!IsInf(wmax)) {
	for(int j=0;j<n;j++) {
	  w(i,j) = Exp(w(i,j)-wmax);
	  wtot += w(i,j);
	}
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"Weight for example "<<i<<": "<<wtot);
      //normalize
      if(wtot > 0.0) 
	for(int j=0;j<n;j++) w(i,j) /= wtot;
      else {
	//ill fitting GMM, do a reassignment
	ndropped++;
	//assign the most likely gaussian
	Real maxlp = -Inf;
	int best = 0;
	for(int j=0;j<n;j++) {
	  if(phi[j] == 0.0) continue;
	  if(lp(i,j) > maxlp) {
	    maxlp = lp(i,j);
	    best = j;
	  }
	}
	for(int j=0;j<n;j++) w(i,j) = 0.0;
	w(i,best) = 1.0;
      }
    }
    if(ndropped != 0) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, "<<ndropped<<"/"<<m);
      KrisLibrary::loggerWait();
    }

    Real elikelihood = 0.0;
    for(size_t i=0;i<examples.size();i++)
      for(int j=0;j<n;j++)
	if(phi[j] > 0)
	  elikelihood += w(i,j)*lp(i,j);
    if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"E step changed likelihood to "<<elikelihood);

  
    //if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"M...\n");
    //M step - maximize likelihood
    for(int j=0;j<n;j++) {
      if(phi[j] == 0.0) continue;
      /*
      //fit mean, phi
      Real sum_wj = Zero;
      Vector sum_wxj(d,Zero);
      for(int i=0;i<m;i++) {
	sum_wj += w(i,j);
	sum_wxj.madd(examples[i],w(i,j));
      }
      if(!IsFinite(sum_wj)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Numerical error on gaussian "<<j<<": probability "<<sum_wj);
	LOG4CXX_INFO(KrisLibrary::logger(),"phi "<<phi[j]);
	LOG4CXX_INFO(KrisLibrary::logger(),"mean "<<gaussians[j].mu);
	LOG4CXX_INFO(KrisLibrary::logger(),"L "<<gaussians[j].L);
	phi[j] = 0.0;
	KrisLibrary::loggerWait();
	continue;
      }
      LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j<<" probability changed from "<<phi[j]<<" to "<<sum_wj/m);
      if(sum_wj == 0) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j);
	phi[j] = 0.0;
	continue;
      }
      if(j == 0) 
	LOG4CXX_INFO(KrisLibrary::logger(),"Pre: "<<gaussians[j].mu);
      phi[j] = sum_wj/m;
      gaussians[j].mu.mul(sum_wxj,Inv(sum_wj));
      if(j == 0) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Post: "<<gaussians[j].mu);
	KrisLibrary::loggerWait();
      }

      //fit mean
      Matrix sum_wxtx(d,d,Zero);
      Vector temp;
      for(int i=0;i<m;i++) {
	temp.sub(examples[i],gaussians[j].mu);
	for(int p=0;p<d;p++) {
	  for(int q=0;q<d;q++) {
	    sum_wxtx(p,q) += temp(p)*temp(q)*w(i,j);
	  }
	}
      }
      sum_wxtx*=1.0/sum_wj;
      if(j == 0) {
	Matrix cov;
	gaussians[j].getCovariance(cov);
	LOG4CXX_INFO(KrisLibrary::logger(),cov);
      }
      gaussians[j].setCovariance(sum_wxtx);
      if(j == 0)
	LOG4CXX_INFO(KrisLibrary::logger(),sum_wxtx);
      */
      Real sum_wj = Zero;
      for(int i=0;i<m;i++) sum_wj += w(i,j);
      if(sum_wj == Zero) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Associations to Gaussian "<<j);
	LOG4CXX_INFO(KrisLibrary::logger(),"phi "<<phi[j]);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].mu);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].L);
	for(int i=0;i<m;i+=10000) {
	  Real maxlp = -Inf;
	  int best = 0;
	  for(int k=0;k<n;k++) {
	    if(lp(i,k) > maxlp) {
	      maxlp = lp(i,k);
	      best = k;
	    }
	  }
	  LOG4CXX_INFO(KrisLibrary::logger(),"ll "<<i<<": "<<lp(i,j)<<" (best "<<maxlp<<" in position "<<best<<")\n");
  }
	KrisLibrary::loggerWait();
	gaussians[j].mu.setZero();
	gaussians[j].L.setZero();
      }
      else {
	phi[j] = sum_wj/m;
	std::vector<Real> wj(m);
	for(int i=0;i<m;i++) wj[i]=w(i,j)/sum_wj;
	bool res=gaussians[j].setMaximumLikelihood(examples,wj);
	if(!res) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error setting maximum likelihood for gaussian "<<j);
	}
	if(numStructural > 0) {
	  Matrix cov;
	  gaussians[j].getCovariance(cov);
	  for(int p=0;p<cov.m;p++)
	    for(int q=0;q<cov.m;q++)
	      if(Abs(allcov(p,q)) < structuralZeroTolerance) 
		cov(p,q) = 0.0;
	  res = gaussians[j].setCovariance(cov);
	  if(!res) {
	    LOG4CXX_ERROR(KrisLibrary::logger(),"Error setting structural covariance for gaussian "<<j);
	  }	  
	}
      }

      //sanity check
      int nzero = 0;
      for(int i=0;i<examples[0].n;i++) {
	if(gaussians[j].L(i,i) <= covarianceRegularizationFactor) {
	  nzero++;
	  //LOG4CXX_INFO(KrisLibrary::logger(),sum_wxtx);
	  //phi[j] = 0.0;
	  //KrisLibrary::loggerWait();
	  gaussians[j].L(i,i) = covarianceRegularizationFactor;
	  //break;
	}
      }
      if(nzero == examples[0].n) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j);
	LOG4CXX_INFO(KrisLibrary::logger(),"phi "<<phi[j]);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].mu);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].L);
	for(int i=0;i<m;i+=10000){
	  LOG4CXX_INFO(KrisLibrary::logger(),"ll "<<i<<": "<<lp(i,j));	
  }
  KrisLibrary::loggerWait();
	phi[j] = 0.0;
      }
      else if(nzero > 0) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j<<" became degenerate on "<<nzero);
      }
      for(int i=0;i<examples[0].n;i++) {
	bool stop=false;
	for(int k=0;k<examples[0].n;k++) {
	  if(!IsFinite(gaussians[j].L(i,k))) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j);
	    //LOG4CXX_INFO(KrisLibrary::logger(),sum_wxtx);
	    LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].L);
	    phi[j] = 0.0;
	    stop=true;
	    KrisLibrary::loggerWait();
	    break;
	  }
	}
	if(stop) break;
      }
    }
    //may need an adjustment -- dropped examples
    Normalize(phi);

    //fix some degeneracies for zero-weight gaussians
    int splits=0;
    for(int j=0;j<n;j++) {
      if(phi[j] != 0.0) continue;
      //pick the component with maximum phi, split it along the direction of maximum covariance
      int split = 0;
      Real maxphi = 0.0;
      for(int k=0;k<n;k++) 
	if(phi[k]>maxphi) {
	  maxphi = phi[k];
	  split = k;
	}
      SVDecomposition<Real> svd;
      if(!svd.set(gaussians[split].L)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"SVD of gaussian "<<split);
	continue;
      }
      splits++;
      int bestDir;
      Real maxStd = svd.W.maxAbsElement(&bestDir);
      LOG4CXX_INFO(KrisLibrary::logger(),"Splitting "<<j<<" off of gaussian "<<split<<", p "<<phi[split]<<" stddev "<<maxStd);
      gaussians[j].mu = gaussians[split].mu;
      Vector d;
      svd.U.getColRef(bestDir,d);
      Real u = 0.5;
      Real h = u*maxStd;
      //h^2 + s'^2 = s^2
      //let h = u*s =>  s'^2 = (1-u^2) s^2
      gaussians[split].mu.madd(d,h);
      gaussians[j].mu.madd(d,-h);
      svd.W(bestDir) = Sqrt((1.0-Sqr(u)))*maxStd;
      Matrix temp;
      svd.W.postMultiply(svd.U,temp);
      svd.W.postMultiply(temp,temp);
      Matrix cov;
      cov.mulTransposeB(temp,svd.U);
      if(numStructural > 0) {
	for(int p=0;p<cov.m;p++)
	  for(int q=0;q<cov.m;q++)
	    if(Abs(allcov(p,q)) < structuralZeroTolerance) 
	      cov(p,q) = 0.0;
      }
      bool res=gaussians[split].setCovariance(cov);
      assert(res);

      int nzero=0;
      for(int i=0;i<examples[0].n;i++) 
	if(gaussians[split].L(i,i) <= 1e-3) {
	  gaussians[split].L(i,i) = 1e-3;
	  nzero++;
	}

      gaussians[j].L = gaussians[split].L;
      phi[j] = maxphi*0.5;
      phi[split] = maxphi*0.5;
    }
    if(splits > 0) KrisLibrary::loggerWait();

    //recompute log probabilities
    for(int i=0;i<m;i++)
      for(int j=0;j<n;j++)
	if(phi[j] > 0)
	  lp(i,j) = Log(phi[j])+gaussians[j].logProbability(examples[i]);
	else
	  lp(i,j) = -Inf;

    //calculate likelihood of data with updated components
    Real oldlikelihood = likelihood;
    likelihood = 0.0;
    for(int i=0;i<m;i++)
      for(int j=0;j<n;j++)
	if(phi[j] > 0)
	  likelihood += w(i,j)*lp(i,j);
    if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"New likelihood: "<<likelihood);
    if(Abs(likelihood - oldlikelihood) < tol) {
      tol = likelihood;
      return true;
    }
    if(likelihood < bestlikelihood*(1.0 - 0.05*Sign(bestlikelihood))) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: EM algorithm is diverging?");
      //return true;
    }
    bestlikelihood = Max(likelihood,bestlikelihood);

  }
  return false;
}

bool GaussianMixtureModel::TrainDiagonalEM(const std::vector<Vector>& examples,Real& tol,int maxIters,int verbose)
{
  int m=(int)examples.size();
  int n=(int)gaussians.size();
  int d=NumDims();

  if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Training diagonal GMM with "<< m<<" examples, "<<n);
  assert(m > 0);
  assert(n > 0);
  Real bestlikelihood, likelihood=LogLikelihood(examples);
  bestlikelihood = likelihood;
  if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Initial likelihood: "<< likelihood);
  Matrix w(m,n),lp(m,n);
  //initialize log probabilities
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++) 
      if(phi[j] > 0) 
	lp(i,j) = gaussians[j].logProbability(examples[i])+Log(phi[j]);
      else
	lp(i,j) = -Inf;
  for(int s=0;s<maxIters;s++) {
    if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"Iteration "<< s);

    //if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"E...\n");
    //E step - calculate expectation of predictions
    int ndropped = 0;
    w = lp;
    for(int i=0;i<m;i++) {
      //exponentiate and normalize columns of w 
      Real wmax = -Inf;
      for(int j=0;j<n;j++) {
	wmax = Max(w(i,j),wmax);
      }
      Real wtot = Zero;
      if(!IsInf(wmax)) {
	for(int j=0;j<n;j++) {
	  w(i,j) = Exp(w(i,j)-wmax);
	  wtot += w(i,j);
	}
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"Weight for example "<<i<<": "<<wtot);
      //normalize
      if(wtot > 0.0) 
	for(int j=0;j<n;j++) w(i,j) /= wtot;
      else {
	//ill fitting GMM, do a reassignment
	ndropped++;
	//assign the most likely gaussian
	Real maxlp = -Inf;
	int best = 0;
	for(int j=0;j<n;j++) {
	  if(phi[j] == 0.0) continue;
	  if(lp(i,j) > maxlp) {
	    maxlp = lp(i,j);
	    best = j;
	  }
	}
	for(int j=0;j<n;j++) w(i,j) = 0.0;
	w(i,best) = 1.0;
      }
    }
    if(ndropped != 0) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, "<<ndropped<<"/"<<m);
      KrisLibrary::loggerWait();
    }

    Real elikelihood = 0.0;
    for(size_t i=0;i<examples.size();i++)
      for(int j=0;j<n;j++)
	if(phi[j] > 0)
	  elikelihood += w(i,j)*lp(i,j);
    if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"E step changed likelihood to "<<elikelihood);

  
    //if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"M...\n");
    //M step - maximize likelihood
    for(int j=0;j<n;j++) {
      if(phi[j] == 0.0) continue;
      Real sum_wj = Zero;
      for(int i=0;i<m;i++) sum_wj += w(i,j);
      if(sum_wj == Zero) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Associations to Gaussian "<<j);
	LOG4CXX_INFO(KrisLibrary::logger(),"phi "<<phi[j]);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].mu);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].L);
	for(int i=0;i<m;i+=10000) {
	  Real maxlp = -Inf;
	  int best = 0;
	  for(int k=0;k<n;k++) {
	    if(lp(i,k) > maxlp) {
	      maxlp = lp(i,k);
	      best = k;
	    }
	  }
	  LOG4CXX_INFO(KrisLibrary::logger(),"ll "<<i<<": "<<lp(i,j)<<" (best "<<maxlp<<" in position "<<best<<")\n");
  }
	KrisLibrary::loggerWait();
	gaussians[j].mu.setZero();
	gaussians[j].L.setZero();
      }
      else {
	phi[j] = sum_wj/m;
	std::vector<Real> wj(m);
	for(int i=0;i<m;i++) wj[i]=w(i,j)/sum_wj;
	gaussians[j].setMaximumLikelihoodDiagonal(examples,wj);
      }

      //sanity check
      int nzero = 0;
      for(int i=0;i<examples[0].n;i++) {
	if(gaussians[j].L(i,i) <= 1e-3) {
	  nzero++;
	  //LOG4CXX_INFO(KrisLibrary::logger(),sum_wxtx);
	  //phi[j] = 0.0;
	  //KrisLibrary::loggerWait();
	  gaussians[j].L(i,i) = 1e-3;
	  //break;
	}
      }
      if(nzero == examples[0].n) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j);
	LOG4CXX_INFO(KrisLibrary::logger(),"phi "<<phi[j]);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].mu);
	LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].L);
	for(int i=0;i<m;i+=10000){
	  LOG4CXX_INFO(KrisLibrary::logger(),"ll "<<i<<": "<<lp(i,j));
  }
  KrisLibrary::loggerWait();
	phi[j] = 0.0;
      }
      else if(nzero > 0) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j<<" became degenerate on "<<nzero);
      }
      for(int i=0;i<examples[0].n;i++) {
	bool stop=false;
	for(int k=0;k<examples[0].n;k++) {
	  if(!IsFinite(gaussians[j].L(i,k))) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<j);
	    //LOG4CXX_INFO(KrisLibrary::logger(),sum_wxtx);
	    LOG4CXX_INFO(KrisLibrary::logger(),gaussians[j].L);
	    phi[j] = 0.0;
	    stop=true;
	    KrisLibrary::loggerWait();
	    break;
	  }
	}
	if(stop) break;
      }
    }
    //may need an adjustment -- dropped examples
    Normalize(phi);

    //fix some degeneracies for zero-weight gaussians
    int splits=0;
    for(int j=0;j<n;j++) {
      if(phi[j] != 0.0) continue;
      //pick the component with maximum phi, split it along the direction of maximum covariance
      int split = 0;
      Real maxphi = 0.0;
      for(int k=0;k<n;k++) 
	if(phi[k]>maxphi) {
	  maxphi = phi[k];
	  split = k;
	}
      SVDecomposition<Real> svd;
      if(!svd.set(gaussians[split].L)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"SVD of gaussian "<<split);
	continue;
      }
      splits++;
      int bestDir;
      Real maxStd = svd.W.maxAbsElement(&bestDir);
      LOG4CXX_INFO(KrisLibrary::logger(),"Splitting "<<j<<" off of gaussian "<<split<<", p "<<phi[split]<<" stddev "<<maxStd);
      gaussians[j].mu = gaussians[split].mu;
      Vector d;
      svd.U.getColRef(bestDir,d);
      Real u = 0.5;
      Real h = u*maxStd;
      //h^2 + s'^2 = s^2
      //let h = u*s =>  s'^2 = (1-u^2) s^2
      gaussians[split].mu.madd(d,h);
      gaussians[j].mu.madd(d,-h);
      svd.W(bestDir) = Sqrt((1.0-Sqr(u)))*maxStd;
      Matrix temp;
      svd.W.postMultiply(svd.U,temp);
      svd.W.postMultiply(temp,temp);
      Matrix cov;
      cov.mulTransposeB(temp,svd.U);
      bool res=gaussians[split].setCovariance(cov);
      assert(res);

      int nzero=0;
      for(int i=0;i<examples[0].n;i++) 
	if(gaussians[split].L(i,i) <= 1e-3) {
	  gaussians[split].L(i,i) = 1e-3;
	  nzero++;
	}

      gaussians[j].L = gaussians[split].L;
      phi[j] = maxphi*0.5;
      phi[split] = maxphi*0.5;
    }
    if(splits > 0) KrisLibrary::loggerWait();

    //recompute log probabilities
    for(int i=0;i<m;i++)
      for(int j=0;j<n;j++)
	if(phi[j] > 0)
	  lp(i,j) = Log(phi[j])+gaussians[j].logProbability(examples[i]);
	else
	  lp(i,j) = -Inf;

    //calculate likelihood of data with updated components
    Real oldlikelihood = likelihood;
    likelihood = 0.0;
    for(int i=0;i<m;i++)
      for(int j=0;j<n;j++)
	if(phi[j] > 0)
	  likelihood += w(i,j)*lp(i,j);
    if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"New likelihood: "<<likelihood);
    if(Abs(likelihood - oldlikelihood) < tol) {
      tol = likelihood;
      return true;
    }
    if(likelihood < bestlikelihood*(1.0 - 0.05*Sign(bestlikelihood))) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: EM algorithm is diverging?");
      //return true;
    }
    bestlikelihood = Max(likelihood,bestlikelihood);

  }
  return false;
}


int GaussianMixtureModel::PickGaussian() const
{
  return WeightedSample(phi);
}

Real GaussianMixtureModel::Probability(const Vector& x) const
{
  Real p=Zero;
  for(size_t i=0;i<gaussians.size();i++) {
    if(phi[i] > 0.0)
      p += gaussians[i].probability(x)*phi[i];
  }
  return p;
}

void GaussianMixtureModel::Generate(Vector& x) const
{
  int z=PickGaussian();
  gaussians[z].generate(x);
}

void GaussianMixtureModel::GetMean(Vector& x) const
{
  x.mul(gaussians[0].mu,phi[0]);
  for(size_t i=1;i<gaussians.size();i++)
    x.madd(gaussians[i].mu,phi[i]);
}

void GaussianMixtureModel::GetMode(Vector& x) const
{
  x = gaussians[0].mu;
  Real maxp = phi[0]/gaussians[0].normalizationFactor();
  for(size_t i=1;i<gaussians.size();i++)
    if(phi[i]/gaussians[i].normalizationFactor() > maxp) {
      x = gaussians[i].mu;
      maxp = phi[i]/gaussians[i].normalizationFactor();
    }
}

void GaussianMixtureModel::GetCovariance(Matrix& cov) const
{
  Vector mean;
  GetMean(mean);
  cov.resize(mean.n,mean.n);
  cov.setZero();
  Matrix temp;
  Vector diff;
  for(size_t i=0;i<gaussians.size();i++) {
    if(phi[i] == 0.0) continue;
    assert(gaussians[i].L.n == mean.n);
    assert(gaussians[i].L.m == mean.n);
    gaussians[i].getCovariance(temp);
    assert(cov.m == temp.m);
    assert(cov.n == temp.n);
    cov.madd(temp,phi[i]);
    diff.sub(gaussians[i].mu,mean);
    for(int j=0;j<mean.n;j++)
      for(int k=0;k<mean.n;k++)
	cov(j,k) += phi[i]*diff(j)*diff(k);
  }
}

void GaussianMixtureModel::GetVariance(Vector& var) const
{
  Vector mean;
  GetMean(mean);
  var.resize(mean.n);
  var.setZero();
  Vector temp;
  Vector diff;
  for(size_t i=0;i<gaussians.size();i++) {
    gaussians[i].getVariance(temp);
    var += temp;
    diff.sub(gaussians[i].mu,mean);
    for(int j=0;j<mean.n;j++)
      var(j) += diff(j)*diff(j);
  }
}





GaussianMixtureModelRaw::GaussianMixtureModelRaw()
{}

GaussianMixtureModelRaw::GaussianMixtureModelRaw(int k,int d)
{
  Resize(k,d);
}

GaussianMixtureModelRaw::GaussianMixtureModelRaw(const GaussianMixtureModelRaw& m)
{
  means = m.means;
  covariances = m.covariances;
  phi = m.phi;
}


GaussianMixtureModelRaw::GaussianMixtureModelRaw(const GaussianMixtureModel& m)
{
  Set(m);
}

void GaussianMixtureModelRaw::Set(const GaussianMixtureModel& m)
{
  means.resize(m.gaussians.size());
  covariances.resize(m.gaussians.size());
  for(size_t i=0;i<m.gaussians.size();i++) {
    means[i] = m.gaussians[i].mu;
    m.gaussians[i].getCovariance(covariances[i]);
  }
  phi = m.phi;
}


bool GaussianMixtureModelRaw::Get(GaussianMixtureModel& m) const
{
  bool res=true;
  m.gaussians.resize(means.size());
  for(size_t i=0;i<m.gaussians.size();i++) {
    m.gaussians[i].mu = means[i];
    if(!m.gaussians[i].setCovariance(covariances[i],1)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Failed to decompose covariance "<<i);
      res=false;
    }
  }
  m.phi=phi;
  return res;
}

int GaussianMixtureModelRaw::NumDims() const 
{ 
  if(means.empty()) return 0;
  return means[0].n;
}

void GaussianMixtureModelRaw::Resize(int k,int d)
{
  means.resize(k);
  covariances.resize(k);
  phi.resize(k,One/Real(k));
  for(size_t i=0;i<means.size();i++) {
    means[i].resize(d);
    covariances[i].resize(d,d);
  }
}

void GaussianMixtureModelRaw::SetLinearTransform(const GaussianMixtureModelRaw& gmm,const Matrix& A,const Vector& b)
{
  if(this == &gmm) {
    GaussianMixtureModelRaw temp=gmm;
    SetLinearTransform(temp,A,b);
    return;
  }
  phi = gmm.phi;
  means.resize(gmm.means.size());
  covariances.resize(gmm.covariances.size());
  for(size_t i=0;i<means.size();i++) {
    GaussianTransform(gmm.means[i],gmm.covariances[i],A,b,
		      means[i],covariances[i]);
  }
}

void GaussianMixtureModelRaw::SetSubset(const GaussianMixtureModelRaw& gmm,const vector<int>& keptIndices)
{
  if(this == &gmm) {
    GaussianMixtureModelRaw temp=gmm;
    SetSubset(temp,keptIndices);
    return;
  }
  phi = gmm.phi;
  means.resize(gmm.means.size());
  covariances.resize(gmm.covariances.size());
  for(size_t i=0;i<means.size();i++) {
    means[i].resize(keptIndices.size());
    covariances[i].resize(keptIndices.size(),keptIndices.size());
    GetElements(gmm.means[i],keptIndices,means[i]);
    GetElements(gmm.covariances[i],keptIndices,keptIndices,covariances[i]);
  }
}

void GaussianMixtureModelRaw::SetMarginalized(const GaussianMixtureModelRaw& gmm,const vector<int>& dropIndices)
{
  vector<bool> kept(gmm.means[0].n,true);
  vector<int> keptIndices;
  for(size_t i=0;i<dropIndices.size();i++)
    kept[dropIndices[i]]=false;
  for(size_t i=0;i<kept.size();i++)
    if(kept[i]) keptIndices.push_back(i);
  SetSubset(gmm,keptIndices);
}

void GaussianMixtureModelRaw::SetCombination(const vector<GaussianMixtureModelRaw>& gmms,const std::vector<Real>& weights)
{
  assert(gmms.size()==weights.size());
  size_t n=0;
  for(size_t i=0;i<gmms.size();i++) n+=gmms[i].phi.size();
  phi.resize(n);
  means.resize(n);
  covariances.resize(n);
  n = 0;
  for(size_t i=0;i<gmms.size();i++) {
    for(size_t j=0;j<gmms[i].phi.size();j++)
      phi[j+n] = weights[i]*gmms[i].phi[j];
    copy(gmms[i].means.begin(),gmms[i].means.end(),means.begin()+n);
    copy(gmms[i].covariances.begin(),gmms[i].covariances.end(),covariances.begin()+n);
    n += gmms[i].phi.size();
  }
}

int GaussianMixtureModelRaw::PickGaussian() const
{
  return WeightedSample(phi);
}

void GaussianMixtureModelRaw::GetMean(Vector& x) const
{
  x.mul(means[0],phi[0]);
  for(size_t i=1;i<means.size();i++)
    x.madd(means[i],phi[i]);
}

void GaussianMixtureModelRaw::GetMode(Vector& x) const
{
  GaussianMixtureModel gmm;
  Get(gmm);
  gmm.GetMode(x);
}

void GaussianMixtureModelRaw::GetCovariance(Matrix& cov) const
{
  Vector mean;
  GetMean(mean);
  cov.resize(mean.n,mean.n);
  cov.setZero();
  Matrix temp;
  Vector diff;
  for(size_t i=0;i<covariances.size();i++) {
    if(phi[i] == 0.0) continue;
    assert(covariances[i].n == mean.n);
    assert(covariances[i].m == mean.n);
    cov.madd(covariances[i],phi[i]);
    diff.sub(means[i],mean);
    for(int j=0;j<mean.n;j++)
      for(int k=0;k<mean.n;k++)
	cov(j,k) += phi[i]*diff(j)*diff(k);
  }
}

void GaussianMixtureModelRaw::GetVariance(Vector& var) const
{
  Vector mean;
  GetMean(mean);
  var.resize(mean.n);
  var.setZero();
  Vector temp;
  Vector diff;
  for(size_t i=0;i<covariances.size();i++) {
    diff.sub(means[i],mean);
    for(int j=0;j<mean.n;j++)
      var(j) += Sqr(covariances[i](j,j))+ Sqr(diff(j));
  }
}





void GaussianRegression::Set(const Gaussian<Real>& g,const std::vector<int>& xindices,const std::vector<int>& yindices)
{
  Matrix cov;
  g.getCovariance(cov);
  Set(g.mu,cov,xindices,yindices);
}

void GaussianRegression::Set(const Vector& xymean,const Matrix& cov,const std::vector<int>& xindices,const std::vector<int>& yindices) 
{
  yxcov.resize(yindices.size(),xindices.size());
  ycov.resize(yindices.size(),yindices.size());
  xmean.resize(xindices.size());
  ymean.resize(yindices.size());

  Matrix xcov(xindices.size(),xindices.size());
  GetElements(cov,xindices,xindices,xcov);
  GetElements(cov,yindices,xindices,yxcov);
  GetElements(cov,yindices,yindices,ycov);
  GetElements(xymean,xindices,xmean);
  GetElements(xymean,yindices,ymean);

  Matrix invC22;
  LDLDecomposition<Real> cholesky;
  cholesky.set(xcov);
  //cholesky.getInverse(xcovinv);
  cholesky.getPseudoInverse(xcovinv);
  //assert(xcovinv.isSymmetric());
  //assert(ycov.isSymmetric());

  //compute covariance via schur complement
  Matrix mtemp,mtemp2;
  mtemp.mulTransposeB(xcovinv,yxcov);
  mtemp2.mul(yxcov,mtemp);
  ycov -= mtemp2;

  //compute regression matrix
  A.mul(yxcov,xcovinv);
  A.mul(xmean,b);
  b.inplaceNegative();
  b += ymean;
}

void GaussianRegression::GetY(const Vector& x,Vector& mu,Matrix& cov) const
{
  //compute new mean
  A.mul(x,mu);
  mu += b;
  cov=ycov;
}

void GaussianRegression::GetY(const Vector& x,Gaussian<Real>& y) const
{
  Vector mean;
  Matrix cov;
  GetY(x,mean,cov);
  y.setMean(mean);
  bool res=y.setCovariance(cov,0);
  if(!res) LOG4CXX_INFO(KrisLibrary::logger(),"GaussianRegression: failed to set covariance\n");
  for(int j=0;j<y.L.m;j++)
    if(y.L(j,j) <= 0.0) {
      LOG4CXX_INFO(KrisLibrary::logger(),"GaussianRegression: zero entry on diagonal? "<<y.L(j,j));
    }
}

void GaussianRegression::GetY(const Vector& xmean2,const Matrix& xcov,Vector& ymean,Matrix& ycov) const
{
  GetY(xmean2,ymean,ycov);
  //y = ymean + yxcov*xcovinv*(x-xmean) = b + A*x;
  //add A xcov A^T to the covariance matrix
  Matrix temp,temp2;
  temp.mul(A,xcov);
  temp2.mulTransposeB(temp,A);
  temp.clear();
  /*
  //TEST: sanity check
  temp.setTranspose(temp2);
  if(!temp.isEqual(temp2,1e-5*(1.0+temp2.maxAbsElement()))) {
    LOG4CXX_INFO(KrisLibrary::logger(),"GetY: result is not symmetric!\n");
    LOG4CXX_INFO(KrisLibrary::logger(),temp);
    LOG4CXX_INFO(KrisLibrary::logger(),xcov);
    KrisLibrary::loggerWait();
  }
  */
  ycov += temp2;
}

void GaussianRegression::GetJoint(const Vector& xmean2,const Matrix& xcov,Vector& xymean,Matrix& xyvar) const
{
  int nx=xmean.n;
  int ny=ymean.n;
  xymean.resize(nx+ny);
  xymean.copySubVector(0,xmean2);
  Vector ymean;
  Matrix ycov,xycov,yxcov;
  ymean.setRef(xymean,xmean.n,1,ny);
  xyvar.resize(xymean.n,xymean.n);
  xyvar.copySubMatrix(0,0,xcov);
  xycov.setRef(xyvar,0,xmean.n,1,1,nx,ny);
  yxcov.setRef(xyvar,xmean.n,0,1,1,ny,nx);
  ycov.setRef(xyvar,xmean.n,xmean.n,1,1,ny,ny);

  GetY(xmean2,ymean,ycov);
  //y = ymean + yxcov*xcovinv*(x-xmean) = b + A*x;
  //add A xcov A^T to the covariance matrix
  Matrix temp;
  yxcov.mul(A,xcov);
  xycov.setTranspose(yxcov);
  temp.mulTransposeB(yxcov,A);
  ycov += temp;
}

void GaussianRegression::GetLinearEquation(Matrix& _A,Vector& _b) const
{
  _A = A;
  _b = b;
}

void GaussianRegression::GetNoiseCovariance(Matrix& sigma) const
{
  sigma = ycov;
}

GaussianMixtureRegression::GaussianMixtureRegression()
{}

GaussianMixtureRegression::GaussianMixtureRegression(const GaussianMixtureModel& _joint)
  :joint(_joint)
{}

GaussianMixtureRegression::GaussianMixtureRegression(const GaussianMixtureModelRaw& _joint)
{
  _joint.Get(joint);
}


void GaussianMixtureRegression::SetXIndices(const std::vector<int>& _xindices)
{
  xindices = _xindices;
  yindices.resize(0);
  xgmm.phi = joint.phi;
  xgmm.gaussians.resize(joint.gaussians.size());
  for(int i=0;i<joint.gaussians[0].mu.n;i++) {
    int index=-1;
    for(size_t j=0;j<xindices.size();j++) {
      if(xindices[j] == i) {
	index = (int)j;
	break;
      }
    }
    if(index < 0) yindices.push_back(i);
  }
  assert(int(yindices.size() + xindices.size()) == joint.gaussians[0].mu.n);
  for(size_t i=0;i<joint.gaussians.size();i++) {
    xgmm.gaussians[i].setMarginalized(joint.gaussians[i],yindices);
  }

  regressions.resize(joint.gaussians.size());
  for(size_t i=0;i<regressions.size();i++)
    regressions[i].Set(joint.gaussians[i],xindices,yindices);
}

Real GaussianMixtureRegression::ProbabilityX(const Vector& x) const
{
  return xgmm.Probability(x);
}

void GaussianMixtureRegression::GetY(const Vector& x,GaussianMixtureModelRaw& y) const
{
  assert(x.n == (int)xindices.size());
  assert(xgmm.gaussians.size() == joint.gaussians.size());
  y.phi.resize(joint.gaussians.size());
  y.means.resize(joint.gaussians.size());
  y.covariances.resize(joint.gaussians.size());
  for(size_t i=0;i<y.phi.size();i++) {
    if(joint.phi[i] == 0.0) {
      y.phi[i] = -Inf;
      y.means[i].resize((int)yindices.size(),Zero);
      y.covariances[i].resize((int)yindices.size(),(int)yindices.size(),Zero);
    }
    else {
      y.phi[i] = xgmm.gaussians[i].logProbability(x)+Log(joint.phi[i]);
      if(!IsInf(y.phi[i])) {
	regressions[i].GetY(x,y.means[i],y.covariances[i]);
      }
      else {
	y.means[i].resize((int)yindices.size(),Zero);
	y.covariances[i].resize((int)yindices.size(),(int)yindices.size(),Zero);
      }
    }
  }
  ExpNormalize(y.phi);
}

void GaussianMixtureRegression::GetY(const Vector& x,GaussianMixtureModel& y) const
{
  assert(x.n == (int)xindices.size());
  assert(xgmm.gaussians.size() == joint.gaussians.size());
  y.gaussians.resize(joint.gaussians.size());
  y.phi.resize(y.gaussians.size());
  for(size_t i=0;i<y.gaussians.size();i++) {
    if(joint.phi[i] == 0.0) {
      y.phi[i] = -Inf;
      y.gaussians[i].resize(joint.gaussians[i].mu.n-(int)xindices.size());
      y.gaussians[i].L.setZero();
    }
    else {
      y.phi[i] = xgmm.gaussians[i].logProbability(x)+Log(joint.phi[i]);
      if(!IsInf(y.phi[i])) {
	regressions[i].GetY(x,y.gaussians[i]);
      }
      else {
	y.gaussians[i].resize(joint.gaussians[i].mu.n-(int)xindices.size());
	y.gaussians[i].L.setZero();
      }
    }
  }
  ExpNormalize(y.phi);
}

Real logDot(const Gaussian<Real>& g,const Vector& xmean,const Matrix& xcov)
{
  //x1 ~ N(mug,Kg)
  //x2 ~ N(mux,Kx)
  //Let y = x1-x2
  //P(x1=x2) = P(y=0)
  //y ~ N(mug-mux,Kg+Kx)
  Gaussian<Real> gy;
  Matrix gcov;
  g.getCovariance(gcov);
  gy.setMean(g.mu-xmean);
  gcov += xcov;
  if(!gy.setCovariance(gcov,0)) return -Inf;
  Vector zero(xmean.n,0.0);
  return gy.logProbability(zero);
}

void GaussianMixtureRegression::GetY(const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& y) const
{
  assert(xmean.n == (int)xindices.size());
  assert(xcov.m == (int)xindices.size());
  assert(xcov.n == (int)xindices.size());
  assert(xgmm.gaussians.size() == joint.gaussians.size());
  y.phi.resize(joint.gaussians.size());
  y.means.resize(joint.gaussians.size());
  y.covariances.resize(joint.gaussians.size());
  for(size_t i=0;i<y.phi.size();i++) {
    if(joint.phi[i] == 0.0) {
      y.phi[i] = -Inf;
      y.means[i].resize((int)yindices.size(),Zero);
      y.covariances[i].resize((int)yindices.size(),(int)yindices.size(),Zero);
    }
    else {
      y.phi[i] = logDot(xgmm.gaussians[i],xmean,xcov)+Log(joint.phi[i]);
      if(!IsInf(y.phi[i])) {
	regressions[i].GetY(xmean,xcov,y.means[i],y.covariances[i]);
      }
      else {
	y.means[i].resize((int)yindices.size(),Zero);
	y.covariances[i].resize((int)yindices.size(),(int)yindices.size(),Zero);
      }
    }
  }
  ExpNormalize(y.phi);
}

void GaussianMixtureRegression::GetJoint(const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& xy) const
{
  assert(xmean.n == (int)xindices.size());
  assert(xcov.m == (int)xindices.size());
  assert(xcov.n == (int)xindices.size());
  assert(xgmm.gaussians.size() == joint.gaussians.size());
  int nxy = (int)(xindices.size()+yindices.size());
  xy.phi.resize(joint.gaussians.size());
  xy.means.resize(joint.gaussians.size());
  xy.covariances.resize(joint.gaussians.size());
  for(size_t i=0;i<xy.phi.size();i++) {
    if(joint.phi[i] == 0.0) {
      xy.phi[i] = -Inf;
      xy.means[i].resize(nxy,Zero);
      xy.covariances[i].resize(nxy,nxy,Zero);
    }
    else {
      xy.phi[i] = logDot(xgmm.gaussians[i],xmean,xcov)+Log(joint.phi[i]);
      if(!IsInf(xy.phi[i])) {
	regressions[i].GetJoint(xmean,xcov,xy.means[i],xy.covariances[i]);
      }
      else {
	xy.means[i].resize(nxy,Zero);
	xy.covariances[i].resize(nxy,nxy,Zero);
      }
    }
  }
  ExpNormalize(xy.phi);
}





void Subsample(vector<Real>& weights,vector<int>& values)
{
  assert(values.size() <= weights.size());
  vector<Real> importance(values.size());
  Real sumw=0.0;
  for(size_t j=0;j<weights.size();j++) sumw += weights[j];
  assert(sumw > 0.0);
  for(size_t i=0;i<values.size();i++) {
    Real sumwi=0.0;
    for(size_t j=0;j<weights.size();j++) sumwi += weights[j];
    if(sumwi == 0.0) {
      //early terminate
      importance.resize(i);
      values.resize(i);
      break;
    }
    //p1 = 0.1, p2 = 0.4, p3 = 0.5
    //sample 2 weighted w/o replacement, weight evenly:
    //p1,p2: occurs with freq 0.1*0.4/0.9 + 0.4*0.1/0.6 = 5/45
    //p2,p3: occurs with freq 0.4*0.5/0.6 + 0.5*0.4/0.5 = 33/45
    //p1,p3: occurs with freq 0.1*0.5/0.9 + 0.5*0.1/0.5 = 7/45
    //p1,p2,p3 occurs with probability 12/45, 38/45, and 40/45 resp
    //
    int k=WeightedSample(weights,sumwi);
    values[i] = k;
    importance[i] = 1.0;
    weights[k] = 0.0;
  }
  for(size_t i=0;i<importance.size();i++) if(!IsFinite(importance[i])) importance[i]=0.0;
  Normalize(importance);
  weights = importance;
}

void GaussianMixtureModel::Resample(int numComponents)
{
  if(numComponents > (int)phi.size()) {
    vector<int> num(phi.size(),0);
    RandomAllocate(num,numComponents,phi);
    
    GaussianMixtureModel res;
    res.phi.resize(numComponents,1.0/Real(numComponents));
    res.gaussians.resize(numComponents);
    int k=0;
    for(size_t i=0;i<num.size();i++) {
      for(int j=0;j<num[i];j++,k++) {
	res.phi[k] = 1.0/num[i];
	res.gaussians[k] = gaussians[i];
      }
    }
    assert(k == numComponents);
    Normalize(res.phi);
    phi = res.phi;
    gaussians = res.gaussians;
  }
  else {
    vector<int> values(numComponents);
    Subsample(phi,values);
    vector<Gaussian<Real> > newgaussians;
    newgaussians.resize(numComponents);
    for(size_t i=0;i<values.size();i++) {
      newgaussians[i] = gaussians[values[i]];
    }
    swap(newgaussians,gaussians);
  }
}

void GaussianMixtureModelRaw::Resample(int numComponents)
{
  if(numComponents > (int)phi.size()) {
    vector<int> num(phi.size(),0);
    RandomAllocate(num,numComponents,phi);
    
    GaussianMixtureModelRaw res;
    res.phi.resize(numComponents,1.0/Real(numComponents));
    res.means.resize(numComponents);
    res.covariances.resize(numComponents);
    int k=0;
    for(size_t i=0;i<num.size();i++) {
      for(int j=0;j<num[i];j++,k++) {
	res.phi[k] = phi[i]/num[i];
	res.means[k] = means[i];
	res.covariances[k] = covariances[i];
      }
    }
    assert(k == numComponents);
    Normalize(res.phi);
    phi = res.phi;
    means = res.means;
    covariances = res.covariances;
  }
  else {
    vector<int> values(numComponents);
    Subsample(phi,values);
    vector<Vector> newmeans(numComponents);
    vector<Matrix> newcovs(numComponents);
    for(size_t i=0;i<values.size();i++) {
      newmeans[i] = means[values[i]];
      newcovs[i] = covariances[values[i]];
    }
    swap(newmeans,means);
    swap(newcovs,covariances);
  }
}



/**@brief Computes int_x b1(x)b2(x) dx
 * with b1(x) = N(x;mu1,K1), b2 = N(x;mu2,K2)
 *
 * Method considers joint distribution of variables
 * X1 ~ N(mu1,K1), X2 ~ N(mu2,K2), Y=X1-X2, X1 independent of X2:
 * [X1;X2;Y] ~ N([mu1;mu2;mu1-mu2],[K1,0,K1; 0,K2,-K2; K1,-K2,K1+K2])
 * and then considers the probability that Y=0.
 * Y ~= N(mu1-mu2,K1+K2)
 * => N(0;mu1-mu2,K1+K2) = int_x b1(x)b2(x) dx 
 */
Real LogDotProduct(const Vector& mu1,const Matrix& K1,const Vector& mu2,const Matrix& K2)
{
  Matrix K;
  K.add(K1,K2);
  Vector mu;
  mu.sub(mu1,mu2);
  Gaussian<Real> g;
  g.setMean(mu);
  if(!g.setCovariance(K,0)) return -Inf;
  mu.setZero();
  return g.logProbability(mu);
}

//cos(theta) = N1 dot N2 / (sqrt(N1 dot N1) sqrt(N2 dot N2))
//N1 dot N1 = N(0;0,2K1) = 1/Z(2K1)
//N2 dot N2 = N(0;0,2K2) = 1/Z(2K2)
//Z(K) = (2pi)^d/2 sqrt(|K|)
//so Z(2K) = Z(K) 2^d/2
//log cos theta = log N1 dot N2 - 1/2 log N1 dot N1 - 1/2 log N2 dot N2
Real LogCosAngle(const Vector& mu1,const Matrix& K1,const Vector& mu2,const Matrix& K2)
{
  LDLDecomposition<Real> ldl1,ldl2;
  ldl1.verbose = ldl2.verbose = 0;
  ldl1.set(K1);
  ldl2.set(K2);
  Vector D1,D2;
  ldl1.getD(D1);
  ldl2.getD(D2);
  int d1=0,d2=0;
  Real logdet1=Zero,logdet2=Zero;
  for(int i=0;i<D1.n;i++) {
    if(D1(i) != 0.0) {
      logdet1 += Log(D1(i));
      d1++;
    }
  }
  for(int i=0;i<D2.n;i++) {
    if(D2(i) != 0.0) {
      logdet2 += Log(D2(i));
      d2++;
    }
  }
  return LogDotProduct(mu1,K1,mu2,K2) + Half*(Half*Log(TwoPi)*Real(d1+d2) + Half*(logdet1 + logdet2));
}

Real LogCosAngle(const Gaussian<Real>& g1,const Gaussian<Real>& g2)
{
  int d1=0,d2=0;
  Real logdet1=Zero,logdet2=Zero;
  for(int i=0;i<g1.mu.n;i++) {
    if(g1.L(i,i) != 0.0) {
      logdet1 += Log(g1.L(i,i));
      d1++;
    }
  }
  for(int i=0;i<g2.mu.n;i++) {
    if(g2.L(i,i) != 0.0) {
      logdet2 += Log(g2.L(i,i));
      d2++;
    }
  }
  
  Matrix cov1,cov2;
  g1.getCovariance(cov1);
  g2.getCovariance(cov2);
  return LogDotProduct(g1.mu,cov1,g2.mu,cov2) + Half*(Half*Log(TwoPi)*Real(d1+d2) + Half*(logdet1 + logdet2));
}

void GaussianMixtureModel::Cluster(const GaussianMixtureModel& gmm_orig,int numComponents)
{
  GaussianMixtureModelRaw gmmr;
  gmmr.Cluster(gmm_orig,numComponents);
  gmmr.Get(*this);
}


void GaussianMixtureModelRaw::Cluster(const GaussianMixtureModel& gmm_orig,int numComponents)
{
  GaussianMixtureModel clusters = gmm_orig;
  //initialize centers
  clusters.Resample(numComponents);
  numComponents = (int)clusters.phi.size();
  //probability that i'th original gaussian belongs to j'th new gaussian
  vector<vector<Real> > p(gmm_orig.gaussians.size());
  for(size_t i=0;i<p.size();i++)
    p[i].resize(numComponents);

  //compute probabilities of belonging to gaussians
  for(size_t i=0;i<p.size();i++) {
    for(int j=0;j<numComponents;j++) 
      //p[i][j] = LogCosAngle(orig.gaussians[i],clusters.gaussians[j]);
      p[i][j] = -gmm_orig.gaussians[i].klDivergence(clusters.gaussians[j]);
  }

  //find the most likely assignment
  vector<int> assignments(p.size(),0);
  for(size_t i=0;i<p.size();i++) {
    for(int j=0;j<numComponents;j++) {
      if(p[i][j] > p[i][assignments[i]])
	assignments[i] = j;
    }
  }
  /*
    LOG4CXX_INFO(KrisLibrary::logger(),"assignments:");
    for(size_t i=0;i<p.size();i++)
      LOG4CXX_INFO(KrisLibrary::logger(),""<<assignments[i]);
    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"probabilities:");
    for(size_t i=0;i<p.size();i++) {
      for(size_t j=0;j<p[i].size();j++)
	LOG4CXX_INFO(KrisLibrary::logger(),""<<p[i][j]);
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
    }
  */

  phi.resize(numComponents);
  means.resize(numComponents);
  covariances.resize(numComponents);
  for(int j=0;j<numComponents;j++) {
    phi[j]=0.0;
    means[j].setZero();
    covariances[j].setZero();
  }
  for(size_t i=0;i<p.size();i++) {
    int j=assignments[i];
    phi[j] += gmm_orig.phi[i];
    means[j].madd(gmm_orig.gaussians[i].mu,gmm_orig.phi[i]);
  }
  for(int j=0;j<numComponents;j++) 
    if(phi[j] != 0.0)
      means[j] /= phi[j];
  Vector diff;
  Matrix cov;
  for(size_t i=0;i<p.size();i++) {
    int j=assignments[i];
    diff.sub(gmm_orig.gaussians[i].mu,means[j]);
    gmm_orig.gaussians[i].getCovariance(cov);
    /*
    if(!IsFinite(cov)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Cluster: original covariance "<<i<<" became non-finite");
      LOG4CXX_INFO(KrisLibrary::logger(),cov);
      KrisLibrary::loggerWait();
    }*/
    covariances[j].madd(cov,gmm_orig.phi[i]);
    /*
    if(!IsFinite(covariances[j])) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Cluster: covariance "<<j<<" became non-finite after add");
      LOG4CXX_INFO(KrisLibrary::logger(),cov);
      KrisLibrary::loggerWait();
    }
    */
    for(int m=0;m<cov.m;m++)
      for(int n=0;n<cov.n;n++)
	covariances[j](m,n) += diff[m]*diff[n]*gmm_orig.phi[i];
    /*
    if(!IsFinite(covariances[j])) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Cluster: covariance "<<j<<" became non-finite after outer product");
      LOG4CXX_INFO(KrisLibrary::logger(),diff);
      LOG4CXX_INFO(KrisLibrary::logger(),covariances[j]);
      KrisLibrary::loggerWait();
    }
    */
  }
  for(int j=0;j<numComponents;j++) {
    if(phi[j] != 0.0)
      covariances[j] /= phi[j];
    else
      covariances[j].setIdentity();
    /*
    if(!IsFinite(covariances[j])) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Cluster: covariance "<<j<<" became non-finite");
      LOG4CXX_INFO(KrisLibrary::logger(),covariances[j]);
      LOG4CXX_INFO(KrisLibrary::logger(),phi[j]);
      KrisLibrary::loggerWait();
    }
    */
  }

  Normalize(phi);

  for(size_t i=0;i<phi.size();i++)
    if(phi[i] == 0.0) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Dropping gaussian "<<i);
      phi[i] = phi.back();
      swap(means[i],means.back());
      swap(covariances[i],covariances.back());
      phi.resize(phi.size()-1);
      means.resize(means.size()-1);
      covariances.resize(covariances.size()-1);
      i--;
    }
}




ostream& operator << (ostream& out,const GaussianMixtureModel& gmm)
{
  GaussianMixtureModelRaw gmmraw(gmm);
  return out<<gmmraw;
}

ostream& operator << (ostream& out,const GaussianMixtureModelRaw& gmm)
{
  out<<gmm.means.size()<<" "<<gmm.means[0].n<<endl;
  for(size_t i=0;i<gmm.phi.size();i++) {
    out<<endl;
    out<<gmm.phi[i]<<endl;
    out<<gmm.means[i]<<endl;
    out<<gmm.covariances[i]<<endl;
  }
  return out;
}

istream& operator >> (istream& in,GaussianMixtureModel& gmm)
{
  GaussianMixtureModelRaw gmmraw;
  in>>gmmraw;
  if(!in) return in;
  gmmraw.Get(gmm);
  return in;
}

istream& operator >> (istream& in,GaussianMixtureModelRaw& gmm)
{
  int k,d;
  in>>k>>d;
  if(!in) return in;
  if(k < 0 || d < 0) { in.setstate(ios::badbit); return in; }
  gmm.Resize(k,d);
  for(int i=0;i<k;i++) {
    in>>gmm.phi[i];
    in>>gmm.means[i];
    in>>gmm.covariances[i];
    if(!in) return in;
  }
  return in;
}

} //namespace Statistics
