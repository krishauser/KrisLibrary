#include <KrisLibrary/Logger.h>
#include "GaussianHMM.h"
#include "OnlineMoments.h"
#include <math/indexing.h>
#include <fstream>
using namespace Math;
using namespace std;

namespace Statistics {

  //const static Real covarianceRegularizationFactor = 1e-3;
  const static Real covarianceRegularizationFactor = 1;

  //These are defined in GaussianMixtureRegression.cpp
  Real Normalize(vector<Real>& w);
  Real NormalizeProbability(Vector& w);
  Real ExpNormalize(vector<Real>& w);
  Real ExpNormalize(Vector& w);
  Real logDot(const Gaussian<Real>& g,const Vector& xmean,const Matrix& xcov);

  inline void AddOuterProduct(Matrix& A,const Vector& u,const Vector& v)
  {
    assert(A.m == u.n);
    assert(A.n == v.n);
    for(int i=0;i<u.n;i++)
      for(int j=0;j<v.n;j++)
	A(i,j) += u(i)*v(j);
  }

GaussianHMM::GaussianHMM()
{}

GaussianHMM::GaussianHMM(int k,int d)
{
  Resize(k,d);
}

GaussianHMM::GaussianHMM(const GaussianMixtureModel& gmm)
{
  discretePrior = gmm.phi;
  transitionMatrix.resize(gmm.phi.size(),gmm.phi.size());
  for(size_t i=0;i<gmm.phi.size();i++)
    transitionMatrix.copyCol(i,Vector(gmm.phi));
  /*
  transitionMatrix *= 0.1;
  for(size_t i=0;i<gmm.phi.size();i++)
    transitionMatrix(i,i) += 0.9;
  */
  emissionModels = gmm.gaussians;
}

int GaussianHMM::NumDims() const
{
  if(emissionModels.empty()) return 0;
  return emissionModels[0].mu.n;
}

void GaussianHMM::Resize(int k,int d)
{
  discretePrior.resize(k,1.0/Real(k));
  transitionMatrix.resize(k,k,1.0/Real(k));
  emissionModels.resize(k);
  for(int i=0;i<k;i++)
    emissionModels[i].resize(d);
}

void GaussianHMM::SetUniformTransitions()
{
  transitionMatrix.set(1.0/Real(discretePrior.size()));
}

void GaussianHMM::SetUniformExitProbabilities(Real pExit)
{
  for(int i=0;i<discretePrior.n;i++) {
    for(int j=0;j<discretePrior.n;j++) {
      if(i==j)
	transitionMatrix(i,j) = 1.0-pExit;
      else
	transitionMatrix(i,j) = pExit/(discretePrior.n-1);
    }
  }
}

bool GaussianHMM::TrainEM(const vector<vector<Vector> >& examples,Real& tol,int maxIters,int verbose)
{
  vector<Real> flatWeights;
  vector<Vector> flatExamples;
  for(size_t i=0;i<examples.size();i++)
    flatExamples.insert(flatExamples.end(),examples[i].begin(),examples[i].end());
  flatWeights.resize(flatExamples.size());

  //weights of each example belonging to each class 
  vector<Vector> w;
  Vector p0accum;
  Matrix taccum;
  vector<OnlineMoments> momentaccum(discretePrior.n);
  Real lltotal_old = -Inf;
  for(int iters=0;iters<maxIters;iters++) {
    Real lltotal=0.0;
    for(size_t i=0;i<examples.size();i++)
      lltotal += LogLikelihood(examples[i]);
    LOG4CXX_INFO(KrisLibrary::logger(),"Log likelihood of data: "<<lltotal);
    if(Abs(lltotal-lltotal_old) < tol) {
      return true;
    }
    if((iters+1) % 10 == 0) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Saving progress to temp.ghmm\n");
      ofstream out("temp.ghmm");
      out<<*this<<endl;
      out.close();
    }

    LOG4CXX_INFO(KrisLibrary::logger(),"Performing EM iteration "<<iters);

    //interleaved E/M steps to reduce memory consumption
    p0accum.resize(discretePrior.n);
    p0accum.setZero();
    taccum.resize(discretePrior.n,discretePrior.n);
    taccum.setZero();
    for(int k=0;k<discretePrior.n;k++) {
      momentaccum[k].Clear();
    }
    for(size_t i=0;i<examples.size();i++) {
      //E step
      w.resize(examples[i].size());
      Posterior(examples[i],w,taccum);

      //M accumulations...
      //M discrete prior
      p0accum += w[0];
      //M transition is already taken care of in taccum
      //M moments
      for(size_t j=1;j<w.size();j++) {
	for(int k=0;k<discretePrior.n;k++) {
	  momentaccum[k].AddPoint(examples[i][j-1],w[j](k));
	}
      }
    }
    discretePrior = p0accum;
    NormalizeProbability(discretePrior);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Priors: "<<discretePrior);
    //now maximize transition matrix
    transitionMatrix = taccum;
    //LOG4CXX_INFO(KrisLibrary::logger(),"Counts: "<<transitionMatrix);
    for(int i=0;i<transitionMatrix.m;i++) {
      Vector temp;
      transitionMatrix.getColRef(i,temp);
      NormalizeProbability(temp);
    }
    //now maximize emission probabilities
    for(size_t i=0;i<emissionModels.size();i++) {
      if(momentaccum[i].sumWeight == 0.0 ) {
	emissionModels[i].mu.setZero();
	emissionModels[i].L.setIdentity();
	emissionModels[i].L *= covarianceRegularizationFactor;
      }
      else {
	//add a small amount to the diagonal
	for(int k=0;k<examples[0][0].n;k++) 
	  momentaccum[i].cov(k,k) += Sqr(covarianceRegularizationFactor);
	emissionModels[i].mu = momentaccum[i].mean;
	bool res=emissionModels[i].setCovariance(momentaccum[i].cov,verbose);
	if(!res) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error setting gaussian "<<i);
	  return false;
	}
	int nzero = 0;
	for(int k=0;k<examples[0][0].n;k++) {
	  if(emissionModels[i].L(k,k) <= covarianceRegularizationFactor) {
	    nzero++;
	    emissionModels[i].L(k,k) = covarianceRegularizationFactor;
	  }
	}
	if(nzero > 0) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian "<<i<<" became degenerate on "<<nzero);
	}
      }
    }
  }
  return true;
}

  //bool GaussianHMM::TrainDiagonalEM(const vector<vector<Vector> >& examples,Real& tol,int maxIters,int verbose);

Real GaussianHMM::Probability(const Vector& p0,const Vector& obs) const
{
  Real p = 0.0;
  for(size_t k=0;k<emissionModels.size();k++)
    p += p0[k]*emissionModels[k].probability(obs);
  return p;
}

Real GaussianHMM::LogLikelihood(const vector<int>& dstates,const vector<Vector>& observations) const
{
  assert(dstates.size()==observations.size()+1);
  if(dstates.empty()) return 0.0;
  assert(0 <= dstates[0] && dstates[0] < discretePrior.size());
  Real ll=Log(discretePrior[dstates[0]]);
  for(size_t i=1;i<dstates.size();i++) {
    assert(0 <= dstates[i] && dstates[i] < transitionMatrix.m);
    ll += Log(transitionMatrix(dstates[i],dstates[i-1]));
    ll += emissionModels[dstates[i]].logProbability(observations[i-1]);
  }
  return ll;
}

Real GaussianHMM::LogLikelihood(const vector<Vector>& observations) const
{
  Real ll=0.0;
  Vector p = discretePrior,temp;
  for(size_t i=0;i<observations.size();i++) {
    Predict(p,temp);
    Real pi = Probability(temp,observations[i]);
    ll += Log(pi);
    p = temp;
  }
  return ll;
}


void GaussianHMM::Predict(const Vector& p0,Vector& pt,int numsteps) const
{
  //TODO: repeated squaring if the number of steps is large enough
  Vector temp;
  pt = p0;
  for(int i=0;i<numsteps;i++) {
    transitionMatrix.mul(pt,temp);
    pt = temp;
  }
}

void GaussianHMM::Update(const Vector& p0,const Vector& obs,Vector& pobs) const
{
  pobs=p0;
  for(size_t i=0;i<emissionModels.size();i++) {
    if(pobs[i] != 0.0)
      pobs[i] *= emissionModels[i].probability(obs);
  }
  Real sumw=NormalizeProbability(pobs);
  if(sumw == 0) {
    //underflow occurred.  redo, with log probabilities instead
    for(int i=0;i<p0.n;i++)
      pobs[i] = Log(p0[i]);
    for(size_t i=0;i<emissionModels.size();i++) 
      if(IsInf(pobs[i]) != -1)
	 pobs[i] += emissionModels[i].logProbability(obs);
    ExpNormalize(pobs);
  }
}

void GaussianHMM::ObservationDistribution(const Vector& p0,GaussianMixtureModel& model) const
{
  model.phi = p0;
  model.gaussians = emissionModels;
}

void GaussianHMM::Filter(const Vector& p0,const Vector& obs,Vector& pnext) const
{
  assert(p0.n == transitionMatrix.n);
  Vector temp;
  transitionMatrix.mul(p0,temp);
  Update(temp,obs,pnext);
}

void GaussianHMM::MAP(const vector<Vector>& observations,vector<int>& dstates) const
{
  dstates.resize(observations.size()+1);
  vector<Vector> maxp(observations.size()+1);
  vector<vector<int> > pred(observations.size()+1);
  for(size_t i=0;i<observations.size();i++) {
    maxp[i].resize(discretePrior.size());
    pred[i].resize(discretePrior.size(),-1);
  }
  maxp[0] = discretePrior;

  for(size_t i=0;i<observations.size();i++) {
    for(int k=0;k<discretePrior.size();k++) {
      Real best = -Inf;
      int bestInd = 0;
      for(int j=0;j<discretePrior.size();j++)
	if(maxp[i][j]*transitionMatrix(k,j) > best) {
	  best = maxp[i][j]*transitionMatrix(k,j);
	  bestInd = j;
	}
      pred[i+1][k]=bestInd;
      maxp[i+1][k]=best*emissionModels[k].probability(observations[i]);
    }
    //normalize maxp for better numerical performance
    NormalizeProbability(maxp[i+1]);
  }
  //do backwards pass
  maxp.back().maxElement(&dstates.back());
  for(int i=(int)maxp.size()-1;i>0;i--) 
    dstates[i-1] = pred[i][dstates[i]];
}

void GaussianHMM::Posterior(const vector<Vector>& observations,vector<Vector>& pstate) const
{
  vector<Vector> pforward(observations.size()+1),pbackward(observations.size()+1);
  Matrix obsMat(observations.size(),discretePrior.n);
  for(size_t i=0;i<observations.size();i++)
    for(int j=0;j<discretePrior.n;j++)
      obsMat(i,j) = emissionModels[j].probability(observations[i]);
  //forward pass
  pforward[0] = discretePrior;
  for(size_t i=0;i<observations.size();i++) {
    //Filter(pforward[i],observations[i],pforward[i+1]);
    transitionMatrix.mul(pforward[i],pforward[i+1]);
    for(size_t j=0;j<emissionModels.size();j++) 
      pforward[i+1](j) *= obsMat(i,j);
    Real sumw=NormalizeProbability(pforward[i+1]);
    assert(sumw != 0.0);
  }
  //backward pass
  pbackward.back().resize(discretePrior.size(),1.0);
  for(int i=(int)observations.size()-1;i>=0;i--) {
    //probability of o[i+1],...,o[T] given X[i] = k 
    //=Sum_j P(X[i+1]=j|X[i]=k)*P(o[i+1]|X[i+1]=j)*P(o[i+2],...,o[T]|X[i+1]=j)
    Vector pobs(discretePrior.size());
    for(int j=0;j<pobs.n;j++)
      pobs[j] = pbackward[i+1][j]*obsMat(i,j);
    transitionMatrix.mulTranspose(pobs,pbackward[i]);
    NormalizeProbability(pbackward[i]);
    //for(size_t k=0;j<discretePrior.size();k++) 
    //  for(size_t j=0;j<discretePrior.size();j++) 
    //    pbackward[i][k] += pobs[j]*transitionMatrix(j,k);
  }
  //combine via elementwise product and normalize
  pstate.resize(observations.size()+1);
  for(size_t i=0;i<=observations.size();i++) {
    pstate[i].resize(discretePrior.size()); 
    for(int k=0;k<pstate[i].n;k++)
      pstate[i][k] = pforward[i][k]*pbackward[i][k];
    NormalizeProbability(pstate[i]);
  }
}

void GaussianHMM::Posterior(const vector<Vector>& observations,vector<Vector>& pstate,Matrix& tstate) const
{
  vector<Vector> pforward(observations.size()+1),pbackward(observations.size()+1);
  Matrix obsMat(observations.size(),discretePrior.n);
  for(size_t i=0;i<observations.size();i++)
    for(int j=0;j<discretePrior.n;j++)
      obsMat(i,j) = emissionModels[j].probability(observations[i]);
  //forward pass
  pforward[0] = discretePrior;
  for(size_t i=0;i<observations.size();i++) {
    //Filter(pforward[i],observations[i],pforward[i+1]);
    transitionMatrix.mul(pforward[i],pforward[i+1]);
    for(size_t j=0;j<emissionModels.size();j++) 
      pforward[i+1](j) *= obsMat(i,j);
    Real sumw=NormalizeProbability(pforward[i+1]);
    assert(sumw != 0.0);
  }
  //backward pass
  pbackward.back().resize(discretePrior.size(),1.0);
  for(int i=(int)observations.size()-1;i>=0;i--) {
    //probability of o[i+1],...,o[T] given X[i] = k 
    //=Sum_j P(X[i+1]=j|X[i]=k)*P(o[i+1]|X[i+1]=j)*P(o[i+2],...,o[T]|X[i+1]=j)
    Vector pobs(discretePrior.size());
    for(int j=0;j<pobs.n;j++)
      pobs[j] = pbackward[i+1][j]*obsMat(i,j);
    transitionMatrix.mulTranspose(pobs,pbackward[i]);
    NormalizeProbability(pbackward[i]);
    //for(size_t k=0;j<discretePrior.size();k++) 
    //  for(size_t j=0;j<discretePrior.size();j++) 
    //    pbackward[i][k] += pobs[j]*transitionMatrix(j,k);
  }
  //combine via elementwise product and normalize
  pstate.resize(observations.size()+1);
  for(size_t i=0;i<=observations.size();i++) {
    pstate[i].resize(discretePrior.size()); 
    for(int k=0;k<pstate[i].n;k++)
      pstate[i][k] = pforward[i][k]*pbackward[i][k];
    NormalizeProbability(pstate[i]);
  }

  //compute transition models
  Matrix temp(discretePrior.n,discretePrior.n);
  if(tstate.isEmpty()) tstate.resize(discretePrior.n,discretePrior.n,0.0);
  for(size_t i=0;i<observations.size();i++) {
    Real sum=0.0;
    for(int k=0;k<discretePrior.n;k++) {
      Real pobs=pbackward[i+1][k]*obsMat(i,k);
      for(int j=0;j<discretePrior.n;j++) {
	temp(k,j) = pforward[i][j]*transitionMatrix(k,j)*pobs;
	sum += temp(k,j);
      }
    }
    temp *= 1.0/sum;
    tstate += temp;
  }
}


std::ostream& operator << (std::ostream& out,const GaussianHMM& gmm)
{
  out<<gmm.discretePrior<<endl<<endl;
  out<<gmm.transitionMatrix<<endl<<endl;
  assert((int)gmm.emissionModels.size() == gmm.discretePrior.size());
  Matrix cov;
  for(size_t i=0;i<gmm.emissionModels.size();i++) {
    out<<gmm.emissionModels[i].mu<<endl;
    gmm.emissionModels[i].getCovariance(cov);
    out<<cov<<endl<<endl; 
  }
  return out;
}

std::istream& operator >> (std::istream& in,GaussianHMM& gmm)
{
  in>>gmm.discretePrior;
  if(!in) return in;
  in>>gmm.transitionMatrix;
  if(!in) return in;
  gmm.emissionModels.resize(gmm.discretePrior.size());
  Matrix cov;
  for(size_t i=0;i<gmm.emissionModels.size();i++) {
    in>>gmm.emissionModels[i].mu;
    if(!in) return in;
    in>>cov;
    if(!in) return in;
    gmm.emissionModels[i].setCovariance(cov);
  }
  return in;
}


GaussianHMMRegression::GaussianHMMRegression()
{}

GaussianHMMRegression::GaussianHMMRegression(const GaussianHMM& _joint)
  :joint(_joint)
{}

void GaussianHMMRegression::SetXIndices(const std::vector<int>& _xindices)
{
  xindices = _xindices;
  yindices.resize(0);
  xEmissionModels.resize(joint.emissionModels.size());
  for(int i=0;i<joint.emissionModels[0].mu.n;i++) {
    int index=-1;
    for(size_t j=0;j<xindices.size();j++) {
      if(xindices[j] == i) {
	index = (int)j;
	break;
      }
    }
    if(index < 0) yindices.push_back(i);
  }
  assert(int(yindices.size() + xindices.size()) == joint.emissionModels[0].mu.n);
  for(size_t i=0;i<joint.emissionModels.size();i++) {
    xEmissionModels[i].setMarginalized(joint.emissionModels[i],yindices);
  }

  regressions.resize(joint.emissionModels.size());
  for(size_t i=0;i<regressions.size();i++)
    regressions[i].Set(joint.emissionModels[i],xindices,yindices);
}

Real GaussianHMMRegression::ProbabilityX(const Vector& p0,const Vector& x) const
{
  assert(p0.n == (int)xEmissionModels.size());
  Real p = 0.0;
  for(size_t i=0;i<xEmissionModels.size();i++)
    p += p0(i)*xEmissionModels[i].probability(x);
  return p;
}

void GaussianHMMRegression::Filter(const Vector& p0,const Vector& x,Vector& pnext) const
{
  assert(p0.n == (int)xEmissionModels.size());
  joint.Predict(p0,pnext);
  for(size_t i=0;i<xEmissionModels.size();i++)
    pnext(i) *= xEmissionModels[i].probability(x);
  NormalizeProbability(pnext);
}

void GaussianHMMRegression::GetYUnconditional(const Vector& p0,GaussianMixtureModelRaw& y) const
{
  assert(p0.n == (int)xEmissionModels.size());
  y.phi = p0;
  y.means.resize(xEmissionModels.size());
  y.covariances.resize(xEmissionModels.size());
  Matrix cov;
  for(size_t i=0;i<regressions.size();i++) {
    GetElements(joint.emissionModels[i].mu,yindices,y.means[i]);
    joint.emissionModels[i].getCovariance(cov);
    GetElements(cov,yindices,yindices,y.covariances[i]);
  }
}

void GaussianHMMRegression::GetY(const Vector& p0,const Vector& x,GaussianMixtureModelRaw& y) const
{
  assert(p0.n == (int)xEmissionModels.size());
  assert(x.n == (int)xindices.size());

  y.phi.resize(xEmissionModels.size());
  y.means.resize(xEmissionModels.size());
  y.covariances.resize(xEmissionModels.size());  
  int ny=(int)yindices.size();
  for(size_t i=0;i<regressions.size();i++) {
    if(p0(i) == 0) {
      y.phi[i] = -Inf;
      y.means[i].resize(ny,0.0);
      y.covariances[i].resize(ny,ny,0.0);
    }
    else {
      y.phi[i] = Log(p0(i)) + xEmissionModels[i].logProbability(x);
      regressions[i].GetY(x,y.means[i],y.covariances[i]);
    }
  }
  ExpNormalize(y.phi);
}


void GaussianHMMRegression::GetY(const Vector& p0,const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& y) const
{
  assert(p0.n == (int)xEmissionModels.size());
  assert(xmean.n == (int)xindices.size());
  assert(xcov.m == (int)xindices.size());
  assert(xcov.n == (int)xindices.size());
  y.phi.resize(xEmissionModels.size());
  y.means.resize(xEmissionModels.size());
  y.covariances.resize(xEmissionModels.size());  
  int ny=(int)yindices.size();
  for(size_t i=0;i<regressions.size();i++) {
    if(p0[i] == 0.0) {
      y.phi[i] = -Inf;
      y.means[i].resize(ny,Zero);
      y.covariances[i].resize(ny,ny,Zero);
    }
    else {
      y.phi[i] = logDot(xEmissionModels[i],xmean,xcov)+Log(p0[i]);
      regressions[i].GetY(xmean,xcov,y.means[i],y.covariances[i]);
    }
  }
  ExpNormalize(y.phi);
}

void GaussianHMMRegression::GetJoint(const Vector& p0,const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& xy) const
{
  assert(p0.n == (int)xEmissionModels.size());
  assert(xmean.n == (int)xindices.size());
  assert(xcov.m == (int)xindices.size());
  assert(xcov.n == (int)xindices.size());
  xy.phi.resize(xEmissionModels.size());
  xy.means.resize(xEmissionModels.size());
  xy.covariances.resize(xEmissionModels.size());  
  int nxy = (int)(xindices.size()+yindices.size());
  for(size_t i=0;i<regressions.size();i++) {
    if(p0[i] == 0.0) {
      xy.phi[i] = -Inf;
      xy.means[i].resize(nxy,Zero);
      xy.covariances[i].resize(nxy,nxy,Zero);
    }
    else {
      xy.phi[i] = logDot(xEmissionModels[i],xmean,xcov)+Log(p0[i]);
      regressions[i].GetJoint(xmean,xcov,xy.means[i],xy.covariances[i]);
    }
  }
  ExpNormalize(xy.phi);
}

} //namespace Statistics
