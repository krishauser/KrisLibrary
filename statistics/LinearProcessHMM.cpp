#include <KrisLibrary/Logger.h>
#include "LinearProcessHMM.h"
#include <math/sample.h>
#include <math/indexing.h>
#include <math/linalgebra.h>
#include <fstream>
#include <iostream>
using namespace Math;
using namespace std;

namespace Statistics {

  #define PRINT_PREDICT_FIT 1
  #define NAIVE_PREDICT_FIT_A 1
  #define NAIVE_PREDICT_FIT_SIGMA 0
  //const static int gTrainEMPredictionHorizon = 10;
  const static int gTrainEMPredictionHorizon = 1;

  const static Real covarianceRegularizationFactor = 1e-3;
  //const static Real covarianceRegularizationFactor = 1;


  //These are defined in GaussianMixtureRegression.cpp
  Real Normalize(vector<Real>& w);
  Real NormalizeProbability(Vector& w);
  Real ExpNormalize(vector<Real>& w);
  Real ExpNormalize(Vector& w);
  Real logDot(const Gaussian<Real>& g,const Vector& xmean,const Matrix& xcov);


Real LinearProcess::Probability(const Vector& x,const Vector& y) const
{
  Vector temp;
  A.mul(x,temp);
  temp -= y;
  temp.inplaceNegative();
  return error.probability(temp);
}

Real LinearProcess::LogProbability(const Vector& x,const Vector& y) const
{
  Vector temp;
  A.mul(x,temp);
  temp -= y;
  temp.inplaceNegative();
  return error.logProbability(temp);
}

void LinearProcess::GetY(const Vector& x,Gaussian<Real>& y) const
{
  Assert(x.n == A.n);
  A.mul(x,y.mu);
  y.mu += error.mu;
  y.L = error.L;
}

void LinearProcess::GetY(const Vector& x,Vector& ymean,Matrix& ycov) const
{
  Assert(x.n == A.n);
  A.mul(x,ymean);
  ymean += error.mu;
  error.getCovariance(ycov);
}

void LinearProcess::GetY(const Vector& xmean,const Matrix& xcov,Vector& ymean,Matrix& ycov) const
{
  Assert(xmean.n == A.n);
  A.mul(xmean,ymean);
  ymean += error.mu;
  error.getCovariance(ycov);

  Matrix temp1,temp2;
  temp1.mul(A,xcov);
  temp2.mulTransposeB(temp1,A);
  ycov += temp2;
}

void LinearProcess::GetJoint(const Vector& xmean,const Matrix& xcov,Vector& xymean,Matrix& xyvar) const
{
  Assert(xmean.n == A.n);
  GaussianTransformJoint(xmean,xcov,A,error.mu,xymean,xyvar);
  Matrix ycov,ecov;
  ycov.setRef(xyvar,xmean.n,xmean.n,1,1,A.m,A.m);
  error.getCovariance(ecov);
  ycov += ecov;
}

bool LinearProcess::WeightedOLS(const vector<Vector>& x,const vector<Vector>& y,const std::vector<Real>& weights,int verbose)
{
  Assert(x.size()==y.size());
  Assert(x.size()==weights.size());
  vector<Vector> xy(x.size());
  for(size_t i=0;i<x.size();i++) {
    xy[i].resize(x[i].n+y[i].n);
    xy[i].copySubVector(0,x[i]);
    xy[i].copySubVector(x[i].n,y[i]);
  }
  vector<int> xindices(x[0].n),yindices(y[0].n);
  for(int i=0;i<x[0].n;i++) xindices[i]=i;
  for(int i=0;i<y[0].n;i++) yindices[i]=i+x[0].n;
  Gaussian<Real> g;
  if(!g.setMaximumLikelihood(xy,weights,verbose)) return false;

  Matrix cov;
  g.getCovariance(cov);

  GaussianRegression reg;
  reg.Set(g,xindices,yindices);
  reg.GetLinearEquation(A,error.mu);
  for(int i=0;i<reg.ycov.n;i++)
    reg.ycov(i,i) += covarianceRegularizationFactor;
  error.setCovariance(reg.ycov);
  return true;
}

LinearProcessHMM::LinearProcessHMM()
{}

LinearProcessHMM::LinearProcessHMM(int k,int d)
{
  Resize(k,d);
}

LinearProcessHMM::LinearProcessHMM(const GaussianMixtureModel& gmm)
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
  continuousPriors = gmm.gaussians;
  emissionModels.resize(gmm.gaussians.size());
  for(size_t i=0;i<emissionModels.size();i++) {
    emissionModels[i].A.resize(gmm.gaussians[0].mu.n,gmm.gaussians[0].mu.n);
    emissionModels[i].A.setZero();
    emissionModels[i].error = gmm.gaussians[i];
  }
}

int LinearProcessHMM::NumDims() const
{
  if(continuousPriors.empty()) return 0;
  return continuousPriors[0].mu.n;
}

void LinearProcessHMM::Resize(int k,int d)
{
  discretePrior.resize(k,1.0/Real(k));
  transitionMatrix.resize(k,k,1.0/Real(k));
  continuousPriors.resize(k);
  emissionModels.resize(k);
  for(int i=0;i<k;i++) {
    continuousPriors[i].resize(d);
    emissionModels[i].A.resize(d,d);
    emissionModels[i].error.resize(d);
  }
}

void LinearProcessHMM::SetUniformTransitions()
{
  transitionMatrix.set(1.0/Real(discretePrior.size()));
}

void LinearProcessHMM::SetUniformExitProbabilities(Real pExit)
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

bool LinearProcessHMM::TrainEM(const vector<vector<Vector> >& examples,Real& tol,int maxIters,int verbose)
{
  vector<Real> flatWeights,initWeights;
  vector<Vector> flatExamples,flatExamplesPrev,initExamples;
  int predictionHorizon = gTrainEMPredictionHorizon;
  for(size_t i=0;i<examples.size();i++) {
    flatExamples.insert(flatExamples.end(),examples[i].begin()+predictionHorizon,examples[i].end());
    flatExamplesPrev.insert(flatExamplesPrev.end(),examples[i].begin(),examples[i].end()-predictionHorizon);
  }
  flatWeights.resize(flatExamples.size());
  for(size_t i=0;i<examples.size();i++)
    initExamples.push_back(examples[i][0]);
  initWeights.resize(initExamples.size());

  //weights of each example belonging to each class 
  vector<vector<Vector> > w(examples.size());
  Matrix taccum;
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
      LOG4CXX_INFO(KrisLibrary::logger(),"Saving progress to temp.lphmm\n");
      ofstream out("temp.lphmm");
      out<<*this<<endl;
      out.close();
    }

    LOG4CXX_INFO(KrisLibrary::logger(),"Performing E step iteration "<<iters);

    //E step
    taccum.resize(discretePrior.n,discretePrior.n);
    taccum.setZero();
    for(size_t i=0;i<examples.size();i++) {
      Posterior(examples[i],w[i],taccum);
    }
    //M step
    LOG4CXX_INFO(KrisLibrary::logger(),"Performing M step...\n");
    //first maximize discrete prior
    discretePrior.setZero();
    for(size_t i=0;i<examples.size();i++) {
      discretePrior += w[i][0];
      /*
      LOG4CXX_INFO(KrisLibrary::logger(),"Initialization probability "<<i<<": "<<w[i][0]);
      LOG4CXX_INFO(KrisLibrary::logger(),"Emission probabilities: ");
      for(int j=0;j<discretePrior.n;j++)
	LOG4CXX_INFO(KrisLibrary::logger(),""<<emissionModels[j].probability(examples[i][0]));
      LOG4CXX_INFO(KrisLibrary::logger(),"\n");
      */
    }
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
    //now maximize continuous priors
    for(size_t i=0;i<emissionModels.size();i++) {
      for(size_t k=0;k<w.size();k++) 
	initWeights[k] = w[k][0](i);
      continuousPriors[i].setMaximumLikelihood(initExamples,initWeights,verbose);
      for(int j=0;j<continuousPriors[i].L.n;j++)
	if(continuousPriors[i].L(j,j) < covarianceRegularizationFactor)
	  continuousPriors[i].L(j,j) = covarianceRegularizationFactor;
    }
    //now maximize emission probabilities
    for(size_t i=0;i<emissionModels.size();i++) {
      int m=0;
      for(size_t k=0;k<w.size();k++) 
	for(size_t j=(size_t)predictionHorizon;j<w[k].size();j++) 
	  flatWeights[m++]=w[k][j](i);
      Assert(m == (int)flatWeights.size());
      if(!emissionModels[i].WeightedOLS(flatExamplesPrev,flatExamples,flatWeights,verbose)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error doing OLS\n");
	emissionModels[i].A.setIdentity();
	emissionModels[i].error.mu.setZero();
	emissionModels[i].error.L.setIdentity();
	emissionModels[i].error.L *= covarianceRegularizationFactor;
	//LOG4CXX_ERROR(KrisLibrary::logger(),"Terminated by error doing OLS\n");
	//return false;
      }
      else {
	//downscale by predictionHorizon
	if(predictionHorizon != 1) {
#if PRINT_PREDICT_FIT
	  LOG4CXX_INFO(KrisLibrary::logger(),"Prediction A");
	  LOG4CXX_INFO(KrisLibrary::logger(),emissionModels[i].A);
#endif //PRINT_PREDICT_FIT

	  Matrix Ades = emissionModels[i].A;
	  vector<Matrix> Ai(predictionHorizon);
	  Matrix temp;
#if NAIVE_PREDICT_FIT_A
	  //Naive method
	  for(int p=0;p<emissionModels[i].A.m;p++)
	    emissionModels[i].A(p,p) = 1.0 + (emissionModels[i].A(p,p)-1.0)/predictionHorizon;
	  for(int p=0;p<emissionModels[i].A.m;p++)
	    for(int q=0;q<emissionModels[i].A.m;q++)
	      if(p!=q) 
		emissionModels[i].A(p,q) = emissionModels[i].A(p,q)/predictionHorizon;

#else
	  for(int p=0;p<emissionModels[i].A.m;p++)
	    emissionModels[i].A(p,p) = Pow(Abs(emissionModels[i].A(p,p)),1.0/predictionHorizon);
	  for(int p=0;p<emissionModels[i].A.m;p++)
	    for(int q=0;q<emissionModels[i].A.n;q++) {
	      if(p!=q) {
		Real scale = 0.0;
		for(int j=0;j<predictionHorizon;j++)
		  scale += Pow(emissionModels[i].A(p,p),j)*Pow(emissionModels[i].A(q,q),predictionHorizon-j);
		if(scale != 0.0)
		  emissionModels[i].A(p,q) = emissionModels[i].A(p,q)/scale;
	      }
	    }
	  Ai[0]=emissionModels[i].A;
	  for(int j=1;j<predictionHorizon;j++) {
	    temp.mul(Ai[j-1],emissionModels[i].A);
	    Ai[j]=temp;
	  }
	  for(int p=0;p<emissionModels[i].A.m;p++) {
	    Real kscale = Ades(p,p)/temp(p,p);
	    Real scale = Pow(kscale,1.0/predictionHorizon);
	    if(!IsFinite(scale) || scale < 0.0) {
	      LOG4CXX_INFO(KrisLibrary::logger(),"Sign change "<<scale);
	    }
	    else {
	      scale = Sqrt(scale);
	      for(int q=0;q<emissionModels[i].A.m;q++) {
		emissionModels[i].A(p,q) *= scale;
		emissionModels[i].A(q,p) *= scale;
	      }
	    }
	  }
#endif // NAIVE_PREDICT_FIT

	  Ai[0]=emissionModels[i].A;
	  Matrix Asum(emissionModels[i].A.m,emissionModels[i].A.n);
	  Asum.setIdentity();
	  Ai[0]=emissionModels[i].A;
	  for(int j=1;j<predictionHorizon;j++) {
	    Asum += Ai[j-1];
	    temp.mul(Ai[j-1],emissionModels[i].A);
	    Ai[j]=temp;
	  }
#if PRINT_PREDICT_FIT
	  LOG4CXX_INFO(KrisLibrary::logger(),"one step (1/k'th power) ~=");
	  LOG4CXX_INFO(KrisLibrary::logger(),emissionModels[i].A);
	  LOG4CXX_INFO(KrisLibrary::logger(),"raised to k'th power =");
	  LOG4CXX_INFO(KrisLibrary::logger(),temp);
#endif //PRINT_PREDICT_FIT

	  Vector vtemp;
	  MatrixEquation eq(Asum,emissionModels[i].error.mu);
	  bool res=eq.Solve_SVD(vtemp);
	  if(!res) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't solve for mean\n");
	    emissionModels[i].error.mu /= predictionHorizon;
	  }
	  else {
#if PRINT_PREDICT_FIT
	    LOG4CXX_ERROR(KrisLibrary::logger(),"Original predicted mean"<<emissionModels[i].error.mu);
	    LOG4CXX_INFO(KrisLibrary::logger(),"New 1-step mean"<<vtemp);
#endif // PRINT_PREDICT_FIT
	    emissionModels[i].error.mu = vtemp;
	  }
	  //solve for matrix equation 
	  //sum_i=0^predictionHorizon-1 A^i Sigma A^T^i = Sigma'
	  Matrix ASigmaCoeffs(Asum.m*Asum.n,Asum.m*Asum.n,0.0);
	  Vector bSigmaCoeffs(Asum.m*Asum.n,0.0);
	  Matrix covdes;
	  emissionModels[i].error.getCovariance(covdes);
#if PRINT_PREDICT_FIT
	  LOG4CXX_INFO(KrisLibrary::logger(),"Predicted covariance");
	  LOG4CXX_INFO(KrisLibrary::logger(),covdes);
#endif
	  //stack rhs
	  for(int j=0;j<covdes.n;j++)
	    bSigmaCoeffs.copySubVector(j*covdes.m,covdes.col(j));
	  //stack Asigma
	  for(int p=0;p<covdes.m;p++) {
	    for(int q=0;q<covdes.n;q++) {
	      int row = p+covdes.m*q;
	      for(int r=0;r<covdes.m;r++) {
		for(int s=0;s<covdes.n;s++) {
		  int col = r+covdes.m*s;
		  if(p==r && q==s)
		    ASigmaCoeffs(row,col) += 1.0;
		  for(int k=0;k+1<predictionHorizon;k++)
		    ASigmaCoeffs(row,col) += Ai[k](r,p)*Ai[k](s,q);
		}
	      }
	    }
	  }
	  MatrixEquation sigmaeq(ASigmaCoeffs,bSigmaCoeffs);
	  Vector newsigma;
	  res=sigmaeq.Solve_SVD(newsigma);
	  if(!res) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't solve for covariance\n");
	    emissionModels[i].error.L /= Sqrt(Real(predictionHorizon));
	  }
	  else {
#if NAIVE_PREDICT_FIT_SIGMA
	    //Naive way
	    emissionModels[i].error.L /= Sqrt(Real(predictionHorizon));
	    emissionModels[i].error.getCovariance(covdes);
#else
	    //unstack newsigma
	    vtemp.clear();
	    for(int j=0;j<covdes.n;j++) {
	      vtemp.setRef(newsigma,j*covdes.m,1,covdes.m);
	      covdes.copyCol(j,vtemp);
	    }
	    Matrix oldL = emissionModels[i].error.L;
	    res=emissionModels[i].error.setCovariance(covdes);
	    if(!res) {
	      LOG4CXX_ERROR(KrisLibrary::logger(),"Error setting covariance matrix"<<covdes<<"\n");
	      emissionModels[i].error.L = oldL;
	      emissionModels[i].error.L /= Sqrt(Real(predictionHorizon));
	      emissionModels[i].error.getCovariance(covdes);
	    }
	    else {
#if PRINT_PREDICT_FIT
	      LOG4CXX_INFO(KrisLibrary::logger(),"Solved 1-step covariance");
	      LOG4CXX_INFO(KrisLibrary::logger(),covdes);
#endif // PRINT_PREDICT_FIT
	    }
#endif // NAIVE_PREDICT_FIT_SIGMA

#if PRINT_PREDICT_FIT
	    LOG4CXX_INFO(KrisLibrary::logger(),"Compounded covariance");
	    temp = covdes;
	    for(int j=0;j<predictionHorizon-1;j++) {
	      Matrix temp2,temp3;
	      temp2.mulTransposeA(Ai[j],covdes);
	      temp3.mul(temp2,Ai[j]);
	      temp += temp3;
	    }
	    LOG4CXX_INFO(KrisLibrary::logger(),temp);
	    KrisLibrary::loggerWait();
#endif //PRINT_PREDICT_FIT
	  }
	}
	for(int j=0;j<emissionModels[i].error.L.n;j++)
	  if(emissionModels[i].error.L(j,j) < covarianceRegularizationFactor)
	    emissionModels[i].error.L(j,j) = covarianceRegularizationFactor;
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),"Center "<<i<<": "<<emissionModels[i].mu);
    }
  }
  return true;
}

  //bool LinearProcessHMM::TrainDiagonalEM(const vector<vector<Vector> >& examples,Real& tol,int maxIters,int verbose);

Real LinearProcessHMM::Probability(const Vector& p0,const Vector& prevObs,const Vector& obs) const
{
  Real p = 0.0;
  for(size_t k=0;k<emissionModels.size();k++) {
    if(p0[k] > 0.0)
      p += p0[k]*emissionModels[k].Probability(prevObs,obs);
  }
  return p;
}

Real LinearProcessHMM::LogLikelihood(const vector<int>& dstates,const vector<Vector>& observations) const
{
  Assert(dstates.size()==observations.size());
  if(dstates.empty()) return 0.0;
  Assert(0 <= dstates[0] && dstates[0] < discretePrior.size());
  Real ll=Log(discretePrior[dstates[0]]);
  ll += continuousPriors[dstates[0]].logProbability(observations[0]);
  for(size_t i=1;i<dstates.size();i++) {
    Assert(0 <= dstates[i] && dstates[i] < transitionMatrix.m);
    ll += Log(transitionMatrix(dstates[i],dstates[i-1]));
    ll += emissionModels[dstates[i]].LogProbability(observations[i-1],observations[i]);
  }
  return ll;
}

Real LinearProcessHMM::LogLikelihood(const vector<Vector>& observations) const
{
  Real ll=0.0;
  Vector p = discretePrior,temp;
  Real pobs0=0;
  for(int i=0;i<discretePrior.n;i++) {
    if(discretePrior[i] > 0) 
      p(i) *= continuousPriors[i].probability(observations[0]);
  }
  pobs0 = NormalizeProbability(p);
  if(pobs0 == 0) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, probability of initial observation is 0\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Discrete distribution "<<discretePrior);
  }
  ll += Log(pobs0);
  for(size_t i=1;i<observations.size();i++) {
    Predict(p,temp);
    Real pi = Probability(temp,observations[i-1],observations[i]);
    if(pi == 0) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, probability of transition "<<i);
      LOG4CXX_INFO(KrisLibrary::logger(),"Discrete distribution "<<temp);
      LOG4CXX_INFO(KrisLibrary::logger(),"Observations: "<<observations[i-1]<<" "<<observations[i]);
      for(int j=0;j<discretePrior.n;j++)
	LOG4CXX_INFO(KrisLibrary::logger(),"Logp["<<j<<"] = "<<emissionModels[j].LogProbability(observations[i-1],observations[i]));
    }
    ll += Log(pi);
    Update(temp,observations[i-1],observations[i],p);
  }
  return ll;
}


void LinearProcessHMM::Predict(const Vector& p0,Vector& pt,int numsteps) const
{
  //TODO: repeated squaring if the number of steps is large enough
  Vector temp;
  pt = p0;
  for(int i=0;i<numsteps;i++) {
    transitionMatrix.mul(pt,temp);
    pt = temp;
  }
}

void LinearProcessHMM::Update(const Vector& p0,const Vector& prevObs,const Vector& obs,Vector& pobs) const
{
  pobs=p0;
  for(size_t i=0;i<emissionModels.size();i++) {
    if(pobs[i] != 0.0)
      pobs[i] *= emissionModels[i].Probability(prevObs,obs);
  }
  Real sumw=NormalizeProbability(pobs);
  if(sumw == 0) {
    //underflow occurred.  redo, with log probabilities instead
    for(int i=0;i<p0.n;i++)
      pobs[i] = Log(p0[i]);
    for(size_t i=0;i<emissionModels.size();i++) 
      if(IsInf(pobs[i]) != -1)
	pobs[i] += emissionModels[i].LogProbability(prevObs,obs);
    ExpNormalize(pobs);
  }
}

void LinearProcessHMM::ObservationDistribution(const Vector& p0,const Vector& prevObs,GaussianMixtureModel& model) const
{
  model.phi = p0;
  model.gaussians.resize(emissionModels.size());
  for(size_t i=0;i<emissionModels.size();i++)
    emissionModels[i].GetY(prevObs,model.gaussians[i]);
}

void LinearProcessHMM::Filter(const Vector& p0,const Vector& prevObs,const Vector& obs,Vector& pnext) const
{
  Assert(p0.n == transitionMatrix.n);
  Vector temp;
  transitionMatrix.mul(p0,temp);
  Update(temp,prevObs,obs,pnext);
}

void LinearProcessHMM::MAP(const vector<Vector>& observations,vector<int>& dstates) const
{
  dstates.resize(observations.size());
  vector<Vector> maxp(observations.size()+1);
  vector<vector<int> > pred(observations.size()+1);
  for(size_t i=0;i<observations.size();i++) {
    maxp[i].resize(discretePrior.size());
    pred[i].resize(discretePrior.size(),-1);
  }
  //initialize
  maxp[0] = discretePrior;
  for(int j=0;j<discretePrior.n;j++)
    maxp[0](j) *= continuousPriors[j].probability(observations[0]);
  NormalizeProbability(maxp[0]);

  //propagate
  for(size_t i=1;i<observations.size();i++) {
    for(int k=0;k<discretePrior.size();k++) {
      Real best = -Inf;
      int bestInd = 0;
      for(int j=0;j<discretePrior.size();j++)
	if(maxp[i-1][j]*transitionMatrix(k,j) > best) {
	  best = maxp[i-1][j]*transitionMatrix(k,j);
	  bestInd = j;
	}
      pred[i][k]=bestInd;
      maxp[i][k]=best*emissionModels[k].Probability(observations[i-1],observations[i]);
    }
    //normalize maxp for better numerical performance
    NormalizeProbability(maxp[i]);
  }
  //do backwards pass
  maxp.back().maxElement(&dstates.back());
  for(int i=(int)maxp.size()-1;i>0;i--) 
    dstates[i-1] = pred[i][dstates[i]];
}

void LinearProcessHMM::Posterior(const vector<Vector>& observations,vector<Vector>& pstate) const
{
  vector<Vector> pforward(observations.size()),pbackward(observations.size());
  Matrix obsMat(observations.size(),discretePrior.n);
  for(int j=0;j<discretePrior.n;j++)
    obsMat(0,j) = continuousPriors[j].probability(observations[0]);
  for(size_t i=1;i<observations.size();i++)
    for(int j=0;j<discretePrior.n;j++)
      obsMat(i,j) = emissionModels[j].Probability(observations[i-1],observations[i]);

  //forward pass
  pforward[0] = discretePrior;
  for(size_t j=0;j<emissionModels.size();j++) 
    pforward[0](j) *= obsMat(0,j);
  Real sumw=NormalizeProbability(pforward[0]);
  Assert(sumw != 0.0);
  for(size_t i=1;i<observations.size();i++) {
    transitionMatrix.mul(pforward[i-1],pforward[i]);
    for(size_t j=0;j<emissionModels.size();j++) 
      pforward[i](j) *= obsMat(i,j);
    Real sumw=NormalizeProbability(pforward[i]);
    Assert(sumw != 0.0);
  }

  //backward pass
  pbackward.back().resize(discretePrior.size(),1.0);
  for(int i=(int)observations.size()-1;i>0;i--) {
    //probability of o[i+1],...,o[T] given X[i] = k 
    //=Sum_j P(X[i+1]=j|X[i]=k)*P(o[i+1]|X[i+1]=j)*P(o[i+2],...,o[T]|X[i+1]=j)
    Vector pobs(discretePrior.size());
    for(int j=0;j<pobs.n;j++)
      pobs[j] = pbackward[i][j]*obsMat(i,j);
    transitionMatrix.mulTranspose(pobs,pbackward[i-1]);
    NormalizeProbability(pbackward[i-1]);
  }
  //combine via elementwise product and normalize
  pstate.resize(observations.size());
  for(size_t i=0;i<observations.size();i++) {
    pstate[i].resize(discretePrior.size()); 
    for(int k=0;k<pstate[i].n;k++)
      pstate[i](k) = pforward[i](k)*pbackward[i](k);
    NormalizeProbability(pstate[i]);
  }
}

void LinearProcessHMM::Posterior(const vector<Vector>& observations,vector<Vector>& pstate,Matrix& tstate) const
{
  vector<Vector> pforward(observations.size()),pbackward(observations.size());
  Matrix obsMat(observations.size(),discretePrior.n);
  for(int j=0;j<discretePrior.n;j++) {
    obsMat(0,j) = continuousPriors[j].logProbability(observations[0]);
  }
  for(size_t i=1;i<observations.size();i++)
    for(int j=0;j<discretePrior.n;j++)
      obsMat(i,j) = emissionModels[j].LogProbability(observations[i-1],observations[i]);

  //forward pass
  pforward[0].resize(discretePrior.n);
  for(size_t j=0;j<emissionModels.size();j++) 
    pforward[0](j) = Log(discretePrior(j)) + obsMat(0,j);
  Real sumw=ExpNormalize(pforward[0]);
  Assert(sumw != 0.0);
  for(size_t i=1;i<observations.size();i++) {
    transitionMatrix.mul(pforward[i-1],pforward[i]);
    for(size_t j=0;j<emissionModels.size();j++) 
      pforward[i](j) = Log(pforward[i](j)) + obsMat(i,j);
    Real sumw=ExpNormalize(pforward[i]);
    Assert(sumw != 0.0);
  }
  //backward pass
  pbackward.back().resize(discretePrior.size(),1.0);
  for(int i=(int)observations.size()-1;i>0;i--) {
    //probability of o[i+1],...,o[T] given X[i] = k 
    //=Sum_j Int_o[i] P(X[i+1]=j|X[i]=k)*P(o[i+1]|X[i+1]=j,o[i])*P(o[i+2],...,o[T]|X[i+1]=j)
    Vector pobs(discretePrior.size());
    for(int j=0;j<pobs.n;j++)
      pobs[j] = Log(pbackward[i][j])+obsMat(i,j);
    ExpNormalize(pobs);
    transitionMatrix.mulTranspose(pobs,pbackward[i-1]);
    NormalizeProbability(pbackward[i-1]);
  }
  //combine via elementwise product and normalize
  pstate.resize(observations.size());
  for(size_t i=0;i<observations.size();i++) {
    pstate[i].resize(discretePrior.size()); 
    for(int k=0;k<pstate[i].n;k++)
      pstate[i][k] = pforward[i][k]*pbackward[i][k];
    NormalizeProbability(pstate[i]);
  }

  //compute transition models
  Matrix temp(discretePrior.n,discretePrior.n);
  if(tstate.isEmpty()) tstate.resize(discretePrior.n,discretePrior.n,0.0);
  for(size_t i=1;i<observations.size();i++) {
    for(int k=0;k<discretePrior.n;k++) {
      Real pobs=Log(pbackward[i][k])+obsMat(i,k);
      for(int j=0;j<discretePrior.n;j++) {
	temp(k,j) = Log(pforward[i-1][j])+Log(transitionMatrix(k,j))+pobs;
      }
    }
    //exponential+normalize
    Real vmax = temp.maxElement();
    Real sum = 0.0;
    for(int k=0;k<discretePrior.n;k++) 
      for(int j=0;j<discretePrior.n;j++) {
	temp(k,j) = Exp(temp(k,j)-vmax);
	sum += temp(k,j);
      }
    temp *= 1.0/sum;
    tstate += temp;
  }
}


std::ostream& operator << (std::ostream& out,const LinearProcessHMM& gmm)
{
  out<<gmm.discretePrior<<endl<<endl;
  out<<gmm.transitionMatrix<<endl<<endl;
  Assert((int)gmm.emissionModels.size() == gmm.discretePrior.size());
  Matrix cov;
  for(size_t i=0;i<gmm.emissionModels.size();i++) {
    out<<gmm.continuousPriors[i].mu<<endl;
    gmm.continuousPriors[i].getCovariance(cov);
    out<<cov<<endl<<endl; 
  }
  for(size_t i=0;i<gmm.emissionModels.size();i++) {
    out<<gmm.emissionModels[i].A<<endl;
    out<<gmm.emissionModels[i].error.mu<<endl;
    gmm.emissionModels[i].error.getCovariance(cov);
    out<<cov<<endl<<endl; 
  }
  return out;
}

std::istream& operator >> (std::istream& in,LinearProcessHMM& gmm)
{
  in>>gmm.discretePrior;
  if(!in) return in;
  in>>gmm.transitionMatrix;
  if(!in) return in;
  gmm.continuousPriors.resize(gmm.discretePrior.size());
  Matrix cov;
  for(size_t i=0;i<gmm.continuousPriors.size();i++) {
    in>>gmm.continuousPriors[i].mu;
    if(!in) return in;
    in>>cov;
    if(!in) return in;
    gmm.continuousPriors[i].setCovariance(cov);
  }
  gmm.emissionModels.resize(gmm.discretePrior.size());
  for(size_t i=0;i<gmm.emissionModels.size();i++) {
    in>>gmm.emissionModels[i].A;
    in>>gmm.emissionModels[i].error.mu;
    if(!in) return in;
    in>>cov;
    if(!in) return in;
    gmm.emissionModels[i].error.setCovariance(cov);
  }
  return in;
}


void MixtureCollapse(GaussianMixtureModelRaw& x,int k,bool refit)
{
  if((int)x.phi.size() <= k) return;
  if(refit) {
    GaussianMixtureModel xgmm;
    x.Get(xgmm);
    x.Cluster(xgmm,k);
  }
  else 
    x.Resample(k);
}

void LinearProcessHMMState::CollapseByState(int k,bool refit)
{
  if(y.empty()) {
   for(size_t i=0;i<xy.size();i++)
    MixtureCollapse(xy[i],k,refit);
  }
  else {
    Assert(xy.empty()); 
    for(size_t i=0;i<y.size();i++)
      MixtureCollapse(y[i],k,refit);
  }
}

void LinearProcessHMMState::CollapseToSingleState(int k,bool refit)
{
  if(y.empty()) {
    Assert(!xy.empty());
    GaussianMixtureModelRaw xyall;
    GetXY(xyall);
    xy.resize(1);
    xy[0] = xyall;
    MixtureCollapse(xy[0],k,refit);
  }
  else {
    Assert(xy.empty());
    GaussianMixtureModelRaw yall;
    GetY(yall);
    y.resize(1);
    y[0] = yall;
    MixtureCollapse(y[0],k,refit);
  }
}

void LinearProcessHMMState::CollapseWeighted(int ktotal,bool refit)
{
  if(y.empty()) {
    Assert(!xy.empty());
    if(ktotal == (int)xy.size()) {
      //collapse each into 1 component
      CollapseByState(1,refit);
      return;
    }
    if(ktotal < (int)xy.size()) {
      //collapse into state independent, k components
      CollapseToSingleState(ktotal,refit);
      return;
    }
    //otherwise, distribute samples evenly
    vector<int> num(xy.size());
    RandomAllocate(num,ktotal-xy.size(),p);
    for(size_t i=0;i<xy.size();i++) {
      MixtureCollapse(xy[i],1+num[i],refit);    
    }
  }
  else {
    Assert(!y.empty());
    if(ktotal == (int)y.size()) {
      //collapse each into 1 component
      CollapseByState(1,refit);
      return;
    }
    if(ktotal < (int)y.size()) {
      //collapse into state independent, k components
      CollapseToSingleState(ktotal,refit);
      return;
    }
    //otherwise, distribute samples evenly
    vector<int> num(y.size());
    RandomAllocate(num,ktotal-y.size(),p);
    for(size_t i=0;i<y.size();i++) {
      MixtureCollapse(y[i],1+num[i],refit);    
    }
  }
}

void LinearProcessHMMState::GetXY(GaussianMixtureModelRaw& xycollapsed) const
{
  if(y.empty()) {
    Assert(!xy.empty());
    if(xy.size()==1) {
      xycollapsed = xy[0];
      return;
    }
    else {
      Assert((int)xy.size()==p.n);
      int nc=0;
      for(size_t i=0;i<xy.size();i++)
	nc += xy[i].phi.size();
      xycollapsed.phi.resize(nc);
      xycollapsed.means.resize(nc);
      xycollapsed.covariances.resize(nc);
      int k=0;
      for(size_t i=0;i<xy.size();i++)
	for(size_t j=0;j<xy[i].phi.size();j++) {
	  xycollapsed.phi[k] = xy[i].phi[j]*p[i];
	  xycollapsed.means[k] = xy[i].means[j];
	  xycollapsed.covariances[k] = xy[i].covariances[j];
	  k++;
	}
    }
  }
  else {
    Assert(xy.empty());
    Assert(!x.empty());
    int nx = (int)xindices->size();
    int ny = (int)yindices->size();
    if(y.size()==1) {
      xycollapsed.phi = y[0].phi;
      xycollapsed.means.resize(y[0].phi.size());
      xycollapsed.covariances.resize(y[0].phi.size());
      for(size_t i=0;i<y[0].phi.size();i++) {
	xycollapsed.means[i].resize(nx+ny);
	xycollapsed.means[i].copySubVector(0,x);
	xycollapsed.means[i].copySubVector(nx,y[0].means[i]);
	xycollapsed.covariances[i].resize(nx+ny,nx+ny);
	xycollapsed.covariances[i].setZero();
	xycollapsed.covariances[i].copySubMatrix(nx,nx,y[0].covariances[i]);
      }
      return;
    }
    Assert((int)y.size()==p.n);
    int nc=0;
    for(size_t i=0;i<y.size();i++)
      nc += y[i].phi.size();
    xycollapsed.phi.resize(nc);
    xycollapsed.means.resize(nc);
    xycollapsed.covariances.resize(nc);
    int k=0;
    for(size_t i=0;i<y.size();i++)
      for(size_t j=0;j<y[i].phi.size();j++) {
	xycollapsed.phi[k] = y[i].phi[j]*p[i];
	xycollapsed.means[k].resize(nx+ny);
	xycollapsed.means[k].copySubVector(0,x);
	xycollapsed.means[k].copySubVector(nx,y[i].means[j]);
	xycollapsed.covariances[k].resize(nx+ny,nx+ny);
	xycollapsed.covariances[k].setZero();
	xycollapsed.covariances[k].copySubMatrix(nx,nx,y[i].covariances[j]);
	k++;
      }
  }
}

void LinearProcessHMMState::GetY(GaussianMixtureModelRaw& ycollapsed) const
{
  if(y.empty()) {
    Assert(!xy.empty());
    if(xy.size()==1) {
      ycollapsed.SetMarginalized(xy[0],*xindices);
      return;
    }
    else {
      Assert((int)xy.size()==p.n);
      int nc=0;
      for(size_t i=0;i<xy.size();i++)
	nc += xy[i].phi.size();
      ycollapsed.phi.resize(nc);
      ycollapsed.means.resize(nc);
      ycollapsed.covariances.resize(nc);
      int k=0;
      int ny = yindices->size();
      for(size_t i=0;i<xy.size();i++)
	for(size_t j=0;j<xy[i].phi.size();j++) {
	  ycollapsed.phi[k] = xy[i].phi[j]*p[i];
	  ycollapsed.means[k].resize(ny);
	  ycollapsed.covariances[k].resize(ny,ny);
	  GetElements(xy[i].means[j],*yindices,ycollapsed.means[k]);
	  GetElements(xy[i].covariances[j],*yindices,*yindices,ycollapsed.covariances[k]);
	  k++;
	}
    }
  }
  else {
    Assert(xy.empty());
    if(y.size()==1) {
      ycollapsed=y[0];
      return;
    }
    Assert((int)y.size()==p.n);
    int nc=0;
    for(size_t i=0;i<y.size();i++)
      nc += y[i].phi.size();
    ycollapsed.phi.resize(nc);
    ycollapsed.means.resize(nc);
    ycollapsed.covariances.resize(nc);
    int k=0;
    for(size_t i=0;i<y.size();i++)
      for(size_t j=0;j<y[i].phi.size();j++) {
	ycollapsed.phi[k] = y[i].phi[j]*p[i];
	ycollapsed.means[k] = y[i].means[j];
	ycollapsed.covariances[k] = y[i].covariances[j];
	k++;
      }
  }
}

void LinearProcessHMMState::GetY(Vector& ymean,Matrix& ycov) const
{
  //TODO: may be able to save memory by doing this all inline
  GaussianMixtureModelRaw yall;
  GetY(yall);
  yall.GetMean(ymean);
  yall.GetCovariance(ycov);
}

LinearProcessHMMRegression::LinearProcessHMMRegression()
{}

LinearProcessHMMRegression::LinearProcessHMMRegression(const LinearProcessHMM& _joint)
  :joint(_joint)
{}

void LinearProcessHMMRegression::SetXIndices(const std::vector<int>& _xindices)
{
  xindices = _xindices;
  yindices.resize(0);
  for(int i=0;i<joint.emissionModels[0].A.m;i++) {
    int index=-1;
    for(size_t j=0;j<xindices.size();j++) {
      if(xindices[j] == i) {
	index = (int)j;
	break;
      }
    }
    if(index < 0) yindices.push_back(i);
  }
  Assert(int(yindices.size() + xindices.size()) == joint.emissionModels[0].A.m);
  xPriors.resize(joint.continuousPriors.size());
  yPriors.resize(joint.continuousPriors.size());
  Matrix cov;
  for(size_t i=0;i<joint.continuousPriors.size();i++) {
    xPriors[i].setMarginalized(joint.continuousPriors[i],yindices);
    GaussianRegression reg;
    reg.Set(joint.continuousPriors[i],xindices,yindices);
    reg.GetLinearEquation(yPriors[i].A,yPriors[i].error.mu);
    reg.GetNoiseCovariance(cov);
    yPriors[i].error.setCovariance(cov);
  }

  xRegressions.resize(joint.emissionModels.size());
  for(size_t i=0;i<xRegressions.size();i++) {
    xRegressions[i].A.resize((int)xindices.size(),(int)xindices.size());
    GetElements(joint.emissionModels[i].A,xindices,xindices,xRegressions[i].A);
    xRegressions[i].error.setMarginalized(joint.emissionModels[i].error,yindices);
  }
}

void LinearProcessHMMRegression::GetInitial(LinearProcessHMMState& s0) const
{
  s0.p = joint.discretePrior;
  s0.x.clear();
  s0.xPrev.clear();
  s0.xy.resize(joint.discretePrior.n);
  s0.xindices = &xindices;
  s0.yindices = &yindices;
  Matrix cov;
  for(size_t i=0;i<yPriors.size();i++) {
    s0.xy[i].Resize(1,(int)yindices.size());
    xPriors[i].getCovariance(cov);
    yPriors[i].GetJoint(xPriors[i].mu,cov,s0.xy[i].means[0],s0.xy[i].covariances[0]);
  }
}

Real LinearProcessHMMRegression::ProbabilityX(const LinearProcessHMMState& s,const Vector& x) const
{
  Assert(s.p.n == joint.discretePrior.n);
  if(s.xPrev.empty() && s.x.empty()) {
    Real p=0;
    for(int i=0;i<s.p.n;i++) {
      if(s.p(i) != 0.0)
	p += s.p(i) * xPriors[i].probability(x);
    }
    return p;
  }
  else {
    Real p=0;
    LOG4CXX_INFO(KrisLibrary::logger(),"PObservation "<<x);
    LOG4CXX_INFO(KrisLibrary::logger(),"  Discrete prior "<<s.p);
    for(int i=0;i<s.p.n;i++) {
      if(s.p(i) != 0.0) {
	if(!s.x.empty()) {
	  Vector vtemp;
	  xRegressions[i].A.mul(s.x,vtemp);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"  Prediction "<<i<<": "<<vtemp+xRegressions[i].error.mu);
	  p += s.p(i) * xRegressions[i].Probability(s.x,x);
	}
	else {
	  Vector vtemp;
	  xRegressions[i].A.mul(s.xPrev,vtemp);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"  Prediction "<<i<<": "<<vtemp+xRegressions[i].error.mu);
	  LOG4CXX_ERROR(KrisLibrary::logger(),"  Error cov L "<<xRegressions[i].error.L);
	  p += s.p(i) * xRegressions[i].Probability(s.xPrev,x);
	}
      }
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"  Probability of obs "<<p);
    return p;
  }
}

void LinearProcessHMMRegression::Update(const LinearProcessHMMState& s,const Vector& x,LinearProcessHMMState& sobs) const
{
  Assert(s.x.empty());
  Assert(s.y.empty());
  Assert(!s.xy.empty());

  //copy observations
  sobs.xPrev = s.xPrev;
  sobs.x = x;
  sobs.xy.clear();
  sobs.xindices = &xindices;
  sobs.yindices = &yindices;

  //compute obsevation probabilities pobs
  Vector pobs(joint.discretePrior.n);
  if(s.xPrev.empty()) {
    for(int i=0;i<joint.discretePrior.n;i++)
      pobs(i) = xPriors[i].probability(x);
  }
  else {
    for(int i=0;i<joint.discretePrior.n;i++)
      pobs(i) = xRegressions[i].Probability(s.xPrev,x);
  } 

  //update discrete distribution and weight by observation probability
  sobs.p = s.p;
  for(int i=0;i<sobs.p.n;i++)
    sobs.p(i) *= pobs(i);
  NormalizeProbability(sobs.p);

  //now compute y update
  vector<vector<GaussianMixtureModel> > newy;
  sobs.y.resize(s.xy.size());
  Vector xmean;
  Matrix xcov;
  for(size_t i=0;i<s.xy.size();i++) {
    sobs.y[i].Resize(s.xy[i].means.size(),yindices.size());
    for(size_t j=0;j<s.xy[i].means.size();j++) {
      GaussianMarginalize(s.xy[i].means[j],s.xy[i].covariances[j],yindices,xmean,xcov);
      Gaussian<Real> gx(xcov,xmean);
      //condition on xt = x
      GaussianCondition(s.xy[i].means[j],s.xy[i].covariances[j],x,xindices,
			sobs.y[i].means[j],sobs.y[i].covariances[j]);
      /*
      LOG4CXX_INFO(KrisLibrary::logger(),"Joint mean"<<s.xy[i].means[j]);
      LOG4CXX_INFO(KrisLibrary::logger(),"Joint cov"<<s.xy[i].covariances[j]);
      LOG4CXX_INFO(KrisLibrary::logger(),"Observation "<<x);
      LOG4CXX_INFO(KrisLibrary::logger(),"new mean: "<<sobs.y[i].means[j]);
      LOG4CXX_INFO(KrisLibrary::logger(),"new cov: "<<sobs.y[i].covariances[j]);
      KrisLibrary::loggerWait();
      */
      //what's the probability that x[t] is x in this component?
      sobs.y[i].phi[j] = s.xy[i].phi[j]*gx.probability(x);
    }
    Real psum=Normalize(sobs.y[i].phi);
    if(sobs.p(i) > 1e-10 && !sobs.y[i].phi.empty()) {
      if(psum == 0.0) {
	LOG4CXX_WARN(KrisLibrary::logger(),"LPHMMR warning: probability of discrete state "<<i<<": "<<sobs.p(i)<<", detailed obs prob sum "<<psum);
	//KrisLibrary::loggerWait();
      }
      //Assert(psum != 0.0);
    }
  }
}


void LinearProcessHMMRegression::Predict(const LinearProcessHMMState& s0,LinearProcessHMMState& st) const
{
  joint.Predict(s0.p,st.p);
  st.xPrev = s0.x;
  st.x.clear();
  st.y.clear();  
  st.xindices = &xindices;
  st.yindices = &yindices;
  if(s0.xy.empty()) {
    //this is slightly faster than converting to xy representation below
    //take prior y and predict using transition model 
    Assert(!s0.x.empty());
    Assert(s0.y.size()==1 || s0.y.size()==joint.discretePrior.size());
    Vector xyp_mean(xindices.size()+yindices.size()),xy_mean;
    Matrix xyp_cov(xindices.size()+yindices.size(),xindices.size()+yindices.size(),Zero),xy_cov;
    SetElements(xyp_mean,xindices,s0.x);
    
    st.xy.resize(yPriors.size());
    for(int k=0;k<joint.discretePrior.size();k++) {
      st.xy[k].phi.resize(0);
      st.xy[k].means.resize(0);
      st.xy[k].covariances.resize(0);
      //transitions from i->k
      for(size_t i=0;i<joint.discretePrior.size();i++) {
	//handle completely collapsed states
	const GaussianMixtureModelRaw& yi = (s0.y.size()==1 ? s0.y[0] : s0.y[i]);
	//transition all components from GMM yi to k
	for(size_t j=0;j<yi.phi.size();j++) {
	  Real pik = joint.transitionMatrix(k,i)*yi.phi[j];
	  if(pik < 1e-10) continue;
	  st.xy[k].phi.push_back(pik);
	  st.xy[k].means.resize(st.xy[k].phi.size());
	  st.xy[k].covariances.resize(st.xy[k].phi.size());

	  //do a forward prediction
	  SetElements(xyp_mean,yindices,yi.means[j]);
	  SetElements(xyp_cov,yindices,yindices,yi.covariances[j]);
	  joint.emissionModels[i].GetY(xyp_mean,xyp_cov,st.xy[k].means.back(),st.xy[k].covariances.back());
	}
      }
      Normalize(st.xy[k].phi);
    }
  }
  else {
    Assert(s0.xy.size()==1 || s0.xy.size()==joint.discretePrior.size());
    st.xy.resize(s0.xy.size());
    for(int k=0;k<joint.discretePrior.size();k++) {
      st.xy[k].phi.resize(0);
      st.xy[k].means.resize(0);
      st.xy[k].covariances.resize(0);
      //transitions from i->k
      for(size_t i=0;i<joint.discretePrior.size();i++) {
	//handle completely collapsed states
	const GaussianMixtureModelRaw& xyi = (s0.xy.size()==1 ? s0.xy[0] : s0.xy[i]);
	//transition all components from GMM s0.y[i] to k
	for(size_t j=0;j<xyi.phi.size();j++) {
	  Real pik = joint.transitionMatrix(k,i)*xyi.phi[j];
	  if(pik < 1e-10) continue;
	  st.xy[k].phi.push_back(pik);
	  st.xy[k].means.resize(st.xy[k].phi.size());
	  st.xy[k].covariances.resize(st.xy[k].phi.size());

	  joint.emissionModels[i].GetY(xyi.means[j],xyi.covariances[j],st.xy[k].means.back(),st.xy[k].covariances.back());
	}
      }
      Normalize(st.xy[k].phi);
    }
  }
}

void LinearProcessHMMRegression::Predict(const LinearProcessHMMState& s0,int numsteps,LinearProcessHMMState& st,int kmax) const
{
  if(kmax < 0) {
    //count initial gaussians
    kmax = 0;
    if(s0.y.empty()) {
      Assert(!s0.xy.empty());
      for(size_t i=0;i<s0.xy.size();i++)
	kmax += (int)s0.xy[i].phi.size();
    }
    else {
      Assert(s0.xy.empty());
      for(size_t i=0;i<s0.y.size();i++)
	kmax += (int)s0.y[i].phi.size();
    }
  }
  st = s0;
  LinearProcessHMMState temp;
  for(int step=0;step<numsteps;step++) {
    temp = st;
    Predict(temp,st);

    //do collapse step
    if(kmax != 0) {
      st.CollapseWeighted(kmax);
    }
  }
}


} //namespace Statistics
