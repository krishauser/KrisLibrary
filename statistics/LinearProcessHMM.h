#ifndef STATISTICS_LINEAR_PROCESS_HMM_H
#define STATISTICS_LINEAR_PROCESS_HMM_H

#include "GaussianMixtureModel.h"

namespace Statistics {
  using namespace Math;

class LinearProcess
{
 public:
  Real Probability(const Vector& x,const Vector& y) const;
  Real LogProbability(const Vector& x,const Vector& y) const;
  void GetY(const Vector& x,Gaussian<Real>& y) const;
  void GetY(const Vector& x,Vector& ymean,Matrix& ycov) const;
  void GetY(const Vector& xmean,const Matrix& xcov,Vector& ymean,Matrix& ycov) const;
  void GetJoint(const Vector& xmean,const Matrix& xcov,Vector& xymean,Matrix& xycov) const;
  bool OLS(const std::vector<Vector>& x,
	   const std::vector<Vector>& y,
	   int verbose=1);
  bool WeightedOLS(const std::vector<Vector>& x,
		   const std::vector<Vector>& y,
		   const std::vector<Real>& w,
		   int verbose=1);

  Matrix A;
  Gaussian<Real> error;
};

/** @ingroup Statistics
 * @brief A model of a temporal sequence consisting of k discrete states
 * each with a gaussian emission probability depending on the prior
 * continuous state.
 *
 * The graphical model representation is as follows:
 *   x0 -> x1 -> ...
 *   |     |
 *   v     v
 *   o0 -> o1 -> ...
 *
 * where each P(o[t] | x[t], o[t-1]) is a linear process o[t]=Ao[t-1]+b+err
 * with the coefficients A, b, and error covariance depending on x[t].
 */
class LinearProcessHMM
{
public:
  LinearProcessHMM();
  LinearProcessHMM(int k,int d);
  LinearProcessHMM(const GaussianMixtureModel&);

  int NumDims() const;
  void Resize(int k,int d);
  //set a uniform transition matrix
  void SetUniformTransitions();
  //set a transition matrix that stays in current state with probability
  //1-pExit, transitions to a state picked uniformly at random otherwise
  void SetUniformExitProbabilities(Real pExit);

  /** @brief Solves the max likelihood model from training data.
   *
   * Runs expectation-maximization, starting from current values of
   * gaussians, given the training data, a tolerance, # of max iterations.
   * Returns true if tolerance was reached, returns log-likelihood
   * of data in tol.
   */
  bool TrainEM(const std::vector<std::vector<Vector> >& examples,Real& tol,int maxIters,int verbose=0);
  bool TrainDiagonalEM(const std::vector<std::vector<Vector> >& examples,Real& tol,int maxIters,int verbose=0);

  /// Computes the log likelihood of the time series
  Real LogLikelihood(const std::vector<int>& dstates,const std::vector<Vector>& observations) const;
  Real LogLikelihood(const std::vector<Vector>& observations) const;
  Real Probability(const Vector& p0,const Vector& prevObs,const Vector& obs) const;

  /// Given prior distribution over discrete states p0, predicts the
  ///distribution after numsteps time steps
  void Predict(const Vector& p0,Vector& pt,int numsteps=1) const;
  /// Given prior distribution over current state p0, the previous
  /// observation, and the current observation obs, updates the state
  /// distribution 
  void Update(const Vector& p0,const Vector& prevObs,const Vector& obs,Vector& pobs) const;
  /// Given prior distribution over previous discrete state p0, the previous
  /// observation, and the current observation obs, return the posterior
  /// distribution over the current state (shortcut to Predict + Update)
  void Filter(const Vector& p0,const Vector& prevObs,const Vector& obs,Vector& pnext) const;
  /// Given distribution over discrete states and the previous observation,
  /// compute distribution over the current observation
  void ObservationDistribution(const Vector& pt,const Vector& prevObs,GaussianMixtureModel& model) const;
  /// Computes the MAP assignment using the Viterbi algorithm
  void MAP(const std::vector<Vector>& observations,std::vector<int>& dstates) const;
  /// Computes the posterior probabilities of each discrete state using the
  /// forward-backward algorithm
  void Posterior(const std::vector<Vector>& observations,std::vector<Vector>& pstate) const;
  /// Computes the posterior probabilities of each discrete state and
  /// accumulates state-to-state transition pairs using the forward-backward
  /// algorithm
  void Posterior(const std::vector<Vector>& observations,std::vector<Vector>& pstate,Matrix& tstate) const;

  Vector discretePrior;
  Matrix transitionMatrix;
  std::vector<Gaussian<Real> > continuousPriors;
  std::vector<LinearProcess> emissionModels;
};

std::ostream& operator << (std::ostream& out,const LinearProcessHMM& gmm);
std::istream& operator >> (std::istream& in,LinearProcessHMM& gmm);


class LinearProcessHMMState
{
 public:
  //collapses all state-dependent mixtures into at most k components
  void CollapseByState(int k=1,bool refit=true);
  //turns state-dependent mixtures into 1 state-independent mixture
  //with at most k components
  void CollapseToSingleState(int k=1,bool refit=true);
  //collapses mixtures into ktotal total mixtures
  void CollapseWeighted(int ktotal=1,bool refit=true);
  //merges all state-dependent mixtures into one
  void GetY(GaussianMixtureModelRaw& ycollapsed) const;
  //merges all state-dependent mixtures into one
  void GetY(Vector& ymean,Matrix& ycov) const;
  //merges all state-dependent mixtures into one
  void GetXY(GaussianMixtureModelRaw& xycollapsed) const;

  //after predict, xy is filled out and y is cleared
  //after update, x and y are filled out and xy is cleared
  Vector p;
  Vector xPrev,x;
  std::vector<GaussianMixtureModelRaw> y;
  std::vector<GaussianMixtureModelRaw> xy;
  const std::vector<int> *xindices, *yindices;
};

class LinearProcessHMMRegression
{
 public:
  LinearProcessHMMRegression();
  LinearProcessHMMRegression(const LinearProcessHMM& joint);
  void SetXIndices(const std::vector<int>& xindices);
  void GetInitial(LinearProcessHMMState& s0) const;
  Real ProbabilityX(const LinearProcessHMMState& s,const Vector& x) const;
  void Update(const LinearProcessHMMState& s,const Vector& x,LinearProcessHMMState& sobs) const;
  void Predict(const LinearProcessHMMState& s0,LinearProcessHMMState& st) const;
  void Predict(const LinearProcessHMMState& s0,int numsteps,LinearProcessHMMState& st,int kmax=-1) const;

  LinearProcessHMM joint;
  std::vector<int> xindices,yindices;
  std::vector<Gaussian<Real> > xPriors;
  std::vector<LinearProcess> yPriors;
  std::vector<LinearProcess> xRegressions;
};

} //namespace Statistics

#endif
