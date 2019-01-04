#ifndef STATISTICS_GAUSSIAN_HMM_H
#define STATISTICS_GAUSSIAN_HMM_H

#include "GaussianMixtureModel.h"

namespace Statistics {
  using namespace Math;

/** @ingroup Statistics
 * @brief A model of a temporal sequence consisting of k discrete states
 * each with a gaussian emission probability.
 *
 * The graphical model representation is as follows:
 *   x0 -> x1 -> x2 -> ...
 *         |     |
 *         v     v
 *         o1    o2    ...
 */
class GaussianHMM
{
public:
  GaussianHMM();
  GaussianHMM(int k,int d);
  GaussianHMM(const GaussianMixtureModel&);

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
  Real Probability(const Vector& p0,const Vector& obs) const;

  /// Given prior distribution over discrete states p0, predicts the
  ///distribution after numsteps time steps
  void Predict(const Vector& p0,Vector& pt,int numsteps=1) const;
  /// Given prior distribution over discrete states p0, updates the
  /// distribution given knowledge of the observation
  void Update(const Vector& p0,const Vector& obs,Vector& pobs) const;
  /// Given prior distribution over discrete states p0 and the observation obs
  /// return the posterior distribution over the next step
  void Filter(const Vector& p0,const Vector& obs,Vector& pnext) const;
  /// Given distribution over discrete states, compute distribution over
  /// observation
  void ObservationDistribution(const Vector& p0,GaussianMixtureModel& model) const;
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
  std::vector<Gaussian<Real> > emissionModels;
};

std::ostream& operator << (std::ostream& out,const GaussianHMM& gmm);
std::istream& operator >> (std::istream& in,GaussianHMM& gmm);

class GaussianHMMRegression
{
 public:
  GaussianHMMRegression();
  GaussianHMMRegression(const GaussianHMM& joint);
  void SetXIndices(const std::vector<int>& xindices);
  Real ProbabilityX(const Vector& p0,const Vector& x) const;
  void Filter(const Vector& p0,const Vector& x,Vector& pnext) const;
  void GetYUnconditional(const Vector& p0,GaussianMixtureModelRaw& y) const;
  void GetY(const Vector& p0,const Vector& x,GaussianMixtureModelRaw& y) const;
  void GetY(const Vector& p0,const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& y) const;
  void GetJoint(const Vector& p0,const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& xygmm) const;

  GaussianHMM joint;
  std::vector<int> xindices,yindices;
  std::vector<Gaussian<Real> > xEmissionModels;
  std::vector<GaussianRegression> regressions;
};


} //namespace Statistics

#endif
