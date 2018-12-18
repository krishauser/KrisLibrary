#ifndef STATISTICS_GMM_H
#define STATISTICS_GMM_H

#include <KrisLibrary/math/gaussian.h>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/matrix.h>
#include <vector>

namespace Statistics {
  using namespace Math;

/** @ingroup Statistics
 * @brief A model of a probability distribution consisting of k gaussians.
 */
class GaussianMixtureModel
{
public:
  GaussianMixtureModel();
  GaussianMixtureModel(int k,int d);
  GaussianMixtureModel(const GaussianMixtureModel&);
  
  int NumDims() const;
  void Resize(int k,int d);
  void SetLinearTransform(const GaussianMixtureModel& gmm,const Matrix& A,const Vector& b);

  /** @brief Solves the max likelihood model from training data.
   *
   * Runs expectation-maximization, starting from current values of
   * gaussians, given the training data, a tolerance, # of max iterations.
   * Returns true if tolerance was reached, returns log-likelihood
   * of data in tol.
   */
  bool TrainEM(const std::vector<Vector>& examples,Real& tol,int maxIters,int verbose=0);
  bool TrainDiagonalEM(const std::vector<Vector>& examples,Real& tol,int maxIters,int verbose=0);

  /// Computes the log likelihood of the data
  Real LogLikelihood(const std::vector<Vector>& data);

  int PickGaussian() const;
  Real Probability(const Vector& x) const;
  void Generate(Vector& x) const;
  void GetMean(Vector& x) const;
  void GetMode(Vector& x) const;
  void GetCovariance(Matrix& cov) const;
  void GetVariance(Vector& var) const;

  //Sets this GMM to exactly k sampled components, with each component
  //sampled with probability proportional to phi[i].  Stratified sampling
  //is used if k > phi.size().  Otherwise, they are picked using weighted
  //sampling without replacement.
  void Resample(int k);
  //Resamples and refits the GMM to exactly k sampled components.  Fitting
  //is done by selecting the original component that minimizes KL divergence
  //to the resampled component
  void Cluster(const GaussianMixtureModel& gmm,int k);
  
  std::vector<Gaussian<Real> > gaussians;
  std::vector<Real> phi;   ///<phi[i] gives probability of choosing gaussian[i]
};

std::ostream& operator << (std::ostream& out,const GaussianMixtureModel& gmm);
std::istream& operator >> (std::istream& in,GaussianMixtureModel& gmm);

/** @ingroup Statistics
 * @brief A more ``raw'' model of a GMM that does not perform a cholesky
 * decomposiition
 */
class GaussianMixtureModelRaw
{
public:
  GaussianMixtureModelRaw();
  GaussianMixtureModelRaw(int k,int d);
  GaussianMixtureModelRaw(const GaussianMixtureModel&);
  GaussianMixtureModelRaw(const GaussianMixtureModelRaw&);

  void Set(const GaussianMixtureModel& gmm);  
  bool Get(GaussianMixtureModel& gmm) const;
  int NumDims() const;
  void Resize(int k,int d);
  void SetLinearTransform(const GaussianMixtureModelRaw& gmm,const Matrix& A,const Vector& b);
  void SetSubset(const GaussianMixtureModelRaw& gmm,const std::vector<int>& keptIndices);
  void SetMarginalized(const GaussianMixtureModelRaw& gmm,const std::vector<int>& dropIndices);
  void SetCombination(const std::vector<GaussianMixtureModelRaw>& gmms,const std::vector<Real>& weights);
  int PickGaussian() const;
  void GetMean(Vector& x) const;
  void GetMode(Vector& x) const;
  void GetCovariance(Matrix& cov) const;
  void GetVariance(Vector& var) const;

  //Sets this GMM to exactly k sampled components, with each component
  //sampled with probability proportional to phi[i].  Stratified sampling
  //is used if k > phi.size().  Otherwise, they are picked using weighted
  //sampling without replacement.
  void Resample(int k);
  //Resamples and refits the GMM to exactly k sampled components.  Fitting
  //is done by selecting the original component that minimizes KL divergence
  //to the resampled component
  void Cluster(const GaussianMixtureModel& gmm,int k);
  
  std::vector<Vector> means;
  std::vector<Matrix> covariances;
  std::vector<Real> phi;   ///<phi[i] gives probability of choosing gaussian[i]
};

std::ostream& operator << (std::ostream& out,const GaussianMixtureModelRaw& gmm);
std::istream& operator >> (std::istream& in,GaussianMixtureModelRaw& gmm);

class GaussianRegression
{
 public:
  void Set(const Gaussian<Real>& g,const std::vector<int>& xindices,const std::vector<int>& yindices);
  void Set(const Vector& mean,const Matrix& cov,const std::vector<int>& xindices,const std::vector<int>& yindices);
  void GetY(const Vector& x,Vector& ymean,Matrix& ycov) const;
  void GetY(const Vector& x,Gaussian<Real>& y) const;
  void GetY(const Vector& xmean,const Matrix& xcov,Vector& ymean,Matrix& ycov) const;
  void GetJoint(const Vector& xmean,const Matrix& xcov,Vector& xymean,Matrix& xycov) const;
  void GetLinearEquation(Matrix& A,Vector& b) const;
  void GetNoiseCovariance(Matrix& sigma) const;

  Vector xmean,ymean,b;
  Matrix ycov,yxcov,xcovinv;
  Matrix A;
};

class GaussianMixtureRegression
{
 public:
  GaussianMixtureRegression();
  GaussianMixtureRegression(const GaussianMixtureModel& joint);
  GaussianMixtureRegression(const GaussianMixtureModelRaw& joint);
  void SetXIndices(const std::vector<int>& xindices);
  Real ProbabilityX(const Vector& x) const;
  void GetY(const Vector& x,GaussianMixtureModelRaw& ygmm) const;
  void GetY(const Vector& x,GaussianMixtureModel& ygmm) const;
  void GetY(const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& ygmm) const;
  void GetJoint(const Vector& xmean,const Matrix& xcov,GaussianMixtureModelRaw& xygmm) const;

  GaussianMixtureModel joint;
  std::vector<int> xindices,yindices;
  GaussianMixtureModel xgmm;
  std::vector<GaussianRegression> regressions;
};

}  //namespace Statistics

#endif
