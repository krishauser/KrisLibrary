#ifndef MATH_GAUSSIAN_H
#define MATH_GAUSSIAN_H

#include "MatrixTemplate.h"
#include <vector>

namespace Math {

/** @ingroup Math
 * @brief Multivariate gaussian N(mu,K) of d dimensions
 *
 * \f$ P(x~N(mu,K)) = c e^{-1/2 (x-mu)^t K^{-1} (x-mu))} \f$
 * where c = 1/sqrt((2pi)^d |K|).
 *
 * Represented by the cholesky decomposition of K = LL^t.
 *
 * \f$ x^t K^{-1} x = 
 *     x^t (L L^t)^{-1} x = 
 *     x^t L^{-t} L^{-1} x = |L^{-1} x|^2 \f$
 *
 * so sqrt(|K|) = |L| = product of diag(L).
 * Equivalent to the change of variable y = L^-1(x-mu), such that
 * P(x~N(mu,K)) = P(y~N(0,I))
 */
template <class T>
class Gaussian
{
public:
  typedef Gaussian<T> MyT;
  typedef VectorTemplate<T> VectorT;
  typedef MatrixTemplate<T> MatrixT;

  Gaussian();
  Gaussian(int d);
  Gaussian(const MatrixT& sigma, const VectorT& mu);
  Gaussian(const MyT& g);
  
  void resize(int d);
  int numDims() const;
  int degeneracy() const;
  Real normalizationFactor() const;

  ///Covariance matrix must be symmetric, strictly positive definite
  bool setCovariance(const MatrixT& sigma,int verbose=1);
  void getCovariance(MatrixT& sigma) const;
  void getVariance(VectorT& sigma) const;
  void getPrecision(MatrixT& sigma) const;
  void setMean(const VectorT& mu);
  const VectorT& getMean() const { return mu; }
  
  ///ML estimation
  bool setMaximumLikelihood(const std::vector<VectorT>& examples,int verbose=1);
  bool setMaximumLikelihood(const std::vector<VectorT>& examples,const std::vector<Real>& weights,int verbose=1);
  void setMaximumLikelihoodDiagonal(const std::vector<VectorT>& examples);
  void setMaximumLikelihoodDiagonal(const std::vector<VectorT>& examples,const std::vector<Real>& weights);
  ///MAP estimation with inverse weishart distribution
  bool setMaximumAPosteriori(const std::vector<VectorT>& examples,
			     const VectorT& meanprior,T meanStrength,const MatrixT& covprior,T covStrength,int verbose=1);
  bool setMaximumAPosteriori(const std::vector<VectorT>& examples,const std::vector<Real>& weights,
			     const VectorT& meanprior,T meanStrength,const MatrixT& covprior,T covStrength,int verbose=1);
  void setMaximumAPosterioriDiagonal(const std::vector<VectorT>& examples,
				     const VectorT& meanprior,T meanStrength,const VectorT& varprior,T covStrength);
  void setMaximumAPosterioriDiagonal(const std::vector<VectorT>& examples,const std::vector<Real>& weights,
				     const VectorT& meanprior,T meanStrength,const VectorT& varprior,T covStrength);

  ///Evaluates the probability of x
  T probability(const VectorT& x) const;
  ///Evaluates the log-probability of x
  T logProbability(const VectorT& x) const;
  ///Evaluates the probability of a partial setting x(elements[i])=x[i]
  T partialProbability(const VectorT& x,const std::vector<int>& elements) const;
  ///Evaluates the log partial probability
  T logPartialProbability(const VectorT& x,const std::vector<int>& elements) const;
  ///Generates a point x according to this distribution
  void generate(VectorT& x) const;

  //combines the two gaussian distributions into an uncorrelated
  //joint distribution
  void setJoint(const MyT& g1,const MyT& g2);
  //combines the two gaussian distributions into a correlated
  //joint distribution
  void setJoint(const MyT& g1,const MyT& g2,const MatrixT& corr);
  //marginalizes the distribution g over the element i
  void setMarginalized(const MyT& g,int i);
  //marginalizes the distribution g over the given elements
  void setMarginalized(const MyT& g,const std::vector<int>& elements);
  //sets the conditional distribution of g given x(i) = xi
  void setConditional(const MyT& g,Real xi,int i);
  //sets the conditional distribution of g given elements x(elements[i]) = x(i)
  void setConditional(const MyT& g,const VectorT& x,const std::vector<int>& elements);

  //computes KL divergence of g from this: D(g||this)
  T klDivergence(const MyT& g) const;
  
  MatrixT L;		///< Cholesky decomposition of the covariance matrix
  VectorT mu;		///< mean
};

///sets the conditional distribution of N(mean,cov) given elements
///x(elements[i]) = x(i)
template <class T>
void GaussianCondition(const VectorTemplate<T>& mean,
		       const MatrixTemplate<T>& cov,
		       const VectorTemplate<T>& x,
		       const std::vector<int>& elements,
		       VectorTemplate<T>& newmean,
		       MatrixTemplate<T>& newcov);

///marginalizes the distribution N(mean,cov) over the given elements
template <class T>
void GaussianMarginalize(const VectorTemplate<T>& mean,
			 const MatrixTemplate<T>& cov,
			 const std::vector<int>& elements,
			 VectorTemplate<T>& newmean,
			 MatrixTemplate<T>& newcov);

///transforms the distribution N(mean,cov) with the linear transform Ax+b
template <class T>
void GaussianTransform(const VectorTemplate<T>& mean,
		       const MatrixTemplate<T>& cov,
		       const MatrixTemplate<T>& A,
		       const VectorTemplate<T>& b,
		       VectorTemplate<T>& newmean,
		       MatrixTemplate<T>& newcov);

///if y=Ax+b and x is distributed w.r.t. N(mean,cov), returns a gaussian
///over the vector [x,y]
template <class T>
void GaussianTransformJoint(const VectorTemplate<T>& mean,
			    const MatrixTemplate<T>& cov,
			    const MatrixTemplate<T>& A,
			    const VectorTemplate<T>& b,
			    VectorTemplate<T>& jointmean,
			    MatrixTemplate<T>& jointcov);

}

#endif
