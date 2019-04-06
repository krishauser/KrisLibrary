#include <KrisLibrary/Logger.h>
#include "gaussian.h"
#include "CholeskyDecomposition.h"
#include "LDL.h"
#include "DiagonalMatrix.h"
#include "backsubstitute.h"
#include "random.h"
#include <errors.h>
using namespace std;

namespace Math {

template <class T>
Gaussian<T>::Gaussian()
{
}

template <class T>
Gaussian<T>::Gaussian(int d)
{
  resize(d);
}

template <class T>
Gaussian<T>::Gaussian(const MatrixT& sigma, const VectorT& _mu)
{
  mu = _mu;
  setCovariance(sigma);
}

template <class T>
Gaussian<T>::Gaussian(const Gaussian& g)
  :L(g.L),mu(g.mu)
{
}

template <class T>
void Gaussian<T>::resize(int d)
{
  L.resize(d,d);
  mu.resize(d,(T)Zero);
}

template <class T>
int Gaussian<T>::numDims() const
{
  return mu.n;
}

template <class T>
int Gaussian<T>::degeneracy() const
{
  int d=0;
  for(int i=0;i<L.m;i++)
    if(L(i,i)==0.0) d++;
  return d;
}

template <class T>
bool Gaussian<T>::setCovariance(const MatrixT& sigma,int verbose)
{
  if(mu.n != 0 && sigma.m != mu.n) {
    FatalError("Invalid dimensions on covariance matrix");
    return false;
  }
  LDLDecomposition<T> chol;
  chol.verbose = verbose;
  chol.set(sigma);
  DiagonalMatrixTemplate<T> D;
  chol.getD(D);
  chol.getL(L);
  bool res=true;
  for(int i=0;i<D.n;i++) {
    if(D(i) < 0.0) {
      if(verbose >= 1) 
	LOG4CXX_WARN(KrisLibrary::logger(),"Gaussian::setCovariance(): Warning, sigma "<<i<<" = "<<D(i));
      if(D(i) < -Epsilon) 
	res = false;
      D(i) = 0.0;
    }
    else
      D(i) = Sqrt(D(i));
  }
  D.postMultiply(L,L);
  return res;
}

template <class T>
void Gaussian<T>::getCovariance(MatrixT& sigma) const
{
  //sigma.mulTransposeB(L,L);
  //faster version -- uses fact that L is a lower triangular matrix
  sigma.resize(L.n,L.n);
  for(int i=0;i<L.n;i++)
    for(int j=0;j<L.n;j++) {
      VectorT Li,Lj;
      L.getRowRef(i,Li);
      L.getRowRef(j,Lj);
      T sum=0.0;
      typename VectorT::ItT iti=Li.begin();
      typename VectorT::ItT itj=Lj.begin();
      for(int k=0;k<=Min(i,j);k++,iti++,itj++)
	sum += (*iti)*(*itj);
      sigma(i,j) = sum;
    }
}

template <class T>
void Gaussian<T>::getVariance(VectorT& sigma) const
{
  sigma.resize(L.n);
  for(int i=0;i<L.n;i++) {
    VectorT Li;
    L.getRowRef(i,Li);
    T sum = 0.0;
    typename VectorT::ItT it=Li.begin();
    for(int j=0;j<=i;j++,it++)
      sum += Sqr(*it);
    sigma(i)=sum;
  }
}

template <class T>
void Gaussian<T>::getPrecision(MatrixT& sigma) const
{
  sigma.resize(L.n,L.n);
  VectorT temp(L.n,(T)Zero),y,x;
  for(int i=0;i<L.n;i++) {
    sigma.getColRef(i,x);  //x &= col i of A
    temp(i)=1.0;
    if(!LBackSubstitute(L,temp,y)) { temp(i)=0.0; x.setZero(); continue; }
    if(!LtBackSubstitute(L,y,x)) { temp(i)=0.0; x.setZero(); continue; }
    temp(i)=0.0;
  }
}

template <class T>
void Gaussian<T>::setMean(const VectorT& _mu)
{
  if(L.m != 0 && _mu.n != L.m) {
    FatalError("Invalid dimensions on mean vector");
  }
  mu = _mu;
}

template <class T>
bool Gaussian<T>::setMaximumLikelihood(const std::vector<VectorT>& examples,int verbose)
{
  assert(examples.size() >= 1);
  mu = examples[0];
  for(size_t i=1;i<examples.size();i++)
    mu += examples[i];
  mu *= 1.0/examples.size();

  MatrixT cov(mu.n,mu.n, (T)Zero);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
      for(int j=0;j<mu.n;j++)
	cov(i,j) += temp(i)*temp(j);
    }
  }
  cov *= (T)(1.0/examples.size());
  return setCovariance(cov,verbose);
}

template <class T>
bool Gaussian<T>::setMaximumLikelihood(const std::vector<VectorT>& examples,const std::vector<Real>& weights,int verbose)
{
  assert(examples.size()==weights.size());
  assert(examples.size() >= 1);

  Real sumw=weights[0];
  mu.mul(examples[0],(T)weights[0]);
  for(size_t i=1;i<examples.size();i++) {
    mu.madd(examples[i],(T)weights[i]);
    sumw += weights[i];
  }
  if(sumw == 0.0) {
    //degenerate
    L.resize(mu.n,mu.n,(T)Zero);
    if(verbose >= 1)
      LOG4CXX_INFO(KrisLibrary::logger(),"Gaussian::setMaximumLikelihood: weights sum to zero\n");
    return false;
  }

  //compute mean and covariance
  if(sumw < 1e-3)
    mu /= (T)sumw;
  else
    mu *= (T)(1.0/sumw);

  MatrixT cov(mu.n,mu.n,(T)Zero);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
      for(int j=0;j<mu.n;j++)
	cov(i,j) += temp(i)*temp(j)*(T)weights[k];
    }
  }
  if(sumw < 1e-3)
    cov /= (T)sumw;
  else
    cov *= (T)(1.0/sumw);
  return setCovariance(cov,verbose);
}

template <class T>
void Gaussian<T>::setMaximumLikelihoodDiagonal(const std::vector<VectorT>& examples)
{
  assert(examples.size() >= 1);
  mu = examples[0];
  for(size_t i=1;i<examples.size();i++)
    mu += examples[i];
  mu *= (T)(1.0/examples.size());

  VectorT cov(mu.n, (T)Zero);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
	cov(i) += temp(i)*temp(i);
    }
  }
  cov *= (T)(1.0/examples.size());
  L.setZero();
  for(int i=0;i<mu.n;i++)
    L(i,i) = Sqrt(cov(i));
}

template <class T>
void Gaussian<T>::setMaximumLikelihoodDiagonal(const std::vector<VectorT>& examples,const std::vector<Real>& weights)
{
  assert(examples.size()==weights.size());
  assert(examples.size() >= 1);

  Real sumw=weights[0];
  mu.mul(examples[0], (T)weights[0]);
  for(size_t i=1;i<examples.size();i++) {
    mu.madd(examples[i], (T)weights[i]);
    sumw += weights[i];
  }
  if(sumw == 0.0) {
    //degenerate
    L.resize(mu.n,mu.n, (T)Zero);
    return;
  }

  //compute mean and covariance
  if(sumw < 1e-3)
    mu /= (T)sumw;
  else
    mu *= (T)(1.0/sumw);

  VectorT cov(mu.n, (T)Zero);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
	cov(i) += temp(i)*temp(i)*(T)weights[k];
    }
  }
  if(sumw < 1e-3)
    cov /= (T)sumw;
  else
    cov *= (T)(1.0/sumw);
  L.setZero();
  for(int i=0;i<mu.n;i++)
    L(i,i) = Sqrt(cov(i));
}


template <class T>
bool Gaussian<T>::setMaximumAPosteriori(const std::vector<VectorT>& examples,
					const VectorT& meanprior,T meanStrength,const MatrixT& covprior,T covStrength,int verbose)
{
  assert(examples.size() >= 1);
  assert(examples[0].n == meanprior.n);
  assert(covprior.m == meanprior.n);
  assert(covprior.n == meanprior.n);
  mu.mul(meanprior,meanStrength);
  for(size_t i=0;i<examples.size();i++)
    mu += examples[i];
  mu *= (T)(1.0/(meanStrength+T(examples.size())));

  MatrixT cov;
  cov.mul(covprior,covStrength);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
      for(int j=0;j<mu.n;j++)
	cov(i,j) += temp(i)*temp(j);
    }
  }
  cov *= (T)(1.0/(covStrength+T(examples.size())));
  return setCovariance(cov,verbose);  
}

template <class T>
bool Gaussian<T>::setMaximumAPosteriori(const std::vector<VectorT>& examples,const std::vector<Real>& weights,
					const VectorT& meanprior,T meanStrength,const MatrixT& covprior,T covStrength,int verbose)
{
  assert(examples.size()==weights.size());
  assert(examples.size() >= 1);
  assert(examples[0].n == meanprior.n);
  assert(covprior.m == meanprior.n);
  assert(covprior.n == meanprior.n);

  Real sumw=0.0;
  mu.mul(meanprior,meanStrength);
  for(size_t i=0;i<examples.size();i++) {
    mu.madd(examples[i], (T)weights[i]);
    sumw += weights[i];
  }
  mu *= (T)(1.0/(sumw+meanStrength));

  MatrixT cov;
  cov.mul(covprior,covStrength);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
      for(int j=0;j<mu.n;j++)
	cov(i,j) += temp(i)*temp(j)*(T)weights[k];
    }
  }
  cov *= (T)(1.0/(sumw+covStrength));
  return setCovariance(cov,verbose);
}

template <class T>
void Gaussian<T>::setMaximumAPosterioriDiagonal(const std::vector<VectorT>& examples,
					const VectorT& meanprior,T meanStrength,const VectorT& covprior,T covStrength)
{
  assert(examples.size() >= 1);
  assert(examples[0].n == meanprior.n);
  assert(covprior.n == meanprior.n);
  mu.mul(meanprior,meanStrength);
  for(size_t i=0;i<examples.size();i++)
    mu += examples[i];
  mu *= (T)(1.0/(meanStrength+T(examples.size())));

  VectorT cov;
  cov.mul(covprior,covStrength);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
	cov(i) += temp(i)*temp(i);
    }
  }
  cov *= (T)(1.0/(covStrength+T(examples.size())));
  L.setZero();
  for(int i=0;i<mu.n;i++)
    L(i,i) = Sqrt(cov(i));
}

template <class T>
void Gaussian<T>::setMaximumAPosterioriDiagonal(const std::vector<VectorT>& examples,const std::vector<Real>& weights,
					const VectorT& meanprior,T meanStrength,const VectorT& covprior,T covStrength)
{
  assert(examples.size()==weights.size());
  assert(examples.size() >= 1);
  assert(examples[0].n == meanprior.n);
  assert(covprior.n == meanprior.n);

  Real sumw=0.0;
  mu.mul(meanprior,meanStrength);
  for(size_t i=0;i<examples.size();i++) {
    mu.madd(examples[i], (T)weights[i]);
    sumw += weights[i];
  }
  mu *= (T)(1.0/(sumw+meanStrength));

  VectorT cov;
  cov.mul(covprior,covStrength);
  VectorT temp;
  for(size_t k=0;k<examples.size();k++) {
    temp.sub(examples[k],mu);
    for(int i=0;i<mu.n;i++) {
	cov(i) += temp(i)*temp(i)*(T)weights[k];
    }
  }
  cov *= (T)(1.0/(sumw+covStrength));
  L.setZero();
  for(int i=0;i<mu.n;i++)
    L(i,i) = Sqrt(cov(i));
}


template <class T>
Real Gaussian<T>::normalizationFactor() const
{
  int d = numDims();
  T det=One;
  for(int i=0;i<d;i++) {
    if(L(i,i) == 0.0) d--;
    else det*=L(i,i);
  }
  T invc = (T)(Pow(2.0*Pi,0.5*T(d))*det);
  return invc;
}

template <class T>
T Gaussian<T>::probability(const VectorT& x) const
{
  //TODO: what if the determinant is zero?
  int d = numDims();
  T det=One;
  for(int i=0;i<d;i++) {
    if(L(i,i) == 0.0) d--;
    else det*=L(i,i);
  }
  T invc = Pow(T(2.0*Pi),T(0.5*d))*det;
  VectorT x_mu,y;
  x_mu.sub(x,mu);
  y.resize(L.n);
  bool res=LBackSubstitute(L,x_mu,y);
  if(!res) {
    return 0;
  }
  assert(res);
  return Exp(-(T)Half*y.normSquared())/invc;
}

template <class T>
T Gaussian<T>::logProbability(const VectorT& x) const
{
  //TODO: what if the determinant is zero?
  int d = numDims();
  T logdet=(T)Zero;
  for(int i=0;i<d;i++) {
    if(L(i,i) == 0.0) d--;
    else logdet+=Log(L(i,i));
  }
  T loginvc = T(0.5*d*Log(2.0*Pi))+logdet;
  VectorT x_mu,y;
  x_mu.sub(x,mu);
  y.resize(L.n);
  bool res=LBackSubstitute(L,x_mu,y);
  if(!res) {
    return -Inf;
  }
  assert(res);
  //sanity check?
  //Assert(FuzzyEquals(probability(x),Exp(-Half*y.normSquared()-loginvc),1e-4));
  return -(T)Half*y.normSquared()-loginvc;
}

template <class T>
T Gaussian<T>::partialProbability(const VectorT& x,const std::vector<int>& elements) const
{
  //TODO: what if the determinant is zero?
  MatrixT cov,cov11;
  cov.mulTransposeB(L,L);
  cov11.resize(elements.size(),elements.size());
  for(size_t i=0;i<elements.size();i++)
    for(size_t j=0;j<elements.size();j++)
      cov11(i,j) = cov(elements[i],elements[j]);
  LDLDecomposition<T> chol;
  chol.verbose = 1;
  chol.set(cov11);
  VectorT D;
  chol.getD(D);
  T det = 1.0;
  for(int i=0;i<D.n;i++)  det *= D(i);

  T invc = (T)(Pow(2.0*Pi,0.5*T(elements.size()))*det);
  VectorT mu1(elements.size());
  for(int i=0;i<mu1.n;i++)
    mu1(i) = mu(elements[i]);
  VectorT x_mu,y;
  x_mu.sub(x,mu1);
  bool res=chol.backSub(x_mu,y);
  return (T)Exp(-Half*dot(x_mu,y))/invc;
}

template <class T>
T Gaussian<T>::logPartialProbability(const VectorT& x,const std::vector<int>& elements) const
{
  //TODO: what if the determinant is zero?
  MatrixT cov,cov11;
  cov.mulTransposeB(L,L);
  cov11.resize(elements.size(),elements.size());
  for(size_t i=0;i<elements.size();i++)
    for(size_t j=0;j<elements.size();j++)
      cov11(i,j) = cov(elements[i],elements[j]);
  LDLDecomposition<T> chol;
  chol.verbose = 1;
  chol.set(cov11);
  VectorT D;
  chol.getD(D);
  T logdet = 0.0;
  int d=elements.size();
  for(int i=0;i<D.n;i++) {
    if(D(i) == 0.0) d--;
    else logdet += Log(D(i));
  }

  T loginvc = (T)(0.5*T(d)*Log(2.0*Pi))+logdet;
  VectorT mu1(elements.size());
  for(int i=0;i<mu1.n;i++)
    mu1(i) = mu(elements[i]);
  VectorT x_mu,y;
  x_mu.sub(x,mu1);
  bool res=chol.backSub(x_mu,y);
  return -(T)Half*dot(x_mu,y)-loginvc;
}

template <class T>
void Gaussian<T>::generate(VectorT& x) const
{
  //generate a normalized gaussian distributed variable y
  VectorT y(numDims());
  for(int i=0;i<y.n;i++) y(i) = (T)RandGaussian();
  L.mul(y,x);
  x += mu;
}

template <class T>
void Gaussian<T>::setJoint(const MyT& g1,const MyT& g2)
{
  mu.resize(g1.mu.n+g2.mu.n);
  mu.copySubVector(0,g1.mu);
  mu.copySubVector(g1.mu.n,g2.mu);
  L.resize(mu.n,mu.n);
  L.setZero();
  L.copySubMatrix(0,0,g1.L);
  L.copySubMatrix(g1.mu.n,g1.mu.n,g2.L);
}

template <class T>
void Gaussian<T>::setJoint(const MyT& g1,const MyT& g2,const MatrixT& corr)
{
  setJoint(g1,g2);
  assert(corr.m == g1.mu.n);
  assert(corr.n == g2.mu.n);
  //[L11 0  ][L11^T X^T  ]=[K11 K12]
  //[X   L22][0     L22^T] [K21 K22]
  //L11 X^T = K12 = corr
  //X^T = L11^-1 corr
  MatrixT X,Xt;
  X.setRef(L,g1.mu.n,0,1,1,g2.mu.n,g1.mu.n);
  Xt.setRefTranspose(X);
  if(!LBackSubstitute(g1.L,corr,Xt)) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning: Gaussian::setJoint failed!\n");
    X.setZero();
  }
}

template <class T>
void Gaussian<T>::setMarginalized(const MyT& g,int i)
{
  MatrixT cov;
  cov.mulTransposeB(g.L,g.L);
  VectorT mean1(g.mu.n-1);
  MatrixT cov11(cov.m-1,cov.n-1);
  for(int p=0;p<cov.m-1;p++) {
    int r=(p>=i?p+1:p);
    for(int q=0;q<cov.n-1;q++) {
      int c=(q>=i?q+1:q);
      cov11(p,q)=cov(r,c);
    }
    mean1(p)=g.mu(r);
  }
  mu = mean1;
  setCovariance(cov11);
}

template <class T>
void Gaussian<T>::setMarginalized(const MyT& g,const std::vector<int>& elements)
{
  MatrixT cov,newcov;
  cov.mulTransposeB(g.L,g.L);
  GaussianMarginalize(g.mu,cov,elements,mu,newcov);
  setCovariance(newcov);
}

template <class T>
void Gaussian<T>::setConditional(const MyT& g,Real xi,int i)
{
  MatrixT cov;
  cov.mulTransposeB(g.L,g.L);
  VectorT mean1(g.mu.n-1);
  MatrixT cov11(cov.m-1,cov.n-1);
  VectorT cov12(cov.m-1);
  for(int p=0;p<cov.m-1;p++) {
    int r=(p>=i?p+1:p);
    for(int q=0;q<cov.n-1;q++) {
      int c=(q>=i?q+1:q);
      cov11(p,q)=cov(r,c);
    }
    cov12(p)=cov(r,i);
    mean1(p)=g.mu(r);
  }

  T invC22= (T)(1.0/cov(i,i));
  mu = mean1;
  mu.madd(cov12,(xi-g.mu(i))*invC22);
  for(int p=0;p<cov11.m;p++)
    for(int q=0;q<cov11.n;q++)
      cov11(p,q) -= cov12(p)*cov12(q)*invC22;
  setCovariance(cov11);
}


template <class T>
void Gaussian<T>::setConditional(const MyT& g,const VectorT& x,const std::vector<int>& elements)
{
  MatrixT cov,newcov;
  cov.mulTransposeB(g.L,g.L);
  GaussianCondition(g.mu,cov,x,elements,mu,newcov);
  setCovariance(newcov);
}

//KL divergence of g from this: D(g||this)
template <class T>
T Gaussian<T>::klDivergence(const Gaussian<T>& g) const
{
  MatrixT K1inv,K2;
  this->getPrecision(K1inv);
  g.getCovariance(K2);
  VectorT temp1,temp2;
  T trK1invK2=0.0;
  for(int i=0;i<K2.n;i++) {
    K1inv.getRowRef(i,temp1);
    K2.getRowRef(i,temp2);
    trK1invK2 += temp1.dot(temp2);
  }
  int d1=0,d2=0;
  T logdet1=Zero,logdet2=Zero;
  for(int i=0;i<this->mu.n;i++) {
    if(this->L(i,i) != 0.0) {
      logdet1 += Log(this->L(i,i));
      d1++;
    }
  }
  for(int i=0;i<g.mu.n;i++) {
    if(g.L(i,i) != 0.0) {
      logdet2 += Log(g.L(i,i));
      d2++;
    }
  }
  temp1.clear();
  temp2.clear();
  temp1.sub(this->mu,g.mu);
  K1inv.mul(temp1,temp2);
  T ofs = temp1.dot(temp2);
  return (T)Half*(trK1invK2 + ofs - logdet2 + logdet1 - (d1+d2)/2);
}


template <class T>
void GaussianCondition(const VectorTemplate<T>& mean,
		       const MatrixTemplate<T>& cov,
		       const VectorTemplate<T>& x,
		       const std::vector<int>& elements,
		       VectorTemplate<T>& newmean,
		       MatrixTemplate<T>& newcov)
{
  assert(&mean != &newmean);
  assert(&cov != &newcov);
  assert(mean.n == cov.m);
  assert(mean.n == cov.n);
  assert(x.n == (int)elements.size());
  std::vector<int> map1,map2;
  std::vector<bool> keep(mean.n,true);
  for(size_t i=0;i<elements.size();i++) {
    assert(elements[i] >= 0 && elements[i] < mean.n);
    keep[elements[i]] = false;
  }
  int n=0;
  for(size_t i=0;i<keep.size();i++)
    if(keep[i]) n++;
  map1.resize(n);
  map2.resize(mean.n-n);
  n=0;
  int n2=0;
  for(size_t i=0;i<keep.size();i++)
    if(keep[i]) {
      map1[n]=i;
      n++;
    }
    else {
      map2[n2]=i;
      n2++;
    }
  assert(n2 == x.n);

  //select out the entries of the covariance matrix and mean
  VectorTemplate<T> mean1(n),mean2(n2);
  MatrixTemplate<T> cov11(n,n);
  MatrixTemplate<T> cov12(n,n2);
  MatrixTemplate<T> cov22(n2,n2);
  for(int p=0;p<n;p++) {
    int r=map1[p];
    for(int q=0;q<n;q++) {
      int c=map1[q];
      cov11(p,q)=cov(r,c);
    }
    for(int q=0;q<n2;q++) {
      int c=map2[q];
      cov12(p,q)=cov(r,c);
    }
    mean1(p)=mean(r);
  }
  for(int p=0;p<n2;p++) {
    int r=map2[p];
    for(int q=0;q<n2;q++) {
      int c=map2[q];
      cov22(p,q)=cov(r,c);
    }
    mean2(p)=mean(r);
  }

  MatrixTemplate<T> invC22;
  LDLDecomposition<T> cholesky;
  cholesky.verbose = 1;
  cholesky.set(cov22);
  cholesky.getInverse(invC22);
  //compute new mean
  newmean = mean1;
  VectorTemplate<T> temp;
  invC22.mul(x-mean2,temp);
  cov12.madd(temp,newmean);
  //compute covariance via schur complement
  MatrixTemplate<T> mtemp,mtemp2;
  mtemp.mulTransposeB(invC22,cov12);
  mtemp2.mul(cov12,mtemp);
  newcov.sub(cov11,mtemp2);
}


template <class T>
void GaussianMarginalize(const VectorTemplate<T>& mean,
			 const MatrixTemplate<T>& cov,
			 const std::vector<int>& elements,
			 VectorTemplate<T>& newmean,
			 MatrixTemplate<T>& newcov)
{
  assert(&newmean != &mean);
  assert(&newcov != &cov);
  std::vector<int> map;
  std::vector<bool> keep(mean.n,true);
  for(size_t i=0;i<elements.size();i++)
    keep[elements[i]] = false;
  int n=0;
  for(size_t i=0;i<keep.size();i++)
    if(keep[i]) n++;
  map.resize(n);
  n=0;
  for(size_t i=0;i<keep.size();i++)
    if(keep[i]) {
      map[n]=i;
      n++;
    }

  newmean.resize(n);
  newcov.resize(n,n);
  for(int p=0;p<n;p++) {
    int r=map[p];
    for(int q=0;q<n;q++) {
      int c=map[q];
      newcov(p,q)=cov(r,c);
    }
    newmean(p)=mean(r);
  }
}


template <class T>
void GaussianTransform(const VectorTemplate<T>& mean,
		       const MatrixTemplate<T>& cov,
		       const MatrixTemplate<T>& A,
		       const VectorTemplate<T>& b,
		       VectorTemplate<T>& newmean,
		       MatrixTemplate<T>& newcov)
{
  MatrixTemplate<T> temp;
  A.mul(mean,newmean);
  newmean += b;
  temp.mul(A,cov);
  newcov.mulTransposeB(temp,A);
}

template <class T>
void GaussianTransformJoint(const VectorTemplate<T>& xmean,
			    const MatrixTemplate<T>& xcov,
			    const MatrixTemplate<T>& A,
			    const VectorTemplate<T>& b,
			    VectorTemplate<T>& jointmean,
			    MatrixTemplate<T>& jointcov)
{
  Assert(A.m == b.n);
  Assert(xmean.n == A.n);
  Assert(xmean.n == xcov.m);
  Assert(xmean.n == xcov.n);
  int nx=xmean.n;
  int ny=A.m;
  jointmean.resize(nx+ny);
  jointmean.copySubVector(0,xmean);
  VectorTemplate<T> ymean;
  MatrixTemplate<T> ycov,xycov,yxcov;
  ymean.setRef(jointmean,nx,1,ny);
  jointcov.resize(jointmean.n,jointmean.n);
  jointcov.copySubMatrix(0,0,xcov);
  xycov.setRef(jointcov,0,xmean.n,1,1,nx,ny);
  yxcov.setRef(jointcov,xmean.n,0,1,1,ny,nx);
  ycov.setRef(jointcov,xmean.n,xmean.n,1,1,ny,ny);

  A.mul(xmean,ymean);
  ymean += b;
  //y = ymean + yxcov*xcovinv*(x-xmean) = b + A*x;
  //add A xcov A^T to the covariance matrix
  yxcov.mul(A,xcov);
  xycov.setTranspose(yxcov);
  ycov.mulTransposeB(yxcov,A);
}


template class Gaussian<float>;
template class Gaussian<double>;
template void GaussianCondition(const VectorTemplate<float>& mean,
		       const MatrixTemplate<float>& cov,
		       const VectorTemplate<float>& x,
		       const std::vector<int>& elements,
		       VectorTemplate<float>& newmean,
		       MatrixTemplate<float>& newcov);
template void GaussianMarginalize(const VectorTemplate<float>& mean,
			 const MatrixTemplate<float>& cov,
			 const std::vector<int>& elements,
			 VectorTemplate<float>& newmean,
			 MatrixTemplate<float>& newcov);
template void GaussianTransform(const VectorTemplate<float>& mean,
		       const MatrixTemplate<float>& cov,
		       const MatrixTemplate<float>& A,
		       const VectorTemplate<float>& b,
		       VectorTemplate<float>& newmean,
		       MatrixTemplate<float>& newcov);
template void GaussianTransformJoint(const VectorTemplate<float>& mean,
			    const MatrixTemplate<float>& cov,
			    const MatrixTemplate<float>& A,
			    const VectorTemplate<float>& b,
			    VectorTemplate<float>& jointmean,
			    MatrixTemplate<float>& jointcov);
template void GaussianCondition(const VectorTemplate<double>& mean,
		       const MatrixTemplate<double>& cov,
		       const VectorTemplate<double>& x,
		       const std::vector<int>& elements,
		       VectorTemplate<double>& newmean,
		       MatrixTemplate<double>& newcov);
template void GaussianMarginalize(const VectorTemplate<double>& mean,
			 const MatrixTemplate<double>& cov,
			 const std::vector<int>& elements,
			 VectorTemplate<double>& newmean,
			 MatrixTemplate<double>& newcov);
template void GaussianTransform(const VectorTemplate<double>& mean,
		       const MatrixTemplate<double>& cov,
		       const MatrixTemplate<double>& A,
		       const VectorTemplate<double>& b,
		       VectorTemplate<double>& newmean,
		       MatrixTemplate<double>& newcov);
template void GaussianTransformJoint(const VectorTemplate<double>& mean,
			    const MatrixTemplate<double>& cov,
			    const MatrixTemplate<double>& A,
			    const VectorTemplate<double>& b,
			    VectorTemplate<double>& jointmean,
			    MatrixTemplate<double>& jointcov);


} //namespace Math

