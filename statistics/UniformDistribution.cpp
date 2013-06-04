#include "UniformDistribution.h"
#include <math/random.h>
using namespace Statistics;

UniformProbabilityDistribution::UniformProbabilityDistribution(Real _a,Real _b)
  :a(_a),b(_b)
{}

void UniformProbabilityDistribution::GetParameters(Vector& parameters) 
{
  parameters.resize(2);
  parameters(0)=a;
  parameters(1)=b;
}

void UniformProbabilityDistribution::SetParameters(const Vector& parameters) 
{
  Assert(parameters.n==2);
  a=parameters(0);
  b=parameters(1);
}

Real UniformProbabilityDistribution::PDF(Real value)
{
  if(value < a || value > b) return 0;
  return 1.0/(b-a);
}

Real UniformProbabilityDistribution::CDF(Real value)
{
  if(value < a) return 0;
  else if(value > b) return 1;
  return (value-a)/(b-a);
}


BoxProbabilityDistribution::BoxProbabilityDistribution()
{}

BoxProbabilityDistribution::BoxProbabilityDistribution(const Vector& a,const Vector& b)
  :bmin(a),bmax(b)
{
  Assert(bmin.n == bmax.n);
}

void BoxProbabilityDistribution::GetParameters(Vector& parameters)
{
  Assert(bmin.n == bmax.n);
  parameters.resize(bmin.n*2);
  for(int i=0;i<bmin.n;i++) {
    parameters(i*2) = bmin(i);
    parameters(i*2+1) = bmax(i);
  }
}

void BoxProbabilityDistribution::SetParameters(const Vector& parameters)
{
  Assert(parameters.n == bmin.n*2);
  for(int i=0;i<bmin.n;i++) {
    bmin(i) = parameters(i*2);
    bmax(i) = parameters(i*2+1);
  }
}

Real BoxProbabilityDistribution::PDF(const Vector& value)
{
  Assert(value.n == bmin.n);
  Real vol=1;
  for(int i=0;i<bmin.n;i++) {
    if(value(i) < bmin(i) || value(i) > bmax(i)) return 0;
    vol *= bmax(i)-bmin(i);
  }
  return 1.0/vol;
}

Real BoxProbabilityDistribution::CDF(const Vector& value)
{
  Assert(value.n == bmin.n);
  Real vol=1;
  for(int i=0;i<bmin.n;i++) {
    if(value(i) < bmin(i)) return 0;
    if(value(i) > bmax(i)) continue;
    vol *= (value(i)-bmin(i))/(bmax(i)-bmin(i));
  }
  return vol;
}

void BoxProbabilityDistribution::Covariance(Matrix& var)
{
  /* cov = integral{x} (x - mean)(x - mean)^T P(x) dx
     assume a centered box
     then cov = integral{-b->b} xx^T P(x) dx
     cov[ij] =  integral{-b->b} xi xj P(x) dx

     if i != j:
     cov[ij] =  1/(2bi*2bj) integral{-bj->bj} integral{-bi->bi} xi xj dxi dxj
     integral{-bj->bj} integral{-bi->bi} xi xj dxi dxj = 
       1/2 xi^2 | [-bi,bi] * 1/2 xj^2 | [-bj,bj] =
       0 
     
     if i == j:
     cov[ii] = 1/(2bi) integral{-bi->bi} xi^2 dxi =
       1/(6bi) xi^3 | [-bi->bi] =
       1/(3bi) bi^3 =
       1/3 bi^2 = 1/12 (b-a)^2
  */
  var.resize(bmin.n,bmin.n,Zero);
  for(int i=0;i<bmin.n;i++)
    var(i,i) = Sqr(bmax(i)-bmin(i))/12.0;
}


void BoxProbabilityDistribution::Sample(Vector& x)
{
  x.resize(bmin.n);
  for(int i=0;i<x.n;i++) x(i) = Rand(bmin(i),bmax(i));
}

UnivariateProbabilityDistribution* BoxProbabilityDistribution::Conditional(int index,const Vector& x)
{
  for(int i=0;i<x.n;i++) 
    if(i!=index) Assert(bmin[i] <= x[i] && x[i] <= bmax[i]);
  return new UniformProbabilityDistribution(bmin[index],bmax[index]);
}

MultivariateProbabilityDistribution* BoxProbabilityDistribution::Conditional(const std::vector<int> indices,const Vector& x)
{
  std::vector<bool> returned(x.n,false);
  for(size_t i=0;i<indices.size();i++)
    returned[indices[i]]=true;
  Vector newmin(indices.size()),newmax(indices.size());
  int k=0;
  for(int i=0;i<x.n;i++) {
    if(!returned[i]) Assert(x[i] <= bmin[i] && x[i] >= bmax[i]);
    else {
      newmin[k] = bmin[i];
      newmax[k] = bmax[i];
      k++;
    }
  }
  return new BoxProbabilityDistribution(newmin,newmax);
}
