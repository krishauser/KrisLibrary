#include "quadrature.h"
#include "vector.h"

namespace Math {
namespace Quadrature {

static const Real Third=(Real)0.33333333333333333333333333333333333333;

Real Quadrature(RealFunction& f,const Real* c,const Real* x,int k)
{
  Real sum=Zero;
  for(int i=0;i<k;i++)
    sum += c[i]*f(x[i]);
  return sum;
}

void Quadrature(VectorFunction& f,const Real* c,const Real* x,int k,Vector& res)
{
  Vector temp(res.n);
  res.setZero();
  for(int i=0;i<k;i++) {
    f(x[i],temp);
    res.madd(temp,c[i]);
  }
}

Real trapezoidal(RealFunction& f, Real a, Real b)
{
	Real h = (b-a);
	return h*trapezoidal(f(a),f(b));
}

Real trapezoidal(Real f0, Real f1)
{
  return Half*(f0+f1);
}

Real simpsons(RealFunction& f, Real a, Real b)
{
	Real h = (b-a)*Half;
	return h*simpsons(f(a),f(a+h),f(b));
}

Real simpsons(Real f0, Real f1, Real f2)
{
	static const Real w = 4.0;
	return Third*(f0+w*f1+f2);
}

Real simpsons_3_8(RealFunction& f, Real a, Real b)
{
	Real h = (b-a)/3.0;
	return h*simpsons_3_8(f(a),f(a+h),f(a+h+h),f(b));
}

Real simpsons_3_8(Real f0, Real f1, Real f2, Real f3)
{
	static const Real w = 3.0;
	static const Real scale = 1.0/8.0;
	return scale*(f0+w*f1+w*f2+f3);
}

Real NC4(RealFunction& f, Real a, Real b)
{
	Real h = (b-a)/4.0;
  return h*NC4(f(a),f(a+h),f(a+h+h),f(b-h),f(b));
}

Real NC4(Real f0, Real f1, Real f2, Real f3, Real f4)
{
	static const Real w1 = 7.0, w2 = 32.0, w3 = 12.0;
	static const Real scale = 2.0/45.0;
	return scale*(w1*f0+w2*f1+w3*f2+w2*f3+w1*f4);
}

const Real xGaussian2[1] = {0.57735026918962576450914878050196};
const Real cGaussian2[1] = {1.0};
const Real xGaussian3[2] = {0,0.77459667};
const Real cGaussian3[2] = {0.88888889,0.55555555};
const Real xGaussian4[2] = {0.33998104,0.65214515};
const Real cGaussian4[2] = {0.65214515,0.34785485};
const Real xGaussian5[3] = {0.0,0.53846931,0.90617985};
const Real cGaussian5[3] = {0.56888889,0.47862867,0.23692689};
const Real xGaussian6[3] = {0.23861918,0.66120939,0.93246951};
const Real cGaussian6[3] = {0.46791393,0.36076157,0.17132449};
const Real xGaussian7[4] = {0.0,0.40584515,0.74153119,0.94910791};
const Real cGaussian7[4] = {0.41795918,0.38183005,0.27970539,0.12948497};
const Real xGaussian8[4] = {0.18343464,0.52553241,0.79666648,0.96028986};
const Real cGaussian8[4] = {0.36268378,0.31370665,0.22238103,0.10122854};
const Real xGaussian10[5] = {0.14887434,0.43339539,0.67940957,0.86506337,0.97390653};
const Real cGaussian10[5] = {0.29552422,0.26926672,0.21908636,0.14945135,0.06667134};


const Real cGaussian2_x_unit[2]={0.1819586182,0.3180413818};
const Real xGaussian2_x_unit[2]={0.3550510257,0.8449489743};
const Real cGaussian3_x_unit[3]={0.2009319140,0.06982697994,0.2292411061};const Real xGaussian3_x_unit[3]={0.9114120405,0.2123405382,0.5905331354};

//f*x^2 on [0,1]
const Real cGaussian2_x2_unit[2]={0.1007858821,0.2325474512};
const Real xGaussian2_x2_unit[2]={0.4558481560,0.8774851770};
const Real cGaussian3_x2_unit[3]={0.1571363612,0.02995070308,0.1462462695};
const Real xGaussian3_x2_unit[3]={0.9270059759,0.2949977900,0.6529962340};

//f*x(1-x) on [0,1]
const Real cGaussian2_x1x_unit[2]={0.08333333333,0.08333333333};
const Real xGaussian2_x1x_unit[2]={0.2763932023,0.7236067977};
const Real cGaussian3_x1x_unit[3]={0.08888888899,0.03888888889,0.03888888889};
const Real xGaussian3_x1x_unit[3]={0.5,0.8273268354,0.1726731646};


const int maxGaussianDegree = 10;
const Real* xGaussian [11] = {NULL,NULL,xGaussian2,xGaussian3,xGaussian4,
xGaussian5,xGaussian6,xGaussian7,xGaussian8,NULL,xGaussian10 };
const Real* cGaussian [11] = {NULL,NULL,cGaussian2,cGaussian3,cGaussian4,
cGaussian5,cGaussian6,cGaussian7,cGaussian8,NULL,cGaussian10 };

//Gaussian quadrature rule with k points on [-1,1], [a,b] respectively
Real Gaussian(RealFunction& f,int k)
{
  if(k < 2) {
    return Two*f(0);
  }
  if(k > maxGaussianDegree) k = maxGaussianDegree;
  while(!cGaussian[k]) k--;
  const Real* c=cGaussian[k];
  const Real* x=xGaussian[k];
  if(k&1) { //odd
    Real sum = c[0]*f(x[0]);
    int n=k+1/2;
    for(int i=1;i<n;i++) {
      sum += c[i]*(f(x[i])+f(-x[i]));
    }
    return sum;
  }
  else {
    Real sum=Zero;
    int n=k/2;
    for(int i=0;i<n;i++) {
      sum += c[i]*(f(x[i])+f(-x[i]));
    }
    return sum;
  }
}

Real Gaussian(RealFunction& f,Real a,Real b,int k)
{
  Real m=(a+b)*Half;
  Real h=(b-a)*Half;
  if(k < 2) {
    return h*Two*f(0);
  }
  if(k > maxGaussianDegree) k = maxGaussianDegree;
  while(!cGaussian[k]) k--;
  const Real* c=cGaussian[k];
  const Real* x=xGaussian[k];
  if(k&1) { //odd
    Real sum = c[0]*f(m+h*x[0]);
    int n=k+1/2;
    for(int i=1;i<n;i++) {
      sum += c[i]*(f(m+h*x[i])+f(m-h*x[i]));
    }
    return h*sum;
  }
  else {
    Real sum=Zero;
    int n=k/2;
    for(int i=0;i<n;i++) {
      sum += c[i]*(f(m+h*x[i])+f(m-h*x[i]));
    }
    return h*sum;
  }
}

Real compositeTrapezoidal(RealFunction& f, Real a, Real b, int n)
{
	Real h = (b-a)/n;

	int i;
	Real sum = Zero;
	for(i=1; i<n-1; i++)
		sum += f(a+h*i);
	sum = sum+sum;		//x 2
	sum += f(a) + f(b);
	sum *= Half*h;
	return sum;
}

//composite simpsons rule on n segments
//(n assumed to be even)
//integral[a,b] f dx ~= 1/3h(f(a) + 4*sum[i=1:n-1, odd]f(a+ih) + 2*sum[i=1:n-1, even]f(a+ih) + f(b)) + (b-a)O(h^4)
Real compositeSimpsons(RealFunction& f, Real a, Real b, int n)
{
	if(n%2 == 1)
		n++;
	Real h = (b-a)/n;

	int i;
	Real sum = Zero;
	for(i=1; i<n-1; i+=2)
		sum += 4*f(a+h*i);
	for(i=2; i<n-2; i+=2)
		sum += 2*f(a+h*i);
	sum += f(a) + f(b);
	sum *= h*Third;
	return sum;	
}

Real composite(QuadratureFunction q, RealFunction& f, Real a, Real b, int n)
{
	Real h = (b-a)/n;
	Real sum = 0.0;
	for(int i=0; i<n; i++)
		sum += q(f,a,a+h*i);
	return sum;
}

Real trapezoidal2D(RealFunction2& f, Real a, Real b, Real c, Real d)
{
	Real hx = (b-a), hy = (d-c);
	return 0.25*hx*hy*(f(a,c) + f(a,d) + f(b,c) + f(b,d));
}

Real simpsons2D(RealFunction2& f, Real a, Real b, Real c, Real d)
{
	const static Real w = 4.0;
	Real hx = (b-a)*0.5, hy = (d-c)*0.5;
	return hx*hy/9.0*(f(a,c) + w*f(a,c+hy) + f(a,d) +
				w*(f(a+hx,c) + w*f(a+hx,c+hy) + f(a+hx,d)) + 
				f(b,c) + w*f(b,c+hy) + f(b,d));
}

Real simpsons2D(RealFunction2& f, Real a, Real b, RealFunction& c, RealFunction& d)
{
	const static Real w = 4.0;
	Real hx = (b-a)*0.5;
	Real c1 = c(a), c2 = c(a+hx), c3 = c(b);
	Real d1 = d(a), d2 = d(a+hx), d3 = d(b);
	Real hy1 = (d1-c1)*0.5, hy2 = (d2-c2)*0.5, hy3 = (d3-c3)*0.5;
	return hx/9.0*(hy1*(f(a,c1) + w*f(a,c1+hy1) + f(a,d1)) +
				hy2*w*(f(a+hx,c2) + w*f(a+hx,c2+hy2) + f(a+hx,d2)) + 
				hy3*(f(b,c3) + w*f(b,c3+hy3) + f(b,d3)));
}

} //namespace Quadrature
} //namespace Math

