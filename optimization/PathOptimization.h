#ifndef OPTIMIZATION_PATH_OPTIMIZATION_H
#define OPTIMIZATION_PATH_OPTIMIZATION_H

#include <math/LinearPath.h>
#include <math/BlockTridiagonalMatrix.h>
#include <math/function.h>
#include <math/root.h>

namespace Optimization {
  using namespace Math;
  using namespace std;

class PathOptimization 
{
public:
  PathOptimization(PiecewiseLinearPath& y);
  void Initialize();
  ConvergenceResult SDStep(Real tolx,int maxMinimizationIters); //steepest descent
  void NewtonStep();
  Real CalcObjective();
  void EvalIntegrals();
  void EvalSDIntegrals();
  void EvalIntegralsFast();
  void EvalIntegralsGaussian();
  void EvalY(Real t,Vector& y) const;
  Real EvalAlpha(Real t) const;
  void EvalMu(Real t,Vector& mu) const;
  void EvalGradients(const Vector& y,const Vector& mu,
		     Vector& g,Matrix& H,Vector& gcmu,Matrix& Hcmu) const;
  void SolveAlpha();
  void Reparameterize(int N);
  void CalcDySD();
  Real EvalDy(Real c);  //sets y=y0+c*dy, evals objective
  VectorFieldFunction* Constraint() const;

  ScalarFieldFunction* f;   //optimize f along path
  VectorFieldFunction* C;   //maintain C(x)=0, D ignored as of now
  PiecewiseLinearPath& y;
  Real h;
  bool startFixed, endFixed;

  //temp Lagrange multipliers
  Vector a;   //an element per mesh point
  vector<Vector> mu;

  //temp storage
  PiecewiseLinearPath y0;
  BlockTridiagonalMatrix A,A1,A2,A3,A4;
  BlockVector b,b1,b2,b3,b4,dy;
  vector<Real> fy;  //f evaluations
  vector<Real> f0;  //integrals of f(s) along edge
  vector<Vector> g1,g2; //integrals of s*grad f(s) of (-) side, (+) side resp
  vector<Matrix> H1,H2; //integrals of s^2*H f(s) of (-) side, (+) side resp
  vector<Matrix> H3; //integrals of s(1-s)*H f(s)
  vector<Vector> gcmu1,gcmu2; //integrals of s*grad c^t(s)*mu of (-) side, (+) side resp
  vector<Matrix> Hcmu1,Hcmu2; //integrals of s^2*Hc^t(s)*mu of (-) side, (+) side resp
  vector<Matrix> Hcmu3; //integrals of s(1-s)*Hc^t(s)*mu
  BlockTridiagonalMatrix Ya;
  Vector YaTb;
  Matrix YaTYa;
};

} // namespace Optimization

#endif

