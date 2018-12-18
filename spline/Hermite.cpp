#include "Hermite.h"
#include <errors.h>
using namespace std;

namespace Spline {

///Returns the hermite interpolator between (x1,v1) and (x2,v2).  p takes parameters in the range [0,1].
void HermitePolynomial(double x1,double v1,double x2,double v2,Polynomial<double>& p)
{
  double c3 = 2*x1 - 2*x2 + v1 + v2;
  double c2 = -3*x1 + 3*x2 - 2*v1 - v2;
  double c1 = v1;
  double c0 = x1;
  p.coef.resize(4);
  p.coef[0] = c0;
  p.coef[1] = c1;
  p.coef[2] = c2;
  p.coef[3] = c3;
}

///Hermite interpolation between (x1,v1) and (x2,v2) at parameter u in range [0,1]
void HermiteInterpolate(double x1,double v1,
			double x2,double v2,
			double u,double& x,double& dx)
{
  double u2 = u*u;
  double u3 = u*u*u;
  double cx1 = 2.0*u3-3.0*u2+1;
  double cx2 = -2.0*u3+3.0*u2;
  double cv1 = u3-2.0*u2+u;
  double cv2 = u3-u2;
  double dcx1 = 6.0*u2-6.0*u;
  double dcx2 = -6.0*u2+6.0*u;
  double dcv1 = 3.0*u2-4.0*u+1.0;
  double dcv2 = 3.0*u2-2.0*u;
  x = cx1*x1 + cx2*x2 + cv1*v1 + cv2*v2;
  dx = dcx1*x1 + dcx2*x2 + dcv1*v1 + dcv2*v2;
}

///Hermite interpolation between (t1,x1,v1) and (t2,x2,v2) at time t in range [t1,t2]
void HermiteInterpolate(double t1,double x1,double v1,
			double t2,double x2,double v2,
			double t,double& x,double& dx)
{
  Assert(t2 > t1);
  double dt=(t2-t1);
  double u = (t-t1)/dt;
  double u2 = u*u;
  double u3 = u*u*u;
  double cx1 = 2.0*u3-3.0*u2+1;
  double cx2 = -2.0*u3+3.0*u2;
  double cv1 = dt*(u3-2.0*u2+u);
  double cv2 = dt*(u3-u2);
  double dcx1 = (6.0*u2-6.0*u)/dt;
  double dcx2 = (-6.0*u2+6.0*u)/dt;
  double dcv1 = 3.0*u2-4.0*u+1.0;
  double dcv2 = 3.0*u2-2.0*u;
  x = cx1*x1 + cx2*x2 + cv1*v1 + cv2*v2;
  dx = dcx1*x1 + dcx2*x2 + dcv1*v1 + dcv2*v2;
}

///Hermite interpolation between (x1,v1) and (x2,v2) at parameter u in range [0,1]
void HermiteInterpolate(const vector<double>& x1,const vector<double>& v1,
			const vector<double>& x2,const vector<double>& v2,
			double u,vector<double>& x,vector<double>& dx)
{
  Assert(x2.size()==x1.size());
  Assert(v1.size()==x1.size());
  Assert(v2.size()==x1.size());
  x.resize(x1.size());
  dx.resize(x1.size());
  double u2 = u*u;
  double u3 = u*u*u;
  double cx1 = 2.0*u3-3.0*u2+1.0;
  double cx2 = -2.0*u3+3.0*u2;
  double cv1 = (u3-2.0*u2+u);
  double cv2 = (u3-u2);
  double dcx1 = (6.0*u2-6.0*u);
  double dcx2 = (-6.0*u2+6.0*u);
  double dcv1 = 3.0*u2-4.0*u+1.0;
  double dcv2 = 3.0*u2-2.0*u;
  for(size_t i=0;i<x1.size();i++) {
    x[i] = cx1*x1[i] + cx2*x2[i] + cv1*v1[i] + cv2*v2[i];
    dx[i] = dcx1*x1[i] + dcx2*x2[i] + dcv1*v1[i] + dcv2*v2[i];
  }
}


///Hermite interpolation between (t1,x1,v1) and (t2,x2,v2) at time t in range [t1,t2]
void HermiteInterpolate(double t1,const vector<double>& x1,const vector<double>& v1,
			double t2,const vector<double>& x2,const vector<double>& v2,
			double t,vector<double>& x,vector<double>& dx)
{
  Assert(t2 >= t && t >= t1);
  Assert(x2.size()==x1.size());
  Assert(v1.size()==x1.size());
  Assert(v2.size()==x1.size());
  Assert(t2 > t1);
  x.resize(x1.size());
  dx.resize(x1.size());
  double dt=(t2-t1);
  double u = (t-t1)/dt;
  double u2 = u*u;
  double u3 = u*u*u;
  double cx1 = 2.0*u3-3.0*u2+1.0;
  double cx2 = -2.0*u3+3.0*u2;
  double cv1 = dt*(u3-2.0*u2+u);
  double cv2 = dt*(u3-u2);
  double dcx1 = (6.0*u2-6.0*u)/dt;
  double dcx2 = (-6.0*u2+6.0*u)/dt;
  double dcv1 = 3.0*u2-4.0*u+1.0;
  double dcv2 = 3.0*u2-2.0*u;
  for(size_t i=0;i<x1.size();i++) {
    x[i] = cx1*x1[i] + cx2*x2[i] + cv1*v1[i] + cv2*v2[i];
    dx[i] = dcx1*x1[i] + dcx2*x2[i] + dcv1*v1[i] + dcv2*v2[i];
  }
}


///Returns the starting and ending accelerations at u=0 and u=1
void HermiteAccelerations(double x1,double v1,
			  double x2,double v2,
			  double& a0,double& a1)
{
  a0 = -6*x1+6*x2-4*v1-2*v2;
  a1 = 6*x1-6*x2+2*v1+4*v2;
}

///Returns the starting and ending accelerations at t1 and t2
void HermiteAccelerations(double t1,double x1,double v1,
			  double t2,double x2,double v2,
			  double& a0,double& a1)
{
  Assert(t2 > t1);
  double dt=t2-t1;
  a0 = (-6*x1+6*x2)/dt-4*v1-2*v2;
  a1 = (6*x1-6*x2)/dt+2*v1+4*v2;
}


} //namespace Spline
