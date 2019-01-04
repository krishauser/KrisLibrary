#ifndef SPLINE_HERMITE_H
#define SPLINE_HERMITE_H

#include <vector>
#include "Polynomial.h"

namespace Spline {

///Returns the hermite interpolator between (x1,v1) and (x2,v2).  p takes parameters in the range [0,1].
void HermitePolynomial(double x1,double v1,double x2,double v2,Polynomial<double>& p);

///Hermite interpolation between (x1,v1) and (x2,v2) at parameter u in range [0,1]
void HermiteInterpolate(double x1,double v1,
			double x2,double v2,
			double u,double& x,double& dx);

///Hermite interpolation between (t1,x1,v1) and (t2,x2,v2) at time t in range [t1,t2]
void HermiteInterpolate(double t1,double x1,double v1,
			double t2,double x2,double v2,
			double t,double& x,double& dx);

///Hermite interpolation between (x1,v1) and (x2,v2) at parameter u in range [0,1]
void HermiteInterpolate(const std::vector<double>& x1,const std::vector<double>& v1,
			const std::vector<double>& x2,const std::vector<double>& v2,
			double u,std::vector<double>& x,std::vector<double>& dx);

///Hermite interpolation between (t1,x1,v1) and (t2,x2,v2) at time t in range [t1,t2]
void HermiteInterpolate(double t1,const std::vector<double>& x1,const std::vector<double>& v1,
			double t2,const std::vector<double>& x2,const std::vector<double>& v2,
			double t,std::vector<double>& x,std::vector<double>& dx);

///Returns the starting and ending accelerations at u=0 and u=1
void HermiteAccelerations(double x1,double v1,
			  double x2,double v2,
			  double& a0,double& a1);

///Returns the starting and ending accelerations at t1 and t2
void HermiteAccelerations(double t1,double x1,double v1,
			  double t2,double x2,double v2,
			  double& a0,double& a1);


} //namespace Spline

#endif
