#ifndef ROBOTICS_SIMULATE_H
#define ROBOTICS_SIMULATE_H

#include "RobotDynamics3D.h"
#include "DynamicChain3D.h"
#include "DynamicChain.h"
#include <KrisLibrary/math/diffeq.h>

//second order differential equation
class DiffEq2Function : public DiffEqFunction
{
public:
	//y=(x,dx),dy=(dx,ddx)
	virtual void Eval(Real t, const Vector& y, Vector& fy)
	{
		YToX1X2(y,x,dx);
		GetDDx(t,x,dx,ddx);
		X1X2ToY(dx,ddx,fy);
	}

	virtual void GetDDx(Real t,const Vector& x,const Vector& dx,Vector& ddx) =0;

	Vector x,dx,ddx;


	static void X1X2ToY(const Vector& x1, const Vector& x2, Vector& y);
	static void YToX1X2(const Vector& y, Vector& x1, Vector& x2);
};

///Deprecated: very basic numerical simulation of a robot under gravity and
///external forcing
template <class Robot,class NVector>
class SimFunction : public DiffEq2Function
{
public:
  SimFunction();
  //can be overwritten to include other external forces
  virtual void GetDDx(Real t,const Vector& q,const Vector& dq,Vector& ddq);
  virtual void GetTorques(Real t,const Vector&q,const Vector&dq,Vector& fext);
  void SimulateEuler(Real dt,int nSteps);
  void SimulateRK4(Real dt,int nSteps);
  
  Robot* robot;
  NVector gravity;
  Real viscousFriction;
  //temp
  Vector fext;
};

typedef SimFunction<RobotDynamics3D,Vector3> SimFunction3D;
typedef SimFunction<DynamicChain3D,Vector3> SimFunction3D_Old;
typedef SimFunction<DynamicChain2D,Vector2> SimFunction2D_Old;



#endif
