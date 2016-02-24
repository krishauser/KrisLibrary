#include "Simulate.h"


void DiffEq2Function::X1X2ToY(const Vector& x1, const Vector& x2, Vector& y)
{
  y.resize(x1.n+x2.n);
  for(int i=0;i<x1.n;i++) {
    y(i)=x1(i);
    y(i+x1.n)=x2(i);
  }
}
void DiffEq2Function::YToX1X2(const Vector& y, Vector& x1, Vector& x2)
{
  x1.resize(y.n/2);
  x2.resize(y.n/2);		
  for(int i=0;i<x1.n;i++) {
    x1(i)=y(i);
    x2(i)=y(i+x1.n);
  }
}


template <class Robot,class NVector>
SimFunction<Robot,NVector>::SimFunction()
  :robot(NULL),gravity(Zero),viscousFriction(Zero)
{}

template <class Robot,class NVector>
void SimFunction<Robot,NVector>::GetDDx(Real t,const Vector& q,const Vector& dq,Vector& ddq)
{
  robot->q=q;
  robot->dq=dq;
  robot->UpdateDynamics();
  GetTorques(t,q,dq,fext);
  robot->GetAcceleration(ddq,fext);
}

template <class Robot,class NVector>
void SimFunction<Robot,NVector>::GetTorques(Real t,const Vector&q,const Vector&dq,Vector& fext)
{
  if(!gravity.isZero()) {
    robot->GetGravityTorques(gravity,fext);
  }
  if(viscousFriction != 0) {
    for(int i=0;i<fext.n;i++)
      fext(i) -= viscousFriction*dq(i);
  }
}

template <class Robot,class NVector>
void SimFunction<Robot,NVector>::SimulateEuler(Real dt,int nSteps)
{
  Vector y;
  DiffEq2Function::X1X2ToY(robot->q,robot->dq,y);
  Euler(this,0,dt,y,nSteps,y);
  DiffEq2Function::YToX1X2(y,robot->q,robot->dq);
}

template <class Robot,class NVector>
void SimFunction<Robot,NVector>::SimulateRK4(Real dt,int nSteps)
{
  Vector y;
  DiffEq2Function::X1X2ToY(robot->q,robot->dq,y);
  RungeKutta4(this,0,dt,y,nSteps,y);
  DiffEq2Function::YToX1X2(y,robot->q,robot->dq);
}

