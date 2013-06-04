#ifndef ROBOTICS_SIMULATE_H
#define ROBOTICS_SIMULATE_H

#include "RobotDynamics3D.h"
#include "DynamicChain3D.h"
#include "DynamicChain.h"
#include <math/diffeq.h>

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
