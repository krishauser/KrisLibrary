#include "ConstrainedDynamics.h"
#include "NewtonEuler.h"
#include "IKFunctions.h"
#include <math/LDL.h>
using namespace std;

const static Vector3 gGravity(0,0,-9.8);

void GetConstraintEquation(RobotDynamics3D& robot,const vector<int>& fixedLinks,const vector<int>& fixedDofs,Matrix& J,Vector& ddx)
{
  //compute jacobian of links
  ddx.resize(fixedLinks.size()*6+fixedDofs.size());
  J.resize(fixedLinks.size()*6+fixedDofs.size(),robot.links.size());
  if(!fixedLinks.empty()) {
    NewtonEulerSolver ne(robot);
    ne.CalcLinkAccel(Vector(robot.links.size(),Zero));
    for(size_t i=0;i<fixedLinks.size();i++) {
      Matrix Ji;
      Ji.setRef(J,i*6,0,1,1,6,robot.links.size());
      robot.GetFullJacobian(Vector3(Zero),fixedLinks[i],Ji);
      ne.accelerations[fixedLinks[i]].w.get(&ddx(i*6));
      ne.accelerations[fixedLinks[i]].v.get(&ddx(i*6+3));
    }
    ddx *= -1;
  }
  if(!fixedDofs.empty()) {
    Matrix Jfixed;
    Jfixed.setRef(J,fixedLinks.size()*6,0);
    Jfixed.setZero();
    for(size_t i=0;i<fixedDofs.size();i++)
      Jfixed(i,fixedDofs[i]) = 1;
    Vector ddxfixed;
    ddxfixed.setRef(ddx,fixedLinks.size()*6);
    ddxfixed.setZero();
  }
}

/*
  C(q) = c
  Jc(q)*dq = dc
  Jc(q)*ddq + dq^T Jc'(q)dq = ddc
  Jc(q)*ddq = ddc - dq^T Jc'(q)dq = ddx
*/
bool ConstrainedCalcAccel(RobotDynamics3D& robot,const vector<int>& fixedLinks,const std::vector<int>& fixedDofs,const Vector& t,Vector& ddq,Vector* f)
{
  Matrix J;
  Vector ddx;
  GetConstraintEquation(robot,fixedLinks,fixedDofs,J,ddx);
  return ConstrainedCalcAccel(robot,ddx,J,t,ddq,f);
}

bool ConstrainedCalcTorque(RobotDynamics3D& robot,const vector<int>& fixedLinks,const std::vector<int>& fixedDofs,const Vector& ddq,Vector& t,Vector* f)
{
  Matrix J;
  Vector ddx;
  GetConstraintEquation(robot,fixedLinks,fixedDofs,J,ddx);
  return ConstrainedCalcTorque(robot,ddx,J,ddq,t,f);
}


bool ConstrainedCalcAccel(RobotDynamics3D& robot,const vector<IKGoal>& constraints,const Vector& t,Vector& ddq,Vector* _f)
{
  RobotIKFunction f(robot);
  f.UseIK(constraints);
  Matrix J;
  f.Jacobian(robot.q,J);
  Vector zero(f.NumDimensions(),Zero);
  //WARNING: this doesn't compute residual accelerations at the constraints properly
  return ConstrainedCalcAccel(robot,zero,J,t,ddq,_f);
}

bool ConstrainedCalcTorque(RobotDynamics3D& robot,const vector<IKGoal>& constraints,const Vector& ddq,Vector& t,Vector* _f)
{
  RobotIKFunction f(robot);
  f.UseIK(constraints);
  Matrix J;
  f.Jacobian(robot.q,J);
  Vector zero(f.NumDimensions(),Zero);
  //WARNING: this doesn't compute residual accelerations at the constraints properly
  return ConstrainedCalcTorque(robot,zero,J,ddq,t,_f);
}

bool ConstrainedCalcAccel(RobotDynamics3D& robot,const Vector& ddx,const Matrix& Jc,const Vector& t,Vector& ddq,Vector* _f)
{
  /*
    Jc * ddq = ddx
    B*ddq + C + G = t + Jc^T*f
    ddq = B^-1(t + Jc^T*f - C - G )
    ddx = Jc*B^-1(t + Jc^T*f - C - G )
        = Jc*B^-1*Jc^T*f + Jc*B^-1(t - C - G)
    let B^-1(t - C - G) = ddq0
    let Jc*ddq0 = ddx0
    ddx-ddx0 = Jc*B^-1*Jc^T*f
    f = (Jc*B^-1*Jc)^-1*(ddx-ddx0)
    ddq = ddq0 + B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*(ddx-ddx0)
   */
  NewtonEulerSolver ne(robot);
  Matrix Jt,BinvJt,JBinvJt;
  Vector ddq0,ddx0;
  ne.SetGravityWrenches(gGravity);
  ne.CalcAccel(t,ddq0);
  Jc.mul(ddq0,ddx0);
  Jt.setRefTranspose(Jc);
  ne.MulKineticEnergyMatrixInverse(Jt,BinvJt);
  JBinvJt.mul(Jc,BinvJt);
  LDLDecomposition<Real> ldl;
  ldl.set(JBinvJt);
  Vector f,Jcf,BinvJCf;
  if(!ldl.backSub(ddx-ddx0,f)) {
    return false;
  }  
  Jc.mulTranspose(f,Jcf);
  ne.MulKineticEnergyMatrixInverse(Jcf,BinvJCf);
  ddq.add(ddq0,BinvJCf);
  if(_f) *_f = f;
  return true;
}

bool ConstrainedProjector(RobotDynamics3D& robot,const vector<int>& fixedLinks,const vector<int>& fixedDofs,Matrix& A,Vector& b)
{
  Matrix J;
  Vector ddx;
  GetConstraintEquation(robot,fixedLinks,fixedDofs,J,ddx);
  return ConstrainedProjector(robot,ddx,J,A,b);
}

bool ConstrainedProjector(RobotDynamics3D& robot,const Vector& ddx,const Matrix& Jc,Matrix& A,Vector& b)
{
  //from above
  //ddq = ddq0 + B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*(ddx-Jc*ddq0)
  //    = (I-B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*Jc) ddq0 + B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*ddx
  NewtonEulerSolver ne(robot);
  Matrix Jt,BinvJt,JBinvJt,JBinvJT_Inv;
  ne.SetGravityWrenches(gGravity);
  Jt.setRefTranspose(Jc);
  ne.MulKineticEnergyMatrixInverse(Jt,BinvJt);
  JBinvJt.mul(Jc,BinvJt);
  LDLDecomposition<Real> ldl;
  ldl.set(JBinvJt);
  if(!ldl.getInverse(JBinvJT_Inv)) {
    return false;
  }
  Matrix mtemp,mtemp2;
  mtemp.mul(BinvJt,JBinvJT_Inv);
  mtemp2.mul(mtemp,Jc);
  A.resize(mtemp2.m,mtemp2.n);
  for(int i=0;i<mtemp2.m;i++)
    for(int j=0;j<mtemp2.n;j++)
      A(i,j) = Delta(i,j) - mtemp2(i,j);

  Vector temp;
  if(!ldl.backSub(ddx,temp)) {
    return false;
  }  
  BinvJt.mul(temp,b);
  return true;
}


bool ConstrainedForwardDynamics(RobotDynamics3D& robot,const vector<int>& fixedLinks,const vector<int>& fixedDofs,Matrix& A,Vector& b)
{
  Matrix J;
  Vector ddx;
  GetConstraintEquation(robot,fixedLinks,fixedDofs,J,ddx);
  return ConstrainedForwardDynamics(robot,ddx,J,A,b);
}

bool ConstrainedForwardDynamics(RobotDynamics3D& robot,const Vector& ddx,const Matrix& Jc,Matrix& A,Vector& b)
{
  /*
    If ddq0 is the unconstrained accel,
    ddq0 = B^-1*t + B^-1*(-C - G)
    and by the constrained projector
    ddq = Ap*ddq0 + bp
        = Ap*B^-1 t + (bp - Ap*B^-1(C+G))
   */
  //from above: Ap = (I-B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*Jc),
  //bp = B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*ddx
  //ddq = ddq0 + B^-1 Jc^T*(Jc*B^-1*Jc^T)^-1*(ddx-Jc*ddq0)
  //    = (B^-1 - B^-1*Jc^T*(Jc*B^-1*Jc^T)^-1*Jc*B^-1) t + bp - B^-1 (C+G)
  Vector bp;
  NewtonEulerSolver ne(robot);
  Matrix Binv,BinvJt,JBinvJt,JBinvJT_Inv;
  Vector ddqref,ddxref;
  ne.SetGravityWrenches(gGravity);
  ne.CalcResidualAccel(ddqref);
  ne.CalcKineticEnergyMatrixInverse(Binv);
  BinvJt.mulTransposeB(Binv,Jc);
  JBinvJt.mul(Jc,BinvJt);
  LDLDecomposition<Real> ldl;
  ldl.set(JBinvJt);
  if(!ldl.getInverse(JBinvJT_Inv)) {
    return false;
  }

  Matrix mtemp,mtemp2;
  mtemp.mul(BinvJt,JBinvJT_Inv);
  mtemp2.mulTransposeB(mtemp,BinvJt);
  A = Binv;
  A -= mtemp2;

  Jc.mul(ddqref,ddxref);
  Vector f0;
  if(!ldl.backSub(ddx-ddxref,f0)) {
    return false;
  }  
  BinvJt.mul(f0,bp);
  b = bp + ddqref;

  /*
  //Inspection code
  Vector D;
  ldl.getD(D);
  cout<<"LDL D: "<<D<<endl;

  cout<<"Jc:"<<endl<<Jc<<endl;
  cout<<"B^-1:"<<endl<<Binv<<endl;

  Matrix B;
  robot.UpdateDynamics();
  robot.GetKineticEnergyMatrix(B);
  cout<<"B: "<<endl<<B<<endl;
  LDLDecomposition<Real> ldlB;
  ldlB.set(B);
  ldlB.getInverse(Binv);
  cout<<"Binv2: "<<Binv<<endl;

  cout<<"J*B^-1*Jt:"<<endl<<JBinvJt<<endl;
  cout<<"(J*B^-1*Jt)^-1:"<<endl<<JBinvJT_Inv<<endl;
  cout<<"Ddqref: "<<ddqref<<endl;
  cout<<"Ddxref: "<<ddxref<<endl;
  cout<<"f0:"<<f0<<endl;
  cout<<"BinvJt*f: "<<bp<<endl;
  cout<<"A:"<<endl<<A<<endl;
  cout<<"b: "<<b<<endl;
  getchar();
  */

  //checking
  /*
  for(int i=0;i<robot.q.n;i++) {
    Vector ei(robot.q.n,Zero);
    ei(i) = 1.0;
    Vector ti,temp;
    ConstrainedCalcAccel(robot,ddx,Jc,ei,ti);
    A.mul(ei,temp);
    temp += b;
    if(ti.isEqual(temp,1e-3)) {
      printf("Agreement between CalcAccel and ForwardDynamics\n");
    }
    else {
      printf("Disagreement between CalcAccel and ForwardDynamics\n");
      cout<<"CalcAccel: "<<ti<<endl;
      cout<<"ForwardDynamics: "<<temp<<endl;
      cout<<"bp: "<<bp<<endl;
      cout<<"f0: "<<f0<<endl;
      cout<<"ddqref: "<<ddqref<<endl;
      getchar();
    }
  }
  */
  return true;
}


bool ConstrainedCalcTorque(RobotDynamics3D& robot,const Vector& ddx,const Matrix& dC_dq,const Vector& ddq,Vector& t,Vector* f)
{
  Matrix A;
  Vector b;
  ConstrainedForwardDynamics(robot,ddx,dC_dq,A,b);
  //ddq = A*t + b
  Vector temp;
  temp.sub(ddq,b);
  LDLDecomposition<Real> ldl;
  ldl.set(A);
  ldl.zeroTolerance = 1e-6;
  if(!ldl.backSub(temp,t)) {
    fprintf(stderr,"ConstrainedCalcTorque could not invert dynamics matrix\n");
    return false;
  }
  return true;
  /*
    From above:
    f = (Jc*B^-1*Jc)^-1*(ddx-ddx0)
    with Jc B^-1(t - C - G) = ddx0
    so 
    t = B*ddq + C + G - Jc^T*f
      = B*ddq + C + G - Jc^T*(Jc*B^-1*Jc)^-1*(ddx-Jc B^-1(t - C - G))
      = B*ddq + C + G - Jc^T*(Jc*B^-1*Jc)^-1*(ddx+Jc B^-1(C + G)) + Jc^T*(Jc*B^-1*Jc)^-1 Jc B^-1 t
    Let B*ddq + C + D = t0
    (I - Jc^T*(Jc*B^-1*Jc)^-1 Jc B^-1)t = t0 - Jc^T*(Jc*B^-1*Jc)^-1*(accel at constraints w zero torque)
   */
  FatalError("Fast version not done yet");
  return false;
}
