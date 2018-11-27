#include <KrisLibrary/Logger.h>
#include "NewtonEuler.h"
#include <math/random.h>
#include <math/CholeskyDecomposition.h>
#include <math/MatrixPrinter.h>
#include <math/VectorPrinter.h>
using namespace std;


void RigidBodyVelocity::setTransformed(const RigidBodyVelocity& vel,const RigidTransform& T)
{
  v = T.R*(vel.v + cross(vel.w,T.t));
  w = T.R*vel.w;
}

void RigidBodyVelocity::setShifted(const RigidBodyVelocity& vel,const Vector3& shift)
{
  v = vel.v + cross(vel.w,shift);
  w = vel.w;
}

void Wrench::setForceAtPoint(const Vector3& _f,const Vector3& _p)
{
  f = _f;
  m.setCross(_p,_f);
}

void Wrench::setTransformed(const Wrench& wr,const RigidTransform& T)
{
  f = T.R*wr.f;
  m = T.R*(cross(T.t,wr.f)+wr.m);
}

void Wrench::setShifted(const Wrench& wr,const Vector3& shift)
{
  f = wr.f;
  m = cross(shift,wr.f)+wr.m;
}



SpatialVector::SpatialVector()
  :Vector(6,Zero)
{}

void SpatialVector::set(const Vector3& a,const Vector3& b)
{
  a.get((*this)(0),(*this)(1),(*this)(2));
  b.get((*this)(3),(*this)(4),(*this)(5));
}

void SpatialVector::get(Vector3& a,Vector3& b) const
{
  a.set((*this)(0),(*this)(1),(*this)(2));
  b.set((*this)(3),(*this)(4),(*this)(5));
}

SpatialMatrix::SpatialMatrix()
  :Matrix(6,6,Zero)
{}

void SpatialMatrix::setUpperLeft(const Matrix3& mat11)
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      (*this)(i,j) = mat11(i,j);
}

void SpatialMatrix::setLowerRight(const Matrix3& mat22)
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      (*this)(i+3,j+3) = mat22(i,j);
}

void SpatialMatrix::setUpperRight(const Matrix3& mat12)
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      (*this)(i,j+3) = mat12(i,j);
}

void SpatialMatrix::setLowerLeft(const Matrix3& mat21)
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      (*this)(i+3,j) = mat21(i,j);
}

void SpatialMatrix::getUpperLeft(Matrix3& mat11) const
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat11(i,j) = (*this)(i,j);
}

void SpatialMatrix::getLowerRight(Matrix3& mat22) const
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat22(i,j) = (*this)(i+3,j+3);
}

void SpatialMatrix::getUpperRight(Matrix3& mat12) const
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat12(i,j) = (*this)(i,j+3);
}

void SpatialMatrix::getLowerLeft(Matrix3& mat21) const
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat21(i,j) = (*this)(i+3,j);
}

void SpatialMatrix::setForceShift(const Vector3& origMomentCenter,const Vector3& newMomentCenter)
{
  setZero();
  setIdentity();
  Matrix3 cp;
  cp.setCrossProduct(newMomentCenter-origMomentCenter);
  cp.inplaceTranspose();
  setLowerLeft(cp);
}

void SpatialMatrix::setVelocityShift(const Vector3& origRefPoint,const Vector3& newRefPoint)
{
  setZero();
  setIdentity();
  Matrix3 cp;
  cp.setCrossProduct(origRefPoint-newRefPoint);
  setUpperRight(cp);
}

void SpatialMatrix::setMassMatrix(const Real& mass,const Matrix3& inertia)
{
  setZero();
  for(int i=0;i<3;i++) (*this)(i,i) = mass;
  setLowerRight(inertia);
}




NewtonEulerSolver::NewtonEulerSolver(RobotDynamics3D& _robot)
  :robot(_robot)
{
  robot.GetChildList(children);
  velocities.resize(robot.links.size());
  accelerations.resize(robot.links.size());
  externalWrenches.resize(robot.links.size());
  jointWrenches.resize(robot.links.size());
  for(size_t i=0;i<externalWrenches.size();i++) {
    externalWrenches[i].f.setZero();
    externalWrenches[i].m.setZero();
  }
}

void NewtonEulerSolver::CalcVelocities()
{
  //go down the tree and calculate the velocities
  for(size_t n=0;n<robot.links.size();n++) {
    //get the velocity of the local frame's origin, given it's parent
    int p=robot.parents[n];
    if(p<0) {
      velocities[n].v.setZero();
      velocities[n].w.setZero();
    }
    else {
      velocities[n].v = velocities[p].v + cross(velocities[p].w,robot.links[n].T_World.t-robot.links[p].T_World.t);
      velocities[n].w = velocities[p].w;
    }
    //add on the local velocity
    Real dq=robot.dq(n);
    if(robot.links[n].type == RobotLink3D::Revolute) {
      velocities[n].w += dq*(robot.links[n].T_World.R*robot.links[n].w);
    }
    else {
      velocities[n].v += dq*(robot.links[n].T_World.R*robot.links[n].w);
    }
  }
}

void NewtonEulerSolver::CalcLinkAccel(const Vector& ddq)
{
  CalcVelocities();

  //go down the tree and calculate the accelerations
  for(size_t n=0;n<robot.links.size();n++) {
    //get the velocity of the local frame's origin, given it's parent
    int p=robot.parents[n];
    if(p<0) {
      accelerations[n].v.setZero();
      accelerations[n].w.setZero();
    }
    else {
      const Vector3 &v1=velocities[p].v, &v2=velocities[n].v;
      const Vector3 &w1=velocities[p].w, &w2=velocities[n].w;
      Vector3 odiff = robot.links[n].T_World.t-robot.links[p].T_World.t;
      accelerations[n].v = accelerations[p].v + cross(accelerations[p].w,odiff) + Two*cross(w1,v2-v1) - cross(w1,cross(w1,odiff));
      accelerations[n].w = accelerations[p].w - cross(w2,w1);
    }
    //add on the local acceleration
    Real a=ddq(n);
    if(robot.links[n].type == RobotLink3D::Revolute) {
      accelerations[n].w += a*(robot.links[n].T_World.R*robot.links[n].w);
    }
    else {
      accelerations[n].v += a*(robot.links[n].T_World.R*robot.links[n].w);
    }
  }
}

void NewtonEulerSolver::SetGravityWrenches(const Vector3& gravity)
{
  for(size_t i=0;i<externalWrenches.size();i++) {
    externalWrenches[i].f.mul(gravity,robot.links[i].mass);
    externalWrenches[i].m.setZero();
  }
}

void NewtonEulerSolver::CalcTorques(const Vector& ddq,Vector& t)
{
  CalcLinkAccel(ddq);
  t.resize(robot.links.size());
  
  Vector3 cmLocal,cmWorld;
  Vector3 vcm,wcm;
  Vector3 fcm,mcm;
  Matrix3 Iworld;
  //go down the list backwards... we can do so because parent indices are always < than their children
  for(int n=(int)robot.links.size()-1;n>=0;--n) {
    cmLocal = robot.links[n].T_World.R*robot.links[n].com;
    cmWorld = cmLocal + robot.links[n].T_World.t;
    vcm = accelerations[n].v + cross(accelerations[n].w,cmLocal) + cross(velocities[n].w,cross(velocities[n].w,cmLocal));
    wcm = accelerations[n].w;
    fcm.mul(vcm,robot.links[n].mass);
    robot.links[n].GetWorldInertia(Iworld);
    mcm = Iworld*wcm + cross(velocities[n].w,Iworld*velocities[n].w);  //inertia*accel + gyroscopic force
    
    //subtract the external force
    fcm -= externalWrenches[n].f;
    mcm -= externalWrenches[n].m;

    //add the child wrenches (must be transformed to moment about cm)
    for(size_t i=0;i<children[n].size();i++) {
      int c=children[n][i];
      Assert(c > n);
      fcm += jointWrenches[c].f;
      mcm += jointWrenches[c].m + cross(robot.links[c].T_World.t - cmWorld,jointWrenches[c].f);
    }

    //fcm,mcm must be created by joint force, moment at link origin
    //joint force/moment jf/jm causes moment jm - cmlocal x jf = mcm
    jointWrenches[n].f = fcm;
    jointWrenches[n].m = mcm + cross(cmLocal,fcm);

    //LOG4CXX_INFO(KrisLibrary::logger(),"Wrench "<<n<<": "<<jointWrenches[n].m<<", "<<jointWrenches[n].f);
    if(robot.links[n].type == RobotLink3D::Revolute)
      t(n) = dot(jointWrenches[n].m,robot.links[n].T_World.R*robot.links[n].w);
    else
      t(n) = dot(jointWrenches[n].f,robot.links[n].T_World.R*robot.links[n].w);
  }
}


void NewtonEulerSolver::CalcAccel(const Vector& t,Vector& ddq)
{
  ddq.resize(robot.links.size());
  inertiaMatrices.resize(robot.links.size());
  biasingForces.resize(robot.links.size());
  CalcVelocities();

  //initialize the matrices
  Matrix3 Iworld;
  //velocity dependent accelerations (centrifugal and coriolis terms)
  //calculated at center of mass!
  vector<SpatialVector> velDepAccels(robot.links.size());
  for(size_t n=0;n<robot.links.size();n++) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Velocity n: "<<velocities[n].v<<", "<<velocities[n].w);
    //LOG4CXX_INFO(KrisLibrary::logger()," Vel at com: "<<velocities[n].v + cross(velocities[n].w,robot.links[n].T_World.R*robot.links[n].com));
    robot.links[n].GetWorldInertia(Iworld);
    inertiaMatrices[n].setMassMatrix(robot.links[n].mass,Iworld);
    Vector3 gyroscopicForce = cross(velocities[n].w,Iworld*velocities[n].w);
    int p = robot.parents[n];
    if(p < 0) {
      const Vector3 &w2=velocities[n].w;
      Vector3 com_n_local = robot.links[n].T_World.R*robot.links[n].com;
      Vector3 v = cross(w2,cross(w2,com_n_local));
      velDepAccels[n].set(v,Vector3(Zero));
    }
    else {
      //calculate these quantities about the center of mass of each link
      Vector3 com_n_local = robot.links[n].T_World.R*robot.links[n].com;
      Vector3 com_p_local = robot.links[p].T_World.R*robot.links[p].com;
      Vector3 v1=velocities[p].v+cross(velocities[p].w,com_p_local);
      Vector3 v2=velocities[n].v+cross(velocities[n].w,com_n_local);
      const Vector3 &w1=velocities[p].w, &w2=velocities[n].w;
      Vector3 cdiff = robot.links[n].T_World*robot.links[n].com-robot.links[p].T_World*robot.links[p].com;
      Vector3 v = cross(w1,v2-v1);
      if(robot.links[n].type == RobotLink3D::Prismatic) {
	v += robot.dq(n)*cross(w1,robot.links[p].T_World.R*robot.links[n].w); 
      }
      else {
	v += cross(w2,cross(w2-w1,com_n_local));
      }
      Vector3 w = cross(w1,w2);
      velDepAccels[n].set(v,w);
    }
    biasingForces[n].set(-externalWrenches[n].f,gyroscopicForce-externalWrenches[n].m);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Velocity dependent accel: "<<velDepAccels[n]);
  }
  //go backward down the list
  SpatialMatrix cToP,pToC,IaaI,temp,temp2;
  SpatialVector A,Ia,bf_Ivda,vtemp;
  for(int n=(int)robot.links.size()-1;n>=0;--n) {
    if(children[n].empty()) continue;
    Vector3 com_n = robot.links[n].T_World*robot.links[n].com;
    for(size_t i=0;i<children[n].size();i++) {
      int c = children[n][i];
      //compute the transformations
      Vector3 com_c_local =robot.links[c].T_World.R*robot.links[c].com;
      Vector3 com_c = com_c_local + robot.links[c].T_World.t;
      cToP.setForceShift(com_c,com_n);
      pToC.setVelocityShift(com_n,com_c);
      //compute quantities with A
      Vector3 axis_w=robot.links[c].T_World.R*robot.links[c].w;
      if(robot.links[c].type == RobotLink3D::Revolute)
	A.set(cross(axis_w,com_c_local),axis_w);
      else
	A.set(axis_w,Vector3(Zero));
      inertiaMatrices[c].mul(A,Ia);
      Real aIa = Ia.dot(A);
      //IaaI.setOuterProduct(Ia);
      for(int i=0;i<6;i++)
	for(int j=0;j<6;j++)
	  IaaI(i,j) = Ia(i)*Ia(j);

      if(!(aIa > 0.0)) {
	//check if the child is a frozen link.  If so, add the inertias directly
	if(robot.qMin[c] == robot.qMax[c]) {
	  temp2.mul(cToP,temp);
	  temp.mul(temp2,pToC);
	  inertiaMatrices[n] += temp;

	  bf_Ivda = biasingForces[c];
	  inertiaMatrices[c].madd(velDepAccels[c],bf_Ivda);
	  cToP.madd(bf_Ivda,biasingForces[n]);
	  continue;
	}
		LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonEulerSolver: Warning, axis-wise inertia on link "<<n<<" is invalid; "<<aIa);
	aIa = Epsilon;
      }
      //add this child's contribution to the revised inertia matrix
      //I[n] += cToP*(I-Ia*Iat/aIa)*pToC
      temp = inertiaMatrices[c];
      temp.madd(IaaI,-1.0/aIa);
      temp2.mul(cToP,temp);
      temp.mul(temp2,pToC);
      inertiaMatrices[n] += temp;
      
      //add this child's contribution to the revised biasing force
      //bf[n] += cToP*(bf+I*vda + I*A*(t[c]-At*(bf+I*vda))/aIa)
      bf_Ivda = biasingForces[c];
      inertiaMatrices[c].madd(velDepAccels[c],bf_Ivda);
      Real At_bf_Ivda = A.dot(bf_Ivda);
      vtemp.mul(Ia,(t[c]-At_bf_Ivda)/aIa);
      vtemp += bf_Ivda;
      cToP.madd(vtemp,biasingForces[n]);
    }
    //LOG4CXX_INFO(KrisLibrary::logger(),"Revised inertia matrix "<<n<<": "<<MatrixPrinter(inertiaMatrices[n])<<"\n");
    //LOG4CXX_INFO(KrisLibrary::logger(),"Revised biasing force "<<n<<": "<<VectorPrinter(biasingForces[n])<<"\n");
    //KrisLibrary::loggerWait();
  }
  //go down the heirarchy, computing accelerations along the way
  //joint force is X[cm->origin]*(I*a+bf)
  //NOTE: acceleration variables are in com frame, transform them later
  SpatialVector nAccel,pAccel;
  for(size_t n=0;n<robot.links.size();n++) {
    Vector3 com_n_local =robot.links[n].T_World.R*robot.links[n].com;
    //compute quantities with A
    Vector3 axis_w=robot.links[n].T_World.R*robot.links[n].w;
    if(robot.links[n].type == RobotLink3D::Revolute)
      A.set(cross(axis_w,com_n_local),axis_w);
    else
      A.set(axis_w,Vector3(Zero));
    inertiaMatrices[n].mul(A,Ia);

    int p=robot.parents[n];
    if(p>=0) {
      vtemp.set(accelerations[p].v,accelerations[p].w);
      Vector3 com_n = robot.links[n].T_World*robot.links[n].com;
      Vector3 com_p = robot.links[p].T_World*robot.links[p].com;
      //which way, + or -?
      pToC.setVelocityShift(com_p,com_n);
      pToC.mul(vtemp,pAccel);
    }
    else 
      pAccel.setZero();

    Real aIa = Ia.dot(A);
    if(!(aIa > 0.0)) {
      if(robot.qMin[n] == robot.qMax[n]) {
	ddq(n) = 0;
	accelerations[n].v.setZero();
	accelerations[n].w.setZero();
	continue;
      }
            LOG4CXX_ERROR(KrisLibrary::logger(),"NewtonEulerSolver: Warning, axis-wise inertia on link "<<n<<" is invalid; "<<aIa);
      aIa = Epsilon;
    }
    Assert(aIa > 0.0);

    //ddq = (t-A^t*I*X[p->n]*a[p] - A^t(bf+I*vda))/(A^tIA)
    bf_Ivda = biasingForces[n];
    inertiaMatrices[n].madd(velDepAccels[n],bf_Ivda);
    ddq(n) = t(n)-Ia.dot(pAccel);
    ddq(n) -= A.dot(bf_Ivda);
    ddq(n) /= aIa;

    //a = X[p->n]*a[p] + vda + q''*A
    vtemp = velDepAccels[n];
    vtemp.madd(A,ddq(n));
    vtemp += pAccel;
    //vtemp is now the acceleration about the center of mass
    vtemp.get(accelerations[n].v,accelerations[n].w);
    //LOG4CXX_INFO(KrisLibrary::logger(),"CM acceleration: "<<vn.v<<", "<<vn.w);
  }

  for(size_t n=0;n<robot.links.size();n++) {
    Vector3 com_n_local =robot.links[n].T_World.R*robot.links[n].com;

    //transform to origin
    RigidBodyVelocity vn=accelerations[n];
    accelerations[n].setShifted(vn,-com_n_local);

    SpatialVector jointForce = biasingForces[n];
    inertiaMatrices[n].madd(vtemp,jointForce);
    //now jointForce is in the com frame
    Wrench fn;
    jointForce.get(fn.f,fn.m);
    //transform to joint frame
    jointWrenches[n].setShifted(fn,-com_n_local);
  }
}

void NewtonEulerSolver::CalcKineticEnergyMatrix(Matrix& B)
{
  //by virtue of the relationship
  //Bq'' + C + G = t
  //if we let q'' = ei, we can get column i of B (denoted Bi)
  //such that Bi = t-C-G
  //C+G can be obtained by setting q''=0, then t = C+G
  B.resize(robot.links.size(),robot.links.size());
  Vector t0,t;
  Vector ddq(robot.links.size());
  ddq.setZero();
  CalcTorques(ddq,t0);
  for(int i=0;i<ddq.n;i++) {
    ddq(i) = 1;
    CalcTorques(ddq,t);
    t -= t0;
    B.copyCol(i,t);
    ddq(i) = 0;
  }
}

void NewtonEulerSolver::MulKineticEnergyMatrix(const Vector& x,Vector& Bx)
{
  Assert(x.n == (int)robot.links.size());
  Vector t0;
  Vector ddq(robot.links.size());
  ddq.setZero();
  CalcTorques(ddq,t0);
  CalcTorques(x,Bx);
  Bx -= t0;
}

void NewtonEulerSolver::MulKineticEnergyMatrix(const Matrix& A,Matrix& BA)
{
  Assert(A.m == (int)robot.links.size());
  Vector t0;
  Vector ddq(robot.links.size());
  ddq.setZero();
  CalcTorques(ddq,t0);
  BA.resize(A.m,A.n);
  for(int i=0;i<A.n;i++) {
    Vector Ai,BAi;
    A.getColRef(i,Ai);
    BA.getColRef(i,BAi);
    CalcTorques(Ai,BAi);
    BAi -= t0;
  }
}

void NewtonEulerSolver::SelfTest()
{
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() first testing basic stuff.");
  {
    Real h=1e-3,tol=1e-2;
    //let Jp = jacobian of position/orientation, col j=dp/dqj
    //dJp/dqi = hessian = d^2p/dqidqj
    Matrix J0,J1;
    Matrix H1,H2,H3,H4,H5,H6;
    Matrix* Ho[3]={&H1,&H2,&H3};
    Matrix* Hp[3]={&H4,&H5,&H6};
    for(size_t m=0;m<robot.links.size();m++) {
      robot.UpdateFrames();
      robot.GetJacobianDeriv(robot.links[m].com,m,Ho,Hp);
      robot.GetFullJacobian(robot.links[m].com,m,J0);
      for(size_t i=0;i<robot.links.size();i++) {
	Real oldq = robot.q(i);
	robot.q(i) += h;
	robot.UpdateFrames();
	robot.GetFullJacobian(robot.links[m].com,m,J1);
	J1 -= J0;
	J1 *= 1.0/h;
	Matrix Hi(6,robot.links.size());
	Vector temp;
	H1.getColRef(i,temp); Hi.copyRow(0,temp);
	H2.getColRef(i,temp); Hi.copyRow(1,temp);
	H3.getColRef(i,temp); Hi.copyRow(2,temp);
	H4.getColRef(i,temp); Hi.copyRow(3,temp);
	H5.getColRef(i,temp); Hi.copyRow(4,temp);
	H6.getColRef(i,temp); Hi.copyRow(5,temp);
	if(!J1.isEqual(Hi,tol*Max(Hi.maxAbsElement(),1.0))) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing hessian!");
	  LOG4CXX_ERROR(KrisLibrary::logger(),"dp"<<m<<"/dq"<<i<<":");
	  LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(Hi,MatrixPrinter::AsciiShade));
	  LOG4CXX_ERROR(KrisLibrary::logger(),"diff");
	  LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(J1,MatrixPrinter::AsciiShade));
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error: ");
	  J1 -= Hi;
	  LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(J1,MatrixPrinter::AsciiShade));
	  Abort();
	}
	robot.q(i) = oldq;
      }
    }
    

    Matrix B0,B1,dB;
    robot.UpdateFrames();
    robot.UpdateDynamics();
    robot.GetKineticEnergyMatrix(B0);
    for(size_t i=0;i<robot.links.size();i++) {
      Real oldq = robot.q(i);
      robot.GetKineticEnergyMatrixDeriv(i,dB);
      robot.q(i) += h;
      robot.UpdateFrames();
      robot.UpdateDynamics();
      robot.GetKineticEnergyMatrix(B1);
      robot.q(i) = oldq;

      B1 -= B0;
      B1 *= 1.0/h;
      if(!B1.isEqual(dB,tol*Max(dB.maxAbsElement(),1.0))) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing kinetic energy matrix derivative!");
	LOG4CXX_ERROR(KrisLibrary::logger(),"dB/dq"<<i<<":");
	LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(dB,MatrixPrinter::AsciiShade));
	LOG4CXX_ERROR(KrisLibrary::logger(),"dB diff");
	LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(B1,MatrixPrinter::AsciiShade));
	LOG4CXX_ERROR(KrisLibrary::logger(),"Error: ");
	B1 -= dB;
	LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(B1,MatrixPrinter::AsciiShade));
	Abort();
      }
    }
    robot.UpdateFrames();
    robot.UpdateDynamics();
  }

  //check velocity computation
  CalcVelocities();
  Vector3 v,w;
  for(size_t i=0;i<velocities.size();i++) {
    robot.GetWorldVelocity(Vector3(Zero),i,robot.dq,v);
    robot.GetWorldAngularVelocity(i,robot.dq,w);
    Assert(v.isEqual(velocities[i].v,1e-3));
    Assert(w.isEqual(velocities[i].w,1e-3));
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Passed velocity test.");

  //check acceleration computation
  Vector ddq(robot.links.size());
  Vector t,ddqtest;
  //fill with random values
  for(int i=0;i<ddq.n;i++)
    ddq(i) = Rand(-One,One);
  CalcLinkAccel(ddq);
  vector<RigidBodyVelocity> oldVels=velocities;
  Vector oldDq=robot.dq;
  Vector oldq=robot.q;
  
  Real h=1e-3;
  vector<RigidBodyVelocity> v1,v2,adiff;
  robot.q.madd(oldDq,h);
  robot.dq.madd(ddq,h);
  robot.UpdateFrames();
  CalcVelocities();
  v2=velocities;

  robot.q.madd(oldDq,-Two*h);
  robot.dq.madd(ddq,-Two*h);
  robot.UpdateFrames();
  CalcVelocities();
  v1=velocities;
  
  adiff.resize(v1.size());
  for(size_t i=0;i<adiff.size();i++) {
    adiff[i].v = (v2[i].v-v1[i].v)/(Two*h);
    adiff[i].w = (v2[i].w-v1[i].w)/(Two*h);
  }

  Real tol=1e-3;
  for(size_t i=0;i<adiff.size();i++) {
    if(!adiff[i].v.isEqual(accelerations[i].v,tol*Max(1.0,accelerations[i].v.maxAbsElement()))) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing linear acceleration at link "<<i<<": "<<accelerations[i].v<<" vs. "<<adiff[i].v);
      int p=robot.parents[i];
      if(p != -1) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Linear velocity is "<<oldVels[i].v<<", angular "<<oldVels[i].w);
	LOG4CXX_INFO(KrisLibrary::logger(),"Parent's linear is "<<oldVels[p].v<<", angular "<<oldVels[p].w);
      }
      KrisLibrary::loggerWait();
    }
    if(!adiff[i].w.isEqual(accelerations[i].w,tol*Max(1.0,accelerations[i].w.maxAbsElement()))) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing angular acceleration at link "<<i<<": "<<accelerations[i].w<<" vs. "<<adiff[i].w);
      int p=robot.parents[i];
      if(p != -1) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Linear velocity is "<<oldVels[i].v<<", angular "<<oldVels[i].w);
	LOG4CXX_INFO(KrisLibrary::logger(),"Parent's linear is "<<oldVels[p].v<<", angular "<<oldVels[p].w);
      }
      KrisLibrary::loggerWait();
    }
  }
  robot.q=oldq;
  robot.dq=oldDq;
  robot.UpdateFrames();
  velocities=oldVels;
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Passed acceleration test.");

  //check coriolis forces (set ddq = G = 0)
  SetGravityWrenches(Vector3(Zero));
  ddq.setZero();
  CalcTorques(ddq,t);
  Vector C;
  robot.UpdateDynamics();
  robot.GetCoriolisForces(C);
  if(!t.isEqual(C,1e-3)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Coriolis forces computed traditionally do not match with"
      <<"newton-euler!");
    LOG4CXX_ERROR(KrisLibrary::logger(),"N-E: "<<VectorPrinter(t));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Traditional: "<<VectorPrinter(C));
    t -= C;
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error: "<<VectorPrinter(t));
    Abort();
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Passed coriolis force test.");

  //check forward dynamics
  ddq.setZero();
  ddq(0) = 1.0;
  //for(int i=0;i<ddq.n;i++)
    //ddq(i) = Rand(-One,One);
  //SetGravityWrenches(Vector3(0,0,-9.8));
  for(int i=0;i<ddq.n;i++) {
    ddq(i) = 1.0;
    CalcTorques(ddq,t);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Torques: "<<t);
    CalcAccel(t,ddqtest);
    if(!ddq.isEqual(ddqtest,1e-4)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Forward dynamics is not an exact inverse of inverse dynamics!");
      LOG4CXX_ERROR(KrisLibrary::logger(),"Desired ddq: "<<ddq);
      LOG4CXX_ERROR(KrisLibrary::logger(),"Resulting: "<<ddqtest);
      ddqtest -= ddq;
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error: "<<ddqtest);
      Abort();
    }
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Passed forward dynamics test.");

  //check kinetic energy computations
  Matrix B,Btrue,Btemp;
  CalcKineticEnergyMatrix(B);
  robot.GetKineticEnergyMatrix(Btrue);
  if(!B.isEqual(Btrue,1e-3)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Kinetic energy matrix computed with newton-euler doesn't"
      <<"match that of traditional method!");
    LOG4CXX_ERROR(KrisLibrary::logger(),"N-E:");
    LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(B,MatrixPrinter::AsciiShade));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Traditional:");
    LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(Btrue,MatrixPrinter::AsciiShade));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error: ");
    Btemp.sub(Btrue,B);
    LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(Btemp,MatrixPrinter::AsciiShade));
    Abort();
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Passed kinetic energy matrix test.");

  //check inverse kinetic energy computations
  Matrix Binv;
  CalcKineticEnergyMatrixInverse(Binv);
  Btemp.mul(Btrue,Binv);
  for(int i=0;i<ddq.n;i++) Btemp(i,i)-=1.0;
  if(!Btemp.isZero(1e-3)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Inverse kinetic energy matrix computed with newton-euler"
	<<"doesn't match that of traditional method!");
    LOG4CXX_ERROR(KrisLibrary::logger(),"N-E:");
    LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(Binv,MatrixPrinter::AsciiShade));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Traditional:");
    Matrix BinvTrue;
    CholeskyDecomposition<Real> chol(Btrue);
    chol.getInverse(BinvTrue);
    LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(BinvTrue,MatrixPrinter::AsciiShade));
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error: ");
    LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(Btemp,MatrixPrinter::AsciiShade));
    Abort();
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Passed kinetic energy inverse test.");
  LOG4CXX_INFO(KrisLibrary::logger(),"NewtonEulerSolver::SelfTest() Done!");
}

void NewtonEulerSolver::CalcKineticEnergyMatrixInverse(Matrix& Binv)
{
  //by virtue of the relationship
  //Bq'' + C + G = t
  //q'' = B^-1(t-C-G)
  //if we let t = ei, we can get column i of B^-1 (denoted Bi)
  //such that Bi = t-B^-1(C+G)
  //-B^-1(C+G) can be obtained by setting t=0, then q'' = -B^-1(C+G)
  Binv.resize(robot.links.size(),robot.links.size());
  Vector t(robot.links.size());
  Vector ddq0,ddq;
  t.setZero();
  CalcAccel(t,ddq0);
  for(int i=0;i<t.n;i++) {
    t(i) = 1;
    CalcAccel(t,ddq);
    ddq -= ddq0;
    Binv.copyCol(i,ddq);
    t(i) = 0;
  }
}

void NewtonEulerSolver::MulKineticEnergyMatrixInverse(const Vector& x,Vector& Binvx)
{
  Assert(x.n == (int)robot.links.size());
  Vector t(robot.links.size());
  Vector ddq0;
  t.setZero();
  CalcAccel(t,ddq0);
  CalcAccel(x,Binvx);
  Binvx -= ddq0;
}

void NewtonEulerSolver::MulKineticEnergyMatrixInverse(const Matrix& A,Matrix& BinvA)
{
  Assert(A.m == (int)robot.links.size());
  Vector t(robot.links.size());
  Vector ddq0;
  t.setZero();
  CalcAccel(t,ddq0);
  BinvA.resize(A.m,A.n);
  for(int i=0;i<A.n;i++) {
    Vector Ai,BAi;
    A.getColRef(i,Ai);
    BinvA.getColRef(i,BAi);
    CalcAccel(Ai,BAi);
    BAi -= ddq0;
  }
}

void NewtonEulerSolver::CalcResidualTorques(Vector& CG)
{
  Vector ddq(robot.links.size(),Zero);
  CalcTorques(ddq,CG);
}

void NewtonEulerSolver::CalcResidualAccel(Vector& ddq0)
{
  Vector t(robot.links.size(),Zero);
  CalcAccel(t,ddq0);
}

