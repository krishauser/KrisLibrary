#include <KrisLibrary/Logger.h>
#include "RobotDynamics3D.h"
#include <math/CholeskyDecomposition.h>
#include <Timer.h>
using namespace std;

void RobotDynamics3D::Initialize(int numLinks)
{
  RobotKinematics3D::Initialize(numLinks);
  dq.resize(numLinks,Zero);
  torqueMax.resize(numLinks,Inf);
  velMin.resize(numLinks,-Inf);
  velMax.resize(numLinks,Inf);
  powerMax.resize(numLinks,Inf);
}

void RobotDynamics3D::InitializeRigidObject()
{
  RobotKinematics3D::InitializeRigidObject();
  dq.resize(6,Zero);
  torqueMax.resize(6,Inf);
  velMin.resize(6,-Inf);
  velMax.resize(6,Inf);
  powerMax.resize(6,Inf);
}

void RobotDynamics3D::Merge(const std::vector<RobotDynamics3D*>& robots)
{
  size_t nl = 0;
  vector<size_t> offset(robots.size());
  for(size_t i=0;i<robots.size();i++) {
    offset[i] = nl;
    nl += robots[i]->links.size();
  }
  Initialize(nl);
  for(size_t i=0;i<robots.size();i++) {
    for(size_t j=0;j<robots[i]->links.size();j++) {
      links[offset[i]+j] = robots[i]->links[j];
      if(robots[i]->parents[j] >= 0)
	parents[offset[i]+j] = robots[i]->parents[j]+offset[i];
      else
	parents[offset[i]+j]=-1;
      q(offset[i]+j) = robots[i]->q(j);
      qMin(offset[i]+j) = robots[i]->qMin(j);
      qMax(offset[i]+j) = robots[i]->qMax(j);
      dq(offset[i]+j) = robots[i]->dq(j);
      torqueMax(offset[i]+j) = robots[i]->torqueMax(j);
      velMin(offset[i]+j) = robots[i]->velMin(j);
      velMax(offset[i]+j) = robots[i]->velMax(j);
      powerMax(offset[i]+j) = robots[i]->powerMax(j);
    }
  }
}

void RobotDynamics3D::Subset(const RobotDynamics3D& robot,const std::vector<int>& subset)
{
  RobotKinematics3D::Subset(robot,subset);
  for(size_t i=0;i<subset.size();i++) {
    dq(i) = robot.dq(subset[i]);
    torqueMax(i) = robot.torqueMax(subset[i]);
    velMin(i) = robot.velMin(subset[i]);
    velMax(i) = robot.velMax(subset[i]);
    powerMax(i) = robot.powerMax(subset[i]);
  }
}

void RobotDynamics3D::UpdateDynamics()
{
	UpdateFrames();
	Update_J();
}

void RobotDynamics3D::Update_J()
{
	JP.resize(q.n,q.n,Vector3(Zero));
	JO.resize(q.n,q.n,Vector3(Zero));
	for(int i=0;i<q.n;i++) {
	  Vector3 p;
	  GetWorldPosition(links[i].com,i,p);
	  //j must be ancestor of i
	  for(int j=i;j!=-1;j=parents[j]) {
	    links[j].GetJacobian(q(j),p,JO(i,j),JP(i,j));
	  }
	}
}

void RobotDynamics3D::Update_dB_dq()
{
	dB_dq.resize(q.n);
	for(int z=0;z<q.n;z++) {
		GetKineticEnergyMatrixDeriv(z,dB_dq[z]);
	}
}


bool RobotDynamics3D::InVelocityLimits(const Config& dq) const
{
  Assert(dq.n == (int)links.size());
  for(int i=0;i<dq.n;i++) {
    if(velMin[i]>dq[i] || dq[i]>velMax[i]) {
      return false;
    }
  }
  return true;
}

bool RobotDynamics3D::InPowerLimits(const Config& dq,const Vector& torques) const
{
  Assert(dq.n == (int)links.size());
  Assert(torques.n == (int)links.size());
  for(int i=0;i<dq.n;i++) {
    if(Abs(dq[i]*torques[i]) > powerMax[i])
      return false;
  }
  return true;
}

//gets the derivative of the jacobian dpi/dqj wrt t, i.e. ddpi/dqjdqk*(q')k
bool RobotDynamics3D::GetJacobianDt(const Vector3& pi, int i, int j, Vector3&dtheta_dt,Vector3& dp_dt) const
{
  dtheta_dt.setZero();
  dp_dt.setZero();
  //both j and k must be ancestors of i
  if(!IsAncestor(i,j)) return false;
  Vector3 ddtheta,ddp;
  for(int k=i;k!=-1;k=parents[k]) {
    if(GetJacobianDeriv(pi,i,j,k,ddtheta,ddp)) {
      dtheta_dt.madd(ddtheta,dq(k));
      dp_dt.madd(ddp,dq(k));
    }
  }
  return true;
}

void RobotDynamics3D::GetWorldAcceleration(const Vector3& pi, int i, const Vector& ddq, Vector3&dw,Vector3& dv) const
{
  Vector3 dw_residual,dv_residual;  //residual
  GetResidualAcceleration(pi,i,dw_residual,dv_residual);
  //this is a hack to get J*ddq
  GetWorldVelocity(pi,i,ddq,dv);
  GetWorldAngularVelocity(i,ddq,dw);
  dw += dw_residual;
  dv += dv_residual;
}


//gets the residual acceleration at pi, i.e. the accel that would result
//from no joint acceleration, ddpi/dqjdqk*(q')j*(q')k
void RobotDynamics3D::GetResidualAcceleration(const Vector3& pi, int i, Vector3&dw,Vector3& dv) const
{
  Vector3 dtheta_dt,dp_dt;
  Vector3 ddtheta,ddp;

  dw.setZero();
  dv.setZero();
  //both j and k must be ancestors of i
  for(int j=i;j!=-1;j=parents[j]) {
    dtheta_dt.setZero();
    dp_dt.setZero();
    for(int k=i;k!=-1;k=parents[k]) {
      GetJacobianDeriv_Fast(pi,i,j,k,ddtheta,ddp);
      dtheta_dt.madd(ddtheta,dq(k));
      dp_dt.madd(ddp,dq(k));
    }

    dw.madd(dtheta_dt,dq(j));
    dv.madd(dp_dt,dq(j));
  }
}

Vector3 RobotDynamics3D::GetLinearMomentum(int i) const
{
  Vector3 P;
  GetWorldVelocity(links[i].com,i,dq,P);
  P *= links[i].mass;
  return P;
}

Vector3 RobotDynamics3D::GetLinearMomentum() const
{
  Vector3 P,temp;
  for(size_t i=0;i<links.size();i++) {
    GetWorldVelocity(links[i].com,i,dq,temp);
    P += links[i].mass*temp;
  }
  return P;
}

Vector3 RobotDynamics3D::GetAngularMomentum(int i) const
{
  Vector3 w;
  Matrix3 I;
  GetWorldAngularVelocity(i,dq,w);
  links[i].GetWorldInertia(I);
  return I*w;
}

Vector3 RobotDynamics3D::GetAngularMomentum() const
{
  Vector3 w,L(Zero);
  Matrix3 I;
  for(size_t i=0;i<links.size();i++) {
    GetWorldAngularVelocity(i,dq,w);
    links[i].GetWorldInertia(I);
    L += I*w;
  }
  return L;
}

Real RobotDynamics3D::GetKineticEnergy(int i) const
{
	Vector3 comVel;
	Vector3 comAngVel;
	GetWorldVelocity(links[i].com,i,dq,comVel);
	GetWorldAngularVelocity(i,dq,comAngVel);
	Matrix3 inertiaWorld;
	links[i].GetWorldInertia(inertiaWorld);
	return Half*(links[i].mass*comVel.normSquared() + dot(comAngVel,inertiaWorld*comAngVel));
}

Real RobotDynamics3D::GetKineticEnergy() const
{
	Real val=Zero;
	for(unsigned int i=0;i<links.size();i++)
		val+=GetKineticEnergy(i);
	return val;
}

void RobotDynamics3D::GetKineticEnergyMatrix(Matrix& B) const
{
  if(JP.n != q.n || JP.m != q.n || JO.n != q.n || JO.m != q.n) 
    FatalError("RobotDynamics3D::GetKineticEnergyMatrix: Dynamics not updated");
	Matrix3 inertiaWorld;
	B.resize(q.n,q.n,Zero);
	for(int k=0;k<q.n;k++) {
	  links[k].GetWorldInertia(inertiaWorld);
	  for(int i=k;i!=-1;i=parents[i]) {
	    for(int j=k;j!=-1;j=parents[j]) {
	      //both i and j must be ancestors of k
	      B(i,j) += links[k].mass*dot(JP(k,i),JP(k,j))+dot(JO(k,i),inertiaWorld*JO(k,j));
	    }
	  }
	}
}
//let dcm[k]/dq = Jp[k], dR[k]/dq = Jo[k]
//B = sum[k] mk*Jp[k]^t*Jp[k] + Jo[k]^t*Ik*Jo[k]
//dB/dqz = sum[k] mk*(d/dqz(Jp[k])^t*Jp[k] + Jp[k]^t*d/dqz(Jp[k]))
//       + d/dqz(Jo[k])^t*Ik*Jo[k] + Jo[k]^t*Ik*d/dqz(Jo[k])
//       + Jo[k]^t*dIk/dqz*Jo[k]

/*
void RobotDynamics3D::MulKineticEnergyMatrix(const Vector& in,Vector& out) const
{
  if(JP.n != q.n || JP.m != q.n || JO.n != q.n || JO.m != q.n) 
    FatalError("RobotDynamics3D::MulKineticEnergyMatrix: Dynamics not updated");
  Assert(in.n == q.n);
  out.resize(q.n);
  out.setZero();
	Matrix3 inertiaWorld;
	for(int k=0;k<q.n;k++) {
	  links[k].GetWorldInertia(inertiaWorld);
	  for(int i=k;i!=-1;i=parents[i]) {
	    for(int j=k;j!=-1;j=parents[j]) {
	      //both i and j must be ancestors of k
	      out(i) += in(j)*(links[k].mass*dot(JP(k,i),JP(k,j))+dot(JO(k,i),inertiaWorld*JO(k,j)));
	    }
	  }
	}
}
*/

//dBij/dqz
Real RobotDynamics3D::GetKineticEnergyDeriv(int i,int j,int z) const
{
  if(JP.n != q.n || JP.m != q.n || JO.n != q.n || JO.m != q.n) 
    FatalError("RobotDynamics3D::GetKineticEnergyDeriv: Dynamics not updated");

  Real ke_dz=Zero;
  Vector3 dJOi,dJOj,dJPi,dJPj;
  Matrix3 inertiaWorld;
  for(int k=0;k<q.n;k++) {
    //i,j, and z must be ancestors of k
    if(!GetJacobianDeriv(links[k].com,k,i,z,dJOi,dJPi)) { continue; }
    if(!GetJacobianDeriv(links[k].com,k,j,z,dJOj,dJPj)) { continue; }
    Assert(IsAncestor(k,z) && IsAncestor(k,j) && IsAncestor(k,i));
    const Vector3& JOi=JO(k,i);
    const Vector3& JOj=JO(k,j);
    const Vector3& JPi=JP(k,i);
    const Vector3& JPj=JP(k,j);
    
    links[k].GetWorldInertia(inertiaWorld);
    
    Real val=Zero;
    val+=links[k].mass*(dot(JPi,dJPj)+dot(dJPi,JPj));
    //derivative of angular momentum
    val+=(dot(dJOi,inertiaWorld*JOj)+dot(JOi,inertiaWorld*dJOj));
    //coriolis forces
    //val+=([w]*inertiaWorld-inertiaWorld*[w])*JOi*JOj
    if(links[z].type == RobotLink3D::Revolute) {
      Matrix3 wcross;
      wcross.setCrossProduct(links[z].T_World.R*links[z].w);
      Matrix3 temp = wcross*inertiaWorld-inertiaWorld*wcross;
      val+=dot(JOi,temp*JOj);
    }
    ke_dz += val;
  }
  
  return ke_dz;
}

void RobotDynamics3D::GetKineticEnergyMatrixDeriv(int z,Matrix& dB) const
{
  if(JP.n != q.n || JP.m != q.n || JO.n != q.n || JO.m != q.n) 
    FatalError("RobotDynamics3D::GetKineticEnergyMatrixDeriv: Dynamics not updated");

  //O(n) descendants
  vector<bool> descendants;
  GetDescendants(z,descendants);
  for(size_t i=0;i<descendants.size();i++)
    if(descendants[i]) Assert(IsAncestor(i,z));

  dB.resize(q.n,q.n,Zero);
  //no need for an initializer, we don't use these things if they don't
  Array2D<Vector3> dJP(q.n,q.n);
  Array2D<Vector3> dJO(q.n,q.n);
  //dJp(k,j) = ddpk/djdz
  //dJo(k,j) = ddok/djdz
  //for dJP(k,j) to be nonzero, z,j must be ancestor of k
  for(int k=0;k<q.n;k++) {
    if(!descendants[k]) continue;
    for(int j=k;j!=-1;j=parents[j]) { //j<k  z<k ???
      //GetJacobianDeriv_Fast(links[k].com,k,j,z,dJO(k,j),dJP(k,j));
      if(!GetJacobianDeriv(links[k].com,k,j,z,dJO(k,j),dJP(k,j))) {
	FatalError("Error, an invalid jacobian derivative in dB setup");
	dJO(k,j).setZero();
	dJP(k,j).setZero();
      }
      if(z > j) Assert(dJO(k,j).isZero());
    }
  }
  //for an entry k to be added to i,j,
  //z, i, and j must be ancestors of k
  Matrix3 inertiaWorld;
  //sum up d/dz(Jk^t*Mk*Jk) over all links k
  for(int k=0;k<q.n;k++) {
    if(!descendants[k]) continue;
    links[k].GetWorldInertia(inertiaWorld);
    Matrix3 temp;
    if(links[z].type == RobotLink3D::Revolute) {
      Matrix3 wcross;
      wcross.setCrossProduct(links[z].T_World.R*links[z].w);
      temp.sub(wcross*inertiaWorld,inertiaWorld*wcross);
    }
    else temp.setZero();
    
    for(int i=k;i!=-1;i=parents[i]) {
      const Vector3& JPi=JP(k,i);
      const Vector3& dJPi=dJP(k,i);
      const Vector3& JOi=JO(k,i);
      const Vector3& dJOi=dJO(k,i);
      for(int j=k;j!=-1;j=parents[j]) {
	const Vector3& JPj=JP(k,j);
	const Vector3& dJPj=dJP(k,j);
	const Vector3& JOj=JO(k,j);
	const Vector3& dJOj=dJO(k,j);
	
	Real val=Zero;
	val+=links[k].mass*(dot(JPi,dJPj)+dot(dJPi,JPj));
	//derivative of angular momentum
	val+=(dot(dJOi,inertiaWorld*JOj)+dot(JOi,inertiaWorld*dJOj));
	//coriolis forces
	//val+=([w]*inertiaWorld-inertiaWorld*[w])*JOi*JOj
	val+=dot(JOi,temp*JOj);
	dB(i,j) += val;
      }
    }
  }
  
  /*
    for(int i=0;i<q.n;i++) {
    for(int j=0;j<q.n;j++) {
    Real ke_dz=GetKineticEnergyDeriv(i,j,z);
    if(!FuzzyEquals(ke_dz,dB(i,j),(Real)1e-7)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in kinetic energy deriv "<<i<<" "<<j<<" : "<<ke_dz<<" vs "<<dB(i,j));
    LOG4CXX_INFO(KrisLibrary::logger(),"Difference "<<ke_dz-dB(i,j));
    Assert(ke_dz == dB(i,j));
    }
    }
    }
  */
}

void RobotDynamics3D::GetKineticEnergyMatrixTimeDeriv(Matrix& dB) const
{
	Matrix dB_dz;
	dB.resize(q.n,q.n,Zero);
	for(int z=0;z<q.n;z++) {
		GetKineticEnergyMatrixDeriv(z,dB_dz);
		dB.madd(dB_dz,dq(z));
	}
}

void RobotDynamics3D::GetCoriolisForceMatrix(Matrix& C)
{
	C.resize(q.n,q.n);
	Update_dB_dq();
	for(int i=0;i<q.n;i++) {
		for(int j=0;j<q.n;j++) {
			Real sum = Zero;
			for(int k=0;k<q.n;k++) {
				Real dbij_dqk,dbik_dqj,dbjk_dqi;
				dbij_dqk=dB_dq[k](i,j);
				dbik_dqj=dB_dq[j](i,k);
				dbjk_dqi=dB_dq[i](j,k);
				sum += (dbij_dqk + dbik_dqj - dbjk_dqi)*dq(k);
			}
			C(i,j)=sum*Half;
		}
	}
}

void RobotDynamics3D::GetCoriolisForces(Vector& Cdq)
{
	Cdq.resize(q.n);
	Update_dB_dq();
	for(int i=0;i<q.n;i++) {
		Real val=Zero;
		for(int j=0;j<q.n;j++) {
			Real Cij = Zero;
			for(int k=0;k<q.n;k++) {
				Real dbij_dqk,dbik_dqj,dbjk_dqi;
				dbij_dqk=dB_dq[k](i,j);
				dbik_dqj=dB_dq[j](i,k);
				dbjk_dqi=dB_dq[i](j,k);
				Cij += (dbij_dqk + dbik_dqj - dbjk_dqi)*dq(k);
			}
			val += Cij*dq(j);
		}
		val*=Half;
		Cdq(i)=val;
	}
}

//B*ddq + C*dq = fext
void RobotDynamics3D::CalcAcceleration(Vector& ddq, const Vector& fext)
{
	Matrix B;
	GetKineticEnergyMatrix(B);
	CholeskyDecomposition<Real> cholesky;
	if(!cholesky.set(B)) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Kinetic energy matrix is not positive definite!");
		LOG4CXX_ERROR(KrisLibrary::logger(),B);
		Abort();
	}

	Vector Cdq;
	GetCoriolisForces(Cdq);
	Vector f_Cdq;
	if(fext.n==0) f_Cdq.setNegative(Cdq);
	else f_Cdq.sub(fext,Cdq);
	cholesky.backSub(f_Cdq,ddq);
}


void RobotDynamics3D::CalcTorques(const Vector& ddq,Vector& fext)
{
	Matrix B;
	GetKineticEnergyMatrix(B);
	B.mul(ddq,fext);

	Vector Cdq;
	GetCoriolisForces(Cdq);
	fext += Cdq;
}

