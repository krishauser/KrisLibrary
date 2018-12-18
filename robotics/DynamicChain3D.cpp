#include <KrisLibrary/Logger.h>
#include "DynamicChain3D.h"
#include <math/CholeskyDecomposition.h>
#include <Timer.h>

void DynamicLink3D::GetWorldInertia(Matrix3& inertiaWorld) const
{
	//inertiaWorld = R*I*Rt
	Matrix3 temp;
	temp.mul(T_World.R,inertia);
	inertiaWorld.mulTransposeB(temp,T_World.R);
}

void DynamicChain3D::Initialize(int numBodies)
{
	KinematicChain3DTemplate<DynamicLink3D>::Initialize(numBodies);
	dq.resize(numBodies,Zero);
}

void DynamicChain3D::UpdateDynamics()
{
	UpdateFrames();
	Update_J();
}

void DynamicChain3D::Update_J()
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

void DynamicChain3D::Update_dB_dq()
{
	dB_dq.resize(q.n);
	for(int z=0;z<q.n;z++) {
		GetKineticEnergyMatrixDeriv(z,dB_dq[z]);
	}
}

//gets the derivative of the jacobian dpm/dqi wrt qk, i.e. ddpm/dqidqk
bool DynamicChain3D::GetJacobianDeriv(const Vector3& pm, int m, int i, int j, Vector3&ddtheta,Vector3& ddp) const
{
	if(!IsAncestor(m,i)) return false;
	if(!IsAncestor(m,j)) return false;
	if(IsAncestor(j,i)) Swap(j,i);  //order it so j<i<=m
	GetJacobianDeriv_Fast(pm,m,i,j,ddtheta,ddp);
	return true;
}

//just like above, but assumes j<=i<=m
void DynamicChain3D::GetJacobianDeriv_Fast(const Vector3& pm, int m, int i, int j, Vector3&ddtheta,Vector3& ddp) const
{
	/*if(i==j) {
	  Vector3 p;
	  links[m].T_World.mulPoint(pm,p);
		//JPmi = Ri0[wi]Riloc*pi + vi
		//dJPmi/di = Ri0[wi][wi]Riloc*pi
		//in 3D we might as well just cross product this thing with wi
		Real JOmi; Vector3 JPmi;
		links[i].GetJacobian(q[i],p,JOmi,JPmi);
		ddp = cross(JPmi,links[i].w);
		ddtheta=Zero;
	}*/
  //Get i's Jacobian then transform to j, and modify
  Frame3D JPmi,JPjj;
  Matrix3 R0_j;
  links[i].GetJacobian(q[i],links[m].T_World,JPmi);
  links[j].GetJacobian(q[j],links[j].T_World,JPjj);
  R0_j.setTranspose(links[j].T_World.R);
  
  Matrix3 RA1B = JPjj.R*R0_j;
  Frame3D J;
  J.R = RA1B*JPmi.R;
  J.t = RA1B*JPmi.t;
  ddp = J*pm;
  ddtheta=RA1B*links[i].T_World.R*links[i].w;
}

void DynamicChain3D::GetHessian(const Vector3& pm, int m, Matrix* Htheta[3], Matrix* Hp[3]) const
{
  for(int z=0;z<3;z++) {
    if(Htheta[z]) Htheta[z]->resize(q.n,q.n,Zero);
    if(Hp[z]) Hp[z]->resize(q.n,q.n,Zero);
  }
  Vector3 ddtheta,ddp;
  //NOTE: H(i,j) != 0 only if i,j are ancestors of m
  //Also H(i,j) is symmetric, only need to consider j as an ancestor of i (j<i)
  for(int i=m;i!=-1;i=parents[i]) {
    for(int j=i;j!=-1;j=parents[j]) {
      GetJacobianDeriv_Fast(pm,m,i,j,ddtheta,ddp);
      for(int z=0;z<3;z++) {
	if(Htheta[z]) (*Htheta[z])(i,j) = (*Htheta[z])(j,i) = ddtheta[z];
	if(Hp[z]) (*Hp[z])(i,j) = (*Hp[z])(j,i) = ddp[z];
      }
    }
  }
  /*
  for(int i=0;i<q.n;i++) {
    for(int j=i;j<q.n;j++) {
      if(GetJacobianDeriv(pm,m,i,j,ddtheta,ddp)) {
        for(int z=0;z<3;z++) {
	  (*Htheta[z])(i,j)=(*Htheta[z])(j,i)=ddtheta[z];
	  (*Hp[z])(i,j)=(*Hp[z])(j,i)=ddp[z];
        }
      }
      else {
        for(int z=0;z<3;z++) {
	  (*Htheta[z])(i,j)=(*Htheta[z])(j,i)=Zero;
	  (*Hp[z])(i,j)=(*Hp[z])(j,i)=Zero;
        }
      }
    }
  }
  */
}

void DynamicChain3D::GetDirectionalHessian(const Vector3& pm, int m, const Vector3& v, Matrix& Hv) const
{
  Hv.resize(q.n,q.n,Zero);
  Vector3 ddtheta,ddp;
  //NOTE: H(i,j) != 0 only if i,j are ancestors of m
  //Also H(i,j) is symmetric, only need to consider j as an ancestor of i (j<i)
  for(int i=m;i!=-1;i=parents[i]) {
    for(int j=i;j!=-1;j=parents[j]) {
      GetJacobianDeriv_Fast(pm,m,i,j,ddtheta,ddp);
      Hv(i,j) = Hv(j,i) = dot(v,ddp);
    }
  }
}

//gets the derivative of the jacobian dpi/dqj wrt t, i.e. ddpi/dqjdqk*(q')k
bool DynamicChain3D::GetJacobianDt(const Vector3& pi, int i, int j, Vector3&dtheta_dt,Vector3& dp_dt) const
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

void DynamicChain3D::GetWorldAcceleration(const Vector3& pi, int i, const Vector& ddq, Vector3&dw,Vector3& dv) const
{
  Vector3 dw_residual,dv_residual;  //residual
  GetResidualAcceleration(pi,i,dw_residual,dv_residual);
  //this is a hack to get J*ddq
  GetWorldVelocity(pi,i,ddq,dw);
  GetWorldAngularVelocity(i,ddq,dv);
  dw += dw_residual;
  dv += dv_residual;
}

//gets the residual acceleration at pi, i.e. the accel that would result
//from no joint acceleration, ddpi/dqjdqk*(q')j*(q')k
void DynamicChain3D::GetResidualAcceleration(const Vector3& pi, int i, Vector3&dw,Vector3& dv) const
{
  dw.setZero();
  dv.setZero();
  Vector3 dtheta_dt,dp_dt;
  Vector3 ddtheta,ddp;
  //both j and k must be ancestors of i
  for(int j=i;j!=-1;j=parents[j]) {
    dtheta_dt.setZero();
    dp_dt.setZero();
    //reduce calculation by approx half
    //don't double count j
    GetJacobianDeriv_Fast(pi,i,j,j,ddtheta,ddp);
    dtheta_dt.madd(ddtheta,dq(j));
    dp_dt.madd(ddp,dq(j));
    //double count the parents of j
    for(int k=parents[j];k!=-1;k=parents[k]) {
      GetJacobianDeriv_Fast(pi,i,j,k,ddtheta,ddp);
      dtheta_dt.madd(ddtheta,dq(k)*Two);
      dp_dt.madd(ddp,dq(k)*Two);
    }
    dw.madd(dtheta_dt,dq(j));
    dv.madd(dp_dt,dq(j));
  }
}

Real DynamicChain3D::GetTotalMass() const
{
	Real totalMass=Zero;
	for(size_t i=0; i<links.size(); i++) totalMass += links[i].mass;
	return totalMass;
}

Vector3 DynamicChain3D::GetCOM() const
{
	//add up the moments of all the links, divide by total mass
	Vector3 com;
	Real totalMass=Zero;
	com.setZero();
	for(size_t i=0; i<links.size(); i++) {
		Vector3 comWorld;
		links[i].GetWorldCOM(comWorld);
		com.madd(comWorld,links[i].mass);
		totalMass += links[i].mass;
	}
	com /= totalMass;
	return com;
}

void DynamicChain3D::GetCOMJacobian(Matrix& J) const
{
  J.resize(3,q.n);

  Vector3 dp;

  J.set(Zero);
  for(int k=0;k<q.n;k++) {
    for(int j=k;j!=-1;j=parents[j]) {
      GetPositionJacobian(links[k].com,k,j,dp);
      dp *= links[k].mass;
      J(0,j) += dp.x;
      J(1,j) += dp.y;
      J(2,j) += dp.z;
    }
  }
  Real mtotal = GetTotalMass();
  J /= mtotal;
}

void DynamicChain3D::GetCOMHessian(Matrix& Hx,Matrix& Hy,Matrix& Hz) const
{
  Hx.resize(q.n,q.n,Zero);
  Hy.resize(q.n,q.n,Zero);
  Hz.resize(q.n,q.n,Zero);
  Real mtotal = GetTotalMass();
  /*
  Vector3 ddtheta,ddp;
  //NOTE: H(i,j) != 0 only if i,j are ancestors of k
  //Also H(i,j) is symmetric, only need to consider j as an ancestor of i (j<i)
  for(int k=0;k<q.n;k++) {
    for(int i=k;i!=-1;i=parents[i]) {
      for(int j=i;j!=-1;j=parents[j]) {
	GetJacobianDeriv_Fast(links[k].com,k,i,j,ddtheta,ddp);
	Hx(i,j) += links[k].mass*ddp.x;
	Hy(i,j) += links[k].mass*ddp.y;
	Hz(i,j) += links[k].mass*ddp.z;
      }
    }
  }
  //fill in lower triangle
  for(int i=0;i<q.n;i++) {
    for(int j=i+1;j<q.n;j++) {
      Hx(i,j) /= mtotal;
      Hy(i,j) /= mtotal;
      Hz(i,j) /= mtotal;
      Hx(j,i) = Hx(i,j);
      Hy(j,i) = Hy(i,j);
      Hz(j,i) = Hz(i,j);
    }
    }*/

  Matrix Htempx,Htempy,Htempz;
  Matrix* Htheta[3]={NULL,NULL,NULL};
  Matrix* Hp[3]={&Htempx,&Htempy,&Htempz};
  for(int k=0;k<q.n;k++) {
    GetHessian(links[k].com,k,Htheta,Hp);
    Hx.madd(Htempx,links[k].mass);
    Hy.madd(Htempy,links[k].mass);
    Hz.madd(Htempz,links[k].mass);
  }
  Hx /= mtotal;
  Hy /= mtotal;
  Hz /= mtotal;
}

Real DynamicChain3D::GetKineticEnergy(int i) const
{
	Vector3 comVel;
	Vector3 comAngVel;
	GetWorldVelocity(links[i].com,i,dq,comVel);
	GetWorldAngularVelocity(i,dq,comAngVel);
	Matrix3 inertiaWorld;
	links[i].GetWorldInertia(inertiaWorld);
	return Half*(links[i].mass*comVel.normSquared() + dot(comAngVel,inertiaWorld*comAngVel));
}

Real DynamicChain3D::GetKineticEnergy() const
{
	Real val=Zero;
	for(unsigned int i=0;i<links.size();i++)
		val+=GetKineticEnergy(i);
	return val;
}

void DynamicChain3D::GetKineticEnergyMatrix(Matrix& B) const
{
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

//dBij/dqz
Real DynamicChain3D::GetKineticEnergyDeriv(int i,int j,int z) const
{
	Real ke_dz=Zero;
	Vector3 dJOi,dJOj,dJPi,dJPj;
	Matrix3 inertiaWorld;
	for(int k=0;k<q.n;k++) {
		//i or j must be ancestor of k
		if(!GetJacobianDeriv(links[k].com,k,i,z,dJOi,dJPi)) { continue; }
		if(!GetJacobianDeriv(links[k].com,k,j,z,dJOj,dJPj)) { continue; }
		const Vector3& JOi=JO(k,i);
		const Vector3& JOj=JO(k,j);
		const Vector3& JPi=JP(k,i);
		const Vector3& JPj=JP(k,j);

		links[k].GetWorldInertia(inertiaWorld);
		Matrix3 wcross;  //[w_k] or [w_0]?
		wcross.setCrossProduct(links[k].w);
		Matrix3 temp = wcross*inertiaWorld-inertiaWorld*wcross;

		Real val=Zero;
		val+=links[k].mass*(dot(JPi,dJPj)+dot(dJPi,JPj));
		//derivative of angular momentum
		val+=(dot(dJOi,inertiaWorld*JOj)+dot(JOi,inertiaWorld*dJOj));
		//coriolis forces
		//val+=([w]*inertiaWorld-inertiaWorld*[w])*JOi*JOj
		val+=dot(JOi,temp*JOj);
		if(!IsAncestor(k,z) || !IsAncestor(k,j) || !IsAncestor(k,i)) {
		  assert(val == Zero);
		}
		ke_dz += val;
	}

	if(!FuzzyEquals(dB_dq[z](i,j),ke_dz)) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"They don't agree on "<<i<<" "<<j<<" "<<z);
	  LOG4CXX_INFO(KrisLibrary::logger(),dB_dq[z](i,j)<<" vs "<<ke_dz);
	  KrisLibrary::loggerWait();
	}

	return ke_dz;
}

void DynamicChain3D::GetKineticEnergyMatrixDeriv(int z,Matrix& dB) const
{
	dB.resize(q.n,q.n,Zero);
	Array2D<Vector3> dJP(q.n,q.n);
	Array2D<Vector3> dJO(q.n,q.n);
	//for dJP(i,j) to be nonzero, z,j must be ancestor of i
	for(int i=0;i<q.n;i++) {
	  if(!IsAncestor(i,z)) continue;
	  for(int j=i;j!=-1;j=parents[j]) {
	    GetJacobianDeriv_Fast(links[i].com,i,j,z,dJO(i,j),dJP(i,j));
	    /*
	    if(!GetJacobianDeriv(links[i].com,i,j,z,dJO(i,j),dJP(i,j))) { 
	      LOG4CXX_ERROR(KrisLibrary::logger(),"Error, an invalid jacobian derivative in dB setup");
	      Abort(); }
	    */
	  }
	}
	//for an entry k to be added to i,j,
	//z, i, and j must be ancestors of k
	Matrix3 inertiaWorld;
	for(int k=0;k<q.n;k++) {
	  if(!IsAncestor(k,z)) continue;
	  links[k].GetWorldInertia(inertiaWorld);
	  Matrix3 wcross;
	  wcross.setCrossProduct(links[k].w);
	  Matrix3 temp = wcross*inertiaWorld-inertiaWorld*wcross;
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
	/* check?
	for(int i=0;i<q.n;i++) {
	  for(int j=0;j<q.n;j++) {
	    Real ke_dz=GetKineticEnergyDeriv(i,j,z);
	  }
	}
	*/
	
}

void DynamicChain3D::GetKineticEnergyMatrixTimeDeriv(Matrix& dB) const
{
	Matrix dB_dz;
	dB.resize(q.n,q.n,Zero);
	for(int z=0;z<q.n;z++) {
		GetKineticEnergyMatrixDeriv(z,dB_dz);
		dB.madd(dB_dz,dq(z));
	}
}

void DynamicChain3D::GetCoriolisForceMatrix(Matrix& C)
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

void DynamicChain3D::GetCoriolisForces(Vector& Cdq)
{
  Timer timer;
	Cdq.resize(q.n);
	Update_dB_dq();
	LOG4CXX_INFO(KrisLibrary::logger(),"Update took "<<timer.ElapsedTime());
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
	LOG4CXX_INFO(KrisLibrary::logger(),"triple product took "<<timer.ElapsedTime());
}

void DynamicChain3D::GetForceVector(const Vector3& torque, const Vector3& force, int i, Vector& F) const
{
	F.resize(q.n,Zero);
	AddForceVector(torque,force,i,F);
}

void DynamicChain3D::AddForceVector(const Vector3& torque, const Vector3& force, int i, Vector& F) const
{
	//multiply by full jacobian of link i transpose
	int j=i;
	while(j!=-1) {
		F(j)+=dot(JO(i,j),torque) + dot(JP(i,j),force);
		j=parents[j];
	}
	/* non-sparse version
	Matrix Ji;
	GetFullJacobian(links[i].com,i,Ji);
	for(int i=0;i<q.n;i++) {
		F(i) += Ji(0,i)*torque + Ji(1,i)*force.x + Ji(3,i)*force.y;
	}*/
}

void DynamicChain3D::GetGravityVector(const Vector3& g0, Vector& G) const
{
	G.resize(q.n,Zero);
	Vector3 zero(Zero);
	for(int i=0;i<q.n;i++) {
		AddForceVector(zero,-links[i].mass*g0,i,G);
	}
}

Real DynamicChain3D::GetGravityPotentialEnergy(const Vector3& g0)
{
	Real val=Zero;
	for(unsigned int i=0;i<links.size();i++) {
		Vector3 comWorld;
		links[i].GetWorldCOM(comWorld);
		val += dot(g0,comWorld)*links[i].mass;
	}
	return val;
}

//B*ddq + C*dq = fext
void DynamicChain3D::GetAcceleration(Vector& ddq, const Vector& fext)
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

