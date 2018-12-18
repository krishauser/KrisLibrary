#include <KrisLibrary/Logger.h>
#include "DynamicChain.h"
#include <iostream>
#include <math/CholeskyDecomposition.h>

void DynamicChain2D::Initialize(int numBodies)
{
	KinematicChain2DTemplate<DynamicLink2D>::Initialize(numBodies);
	dq.resize(numBodies,Zero);
}

void DynamicChain2D::UpdateDynamics()
{
	UpdateFrames();
	Update_J();
}

void DynamicChain2D::Update_J()
{
	JP.resize(q.n,q.n);
	JO.resize(q.n,q.n);
	for(int i=0;i<q.n;i++) {
		for(int j=0;j<q.n;j++) {
			if(!GetJacobian(links[i].com,i,j,JO(i,j),JP(i,j))) { }
		}
	}
}

void DynamicChain2D::Update_dB_dq()
{
	dB_dq.resize(q.n);
	for(int i=0;i<q.n;i++)
		GetKineticEnergyMatrixDeriv(i,dB_dq[i]);
}

//gets the derivative of the jacobian dpi/dqj wrt qk, i.e. ddpi/dqjdqk
bool DynamicChain2D::GetJacobianDeriv(const Vector2& pi, int i, int j, int k, Real&ddtheta,Vector2& ddp) const
{
	if(!IsAncestor(i,j)) return false;
	if(!IsAncestor(i,k)) return false;
	/*if(j==k) {
	  Vector3 p;
	  links[i].T_World.mulPoint(pi,p);
		//JPij = Rj0[wj]Rjloc*pj + vj
		//dJPij/dj = Rj0[wj][wj]Rjloc*pj
		//in 2D we might as well just cross product this thing with wj
		Real JOij; Vector2 JPij;
		links[j].GetJacobian(q[j],p,JOij,JPij);
		ddp = cross(JPij,links[j].w);
		ddtheta=Zero;
	}*/
	if(IsAncestor(k,j)) Swap(k,j);  //order it so k<j<=i
	//Get j's Jacobian then transform to k, and modify
	Frame2D JPij,JPkk;
	Matrix2 R0_k;
	links[j].GetJacobian(q[j],links[i].T_World,JPij);
	links[k].GetJacobian(q[k],links[k].T_World,JPkk);
	R0_k.setTranspose(links[k].T_World.R);

	Matrix2 RA1B = JPkk.R*R0_k;
	Frame2D J;
	J.R = RA1B*JPij.R;
	J.t = RA1B*JPij.t;
	ddp = J*pi;
	ddtheta=Zero;
	return true;
}

Vector2 DynamicChain2D::GetCenterOfMass() const
{
	//add up the moments of all the links, divide by total mass
	Vector2 com;
	Real totalMass=Zero;
	com.setZero();
	for(size_t i=0; i<links.size(); i++) {
		Vector2 comWorld;
		links[i].GetWorldCOM(comWorld);
		com.madd(comWorld,links[i].mass);
		totalMass += links[i].mass;
	}
	com /= totalMass;
	return com;
}

Real DynamicChain2D::GetKineticEnergy(int i) const
{
	Vector2 comVel;
	Real comAngVel;
	GetWorldVelocity(links[i].com,i,dq,comVel);
	GetWorldAngularVelocity(i,dq,comAngVel);
	Real inertiaWorld;
	links[i].GetWorldInertia(inertiaWorld);
	return Half*(links[i].mass*comVel.normSquared() + inertiaWorld*comAngVel*comAngVel);
}

Real DynamicChain2D::GetKineticEnergy() const
{
	Real val=Zero;
	for(unsigned int i=0;i<links.size();i++)
		val+=GetKineticEnergy(i);
	return val;
}

void DynamicChain2D::GetKineticEnergyMatrix(Matrix& B) const
{
	B.resize(q.n,q.n);
	for(int i=0;i<q.n;i++) {
		for(int j=0;j<q.n;j++) {
			Real inertiaWorld;
			B(i,j) = Zero;
			for(int k=0;k<q.n;k++) {
				links[k].GetWorldInertia(inertiaWorld);
				B(i,j) += links[k].mass*dot(JP(k,i),JP(k,j))+inertiaWorld*JO(k,i)*JO(k,j);
			}
		}
	}
}

//dBij/dqz
Real DynamicChain2D::GetKineticEnergyDeriv(int i,int j,int z) const
{
	Real val=Zero;
	Real JOi,JOj,dJOi,dJOj;
	Vector2 JPi,JPj,dJPi,dJPj;
	Real inertiaWorld;
	for(int k=0;k<q.n;k++) {
		//if(!IsAncestor(k,z)) continue;
		//JOi=JO(k,i);
		//JOj=JO(k,j);
		JPi=JP(k,i);
		JPj=JP(k,j);
		if(!GetJacobianDeriv(links[k].com,k,i,z,dJOi,dJPi)) { dJOi=Zero; dJPi=Zero; }
		if(!GetJacobianDeriv(links[k].com,k,j,z,dJOj,dJPj)) { dJOj=Zero; dJPj=Zero; }
		links[k].GetWorldInertia(inertiaWorld);
		val+=links[k].mass*(dot(JPi,dJPj)+dot(dJPi,JPj));
		//no derivative of angular rotation
		//val+=(inertiaWorld*dJOi*JOj+inertiaWorld*JOi*dJOj);
		//no coriolis forces
		//val+=([w]*inertiaWorld-inertiaWorld*[w])*JOi*JOj
	}
	return val;
}

void DynamicChain2D::GetKineticEnergyMatrixDeriv(int z,Matrix& dB) const
{
	dB.resize(q.n,q.n);
	Array2D<Vector2> dJP(q.n,q.n);
	//no angular rotation needed
	//Array2D<Real> dJO(q.n,q.n);
	Real dJOTemp;
	for(int i=0;i<q.n;i++) {
		for(int j=0;j<q.n;j++) {
			if(!GetJacobianDeriv(links[i].com,i,j,z,dJOTemp,dJP(i,j))) { dJP(i,j)=Zero; }
		}
	}
	for(int i=0;i<q.n;i++) {
		for(int j=0;j<q.n;j++) {
			Real val=Zero;
			Real inertiaWorld;
			for(int k=0;k<q.n;k++) {
				//if(!IsAncestor(k,z)) continue;
				const Vector2& JPi=JP(k,i);
				const Vector2& JPj=JP(k,j);
				const Vector2& dJPi=dJP(k,i);
				const Vector2& dJPj=dJP(k,j);
				links[k].GetWorldInertia(inertiaWorld);
				val+=links[k].mass*(dot(JPi,dJPj)+dot(dJPi,JPj));

				//no derivative of angular rotation
				//val+=(inertiaWorld*dJOi*JOj+inertiaWorld*JOi*dJOj);
				//no coriolis forces
				//val+=([w]*inertiaWorld-inertiaWorld*[w])*JOi*JOj
			}

			dB(i,j)=val;
		}
	}
}

void DynamicChain2D::GetKineticEnergyMatrixTimeDeriv(Matrix& dB) const
{
	Matrix dB_dz;
	dB.resize(q.n,q.n,Zero);
	for(int z=0;z<q.n;z++) {
		GetKineticEnergyMatrixDeriv(z,dB_dz);
		dB.madd(dB_dz,dq(z));
	}
}

void DynamicChain2D::GetCoriolisForceMatrix(Matrix& C)
{
	std::vector<Matrix> dBdi(q.n);
	for(int z=0;z<q.n;z++)
		GetKineticEnergyMatrixDeriv(z,dBdi[z]);
	C.resize(q.n,q.n);
	Update_dB_dq();
	for(int i=0;i<q.n;i++) {
		for(int j=0;j<q.n;j++) {
			C(i,j) = Zero;
			for(int k=0;k<q.n;k++) {
				Real dbij_dqk,dbik_dqj,dbjk_dqi;
				dbij_dqk=dB_dq[k](i,j);
				dbik_dqj=dB_dq[j](i,k);
				dbjk_dqi=dB_dq[i](j,k);
				C(i,j) += (dbij_dqk + dbik_dqj - dbjk_dqi)*dq(k);
			}
			C(i,j)*=Half;
		}
	}
}

void DynamicChain2D::GetCoriolisForces(Vector& Cdq)
{
	//LOG4CXX_INFO(KrisLibrary::logger(),"q: "<<q);
	//LOG4CXX_INFO(KrisLibrary::logger(),"dq: "<<dq);
	std::vector<Matrix> dBdi(q.n);
	for(int z=0;z<q.n;z++)
		GetKineticEnergyMatrixDeriv(z,dBdi[z]);
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
	//LOG4CXX_INFO(KrisLibrary::logger(),"Done");
}

void DynamicChain2D::GetForceVector(Real torque, const Vector2& force, int i, Vector& F) const
{
	F.resize(q.n,Zero);
	AddForceVector(torque,force,i,F);
}

void DynamicChain2D::AddForceVector(Real torque, const Vector2& force, int i, Vector& F) const
{
	//multiply by full jacobian of link i transpose
	int j=i;
	while(j!=-1) {
		F(j)+=JO(i,j)*torque + dot(JP(i,j),force);
		j=parents[j];
	}
	/* non-sparse version
	Matrix Ji;
	GetFullJacobian(links[i].com,i,Ji);
	for(int i=0;i<q.n;i++) {
		F(i) += Ji(0,i)*torque + Ji(1,i)*force.x + Ji(2,i)*force.y;
	}*/
}

void DynamicChain2D::GetGravityVector(const Vector2& g0, Vector& G) const
{
	G.resize(q.n,Zero);
	for(int i=0;i<q.n;i++) {
		AddForceVector(0,-links[i].mass*g0,i,G);
	}
}

Real DynamicChain2D::GetGravityPotentialEnergy(const Vector2& g0)
{
	Real val=Zero;
	for(unsigned int i=0;i<links.size();i++) {
		Vector2 comWorld;
		links[i].GetWorldCOM(comWorld);
		val += dot(g0,comWorld)*links[i].mass;
	}
	return val;
}

//B*ddq + C*dq = fext
void DynamicChain2D::GetAcceleration(Vector& ddq, const Vector& fext)
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


