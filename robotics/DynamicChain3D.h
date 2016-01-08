#ifndef ROBOTICS_DYNAMIC_CHAIN3D_H
#define ROBOTICS_DYNAMIC_CHAIN3D_H

#include "KinematicChain3D.h"
#include <KrisLibrary/structs/array2d.h>

class DynamicLink3D : public KinematicLink3D
{
public:
	void GetWorldInertia(Matrix3& inertiaWorld) const;
	void GetWorldCOM(Vector3& comWorld) const { T_World.mulPoint(com,comWorld); }

	//in local frame
	Real mass;
	Matrix3 inertia;
	Vector3 com;
};

class DynamicChain3D : public KinematicChain3DTemplate<DynamicLink3D>
{
public:
	void Initialize(int numBodies);
	void UpdateDynamics();			//this updates frames as well as jacobians and other temporary storage
	//temp storage updates -- full Jacobian must be updated before using the computation methods
	void Update_J();
	void Update_dB_dq();

	//Really the i,jth component of Hessian,
	//the name "JacobianDeriv" is stuck historically
	bool GetJacobianDeriv(const Vector3& pm, int m, int i, int j, Vector3& ddtheta,Vector3& ddp) const;
	//assumes j<=i<=m
	void GetJacobianDeriv_Fast(const Vector3& pm, int m, int i, int j, Vector3& ddtheta,Vector3& ddp) const;
	void GetHessian(const Vector3& pm, int m, Matrix* Htheta[3], Matrix* Hp[3]) const;
	void GetDirectionalHessian(const Vector3& pm, int m, const Vector3& v, Matrix& Hpv) const;
	bool GetJacobianDt(const Vector3& pi, int i, int j, Vector3&dtheta_dt,Vector3& dp_dt) const;
	void GetWorldAcceleration(const Vector3& pi, int i, const Vector& ddq, Vector3& dw,Vector3& dv) const;
	void GetResidualAcceleration(const Vector3& pi, int i, Vector3& dw,Vector3& dv) const;
	Real GetTotalMass() const;
	Vector3 GetCOM() const;
	void GetCOMJacobian(Matrix& Jc) const;
	void GetCOMHessian(Matrix& Hx,Matrix& Hy,Matrix& Hz) const;
	Real GetKineticEnergy(int i) const;
	Real GetKineticEnergy() const;
	void GetKineticEnergyMatrix(Matrix& B) const;	//gets the derivative of the jacobian dpi/dqj wrt qk, i.e. ddpi/dqjdqk
	//dBij/dqz
	Real GetKineticEnergyDeriv(int i,int j,int z) const;
	void GetKineticEnergyMatrixDeriv(int z,Matrix& dB) const;
	//dB/dt
	void GetKineticEnergyMatrixTimeDeriv(Matrix& dB) const;
	void GetCoriolisForceMatrix(Matrix& C);
	void GetCoriolisForces(Vector& Cdq);
	//torque/force pair on body i (in world coordinates)
	void GetForceVector(const Vector3& torque, const Vector3& force, int i, Vector& F) const;
	void AddForceVector(const Vector3& torque, const Vector3& force, int i, Vector& F) const;
	void GetGravityVector(const Vector3& g0, Vector& G) const;
	Real GetGravityPotentialEnergy(const Vector3& g0);

	//B*ddq + C*dq = fext
	void GetAcceleration(Vector& ddq, const Vector& fext);

	Vector dq;

	//temp storage
	Array2D<Vector3> JP;
	Array2D<Vector3> JO;
	std::vector<Matrix> dB_dq;
};


#endif
