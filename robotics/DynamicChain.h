#ifndef ROBOTICS_DYNAMIC_CHAIN_H
#define ROBOTICS_DYNAMIC_CHAIN_H

#include "KinematicChain.h"
#include <KrisLibrary/structs/array2d.h>

class DynamicLink2D : public KinematicLink2D
{
public:
	void GetWorldInertia(Real& inertiaWorld) const { inertiaWorld=inertia; }
	void GetWorldCOM(Vector2& comWorld) const { T_World.mulPoint(com,comWorld); }

	//in local frame
	Real mass;
	Real inertia;
	Vector2 com;
};

class DynamicChain2D : public KinematicChain2DTemplate<DynamicLink2D>
{
public:
	void Initialize(int numBodies);
	void UpdateDynamics();			//this updates frames as well as jacobians and other temporary storage
	//temp storage updates -- full Jacobian must be updated before using the computation methods
	void Update_J();
	void Update_dB_dq();

	bool GetJacobianDeriv(const Vector2& pi, int i, int j, int k, Real&ddtheta,Vector2& ddp) const;
	Vector2 GetCenterOfMass() const;
	Real GetKineticEnergy(int i) const;
	Real GetKineticEnergy() const;	void GetKineticEnergyMatrix(Matrix& B) const;	//gets the derivative of the jacobian dpi/dqj wrt qk, i.e. ddpi/dqjdqk
	//dBij/dqz
	Real GetKineticEnergyDeriv(int i,int j,int z) const;
	void GetKineticEnergyMatrixDeriv(int z,Matrix& dB) const;
	//dB/dt
	void GetKineticEnergyMatrixTimeDeriv(Matrix& dB) const;
	void GetCoriolisForceMatrix(Matrix& C);
	void GetCoriolisForces(Vector& Cdq);
	//torque/force pair on body i (in world coordinates)
	void GetForceVector(Real torque, const Vector2& force, int i, Vector& F) const;
	void AddForceVector(Real torque, const Vector2& force, int i, Vector& F) const;
	void GetGravityVector(const Vector2& g0, Vector& G) const;
	Real GetGravityPotentialEnergy(const Vector2& g0);

	//B*ddq + C*dq = fext
	void GetAcceleration(Vector& ddq, const Vector& fext);

	Vector dq;

	//temp storage
	Array2D<Vector2> JP;
	Array2D<Real> JO;
	std::vector<Matrix> dB_dq;
};


#endif
