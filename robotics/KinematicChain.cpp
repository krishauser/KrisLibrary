#include "KinematicChain.h"

KinematicLink2D::Type KinematicLink2D::GetType() const
{
	if(w==Zero) {
		if(v==Vector2(Zero))
			return TypeNone;
		return TypeTranslation;
	}
	if(v==Vector2(Zero))
		return TypeRotation;
	return TypeCombination;
}

void KinematicLink2D::SetRotationJoint(Real _w)
{
	w=_w;
	v.setZero();
}

void KinematicLink2D::SetTranslationJoint(const Vector2& _v)
{
	w=0;
	v=_v;
}

void KinematicLink2D::GetJacobian(Real qi,const Vector2& p, Real& Jo, Vector2& Jp) const
{
	Matrix2 RLoc_World;
	Matrix2 RLoc,RLocInv;
	RLoc.setRotate(qi*w);
	RLocInv.setTranspose(RLoc);
	RLoc_World.mul(T_World.R,RLocInv);

	Vector2 pi;
	T_World.mulInverse(p,pi);

	Jp = RLoc_World*(cross(w,RLoc*pi) + v);

	//no local transform necessary for w
	Jo = w;
}

void KinematicLink2D::GetJacobian(Real qi,Frame2D& J) const
{
  //J*p = R[Loc->0]*([w]*RLoc*pi + v)
  //    = R[Loc->0]*[w]*RLoc*(R[0->i]*p-t[0->i]) + R[Loc->0]*v
  //    = R[Loc->0]*[w]*RLoc*R[0->i]*p + R[Loc->0]*(v - [w]*RLoc*t[0->i])
	Matrix2 RLoc_World;
	Matrix2 RLoc,RLocInv;
	RLoc.setRotate(qi*w);
	RLocInv.setTranspose(RLoc);
	RLoc_World.mul(T_World.R,RLocInv);

	Frame2D TWorld_i;
	TWorld_i.setInverse(T_World);

	Frame2D JLoc;
	JLoc.R = crossProductMatrix2(w)*RLoc;
	JLoc.t = v;
	Frame2D JWorld_Loc=FrameMulWB1(JLoc,TWorld_i);
	J.R.mul(RLoc_World,JWorld_Loc.R);
	RLoc_World.mul(JWorld_Loc.t,J.t);
}

void KinematicLink2D::GetJacobian(Real qi,const Frame2D& Tj_World,Frame2D& J) const
{
  Frame2D JWorld;
  GetJacobian(qi,JWorld);
  J=FrameMulWB1(JWorld,Tj_World);
}

void KinematicLink2D::GetVelocity(Real qi,Real dqi,const Vector2& p,Vector2& vel) const
{
	Real Jo; Vector2 Jp;
	GetJacobian(qi,p,Jo,Jp);
	vel=Jp*dqi;
}

void KinematicLink2D::GetAngularVelocity(Real qi,Real dqi,Real& omega) const
{
	//Jo = w;
	omega=w*dqi;
}



