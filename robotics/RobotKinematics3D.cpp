#include <KrisLibrary/Logger.h>
#include "RobotKinematics3D.h"
#include "Rotation.h"
#include "Inertia.h"
#include <math/angle.h>
#include <list>
#include <queue>
using namespace std;

void RobotKinematics3D::Initialize(int numLinks)
{
  links.resize(numLinks);
  parents.resize(numLinks);
  q.resize(numLinks,Zero);
  qMin.resize(numLinks,-Inf);
  qMax.resize(numLinks,Inf);
}

void RobotKinematics3D::InitializeRigidObject()
{
  links.resize(6);
  parents.resize(6);
  q.resize(6,Zero);
  qMin.resize(6,-Inf);
  qMax.resize(6,Inf);
  links[0].SetTranslationJoint(Vector3(1,0,0));
  links[1].SetTranslationJoint(Vector3(0,1,0));
  links[2].SetTranslationJoint(Vector3(0,0,1));
  links[3].SetRotationJoint(Vector3(0,0,1));  //yaw
  links[4].SetRotationJoint(Vector3(0,1,0));  //roll
  links[5].SetRotationJoint(Vector3(1,0,0));  //pitch
  for(int i=0;i<6;i++) {
    links[i].T0_Parent.setIdentity();
    links[i].com.setZero();
    links[i].mass = Zero;
    links[i].inertia.setZero();
    parents[i] = i-1;
  }
}

void RobotKinematics3D::Merge(const vector<RobotKinematics3D*>& robots)
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
    }
  }
}


void RobotKinematics3D::Subset(const RobotKinematics3D& robot,const vector<int>& subset)
{
  Initialize(subset.size());
  vector<int> linkMap(robot.links.size(),-1);
  for(size_t i=0;i<subset.size();i++)
    linkMap[subset[i]]=(int)i;
  for(size_t i=0;i<subset.size();i++) {
    links[i] = robot.links[subset[i]];
    int j=robot.parents[subset[i]];
    while(j != -1) {
      if(linkMap[j] >= 0) break;  //maps to a selected link
      j = robot.parents[j];
    }
    //collapse the chain from linkMap[j] to sortedLinks[i]
    if(j >= 0)
      parents[i] = linkMap[j];
    else
      parents[i] = -1;
    q(i) = robot.q(subset[i]);
    qMin(i) = robot.qMin(subset[i]);
    qMax(i) = robot.qMax(subset[i]);
  }
  //compute parent transformations by backtracking from nodes
  for(size_t i=0;i<subset.size();i++) {
    int j = robot.parents[subset[i]];
    while(j != -1) {
      if(linkMap[j] >= 0) break;  //maps to a selected link
      RigidTransform Tj;
      robot.links[j].GetLocalTransform(robot.q(j),Tj);
      //forward transform
      links[i].T0_Parent = robot.links[j].T0_Parent * Tj * links[i].T0_Parent;
      j = robot.parents[j];
    }
  }
  //modify mass properties by propagating up from leaves
  vector<vector<int> > children;
  robot.GetChildList(children);
  for(size_t i=0;i<subset.size();i++) {
    //do we need to update?  any child not included in selected subset
    bool needUpdate = false;
    for(size_t j=0;j<children[subset[i]].size();j++)
      if(linkMap[children[subset[i]][j]] < 0) {
	needUpdate = true;
	break;
      }
    if(!needUpdate) continue;

    //mass properties in world coordinates
    Vector3 com(Zero);
    Real mass(Zero);
    Matrix3 inertia(Zero);
    list<int> accumLinks;
    queue<int> q;
    //compute mass and com
    q.push(subset[i]);
    while(!q.empty()) {
      int link = q.front(); q.pop();
      if(linkMap[link] >= 0) continue;  //it's in the remaining robot, break
      accumLinks.push_back(link);
      //descend to children
      for(size_t j=0;j<children[link].size();j++)
	q.push(children[link][j]);
    }
    for(list<int>::const_iterator j=accumLinks.begin();j!=accumLinks.end();j++) {
      mass += robot.links[*j].mass;
      com.madd(robot.links[*j].T_World*robot.links[*j].com,robot.links[*j].mass);
    }
    com /= mass;
    for(list<int>::const_iterator j=accumLinks.begin();j!=accumLinks.end();j++) {
      Matrix3 I;
      InertiaMatrixAboutPoint(com,robot.links[*j].mass,robot.links[*j].T_World.R*robot.links[*j].inertia,com,I);
      inertia += I;
    }
    links[i].mass = mass;
    robot.links[subset[i]].T_World.mulInverse(com,links[i].com);
    links[i].inertia.mulTransposeA(robot.links[subset[i]].T_World.R,inertia);
  }
}

string RobotKinematics3D::LinkName(int i) const
{
  char temp[20];
  sprintf(temp,"Link[%d]",i);
  return temp;
}

void RobotKinematics3D::UpdateConfig(const Config& q_new)
{
  Assert(q_new.n == q.n);
  q.copy(q_new);
  UpdateFrames();
}

void RobotKinematics3D::UpdateFrames()
{
  //based on the values in q, update the frames T
  //Assert(HasValidOrdering());
  Frame3D Ti;
  //get local transform T(i->i)(q)
  //then do the parent transformations to get them to T(i->0)(q)
  //since the chain is top down, we can loop straight through
  //given parent j and T(j->0)(q), we have
  //T(i->0)(q) = T(j->0)(q)*T0(i->j)*T(i->i)(q)
  for(size_t i=0;i<links.size();i++) {
    RobotLink3D& li = links[i];
    li.GetLocalTransform(q(i),Ti);
    int pi=parents[i];
    if(pi==-1)
      li.T_World.mul(li.T0_Parent,Ti);
    else {
      li.T_World.mul(links[pi].T_World,li.T0_Parent);
      li.T_World*=Ti;
    }
  }
}

void RobotKinematics3D::UpdateSelectedFrames(int link,int base)
{
  Frame3D Ti;
  vector<int> updlinks;
  while(link != base) {
    updlinks.push_back(link);
    link = parents[link];
  }
  if(base != -1)
    updlinks.push_back(base);
  reverse(updlinks.begin(),updlinks.end());
  for(size_t k=0;k<updlinks.size();k++) {
    int i = updlinks[k];
    RobotLink3D& li = links[i];
    li.GetLocalTransform(q(i),Ti);
    int pi=parents[i];
    if(pi==-1)
      li.T_World.mul(li.T0_Parent,Ti);
    else {
      li.T_World.mul(links[pi].T_World,li.T0_Parent);
      li.T_World*=Ti;
    }
  }
}

bool RobotKinematics3D::InJointLimits(const Config& q) const
{
  Assert(q.n == (int)links.size());
  for(int i=0;i<q.n;i++) {
    if(qMin[i]>q[i] || q[i]>qMax[i]) {
      return false;
    }
  }
  return true;
}

void RobotKinematics3D::NormalizeAngles(Config& q) const
{
  for(size_t i=0;i<links.size();i++) {
    if(links[i].type == RobotLink3D::Revolute) {
      if( qMin(i) <= q(i) && q(i) <= qMax(i) ) continue;
      Real qi = AngleNormalize(q(i));
      if(qi > qMax(i)) {
	if(qi - TwoPi >= qMin(i)) 
	  qi -= TwoPi;
	else if(Abs(qi - TwoPi - qMin(i)) < Abs(qi - qMax(i)))
	  qi -= TwoPi;
      }
      if(qi < qMin(i)) {
	if(qi + TwoPi <= qMax(i))
	  qi += TwoPi;
	else if(Abs(qi + TwoPi - qMax(i)) < Abs(qi - qMin(i)))
	  qi += TwoPi;
      }
      q(i) = qi;
    }
  }
}

void RobotKinematics3D::GetWorldPosition(const Vector3& pi, int i, Vector3& p) const
{
  links[i].T_World.mulPoint(pi,p);
}


const Matrix3& RobotKinematics3D::GetWorldRotation(int i) const
{
  return links[i].T_World.R;
}


void RobotKinematics3D::GetWorldRotation_Moment(int i, Vector3& theta) const
{
  MomentRotation rot;
  rot.setMatrix(links[i].T_World.R);
  theta=rot;
}


void RobotKinematics3D::GetWorldVelocity(const Vector3& pi, int i, const Vector& dq, Vector3& dp) const
{
  Vector3 tempp;
  dp.setZero();
  Vector3 p;
  links[i].T_World.mulPoint(pi,p);
  int j=i;
  while(j!=-1) {
    links[j].GetVelocity(q[j],dq[j],p,tempp);
    dp+=tempp;
    j=parents[j];
  }
}


void RobotKinematics3D::GetWorldAngularVelocity(int i, const Vector& dq, Vector3& omega) const
{
  Vector3 tempw;
  omega.setZero();
  int j=i;
  while(j!=-1) {
    links[j].GetAngularVelocity(dq[j],tempw);
    omega+=tempw;
    j=parents[j];
  }
}


bool RobotKinematics3D::GetWorldRotationDeriv(int i, int j, Matrix3& dR) const
{
  if(IsAncestor(i,j)) {
    Vector3 dw;
    links[j].GetOrientationJacobian(dw);
    MatrixDerivative(links[i].T_World.R,dw,dR);
    return true;
  }
  dR.setZero();
  return false;
}


bool RobotKinematics3D::GetWorldRotationDeriv_Moment(int i, int j, Vector3& dm) const
{
  if(IsAncestor(i,j)) {
    Vector3 dw;
    links[j].GetOrientationJacobian(dw);
    MomentDerivative(links[i].T_World.R,dw,dm);
    return true;
  }
  dm.setZero();
  return false;
}


bool RobotKinematics3D::GetWorldRotationDeriv_Moment(int i, int j, const Vector3& m,Vector3& dm) const
{
  if(IsAncestor(i,j)) {
    Vector3 dw;
    links[j].GetOrientationJacobian(dw);
    MomentDerivative(m,links[i].T_World.R,dw,dm);
    return true;
  }
  dm.setZero();
  return false;
}


bool RobotKinematics3D::GetJacobian(const Vector3& pi, int i, int j, Vector3& dw, Vector3& dv) const
{
  if(IsAncestor(i,j)) {
    Vector3 p;
    GetWorldPosition(pi,i,p);
    links[j].GetJacobian(q[j],p,dw,dv);
    return true;
  }
  dw.setZero();
  dv.setZero();
  return false;
}


bool RobotKinematics3D::GetOrientationJacobian(int i, int j, Vector3& dw) const
{
  if(IsAncestor(i,j)) {
    links[j].GetOrientationJacobian(dw);
    return true;
  }
  dw.setZero();
  return false;
}


bool RobotKinematics3D::GetPositionJacobian(const Vector3& pi, int i, int j, Vector3& dv) const
{
  if(IsAncestor(i,j)) {
    Vector3 p;
    GetWorldPosition(pi,i,p);
    links[j].GetPositionJacobian(q[j],p,dv);
    return true;
  }
  dv.setZero();
  return false;
}


void RobotKinematics3D::GetFullJacobian(const Vector3& pi, int i, Matrix& J) const
{
  J.resize(6,q.n,Zero);
  Vector3 w; Vector3 v;
  Vector3 p;
  GetWorldPosition(pi,i,p);
  int j=i;
  while(j!=-1) {
    links[j].GetJacobian(q[j],p,w,v);
    J(0,j)=w.x; J(1,j)=w.y; J(2,j)=w.z;
    J(3,j)=v.x; J(4,j)=v.y; J(5,j)=v.z;
    j=parents[j];
  }
}


void RobotKinematics3D::GetPositionJacobian(const Vector3& pi, int i, Matrix& J) const
{
  J.resize(3,q.n,Zero);
  Vector3 v;
  Vector3 p;
  GetWorldPosition(pi,i,p);
  int j=i;
  while(j!=-1) {
    links[j].GetPositionJacobian(q[j],p,v);
    J(0,j)=v.x; J(1,j)=v.y; J(2,j)=v.z;
    j=parents[j];
  }
}

void RobotKinematics3D::GetWrenchTorques(const Vector3& torque, const Vector3& force, int i, Vector& F) const
{
  F.resize(q.n,Zero);
  AddWrenchTorques(torque,force,i,F);
}

void RobotKinematics3D::AddWrenchTorques(const Vector3& torque, const Vector3& force, int i, Vector& F) const
{
  Vector3 p = links[i].T_World.t;
  //multiply by full jacobian of link i transpose
  Vector3 dw,dv;
  int j=i;
  while(j!=-1) {
    links[j].GetJacobian(q(j),p,dw,dv);
    F(j)+=dot(dw,torque) + dot(dv,force);
    j=parents[j];
  }
}

void RobotKinematics3D::GetForceTorques(const Vector3& f, const Vector3& pi, int i, Vector& F) const
{
  F.resize(q.n,Zero);
  AddForceTorques(f,pi,i,F);
}

void RobotKinematics3D::AddForceTorques(const Vector3& f, const Vector3& pi, int i, Vector& F) const
{
  Vector3 p;
  links[i].T_World.mulPoint(pi,p);
  //multiply by position jacobian of link i transpose
  Vector3 dv;
  int j=i;
  while(j!=-1) {
    links[j].GetPositionJacobian(q(j),p,dv);
    F(j)+= dot(dv,f);
    j=parents[j];
  }
}





Real RobotKinematics3D::GetTotalMass() const
{
	Real totalMass=Zero;
	for(size_t i=0; i<links.size(); i++) totalMass += links[i].mass;
	return totalMass;
}

Vector3 RobotKinematics3D::GetCOM() const
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

Matrix3 RobotKinematics3D::GetTotalInertia() const
{
  Matrix3 I,temp,ccross;
  I.setZero();
  Vector3 cm = GetCOM(),ci;
  for(size_t i=0;i<links.size();i++) {
    links[i].GetWorldInertia(temp);
    I += temp;
    //get the inertia matrix associated with the center of mass
    links[i].T_World.mul(links[i].com,ci);
    ci -= cm;
    Real x,y,z;
    ci.get(x,y,z);
    ccross(0,0)=y*y+z*z; ccross(0,1)=-x*y; ccross(0,2)=-x*z;
    ccross(1,0)=-x*y; ccross(1,1)=x*x+z*z; ccross(1,2)=-y*z;
    ccross(2,0)=-x*z; ccross(2,1)=-y*z; ccross(2,2)=x*x+y*y;
    ccross.inplaceMul(links[i].mass);
    I += ccross;
  }
  return I;
}

void RobotKinematics3D::GetCOMJacobian(Matrix& J) const
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

void RobotKinematics3D::GetCOMHessian(Matrix& Hx,Matrix& Hy,Matrix& Hz) const
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
  Matrix* Hp[3]={&Htempx,&Htempy,&Htempz};
  for(int k=0;k<q.n;k++) {
    GetPositionHessian(links[k].com,k,Hp);
    Hx.madd(Htempx,links[k].mass);
    Hy.madd(Htempy,links[k].mass);
    Hz.madd(Htempz,links[k].mass);
  }
  Hx /= mtotal;
  Hy /= mtotal;
  Hz /= mtotal;
}

void RobotKinematics3D::GetGravityTorques(const Vector3& g0, Vector& G) const
{
  G.resize(q.n,Zero);
  for(int i=0;i<q.n;i++) {
    AddForceTorques(-links[i].mass*g0,links[i].com,i,G);
  }
}


Real RobotKinematics3D::GetGravityPotentialEnergy(const Vector3& g0,Real refHeight) const
{
  Real g = g0.norm();
  if(Abs(g) == 0) return 0;
  Vector3 up = g0;
  up /= -g;

  Real val=Zero;
  for(size_t i=0;i<links.size();i++) {
    Vector3 comWorld;
    links[i].GetWorldCOM(comWorld);
    val += (dot(up,comWorld)-refHeight)*links[i].mass;
  }
  return val*g;
}





//gets the derivative of the jacobian dpm/dqi wrt qj, i.e. d/dqj(dpm/dqi)
bool RobotKinematics3D::GetJacobianDeriv(const Vector3& pm, int m, int i, int j, Vector3&ddtheta,Vector3& ddp) const
{
	if(!IsAncestor(m,i)) return false;
	if(!IsAncestor(m,j)) return false;
	GetJacobianDeriv_Fast(pm,m,i,j,ddtheta,ddp);
	return true;
}

void RobotKinematics3D::GetJacobianDeriv_Fast(const Vector3& pm, int m, int i, int j, Vector3&ddtheta,Vector3& ddp) const
{
  Assert(i <= m);
  Assert(j <= m);
  bool swapped = false;
  if(i < j) {  //order so j < i, 
    Swap(i,j);
    swapped = true;
  }
  //Get i's Jacobian then transform to j, and modify
  Frame3D JPmi,JPjj;
  Matrix3 R0_j;
  links[i].GetJacobian(q[i],links[m].T_World,JPmi);
  links[j].GetJacobian(q[j],links[j].T_World,JPjj);
  R0_j.setTranspose(links[j].T_World.R);
  
  Matrix3 RA1B = JPjj.R*R0_j;
  ddp = RA1B*(JPmi*pm);
  if(swapped) ddtheta.setZero();
  else ddtheta=RA1B*links[i].T_World.R*links[i].w;
}

void RobotKinematics3D::GetJacobianDeriv(const Vector3& pm, int m, Matrix* Htheta[3], Matrix* Hp[3]) const
{
  for(int z=0;z<3;z++) {
    if(Htheta[z]) Htheta[z]->resize(q.n,q.n,Zero);
    if(Hp[z]) Hp[z]->resize(q.n,q.n,Zero);
  }
  Vector3 ddtheta,ddp;
  //NOTE: H(i,j) != 0 only if i,j are ancestors of m
  //Also Hp(i,j) is symmetric, only need to consider j as an ancestor of i
  //(j<i)
  //Ho(i,j) is zero if j > 0.
  for(int i=m;i!=-1;i=parents[i]) {
    for(int j=i;j!=-1;j=parents[j]) {
      GetJacobianDeriv_Fast(pm,m,i,j,ddtheta,ddp);
      for(int z=0;z<3;z++) {
	if(Htheta[z]) {
	  (*Htheta[z])(i,j) = ddtheta[z];
	  (*Htheta[z])(j,i) = 0;
	}
	if(Hp[z]) {
	  (*Hp[z])(i,j) = (*Hp[z])(j,i) = ddp[z];
	}
      }
    }
  }
  /*
  for(int i=0;i<q.n;i++) {
    for(int j=0;j<q.n;j++) {
      if(GetJacobianDeriv(pm,m,i,j,ddtheta,ddp)) {
        for(int z=0;z<3;z++) {
	  (*Htheta[z])(i,j)=ddtheta[z];
	  (*Hp[z])(i,j)=ddp[z];
        }
      }
      else {
        for(int z=0;z<3;z++) {
	  (*Htheta[z])(i,j)=Zero;
	  (*Hp[z])(i,j)=Zero;
        }
      }
    }
  }
  */
}

void RobotKinematics3D::GetPositionHessian(const Vector3& pm, int m, Matrix* Hp[3]) const
{
  for(int z=0;z<3;z++) {
    if(Hp[z]) Hp[z]->resize(q.n,q.n,Zero);
  }
  Vector3 ddtheta,ddp;
  //NOTE: H(i,j) != 0 only if i,j are ancestors of m
  //Also H(i,j) is symmetric, only need to consider j as an ancestor of i (j<i)
  for(int i=m;i!=-1;i=parents[i]) {
    for(int j=i;j!=-1;j=parents[j]) {
      GetJacobianDeriv_Fast(pm,m,i,j,ddtheta,ddp);
      for(int z=0;z<3;z++) {
	if(Hp[z]) (*Hp[z])(i,j) = (*Hp[z])(j,i) = ddp[z];
      }
    }
  }
}

void RobotKinematics3D::GetDirectionalHessian(const Vector3& pm, int m, const Vector3& v, Matrix& Hv) const
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



Real RobotKinematics3D::PointDistanceBound(const Vector3& pi,int i,const Config& q1,const Config& q2) const
{
  Real sumdist = 0;
  int n=i;

  Real distanceToPoint=pi.norm();
  while(n != -1) {
    if(links[n].type == RobotLink3D::Prismatic)
      sumdist += Abs(q1(n)-q2(n));
    else
      sumdist += distanceToPoint*Abs(q1(n)-q2(n));

    distanceToPoint += links[n].T0_Parent.t.norm();
    n = parents[n];
  }
  return sumdist;
}

Real RobotKinematics3D::PointDistanceBound2(const Vector3& pi,int i,const Config& q1,const Config& q2) 
{
  UpdateConfig(q1);
  Vector3 pw = links[i].T_World*pi;

  Real sumdist = 0;
  int n=i;

  while(n != -1) {
    if(links[n].type == RobotLink3D::Prismatic)
      sumdist += Abs(q1(n)-q2(n));
    else {
      Real distanceToPoint = cross(pw-links[n].T_World.t,links[n].T_World.R*links[n].w).norm();
      sumdist += Abs(q1(n)-q2(n))*distanceToPoint;
    }
    n = parents[n];
  }
  return sumdist;
}

Real RobotKinematics3D::SphereDistanceBound(const Vector3& ci,Real r,int i,const Config& q1,const Config& q2)
{
  UpdateConfig(q1);
  Vector3 cw = links[i].T_World*ci;

  Real sumdist = 0;
  int n=i;

  while(n != -1) {
    if(links[n].type == RobotLink3D::Prismatic)
      sumdist += Abs(q1(n)-q2(n));
    else {
      Real distanceToPoint = cross(cw-links[n].T_World.t,links[n].T_World.R*links[n].w).norm()+r;
      sumdist += Abs(q1(n)-q2(n))*distanceToPoint;
    }
    n = parents[n];
  }
  return sumdist;

}


