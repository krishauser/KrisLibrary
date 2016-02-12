#include "TorqueSolver.h"
#include "NewtonEuler.h"
#include <iostream>
#include <fstream>
using namespace Optimization;
using namespace std;

const static Real gTorqueMaxScaleMax = 1000;

int NumContactPoints(const ContactFormation& s)
{
  int n=0;
  for(size_t i=0;i<s.contacts.size();i++)
    n += (int)s.contacts[i].size();
  return n;
}

void GetWrenchMatrix(const ContactFormation& s,const Vector3& cm,SparseMatrix& A)
{
  A.resize(6,NumContactPoints(s)*3);
  A.setZero();
  int m=0;
  for(size_t i=0;i<s.contacts.size();i++) {
    const vector<ContactPoint> h=s.contacts[i];
    for(size_t j=0;j<h.size();j++,m+=3) {
      A(0,m) = A(1,m+1) = A(2,m+2) = One;
      Matrix3 tmp;
      tmp.setCrossProduct(h[j].x-cm);
      for(int p=0;p<3;p++)
	for(int q=0;q<3;q++)
	  A(3+p,m+q) = tmp(p,q);
    }
  }
}

void GetFrictionConePlanes(const ContactFormation& s,int nFrictionConeEdges,SparseMatrix& A)
{
  int nc=NumContactPoints(s);
  A.resize(nFrictionConeEdges*nc,nc*3);
  A.setZero();

  int m=0;
  int p=0;
  for(size_t i=0;i<s.contacts.size();i++) {
    const vector<ContactPoint>& h=s.contacts[i];
    for(size_t j=0;j<h.size();j++,p++) {
      const ContactPoint& pt=h[j];
      FrictionConePolygon fc;
      fc.set(nFrictionConeEdges,pt.n,pt.kFriction);
      for(int i=0;i<nFrictionConeEdges;i++,m++) {
	A(m,p*3) = -fc.planes[i].x;
	A(m,p*3+1) = -fc.planes[i].y;
	A(m,p*3+2) = -fc.planes[i].z;
      }
    }
  }
}





TorqueSolver::TorqueSolver(RobotDynamics3D& _robot,const ContactFormation& _contacts)
  :robot(_robot),contacts(_contacts),gravityVector(Zero)
{
}

void TorqueSolver::SetDynamics(const Vector& ddq)
{
  NewtonEulerSolver ne(robot);
  ne.CalcTorques(ddq,internalForces);
}

//since
//g(q) - J(q)*f = t (split into passive / active)
//LP is
//min{f} |t/tmax|, s.t.
//g(q) - J(q) = 0 (for components 0...5 + passive)
//fi in FC i
//where ti/timax = (g(q)-J(q)*f)_i/timax  (for active dofs)
//# of variables: 3*nContacts
//# of constraints: nPassive + nContacts*numFrictionConeEdges
void TorqueSolver::Init(int nFrictionConeEdges,bool weighted)
{
  active.resize(0);
  passive.resize(0);
  active.reserve(robot.q.n);
  passive.reserve(robot.q.n);
  for(int i=0;i<robot.q.n;i++) {
    if(robot.torqueMax(i)>0.0) active.push_back(i);
    else passive.push_back(i);
  }
  int nActive=(int)active.size(), nPassive=(int)passive.size();
  int nContacts=NumContactPoints(contacts);
  problem.C.resize(nActive,3*nContacts);
  problem.d.resize(nActive);
  problem.Resize(nPassive+nContacts*nFrictionConeEdges,3*nContacts);
  problem.C.setZero();
  problem.A.setZero();

  //fill in the constant parts of problem.A, problem.b

  //first nPassive+nActive*2 are variable
  int m=nPassive;

  //last constraints: contact force bounds
  //Note: Af <= 0
  if(nContacts != 0) {
    SparseMatrix FC;
    GetFrictionConePlanes(contacts,nFrictionConeEdges,FC);
    problem.A.copySubMatrix(m,0,FC);
    
    Vector pFC;
    pFC.setRef(problem.p,m);
    Assert(pFC.n == nContacts*nFrictionConeEdges);
    pFC.setZero();
  }

  weighted = false;
  if(weighted) {
    tmaxMin = Inf;
    for(int i=0;i<robot.q.n;i++)
      if(robot.torqueMax(i) > 0)
	tmaxMin = Min(tmaxMin,robot.torqueMax(i));
  }
  else
    tmaxMin = One;
}

void TorqueSolver::LimitContactForce(int i,Real maximum,const Vector3& dir)
{
  SparseVector v(problem.A.n);
  Assert(i >= 0 && i < problem.A.n);
  v(i*3) = dir.x;
  v(i*3+1) = dir.y;
  v(i*3+2) = dir.z;
  problem.AddConstraint(-Inf,v,maximum);
}

void TorqueSolver::LimitContactForceSum(const std::vector<int>& indices,Real maximum,const Vector3& dir)
{
  SparseVector v(problem.A.n);
  for(size_t j=0;j<indices.size();j++) {
    int i=indices[j];
    Assert(i >= 0 && i < problem.A.n);
    v(i*3)=dir.x;
    v(i*3+1)=dir.y;
    v(i*3+2)=dir.z;
  }
  problem.AddConstraint(-Inf,v,maximum);
}

void TorqueSolver::Clear()
{
  active.clear();
  passive.clear();
  problem.C.clear();
  problem.d.clear();
  problem.A.clear();
  problem.p.clear();
  problem.q.clear();
  problem.l.clear();
  problem.u.clear();
  t.clear();
  f.clear();
  lhs.clear();
  internalForces.clear();
  gravityVector.setZero();
}

void TorqueSolver::FillProblem()
{
  Assert(active.size()+passive.size() == robot.links.size());
  int nActive=(int)active.size(),nPassive=(int)passive.size();

  //fill out the objective matrix
  //min |t/tmax| = min |(g - J*f)/tmax| (for active joints only)
  //fill out the passive rows of A,b
  //t = g - J*f = 0

  //get the LHS of the equation
  lhs.resize(robot.q.n);
  if(!gravityVector.isZero()) robot.GetGravityTorques(gravityVector,lhs);
  else lhs.setZero();
  if(!internalForces.empty()) lhs += internalForces;

  //fills out the bounds of the first nPassive torque constraints
  for(int i=0;i<nPassive;i++) {
    problem.p(i) = problem.q(i) = lhs(passive[i]);
    if(!IsFinite(lhs(passive[i]))) {
      cout<<"Uh... lhs is not finite"<<endl;
      cout<<"robot.q: "<<robot.q<<endl;
    }
    Assert(IsFinite(lhs(passive[i])));
    Assert(problem.ConstraintType(i) == LinearProgram::Fixed);
  }
  //fill out the objective target 
  for(int i=0;i<nActive;i++) 
    problem.d(i) = lhs(active[i]);

  //fill out J
  vector<int> dofMap(robot.q.n,-1);
  for(int i=0;i<nPassive;i++) dofMap[passive[i]] = i;
  for(int i=0;i<nActive;i++) dofMap[active[i]] = i;

  int m=0;
  for(size_t i=0;i<contacts.contacts.size();i++) {
    const vector<ContactPoint>& h=contacts.contacts[i];
    for(size_t j=0;j<h.size();j++,m+=3) {
      //Matrix J; J.setRefTranspose(Jt);
      //robot.GetPositionJacobian(localPos,h.link,J);
      int k = contacts.links[i];
      int t = (contacts.targets.empty() ?  -1 : contacts.targets[i]);
      Assert(k >= 0 && k < robot.links.size());
      Vector3 p = h[j].x;
      //if it's a self-contact transform to world
      if(t >= 0)
	p = robot.links[t].T_World*p;
      Vector3 v;
      while(k!=-1) {
	robot.links[k].GetPositionJacobian(robot.q[k],p,v);
	int n=dofMap[k];
	if(robot.torqueMax(k) > 0) { //fill out active objective
	  Assert(n >= 0 && n < problem.C.m);
	  problem.C(n,m) = v.x;
	  problem.C(n,m+1) = v.y;
	  problem.C(n,m+2) = v.z;
	}
	else {
	  Assert(n >= 0 && n < problem.A.m);
	  //fill out passive constraint
	  problem.A(n,m)=v.x;
	  problem.A(n,m+1)=v.y;
	  problem.A(n,m+2)=v.z;
	}
	k=robot.parents[k];
      }
      if(t >= 0) {
	k = t;
	while(k!=-1) {
	  robot.links[k].GetPositionJacobian(robot.q[k],p,v);
	  int n=dofMap[k];
	  if(robot.torqueMax(k) > 0) { //fill out active objective
	    Assert(n >= 0 && n < problem.C.m);
	    problem.C(n,m) -= v.x;
	    problem.C(n,m+1) -= v.y;
	    problem.C(n,m+2) -= v.z;
	  }
	  else {
	    Assert(n >= 0 && n < problem.A.m);
	    //fill out passive constraint
	    problem.A(n,m)-=v.x;
	    problem.A(n,m+1)-=v.y;
	    problem.A(n,m+2)-=v.z;
	  }
	  k=robot.parents[k];
	}
      }
    }
  }
  for(int i=0;i<nActive;i++) {
    Real scale = Min(gTorqueMaxScaleMax,robot.torqueMax(active[i]));
    problem.C.inplaceMulRow(i,1.0/scale);
    problem.d(i) /= scale;
  }

  if(tmaxMin != 1) {
    //scale the passive rows of A by the inverse of tmaxmin
    for(int i=0;i<nPassive;i++)
      problem.A.inplaceMulRow(i,1.0/tmaxMin);
    //scale p,q similarly
    Vector temp;
    temp.setRef(problem.p,0,1,nPassive);
    temp.inplaceDiv(tmaxMin);
    temp.setRef(problem.q,0,1,nPassive);
    temp.inplaceDiv(tmaxMin);
  }

  //problem.Print(cout);
}

bool TorqueSolver::Solve()
{
  //default initialization
  if(active.empty() && passive.empty()) Init();
  FillProblem();
  for(size_t i=0;i<problem.C.rows.size();i++)
    Assert(problem.C.rows[i].n == problem.C.n);
  problem.Assemble();
  LinearProgram::Result res=problem.Solve(f);
  switch(res) {
  case LinearProgram::Infeasible:
    cout<<"TorqueSolve: the problem is infeasible!"<<endl;
    //problem.Print();
    return false;
  case LinearProgram::Unbounded:
    cout<<"TorqueSolve: the problem is unbounded?!?!?"<<endl;
    {
      cout<<"Writing to temp_lp.txt"<<endl;
      ofstream out("temp_lp.txt");
      problem.lp.Print(out);
      getchar();
    }
    Abort();
    return false;
  case LinearProgram::Error:
    cout<<"TorqueSolve: faced some numerical error..."<<endl;
    return false;
  case LinearProgram::Feasible:
    {
      Vector temp;
      problem.C.mul(f,temp);
      temp -= problem.d;
      temp.inplaceNegative();
      //temp = t/tmax

      t.resize(robot.q.n);
      for(size_t i=0;i<passive.size();i++) {
	t(passive[i])=0;
      }
      for(size_t i=0;i<active.size();i++) {
	Real scale = Min(gTorqueMaxScaleMax,robot.torqueMax(active[i]));
	t(active[i])=temp(i)*scale;
      }
    }
    return true;
  default:
    cout<<"Shouldn't get here"<<endl;
    Abort();
    return false;
  }
}

bool TorqueSolver::InTorqueBounds()
{
  if(active.empty() && passive.empty()) Init();
  FillProblem();
  problem.Assemble();
  LinearProgram::Result res=problem.Solve(f);
  //TODO: a break in the objective?
  switch(res) {
  case LinearProgram::Infeasible:
    cout<<"TorqueSolve: the LP is infeasible!"<<endl;
    //problem.Print();
    return false;
  case LinearProgram::Unbounded:
    cout<<"TorqueSolve: the LP is unbounded?!?!?"<<endl;
    {
      //problem.Print(cout);
      //cout<<NumContactPoints(contacts)<<" contacts"<<endl;
      //getchar();
      cout<<"Writing to temp_lp.txt"<<endl;
      ofstream out("temp_lp.txt");
      problem.lp.Print(out);
    }
    //Abort();
    return false;
  case LinearProgram::Error:
    cout<<"TorqueSolve: faced some numerical error..."<<endl;
    return false;
  case LinearProgram::Feasible:
    {
      Vector temp;
      problem.C.mul(f,temp);
      temp -= problem.d;
      temp.inplaceNegative();
      //temp = t/tmax = saturation

      t.resize(robot.q.n);
      for(size_t i=0;i<passive.size();i++) {
	t(passive[i])=0;
      }
      for(size_t i=0;i<active.size();i++) {
	if(IsFinite(robot.torqueMax(active[i])))
	  t(active[i])=temp(i)*robot.torqueMax(active[i]);
      }
      //if(temp.maxAbsElement() > 1)
	//cout<<"Active torque saturations: "<<temp<<endl;
      return (temp.maxAbsElement() <= 1);
    }
  default:
    cout<<"Shouldn't get here"<<endl;
    Abort();
    return false;
  }
}










