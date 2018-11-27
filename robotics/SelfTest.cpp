#include <KrisLibrary/Logger.h>
#include <math/random.h>
#include <math/VectorPrinter.h>
#include <math3d/random.h>
#include "Rotation.h"
#include "NewtonEuler.h"
#include "RLG.h"
#include <errors.h>
#include "SelfTest.h"
using namespace Math;
using namespace std;


//test the rotation angle differentiation
//the matrix R rotating about the angle z
struct RotationAngleFunction : public RealFunction
{
  virtual void PreEval(Real x) {
    AngleAxisRotation aa;
    aa.angle = x;
    aa.axis = z;
    aa.getMatrix(temp);
    temp = temp*R;
  }
  virtual Real Eval(Real x) {
    AngleAxisRotation aa;
    aa.setMatrix(temp);
    return aa.angle;
  }
  virtual Real Deriv(Real x) {
    return MatrixAngleDerivative(temp,z);
  }
  Matrix3 R;
  Vector3 z;
  Matrix3 temp;
};


//test the MomentRotation differentiation 
//as a function of a rotation t about the axis a
struct RotationAxisFunction : public VectorFunction
{
  virtual int NumDimensions() const { return 3; }
  virtual void Eval(Real t,Vector& v)
  {
    Matrix3 Ra,ARaB;
    AngleAxisRotation aa;
    aa.angle=t;
    aa.axis=a;
    aa.getMatrix(Ra);
    ARaB = A*Ra*B;
    MomentRotation m;
    m.setMatrix(ARaB);

    v.resize(3);
    m.get(&v(0));
  }
  virtual void Deriv(Real t,Vector& dv) {
    Matrix3 Ra,ARaB;
    AngleAxisRotation aa;
    aa.angle=t;
    aa.axis=a;
    aa.getMatrix(Ra);
    ARaB = A*Ra*B;

    Vector3 z,dm;
    A.mul(a,z);
    MomentDerivative(ARaB,z,dm);

    /*
    MomentRotation m;
    m.setMatrix(ARaB);
    MomentDerivative(m,z,dm);
    */
    dv.resize(3);
    dm.get(&dv(0));
  }

  Matrix3 A;
  Vector3 a;
  Matrix3 B;
};

void TestRotationDiff()
{
  LOG4CXX_INFO(KrisLibrary::logger(),"Testing rotation angle differentiation...");
  RotationAngleFunction rf;
  QuaternionRotation r;
  int numTests=100;
  for(int i=0;i<numTests;i++) {
    SampleSphere(One,rf.z);
    RandRotation(r);
    r.getMatrix(rf.R);
    Real t=0;
    LOG4CXX_INFO(KrisLibrary::logger(),"Initial matrix angle: "<<rf(t));
    AngleAxisRotation aa; r.getAngleAxis(aa);
    LOG4CXX_INFO(KrisLibrary::logger(),"Difference between matrix angle and rotation angle: "<<Acos(Clamp(rf.z.dot(aa.axis),-1,1)));
    if(!TestDeriv(&rf,t,0.001,0.001,1e-2)) {
    }
  }

  LOG4CXX_INFO(KrisLibrary::logger(),"Testing moment rotation differentiation...");
  RotationAxisFunction f;
  for(int i=0;i<numTests;i++) {
    SampleSphere(One,f.a);
    RandRotation(r);
    r.getMatrix(f.A);
    RandRotation(r);
    r.getMatrix(f.B);
    Real t=Rand(-3,3);
    if(!TestDeriv(&f,t,0.001,0.001,1e-2)) {
    }
  }
}

void TestRotations()
{
  TestRotationDiff();

  LOG4CXX_INFO(KrisLibrary::logger(),"Testing moment rotations...");
  Real tol=1e-3;
  Matrix3 Rm,Ra;
  MomentRotation m,m2;
  AngleAxisRotation a,a2;
  SampleCube(2,m);
  m.getMatrix(Rm);
  m2.setMatrix(Rm);
  Assert(m.isEqual(m2,tol));
  Rm.mul(m,m2);
  Assert(m.isEqual(m2,tol));
  a.setMoment(m);
  a.getMatrix(Ra);
  Assert(Ra.isEqual(Rm,tol));
  a2.setMatrix(Ra);
  Assert(FuzzyEquals(a.angle,a2.angle,tol));
  Assert(a.axis.isEqual(a2.axis,tol));

  //identity R[x] = [Rx]R for orthogonal matrix R
  m.getMatrix(Rm);
  SampleCube(2,m2);
  Ra.setCrossProduct(m2);
  Matrix3 Rx,RxR;
  Rx.mul(Rm,Ra);
  Ra.setCrossProduct(Rm*m2);
  RxR.mul(Ra,Rm);
  Assert(Rx.isEqual(RxR,tol));

  //singularities
  LOG4CXX_INFO(KrisLibrary::logger(),"Testing singularities");
  SampleSphere(Pi,m);
  Assert(FuzzyEquals(m.norm(),Pi,tol));
  m.getMatrix(Rm);
  m2.setMatrix(Rm);
  if(!FuzzyEquals(Abs(m.dot(m2)),Pi*Pi,Sqrt(tol))) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in singularity!");
    LOG4CXX_INFO(KrisLibrary::logger(),"Rotation "<<m<<" => "<<m2);
    LOG4CXX_INFO(KrisLibrary::logger(),"Matrix "<<Rm<<"\n");
    abort();
  }

  //euler angles
  LOG4CXX_INFO(KrisLibrary::logger(),"Testing Euler angles");
  EulerAngleRotation e,e2;
  SampleCube(4,e);
  e.getMatrixXYZ(Rm);
  e2.setMatrixXYZ(Rm);
  e2.getMatrixXYZ(Ra);
  Assert(Rm.isEqual(Ra,tol));
  e.getMatrixZYX(Rm);
  e2.setMatrixZYX(Rm);
  e2.getMatrixZYX(Ra);
  Assert(Rm.isEqual(Ra,tol));
}




void MakePlanarChain(RobotKinematics3D& robot,int n,Real d=One)
{
  robot.Initialize(n);
  for(int i=0;i<n;i++) {
    robot.parents[i]=i-1;
    robot.links[i].T0_Parent.setIdentity();
    if(i > 0) robot.links[i].T0_Parent.t.x=d;
    //if(RandBool())
    if(i%2 == 0)
      robot.links[i].SetRotationJoint(Vector3(0,0,1));
    else
      robot.links[i].SetRotationJoint(Vector3(0,1,0));
    robot.links[i].mass=One;
    robot.links[i].com.set(Half*d,0,0);
    robot.links[i].inertia.setZero();
    //for rod of negligible thickness
    robot.links[i].inertia(1,1)=robot.links[i].inertia(2,2)=1.0/12.0*Sqr(d);
    robot.qMin(i)=-Pi*0.75;
    robot.qMax(i)=Pi*0.75;
  }
}

ostream& operator << (ostream& out,const WorkspaceBound& b);


void TestRLG() {
  int n=3;
  //create random robot
  RobotKinematics3D chain;
  MakePlanarChain(chain,n);
  chain.UpdateFrames();
  IKGoal ikgoal;
  ikgoal.link=n-1;
  ikgoal.SetFixedPosition(Vector3(2,1,0));
  ikgoal.localPosition.set(1,0,0);
  JointStructure jschain(chain);
  jschain.Init();
  jschain.SolveRootBounds();
  LOG4CXX_INFO(KrisLibrary::logger(),"Initial bounds:");
  for(int i=0;i<n;i++) {
    const WorkspaceBound& b=jschain.bounds[i];
    LOG4CXX_INFO(KrisLibrary::logger(),i<<" is "<<b);
  }
  KrisLibrary::loggerWait();
  jschain.IntersectWorkspaceBounds(ikgoal);
  LOG4CXX_INFO(KrisLibrary::logger(),"Bounds:");
  for(int i=0;i<n;i++) {
    const WorkspaceBound& b=jschain.bounds[i];
    LOG4CXX_INFO(KrisLibrary::logger(),i<<" is "<<b);
  }
  KrisLibrary::loggerWait();

  vector<IKGoal> ik;
  ik.push_back(ikgoal);
  RLG rlg(chain,jschain);
  for(int i=0;i<100;i++) {
    rlg.SampleFrom(0);
    //set q(n-1) to minimize the distance between tgt and end effector
    Frame3D Tn=chain.links[n-2].T_World*chain.links[n-1].T0_Parent;
    Vector3 pLocal;
    Tn.mulPointInverse(ik[0].endPosition,pLocal);
    //try to align (1,0,0) with pLocal => q(n-1) = atan2(pLocal.y,pLocal.x)
    chain.q(n-1) = Clamp(Atan2(pLocal.y,pLocal.x),chain.qMin(n-1),chain.qMax(n-1));
    Frame3D Tloc;
    chain.links[n-1].GetLocalTransform(chain.q(n-1),Tloc);
    chain.links[n-1].T_World = Tn*Tloc;

    LOG4CXX_INFO(KrisLibrary::logger(),"Q is "<<VectorPrinter(chain.q));
    if(!chain.InJointLimits(chain.q)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Chain out of joint limits!");
      abort();
    }
    for(int k=0;k<chain.q.n;k++) {
      LOG4CXX_INFO(KrisLibrary::logger(),k<<": "<<chain.links[k].T_World.t);
      Frame3D tp;
      Frame3D tloc,tk;
      if(chain.parents[k]!=-1) {
	tp=chain.links[chain.parents[k]].T_World;
	chain.links[k].GetLocalTransform(chain.q(k),tloc);
	tk = tp*chain.links[k].T0_Parent*tloc;
	Assert(tk.R.isEqual(chain.links[k].T_World.R,1e-3));
	Assert(tk.t.isEqual(chain.links[k].T_World.t,1e-3));
      }
    }
    Vector3 p;
    chain.GetWorldPosition(ikgoal.localPosition,ikgoal.link,p);
    LOG4CXX_INFO(KrisLibrary::logger(),n<<": "<<p);
    LOG4CXX_INFO(KrisLibrary::logger(),"Goal distance "<<p.distance(ikgoal.endPosition));
    KrisLibrary::loggerWait();
  }
}


void TestNewtonEuler()
{
  RobotDynamics3D robot;
  for(int n=2;n<10;n++) {
    robot.Initialize(n);
    MakePlanarChain(robot,n,One);
    Assert(robot.q.n == n);
    Assert(robot.dq.n == n);
    robot.q.setZero();
    robot.dq.setZero();
    for(int i=0;i<n;i++) {
      robot.q(i) = Rand(-1.0,1.0);
      //robot.q(i) = Pi/Pow(2.0,i+1);
    }
    //robot.q(0) = 0;
    //robot.q(1) = Pi/4.0;
    for(int i=0;i<n;i++) {
      //robot.dq(i) = Rand(-1.0,1.0);
      robot.dq(i) = Rand(-10.0,10.0);
      //robot.dq(i) = 1.0/(i+1.0);
    }
    //robot.dq(0) = 1;
    //robot.dq(1) = 1.0/3.0;
    robot.UpdateFrames();
    robot.UpdateDynamics();
    
    NewtonEulerSolver ne(robot);
    //ne.SetGravityWrenches(Vector3(0,0,-9.8));
    ne.SelfTest();
  }
}
