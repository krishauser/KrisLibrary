#include <KrisLibrary/Logger.h>
#include "Stability.h"
#include <math3d/basis.h>
#include <geometry/PolytopeProjection.h>
#include <iostream>
#include <algorithm>
#include <list>
using namespace std;
using namespace Geometry;
using namespace Optimization;

//Experiments have indicated sparse LP doesn't work as good as regular LP
//for small problems... maybe a compressed-row sparse matrix would be faster?
#define USE_SPARSE_LP 0

inline int NumForceVariables(const vector<ContactPoint>& cps)
{
  return cps.size()*3;
}

inline int NumConstraints(const vector<ContactPoint>& cps,int numFCEdges)
{
  return cps.size()*numFCEdges;
}

inline Vector3 Centroid(const vector<ContactPoint>& cps)
{
  Vector3 com(Zero);
  for(size_t i=0;i<cps.size();i++) 
    com += cps[i].x;
  com /= cps.size();
  return com;
}

inline void GetForceMinimizationDirection(const vector<ContactPoint>& cps,Vector& c)
{
  for(size_t j=0;j<cps.size();j++)
    cps[j].n.get(c(j*3),c(j*3+1),c(j*3+2));
}

inline void SplitForceVector(const Vector& f,const vector<ContactPoint>& cps,vector<Vector3>& fcs)
{
  Assert(f.n == NumForceVariables(cps));
  fcs.resize(cps.size());
  for(size_t i=0;i<cps.size();i++)
    fcs[i].set(f(i*3),f(i*3+1),f(i*3+2));
}

inline int NumForceVariables(const vector<ContactPoint2D>& cps)
{
  return cps.size()*2;
}

inline int NumConstraints(const vector<ContactPoint2D>& cps)
{
  return cps.size()*2;
}

inline Vector2 Centroid(const vector<ContactPoint2D>& cps)
{
  Vector2 com(Zero);
  for(size_t i=0;i<cps.size();i++) 
    com += cps[i].x;
  com /= cps.size();
  return com;
}


inline void GetForceMinimizationDirection(const vector<ContactPoint2D>& cps,Vector& c)
{
  for(size_t j=0;j<cps.size();j++)
    cps[j].n.get(c(j*2),c(j*2+1));
}

inline void SplitForceVector(const Vector& f,const vector<ContactPoint2D>& cps,vector<Vector2>& fcs)
{
  Assert(f.n == NumForceVariables(cps));
  fcs.resize(cps.size());
  for(size_t i=0;i<cps.size();i++)
    fcs[i].set(f(i*2),f(i*2+1));
}

inline int NumForceVariables(const vector<CustomContactPoint>& cps)
{
  int n = 0;
  for(size_t i=0;i<cps.size();i++) 
    n += cps[i].numForceVariables();
  return n;
}

inline int NumConstraints(const vector<CustomContactPoint>& cps)
{
  int n = 0;
  for(size_t i=0;i<cps.size();i++) 
    n += cps[i].numConstraints();
  return n;
}

inline Vector3 Centroid(const vector<CustomContactPoint>& cps)
{
  Vector3 com(Zero);
  for(size_t i=0;i<cps.size();i++) 
    com += cps[i].x;
  com /= cps.size();
  return com;
}


inline void GetForceMinimizationDirection(const vector<CustomContactPoint>& cps,Vector& c)
{
  int m=0;
  for(size_t j=0;j<cps.size();j++) {
    if(cps[j].numForceVariables() == 1)
      c[m] = 1;
    else {
      cps[j].n.get(c(m),c(m+1),c(m+2));
      for(int p=3;p<cps[j].numForceVariables();p++)
        c[m+p] = 0.0;
    }
    m += cps[j].numForceVariables();
  }
}

inline void SplitForceVector(const Vector& f,const vector<CustomContactPoint>& cps,vector<Vector3>& fcs)
{
  Assert(f.n == NumForceVariables(cps));
  fcs.resize(cps.size());
  int m=0;
  for(size_t i=0;i<cps.size();i++) {
    if(cps[i].numForceVariables() == 1)
      fcs[i] = f(m)*cps[i].n;
    else if(cps[i].numForceVariables() >= 3)
      fcs[i].set(f(m),f(m+1),f(m+2));
    m += cps[i].numForceVariables();
  }
}

inline int NumForceVariables(const vector<CustomContactPoint2D>& cps)
{
  int n = 0;
  for(size_t i=0;i<cps.size();i++) 
    n += cps[i].numForceVariables();
  return n;
}

inline int NumConstraints(const vector<CustomContactPoint2D>& cps)
{
  int n = 0;
  for(size_t i=0;i<cps.size();i++) 
    n += cps[i].numConstraints();
  return n;
}

inline Vector2 Centroid(const vector<CustomContactPoint2D>& cps)
{
  Vector2 com(Zero);
  for(size_t i=0;i<cps.size();i++) 
    com += cps[i].x;
  com /= cps.size();
  return com;
}


inline void GetForceMinimizationDirection(const vector<CustomContactPoint2D>& cps,Vector& c)
{
  int m=0;
  for(size_t j=0;j<cps.size();j++) {
    if(cps[j].numForceVariables() == 1)
      c[m] = 1;
    else {
      cps[j].n.get(c(m),c(m+1));
      for(int p=2;p<cps[j].numForceVariables();p++)
        c[m+p] = 0.0;
    }
    m += cps[j].numForceVariables();
  }
}

inline void SplitForceVector(const Vector& f,const vector<CustomContactPoint2D>& cps,vector<Vector2>& fcs)
{
  Assert(f.n == NumForceVariables(cps));
  fcs.resize(cps.size());
  int m=0;
  for(size_t i=0;i<cps.size();i++) {
    if(cps[i].numForceVariables() == 1)
      fcs[i] = f(m)*cps[i].n;
    else if(cps[i].numForceVariables() >= 2)
      fcs[i].set(f(m),f(m+1));
    m += cps[i].numForceVariables();
  }
}

inline int NumForceVariables(const CustomContactFormation& cps) { return cps.numForceVariables(); }

inline int NumConstraints(const CustomContactFormation& cps) { return cps.numConstraints(); }

inline Vector3 Centroid(const CustomContactFormation& cps) { return Centroid(cps.contacts); }

inline void GetForceMinimizationDirection(const CustomContactFormation& cps,Vector& c) { GetForceMinimizationDirection(cps.contacts,c); }

inline void SplitForceVector(const Vector& f,const CustomContactFormation& cps,vector<Vector3>& fcs) { SplitForceVector(f,cps.contacts,fcs); }


void GetWrenchMatrix(const vector<ContactPoint2D>& cps,const Vector2& cm,Matrix& A)
{
  if(A.isEmpty())
    A.resize(3,cps.size()*2);
  else if(A.m < 3 || A.n < (int)cps.size()*2) 
    FatalError("Invalid size of non-empty wrench matrix");
  for(size_t j=0;j<cps.size();j++) {
    A(0,j*2) = 1;
    A(1,j*2+1) = 1;

    Vector2 cpref = cps[j].x-cm;
    A(2,j*2) = -cpref.y;
    A(2,j*2+1) = cpref.x;
  }
}

void GetWrenchMatrix(const vector<ContactPoint2D>& cps,const Vector2& cm,SparseMatrix& A)
{
  if(A.isEmpty())
    A.resize(3,cps.size()*2);
  else if(A.m < 3 || A.n < (int)cps.size()*2) 
    FatalError("Invalid size of non-empty wrench matrix");
  for(size_t j=0;j<cps.size();j++) {
    A(0,j*2) = 1;
    A(1,j*2+1) = 1;

    Vector2 cpref = cps[j].x-cm;
    A(2,j*2) = -cpref.y;
    A(2,j*2+1) = cpref.x;
  }
}

void GetWrenchMatrix(const vector<CustomContactPoint2D>& cps,const Vector2& cm,Matrix& A)
{
  if(A.isEmpty())
    A.resize(3,cps.size()*2);
  else if(A.m < 3 || A.n < (int)cps.size()*2) 
    FatalError("Invalid size of non-empty wrench matrix");
  A.resize(3,cps.size()*2);
  for(size_t j=0;j<cps.size();j++) {
    A(0,j*2) = 1;
    A(1,j*2+1) = 1;

    Vector2 cpref = cps[j].x-cm;
    A(2,j*2) = -cpref.y;
    A(2,j*2+1) = cpref.x;
  }
}

void GetWrenchMatrix(const vector<CustomContactPoint2D>& cps,const Vector2& cm,SparseMatrix& A)
{
  if(A.isEmpty())
    A.resize(3,cps.size()*2);
  else if(A.m < 3 || A.n < (int)cps.size()*2) 
    FatalError("Invalid size of non-empty wrench matrix");
  A.resize(3,cps.size()*2);
  for(size_t j=0;j<cps.size();j++) {
    A(0,j*2) = 1;
    A(1,j*2+1) = 1;

    Vector2 cpref = cps[j].x-cm;
    A(2,j*2) = -cpref.y;
    A(2,j*2+1) = cpref.x;
  }
}

bool TestForceClosure(const vector<ContactPoint> & cps,int numFCEdges)
{
  vector<CustomContactPoint> custom(cps.size());
  for(size_t i=0;i<cps.size();i++)
    custom[i].set(cps[i],numFCEdges);
  return TestForceClosure(custom);
}

bool TestForceClosure(const vector<ContactPoint2D> & cps)
{
  vector<CustomContactPoint2D> custom(cps.size());
  for(size_t i=0;i<cps.size();i++)
    custom[i].set(cps[i]);
  return TestForceClosure(custom);
}


bool TestForceClosure(const vector<CustomContactPoint> & cps)
{
  int n=NumForceVariables(cps),m=NumConstraints(cps);

  //min_{f} c1^T sum fi + c2^T sum (fi x pi) s.t.
  //Ai*fi <= bi
  //solve with different vectors c whose convex hull contain the origin.  If all solns are nonzero, the wrench space contains the origin
  Optimization::LinearProgram_Sparse lp;
  lp.Resize(m,n);
  lp.A.setZero();
  //we can bound it... does this change anything?
  //lp.l.set(-1);
  //lp.u.set(1);
  lp.l.set(-Inf);
  lp.u.set(Inf);
  lp.q.set(-Inf);
  lp.p.set(Inf);
  lp.minimize = true;
  GetFrictionConePlanes(cps,lp.A,lp.p);

  Optimization::RobustLPSolver lps;
  //simplex containing the origin
  for(int i=0;i<7;i++) {
    Vector c(6,0.0);
    if(i == 6) c.set(-1.0);
    else c[i] = 1.0;
    for(size_t j=0;j<cps.size();j++) {
      int col = int(j*3);
      lp.c(col) = c[0];
      lp.c(col+1) = c[1];
      lp.c(col+2) = c[2];
      Vector3 c2(c[3],c[4],c[5]);
      Vector3 coeffs; coeffs.setCross(c2,cps[j].x);
      lp.c(col) += coeffs.x;
      lp.c(col+1) += coeffs.y;
      lp.c(col+2) += coeffs.z;
    }

    Optimization::LinearProgram::Result res;
    if(i==0) res=lps.Solve(lp);
    else res = lps.Solve_NewObjective(lp);
    if(res == Optimization::LinearProgram::Infeasible) {
      return false;
    }
    if(res == Optimization::LinearProgram::Unbounded)  {
      /*
      LOG4CXX_INFO(KrisLibrary::logger(),"Direction "<<c<<" unbounded");
      LOG4CXX_INFO(KrisLibrary::logger(),"GLPK result "<<lps.xopt);
      LOG4CXX_INFO(KrisLibrary::logger(),"Objective result "<<lps.xopt.dot(lp.c));
      */
      continue;
    }
    if(res == Optimization::LinearProgram::Feasible) {
      /*
      LOG4CXX_INFO(KrisLibrary::logger(),"Direction "<<c);
      LOG4CXX_INFO(KrisLibrary::logger(),"GLPK result "<<lps.xopt);
      LOG4CXX_INFO(KrisLibrary::logger(),"Objective result "<<lps.xopt.dot(lp.c));
      */
      //test for zero
      if(lps.xopt.dot(lp.c) > -Epsilon) {
        //got a zero, not force closure
        return false;
      }
    }
  }
  return true;
}

bool TestForceClosure(const vector<CustomContactPoint2D> & cps)
{
  int n=NumForceVariables(cps),m=NumConstraints(cps);

  //min_{f} c1^T sum fi + c2*sum (fi x pi) s.t.
  //Ai*fi <= bi
  //solve with different vectors c whose convex hull contain the origin.  If all solns are nonzero, the wrench space contains the origin
  Optimization::LinearProgram_Sparse lp;
  lp.Resize(m,n);
  lp.A.setZero();
  lp.l.set(-Inf);
  lp.u.set(Inf);
  lp.q.set(-Inf);
  lp.p.set(Inf);
  lp.minimize = true;
  GetFrictionConePlanes(cps,lp.A,lp.p);

  Optimization::RobustLPSolver lps;
  //simplex containing the origin
  for(int i=0;i<4;i++) {
    Vector c(3,0.0);
    if(i == 3) c.set(-1.0);
    else c[i] = 1.0;
    for(size_t j=0;j<cps.size();j++) {
      int col = int(i*2);
      lp.c(col) = c[0];
      lp.c(col+1) = c[1];
      lp.c(col) += -c[2]*cps[j].x.y;
      lp.c(col+1) += c[2]*cps[j].x.x;
    }
    

    Optimization::LinearProgram::Result res;
    if(i==0) res=lps.Solve(lp);
    else res = lps.Solve_NewObjective(lp);
    if(res == Optimization::LinearProgram::Infeasible) {
      return false;
    }
    if(res == Optimization::LinearProgram::Unbounded) 
      continue;
    if(res == Optimization::LinearProgram::Feasible) {
      //test for zero
      if(lps.xopt.dot(lp.c) > -Epsilon) {
        //got a zero, not force closure
        return false;
      }
    }
  }
  return true;
}

/**
 * Solves
 *   sum fk = -fext
 *   sum [cm-pk]fk = 0
 *   fk in FCk
 */
bool TestCOMEquilibrium(const vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,const Vector3& com,vector<Vector3>& f)
{
  EquilibriumTester tester;
  if(tester.TestCOM(contacts,fext,numFCEdges,com)) {
    if(!f.empty()) {
      tester.GetForces(f);
    }
    return true;
  }
  return false;
  /*
  int numContacts = (int)contacts.size();
  int m=6+numContacts*numFCEdges;
  int n=numContacts*3;

#if USE_SPARSE_LP
  Optimization::LinearProgram_Sparse lp;
  lp.Resize(m,n);

  int j;
  Matrix3 crossProd;
  for(j=0;j<numContacts;j++) {
    lp.A.insertEntry(0,j*3,One);
    lp.A.insertEntry(1,j*3+1,One);
    lp.A.insertEntry(2,j*3+2,One);

    crossProd.setCrossProduct(com-contacts[j].x);
    lp.A.insertEntry(3,j*3+1,crossProd(0,1));
    lp.A.insertEntry(3,j*3+2,crossProd(0,2));
    lp.A.insertEntry(4,j*3+0,crossProd(1,0));
    lp.A.insertEntry(4,j*3+2,crossProd(1,2));
    lp.A.insertEntry(5,j*3+0,crossProd(2,0));
    lp.A.insertEntry(5,j*3+1,crossProd(2,1));
  }
  //set the lower bounds for the friction cones
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = 0;
  lp.q(4) = lp.p(4) = 0;
  lp.q(5) = lp.p(5) = 0;

  //FC bound on f's 
  Matrix temp(numFCEdges,3);
  for(int p=0;p<numContacts;p++) {
    FrictionConePolygon fc;
    fc.set(numFCEdges,contacts[p].n,contacts[p].kFriction);
    for(int i=0;i<numFCEdges;i++) {
      temp(i,0) = -fc.planes[i].x;
      temp(i,1) = -fc.planes[i].y;
      temp(i,2) = -fc.planes[i].z;
    }
    lp.A.copySubMatrix(6+p*numFCEdges,3*p,temp);
  }

#else

  Optimization::LinearProgram lp;
  lp.Resize(m,n);

  int j;
  Matrix3 crossProd;
  for(j=0;j<numContacts;j++) {
    lp.A(0,j*3) = 1;
    lp.A(1,j*3+1) = 1;
    lp.A(2,j*3+2) = 1;

    crossProd.setCrossProduct(com-contacts[j].x);
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++)
	lp.A(3+p,j*3+q) = crossProd(p,q);
  }
  //set the lower bounds for the friction cones
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = 0;
  lp.q(4) = lp.p(4) = 0;
  lp.q(5) = lp.p(5) = 0;

  //FC bound on f's 
  Matrix temp;
  temp.setRef(lp.A,6,0,1,1,numContacts*numFCEdges,numContacts*3);
  GetFrictionConePlanes(contacts,numFCEdges,temp);

#endif // USE_SPARSE_LP

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  for(int j=0;j<numContacts;j++)
    contacts[j].n.get(lp.c(j*3),lp.c(j*3+1),lp.c(j*3+2));
  lp.minimize = true;

  Optimization::RobustLPSolver lps;
  Optimization::LinearProgram::Result res=lps.Solve(lp);
  if(res == Optimization::LinearProgram::Feasible) return true;
  Assert(res != Optimization::LinearProgram::Unbounded);
  return false;
  */
}

bool TestCOMEquilibrium(const vector<CustomContactPoint>& contacts,const Vector3& fext,const Vector3& com,vector<Vector3>& f)
{
  EquilibriumTester tester;
  if(tester.TestCOM(contacts,fext,com)) {
    if(!f.empty()) {
      tester.GetForces(f);
    }
    return true;
  }
  return false;
}

/**
 * Solves over fk,cm
 *   sum fk = -fext
 *   sum pk x fk - fext x cm = 0
 *   fk in FCk
 */
bool TestAnyCOMEquilibrium(const vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges)
{
  EquilibriumTester tester;
  return tester.TestAnyCOM(contacts,fext,numFCEdges);
  /*
  Optimization::LinearProgram lp;
  int numContacts = (int)contacts.size();
  int m=6+numContacts*numFCEdges;
  int n=numContacts*3+3;
  lp.Resize(m,n);

  Matrix& A=lp.A;
  int j;
  Matrix3 crossProd;
  for(j=0;j<numContacts;j++) {
    A(0,j*3) = 1;
    A(1,j*3+1) = 1;
    A(2,j*3+2) = 1;

    crossProd.setCrossProduct(contacts[j].x);
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++)
	A(3+p,j*3+q) = crossProd(p,q);
  }
  crossProd.setCrossProduct(fext);
  for(int p=0;p<3;p++)
    for(int q=0;q<3;q++)
      A(3+p,numContacts*3+q) = -crossProd(p,q);

  //set the lower bounds for the friction cones
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = 0;
  lp.q(4) = lp.p(4) = 0;
  lp.q(5) = lp.p(5) = 0;

  //FC bound on f's 
  Matrix temp;
  temp.setRef(lp.A,6,0,1,1,numContacts*numFCEdges,numContacts*3);
  GetFrictionConePlanes(contacts,numFCEdges,temp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  for(int j=0;j<numContacts;j++)
    contacts[j].n.get(lp.c(j*3),lp.c(j*3+1),lp.c(j*3+2));
  lp.minimize = true;

  Optimization::RobustLPSolver lps;
  Optimization::LinearProgram::Result res=lps.Solve(lp);
  if(res == Optimization::LinearProgram::Feasible) {
    return true;
  }
  return false;
  */
}

bool TestAnyCOMEquilibrium(const vector<CustomContactPoint>& contacts,const Vector3& fext)
{
  EquilibriumTester tester;
  return tester.TestAnyCOM(contacts,fext);
}

/**
 * Solves
 *   sum fk = -fext
 *   sum [cm-pk]fk = 0
 *   fk in FCk
 */
bool TestCOMEquilibrium(const vector<ContactPoint2D>& contacts,const Vector2& fext,const Vector2& com,vector<Vector2>& f)
{
  vector<CustomContactPoint2D> custom(contacts.size());
  for(size_t i=0;i<contacts.size();i++)
    custom[i].set(contacts[i]);
  return TestCOMEquilibrium(custom,fext,com,f);
}

bool TestCOMEquilibrium(const vector<CustomContactPoint2D>& contacts,const Vector2& fext,const Vector2& com,vector<Vector2>& f)
{
  if(contacts.empty()) return false;

  int n = NumForceVariables(contacts);
  int m = 3+NumConstraints(contacts);

  Optimization::LinearProgram lp;
  lp.Resize(m,n);

  //make wrench matrix
  GetWrenchMatrix(contacts,com,lp.A);
  //set the lower bounds for the friction cones
  lp.q.set(-Inf); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = 0;

  //FC bound on f's 
  Matrix temp;
  temp.setRef(lp.A,3,0,1,1,lp.A.m-3,n);
  Vector vtemp;
  vtemp.setRef(lp.p,3,1,lp.A.m-3);
  GetFrictionConePlanes(contacts,temp,vtemp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;

  Optimization::RobustLPSolver lps;
  Optimization::LinearProgram::Result res=lps.Solve(lp);
  if(res == Optimization::LinearProgram::Feasible) return true;
  Assert(res != Optimization::LinearProgram::Unbounded);
  return false;
}



bool TestAnyCOMEquilibrium(const vector<ContactPoint2D>& contacts,const Vector2& fext)
{
  vector<CustomContactPoint2D> custom(contacts.size());
  for(size_t i=0;i<contacts.size();i++)
    custom[i].set(contacts[i]);
  return TestAnyCOMEquilibrium(custom,fext);
}

/**
 * Solves over fk,cm
 *   sum fk = -fext
 *   sum pk x fk - fext x cm = 0
 *   fk in FCk
 */
bool TestAnyCOMEquilibrium(const vector<CustomContactPoint2D>& contacts,const Vector2& fext)
{
  if(contacts.empty()) return false;

  Optimization::LinearProgram lp;
  int m=3 + NumConstraints(contacts);
  int n= NumForceVariables(contacts)+2;
  int cmoffset = NumForceVariables(contacts);
  lp.Resize(m,n);

  Matrix& A=lp.A;
  GetWrenchMatrix(contacts,Vector2(0.0),A);
  A(2,cmoffset) = -fext.y;
  A(2,cmoffset) = fext.x;

  //set the lower bounds for the friction cones
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = 0;

  //FC bound on f's 
  Matrix temp;
  temp.setRef(lp.A,3,0,1,1,lp.A.m-3,cmoffset);
  Vector vtemp;
  vtemp.setRef(lp.p,3,1,lp.A.m-3);
  GetFrictionConePlanes(contacts,temp,vtemp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;

  Optimization::RobustLPSolver lps;
  Optimization::LinearProgram::Result res=lps.Solve(lp);
  if(res == Optimization::LinearProgram::Feasible) {
    return true;
  }
  return false;
}







EquilibriumTester::EquilibriumTester()
  :testingAnyCOM(false),conditioningShift(Zero),numFCEdges(0)
{}

bool EquilibriumTester::TestCOM(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,const Vector3& com)
{
  if(contacts.empty()) return false;
  Setup(contacts,fext,numFCEdges,com);
  testedCOM = com;
  return TestCurrent();
}

bool EquilibriumTester::TestCOM(const std::vector<CustomContactPoint>& contacts,const Vector3& fext,const Vector3& com)
{
  if(contacts.empty()) return false;
  Setup(contacts,fext,com);
  testedCOM = com;
  return TestCurrent();
}

bool EquilibriumTester::TestCOM(const CustomContactFormation& contacts,const Vector3& fext,const Vector3& com)
{
  if(contacts.contacts.empty()) return false;
  Setup(contacts,fext,com);
  testedCOM = com;
  return TestCurrent();
}

/**
 * Sets up the LP
 *   sum fk = fext
 *   sum [pk-p]fk = [cm-p]fext
 *   fk in FCk
 * (p is a constant shift)
 */
void EquilibriumTester::Setup(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,const Vector3& com)
{
  this->numFCEdges = numFCEdges;

  testingAnyCOM = false;
  conditioningShift = com;
  int m=6+NumConstraints(contacts,numFCEdges);
  int n=NumForceVariables(contacts);

  lp.Resize(m,n);
  lp.A.setZero();

  GetWrenchMatrix(contacts,conditioningShift,lp.A);

  //set the lower bounds for the friction cones
  //lp.q.set(Zero); lp.p.set(Inf); 
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  Vector3 mext;
  mext.setCross(com-conditioningShift,fext);
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = -mext.x;
  lp.q(4) = lp.p(4) = -mext.y;
  lp.q(5) = lp.p(5) = -mext.z;

  //FC bound on f's 
  SparseMatrix temp;
  GetFrictionConePlanes(contacts,numFCEdges,temp);
  lp.A.copySubMatrix(6,0,temp);
  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;
}

/**
 * Sets up the LP
 *   sum fk = fext
 *   sum [pk-p]fk = [cm-p]fext
 *   fk in FCk
 * (p is a constant shift)
 */
void EquilibriumTester::Setup(const std::vector<CustomContactPoint>& contacts,const Vector3& fext,const Vector3& com)
{
  this->numFCEdges = -1;

  testingAnyCOM = false;
  conditioningShift = com;
  int m=6+NumConstraints(contacts);
  int n=NumForceVariables(contacts);

  lp.Resize(m,n);
  lp.A.setZero();

  GetWrenchMatrix(contacts,conditioningShift,lp.A);

  //set the lower bounds for the friction cones
  //lp.q.set(Zero); lp.p.set(Inf); 
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  Vector3 mext;
  mext.setCross(com-conditioningShift,fext);
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = -mext.x;
  lp.q(4) = lp.p(4) = -mext.y;
  lp.q(5) = lp.p(5) = -mext.z;

  //FC bound on f's 
  SparseMatrix temp;
  Vector btemp;
  GetFrictionConePlanes(contacts,temp,btemp);
  lp.A.copySubMatrix(6,0,temp);
  lp.p.copySubVector(6,btemp);

  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;
}

void EquilibriumTester::Setup(const CustomContactFormation& contacts,const Vector3& fext,const Vector3& com)
{
  this->numFCEdges = -1;

  testingAnyCOM = false;
  conditioningShift = com;
  int m=6+NumConstraints(contacts);
  int n=NumForceVariables(contacts);

  lp.Resize(m,n);
  lp.A.setZero();

  GetWrenchMatrix(contacts,conditioningShift,lp.A);

  //set the lower bounds for the friction cones
  //lp.q.set(Zero); lp.p.set(Inf); 
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  Vector3 mext;
  mext.setCross(com-conditioningShift,fext);
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = -mext.x;
  lp.q(4) = lp.p(4) = -mext.y;
  lp.q(5) = lp.p(5) = -mext.z;

  //FC bound on f's 
  SparseMatrix temp;
  Vector btemp;
  GetFrictionConePlanes(contacts,temp,btemp);
  lp.A.copySubMatrix(6,0,temp);
  lp.p.copySubVector(6,btemp);

  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;
}

bool EquilibriumTester::TestAnyCOM(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges)
{
  this->numFCEdges = numFCEdges;
  if(contacts.empty()) return false;
  SetupAnyCOM(contacts,fext,numFCEdges);
  return TestCurrent();
}

bool EquilibriumTester::TestAnyCOM(const std::vector<CustomContactPoint>& contacts,const Vector3& fext)
{
  this->numFCEdges = -1;
  if(contacts.empty()) return false;
  SetupAnyCOM(contacts,fext);
  return TestCurrent();
}

bool EquilibriumTester::TestAnyCOM(const CustomContactFormation& contacts,const Vector3& fext)
{
  this->numFCEdges = -1;
  if(contacts.contacts.empty()) return false;
  SetupAnyCOM(contacts,fext);
  return TestCurrent();
}


/**
 * Sets up LP over fk,cm
 *   sum fk = fext
 *   sum (pk-p) x fk + fext x cm = 0
 *   fk in FCk
 * (add p to cm to get a stable com)
 */
void EquilibriumTester::SetupAnyCOM(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges)
{
  this->numFCEdges = numFCEdges;

  testingAnyCOM = true;
  int m=6+NumConstraints(contacts,numFCEdges);
  int n=NumForceVariables(contacts)+3;
  int cmoffset = NumForceVariables(contacts);
  lp.Resize(m,n);

  conditioningShift = Centroid(contacts);

  GetWrenchMatrix(contacts,conditioningShift,lp.A);
  Matrix3 crossProd;
  crossProd.setCrossProduct(fext);
  for(int p=0;p<3;p++)
    for(int q=0;q<3;q++)
      lp.A(3+p,cmoffset+q) = crossProd(p,q);

  //set the lower bounds for the friction cones
  //lp.q.set(Zero); lp.p.set(Inf); 
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = 0;
  lp.q(4) = lp.p(4) = 0;
  lp.q(5) = lp.p(5) = 0;

  //FC bound on f's 
  SparseMatrix temp;
  GetFrictionConePlanes(contacts,numFCEdges,temp);
  lp.A.copySubMatrix(6,0,temp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;
}

/**
 * Sets up LP over fk,cm
 *   sum fk = fext
 *   sum (pk-p) x fk + fext x cm = 0
 *   fk in FCk
 * (add p to cm to get a stable com)
 */
void EquilibriumTester::SetupAnyCOM(const std::vector<CustomContactPoint>& contacts,const Vector3& fext)
{
  this->numFCEdges = -1;

  testingAnyCOM = true;
  int m=6+NumConstraints(contacts);
  int n=NumForceVariables(contacts)+3;
  int cmoffset = NumForceVariables(contacts);
  lp.Resize(m,n);

  conditioningShift = Centroid(contacts);

  GetWrenchMatrix(contacts,conditioningShift,lp.A);
  Matrix3 crossProd;
  crossProd.setCrossProduct(fext);
  for(int p=0;p<3;p++)
    for(int q=0;q<3;q++)
      lp.A(3+p,cmoffset+q) = crossProd(p,q);

  //set the lower bounds for the friction cones
  //lp.q.set(Zero); lp.p.set(Inf); 
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = 0;
  lp.q(4) = lp.p(4) = 0;
  lp.q(5) = lp.p(5) = 0;

  //FC bound on f's 
  SparseMatrix temp;
  Vector btemp;
  GetFrictionConePlanes(contacts,temp,btemp);
  lp.A.copySubMatrix(6,0,temp);
  lp.p.copySubVector(6,btemp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;
}

/**
 * Sets up LP over fk,cm
 *   sum fk = fext
 *   sum (pk-p) x fk + fext x cm = 0
 *   fk in FCk
 * (add p to cm to get a stable com)
 */
void EquilibriumTester::SetupAnyCOM(const CustomContactFormation& contacts,const Vector3& fext)
{
  this->numFCEdges = -1;

  testingAnyCOM = true;
  int m=6+NumConstraints(contacts);
  int n=NumForceVariables(contacts)+3;
  int cmoffset = NumForceVariables(contacts);
  lp.Resize(m,n);

  conditioningShift = Centroid(contacts);

  GetWrenchMatrix(contacts,conditioningShift,lp.A);
  Matrix3 crossProd;
  crossProd.setCrossProduct(fext);
  for(int p=0;p<3;p++)
    for(int q=0;q<3;q++)
      lp.A(3+p,cmoffset+q) = crossProd(p,q);

  //set the lower bounds for the friction cones
  //lp.q.set(Zero); lp.p.set(Inf); 
  lp.q.set(-Inf); lp.p.set(Zero); 
  //set the bounds for the equalities
  lp.q(0) = lp.p(0) = -fext.x;
  lp.q(1) = lp.p(1) = -fext.y;
  lp.q(2) = lp.p(2) = -fext.z;
  lp.q(3) = lp.p(3) = 0;
  lp.q(4) = lp.p(4) = 0;
  lp.q(5) = lp.p(5) = 0;

  //FC bound on f's 
  SparseMatrix temp;
  Vector btemp;
  GetFrictionConePlanes(contacts,temp,btemp);
  lp.A.copySubMatrix(6,0,temp);
  lp.p.copySubVector(6,btemp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  GetForceMinimizationDirection(contacts,lp.c);
  lp.minimize = true;
}

void EquilibriumTester::ChangeContacts(const std::vector<ContactPoint>& contacts)
{
  Assert(numFCEdges > 0);
  int oldNumContacts = NumContacts();
  Assert(oldNumContacts == (int)contacts.size());

  GetWrenchMatrix(contacts,conditioningShift,lp.A);

  //FC bound on f's 
  SparseMatrix temp;
  GetFrictionConePlanes(contacts,numFCEdges,temp);
  lp.A.copySubMatrix(6,0,temp);

  GetForceMinimizationDirection(contacts,lp.c);
} 

void EquilibriumTester::ChangeContact(int i,ContactPoint& contact)
{
  Assert(numFCEdges > 0);
  int numContacts = NumContacts();
  Assert(i >= 0 && i < numContacts);

  Matrix3 crossProd;
  crossProd.setCrossProduct(contact.x-conditioningShift);
  for(int p=0;p<3;p++)
    for(int q=0;q<3;q++)
      lp.A(3+p,i*3+q) = crossProd(p,q);

  //FC bound on f's 
  Matrix temp;
  GetFrictionConePlanes(contact,numFCEdges,temp);
  lp.A.copySubMatrix(6+i*numFCEdges,i*3,temp);

  //change objective -- not strictly necessary
  contact.n.get(lp.c(i*3),lp.c(i*3+1),lp.c(i*3+2));
}

void EquilibriumTester::ChangeGravity(const Vector3& fext)
{
  Assert(numFCEdges > 0);
  if(testingAnyCOM) {
    int numContacts = NumContacts();
    Matrix3 crossProd;
    crossProd.setCrossProduct(fext);
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++)
	lp.A(3+p,numContacts*3+q) = crossProd(p,q);
    lp.q(0) = lp.p(0) = -fext.x;
    lp.q(1) = lp.p(1) = -fext.y;
    lp.q(2) = lp.p(2) = -fext.z;
  }
  else {
    Vector3 mext;
    mext.setCross(testedCOM-conditioningShift,fext);
    lp.q(0) = lp.p(0) = -fext.x;
    lp.q(1) = lp.p(1) = -fext.y;
    lp.q(2) = lp.p(2) = -fext.z;
    lp.q(3) = lp.p(3) = -mext.x;
    lp.q(4) = lp.p(4) = -mext.y;
    lp.q(5) = lp.p(5) = -mext.z;
  }
}

void EquilibriumTester::ChangeCOM(const Vector3& com)
{
  Assert(numFCEdges > 0);
  if(testingAnyCOM) {
  }
  else {
    Vector3 fext(lp.q(0),lp.q(1),lp.q(2));
    Vector3 mext;
    mext.setCross(com-conditioningShift,fext);
    lp.q(3) = lp.p(3) = -mext.x;
    lp.q(4) = lp.p(4) = -mext.y;
    lp.q(5) = lp.p(5) = -mext.z;
  }
}

bool EquilibriumTester::TestCurrent()
{
  Optimization::LinearProgram::Result res=lps.Solve(lp);
  if(res == Optimization::LinearProgram::Feasible) {
    return true;
  }
  if(res == Optimization::LinearProgram::Unbounded) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Strange, equilibrium test problem is unbounded?\n");
    lp.Print(cout);
  }
  Assert(res != Optimization::LinearProgram::Unbounded);
  return false;
}

void EquilibriumTester::GetValidCOM(Vector3& com) const
{
  Assert(testingAnyCOM);
  int base = lp.A.n - 3;
  com.set(lps.xopt(base),lps.xopt(base+1),lps.xopt(base+2));
  com += conditioningShift;
}

void EquilibriumTester::SetRobustnessFactor(Real frobust)
{
  //all forces must be within this distance of the friction cone edge
  for(int i=6;i<lp.p.n;i++)
    lp.p(i) = frobust;
}

void EquilibriumTester::SetRobustnessFactor(int i,Real frobust)
{
  Assert(numFCEdges > 0);
  for(int j=0;j<numFCEdges;j++)
    lp.p(6+i*numFCEdges+j) = frobust;
}

void EquilibriumTester::LimitContactForce(int i,Real maximum,const Vector3& dir)
{
  Assert(numFCEdges > 0);
  SparseVector v(lp.A.n);
  v(i*3)=dir.x;
  v(i*3+1)=dir.y;
  v(i*3+2)=dir.z;
  lp.AddConstraint(-Inf,v,maximum);
}

void EquilibriumTester::LimitContactForceSum(const std::vector<int>& indices,Real maximum,const Vector3& dir)
{
  Assert(numFCEdges > 0);
  SparseVector v(lp.A.n);
  for(size_t j=0;j<indices.size();j++) {
    int i=indices[j];
    v(i*3)=dir.x;
    v(i*3+1)=dir.y;
    v(i*3+2)=dir.z;
  }
  lp.AddConstraint(-Inf,v,maximum);
}

void EquilibriumTester::GetForceVector(Vector& f) const
{
  if(testingAnyCOM) {
    int size = lp.A.n - 3;
    f.resize(size);
    lps.xopt.getSubVectorCopy(0,f);
  }
  else {
    f = lps.xopt;
  }
}

void EquilibriumTester::GetForces(vector<Vector3>& f) const
{
  Assert(numFCEdges > 0);
  f.resize(NumContacts());
  Vector fv;
  GetForceVector(fv);
  Assert(fv.n == (int)f.size()*3);
  for(size_t i=0;i<f.size();i++) {
    int k=(int)i*3;
    f[i].set(fv(k),fv(k+1),fv(k+2));
  }
}

bool EquilibriumTester::IsEmpty()
{
  return lp.A.n == 0;
}

void EquilibriumTester::Clear()
{
  testingAnyCOM = false;
  lp.Resize(0,0);
  lp.A.clear();
}

int EquilibriumTester::NumContacts() const
{
  Assert(numFCEdges > 0);
  if(testingAnyCOM) return (lp.A.n - 3)/3;
  else return lp.A.n/3;
}

int EquilibriumTester::NumFCEdges() const
{
  return numFCEdges;
}


















/* max_{x,y,f} ax+by s.t.
 * sum fi + G = 0
 * sum pi x fi + (x,y,z)x G = 0
 * Ai*fi <= 0 for all i
 *
 * (x,y,z)xG = (y g, -x g, 0)
 */
bool SupportPolygon::Set(const std::vector<ContactPoint>& cp,const Vector3& _fext,int _numFCEdges,int maxExpandDepth)
{
  fext=_fext;
  numFCEdges=_numFCEdges;
  contacts = cp;
 
  if(!(fext.x == Zero && fext.y == Zero && fext.z != Zero)) {
    FatalError("SupportPolygon can only be solved for a z direction force");
  }

  //setup LP 
  Optimization::LinearProgram lp;
  int m=6+NumConstraints(cp,_numFCEdges);
  int n=NumForceVariables(cp)+2;
  int numContacts = cp.size();
  lp.Resize(m,n);

  Matrix& A=lp.A;
  int j;
  //flip the normal order so that xc,yc go on top
  A(0,0) = -fext.z;
  A(1,1) = fext.z;
  A(0,1) = A(0,1) = 0;
  for(j=2;j<6;j++)
    A(j,0) = A(j,1) = 0;
  //note that the y moment is 1, x moment is 2
  Vector3 moment;
  for(j=0;j<numContacts;j++) {
    //moment for x force
    moment.setCross(contacts[j].x,Vector3(1,0,0));
    A(0,j*3+2) = moment.y;
    A(1,j*3+2) = moment.x;
    A(5,j*3+2) = moment.z;
    //moment for y force
    moment.setCross(contacts[j].x,Vector3(0,1,0));
    A(0,j*3+2+1) = moment.y;
    A(1,j*3+2+1) = moment.x;
    A(5,j*3+2+1) = moment.z;
    //moment for z force
    moment.setCross(contacts[j].x,Vector3(0,0,1));
    A(0,j*3+2+2) = moment.y;
    A(1,j*3+2+2) = moment.x;
    A(5,j*3+2+2) = moment.z;

    A(2,j*3+2) = 1;
    A(3,j*3+2+1) = 1;
    A(4,j*3+2+2) = 1;
  }
  //set the lower bounds for the friction cones
  lp.q.set(Zero); lp.p.set(Inf); 
  //set the bounds for the equalities
  for(j=0;j<6;j++)
    lp.q(j) = lp.p(j) = 0;
  lp.q(4) = lp.p(4) = -fext.z;

  //FC bound on f's 
  Matrix temp;
  temp.setRef(A,6,2,1,1,numContacts*numFCEdges,numContacts*3);
  for(j=0;j<numContacts;j++) {
    FrictionConePolygon fc;
    fc.set(numFCEdges,contacts[j].n,contacts[j].kFriction);
    for(int i=0;i<numFCEdges;i++) {
      temp(j*numFCEdges+i,j*3) =  fc.planes[i].x;
      temp(j*numFCEdges+i,j*3+1) =  fc.planes[i].y;
      temp(j*numFCEdges+i,j*3+2) =  fc.planes[i].z;
    }
  }

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  lp.minimize = false;
  Assert(!lp.HasLowerBound(lp.VariableType(0)));
  Assert(!lp.HasLowerBound(lp.VariableType(1)));
  Assert(!lp.HasUpperBound(lp.VariableType(0)));
  Assert(!lp.HasUpperBound(lp.VariableType(1)));

  //the optimized x will be [xc,yc,q]
  // lp.Print();

  //now expand the support polygon
  //start with 2 points
  PolytopeProjection2D expander(lp);
  expander.maxDepth = maxExpandDepth;
  expander.Expand();
  expander.Create(*this);

  /*
  for(size_t i=0;i<planes.size();i++) {
    LOG4CXX_INFO(KrisLibrary::logger(),"plane ["<<planes[i].normal<<"].x <= "<<planes[i].offset);
  }
  */

  return true;
}

/* max_{x,y,f} ax+by s.t.
 * sum fi + G = 0
 * sum pi x fi + (x,y,z)x G = 0
 * Ai*fi <= 0 for all i
 *
 * (x,y,z)xG = (y g, -x g, 0)
 */
bool SupportPolygon::Set(const std::vector<CustomContactPoint>& cp,const Vector3& _fext,int maxExpandDepth)
{
  fext=_fext;
  numFCEdges=-1;
  contacts.resize(cp.size());
  for(size_t i=0;i<cp.size();i++) {
    contacts[i].x = cp[i].x;
    contacts[i].n = cp[i].n;
    contacts[i].kFriction = cp[i].kFriction;
  }
 
  if(!(fext.x == Zero && fext.y == Zero && fext.z != Zero)) {
    FatalError("SupportPolygon can only be solved for a z direction force");
  }

  //setup LP 
  Optimization::LinearProgram lp;
  int numContacts = (int)contacts.size();
  int m=6+NumConstraints(cp);
  int n=NumForceVariables(cp)+2;
  lp.Resize(m,n);

  Matrix& A=lp.A;
  int j;
  //flip the normal order so that xc,yc go on top
  A(0,0) = -fext.z;
  A(1,1) = fext.z;
  A(0,1) = A(0,1) = 0;
  for(j=2;j<6;j++)
    A(j,0) = A(j,1) = 0;
  //note that the y moment is 1, x moment is 2
  Vector3 moment;
  for(j=0;j<numContacts;j++) {
    //moment for x force
    moment.setCross(contacts[j].x,Vector3(1,0,0));
    A(0,j*3+2) = moment.y;
    A(1,j*3+2) = moment.x;
    A(5,j*3+2) = moment.z;
    //moment for y force
    moment.setCross(contacts[j].x,Vector3(0,1,0));
    A(0,j*3+2+1) = moment.y;
    A(1,j*3+2+1) = moment.x;
    A(5,j*3+2+1) = moment.z;
    //moment for z force
    moment.setCross(contacts[j].x,Vector3(0,0,1));
    A(0,j*3+2+2) = moment.y;
    A(1,j*3+2+2) = moment.x;
    A(5,j*3+2+2) = moment.z;

    A(2,j*3+2) = 1;
    A(3,j*3+2+1) = 1;
    A(4,j*3+2+2) = 1;
  }
  //set the lower bounds for the friction cones
  lp.q.set(Zero); lp.p.set(Inf); 
  //set the bounds for the equalities
  for(j=0;j<6;j++)
    lp.q(j) = lp.p(j) = 0;
  lp.q(4) = lp.p(4) = -fext.z;

  //FC bound on f's 
  Matrix temp;
  temp.setRef(A,6,2,1,1,NumConstraints(cp),NumForceVariables(cp));
  Vector btemp;
  btemp.setRef(lp.p,6,1,NumConstraints(cp));
  GetFrictionConePlanes(cp,temp,btemp);

  //lp.Print(cout);
  //KrisLibrary::loggerWait();

  lp.c.setZero();
  lp.minimize = false;
  Assert(!lp.HasLowerBound(lp.VariableType(0)));
  Assert(!lp.HasLowerBound(lp.VariableType(1)));
  Assert(!lp.HasUpperBound(lp.VariableType(0)));
  Assert(!lp.HasUpperBound(lp.VariableType(1)));

  //the optimized x will be [xc,yc,q]
  // lp.Print();

  //now expand the support polygon
  //start with 2 points
  PolytopeProjection2D expander(lp);
  expander.maxDepth = maxExpandDepth;
  expander.Expand();
  expander.Create(*this);

  /*
  for(size_t i=0;i<planes.size();i++) {
    LOG4CXX_INFO(KrisLibrary::logger(),"plane ["<<planes[i].normal<<"].x <= "<<planes[i].offset);
  }
  */

  return true;
}

/* max_{x,y,f} ax+by s.t.
 * sum fi + G = 0
 * sum pi x fi + (x,y,z)x G = 0
 * Ai*fi <= 0 for all i
 *
 * (x,y,z)xG = (y g, -x g, 0)
 */
bool SupportPolygon::Set(const CustomContactFormation& cp,const Vector3& _fext,int maxExpandDepth)
{
  fext=_fext;
  numFCEdges=-1;
  contacts.resize(cp.contacts.size());
  for(size_t i=0;i<cp.contacts.size();i++) {
    contacts[i].x = cp.contacts[i].x;
    contacts[i].n = cp.contacts[i].n;
    contacts[i].kFriction = cp.contacts[i].kFriction;
  }
 
  if(!(fext.x == Zero && fext.y == Zero && fext.z != Zero)) {
    FatalError("SupportPolygon can only be solved for a z direction force");
  }

  //setup LP 
  Optimization::LinearProgram_Sparse lp;
  int numContacts = (int)cp.contacts.size();
  int m=6+NumConstraints(cp);
  int n=NumForceVariables(cp)+2;
  lp.Resize(m,n);

  SparseMatrix& A=lp.A;
  int j;
  //flip the normal order so that xc,yc go on top
  A(0,0) = -fext.z;
  A(1,1) = fext.z;
  A(0,1) = A(0,1) = 0;
  for(j=2;j<6;j++)
    A(j,0) = A(j,1) = 0;
  //note that the y moment is 1, x moment is 2
  Vector3 moment;
  for(j=0;j<numContacts;j++) {
    //moment for x force
    moment.setCross(contacts[j].x,Vector3(1,0,0));
    A(0,j*3+2) = moment.y;
    A(1,j*3+2) = moment.x;
    A(5,j*3+2) = moment.z;
    //moment for y force
    moment.setCross(contacts[j].x,Vector3(0,1,0));
    A(0,j*3+2+1) = moment.y;
    A(1,j*3+2+1) = moment.x;
    A(5,j*3+2+1) = moment.z;
    //moment for z force
    moment.setCross(contacts[j].x,Vector3(0,0,1));
    A(0,j*3+2+2) = moment.y;
    A(1,j*3+2+2) = moment.x;
    A(5,j*3+2+2) = moment.z;

    A(2,j*3+2) = 1;
    A(3,j*3+2+1) = 1;
    A(4,j*3+2+2) = 1;
  }
  //set the lower bounds for the friction cones
  lp.q.set(Zero); lp.p.set(Inf); 
  //set the bounds for the equalities
  for(j=0;j<6;j++)
    lp.q(j) = lp.p(j) = 0;
  lp.q(4) = lp.p(4) = -fext.z;

  //FC bound on f's 
  SparseMatrix temp;
  Vector btemp;
  btemp.setRef(lp.p,6,1,NumConstraints(cp));
  GetFrictionConePlanes(cp,temp,btemp);
  A.copySubMatrix(6,2,temp);

  lp.c.setZero();
  lp.minimize = false;
  Assert(!lp.HasLowerBound(lp.VariableType(0)));
  Assert(!lp.HasLowerBound(lp.VariableType(1)));
  Assert(!lp.HasUpperBound(lp.VariableType(0)));
  Assert(!lp.HasUpperBound(lp.VariableType(1)));

  //now expand the support polygon
  //start with 2 points
  PolytopeProjection2D expander(lp);
  expander.maxDepth = maxExpandDepth;
  expander.Expand();
  expander.Create(*this);

  return true;
}




bool SupportPolygon::TestCOM(const Vector3& com) const
{
  return Contains(Vector2(com.x,com.y));
}

Real SupportPolygon::COMMargin(const Vector3& com) const
{
  return Margin(Vector2(com.x,com.y));
}


/*
Two old support polygon computation codes... using polytope enumeration
bool SupportPolygon::SolveA()
{
#ifndef HAS_POLYTOPE
  FatalError("No polytope");
#else
  if(!(fext.x == Zero && fext.y == Zero && fext.z != Zero)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"SupportPolygon can only be solved for a z direction force\n");
    Abort();
  }
  //solve for force limit polyhedron Af=b with all f>=0
  //vertices get mapped to xc,yc via [xc,yc]t = Bf

  HPolytope poly;
  Matrix& A = poly.A;
  Vector& b = poly.b;

  //setup A matrix
  Vector B1,B2;
  int numContacts = (int)contacts.size();
  int numConstraints = numContacts*numFCEdges;
  int m = 4+numConstraints, n=numContacts*3;
  A.resize(m,n,Zero);
  B1.resize(n);
  B2.resize(n);
  int j;
  Matrix3 moment;
  for(j=0;j<numContacts;j++) {
    A(0,j*3) = 1;
    A(1,j*3+1) = 1;
    A(2,j*3+2) = 1;
    moment.setCrossProduct(contacts[j].x);
    B1(j*3) = moment(0,0); B1(j*3+1) = moment(0,1); B1(j*3+2) = moment(0,2);
    B2(j*3) = moment(1,0); B2(j*3+1) = moment(1,1); B2(j*3+2) = moment(1,2);
    A(3,j*3) = moment(2,0); A(3,j*3+1) = moment(2,1); A(3,j*3+2) = moment(2,2);
  }
  B1 *= -fext.z;
  B2 *= fext.z;

  int row=4;
  for(j=0;j<numContacts;j++) {
    FrictionConePolygon fc;
    fc.set(numFCEdges,contacts[j].n,contacts[j].kFriction);
    for(int i=0;i<numFCEdges;i++,row++) {
      A(row,j*3) = -fc.planes[i].x;
      A(row,j*3+1) = -fc.planes[i].y;
      A(row,j*3+2) = -fc.planes[i].z;
    }
  }

  //setup B vector
  b.resize(m,Zero);
  for(int i=0;i<3;i++) { b(i) = fext[i]; }

  poly.equality.resize(4,true); poly.equality.resize(m,false);
  poly.positiveX = false;
  poly.Print(cout);

  //do vertex enumeration on the polytope
  VPolytope vpoly;
  poly.GetVPolytope(vpoly);

  //project that into 2D
  std::vector<PointRay2D> pts;
  PointRay2D y;
  y.isRay = false;
  for(int i=0;i<vpoly.points.m;i++) {
    y.x = vpoly.points.dotRow(i,B1);
    y.y = vpoly.points.dotRow(i,B2);
    pts.push_back(y);
    //LOG4CXX_INFO(KrisLibrary::logger(),"point ("<<y.x<<","<<y.y);
  }
  y.isRay = true;
  for(int i=0;i<vpoly.rays.m;i++) {
    y.x = vpoly.rays.dotRow(i,B1);
    y.y = vpoly.rays.dotRow(i,B2);
    pts.push_back(y);
    //LOG4CXX_INFO(KrisLibrary::logger(),"ray ("<<y.x<<","<<y.y);
  }

  //find the convex hull
  vertices.resize(pts.size()+1);
  int k = ConvexHull2D_Chain_Unsorted(&pts[0],pts.size(),&vertices[0]);
  vertices.resize(k);

  //find the planes
  planes.resize(k);
  int np = Point2DListToPlanes(&vertices[0],k,&planes[0]);
  planes.resize(np);
  return true;
#endif
}

bool SupportPolygon::SolveB()
{
#ifndef HAS_CONTACT
  LOG4CXX_INFO(KrisLibrary::logger(),"No polytope");
  Abort();
#else
  if(!(fext.x == Zero && fext.y == Zero && fext.z != Zero)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"SupportPolygon can only be solved for a z direction force\n");
    Abort();
  }
  //solve for force limit polyhedron Af=b with fi in FCi
  //vertices get mapped to xc,yc via [xc,yc]t = Bf

  //setup A matrix
  Matrix A;
  Vector3 moment;
  Vector B1,B2;
  int n = (int)contacts.size();
  A.resize(4,n);
  B1.resize(n);
  B2.resize(n);
  int j;
  for(j=0;j<n;j++) {
    A(0,j) = contacts[j].n.x;
    A(1,j) = contacts[j].n.y;
    A(2,j) = contacts[j].n.z;
    moment.setCross(contacts[j].x,contacts[j].n);
    B1(j) = moment.x;
    B2(j) = moment.y;
    A(3,j) = moment.z;	
  }
  for(j=0;j<n;j++) {
    B1(j) /= -fext.z;
    B2(j) /= fext.z;
  }

  //setup B vector
  Vector b(4);
  for(int i=0;i<3;i++) { b(i) = fext[i]; }
  b(3) = Zero;

  HPolytope poly;
  poly.A = A;
  poly.b = b;
  poly.equality.resize(4,true);
  poly.positiveX = true;

  //do vertex enumeration on the polytope
  VPolytope vpoly;
  poly.GetVPolytope(vpoly);

  //project that into 2D
  std::vector<PointRay2D> pts;
  PointRay2D y;
  y.isRay = false;
  for(int i=0;i<vpoly.points.m;i++) {
    y.x = vpoly.points.dotRow(i,B1);
    y.y = vpoly.points.dotRow(i,B2);
    pts.push_back(y);
  }
  y.isRay = true;
  for(int i=0;i<vpoly.rays.m;i++) {
    y.x = vpoly.rays.dotRow(i,B1);
    y.y = vpoly.rays.dotRow(i,B2);
    pts.push_back(y);
  }

  //find the convex hull
  vertices.resize(pts.size()+1);
  int k = ConvexHull2D_Chain_Unsorted(&pts[0],pts.size(),&vertices[0]);
  vertices.resize(k);

  //find the planes
  planes.resize(k);
  int np = Point2DListToPlanes(&vertices[0],k,&planes[0]);
  planes.resize(np);
  return true;
#endif
}
*/



bool OrientedSupportPolygon::Set(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges)
{
  /*TEST
  T.setIdentity();
  //T.t -= conditioningShift;
  T.t.x = -1.0;
  Vector3 tfext = fext;
  */

  Vector3 z=fext,x,y;
  z.inplaceNormalize();
  GetCanonicalBasis(z,x,y);

  T.R.set(x,y,z);
  T.t = Centroid(contacts);
  T.inplaceInverse();  //(x,y,z),shift is the transformed -> orig transform
  Vector3 tfext(0,0,fext.norm());
  vector<ContactPoint> tcontacts(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    tcontacts[i].x = T*contacts[i].x;
    tcontacts[i].n = T.R*contacts[i].n;
    tcontacts[i].kFriction = contacts[i].kFriction;
  }
  return sp.Set(tcontacts,tfext,numFCEdges);
}

bool OrientedSupportPolygon::TestCOM(const Vector3& com) const
{
  return sp.TestCOM(T*com);
}

Real OrientedSupportPolygon::COMMargin(const Vector3& com) const
{
  return sp.COMMargin(T*com);
}

void OrientedSupportPolygon::GetXYSlice(Real height,UnboundedPolytope2D& poly) const
{
  //transform SP space to original space
  RigidTransform Tinv;
  Tinv.setInverse(T);
  Vector3 x,y,z;
  Tinv.R.get(x,y,z);
  Matrix2 R2;
  R2(0,0) = T.R(0,0);
  R2(0,1) = T.R(0,1);
  R2(1,0) = T.R(1,0);
  R2(1,1) = T.R(1,1);
  Real det=R2.determinant();
  Assert(Abs(det) > Epsilon);  //otherwise the slice is infinite...
  Assert(Abs(z.z) > Epsilon);  //otherwise the slice is infinite...
  bool flip = (det < 0.0);  //the plane is flipped
  poly.vertices.resize(sp.vertices.size());
  for(size_t i=0;i<sp.vertices.size();i++) {
    const PointRay2D& v=sp.vertices[i];
    Real u;
    //let p = [v.x,v.y,u] (u unknown), q be the transformed point
    if(v.isRay) {
      //pick u s.t. p = T*[q.x,q.y,0]  //parallel to xy plane
      u = (-x.z*v.x - y.z*v.y)/z.z;
      poly.vertices[i].isRay = true;
      poly.vertices[i].x = x.x*v.x + y.x*v.y + u*z.x;
      poly.vertices[i].y = x.y*v.x + y.y*v.y + u*z.y;
    }
    else {
      //pick u s.t. p = T*[q.x,q.y,height]
      u = (height - x.z*v.x - y.z*v.y - Tinv.t.z)/z.z;
      poly.vertices[i].isRay = false;
      poly.vertices[i].x = x.x*v.x + y.x*v.y + u*z.x + Tinv.t.x;
      poly.vertices[i].y = x.y*v.x + y.y*v.y + u*z.y + Tinv.t.y;
    }
  }
  if(flip) //maintain CCW ordering
    reverse(poly.vertices.begin(),poly.vertices.end());
  //laziness
  poly.CalcPlanes();
}

void OrientedSupportPolygon::GetSlice(const RigidTransform& frame,UnboundedPolytope2D& poly) const
{
  OrientedSupportPolygon temp;
  temp.sp = sp;
  temp.T = T*frame;
  temp.GetXYSlice(0,poly);
}

















