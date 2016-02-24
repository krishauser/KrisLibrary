#include "SelfTest.h"
#include "random.h"
#include "matrix.h"
#include "linalgebra.h"
#include "DiagonalMatrix.h"
#include "vectorfunction.h"
#include "differentiation.h"
#include "quadrature.h"
#include "LUDecomposition.h"
#include "BlockTridiagonalMatrix.h"
#include "BlockPrinter.h"
#include "BLASInterface.h"
#include "LAPACKInterface.h"
#include "MatrixPrinter.h"
#include "VectorPrinter.h"
#include "metric.h"
#include <errors.h>
#include <utils/fileutils.h>
#include <string.h>
#include <fstream>
#include <iostream>
using namespace std;

#ifndef _WIN32
#include <unistd.h>
#endif //_WIN32

namespace Math {

inline bool RFuzzyEquals(Real a,Real b,Real rtol) 
{
  Real s=Max(Abs(a),Abs(b));
  return FuzzyEquals(a,b,rtol*s);
}

bool CheckError(Real a,Real b,Real atol,Real rtol)
{
  return (!FuzzyEquals(a,b,atol) && !RFuzzyEquals(a,b,rtol));
}

bool CheckError(const Vector& a,const Vector& b,Vector& err,Real atol,Real rtol)
{
  err.sub(a,b);
  for(int i=0;i<err.n;i++)
    if(err(i) > atol &&
       err(i) > rtol*Max(Abs(a(i)),Abs(b(i)))) return true;
  return false;
}

bool CheckError(const Matrix& a,const Matrix& b,Matrix& err,Real atol,Real rtol)
{
  err.sub(a,b);
  for(int i=0;i<err.m;i++)
    for(int j=0;j<err.n;j++)
      if(err(i,j) > atol &&
	 err(i,j) > rtol*Max(Abs(a(i,j)),Abs(b(i,j)))) return true;
  return false;
}



void SelfTest()
{
  BasicSelfTest();
  VectorSelfTest();
  MatrixSelfTest();
  DifferentiationSelfTest();
  QuadratureSelfTest();
  BlockVectorSelfTest();
  BlockMatrixSelfTest();
}

void BasicSelfTest()
{
  cout<<"Self-testing basic"<<endl;
  cout<<"Not done"<<endl;
  getchar();
}

void TestVectorBasic()
{
  const Real vals [] = { 0,1,2,-3 };
  Vector v1,v2(10),v3(10,55.0),v4(v3),v5(4,vals);
  Assert(v2.n == 10);
  Assert(v3.n == 10);
  Assert(v4.n == 10);
  Assert(v5.n == 4);
  Assert(v3[0] == 55.0);
  Assert(v3[7] == 55.0);
  Assert(v4[1] == 55.0);
  Assert(v4[5] == 55.0);
  v3[4] = 0;
  Assert(v3[4] == 0.0);
  Assert(v4[4] != 0.0);
  v4[4] = v3[4];
  Assert(v2.isValidIndex(1));
  Assert(v2.isValidIndex(9));
  Assert(!v2.isValidIndex(-1));
  Assert(!v2.isValidIndex(10));
  Assert(v5[0] == 0);
  Assert(v5[1] == 1);
  Assert(v5[2] == 2);
  Assert(v5[3] == -3);
  Assert(v4 == v3);
  Assert(v4 != v5);
  Assert(v4 != v2);
  v1.resize(1);
  v1[0] = 2;
  swap(v1,v2);
  Assert(v1.n == 10);
  Assert(v2.n == 1);
}

void TestVectorOps()
{
  cout<<"Vector ops test not done"<<endl;
}

void VectorSelfTest()
{
  cout<<"Self-testing vectors"<<endl;
  TestVectorBasic();
  TestVectorOps();
  cout<<"Done"<<endl;
  getchar();
}

void MatrixSelfTest()
{
  cout<<"Self-testing matrices"<<endl;
  const Real v33[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
  Matrix m(3,3),m2(3,3,v33),m3(m2),m4;
  Vector v(3),v2,v3;
  Vector r,r1,r2;
  m.setIdentity();
  int k=1;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++,k++) {
      if(i==j) Assert(m(i,j) == Delta(i,j));
      Assert(m2(i,j) == k);
      Assert(m3(i,j) == k);
    }
  m2.getRowRef(1,r);
  Assert(r[0] == 4);  Assert(r[1] == 5);  Assert(r[2] == 6);
  m2.getColRef(2,r);
  Assert(r[0] == 3);  Assert(r[1] == 6);  Assert(r[2] == 9);
  Assert(r.n == 3);
  v(0) = 1;
  v(1) = 2;
  v(2) = 3;
  m4.sub(m2,m);
  //cout<<"A"<<endl<<MatrixPrinter(m4)<<endl;
  //cout<<"b  "<<VectorPrinter(v)<<endl;
  LUDecomposition<Real> lud;
  Assert(lud.set(m4));
  lud.backSub(v,v2);
  m4.mul(v2,v3);
  //cout<<"A*A^-1*b = "<<VectorPrinter(v3)<<endl;

  lud.getInverse(m2);
  //cout<<"A^-1"<<endl<<MatrixPrinter(m2)<<endl;
  m3.mul(m2,m4);
  for(int i=0;i<3;i++) 
    for(int j=0;j<3;j++) {
      m2.getRowRef(i,r1);
      m4.getColRef(j,r2);
      if(!FuzzyEquals(m3(i,j),r1.dot(r2))) {
	cout<<"Row "<<i<<" of ainv: "<<VectorPrinter(r1)<<endl;
	cout<<"Col "<<j<<" of a: "<<VectorPrinter(r2)<<endl;
	cout<<"matrix mul isn't correct, "<<i<<","<<j<<", "<<m3(i,j)<<" vs "<<r1.dot(r2)<<endl;
      }
      Assert(FuzzyEquals(m3(i,j),r1.dot(r2)));
    }

  //cout<<"A^-1*A"<<endl<<MatrixPrinter(m3)<<endl;
  Assert(m3.isEqual(m,1e-6));
  cout<<"Done"<<endl;
  getchar();
}

void DifferentiationSelfTest()
{
  cout<<"Self-testing differentiation"<<endl;
  cout<<"Not done"<<endl;
  getchar();
}

void QuadratureSelfTest()
{
  cout<<"Self-testing quadrature"<<endl;
  cout<<"Not done"<<endl;
  getchar();
}

void BlockVectorSelfTest()
{
  cout<<"Self-testing block vectors"<<endl;
  BlockVector v1(4,2),v2(4,2,Zero),v3;
  Assert(v1.numBlocks()==4);
  Assert(v2.hasDims(v1));

  v3.resizeSimilar(v1);
  Assert(v3.hasDims(v2));
  v3.clear();
  for(int i=0;i<4;i++) {
    Assert(v1[i].n == 2);
    v1[i].set(i);
  }
  v3.add(v1,v2);
  Assert(v3 == v1);
  v3.clear();

  v3.mul(v1,3.0);
  for(int i=0;i<4;i++) {
    Assert(v3[i].n == 2);
    Assert(v3[i][0] == i*3);
    Assert(v3[i][1] == i*3);
  }
  v3.clear();

  v2.set(One);
  for(int i=0;i<4;i++)
    v2[i][1] = Zero;
  v3.sub(v1,v2);
  for(int i=0;i<4;i++) {
    Assert(v3[i][0] == i-1);
    Assert(v3[i][1] == i);
  }

  v3.madd(v2,-4);
  for(int i=0;i<4;i++) {
    Assert(v3[i][0] == i-5);
    Assert(v3[i][1] == i);
  }

  cout<<"Done"<<endl;
  getchar();
}

void BlockMatrixSelfTest()
{
  cout<<"Self-testing block matrix"<<endl;
  //simple test
  BlockTridiagonalMatrix mtri(4,2);
  BlockVector v(4,2),v2(4,2),v3;

  mtri.setIdentity();
  for(int i=0;i<4;i++)
    v[i].set(i+1);
  mtri.mul(v,v2);

  for(int i=0;i<4;i++) {
    Assert(v2[i][0] == i+1);
    Assert(v2[i][1] == i+1);
  }
  mtri(1,1)(0,0)=5.0;
  mtri(1,1)(1,1)=6.0;
  mtri.mul(v,v2);
  Assert(v2[1][0] == 10);
  Assert(v2[1][1] == 12);
  mtri.solveInverse_LU(v,v2);
  mtri.mul(v2,v3);
  for(int i=0;i<4;i++) {
    if(!v[i].isEqual(v3[i],1e-3)) {
      cout<<"Error! solveInverse is wrong for diagonal matrix"<<endl;
      cout<<BlockPrinter(v)<<endl;
      cout<<BlockPrinter(v3)<<endl;
      Abort();
    }
  }

  BlockTridiagonalMatrix m1(2,2);
  BlockVector vsmall(2,2),vsmall2(2,2),vsmall3(2,2);
  m1(0,0).setIdentity();
  m1(1,1).setIdentity();
  m1(0,1).setIdentity();
  m1(1,0).setIdentity();
  m1(1,0).inplaceMul(Half);
  vsmall.set(One);
  m1.solveInverse_LU(vsmall,vsmall2);
  m1.mul(vsmall2,vsmall3);
  for(int i=0;i<2;i++) {
    if(!vsmall[i].isEqual(vsmall3[i],1e-3)) {
      cout<<"Error! solveInverse is wrong for simple matrix"<<endl;
      cout<<BlockPrinter(vsmall)<<endl;
      cout<<BlockPrinter(vsmall3)<<endl;
      cout<<"inverse is "<<BlockPrinter(vsmall2)<<endl;
      cout<<"matrix: "<<endl;
      cout<<BlockPrinter(m1)<<endl;
      Abort();
    }
  }


  mtri(0,1).setIdentity();
  mtri(3,2).set(Half);
  mtri.solveInverse_LU(v,v2);
  mtri.mul(v2,v3);
  for(int i=0;i<4;i++) {
    if(!v[i].isEqual(v3[i],1e-3)) {
      cout<<"Error! solveInverse is wrong for non-diagonal matrix"<<endl;
      cout<<BlockPrinter(v)<<endl;
      cout<<BlockPrinter(v3)<<endl;
      cout<<BlockPrinter(mtri)<<endl;
      Abort();
    }
  }  


  cout<<"Done"<<endl;
  getchar();
}

void BLASSelfTest()
{
  Vector v(20),v2(20);
  for(int i=0;i<v.n;i++) {
    v(i) = Rand(-One,One);
    v2(i) = Rand(-One,One);
  }
  Assert(FuzzyEquals(Norm_L2(v),BLASInterface::Norm_L2(v)));
  Assert(FuzzyEquals(Norm_L1(v),BLASInterface::Norm_L1(v)));
  Assert(FuzzyEquals(v.dot(v2),BLASInterface::Dot(v,v2)));
  int index;
  v.maxAbsElement(&index);
  int index2 = BLASInterface::MaxAbsIndex(v);
  //printf("max element %g at %d, BLAS: %g at %d\n",v(index),index,v(index2),index2);
  Assert(FuzzyEquals(v(index),v(index2)));

  Vector temp = v2, temp2 = v2;
  temp.inplaceMul(0.3);
  BLASInterface::InplaceMul(temp2,0.3);
  Assert(temp.isEqual(temp2,1e-7));

  temp.madd(v,2.0);
  BLASInterface::Madd(temp2,v,2.0);
  Assert(temp.isEqual(temp2,1e-7));

  //test referencing with strides
  Vector vref,vref2;
  vref.setRef(v,2,2,4);
  Assert(FuzzyEquals(Norm_L2(vref),BLASInterface::Norm_L2(vref)));
  Assert(FuzzyEquals(Norm_L1(vref),BLASInterface::Norm_L1(vref)));
  vref2.setRef(v2,3,3,4);
  Assert(FuzzyEquals(vref.dot(vref2),BLASInterface::Dot(vref,vref2)));
}

//void LAPACKSelfTest();






void TestDifferentiation(VectorFieldFunction& f,Vector& x,Real h,Real eps)
{
  //Tests
  cout<<"Testing Jacobian/Hessians of "<<f.Label()<<"..."<<endl;
  int n=x.n;
  Vector xold=x;
  int nd=f.NumDimensions();
  f.PreEval(x);
  int a,b;

  //Test Jacobians
  Matrix Jcalc(nd,n),Jdiff(nd,n);
  f.Jacobian(x,Jcalc);
  JacobianCenteredDifference(f,x,h,Jdiff); Assert(xold == x);
  for(int i=0;i<nd;i++) {
    Real err=Zero;
    b=0;
    for(int j=0;j<n;j++) {
      if(Abs(Jdiff(i,j)-Jcalc(i,j)) > err) {
	err = Abs(Jdiff(i,j)-Jcalc(i,j));
	b=j;
      }
    }
    a=i;
    if(err > eps) {
      cout<<"Jacobian differs from centered differences by "<<err<<" at entry "<<a<<","<<b<<endl;
      cout<<"Calculated value: "<<Jcalc(a,b)<<" finite difference "<<Jdiff(a,b)<<endl;
      cout<<"Label: "<<f.Label(a)<<endl;
      getchar();
    }
  }

  //Test hessians
  Matrix Hcalc(n,n),Hdiff(n,n);
  for(int i=0;i<nd;i++) {
    f.Hessian_i(x,i,Hcalc);
    VectorFieldProjectionFunction fi(&f,i);
    HessianCenteredDifference_Grad(fi,x,h,Hdiff); Assert(xold == x);
    Hdiff -= Hcalc;
    Real err = Hdiff.maxAbsElement(&a,&b);
    if(err > eps) {
      cout<<"Hessian "<<i<<" differs from centered differences (grad) by "<<err<<" at entry "<<a<<","<<b<<endl;
      cout<<"Label: "<<f.Label(i)<<endl;
      //Hcalc.print();
      //Hdiff.print();
      getchar();
    }
    else {
      cout<<"Numerical Hessian "<<i<<" error "<<err<<endl;
    }
  }
  cout<<"Done."<<endl;
  getchar();
}







void DebugVector(const char* name,const Vector& v)
{
  char* viewProg = getenv("MATRIX_DEBUGGER");
  if(viewProg) {
    char tempFile[1024];
    char prefix[5];
    strncpy(prefix,name,4);
    prefix[4]=0;
    FileUtils::TempName(tempFile,NULL,prefix);
    ofstream out(tempFile);
    out << "1 ";
    out << v <<endl;
    out.close();

#if _WIN32
	int pid=-1;
#else
    int pid=fork();
#endif
    if(pid < 0) { //error
      cerr<<"Error forking process"<<endl;
    }
    else if(pid==0) {  //child process
      char* buf=new char[strlen(viewProg)+strlen(tempFile)+2];
      sprintf(buf,"%s %s",viewProg,tempFile);
      system(buf);
      bool res=FileUtils::Delete(tempFile);
      if(!res) {
	cerr<<"Could not delete file "<<tempFile<<endl;
      }
      exit(0);
    }
    else { //parent process... keep going
      return;
    }
  }
  else {
    VectorPrinter pv(v); if(v.n > 20) pv.mode = VectorPrinter::AsciiShade;
    cout<<name<<": "<<pv<<endl;
    getchar();
  }
}




void DebugMatrix(const char* name,const Matrix& v)
{
  char* viewProg = getenv("MATRIX_DEBUGGER");
  if(viewProg) {
    char tempFile[1024];
    char prefix[5];
    prefix[4]=0;
    strncpy(prefix,name,4);
    FileUtils::TempName(tempFile,NULL,prefix);
    ofstream out(tempFile);
    out << v <<endl;
    out.close();

#if _WIN32
	int pid=-1;
#else
    int pid=fork();
#endif
    if(pid < 0) { //error
      cerr<<"Error forking process"<<endl;
    }
    else if(pid==0) {  //child process
      char* buf=new char[strlen(viewProg)+strlen(tempFile)+2];
      sprintf(buf,"%s %s",viewProg,tempFile);
      system(buf);
      bool res=FileUtils::Delete(tempFile);
      if(!res) {
	cerr<<"Could not delete file "<<tempFile<<endl;
      }
      exit(0);
    }
    else { //parent process... keep going
      return;
    }
  }
  else {
    MatrixPrinter pv(v); if(v.n > 20) pv.mode = MatrixPrinter::AsciiShade;
    cout<<name<<": "<<endl<<pv<<endl;
    getchar();
  }
}


bool TestDeriv(RealFunction* f,Real t,Real h,Real atol,Real rtol)
{
  f->PreEval(t);
  Real d=f->Deriv(t);
  Real ddiff=dfCenteredDifference(*f,t,h);
  if(CheckError(d,ddiff,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    cout<<"Derivative error "<<Abs(d-ddiff)<<endl;
    cout<<"User supplied: "<<d<<endl;
    cout<<"Finite differenced: "<<ddiff<<endl;
    getchar();
    return false;
  }
  return true;
}

bool TestDeriv(VectorFunction* f,Real t,Real h,Real atol,Real rtol)
{
  Vector J(f->NumDimensions()),Jdiff,err;
  f->PreEval(t);
  f->Deriv(t,J);
  dfCenteredDifference(*f,t,h,Jdiff);
  if(CheckError(J,Jdiff,err,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    DebugVector("Differentation error",err);
    DebugVector("J",J);
    DebugVector("Jdiff",Jdiff);
    return false;
  }
  return true;
}

bool TestGradient(ScalarFieldFunction* f,Vector& x,Real h,Real atol,Real rtol)
{
  Vector g,gdiff,err;
  g.resize(x.n);
  gdiff.resize(x.n);
  f->PreEval(x);
  f->Gradient(x,g);
  GradientCenteredDifference(*f,x,h,gdiff);
  if(CheckError(g,gdiff,err,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    DebugVector("Differentation error",err);
    DebugVector("g",g);
    DebugVector("gdiff",gdiff);
    return false;
  }
  return true;
}

bool TestGradients(ScalarFieldFunction* f,Vector& x,Real h,Real atol,Real rtol)
{
  Vector g,gdiff,err;
  f->PreEval(x);
  f->Gradient(x,g);
  for(int i=0;i<x.n;i++) {
    Real gi=f->Gradient_i(x,i);
    if(gi != g(i)) {
      cout<<"Function "<<f->Label()<<endl;
      cout<<"Error in Gradient_i("<<i<<"), result not equal to Gradient()"<<endl;
      return false;
    }
  }
  GradientCenteredDifference(*f,x,h,gdiff);
  if(CheckError(g,gdiff,err,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    DebugVector("Element-wise differentation error",err);
    DebugVector("g",g);
    DebugVector("gdiff",gdiff);
    return false;
  }
  return true;
}

bool TestDeriv2(RealFunction* f,Real t,Real h,Real atol,Real rtol)
{
  f->PreEval(t);
  Real d=f->Deriv2(t);
  Real ddiff=ddfCenteredDifference(*f,t,h);
  if(CheckError(d,ddiff,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    cout<<"2nd derivative error "<<Abs(d-ddiff)<<endl;
    cout<<"User supplied: "<<d<<endl;
    cout<<"Finite differenced: "<<ddiff<<endl;
    getchar();
    return false;
  }
  return true;
}

bool TestDeriv2(VectorFunction* f,Real t,Real h,Real atol,Real rtol)
{
  Vector J(f->NumDimensions()),Jdiff,err;
  f->PreEval(t);
  f->Deriv2(t,J);
  ddfCenteredDifference(*f,t,h,Jdiff);
  if(CheckError(J,Jdiff,err,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    DebugVector("Deriv2 error",err);
    DebugVector("J",J);
    DebugVector("Jdiff",Jdiff);
    return false;
  }
  return true;
}

bool TestHessian(ScalarFieldFunction* f,Vector& x,Real h,Real atol,Real rtol)
{
  Matrix H,Hdiff,err;
  H.resize(x.n,x.n);
  Hdiff.resize(x.n,x.n);
  f->PreEval(x);
  f->Hessian(x,H);
  HessianCenteredDifference(*f,x,h,Hdiff);
  if(CheckError(H,Hdiff,err,atol,rtol)) {
    cout<<"Function "<<f->Label()<<endl;
    DebugMatrix("Differentation error",err);
    DebugMatrix("H",H);
    DebugMatrix("Hdiff",Hdiff);
    return false;
  }
  return true;
}

bool TestJacobian(VectorFieldFunction* f,Vector& x,Real h,Real atol,Real rtol)
{
  Matrix J,Jdiff,err;
  J.resize(f->NumDimensions(),x.n);
  Jdiff.resize(f->NumDimensions(),x.n);
  f->PreEval(x);
  f->Jacobian(x,J);
  JacobianCenteredDifference(*f,x,h,Jdiff);
  if(CheckError(J,Jdiff,err,atol,rtol)) {
    int i,j;
    Real emax = err.maxAbsElement(&i,&j);
    cout<<"Function "<<f->Label()<<endl;
    DebugMatrix("Differentiation error",err);
    cout<<"Max is "<<emax<<" at "<<f->Label(i)<<" w.r.t. "<<f->VariableLabel(j)<<endl;
    DebugMatrix("J",J);
    DebugMatrix("Jdiff",Jdiff);
    return false;
  }
  return true;
}

bool TestJacobians(VectorFieldFunction* f,Vector& x,Real h,Real atol,Real rtol)
{
  Matrix J,Jdiff,err;
  J.resize(f->NumDimensions(),x.n);
  Jdiff.resize(f->NumDimensions(),x.n);
  f->PreEval(x);
  f->Jacobian(x,J);
  for(int i=0;i<J.m;i++) {
    Vector Ji(x.n),temp;
    f->Jacobian_i(x,i,Ji);
    J.getRowRef(i,temp);
    if(!temp.isEqual(Ji,atol)) {
      cout<<"Function "<<f->Label()<<endl;
      cout<<"Error in Jacobian_i("<<i<<"), result not equal to Jacobian()"<<endl;
      DebugVector("J[i]",temp);
      DebugVector("Ji",Ji);
      temp -= Ji;
      DebugVector("err",temp);
      int index;
      Real max=temp.maxAbsElement(&index);
      cout<<"Maximum error of "<<f->Label(i)<<" is "<<max<<" at "<<f->VariableLabel(index)<<endl;
      return false;
    }
  }
  JacobianCenteredDifference(*f,x,h,Jdiff);
  if(CheckError(J,Jdiff,err,atol,rtol)) {
    int i,j;
    Real emax = err.maxAbsElement(&i,&j);
    cout<<"Function "<<f->Label()<<endl;
    DebugMatrix("Differentiation error",err);
    cout<<"Max is "<<emax<<" at "<<f->Label(i)<<" w.r.t. "<<f->VariableLabel(j)<<endl;
    DebugMatrix("J",J);
    DebugMatrix("Jdiff",Jdiff);
    return false;
  }
  //TODO: the other jacobians (Jacobian_j, Jacobian_ij)
  return true;
}



} //namespace Math
