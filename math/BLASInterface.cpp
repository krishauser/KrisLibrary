#include <KrisLibrary/Logger.h>
#include "BLASInterface.h"
#include <errors.h>
using namespace Math;

#if HAVE_BLAS

extern "C" {

#include "f2c.h"
#include "fblaswr.h"

}

float BLASInterface::Dot(const fVector& x,const fVector& y)
{
  Assert(x.n == y.n);
  integer n=x.n;
  integer xinc = x.stride;
  integer yinc = y.stride;
  return sdot_(&n,x.getStart(),&xinc,y.getStart(),&yinc);
}

double BLASInterface::Dot(const dVector& x,const dVector& y)
{
  Assert(x.n == y.n);
  integer n=x.n;
  integer xinc = x.stride;
  integer yinc = y.stride;
  return ddot_(&n,x.getStart(),&xinc,y.getStart(),&yinc);
}

float BLASInterface::Norm_L2(const fVector& x)
{
  integer n=x.n;
  integer xinc = x.stride;
  return snrm2_(&n,x.getStart(),&xinc);
}

double BLASInterface::Norm_L2(const dVector& x)
{
  integer n=x.n;
  integer xinc = x.stride;
  return dnrm2_(&n,x.getStart(),&xinc);
}

float BLASInterface::Norm_L1(const fVector& x)
{
  integer n=x.n;
  integer xinc = x.stride;
  return sasum_(&n,x.getStart(),&xinc);
}

double BLASInterface::Norm_L1(const dVector& x)
{
  integer n=x.n;
  integer xinc = x.stride;
  return dasum_(&n,x.getStart(),&xinc);
}

int BLASInterface::MaxAbsIndex(const fVector& x)
{
  integer n=x.n;
  integer xinc = x.stride;
  return isamax_(&n,x.getStart(),&xinc)-1;
}

int BLASInterface::MaxAbsIndex(const dVector& x)
{
  integer n=x.n;
  integer xinc = x.stride;
  return idamax_(&n,x.getStart(),&xinc)-1;
}

void BLASInterface::Madd(fVector& v,const fVector& x,float a)
{
  integer n=x.n;
  integer vinc = v.stride;
  integer xinc = x.stride;
  saxpy_(&n,&a,x.getStart(),&xinc,v.getStart(),&vinc);
}

void BLASInterface::Madd(dVector& v,const dVector& x,double a)
{
  integer n=x.n;
  integer vinc = v.stride;
  integer xinc = x.stride;
  daxpy_(&n,&a,x.getStart(),&xinc,v.getStart(),&vinc);
}

void BLASInterface::InplaceMul(fVector& x,float a)
{
  integer n=x.n;
  integer xinc = x.stride;
  sscal_(&n,&a,x.getStart(),&xinc);
}

void BLASInterface::InplaceMul(dVector& x,double a)
{
  integer n=x.n;
  integer xinc = x.stride;
  dscal_(&n,&a,x.getStart(),&xinc);
}

void BLASInterface::Mul(const fMatrix& A,const fVector& x,fVector& out)
{
  out.resize(A.m);
  out.setZero();
  Madd(A,x,out);
}

void BLASInterface::Mul(const dMatrix& A,const dVector& x,dVector& out)
{
  out.resize(A.m);
  out.setZero();
  Madd(A,x,out);
}

void BLASInterface::MulTranspose(const fMatrix& A,const fVector& x,fVector& out)
{
  out.resize(A.n);
  out.setZero();
  MaddTranspose(A,x,out);
}

void BLASInterface::MulTranspose(const dMatrix& A,const dVector& x,dVector& out)
{
  out.resize(A.n);
  out.setZero();
  MaddTranspose(A,x,out);
}


bool IsCompliant(const fMatrix& A)
{
  return (!A.isRowMajor() && A.istride == 1);
}

bool IsCompliant(const dMatrix& A)
{
  return (!A.isRowMajor() && A.istride == 1);
}

void Madd(const fMatrix& A,const fVector& x,fVector& y,float alpha,float beta,bool transpose)
{
  if(A.isRowMajor()) {
    Assert(A.jstride == 1);
    transpose = !transpose;
  }
  else {
    Assert(A.istride == 1);
    //Assert(IsCompliant(A));
  }
  Assert(x.n == A.n);
  Assert(y.n == A.m);
  integer m=A.m;
  integer n=A.n;
  integer lda=A.m;
  integer xinc=x.stride;
  integer yinc=y.stride;
  char trans = (transpose?'T':'N');
  sgemv_(&trans,&m,&n,&alpha,A.getStart(),&lda,x.getStart(),&xinc,&beta,y.getStart(),&yinc);
}

void Madd(const dMatrix& A,const dVector& x,dVector& y,double alpha,double beta,bool transpose)
{
  if(A.isRowMajor()) {
    Assert(A.jstride == 1);
    transpose = !transpose;
  }
  else {
    Assert(A.istride == 1);
    //Assert(IsCompliant(A));
  }
  //Assert(IsCompliant(A));
  Assert(x.n == A.n);
  Assert(y.n == A.m);
  integer m=A.m;
  integer n=A.n;
  integer lda=A.m;
  integer xinc=x.stride;
  integer yinc=y.stride;
  char trans = (transpose?'T':'N');
  dgemv_(&trans,&m,&n,&alpha,A.getStart(),&lda,x.getStart(),&xinc,&beta,y.getStart(),&yinc);
}

void BLASInterface::Madd(const fMatrix& A,const fVector& x,fVector& y,float alpha,float beta)
{
  ::Madd(A,x,y,alpha,beta,false);
}

void BLASInterface::Madd(const dMatrix& A,const dVector& x,dVector& y,double alpha,double beta)
{
  ::Madd(A,x,y,alpha,beta,false);
}

void BLASInterface::MaddTranspose(const fMatrix& A,const fVector& x,fVector& y,float alpha,float beta)
{
  ::Madd(A,x,y,alpha,beta,true);
}


void BLASInterface::MaddTranspose(const dMatrix& A,const dVector& x,dVector& y,double alpha,double beta)
{
  ::Madd(A,x,y,alpha,beta,true);
}




void BLASInterface::Mul(const fMatrix& A,const fMatrix& B,fMatrix& X)
{
  X.resize(A.m,B.n);
  X.setZero();
  if(X.isRowMajor()) std::swap(X.istride,X.jstride);
  Madd(A,B,X);
}

void BLASInterface::Mul(const dMatrix& A,const dMatrix& B,dMatrix& X)
{
  X.resize(A.m,B.n);
  X.setZero();
  if(X.isRowMajor()) std::swap(X.istride,X.jstride);
  Madd(A,B,X);
}

void BLASInterface::Madd(const fMatrix& A,const fMatrix& B,fMatrix& X,bool Atranspose,bool Btranspose,float alpha,float beta)
{
  Assert(IsCompliant(A));
  Assert(IsCompliant(B));
  Assert(IsCompliant(X));
  Assert(X.m == (Atranspose?A.n:A.m));
  Assert(X.n == (Btranspose?B.m:B.n));

  integer m = (Atranspose?A.n:A.m);
  integer n = (Btranspose?B.m:B.n);
  integer k = (Atranspose?A.m:A.n);
  Assert(k == (Btranspose?B.n:B.m));
  integer lda = A.m;
  integer ldb = B.m;
  integer ldx = X.m;
  char transa = (Atranspose?'T':'N');
  char transb = (Btranspose?'T':'N');
  sgemm_(&transa,&transb,&m,&n,&k,
	 &alpha,
	 A.getStart(),&lda,
	 B.getStart(),&ldb,
	 &beta,
	 X.getStart(),&ldx);
}

void BLASInterface::Madd(const dMatrix& A,const dMatrix& B,dMatrix& X,bool Atranspose,bool Btranspose,double alpha,double beta)
{
  Assert(IsCompliant(A));
  Assert(IsCompliant(B));
  Assert(IsCompliant(X));
  Assert(X.m == (Atranspose?A.n:A.m));
  Assert(X.n == (Btranspose?B.m:B.n));

  integer m = (Atranspose?A.n:A.m);
  integer n = (Btranspose?B.m:B.n);
  integer k = (Atranspose?A.m:A.n);
  Assert(k == (Btranspose?B.n:B.m));
  integer lda = A.m;
  integer ldb = B.m;
  integer ldx = X.m;
  char transa = (Atranspose?'T':'N');
  char transb = (Btranspose?'T':'N');
  dgemm_(&transa,&transb,&m,&n,&k,
	 &alpha,
	 A.getStart(),&lda,
	 B.getStart(),&ldb,
	 &beta,
	 X.getStart(),&ldx);
}


#else

#include <iostream>
using namespace std;

float BLASInterface::Dot(const fVector& x,const fVector& y)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

double BLASInterface::Dot(const dVector& x,const dVector& y)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

float BLASInterface::Norm_L2(const fVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

double BLASInterface::Norm_L2(const dVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

float BLASInterface::Norm_L1(const fVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

double BLASInterface::Norm_L1(const dVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

int BLASInterface::MaxAbsIndex(const fVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

int BLASInterface::MaxAbsIndex(const dVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
  return 0;
}

void BLASInterface::Madd(fVector& v,const fVector& x,float a)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Madd(dVector& v,const dVector& x,double a)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::InplaceMul(fVector& x,float a)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::InplaceMul(dVector& x,double a)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Mul(const fMatrix& A,const fVector& x,fVector& out)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Mul(const dMatrix& A,const dVector& x,dVector& out)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::MulTranspose(const fMatrix& A,const fVector& x,fVector& out)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::MulTranspose(const dMatrix& A,const dVector& x,dVector& out)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}


bool IsCompliant(const fMatrix& A)
{
  return (!A.isRowMajor() && A.istride == 1);
}

bool IsCompliant(const dMatrix& A)
{
  return (!A.isRowMajor() && A.istride == 1);
}

void BLASInterface::Madd(const fMatrix& A,const fVector& x,fVector& y,float alpha,float beta)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Madd(const dMatrix& A,const dVector& x,dVector& y,double alpha,double beta)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::MaddTranspose(const fMatrix& A,const fVector& x,fVector& y,float alpha,float beta)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}


void BLASInterface::MaddTranspose(const dMatrix& A,const dVector& x,dVector& y,double alpha,double beta)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}




void BLASInterface::Mul(const fMatrix& A,const fMatrix& B,fMatrix& X)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Mul(const dMatrix& A,const dMatrix& B,dMatrix& X)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Madd(const fMatrix& A,const fMatrix& B,fMatrix& X,bool Atranspose,bool Btranspose,float alpha,float beta)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}

void BLASInterface::Madd(const dMatrix& A,const dMatrix& B,dMatrix& X,bool Atranspose,bool Btranspose,double alpha,double beta)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, BLAS not defined");
}


#endif  //HAVE_BLAS
