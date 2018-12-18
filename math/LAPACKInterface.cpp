#include <KrisLibrary/Logger.h>
#include "LAPACKInterface.h"
#include "complex.h"
#include "errors.h"
using namespace Math;

bool LAPACKInterface::IsCompliant(const fVector& x)
{
  return (x.stride == 1);
}

bool LAPACKInterface::IsCompliant(const dVector& x)
{
  return (x.stride == 1);
}

bool LAPACKInterface::IsCompliant(const cVector& x)
{
  return (x.stride == 1);
}

bool LAPACKInterface::IsCompliant(const fMatrix& A)
{
  if(A.isRowMajor()) return false;
  return (A.istride == 1);
}

bool LAPACKInterface::IsCompliant(const dMatrix& A)
{
  if(A.isRowMajor()) return false;
  return (A.istride == 1);
}

bool LAPACKInterface::IsCompliant(const cMatrix& A)
{
  if(A.isRowMajor()) return false;
  return (A.istride == 1);
}

void LAPACKInterface::MakeCompliant(const fVector& x,fVector& res)
{
  Assert(res.empty());
  res.resize(x.n);
  res.copy(x);
  Assert(IsCompliant(res));
}

void LAPACKInterface::MakeCompliant(const dVector& x,dVector& res)
{
  Assert(res.empty());
  res.resize(x.n);
  res.copy(x);
  Assert(IsCompliant(res));
}

void LAPACKInterface::MakeCompliant(const cVector& x,cVector& res)
{
  Assert(res.empty());
  res.resize(x.n);
  res.copy(x);
  Assert(IsCompliant(res));
}

void LAPACKInterface::MakeCompliant(const fMatrix& A,fMatrix& res)
{
  Assert(res.isEmpty());
  res.resize(A.m,A.n);
  std::swap(res.istride,res.jstride);  //make column major
  Assert(res.isValid());
  Assert(IsCompliant(res));
  res.copy(A);
}

void LAPACKInterface::MakeCompliant(const dMatrix& A,dMatrix& res)
{
  Assert(res.isEmpty());
  res.resize(A.m,A.n);
  std::swap(res.istride,res.jstride);  //make column major
  Assert(res.isValid());
  Assert(IsCompliant(res));
  res.copy(A);
}

void LAPACKInterface::MakeCompliant(const cMatrix& A,cMatrix& res)
{
  Assert(res.isEmpty());
  res.resize(A.m,A.n);
  std::swap(res.istride,res.jstride);  //make column major
  Assert(res.isValid());
  Assert(IsCompliant(res));
  res.copy(A);
}

#if HAVE_CLAPACK

extern "C" {
#include "f2c.h"
#include "clapack.h"
}

bool LAPACKInterface::Solve(const fMatrix& A,const fVector& b,fVector& x)
{
  Assert(A.isSquare());
  Assert(A.n == b.n);
  Assert(x.isEmpty() || !x.isReference());
  fMatrix LU;
  MakeCompliant(A,LU);

  //copy b into x
  x.resize(0);
  MakeCompliant(b,x);

  //solve it
  integer n = A.m;
  integer nrhs = 1;    //nrhs is 1, because only one vector solved for
  integer ldb = x.n;   //ignored, because only one vector solved for
  integer info=0;
  integer* ipiv = new integer[n];
  integer lda = LU.jstride;
  sgesv_(&n,&nrhs,LU.getStart(),&lda,ipiv,x.getStart(),&ldb,&info);
  delete [] ipiv;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::Solve(const dMatrix& A,const dVector& b,dVector& x)
{
  Assert(A.isSquare());
  Assert(A.n == b.n);
  Assert(x.isEmpty() || !x.isReference());
  dMatrix LU;
  MakeCompliant(A,LU);

  //copy b into x
  x.resize(0);
  MakeCompliant(b,x);

  //solve it
  integer n = A.m;
  integer nrhs = 1;    //nrhs is 1, because only one vector solved for
  integer ldb = x.n;   //ignored, because only one vector solved for
  integer info=0;
  integer* ipiv = new integer[n];
  integer lda = LU.jstride;
  dgesv_(&n,&nrhs,LU.getStart(),&lda,ipiv,x.getStart(),&ldb,&info);
  delete [] ipiv;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::LeastSquares(const fMatrix& A,const fVector& b,fVector& x)
{
  Assert(A.m == b.n);
  Assert(x.isEmpty() || !x.isReference());
  fMatrix QR;
  MakeCompliant(A,QR);

  //copy b into x
  x.resize(0);
  MakeCompliant(b,x);
  if(A.m < A.n) { //minimum norm solution
    x.resize(A.m);
    x.copySubVector(0,b);
  }

  //solve it
  char trans='N';
  integer m = A.m;
  integer n = A.n;
  integer nrhs = 1;    //nrhs is 1, because only one vector solved for
  integer lda = QR.jstride;
  integer ldb = x.n;   //ignored, because only one vector solved for
  integer info=0;
  real worktemp;
  integer lwork = -1;
  //query workspace size
  sgels_(&trans,&m,&n,&nrhs,QR.getStart(),&lda,x.getStart(),&ldb,&worktemp,&lwork,&info);
  //do the LS
  lwork = (int)worktemp;
  real* work = new real[lwork];
  sgels_(&trans,&m,&n,&nrhs,QR.getStart(),&lda,x.getStart(),&ldb,&worktemp,&lwork,&info);
  delete [] work;
  x.n = A.n;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::LeastSquares(const dMatrix& A,const dVector& b,dVector& x)
{
  Assert(A.m == b.n);
  Assert(x.isEmpty() || !x.isReference());
  dMatrix QR;
  MakeCompliant(A,QR);

  //copy b into x
  x.resize(0);
  MakeCompliant(b,x);
  if(A.m < A.n) { //minimum norm solution
    x.resize(A.m);
    x.copySubVector(0,b);
  }

  //solve it
  char trans='N';
  integer m = A.m;
  integer n = A.n;
  integer nrhs = 1;    //nrhs is 1, because only one vector solved for
  integer lda = QR.jstride;
  integer ldb = x.n;   //ignored, because only one vector solved for
  integer info=0;
  doublereal worktemp;
  integer lwork = -1;
  //query workspace size
  dgels_(&trans,&m,&n,&nrhs,QR.getStart(),&lda,x.getStart(),&ldb,&worktemp,&lwork,&info);
  //do the LS
  lwork = (int)worktemp;
  doublereal* work = new doublereal[lwork];
  dgels_(&trans,&m,&n,&nrhs,QR.getStart(),&lda,x.getStart(),&ldb,&worktemp,&lwork,&info);
  delete [] work;
  x.n = A.n;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::Eigenvalues_Symmetric(const fMatrix& A,fVector& lambda)
{
  Assert(A.isSquare());
  fMatrix Atemp;
  MakeCompliant(A,Atemp);

  fVector wtemp;
  wtemp.resize(A.n);
  Assert(IsCompliant(wtemp));

  //solve it
  char job='N';  //only eigenvalues
  char uplo='U';
  integer n = Atemp.n;
  integer lda = Atemp.jstride;
  integer info=0;
  real worktemp;
  integer lwork = -1;
  //query workspace size
  ssyev_(&job,&uplo,&n,Atemp.getStart(),&lda,wtemp.getStart(),&worktemp,&lwork,&info);
  //do the LS
  lwork = (int)worktemp;
  real* work = new real[lwork];
  ssyev_(&job,&uplo,&n,Atemp.getStart(),&lda,wtemp.getStart(),work,&lwork,&info);
  delete [] work;
  lambda = wtemp;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::Eigenvalues_Symmetric(const dMatrix& A,dVector& lambda)
{
  Assert(A.isSquare());
  dMatrix Atemp;
  MakeCompliant(A,Atemp);

  dVector wtemp;
  wtemp.resize(A.n);
  Assert(IsCompliant(wtemp));

  //solve it
  char job='N';  //only eigenvalues
  char uplo='U';
  integer n = Atemp.n;
  integer lda = Atemp.jstride;
  integer info=0;
  doublereal worktemp;
  integer lwork = -1;
  //query workspace size
  dsyev_(&job,&uplo,&n,Atemp.getStart(),&lda,wtemp.getStart(),&worktemp,&lwork,&info);
  //do the LS
  lwork = (int)worktemp;
  doublereal* work = new doublereal[lwork];
  dsyev_(&job,&uplo,&n,Atemp.getStart(),&lda,wtemp.getStart(),work,&lwork,&info);
  delete [] work;
  lambda = wtemp;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::Eigenvectors_Symmetric(const fMatrix& A,fVector& lambda,fMatrix& Q)
{
  Assert(A.isSquare());
  Assert(Q.isEmpty() || !Q.isRef());
  MakeCompliant(A,Q);

  fVector wtemp;
  wtemp.resize(A.n);
  Assert(IsCompliant(wtemp));

  //solve it
  char job='V';   //eigenvectors+eigenvalues
  char uplo='U';
  integer n = Q.n;
  integer lda = Q.jstride;
  integer info=0;
  real worktemp;
  integer lwork = -1;
  //query workspace size
  ssyev_(&job,&uplo,&n,Q.getStart(),&lda,wtemp.getStart(),&worktemp,&lwork,&info);
  //do the LS
  lwork = (int)worktemp;
  real* work = new real[lwork];
  ssyev_(&job,&uplo,&n,Q.getStart(),&lda,wtemp.getStart(),work,&lwork,&info);
  delete [] work;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  lambda = wtemp;
  return true;
}

bool LAPACKInterface::Eigenvectors_Symmetric(const dMatrix& A,dVector& lambda,dMatrix& Q)
{
  Assert(A.isSquare());
  Assert(Q.isEmpty() || !Q.isRef());
  MakeCompliant(A,Q);

  dVector wtemp;
  wtemp.resize(A.n);
  Assert(IsCompliant(wtemp));

  //solve it
  char job='V';   //eigenvectors+eigenvalues
  char uplo='U';
  integer n = Q.n;
  integer lda = Q.jstride;
  integer info=0;
  doublereal worktemp;
  integer lwork = -1;
  //query workspace size
  dsyev_(&job,&uplo,&n,Q.getStart(),&lda,wtemp.getStart(),&worktemp,&lwork,&info);
  Assert(info == 0);
  //do the LS
  lwork = (int)worktemp;
  Assert(lwork > 0);
  doublereal* work = new doublereal[lwork];
  dsyev_(&job,&uplo,&n,Q.getStart(),&lda,wtemp.getStart(),work,&lwork,&info);
  delete [] work;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  lambda = wtemp;
  return true;
}

bool LAPACKInterface::SVD(const fMatrix& A,fMatrix& U,fVector& W,fMatrix& Vt)
{
  fMatrix Atemp;
  MakeCompliant(A,Atemp);
  Assert(U.isEmpty() || !U.isRef());
  Assert(Vt.isEmpty() || !Vt.isRef());
  Assert(W.isEmpty() || !W.isReference());
  U.resize(A.m,A.m);
  std::swap(U.jstride,U.istride);
  Vt.resize(A.n,A.n);
  std::swap(Vt.jstride,Vt.istride);
  W.resize(Min(A.m,A.n));
  Assert(W.stride == 1);

  //solve it
  char jobu='A';   //all left SV's in U
  char jobvt='A';  //all right SV's in Vt
  integer m = A.m;
  integer n = A.n;
  integer lda = Atemp.jstride;
  integer ldu = U.jstride;
  integer ldvt = Vt.jstride;
  integer info=0;
  real worktemp;
  integer lwork = -1;
  //query workspace size
  sgesvd_(&jobu,&jobvt,&m,&n,Atemp.getStart(),&lda,W.getStart(),U.getStart(),&ldu,Vt.getStart(),&ldvt,&worktemp,&lwork,&info);
  Assert(info == 0);
  //do the SVD
  lwork = (int)worktemp;
  Assert(lwork > 0);
  real* work = new real[lwork];
  sgesvd_(&jobu,&jobvt,&m,&n,Atemp.getStart(),&lda,W.getStart(),U.getStart(),&ldu,Vt.getStart(),&ldvt,work,&lwork,&info);
  delete [] work;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

bool LAPACKInterface::SVD(const dMatrix& A,dMatrix& U,dVector& W,dMatrix& Vt)
{
  dMatrix Atemp;
  MakeCompliant(A,Atemp);
  Assert(U.isEmpty() || !U.isRef());
  Assert(Vt.isEmpty() || !Vt.isRef());
  Assert(W.isEmpty() || !W.isReference());
  U.resize(A.m,A.m);
  std::swap(U.jstride,U.istride);
  Vt.resize(A.n,A.n);
  std::swap(Vt.jstride,Vt.istride);
  W.resize(Min(A.m,A.n));
  Assert(W.stride == 1);

  //solve it
  char jobu='A';   //all left SV's in U
  char jobvt='A';  //all right SV's in Vt
  integer m = A.m;
  integer n = A.n;
  integer lda = Atemp.jstride;
  integer ldu = U.jstride;
  integer ldvt = Vt.jstride;
  integer info=0;
  doublereal worktemp;
  integer lwork = -1;
  //query workspace size
  dgesvd_(&jobu,&jobvt,&m,&n,Atemp.getStart(),&lda,W.getStart(),U.getStart(),&ldu,Vt.getStart(),&ldvt,&worktemp,&lwork,&info);
  Assert(info == 0);
  //do the LS
  lwork = (int)worktemp;
  Assert(lwork > 0);
  doublereal* work = new doublereal[lwork];
  dgesvd_(&jobu,&jobvt,&m,&n,Atemp.getStart(),&lda,W.getStart(),U.getStart(),&ldu,Vt.getStart(),&ldvt,work,&lwork,&info);
  delete [] work;
  if(info != 0) {  //failed somehow
    if(info < 0) //input error
      FatalError("Uh... info=%d\n",info);
    FatalError("Error... info=%d\n",info);
    return false;
  }
  return true;
}

#else

#include <iostream>
using namespace std;

bool LAPACKInterface::Solve(const fMatrix& A,const fVector& b,fVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::Solve(const dMatrix& A,const dVector& b,dVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::LeastSquares(const fMatrix& A,const fVector& b,fVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::LeastSquares(const dMatrix& A,const dVector& b,dVector& x)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::Eigenvalues_Symmetric(const fMatrix& A,fVector& lambda)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::Eigenvalues_Symmetric(const dMatrix& A,dVector& lambda)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::Eigenvectors_Symmetric(const fMatrix& A,fVector& lambda,fMatrix& Q)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::Eigenvectors_Symmetric(const dMatrix& A,dVector& lambda,dMatrix& Q)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::SVD(const fMatrix& A,fMatrix& U,fVector& W,fMatrix& Vt)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}

bool LAPACKInterface::SVD(const dMatrix& A,dMatrix& U,dVector& W,dMatrix& Vt)
{
  LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, LAPACK not defined");
  return false;
}


#endif
