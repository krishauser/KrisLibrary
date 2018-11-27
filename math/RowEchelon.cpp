#include <KrisLibrary/Logger.h>
#include "RowEchelon.h"
#include "GramSchmidt.h"
#include "MatrixPrinter.h"
#include "complex.h"
#include <limits.h>
#include <iostream>
using namespace std;

namespace Math {

#define SWAP(a,b) { temp=a; a=b; b=temp; }

template <class T>
int RowEchelonDecompose(MatrixTemplate<T>& A,MatrixTemplate<T>& B,Real zeroTol)
{
  if(!B.isEmpty())
    Assert(B.m == A.m);
  int m=A.m,n=A.n;
  int p=B.n;

  int i,j,icur=0,jcur;
  T temp;
  for(jcur=0;jcur<n;jcur++) {
    //find pivot element in col jcur from rows icur..m
    Real big=Zero;
    int ipivot=-1;
    for(i=icur;i<m;i++) {
      if(Abs(A(i,jcur)) > big) {
	ipivot = i;
	big = Abs(A(i,jcur));
      }
    }
    if(!FuzzyZero(big,zeroTol)) { //nonzero pivot found
      //exchange rows ipivot,icur
      if(ipivot != icur) {
	for(j=jcur;j<n;j++) SWAP(A(ipivot,j),A(icur,j));
	for(j=0;j<p;j++) SWAP(B(ipivot,j),B(icur,j));
      }
      //eliminate rows below icur
      T scale;
      for(i=icur+1;i<m;i++) {
	//set row(ai) = row(ai)-aiJ/aIJ*row(aI)
	scale = A(i,jcur)/A(icur,jcur);
	for(j=jcur;j<n;j++) A(i,j) -= A(icur,j)*scale;
	for(j=0;j<p;j++) B(i,j) -= B(icur,j)*scale;
	A(i,jcur)=Zero;
      }
      icur++;
    }
    else {
      //either zero pivot, or very small one
      //set to zero to reduce numerical difficulties later 
      for(i=icur;i<m;i++) A(i,jcur)=Zero;
    }
  }
  return Max(m-icur,n-jcur);
}

template <class T>
void ReduceRowEchelon(MatrixTemplate<T>& A,MatrixTemplate<T>& B)
{
  if(!B.isEmpty())
    Assert(B.m == A.m);

  int i,j,icur,jcur;
  for(icur=A.m-1;icur>=0;icur--) { 
    //find pivot for row icur
    jcur = -1;
    for(j=0;j<A.n;j++) if(A(icur,j) != 0) { jcur=j; break; }
    if(jcur == -1) continue;

    //normalize row by dividing by A(icur,jcur)
    T scale = One/A(icur,jcur);
    A(icur,jcur) = 1;
    for(j=jcur+1;j<A.n;j++) A(icur,j) *= scale;
    for(j=0;j<B.n;j++) B(icur,j) *= scale;

    //zero out columns above (icur,jcur)
    for(i=0;i<icur;i++) {
      if(A(i,jcur) == 0) continue;
      scale = A(i,jcur);
      A(i,jcur) = 0;
      for(j=jcur+1;j<A.n;j++) A(i,j) -= A(icur,j)*scale;
      for(j=0;j<B.n;j++) B(i,j) -= B(icur,j)*scale;
    }
  }
}

template <class T>
bool IsRowEchelon(const MatrixTemplate<T>& A)
{
  int icur=0;
  for(int j=0;j<A.n;j++) {
    //it's ok for column j to have an entry at icur, but none below
    if(A(icur,j) != 0) icur++;
    for(int i=icur+1;i<A.m;i++) if(A(i,j) != 0) return false;
  }
  return true;
}

template <class T>
bool IsReducedRowEchelon(const MatrixTemplate<T>& A)
{
  if(!IsRowEchelon(A)) return false;
  int lastj=-1;
  for(int i=0;i<A.m;i++) {
    //find first nonzero element
    int firstj=-1;
    for(int j=0;j<A.n;j++) if(A(i,j) != 0) { firstj=j; break; }
    if(firstj == -1) {
      lastj = INT_MAX; //make sure the rest of the rows are 0
    }
    else {
      if(firstj <= lastj) return false;
      lastj=firstj;
      //element must be 1
      if(A(i,firstj) != 1) return false;
      //column must be zeros except for that element
      for(int i2=0;i2<A.m;i2++) {
	if(i2 != i) if(A(i2,firstj) != 0) return false;
      }
    }
  }
  return true;
}





template <class T>
RowEchelon<T>::RowEchelon()
{}

template <class T>
RowEchelon<T>::RowEchelon(const MatrixT& A)
{
  set(A);
}

template <class T>
RowEchelon<T>::RowEchelon(const MatrixT& A,const VectorT& b)
{
  set(A,b);
}

template <class T>
RowEchelon<T>::RowEchelon(const MatrixT& A,const MatrixT& B)
{
  set(A,B);
}
  
template <class T>
void RowEchelon<T>::set(const MatrixT& A)
{
  MatrixT B;
  set(A,B);
}

template <class T>
void RowEchelon<T>::set(const MatrixT& A,const VectorT& b)
{
  Assert(b.n == A.m);
  R=A;
  EB.resize(A.m,1);
  EB.copyCol(0,b);
  RowEchelonDecompose(R,EB,Epsilon);
  Assert(IsRowEchelon(R));
  //ReduceRowEchelon(R,EB);
  //Assert(IsReducedRowEchelon(R));

  firstEntry.clear();
  calcFirstEntries();
}

template <class T>
void RowEchelon<T>::set(const MatrixT& A,const MatrixT& B)
{
  if(!B.isEmpty())
    Assert(A.m == B.m);
  R=A;
  EB=B;
  RowEchelonDecompose(R,EB,Epsilon);
  Assert(IsRowEchelon(R));
  //ReduceRowEchelon(R,EB);
  //Assert(IsReducedRowEchelon(R));

  firstEntry.clear();
  calcFirstEntries();
}

template <class T>
void RowEchelon<T>::backSub(VectorT& x) const
{
  Assert(EB.n == 1);
  Assert(EB.m == R.m);
  x.resize(R.n);
  VectorT b;
  EB.getColRef(0,b);

  Assert((int)firstEntry.size() == R.m+1);
  x.setZero();
  int m=R.m,n=R.n;
  for(int i=m-1;i>=0;i--) {
    VectorT ri; R.getRowRef(i,ri);
    //solve to set R*x = b[i]
    //calculate alpha = dot between rest of x[>ji] and R[i]
    int ji=firstEntry[i];
    if(ji == n) continue;
    int ji2=firstEntry[i+1];  //(i+1==m?n:firstEntry[i+1]);
    T alpha;
    if(ji2 == n) alpha = Zero;
    else {
      VectorT rji2; rji2.setRef(ri,ji2,1,R.n-ji2);
      VectorT xji2; xji2.setRef(x,ji2,1,R.n-ji2);
      alpha = xji2.dot(rji2);
    }
    x[ji] = (b[i]-alpha)/ri[ji];
  }
}

template <class T>
int RowEchelon<T>::getRank() const
{
  //search backwards for last zero row of r
  for(int i=R.m-1;i>=0;i--) {
    for(int j=R.n-1;j>=0;j--)
      if(R(i,j) != 0) return i+1;
  }
  return 0;
}

template <class T>
int RowEchelon<T>::getNull() const
{
  return R.n - getRank();
}

template <class T>
void RowEchelon<T>::getNullspace(MatrixT& N) const
{
  if(R.isEmpty()) {
    N.clear();
    return;
  }
  Assert((int)firstEntry.size() == R.m+1);

  int nullspace_dims=getNull();
  N.resize(R.n,nullspace_dims);

  //first get nullspace vectors from 0 to firstEntry[0]
  int i,j,numVecs=0;
  int m=R.m,n=R.n;
  for(j=0;j<firstEntry[0];j++) {
    N.setCol(numVecs,0); N(j,numVecs)=1;
    numVecs++;
  }
  for(i=0;i<m;i++) {
    //cancel out the i'th entry
    for(j=firstEntry[i]+1;j<firstEntry[i+1];j++) {
      if(numVecs >= N.n) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Num nullspace vectors "<<numVecs);
	LOG4CXX_INFO(KrisLibrary::logger(),"Found more nullspace vectors than found dims, row "<<i);
	LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(R));
      }
      Assert(numVecs < N.n);
      VectorT xn; N.getColRef(numVecs,xn);
      xn.setZero();
      xn[firstEntry[i]] = R(i,j);
      xn[j] = -R(i,firstEntry[i]);

      //cancel out all the entries prior to i 
      int isave=i;
      i--;
      for(;i>=0;i--) {
        VectorT ri; R.getRowRef(i,ri);
	//calculate alpha
	int ji=firstEntry[i];
	Assert(ji != n);
	int ji2=firstEntry[i+1]; //(i+1==m?n:firstEntry[i+1]);
	T alpha;
	if(ji2 == n) alpha = Zero;
        else {
          VectorT rji2; rji2.setRef(ri,ji2,1,R.n-ji2);
          VectorT xji2; xji2.setRef(xn,ji2,1,R.n-ji2);
          alpha = xji2.dot(rji2);
        }
	xn[ji] = -alpha/ri[ji];
      }
      i=isave;
      numVecs++;
    }
  }
  if(numVecs != nullspace_dims) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in counting rank in row-eschelon decomposition");
    LOG4CXX_INFO(KrisLibrary::logger(),"Num nullspace vectors "<<numVecs);
    LOG4CXX_INFO(KrisLibrary::logger(),MatrixPrinter(R));
  }
  Assert(numVecs == nullspace_dims);

  /*
  VectorT temp;
  for(int i=0;i<numVecs;i++) {
    VectorT xi; N.getColRef(i,xi);
    xi.print();
    R.mul(xi,temp);
    if(temp.maxAbsElement() > 1e-4) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Nullspace vector "<<i<<" not in null space!");
      xi.print();
      LOG4CXX_INFO(KrisLibrary::logger(),"Result = "); temp.print();
      KrisLibrary::loggerWait();
    }
  }
  */

  VectorT* N0 = new VectorT[nullspace_dims];
  for(int i=0;i<nullspace_dims;i++) N.getColRef(i,N0[i]);
  int num=OrthonormalBasis(N0,N0,nullspace_dims);
  Assert(num == nullspace_dims);
  delete [] N0;
}

template <class T>
void RowEchelon<T>::getAllSolutions(VectorT& x0,MatrixT& N) const
{
  getNullspace(N);

  VectorT* N0 = new VectorT[N.n];
  for(int i=0;i<N.n;i++) N.getColRef(i,N0[i]);
  backSub(x0);
  Orthogonalize(x0,N0,N.n);
  delete [] N0;
}

template <class T>
void RowEchelon<T>::calcFirstEntries()
{
  if(firstEntry.empty()) firstEntry.resize(R.m+1);
  Assert((int)firstEntry.size() == R.m+1);
  int i,j;
  for(i=0;i<R.m;i++) {
    for(j=0;j<R.n;j++) {
      if(R(i,j) != Zero)
	      break;
    }
    firstEntry[i] = j;
  }
  firstEntry[R.m] = R.n;

  int nullspace_dims = getNull();
  for(i=0;i<R.m;i++) {
    if(i > R.n-nullspace_dims) Assert(firstEntry[i] == R.n);
    else if(i > 0) Assert(firstEntry[i] > firstEntry[i-1]);
  }
}

template class RowEchelon<float>;
template class RowEchelon<double>;
//template class RowEchelon<Complex>;

template bool IsRowEchelon<float>(const MatrixTemplate<float>& A);
template bool IsRowEchelon<double>(const MatrixTemplate<double>& A);
template bool IsReducedRowEchelon<float>(const MatrixTemplate<float>& A);
template bool IsReducedRowEchelon<double>(const MatrixTemplate<double>& A);

} //namespace Math
