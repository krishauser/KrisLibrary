#include <KrisLibrary/Logger.h>
#include "LUDecomposition.h"
#include "backsubstitute.h"
#include "complex.h"
#include <errors.h>

namespace Math {

template <class T>
LUDecomposition<T>::LUDecomposition()
:zeroTolerance((T)1e-6)
{}

template <class T>
LUDecomposition<T>::LUDecomposition(const MatrixT& A)
:zeroTolerance((T)1e-6)
{
	if(!set(A)) Abort();
}

template <class T>
bool LUDecomposition<T>::set(const MatrixT& A)
{
  if(!A.isSquare())
    FatalError("Non-square matrix in LU decomposition");
  int n=A.n;

  bool res = true;
  int i,imax,j,k;
  double d,big,dum,temp;
  T sum;
  VectorTemplate<double> vv(n);					//vv stores the implicit scaling of each row.
  
  d=1.0;							//No row interchanges yet.
  for (i=0;i<n;i++) {				//Loop over rows to get the implicit scaling information.
    big=0.0; 
    for (j=0;j<n;j++)
      if ((temp=Abs(A(i,j))) > big) big=temp;
    if (big == 0.0)				//Is a singular matrix
      return false;
    //No nonzero largest element.
    vv[i]=Inv(big);				//Save the scaling.
  }
  P.resize(n);
  LU = A;
  for (j=0;j<n;j++) {				//This is the loop over columns of Crout's method.
    for (i=0;i<j;i++) {			//This is equation (2.3.12) except for i = j.
      sum=LU(i,j);
      for (k=0;k<i;k++)
	sum -= LU(i,k)*LU(k,j);
      LU(i,j)=sum;
    }
    big=0.0;					//Initialize for the search for largest pivot element.
    imax = -1;
    for (i=j;i<n;i++) {			//This is i = j of equation (2.3.12) and i = j+1 : : :N
      sum=LU(i,j);			//of equation (2.3.13).
      for (k=0;k<j;k++)
	sum -= LU(i,k)*LU(k,j);
      LU(i,j)=sum;
      if ( (dum=vv[i]*Abs(sum)) >= big) {
	//Is the figure of merit for the pivot better than the best so far?
	big=dum;
	imax=i;
      }
    }
    assert(imax >= 0);
    if (j != imax) {			//Do we need to interchange rows?
      //LOG4CXX_INFO(KrisLibrary::logger(),"interchanging rows "<< j<<", "<< imax);
      VectorT rowimax,rowj;
      LU.getRowRef(imax,rowimax);
      LU.getRowRef(j,rowj);
      rowimax.swapCopy(rowj);
      std::swap(vv[imax],vv[j]);  //Also interchange the scale factor.
      d = -(d);				//...and change the parity of d.
    }
    P[j]=imax;
    if (FuzzyZero(LU(j,j),zeroTolerance))
      {
	LU(j,j)=zeroTolerance;
	res = false;
      }
    //If the pivot element is zero the matrix is singular (at least to the precision of the
    //algorithm). For some applications on singular matrices, it is desirable to substitute
    //TINY for zero.
    if (j != n-1) { //Now, finally, divide by the pivot element.
      sum=Inv(LU(j,j));
      for (i=j+1;i<n;i++)
	LU(i,j) *= sum;
    }
  } //Go back for the next column in the reduction.
  //if(!res) LOG4CXX_WARN(KrisLibrary::logger(),"LUDecomposition(): Warning, near-singular matrix\n");
  return true;
}

template <class T>
void LUDecomposition<T>::backSub(const VectorT& b, VectorT& x) const
{
  if(!(LU.n == b.n))
    FatalError("Incompatible dimensions");
  
  x.copy(b);
  int i,ii=-1,ip,j;
  T sum;
  //PL back substitution
  for (i=0;i<LU.n;i++) {		//When ii is set to a positive value, it will become the
    ip=P[i];			  	//index of the first nonvanishing element of b. We now
    sum=x[ip];				//do the forward substitution, equation (2.3.6). The
    x[ip]=x[i];				//only new wrinkle is to unscramble the permutation
    if (ii!=-1)	{				//as we go.
      for (j=ii;j<i;j++)
	sum -= LU(i,j)*x[j];
    }
    else if (sum!=0)			//A nonzero element was encountered, so from now on we
      ii=i;          //will have to do the sums in the loop above.
    x[i]=sum;			
  }
  //U back substitution
  for (i=LU.n-1;i>=0;i--) {
    sum=x[i];
    for (j=i+1;j<LU.n;j++)
      sum -= LU(i,j)*x[j];
    x[i]=sum/LU(i,i);
  }
}

template <class T>
void LUDecomposition<T>::LBackSub(const VectorT& b, VectorT& x) const
{
  L1BackSubstitute(LU,b,x);
}

template <class T>
void LUDecomposition<T>::UBackSub(const VectorT& b, VectorT& x) const
{
  UBackSubstitute(LU,b,x);
}

template <class T>
void LUDecomposition<T>::PBackSub(const VectorT& b, VectorT& x) const
{
  x.resize(b.n);
  for(int i=0;i<b.n;i++)
    x[i] = b[P[i]];
}

template <class T>
void LUDecomposition<T>::getInverse(MatrixT& Ainv) const
{
  Assert(LU.isSquare());
  int n=LU.n;
  Ainv.resize(n,n);
  VectorT temp(n,(T)0),coli;
  for(int i=0;i<n;i++) {
    temp(i) = 1;
    Ainv.getColRef(i,coli);
    backSub(temp,coli);
    temp(i) = 0;
  }
}

template <class T>
void LUDecomposition<T>::getL(MatrixT& L) const
{
  L.resize(LU.m,LU.n);
  for(int i=0;i<LU.m;i++) {
    for(int j=0;j<i;j++)
      L(i,j) = LU(i,j);
    L(i,i) = 1;
    for(int j=i+1;j<LU.n;j++)
      L(i,j) = 0;
  }
}

template <class T>
void LUDecomposition<T>::getU(MatrixT& U) const
{
  U.resize(LU.m,LU.n);
  for(int i=0;i<LU.m;i++) {
    for(int j=0;j<i;j++)
      U(i,j) = 0;
    for(int j=i;j<LU.n;j++)
      U(i,j) = LU(i,j);
  }
}

template class LUDecomposition<float>;
template class LUDecomposition<double>;
template class LUDecomposition<Complex>;


} //namespace Math
