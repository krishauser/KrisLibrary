#include <KrisLibrary/Logger.h>
#include "backsubstitute.h"
#include "complex.h"
#include "MatrixPrinter.h"
#include <errors.h>
using namespace std;

namespace Math {

const static Real kBackSubZeroTolerance = (Real)1e-4;

template <class T>
bool UBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
	T aii,sum;
	for(int i=n-1; i>=0; i--) {
		aii=a(i,i);
		sum=b[i];
		for(int j=i+1; j<n; j++)
			sum-=a(i,j)*x[j];
		if(aii == 0) {
		  if(!FuzzyZero(sum,(T)kBackSubZeroTolerance)) {
		    //LOG4CXX_ERROR(KrisLibrary::logger(),"UBackSubstitute: dividing by zero: "<<sum<<"/"<<aii);
		    return false;
		  }
		  x[i]=0;
		}
		else
		  x[i]=sum/aii;
	}
	return true;
}

// If A is lower triangular nxn, solves Ax=b
template <class T>
bool LBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
	T aii,sum;
	for(int i=0; i<n; i++) {
		aii=a(i,i);
		sum=b[i];
		for(int j=0; j<i; j++)
			sum-=a(i,j)*x[j];
		if(aii == 0) {
		  if(!FuzzyZero(sum,(T)kBackSubZeroTolerance)) {
		    //LOG4CXX_ERROR(KrisLibrary::logger(),"LBackSubstitute: dividing by zero: "<<sum<<"/"<<aii);
		    //LOG4CXX_ERROR(KrisLibrary::logger(),MatrixPrinter(a));
		    return false;
		  }
		  x[i]=0;
		}
		else
		  x[i]=sum/aii;
	}
	return true;
}

// If A is lower triangular nxn, solves A^t*x=b
template <class T>
bool LtBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{ 
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
 	T aii,sum;
	for(int i=n-1; i>=0; i--) {
		aii=a(i,i);
		sum=b[i];
		for(int j=i+1; j<n; j++)
			sum-=a(j,i)*x[j];
		if(aii == 0) {
		  if(!FuzzyZero(sum,(T)kBackSubZeroTolerance)) {
		    //LOG4CXX_ERROR(KrisLibrary::logger(),"LtBackSubstitute: dividing by zero: "<<sum<<"/"<<aii);
		    return false;
		  }
		  x[i]=0;
		}
		else
		  x[i]=sum/aii;
	}
	return true;
}

template <class T>
void U1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{ 
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
  T sum;
	for(int i=n-1; i>=0; i--) {
		sum=b(i);
		for(int j=i+1; j<n; j++)
			sum-=a(i,j)*x[j];
    x[i]=sum;
	}
}

template <class T>
void L1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
	T sum;
	for(int i=0; i<n; i++) {
		sum=b[i];
		for(int j=0; j<i; j++)
			sum-=a(i,j)*x[j];
		x[i]=sum;
	}
}

template <class T>
void Lt1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
  T sum;
	for(int i=n-1; i>=0; i--) {
		sum=b[i];
		for(int j=i+1; j<n; j++)
			sum-=a(j,i)*x[j];
		x[i]=sum;
	}
}

template <class T>
inline bool UBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    if(!UBackSubstitute(a,bi,xi)) return false;
  }
  return true;
}
template <class T>
inline bool LBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    if(!LBackSubstitute(a,bi,xi)) return false;
  }
  return true;
}
template <class T>
inline bool LtBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    if(!LtBackSubstitute(a,bi,xi)) return false;
  }
  return true;
}
template <class T>
inline void U1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    U1BackSubstitute<T>(a,bi,xi);
  }
}
template <class T>
inline void L1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    L1BackSubstitute(a,bi,xi);
  }
}
template <class T>
inline void Lt1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    Lt1BackSubstitute(a,bi,xi);
  }
}

#define DEFINEBACKSUBSTITUTE(T) \
template bool UBackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template bool LBackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template bool LtBackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template void U1BackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template void L1BackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template void Lt1BackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template bool UBackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template bool LBackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template bool LtBackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template void U1BackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template void L1BackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template void Lt1BackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);

DEFINEBACKSUBSTITUTE(float);
DEFINEBACKSUBSTITUTE(double);
DEFINEBACKSUBSTITUTE(Complex);

} //namespace Math
