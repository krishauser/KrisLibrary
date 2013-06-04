#include "Polytope.h"
#include <iostream>
#include "LRSInterface.h"
#include "Timer.h"
#include <iostream>
using namespace std;
using namespace Geometry;


bool HPolytope::Contains(const Vector& x) const
{
	Vector y;
	A.mul(x,y);
	for(int i=0;i<y.n;i++) {
		if(Equality(i)) {
			if(!FuzzyEquals(y(i)+b(i),Zero)) return false;
		}
		else {
			if(y(i)+b(i) < Zero) return false;
		}
	}
	if(positiveX) {
	  for(int i=0;i<x.n;i++) if(x(i) < Zero) return false;
	}
	return true;
}

int HPolytope::NumEqualities() const
{
	int n=0;
	for(size_t i=0;i<equality.size();i++)
		if(equality[i]) n++;
	return n;
}

int HPolytope::NumInequalities() const
{
	if(!HasEqualities()) return NumConstraints();
	int n=0;
	for(size_t i=0;i<equality.size();i++)
		if(!equality[i]) n++;
	return n;
}

void HPolytope::Print(std::ostream& out)  const
{
  assert(b.n == A.m);
  for(int i=0;i<b.n;i++) {
    out<<"["<<b(i)<<"] + [";
    for(int j=0;j<A.n;j++) {
      out<<A(i,j)<<" ";
    }
    if(Equality(i))
      out<<"].x = 0"<<endl;
    else
      out<<"].x >= 0"<<endl;
  }
}

void HPolytope::GetVPolytope(VPolytope& v)
{
  Timer timer;
  printf("Doing vertex enumeration\n");

	LRSInterface lrs;
	assert(b.n == A.m);
	lrs.nonnegative = positiveX;
	std::vector<Vector> pts;
	if(HasEqualities()) {
    assert((int)equality.size() == A.m);
	  Matrix Aineq_LRS,Aeq_LRS;
	  int numEq = NumEqualities();
	  int numIneq = NumInequalities();
	  if(numEq) Aeq_LRS.resize(numEq,A.n+1);
	  if(numIneq) Aineq_LRS.resize(numIneq,A.n+1);
	  numEq=numIneq=0;
	  for(int i=0;i<A.m;i++) {
	    if(equality[i]) {
	      Aeq_LRS(numEq,0) = b(i);
	      for(int j=0;j<A.n;j++)
		      Aeq_LRS(numEq,j+1) = A(i,j);
	      numEq++;
	    }
	    else {
	      Aineq_LRS(numIneq,0) = b(i);
	      for(int j=0;j<A.n;j++)
		      Aineq_LRS(numIneq,j+1) = A(i,j);
	      numIneq++;
	    }
	  }
    printf("Constructed matrices (%f secs)\n",timer.ElapsedTime());
    lrs.SolveH2VProblem(Aineq_LRS,Aeq_LRS,pts);
	}
	else {
	  Matrix A_LRS(A.m,A.n+1);
	  for(int i=0;i<A.m;i++) {
	    A_LRS(i,0) = b(i);
	    for(int j=0;j<A.n;j++)
	      A_LRS(i,j+1) = A(i,j);
	  }
    printf("Constructed matrices (%f secs)\n",timer.ElapsedTime());
	  lrs.SolveH2VProblem(A_LRS,pts);
	}
	printf("%d points\n", pts.size());

	int np=0,nr=0;
	int d = Dimension();
	for(size_t i=0;i<pts.size();i++) {
	  //cout<<pts[i]<<endl;
		if(pts[i][0] == 0) nr++;
		else np++;
	}
	if(np) v.points.resize(np,d);
	if(nr) v.rays.resize(nr,d);

	np=nr=0;
	for(size_t i=0;i<pts.size();i++) {
		if(pts[i][0] == 0) {
			assert(pts[i].n==d+1);
			for(int j=0;j<d;j++)
				v.rays(nr,j) = pts[i][j+1];
			nr++;
		}
		else {
      assert(pts[i][0]==1);
			assert(pts[i].n==d+1);
			for(int j=0;j<d;j++)
				v.points(np,j) = pts[i][j+1];
			np++;
		}
	}
  printf("Done (%f secs)\n",timer.ElapsedTime());
}

Real VPolytope::Maximum(const Vector& d) const
{
	Vector y;
	rays.mul(d,y);
	if(y.maxElement() > Zero) return Inf;
	points.mul(d,y);
	return y.maxElement();
}

Real VPolytope::Minimum(const Vector& d) const
{
	Vector y;
	rays.mul(d,y);
	if(y.minElement() < Zero) return -Inf;
	points.mul(d,y);
	return y.minElement();
}
