#include "LRSInterface.h"
#if HAVE_LRS
extern "C" {
#include <lrslib.h>
}
#include <debug.h>

bool LRSInterface::initialized = false;

LRSInterface::LRSInterface()
  :nonnegative(false),precisionL(256)
{
	if(!initialized) {
		if(!lrs_init("My LRS Interface")) {
			ReportError("Unable to initialize LRS");
			abort();
		}
		initialized=true;
	}
}

void LRSInterface::SolveH2VProblem(const Array2D<Value>& A, std::vector<ValueVector>& pts)
{
  Array2D<Value> Aeq;
  SolveH2VProblem(A,Aeq,pts);
}


void LRSInterface::SolveH2VProblem(const Array2D<Value>& Aineq, const Array2D<Value>& Aeq, std::vector<ValueVector>& pts)
{
	lrs_dat* Q;
	lrs_dic* P;
	lrs_mp_vector output;	/* one line of output:ray,vertex,facet,linearity */
	lrs_mp_matrix Lin;    /* holds input linearities if any are found      */

	Q = lrs_alloc_dat("temp");
	if(Aeq.n != 0 && Aineq.n != 0)
	  assert(Aeq.n == Aineq.n);
	Q->m = Aineq.m+Aeq.m;
	Q->n = Max(Aineq.n,Aeq.n);
	if(nonnegative) {
	  if (Aeq.m==0) Q->nonnegative = TRUE;
	  else {
	    Q->m+=Q->n-1;
	  }
	}
	else Q->nonnegative = FALSE;
	P = lrs_alloc_dic(Q);
	output = lrs_alloc_mp_vector (Q->n);

	long *num, *denom;
	num = new long[Q->n];
	denom = new long[Q->n];
	int row=1;
	for(int i=0;i<Aineq.m;i++,row++) {
		for(int j=0;j<Aineq.n;j++) {
			num[j] = Aineq(i,j).num;
			denom[j] = Aineq(i,j).den;
		}
		lrs_set_row(P,Q,row,num,denom,GE);
	}
	for(int i=0;i<Aeq.m;i++,row++) {
		for(int j=0;j<Aeq.n;j++) {
			num[j] = Aeq(i,j).num;
			denom[j] = Aeq(i,j).den;
		}
		lrs_set_row(P,Q,row,num,denom,EQ);
	}
	if(nonnegative && Aeq.m != 0) {
	  //manual inequality constraints
	  for(int j=0;j<Q->n;j++) denom[j]=1;
	  for(int i=1;i<Q->n;i++,row++) {
	    for(int j=0;j<Q->n;j++) {
	      num[j]=(i==j?1:0);
	    }
	    lrs_set_row(P,Q,row,num,denom,GE);
	  }
	}
	delete [] num;
	delete [] denom;

	if (!lrs_getfirstbasis (&P, Q, &Lin, FALSE)) {
		ReportError("Unable to get first basis");
		abort();
	}

	printf("Got first basis!\n");

	//extract output
	do {
		lrs_mp n,d;
		for(int col=0; col<=P->d; col++)
			if (lrs_getsolution (P, Q, output, col)) {
				ValueVector v(Q->n);
				for(int j=0;j<Q->n;j++) {
					copy(n,output[j]);
					copy(d,P->det);
					reduce(n,d);
					v[j].num = mptoi(n);
					v[j].den = mptoi(d);
				}
				pts.push_back(v);
			}
    }
    while (lrs_getnextbasis (&P, Q, FALSE));

	lrs_clear_mp_vector (output, Q->n);
	lrs_free_dic(P,Q);
	lrs_free_dat(Q);
}

void LRSInterface::SolveH2VProblem(const Matrix& A, std::vector<Vector>& pts)
{
  Matrix Aeq;
  SolveH2VProblem(A,Aeq,pts);
}

void LRSInterface::SolveH2VProblem(const Matrix& Aineq, const Matrix& Aeq, std::vector<Vector>& pts)
{
	lrs_dat* Q;
	lrs_dic* P;
	lrs_mp_vector output;	/* one line of output:ray,vertex,facet,linearity */
	lrs_mp_matrix Lin;    /* holds input linearities if any are found      */
  int i,j;

	Q = lrs_alloc_dat("temp");
	if(Aeq.n != 0 && Aineq.n != 0)
	  assert(Aeq.n == Aineq.n);
	Q->m = Aineq.m+Aeq.m;
	Q->n = Max(Aineq.n,Aeq.n);
	if(nonnegative) {
	  if (Aeq.m==0) Q->nonnegative = TRUE;
	  else {
	    Q->m+=Q->n-1;
	  }
	}
	else Q->nonnegative = FALSE;
	P = lrs_alloc_dic(Q);
	output = lrs_alloc_mp_vector (Q->n);

	long *num, *denom;
	num = new long[Q->n];
	denom = new long[Q->n];
	int row=1;
	double precision = double(precisionL);
	for(i=0;i<Aineq.m;i++,row++) {
		for(j=0;j<Aineq.n;j++) {
			num[j] = (long)(Aineq(i,j) * precision);
			denom[j] = 1;
		}
		lrs_set_row(P,Q,row,num,denom,GE);
	}
	for(i=0;i<Aeq.m;i++,row++) {
		for(j=0;j<Aeq.n;j++) {
			num[j] = (long)(Aeq(i,j) * precision);
			denom[j] = 1;
		}
		lrs_set_row(P,Q,row,num,denom,EQ);
	}
	if(nonnegative && Aeq.m != 0) {
	  //manual inequality constraints
	  for(j=0;j<Q->n;j++) denom[j]=1;
	  for(i=1;i<Q->n;i++,row++) {
	    for(j=0;j<Q->n;j++) {
	      num[j]=(i==j?1:0);
	    }
	    lrs_set_row(P,Q,row,num,denom,GE);
	  }
	}
	delete [] num;
	delete [] denom;

	if (!lrs_getfirstbasis (&P, Q, &Lin, TRUE)) {
		ReportError("Unable to get first basis");
		abort();
	}

	//extract output
	i=0;
	do {
		for(int col=0; col<=P->d; col++) {
			if (lrs_getsolution (P, Q, output, col)) {
			  //printf("Get solution\n");
				Vector v(Q->n);
        double x;
        if(zero(output[0])) {
				  for(int j=0;j<Q->n;j++) {
				    //pmp("",P->det); printf("\n");
				    rattodouble(output[j],P->det,&x);
				    v[j]=(Real)x;
				  }
        }
        else {
    			for(int j=0;j<Q->n;j++) {
				    //pmp("",P->det); printf("\n");
				    rattodouble(output[j],output[0],&x);
				    v[j]=(Real)x;
				  }
        }
				pts.push_back(v);
			}
		}
		i++;
  }
  while (lrs_getnextbasis (&P, Q, FALSE));

	printf("Done after %d iterations!!\n",i);

	lrs_clear_mp_vector (output, Q->n);
	lrs_free_dic(P,Q);
	lrs_free_dat(Q);
}

#else

using namespace std;

bool LRSInterface::initialized = false;

LRSInterface::LRSInterface()
{
  cerr<<"Warning: LRS not defined"<<endl;
}

void LRSInterface::SolveH2VProblem(const Array2D<Value>& A, std::vector<ValueVector>& pts)
{
  cerr<<"Warning: LRS not defined"<<endl;
}

void LRSInterface::SolveH2VProblem(const Array2D<Value>& Aineq, const Array2D<Value>& Aeq, std::vector<ValueVector>& pts)
{
  cerr<<"Warning: LRS not defined"<<endl;
}

void LRSInterface::SolveH2VProblem(const Matrix& A, std::vector<Vector>& pts)
{
  cerr<<"Warning: LRS not defined"<<endl;
}

void LRSInterface::SolveH2VProblem(const Matrix& Aineq, const Matrix& Aeq, std::vector<Vector>& pts)
{
  cerr<<"Warning: LRS not defined"<<endl;
}

#endif // HAVE_LRS
