#include <KrisLibrary/Logger.h>
#include "LCP.h"
#include <errors.h>
#include <sstream>
#include <iostream>
using namespace Optimization;
using namespace std;

LemkeLCP::LemkeLCP(const Matrix& M,const Vector& q)
{
  verbose = 1;
  Assert(M.m == M.n);
  Assert(M.m == q.n);
  dictionary.resize(M.m,M.n+2);
  dictionary.copySubMatrix(0,1,M);
  Vector qdic,z0dic;
  dictionary.getColRef(0,qdic);
  qdic.copy(q);
  dictionary.getColRef(M.n+1,z0dic);
  z0dic.set(One);

  basic.resize(M.m);
  nonbasic.resize(M.n+2);
  for(int i=0;i<M.m;i++) basic[i]=WToVar(i);
  for(int i=0;i<M.m;i++) nonbasic[i+1]=ZToVar(i);
  nonbasic[0] = constant;
  nonbasic[M.n+1] = z0;
}

string LemkeLCP::VarName(int var) const
{
  stringstream ss;
  if(IsVarW(var)) ss<<"W"<<VarToW(var)+1;
  else if(IsVarZ(var)) ss<<"Z"<<VarToZ(var)+1;
  else if(var==z0) ss<<"Z0";
  else if(var==constant) ss<<"const";
  else ss<<"InvalidVariable!("<<var<<")";
  return ss.str();
}

bool LemkeLCP::Solve()
{
  if(verbose >= 3) Print(cout);
  int z0index = InitialPivot();
  if(verbose >= 3) Print(cout);
  if(z0index < 0) {
    return true;
  }
  //last column is the swapped-out varialbe
  int lastLeftVar = nonbasic[dictionary.n-1];
  Assert(lastLeftVar >= 0);
  int maxIters = dictionary.m*dictionary.m;
  for(int iters=0;iters<maxIters;iters++) {
    int newEnterVar = (IsVarW(lastLeftVar) ? WToZ(lastLeftVar) : ZToW(lastLeftVar));
    Assert(newEnterVar >= 0);

    int newEnterIndex = -1;
    for(size_t i=0;i<nonbasic.size();i++)
      if(nonbasic[i] == newEnterVar) {
	newEnterIndex=(int)i;
	break;
      }
    Assert(newEnterIndex > 0);

    int newLeaveIndex = PickPivot(newEnterIndex);
    if(newLeaveIndex < 0) {
      if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"LemkeLCP: couldn't find a valid pivot variable for "<<VarName(nonbasic[newEnterIndex])<<" to enter");
      return false;
    }
    int newLeaveVar = basic[newLeaveIndex];
    if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"Picked pivot "<<VarName(newLeaveVar));
    if(!Pivot(newEnterIndex,newLeaveIndex)) {
      if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"LemkeLCP: couldn't pivot variables "<<VarName(nonbasic[newEnterIndex])<<" to enter, "<<VarName(basic[newLeaveIndex])<<" to leave");
      return false;
    }
    if(verbose >= 3) Print(cout);
    Assert(newLeaveVar != constant);
    if(newLeaveVar == z0) {
      return true;
    }
    lastLeftVar = newLeaveVar;
  }
  if(verbose >= 1) LOG4CXX_ERROR(KrisLibrary::logger(),"LemkeLCP: error, maximum of "<<dictionary.m<<" iterations reached");
  return false;
}

void LemkeLCP::GetW(Vector& w) const
{
  w.resize(dictionary.m,Zero);
  for(int i=0;i<w.n;i++) {
    if(IsVarW(basic[i])) 
      w(VarToW(basic[i])) = dictionary(i,0);
  }
}

void LemkeLCP::GetZ(Vector& z) const
{
  z.resize(dictionary.m,Zero);
  for(int i=0;i<z.n;i++) {
    if(IsVarZ(basic[i])) 
      z(VarToZ(basic[i])) = dictionary(i,0);
  }
}

int LemkeLCP::InitialPivot()
{
  //add z0 to the basic variables
  int imin=-1;
  Real min=Inf;
  for(int i=0;i<dictionary.m;i++) {
    if(dictionary(i,0) < min) {
      min = dictionary(i,0);
      imin=i;
    }
    else if(dictionary(i,0) == min) {
      if(verbose >= 1) LOG4CXX_ERROR(KrisLibrary::logger(),"LemkeLCP: Warning, degeneracy!");
    }
  }
  if(min > 0) return -1;

  //make imin leave, z0 enter
  Assert(nonbasic[dictionary.n-1]==z0);
  bool res=Pivot(dictionary.n-1,imin);
  Assert(res==true);

  //must be feasible
  for(int i=0;i<dictionary.m;i++)
    Assert(dictionary(i,0) >= Zero);
  return imin;
}

int LemkeLCP::PickPivot(int E) const
{
  Assert(nonbasic[E] != constant);
  for(int i=0;i<dictionary.m;i++)
    Assert(dictionary(i,0) >= Zero);

  //need to pick entering variable (E) such that the constants do not become negative
  //
  //From below:
  //nonbasic[E] = (basic[L]-sum_{j!=E} d[L,j]*nonbasic[j])/d[L,E]
  //d'[L,0] = -d[L,0]/d[L,E] > 0 => d[L,E] must be negative (since all constants must remain positive)
  //
  //d'[i,0] = d[i,0] - d[L,0]*d[i,E]/d[L,E] > 0
  //
  //from the L that satisfy these conditions, pick the one with the greatest resulting constants?
  int maxL = -1;
  Real max=-Inf;
  Real entry,sum;
  for(int L=0;L<dictionary.m;L++) {
    if(Abs(dictionary(L,E)) == Zero) continue;
    bool valid=true;
    double denom = 1.0/dictionary(L,E);
    double scale = dictionary(L,0)*denom;
    sum=0;
    for(int i=0;i<dictionary.m;i++) {
      if(i == L) 
	entry = -scale;
      else
	entry = dictionary(i,0) - scale*dictionary(i,E);
      if(entry < 0) {
	valid = false;
	break;
      }
      sum += entry;
    }
    if(valid) {
      if(sum > max) {
	max = sum;
	maxL = L;
      }
      else if(sum == max) {
	if(verbose >= 1) LOG4CXX_ERROR(KrisLibrary::logger(),"LemkeLCP: Warning, degeneracy");
      }
    }
  }
  return maxL;
}

bool LemkeLCP::Pivot(int E,int L)
{
  if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"LemkeLCP: Pivoting "<<VarName(nonbasic[E])<<", "<<VarName(basic[L]));
  Assert(nonbasic[E] != constant);
  Assert(basic[L] != constant);
  //nonbasic variable indexed by E (enter) gets moved to the
  //basic variable indexed by L (leave) (and vice versa)
  //
  //basic[L] = sum_{j!=E} d[L,j]*nonbasic[j] + d[L,E]*nonbasic[E]
  //=>nonbasic[E] = (basic[L]-sum_{j!=E} d[L,j]*nonbasic[j])/d[L,E]
  //
  //Replacing this in the dictionary, we get
  //=>basic[i] = sum_{j!=E} d[i,j]*nonbasic[j] + d[i,E]*nonbasic[E]
  //           = sum_{j!=E} d[i,j]*nonbasic[j] + d[i,E]/d[L,E]*basic[L]-sum_{j!=E} d[L,j]*d[i,E]/d[L,E]*nonbasic[j]
  //           = sum_{j!=E} (d[i,j] - d[L,j]*d[i,E]/d[L,E])*nonbasic[j] + d[i,E]/d[L,E]*basic[L]
  if(Abs(dictionary(L,E)) == Zero) return false;
  if(FuzzyZero(dictionary(L,E))) { LOG4CXX_INFO(KrisLibrary::logger(),"LemkeLCP: pivot element is very small! "<<dictionary(L,E)); }
  //replace all rows referencing the Eing variable with the new 
  double denom=1.0/dictionary(L,E);
  for(int i=0;i<dictionary.m;i++) {
    if(i != L) {
      double scale = dictionary(i,E)*denom;
      for(int j=0;j<dictionary.n;j++) {
	if(j != E) 
	  dictionary(i,j) -= scale*dictionary(L,j);
	else //gets the unit on the old basic variable
	  dictionary(i,j) = scale;
      }
      if(!(dictionary(i,0) >= 0)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"LemkeLCP: possible numerical error: "<<dictionary(i,0)<<" < 0");
	Assert(FuzzyZero(dictionary(i,0)));
	dictionary(i,0) = 0;
      }
    }
  }
  //do row L last (it's needed for all other rows)
  for(int j=0;j<dictionary.n;j++) {
    if(j != E)
      dictionary(L,j) = -dictionary(L,j)*denom;
    else 
      dictionary(L,j) = denom;
  }
  Assert(dictionary(L,0) >= 0);
  swap(basic[L],nonbasic[E]);
  return true;
}

void LemkeLCP::Print(ostream& out) const
{
  out<<"   ";
  for(size_t i=0;i<nonbasic.size();i++)
    out<<VarName(nonbasic[i])<<"    ";
  out<<endl;
  for(int i=0;i<dictionary.m;i++) {
    out<<VarName(basic[i])<<" ";
    for(int j=0;j<dictionary.n;j++)
      out<<dictionary(i,j)<<" ";
    out<<endl;
  }
}

namespace Optimization {

bool IterativeLCP(const Matrix& M,const Vector& q,Vector& w,Vector& z,int& maxIters,Real tol)
{
  Assert(M.m == M.n);
  Assert(M.m == q.n);
  if(z.empty()) z.resize(M.m,Zero);
  if(w.empty()) w.resize(M.m);
  for(int i=0;i<M.n;i++)
    Assert(M(i,i) != Zero);

  //iteratively solve/project components of z onto 0
  //w is a temporary
  for(int iters=0;iters<maxIters;iters++) {
    M.mul(z,w);
    w += q;
    if(w.minElement() > -tol && Abs(z.dot(w)) < tol) {
      maxIters = iters;
      return true;
    }
    //z^t ( Mz+q ) = 0
    for(int i=0;i<M.m;i++) {
      //w(i) = sum_{j!=i}M(i,j)z(j) + M(i,i)z(i) + q(i)
      //=> z(i) = (-q(i) - sum_{j!=i}M(i,j)z(j))/M(i,i)
      Real sum = -q(i);
      for(int j=0;j<M.n;j++)
	if(i != j) sum -= M(i,j)*z(j);
      z(i) = Max(sum/M(i,i),Zero);
    }
  }
  return false;
}

} //namespace Optimization
