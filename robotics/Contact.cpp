#include "Contact.h"
#include <math/complex.h>
#include <iostream>
#include <math/random.h>
#include <math3d/basis.h>
using namespace std;

bool ContactPoint::isValidForce(const Vector3& f) const
{
  Real fn = dot(f,n);
  if(fn < Zero) return false;
  Vector3 ftang(f);
  ftang.madd(n,-fn);
  return ftang.normSquared() <= Sqr(kFriction)*Sqr(fn);
}


Real ContactPoint::minFriction(const Vector3& f) const
{
  Real fn = dot(f,n);
  Vector3 ftang(f);
  ftang.madd(n,-fn);
  Real ft=ftang.norm();
  if(ft == 0) return 0;
  return ft/fn;
}

void ContactPoint::quadraticForm(Matrix3& A,Vector3& b,Real& c) const
{
  b.setZero();
  c = 0;
  A.setOuterProduct(n,n);
  A *= (One+Sqr(kFriction));
  for(int i=0;i<3;i++) A(i,i) -= One;

  /*
  TEST
  Vector3 fm(Rand(),Rand(),Rand());
  Real fn = dot(fm,n);
  Real eval = (One+Sqr(kFriction))*Sqr(fn) - fm.normSquared();
  Assert(FuzzyEquals(eval,dot(fm,A*fm)+dot(b,fm)+c));
  */
}

bool ContactPoint2D::isValidForce(const Vector2& f) const
{
  Real fn = dot(f,n);
  if(fn < Zero) return false;
  Vector2 ftang(f);
  ftang.madd(n,-fn);
  return ftang.normSquared() <= Sqr(kFriction)*Sqr(fn);
}


Real ContactPoint2D::minFriction(const Vector2& f) const
{
  Real fn = dot(f,n);
  Vector2 ftang(f);
  ftang.madd(n,-fn);
  Real ft=ftang.norm();
  if(ft == 0) return 0;
  return ft/fn;
}

void ContactPoint2D::quadraticForm(Matrix2& A,Vector2& b,Real& c) const
{
  b.setZero();
  c = 0;
  A.setOuterProduct(n,n);
  A *= (One+Sqr(kFriction));
  for(int i=0;i<2;i++) A(i,i) -= One;

  /*
  TEST
  Vector2 fm(Rand(),Rand());
  Real fn = dot(fm,n);
  Real eval = (One+Sqr(kFriction))*Sqr(fn) - fm.normSquared();
  Assert(FuzzyEquals(eval,dot(fm,A*fm)+dot(b,fm)+c));
  */
}

void ContactFormation::flatten(std::vector<int>& flatlinks,std::vector<ContactPoint>& cps) const
{
  cps.resize(0);
  size_t ncp = 0;
  for(size_t i=0;i<contacts.size();i++)
    ncp += contacts[i].size();
  flatlinks.resize(ncp);
  cps.resize(ncp);
  size_t k=0;
  for(size_t i=0;i<contacts.size();i++) {
    fill(flatlinks.begin()+k,flatlinks.begin()+k+contacts[i].size(),links[i]);
    copy(contacts[i].begin(),contacts[i].end(),cps.begin()+k);
    k += contacts[i].size();
  }
}

void ContactFormation::flatten(std::vector<int>& flatlinks,std::vector<ContactPoint>& cps,std::vector<int>& flattargets) const
{
  cps.resize(0);
  size_t ncp = 0;
  for(size_t i=0;i<contacts.size();i++)
    ncp += contacts[i].size();
  flatlinks.resize(ncp);
  flattargets.resize(ncp);
  cps.resize(ncp);
  size_t k=0;
  for(size_t i=0;i<contacts.size();i++) {
    fill(flatlinks.begin()+k,flatlinks.begin()+k+contacts[i].size(),links[i]);
    int target = (targets.empty() ? -1 : targets[i]);
    fill(flattargets.begin()+k,flattargets.begin()+k+contacts[i].size(),target);
    copy(contacts[i].begin(),contacts[i].end(),cps.begin()+k);
    k += contacts[i].size();
  }
}



void FrictionConePolygon::set(int k,const Vector3& n,Real kFriction)
{
  Assert(k >= 3);
  Assert(kFriction > 0);
  Vector3 xb,yb;
  GetCanonicalBasis(n,xb,yb);
  /*
  Assert(dot(xb,cross(yb,n)) > 0);
  Assert(dot(yb,cross(n,xb)) > 0);
  Assert(dot(n,cross(xb,yb)) > 0);
  */
  Complex x(kFriction,0),dx;
  dx.setPolar(1,TwoPi/k);
  edges.resize(k);
  planes.resize(k);
  for(int i=0;i<k;i++) {
    edges[i] = x.x*xb + x.y*yb + n;
    x *= dx;
  }
  for(int i=0;i<k;i++) {
    planes[i].setCross(edges[i],edges[(i+1)%k]);
    planes[i].inplaceNormalize();
    //Assert(dot(planes[i],n) > 0);
  }
}

bool FrictionConePolygon::contains(const Vector3& f) const
{
  for(size_t i=0;i<planes.size();i++)
    if(dot(f,planes[i]) < 0) return false;
  return true;
}

bool FrictionConePolygon::onBoundary(const Vector3& f) const
{
  bool on=false;
  for(size_t i=0;i<planes.size();i++) {
    if(dot(f,planes[i]) < 0) return false;
    if(dot(f,planes[i]) == 0) on=true;
  }
  return on;
}



void FrictionToFrictionlessContacts(const vector<ContactPoint>& c1,int k,vector<ContactPoint>& c2)
{
  Assert(k >= 3);
  int n=0;
  for(size_t i=0;i<c1.size();i++) {
    if(c1[i].kFriction == 0) n++;
    else if(c1[i].kFriction > 1e6) {
      cout<<"FrictionToFrictionlessContacts: Warning, be careful with the use of this function, behavior is not always correct in kFriction = inf case"<<endl;
      getchar();
    }
    else n+=k;
  }

  c2.resize(n);
  n=0;
  FrictionConePolygon fc;
  for(size_t i=0;i<c1.size();i++) {
    if(c1[i].kFriction == 0) {
      c2[n] = c1[i];
      n++;
    }
    else {
      fc.set(k,c1[i].n,c1[i].kFriction);
      for(int j=0;j<k;j++) {
	c2[n].x = c1[i].x;
	c2[n].n = fc.edges[j];
	c2[n].kFriction = 0;
	n++;
      }
    }
  }
}


void GetFrictionConePlanes(const ContactPoint& c,int nFrictionConeEdges,Matrix& A)
{
  A.resize(nFrictionConeEdges,3);

  FrictionConePolygon fc;
  fc.set(nFrictionConeEdges,c.n,c.kFriction);
  for(int i=0;i<nFrictionConeEdges;i++) {
    A(i,0) = -fc.planes[i].x;
    A(i,1) = -fc.planes[i].y;
    A(i,2) = -fc.planes[i].z;
  }
}


void GetFrictionConePlanes(const vector<ContactPoint>& c,int nFrictionConeEdges,Matrix& A)
{
  int nc=(int)c.size();
  A.resize(nFrictionConeEdges*nc,nc*3,Zero);

  int m=0;
  for(size_t p=0;p<c.size();p++) {
    FrictionConePolygon fc;
    fc.set(nFrictionConeEdges,c[p].n,c[p].kFriction);
    for(int i=0;i<nFrictionConeEdges;i++,m++) {
      A(m,p*3) = -fc.planes[i].x;
      A(m,p*3+1) = -fc.planes[i].y;
      A(m,p*3+2) = -fc.planes[i].z;
    }
  }
}

void GetFrictionConePlanes(const vector<ContactPoint>& c,int nFrictionConeEdges,SparseMatrix& A)
{
  int nc=(int)c.size();
  A.resize(nFrictionConeEdges*nc,nc*3);
  A.setZero();

  int m=0;
  for(size_t p=0;p<c.size();p++) {
    FrictionConePolygon fc;
    fc.set(nFrictionConeEdges,c[p].n,c[p].kFriction);
    for(int i=0;i<nFrictionConeEdges;i++,m++) {
      A(m,p*3) = -fc.planes[i].x;
      A(m,p*3+1) = -fc.planes[i].y;
      A(m,p*3+2) = -fc.planes[i].z;
    }
  }
}







void FrictionToFrictionlessContacts(const vector<ContactPoint2D>& c1,vector<ContactPoint2D>& c2)
{
  int n=0;
  for(size_t i=0;i<c1.size();i++) {
    if(c1[i].kFriction == 0) n++;
    else if(c1[i].kFriction > 1e6) {
      cout<<"FrictionToFrictionlessContacts: Warning, be careful with the use of this function, behavior is not always correct in kFriction = inf case"<<endl;
      getchar();
    }
    else n+=2;
  }

  c2.resize(n);
  n=0;
  FrictionConePolygon fc;
  for(size_t i=0;i<c1.size();i++) {
    if(c1[i].kFriction == 0) {
      c2[n] = c1[i];
      n++;
    }
    else {
      Vector2 t;
      t.setPerpendicular(c1[i].n);
      c2[n].x = c1[i].x;
      c2[n].n = c1[i].n + t*c1[i].kFriction;
      c2[n].n.inplaceNormalize();
      c2[n].kFriction = 0;
      c2[n+1].x = c1[i].x;
      c2[n+1].n = c1[i].n - t*c1[i].kFriction;
      c2[n+1].n.inplaceNormalize();
      c2[n+1].kFriction = 0;
      n += 2;
    }
  }
}


void GetFrictionConePlanes(const ContactPoint2D& c,Matrix2& A)
{
  Vector2 t;
  t.setPerpendicular(c.n);

  Vector2 a = c.n+t*c.kFriction;
  A(0,0)=-a.y;  A(0,1)=a.x;
  a = c.n-t*c.kFriction;
  A(1,0)=a.y;  A(1,1)=-a.x;
}


void GetFrictionConePlanes(const vector<ContactPoint2D>& c,Matrix& A)
{
  int nc=(int)c.size();
  A.resize(2*nc,2*nc);
  A.setZero();

  for(size_t i=0;i<c.size();i++) {
    Matrix2 Aii;
    GetFrictionConePlanes(c[i],Aii);
    A(i*2,i*2)=Aii(0,0);
    A(i*2+1,i*2)=Aii(1,0);
    A(i*2,i*2+1)=Aii(0,1);
    A(i*2+1,i*2+1)=Aii(1,1);
  }
}


void GetFrictionConePlanes(const vector<ContactPoint2D>& c,SparseMatrix& A)
{
  int nc=(int)c.size();
  A.resize(2*nc,2*nc);
  A.setZero();

  for(size_t i=0;i<c.size();i++) {
    Matrix2 Aii;
    GetFrictionConePlanes(c[i],Aii);
    A(i*2,i*2)=Aii(0,0);
    A(i*2+1,i*2)=Aii(1,0);
    A(i*2,i*2+1)=Aii(0,1);
    A(i*2+1,i*2+1)=Aii(1,1);
  }
}
