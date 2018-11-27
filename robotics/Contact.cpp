#include <KrisLibrary/Logger.h>
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

void ContactFormation::set(int link,const std::vector<ContactPoint>& _contacts,int target)
{
  links.resize(1);
  contacts.resize(1);
  links[0] = link;
  contacts[0] = _contacts;
  if(target < 0) targets.resize(0);
  else {
    targets.resize(1);
    targets[0] = target;
  }
}

void ContactFormation::concat(const ContactFormation& formation)
{
  if(targets.empty() && !formation.targets.empty()) 
    targets.resize(links.size(),-1);
  links.insert(links.end(),formation.links.begin(),formation.links.end());
  contacts.insert(contacts.end(),formation.contacts.begin(),formation.contacts.end());
  if(!targets.empty() && formation.targets.empty())
    targets.resize(links.size(),-1);
  else
    targets.insert(targets.end(),formation.targets.begin(),formation.targets.end());
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

int ContactFormation::numContactPoints() const
{ 
  int n=0;
  for(size_t i=0;i<contacts.size();i++)
    n += (int)contacts[i].size();
  return n;
}

int ContactFormation::numForceVariables() const { return 3*numContactPoints(); }



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
      LOG4CXX_WARN(KrisLibrary::logger(),"FrictionToFrictionlessContacts: Warning, be careful with the use of this function, behavior is not always correct in kFriction = inf case");
      KrisLibrary::loggerWait();
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
      LOG4CXX_WARN(KrisLibrary::logger(),"FrictionToFrictionlessContacts: Warning, be careful with the use of this function, behavior is not always correct in kFriction = inf case");
      KrisLibrary::loggerWait();
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






CustomContactPoint::CustomContactPoint()
:x(0.0),n(0,0,1),kFriction(0.0)
{}

CustomContactPoint::CustomContactPoint(const ContactPoint& cp,int numFCEdges)
{
  set(cp,numFCEdges);
}

int CustomContactPoint::numForceVariables() const
{
  if(!wrenchMatrix.isEmpty()) return 6;
  else if(!forceMatrix.isEmpty()) return 3;
  else if(kFriction > 0) return 3;
  else return 1;
}

int CustomContactPoint::numConstraints() const
{
  if(!wrenchMatrix.isEmpty()) return wrenchMatrix.m;
  else return forceMatrix.m;
}

void CustomContactPoint::set(const ContactPoint& cp,int numFCEdges)
{
  x = cp.x;
  n = cp.n;
  kFriction = cp.kFriction;
  calculateForceMatrix(numFCEdges);
}
void CustomContactPoint::setRobustnessFactor(Real offset)
{
  if(forceMatrix.isEmpty()) calculateForceMatrix();
  for(int i=0;i<forceOffset.n;i++)
    forceOffset[i] -= offset;
}

void CustomContactPoint::addNormalForceBounds(Real minimum,Real maximum)
{
  if(forceMatrix.isEmpty()) calculateForceMatrix();
  int numNew = 0;
  if(minimum > 0) numNew++;
  if(!IsInf(maximum)) numNew++;
  if(numNew == 0) return;
  int start = forceMatrix.m;
  Matrix newMat(forceMatrix.m+numNew,forceMatrix.n);
  Vector newVec(forceOffset.n+numNew);
  newMat.copySubMatrix(0,0,forceMatrix);
  newVec.copySubVector(0,forceOffset);
  if(minimum > 0) {
    newMat(start,0) = -n.x;
    newMat(start,1) = -n.y;
    newMat(start,2) = -n.z;
    newVec(start) = minimum;
    start++;
  }
  if(!IsInf(maximum)) {
    newMat(start,0) = n.x;
    newMat(start,1) = n.y;
    newMat(start,2) = n.z;
    newVec(start) = maximum;
    start++;
  }
}

void CustomContactPoint::calculateForceMatrix(int numFCEdges)
{
  ContactPoint cp;
  cp.x = x;
  cp.n = n;
  cp.kFriction = kFriction;
  GetFrictionConePlanes(cp,numFCEdges,forceMatrix);
  forceOffset.resize(forceMatrix.m);
  forceOffset.set(0.0);
}

void CustomContactPoint::calculateWrenchMatrix(int numFCEdges)
{
  if(forceMatrix.isEmpty()) calculateForceMatrix(numFCEdges);
  wrenchMatrix.resize(forceMatrix.m+6,6);
  wrenchOffset.resize(forceOffset.n+6);
  //w = (f,m)
  //m = p x f
  //A f <= b
  //equivalent constraint C w <= d
  //has the form
  //C = [A    0]   d = [b] 
  //    [[p] -I]       [0]
  //    [-[p] I]       [0]
  wrenchMatrix.setZero();
  wrenchMatrix.copySubMatrix(0,0,forceMatrix);
  wrenchOffset.copySubVector(0,forceOffset);
  Matrix3 cp;
  cp.setCrossProduct(x);
  for(int i=0;i<3;i++) {
    int row = forceMatrix.m+i;
    wrenchOffset(row) = 0;
    wrenchMatrix(row,0) = cp(row,0);
    wrenchMatrix(row,1) = cp(row,1);
    wrenchMatrix(row,2) = cp(row,2);
    wrenchMatrix(row,3+i) = -1;
    row += 3;
    wrenchOffset(row) = 0;
    wrenchMatrix(row,0) = -cp(row,0);
    wrenchMatrix(row,1) = -cp(row,1);
    wrenchMatrix(row,2) = -cp(row,2);
    wrenchMatrix(row,3+i) = 1;
  }
}


CustomContactPoint2D::CustomContactPoint2D()
:x(0.0),n(0,1),kFriction(0.0)
{}

CustomContactPoint2D::CustomContactPoint2D(const ContactPoint2D& cp)
{
}

int CustomContactPoint2D::numForceVariables() const
{
  if(!wrenchMatrix.isEmpty()) return 3;
  else if(!forceMatrix.isEmpty()) return 2;
  else if(kFriction > 0) return 2;
  else return 1;
}

int CustomContactPoint2D::numConstraints() const
{
  if(!wrenchMatrix.isEmpty()) return wrenchMatrix.m;
  else return forceMatrix.m;
}



void CustomContactPoint2D::set(const ContactPoint2D& cp)
{
  x = cp.x;
  n = cp.n;
  kFriction = cp.kFriction;
}
void CustomContactPoint2D::setRobustnessFactor(Real offset)
{
  if(forceMatrix.isEmpty()) calculateForceMatrix();
  for(int i=0;i<forceOffset.n;i++)
    forceOffset[i] -= offset;
}

void CustomContactPoint2D::addNormalForceBounds(Real minimum,Real maximum)
{
  if(forceMatrix.isEmpty()) calculateForceMatrix();
  int numNew = 0;
  if(minimum > 0) numNew++;
  if(!IsInf(maximum)) numNew++;
  if(numNew == 0) return;
  int start = forceMatrix.m;
  Matrix newMat(forceMatrix.m+numNew,forceMatrix.n);
  Vector newVec(forceOffset.n+numNew);
  newMat.copySubMatrix(0,0,forceMatrix);
  newVec.copySubVector(0,forceOffset);
  if(minimum > 0) {
    newMat(start,0) = -n.x;
    newMat(start,1) = -n.y;
    newVec(start) = minimum;
    start++;
  }
  if(!IsInf(maximum)) {
    newMat(start,0) = n.x;
    newMat(start,1) = n.y;
    newVec(start) = maximum;
    start++;
  }
}

void CustomContactPoint2D::calculateForceMatrix()
{
  ContactPoint2D cp;
  cp.x = x;
  cp.n = n;
  cp.kFriction = kFriction;
  Matrix2 A;
  GetFrictionConePlanes(cp,A);
  forceMatrix.resize(2,2);
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++)
      forceMatrix(i,j) = A(i,j);
  forceOffset.resize(forceMatrix.m);
  forceOffset.set(0.0);
}

void CustomContactPoint2D::calculateWrenchMatrix()
{
  if(forceMatrix.isEmpty()) calculateForceMatrix();
  wrenchMatrix.resize(forceMatrix.m+2,3);
  wrenchOffset.resize(forceOffset.n+2);
  //w = (f,m)
  //m = p x f
  //A f <= b
  //equivalent constraint C w <= d
  //has the form
  //C = [A    0]   d = [b] 
  //    [[p] -1]       [0]
  //    [-[p] 1]       [0]
  wrenchMatrix.setZero();
  wrenchMatrix.copySubMatrix(0,0,forceMatrix);
  wrenchOffset.copySubVector(0,forceOffset);
  int row = forceMatrix.m;
  wrenchOffset(row) = 0;
  wrenchMatrix(row,0) = -x.y;
  wrenchMatrix(row,1) = x.y;
  wrenchMatrix(row,2) = -1;
  row += 1;
  wrenchOffset(row) = 0;
  wrenchMatrix(row,0) = x.y;
  wrenchMatrix(row,1) = -x.y;
  wrenchMatrix(row,2) = 1;
}


void CustomContactFormation::clear()
{
  links.resize(0);
  contacts.resize(0);
  targets.resize(0);
  constraintGroups.resize(0);
  constraintMatrices.resize(0);
  constraintOffsets.resize(0);
  constraintEqualities.resize(0);
}

void CustomContactFormation::set(int link,const std::vector<ContactPoint>& _contacts,int numFCEdges)
{
  clear();
  links.resize(_contacts.size(),link);
  contacts.resize(_contacts.size());
  for(size_t i=0;i<contacts.size();i++) 
    contacts[i].set(_contacts[i],numFCEdges);
}

void CustomContactFormation::set(int link,const std::vector<CustomContactPoint>& _contacts)
{
  clear();
  links.resize(_contacts.size(),link);
  contacts=_contacts;
}

void CustomContactFormation::set(const ContactFormation& formation,int numFCEdges)
{
  clear();
  vector<ContactPoint> cps;
  formation.flatten(links,cps,targets);
  contacts.resize(cps.size());
  for(size_t i=0;i<cps.size();i++)
    contacts[i].set(cps[i],numFCEdges);
}

void CustomContactFormation::concat(const CustomContactFormation& formation)
{
  int nstart = (int)links.size();
  if(targets.empty() && !formation.targets.empty()) 
    targets.resize(links.size(),-1);
  links.insert(links.end(),formation.links.begin(),formation.links.end());
  contacts.insert(contacts.end(),formation.contacts.begin(),formation.contacts.end());
  if(!targets.empty() && formation.targets.empty())
    targets.resize(links.size(),-1);
  else
    targets.insert(targets.end(),formation.targets.begin(),formation.targets.end());
  int gstart=(int)constraintGroups.size();
  constraintGroups.insert(constraintGroups.end(),formation.constraintGroups.begin(),formation.constraintGroups.end());
  constraintMatrices.insert(constraintMatrices.end(),formation.constraintMatrices.begin(),formation.constraintMatrices.end());
  constraintOffsets.insert(constraintOffsets.end(),formation.constraintOffsets.begin(),formation.constraintOffsets.end());
  constraintEqualities.insert(constraintEqualities.end(),formation.constraintEqualities.begin(),formation.constraintEqualities.end());
  //shift indices
  for(size_t i=gstart;i<constraintGroups.size();i++) {
    for(size_t j=0;j<constraintGroups[i].size();j++)
      constraintGroups[i][j] += nstart;
  }
}

void CustomContactFormation::addLinkForceLimit(int link,const Vector3& direction,Real maximum)
{
  vector<int> contactIndices;
  for(size_t i=0;i<links.size();i++) 
    if(links[i] == link) contactIndices.push_back(int(i));
  if(contactIndices.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"CustomContactFormation::addLinkForceLimit: warning, link "<<link);
    return;
  }
  addForceLimit(contactIndices,direction,maximum);
}

void CustomContactFormation::addLinkWrenchLimit(int link,const Vector3& fdirection,const Vector3& mdirection,Real maximum)
{
  vector<int> contactIndices;
  for(size_t i=0;i<links.size();i++) 
    if(links[i] == link) contactIndices.push_back(int(i));
  if(contactIndices.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"CustomContactFormation::addLinkWrenchLimit: warning, link "<<link);
    return;
  }
  addWrenchLimit(contactIndices,fdirection,mdirection,maximum);
}

void CustomContactFormation::addForceLimit(const std::vector<int>& contacts,const Vector3& direction,Real maximum)
{
  if(contacts.empty()) return;
  vector<Matrix> mats(contacts.size());
  Vector offset(1,maximum);
  mats[0].resize(1,3);
  for(int i=0;i<3;i++)
    mats[0](0,i) = direction[i];
  for(size_t i=1;i<contacts.size();i++)
    mats[i].setRef(mats[0]);
  addForceConstraint(contacts,mats,offset);
}

void CustomContactFormation::addWrenchLimit(const std::vector<int>& contacts,const Vector3& fdirection,const Vector3& mdirection,Real maximum)
{
  if(contacts.empty()) return;
  vector<Matrix> mats(contacts.size());
  Vector offset(1,maximum);
  mats[0].resize(1,6);
  for(int i=0;i<3;i++)
    mats[0](0,i) = fdirection[i];
  for(int i=0;i<3;i++)
    mats[0](0,i+3) = mdirection[i];
  for(size_t i=1;i<contacts.size();i++)
    mats[i].setRef(mats[0]);
  addWrenchConstraint(contacts,mats,offset); 
}

void CustomContactFormation::addLinkForceConstraint(int link,const Matrix& A,const Vector& b,bool equality)
{
  if(A.n != 6) FatalError("addLinkForceConstraint: matrix must have 3 columns");
  if(A.m != b.n) FatalError("addLinkForceConstraint: matrix must have same number of rows as vector");

  vector<int> contactIndices;
  for(size_t i=0;i<links.size();i++) 
    if(links[i] == link) contactIndices.push_back(int(i));
  if(contactIndices.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"CustomContactFormation::addLinkForceConstraint: warning, link "<<link);
    return;
  }
  vector<Matrix> matrices(contactIndices.size());
  for(size_t i=0;i<matrices.size();i++)
    matrices[i].setRef(A);
  addForceConstraint(contactIndices,matrices,b,equality);
}

void CustomContactFormation::addLinkWrenchConstraint(int link,const Matrix& A,const Vector& b,bool equality)
{
  if(A.n != 6) FatalError("addLinkWrenchConstraint: matrix must have 6 columns");
  if(A.m != b.n) FatalError("addLinkWrenchConstraint: matrix must have same number of rows as vector");
  vector<int> contactIndices;
  for(size_t i=0;i<links.size();i++) 
    if(links[i] == link) contactIndices.push_back(int(i));
  if(contactIndices.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"CustomContactFormation::addLinkForceConstraint: warning, link "<<link);
    return;
  }
  vector<Matrix> matrices(contactIndices.size());
  for(size_t i=0;i<matrices.size();i++)
    matrices[i].setRef(A);
  addWrenchConstraint(contactIndices,matrices,b,equality);
}


void CustomContactFormation::addForceConstraint(const vector<int>& contactIndices,const Matrix& A,const Vector& b,bool equality)
{
  if((int)contactIndices.size()*3 != A.n) FatalError("addForceConstraint: stacked matrix is not of correct size");
  vector<Matrix> subMat(contactIndices.size());
  for(size_t i=0;i<contactIndices.size();i++)
    subMat[i].setRef(A,0,i*3,1,1,A.m,3);
  addForceConstraint(contactIndices,subMat,b,equality);
}

void CustomContactFormation::addForceConstraint(const vector<int>& contactIndices,const vector<Matrix>& A,const Vector& b,bool equality)
{
  for(size_t i=0;i<contactIndices.size();i++)
    if(contactIndices[i] < 0 || contactIndices[i] >= (int)links.size())  FatalError("addForceConstraint: specified an invalid contact");
  if(contactIndices.size() != A.size()) FatalError("addForceConstraint: indices and matrices must have same size");
  for(size_t i=0;i<contactIndices.size();i++) {
    if(A[i].n != 6) FatalError("addForceConstraint: matrix must have 3 columns");
    if(A[i].m != b.n) FatalError("addForceConstraint: matrix must have same number of rows as vector");
  }

  constraintGroups.push_back(contactIndices);
  constraintMatrices.resize(constraintMatrices.size()+1);
  constraintMatrices.back().resize(contactIndices.size());
  for(size_t i=0;i<contactIndices.size();i++) {
    int k=contactIndices[i];
    if(contacts[k].numForceVariables() == 1) {
      Vector w(3);
      contacts[k].n.get(w(0),w(1),w(2));
      Vector temp;
      A[i].mul(w,temp);
      constraintMatrices.back()[i].resize(A[i].m,1);
      constraintMatrices.back()[i].copyCol(0,temp);
    }
    else if(contacts[i].numForceVariables() == 3) {
      constraintMatrices.back()[i] = A[i];
    }
    else {
      constraintMatrices.back()[i].resize(A[i].m,6,0.0);
      constraintMatrices.back()[i].copySubMatrix(0,0,A[i]);
    }
  }
  constraintOffsets.push_back(b);
  constraintEqualities.push_back(equality);
}

void CustomContactFormation::addWrenchConstraint(const vector<int>& contactIndices,const Matrix& A,const Vector& b,bool equality)
{
  if((int)contactIndices.size()*6 != A.n) FatalError("addWrenchConstraint: stacked matrix is not of correct size");
  vector<Matrix> subMat(contactIndices.size());
    for(size_t i=0;i<contactIndices.size();i++)
    subMat[i].setRef(A,0,i*6,1,1,A.m,6);
  addForceConstraint(contactIndices,subMat,b,equality);
}

void CustomContactFormation::addWrenchConstraint(const vector<int>& contactIndices,const vector<Matrix>& A,const Vector& b,bool equality)
{
  for(size_t i=0;i<contactIndices.size();i++)
    if(contactIndices[i] < 0 || contactIndices[i] >= (int)links.size())  FatalError("addWrenchConstraint: specified an invalid contact");
  if(contactIndices.size() != A.size()) FatalError("addWrenchConstraint: indices and matrices must have same size");
  for(size_t i=0;i<contactIndices.size();i++) {
    if(A[i].n != 6) FatalError("addWrenchConstraint: matrix must have 3 columns");
    if(A[i].m != b.n) FatalError("addWrenchConstraint: matrix must have same number of rows as vector");
  }

  constraintGroups.push_back(contactIndices);
  constraintMatrices.resize(constraintMatrices.size()+1);
  constraintMatrices.back().resize(contactIndices.size());
  Vector w(6),temp;
  Matrix W(6,3);
  for(size_t i=0;i<contactIndices.size();i++) {
    int k=contactIndices[i];
    if(contacts[k].numForceVariables() == 1) {
      contacts[k].n.get(w(0),w(1),w(2));
      Vector3 cp; cp.setCross(contacts[k].x,contacts[k].n);
      cp.get(w(3),w(4),w(5));
      A[i].mul(w,temp);
      constraintMatrices.back()[i].resize(A[i].m,1);
      constraintMatrices.back()[i].copyCol(0,temp);
    }
    else if(contacts[i].numForceVariables() == 3) {
      W.setZero();
      for(int j=0;j<3;j++)
        W(j,j) = 1;
      Matrix3 cp;
      cp.setCrossProduct(contacts[k].x);
      for(int j=0;j<3;j++)
        for(int k=0;k<3;k++)
          W(j,k) = cp(j,k);
      constraintMatrices.back()[i].mul(A[i],W);
    }
    else {
      constraintMatrices.back()[i] = A[i];
    }
  }
  constraintOffsets.push_back(b);
  constraintEqualities.push_back(equality);
}

int CustomContactFormation::numForceVariables() const
{
  int nf = 0;
  for(size_t i=0;i<contacts.size();i++) 
    nf += contacts[i].numForceVariables();
  return nf;
}

int CustomContactFormation::numConstraints() const
{
  int nc = 0;
  for(size_t i=0;i<contacts.size();i++) 
    nc += contacts[i].numConstraints();
  for(size_t i=0;i<constraintGroups.size();i++)
    nc += constraintOffsets[i].n;
  return nc;
}

void GetWrenchMatrix(const std::vector<ContactPoint>& s,const Vector3& cm,Matrix& A)
{
  if(A.isEmpty())
    A.resize(6,s.size()*3);
  else if(A.m < 6 || A.n < (int)s.size()*3)
    FatalError("Invalid matrix size provided to GetWrenchMatrix");
  else {
    //danger of missing some existing entries
    Matrix temp;
    temp.setRef(A,0,0,1,1,6,s.size()*3);
    temp.setZero();
    GetWrenchMatrix(s,cm,temp);
    return;
  }

  int m=0;
  for(size_t j=0;j<s.size();j++,m+=3) {
    A(0,m) = A(1,m+1) = A(2,m+2) = One;
    Matrix3 tmp;
    tmp.setCrossProduct(s[j].x-cm);
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++)
        A(3+p,m+q) = tmp(p,q);
  }  
}

void GetWrenchMatrix(const std::vector<ContactPoint>& s,const Vector3& cm,SparseMatrix& A)
{
  if(A.isEmpty())
    A.resize(6,s.size()*3);
  else if(A.m < 6 || A.n < (int)s.size()*3)
    FatalError("Invalid matrix size provided to GetWrenchMatrix");
  else if(A.numNonZeros() > 0) {
    //danger of missing some existing entries
    SparseMatrix temp;
    GetWrenchMatrix(s,cm,temp);
    A.copySubMatrix(0,0,temp);
    return;
  }

  int m=0;
  for(size_t j=0;j<s.size();j++,m+=3) {
    A(0,m) = A(1,m+1) = A(2,m+2) = One;
    Matrix3 tmp;
    tmp.setCrossProduct(s[j].x-cm);
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++)
        A(3+p,m+q) = tmp(p,q);
  }  
}

void GetWrenchMatrix(const ContactFormation& s,const Vector3& cm,SparseMatrix& A)
{
  if(A.isEmpty())
    A.resize(6,s.numForceVariables());
  else if(A.m < 6 || A.n < s.numForceVariables())
    FatalError("Invalid matrix size provided to GetWrenchMatrix");
  else if(A.numNonZeros() > 0) {
    //danger of missing some existing entries
    SparseMatrix temp;
    GetWrenchMatrix(s,cm,temp);
    A.copySubMatrix(0,0,temp);
    return;
  }
  int m=0;
  for(size_t i=0;i<s.contacts.size();i++) {
    const vector<ContactPoint>& h=s.contacts[i];
    for(size_t j=0;j<h.size();j++,m+=3) {
      A(0,m) = A(1,m+1) = A(2,m+2) = One;
      Matrix3 tmp;
      tmp.setCrossProduct(h[j].x-cm);
      for(int p=0;p<3;p++)
  for(int q=0;q<3;q++)
    A(3+p,m+q) = tmp(p,q);
    }
  }
}


void GetFrictionConePlanes(const ContactFormation& s,int nFrictionConeEdges,SparseMatrix& A)
{
  int nc=s.numContactPoints();
  A.resize(nFrictionConeEdges*nc,nc*3);
  A.setZero();

  int m=0;
  int p=0;
  for(size_t i=0;i<s.contacts.size();i++) {
    const vector<ContactPoint>& h=s.contacts[i];
    for(size_t j=0;j<h.size();j++,p++) {
      const ContactPoint& pt=h[j];
      FrictionConePolygon fc;
      fc.set(nFrictionConeEdges,pt.n,pt.kFriction);
      for(int i=0;i<nFrictionConeEdges;i++,m++) {
  A(m,p*3) = -fc.planes[i].x;
  A(m,p*3+1) = -fc.planes[i].y;
  A(m,p*3+2) = -fc.planes[i].z;
      }
    }
  }
}



void GetForceMatrix(const std::vector<CustomContactPoint>& c,SparseMatrix& A)
{
  int nf=0;
  for(size_t i=0;i<c.size();i++) 
    nf += c[i].numForceVariables();
  A.resize(3,nf);
  A.setZero();
  int n=0;
  for(size_t i=0;i<c.size();i++) {
    if(c[i].numForceVariables() == 1) {
      c[i].n.get(A(0,n),A(1,n),A(2,n));
    }
    else if(c[i].numForceVariables() >= 3) {
      A(0,n) = 1.0;
      A(1,n+1) = 1.0;
      A(2,n+2) = 1.0;
    }
    n += c[i].numForceVariables();
  }
}

void GetForceMatrix(const CustomContactFormation& c,SparseMatrix& A)
{
  GetForceMatrix(c.contacts,A);
}

void GetWrenchMatrix(const std::vector<CustomContactPoint>& c,const Vector3& cm,Matrix& A)
{
  int nf=0;
  for(size_t i=0;i<c.size();i++) 
    nf += c[i].numForceVariables();
  if(A.isEmpty())
    A.resize(6,nf);
  else if(A.m < 6 || A.n < nf)
    FatalError("Invalid matrix size provided to GetWrenchMatrix");
  else {
    Matrix temp;
    temp.setRef(A,0,0,1,1,6,nf);
    temp.setZero();
    GetWrenchMatrix(c,cm,temp);
    return;
  }
  int n=0;
  for(size_t i=0;i<c.size();i++) {
    if(c[i].numForceVariables() == 1) {
      c[i].n.get(A(0,n),A(1,n),A(2,n));
      Vector3 cp; cp.setCross(c[i].x,c[i].n);
      cp.get(A(3,n),A(4,n),A(5,n));
    }
    else if(c[i].numForceVariables() == 3) {
      A(0,n) = 1.0;
      A(1,n+1) = 1.0;
      A(2,n+2) = 1.0;
      Matrix3 cp;
      cp.setCrossProduct(c[i].x);
      for(int j=0;j<3;j++)
        for(int k=0;k<3;k++)
          A(3+j,n+k) = cp(j,k);
    }
    else {
      for(int j=0;j<6;j++) A(j,n+j) = 1.0;
    }
    n += c[i].numForceVariables();
  }
}

void GetWrenchMatrix(const std::vector<CustomContactPoint>& c,const Vector3& cm,SparseMatrix& A)
{
  int nf=0;
  for(size_t i=0;i<c.size();i++) 
    nf += c[i].numForceVariables();
  if(A.isEmpty())
    A.resize(6,nf);
  else if(A.m < 6 || A.n < nf)
    FatalError("Invalid matrix size provided to GetWrenchMatrix");
  else {
    SparseMatrix temp;
    GetWrenchMatrix(c,cm,temp);
    A.copySubMatrix(0,0,temp);
    return;
  }
  int n=0;
  for(size_t i=0;i<c.size();i++) {
    if(c[i].numForceVariables() == 1) {
      c[i].n.get(A(0,n),A(1,n),A(2,n));
      Vector3 cp; cp.setCross(c[i].x,c[i].n);
      cp.get(A(3,n),A(4,n),A(5,n));
    }
    else if(c[i].numForceVariables() == 3) {
      A(0,n) = 1.0;
      A(1,n+1) = 1.0;
      A(2,n+2) = 1.0;
      Matrix3 cp;
      cp.setCrossProduct(c[i].x);
      for(int j=0;j<3;j++)
        for(int k=0;k<3;k++)
          A(3+j,n+k) = cp(j,k);
    }
    else {
      for(int j=0;j<6;j++) A(j,n+j) = 1.0;
    }
    n += c[i].numForceVariables();
  }
}

void GetWrenchMatrix(const CustomContactFormation& c,const Vector3& cm,SparseMatrix& A)
{
  GetWrenchMatrix(c.contacts,cm,A);
}

void GetFrictionConePlanes(const vector<CustomContactPoint>& c,Matrix& A,Vector& b)
{
  int nf=0;
  int nc = 0;
  for(size_t i=0;i<c.size();i++) {
    nf += c[i].numForceVariables();
    nc += c[i].numConstraints();
  }
  A.resize(nc,nf);
  b.resize(nc);
  A.setZero();

  int n=0;
  int m=0;
  for(size_t p=0;p<c.size();p++) {
    if(c[p].numForceVariables()==1) {
      //any constraints?
    }
    else if(c[p].numForceVariables() == 3) {
      for(int i=0;i<c[p].forceMatrix.m;i++,m++) {
        A(m,n) = c[p].forceMatrix(i,0);
        A(m,n+1) = c[p].forceMatrix(i,1);
        A(m,n+2) = c[p].forceMatrix(i,2);
        b(m) = c[p].forceOffset[i];
      }
    }
    else {
      for(int i=0;i<c[p].wrenchMatrix.m;i++,m++) {
        for(int j=0;j<6;j++)
          A(m,n+j) = c[p].forceMatrix(i,j);
        b(m) = c[p].forceOffset[i];
      }
    }
    n += c[p].numForceVariables();
  }
}

void GetFrictionConePlanes(const vector<CustomContactPoint>& c,SparseMatrix& A,Vector& b)
{
  int nf=0;
  int nc = 0;
  for(size_t i=0;i<c.size();i++) {
    nf += c[i].numForceVariables();
    nc += c[i].numConstraints();
  }
  A.resize(nc,nf);
  b.resize(nc);
  A.setZero();

  int n=0;
  int m=0;
  for(size_t p=0;p<c.size();p++) {
    if(c[p].numForceVariables()==1) {
      //any constraints?
    }
    else if(c[p].numForceVariables() == 3) {
      for(int i=0;i<c[p].forceMatrix.m;i++,m++) {
        A(m,n) = c[p].forceMatrix(i,0);
        A(m,n+1) = c[p].forceMatrix(i,1);
        A(m,n+2) = c[p].forceMatrix(i,2);
        b(m) = c[p].forceOffset[i];
      }
    }
    else {
      for(int i=0;i<c[p].wrenchMatrix.m;i++,m++) {
        for(int j=0;j<6;j++)
          A(m,n+j) = c[p].forceMatrix(i,j);
        b(m) = c[p].forceOffset[i];
      }
    }
    n += c[p].numForceVariables();
  }
}

void GetFrictionConePlanes(const CustomContactFormation& s,SparseMatrix& A,Vector& b)
{
  vector<int> contactOffsets(s.contacts.size());
  int nf = 0;
  int nc = 0;
  for(size_t i=0;i<s.contacts.size();i++) {
    contactOffsets[i] = nf;
    nf += s.contacts[i].numForceVariables();
    nc += s.contacts[i].numConstraints();
  }
  for(size_t i=0;i<s.constraintGroups.size();i++)
    nc += s.constraintOffsets[i].n;
  A.resize(nc,nf);
  A.setZero();
  b.resize(nc);

  const vector<CustomContactPoint>& c = s.contacts;
  int n=0;
  int m=0;
  for(size_t p=0;p<c.size();p++) {
    if(c[p].numForceVariables()==1) {
      //any constraints?
    }
    else if(c[p].numForceVariables() == 3) {
      for(int i=0;i<c[p].forceMatrix.m;i++,m++) {
        A(m,n) = c[p].forceMatrix(i,0);
        A(m,n+1) = c[p].forceMatrix(i,1);
        A(m,n+2) = c[p].forceMatrix(i,2);
        b(m) = c[p].forceOffset[i];
      }
    }
    else {
      for(int i=0;i<c[p].wrenchMatrix.m;i++,m++) {
        for(int j=0;j<6;j++)
          A(m,n+j) = c[p].forceMatrix(i,j);
        b(m) = c[p].forceOffset[i];
      }
    }
    n += c[p].numForceVariables();
  }
  for(size_t i=0;i<s.constraintGroups.size();i++) {
    for(size_t j=0;j<s.constraintGroups[i].size();j++)
      A.copySubMatrix(n,contactOffsets[s.constraintGroups[i][j]],s.constraintMatrices[i][j]);
    b.copySubVector(n,s.constraintOffsets[i]);
    n += s.constraintOffsets[i].n;
  }
}


void GetFrictionConePlanes(const vector<CustomContactPoint2D>& c,Matrix& A,Vector& b)
{
  int nc=(int)c.size();
  int numTotalConstraints = 0;
  for(size_t i=0;i<c.size();i++)
    numTotalConstraints += c[i].forceMatrix.m;
  A.resize(numTotalConstraints,nc*2,Zero);
  b.resize(numTotalConstraints);

  int m=0;
  for(size_t p=0;p<c.size();p++) {
    for(int i=0;i<c[p].forceMatrix.m;i++,m++) {
      A(m,p*2) = c[p].forceMatrix(i,0);
      A(m,p*2+1) = c[p].forceMatrix(i,1);
      b(m) = c[p].forceOffset[i];
    }
  }
}

void GetFrictionConePlanes(const vector<CustomContactPoint2D>& c,SparseMatrix& A,Vector& b)
{
  int nc=(int)c.size();
  int numTotalConstraints = 0;
  for(size_t i=0;i<c.size();i++)
    numTotalConstraints += c[i].forceMatrix.m;
  A.resize(numTotalConstraints,nc*2);
  b.resize(numTotalConstraints);
  A.setZero();

  int m=0;
  for(size_t p=0;p<c.size();p++) {
    for(int i=0;i<c[p].forceMatrix.m;i++,m++) {
      A(m,p*2) = c[p].forceMatrix(i,0);
      A(m,p*2+1) = c[p].forceMatrix(i,1);
      b(m) = c[p].forceOffset[i];
    }
  }
}
