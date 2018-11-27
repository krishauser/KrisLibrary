#include <KrisLibrary/Logger.h>
#include "rotationfit.h"
#include "LinearAlgebra.h"
#include <math/matrix.h>
#include <math/SVDecomposition.h>
#include <iostream>
#include <errors.h>
using namespace std;
using namespace Math;

namespace Math3D {

Real RotationFit(const vector<Vector3>& a,const vector<Vector3>& b,Matrix3& R)
{
  Assert(a.size() == b.size());
  assert(a.size() >= 3);
  Matrix3 C;
  C.setZero();
  for(size_t k=0;k<a.size();k++) {
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
	C(i,j) += a[k][j]*b[k][i];
  }
  //let A=[a1 ... an]^t, B=[b1 ... bn]^t
  //solve for min sum of squares of E=ARt-B
  //let C=AtB
  //solution is given by CCt = RtCtCR

  //Solve C^tR = R^tC with SVD CC^t = R^tC^tCR
  //CtRX = RtCX
  //C = RCtR
  //Ct = RtCRt
  //=> CCt = RCtCRt
  //solve SVD of C and Ct (giving eigenvectors of CCt and CtC
  //C = UWVt => Ct=VWUt
  //=> UWUt = RVWVtRt
  //=> U=RV => R=UVt
  Matrix mC(3,3),mCtC(3,3);
  Copy(C,mC);
  SVDecomposition<Real> svd;
  if(!svd.set(mC)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"RotationFit: Couldn't set svd of covariance matrix");
    R.setIdentity();
    return Inf;
  }

  Matrix mR;
  mR.mulTransposeB(svd.U,svd.V);
  Copy(mR,R);
  if(R.determinant() < 0) {  //it's a mirror
    svd.sortSVs();
    if(!FuzzyZero(svd.W(2),(Real)1e-2)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"RotationFit: Uhh... what do we do?  SVD of rotation doesn't have a zero singular value");
      /*
      LOG4CXX_ERROR(KrisLibrary::logger(),svd.W);
      LOG4CXX_ERROR(KrisLibrary::logger(),"Press any key to continue");
      KrisLibrary::loggerWait();
      */
    }
    //negate the last column of V
    Vector vi;
    svd.V.getColRef(2,vi);
    vi.inplaceNegative();
    mR.mulTransposeB(svd.V,svd.U);
    Copy(mR,R);
    Assert(R.determinant() > 0);
  }

  Real sum=0;
  for(size_t k=0;k<a.size();k++) 
    sum += b[k].distanceSquared(R*a[k]);
  return sum;
}

void AxisRotationFit(const std::vector<Vector3>& a,const std::vector<Vector3>& b,const Vector3& z,Real& theta)
{
  Assert(!a.empty());
  Assert(a.size()==b.size());
  //min sum||R*a[i]-b[i]||^2
  //  = sum (R*a[i]-b[i]).(R*a[i]-b[i])
  //  = sum a[i].a[i] + b[i].b[i] - 2 b[i].R*a[i]
  //differentiating, get
  //  0 = sum b[i].R'*a[i] = sum b[i].[z]R*a[i]
  //Let s=sin(theta) and c=cos(theta). 
  //  R = cI + (1-c)zz' + s[z]
  //so 
  //  0 = sum c*b[i]'[z]a[i] + (1-c)b[i]'[z]zz'a[i] + sb[i]'[z][z]a[i]
  //    = c*sum b[i]'[z]a[i] + s*b[i]'[z][z]a[i]
  // [z] = [0 -z y]
  //       [z 0 -x]
  //       [-y x 0]
  // [z]^2 = [-zz-yy  xy       xz   ]
  //         [xy     -xx-zz    yz   ]
  //         [xz      yz      -xx-yy]
  //collecting terms by s and c,
  //get s(sum -axbx-ayby) + c(sum axby-aybx) = 0
  //solve using theta = atan(sum axby-aybx / sum -axbx-ayby)
  //is it theta or theta+pi?
  Matrix3 zcross,zcross2;
  zcross.setCrossProduct(z);
  zcross2.mul(zcross,zcross);
  Real scoeff=0,ccoeff=0;
  for(size_t i=0;i<a.size();i++)
    scoeff += b[i].dot(zcross2*a[i]);
  for(size_t i=0;i<b.size();i++)
    ccoeff += b[i].dot(zcross*a[i]);

  if(FuzzyZero(scoeff) && FuzzyZero(ccoeff))
    theta=0;
  else 
    theta = Atan2(-ccoeff,scoeff);
   
  Real c=Cos(theta),s=Sin(theta);
  if(c*scoeff-s*ccoeff > 0) {
    theta += Pi;
  }
}

/** @brief Calculate the least squares rotation fit 
 * min_R,t sum||R*a[i]+t-b[i]||^2.  Returns the sum-of-squared errors.
 *
 * Here, a and b provide temporary storage, their contents are modified
 */
Real TransformFit(vector<Vector3>& a,vector<Vector3>& b,Matrix3& R,Vector3& t)
{
  //center at centroid
  Vector3 ca(Zero),cb(Zero);
  for(size_t i=0;i<a.size();i++) ca += a[i];
  ca /= a.size();
  for(size_t i=0;i<a.size();i++) a[i]-=ca;

  for(size_t i=0;i<b.size();i++) cb += b[i];
  cb /= b.size();
  for(size_t i=0;i<b.size();i++) b[i]-=cb;
  //T(x) = Trans[cb](Rot[R](Trans[-ca](x)));
  //so t = cb - R*ca
  Real res=RotationFit(a,b,R);
  R.mul(ca,t);
  t -= cb;
  t.inplaceNegative();
  return res;
}



/** @brief Solve the mixed point/vector fitting problem
 * min sum_j wj||R*aj+t-bj||^2 + sum_k vk||Rc-d||^2
 *
 * R is a rotation, t is a translation, a and b are points, c and d are vectors, and w and v are optional weights.
 */
Real WeightedTransformFit(const vector<Point3D>& a,const vector<Point3D>& b,const vector<Real>& w,const vector<Vector3>& c,const vector<Vector3>& d,const vector<Real>& v,Matrix3& R,Vector3& t)
{
  Assert(a.size()==b.size());
  Assert(c.size()==d.size());
  if(!w.empty()) Assert(a.size()==w.size());
  if(!v.empty()) Assert(c.size()==v.size());
  vector<Vector3> src(a.size()+c.size()),dest(b.size()+d.size());
  Vector3 ca(Zero),cb(Zero);
  if(w.empty()) {
    for(size_t i=0;i<a.size();i++) ca += a[i];
    ca /= a.size();
    for(size_t i=0;i<b.size();i++) cb += b[i];
    cb /= b.size();
  }
  else {
    Real sumw=Zero;
    for(size_t i=0;i<a.size();i++) sumw+=w.size();
    for(size_t i=0;i<a.size();i++) ca.madd(a[i],w[i]);
    ca /= sumw;
    for(size_t i=0;i<b.size();i++) cb.madd(b[i],w[i]);
    cb /= sumw;
  }

  for(size_t i=0;i<a.size();i++) {
    src[i] = a[i]-ca;
    dest[i] = b[i]-cb;
    if(!w.empty()) {
      src[i]*=w[i];
      dest[i]*=w[i];
    }
  }
  for(size_t i=0;i<c.size();i++) {
    src[i+a.size()] = c[i];
    dest[i+a.size()] = d[i];
    if(!v.empty()) {
      src[i+a.size()]*=v[i];
      dest[i+a.size()]*=v[i];
    }
  }
  Real res=RotationFit(src,dest,R);
  R.mul(ca,t);
  t -= cb;
  t.inplaceNegative();
  return res;
}

Real FitFrames(const std::vector<Vector3>& a,const std::vector<Vector3>& b,
	       RigidTransform& Ta,RigidTransform& Tb,Vector3& cov)
{
  //center at centroid
  Vector3 ca(Zero),cb(Zero);
  for(size_t i=0;i<a.size();i++) ca += a[i];
  ca /= a.size();

  for(size_t i=0;i<b.size();i++) cb += b[i];
  cb /= b.size();
  
  Matrix3 C;
  C.setZero();
  for(size_t k=0;k<a.size();k++) {
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
	C(i,j) += (a[k][i]-ca[i])*(b[k][j]-cb[j]);  //note the difference from RotationFit
  }
  Matrix mC(3,3),mCtC(3,3);
  Copy(C,mC);
  SVDecomposition<Real> svd;
  if(!svd.set(mC)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"FitFrames: Couldn't set svd of covariance matrix");
    Ta.R.setIdentity();
    Tb.R.setIdentity();
    return Inf;
  }

  Copy(svd.U,Ta.R);
  Copy(svd.V,Tb.R);  
  Copy(svd.W,cov);
  Tb.R.inplaceTranspose();
  Ta.R.inplaceTranspose();

  if (Ta.R.determinant() < 0 || Tb.R.determinant() < 0) {
	  //need to sort singular values and negate the column according to the smallest SV
	  svd.sortSVs();
	  //negate the last column of V and last column of U
    if(Tb.R.determinant() < 0) {
  	  Vector vi;
  	  svd.V.getColRef(2, vi);
  	  vi.inplaceNegative();
    }
    if(Ta.R.determinant() < 0) {
      Vector ui;
      svd.U.getColRef(2, ui);
      ui.inplaceNegative(); 
    }
	  Copy(svd.U, Ta.R);
	  Copy(svd.V, Tb.R);
	  Copy(svd.W, cov);
	  Tb.R.inplaceTranspose();
	  Ta.R.inplaceTranspose();
	  //TODO: these may now both have a mirroring, but they may compose to rotations?
  }

  //need these to act as though they subtract off the centroids FIRST then apply rotation
  Ta.t = -(Ta.R*ca);
  Tb.t = -(Tb.R*cb);
  Real sum=0;
  for(size_t k=0;k<a.size();k++) 
    sum += (Tb.R*(b[k]-cb)).distanceSquared(Ta.R*(a[k]-ca));
  return sum;
}

} //namespace Math3D
