#ifndef PLANNING_RIGID_BODY_CSPACE_H
#define PLANNING_RIGID_BODY_CSPACE_H

#include "CSpaceHelpers.h"
#include <KrisLibrary/math3d/primitives.h>

/** @brief An axis-aligned subset of the space R^2 with
 * convenient casts from Math3d::Vector2, and with variable
 * names x,y
 */
class R2CSpace : public BoxCSpace
{
public:
  R2CSpace(const Math3D::Vector2& bmin,const Math3D::Vector2& bmax);
  virtual std::string VariableName(int i);
  void SetDomain(const Math3D::Vector2& bmin,const Math3D::Vector2& bmax);
  void GetDomain(Math3D::Vector2& bmin,Math3D::Vector2& bmax);
};

/** @brief An axis-aligned subset of the space R^3 with
 * convenient casts from Math3d::Vector3, and with variable
 * names x,y,z
 */
class R3CSpace : public BoxCSpace
{
public:
  R3CSpace(const Math3D::Vector3& bmin,const Math3D::Vector3& bmax);
  virtual std::string VariableName(int i);
  void SetDomain(const Math3D::Vector3& bmin,const Math3D::Vector3& bmax);
  void GetDomain(Math3D::Vector3& bmin,Math3D::Vector3& bmax);
};

/** @brief The space of rotations SO(2).  The representation
 * is a single angle.
 * Still need to implement IsFeasible, PathChecker
 */
class SO2CSpace : public GeodesicCSpace
{
public:
  virtual int NumDimensions() { return 1; }
  virtual std::string VariableName(int i);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual void Interpolate(const Config& a,const Config& b,Real u,Config& out);
  virtual Real Distance(const Config& a,const Config& b);
  virtual void Properties(PropertyMap& pmap);

  static void GetRotation(const Config& x,Math3D::Matrix2& R);
  static void SetRotation(const Math3D::Matrix2& R,Config& x);

  void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  void Integrate(const Config& a,const Vector& da,Config& b);
};

/** @brief The space of rigid body transforms SE(2). 
 * Still need to implement IsFeasible, PathChecker
 */
class SE2CSpace : public MultiCSpace
{
public:
  SE2CSpace(Real bmin=-Inf,Real bmax=Inf);
  SE2CSpace(const Math3D::Vector2& bmin,const Math3D::Vector2& bmax);

  void SetAngleWeight(Real weight);
  void SetDomain(const Math3D::Vector2& bmin,const Math3D::Vector2& bmax);

  static void GetTransform(const Config& x,Math3D::RigidTransform2D& T);
  static void SetTransform(const Math3D::RigidTransform2D& T,Config& x);
};

/** @brief The space of rotations S0(3).  The representation
 * is a MomentRotation.
 * Still need to implement IsFeasible, PathChecker
 */
class SO3CSpace : public GeodesicCSpace
{
public:
  virtual int NumDimensions() { return 3; }
  virtual std::string VariableName(int i);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual void Interpolate(const Config& a,const Config& b,Real u,Config& out);
  virtual Real Distance(const Config& a,const Config& b);
  virtual void Properties(PropertyMap& pmap);

  void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  void Integrate(const Config& a,const Vector& da,Config& b);

  static void GetRotation(const Config& x,Math3D::Matrix3& R);
  static void SetRotation(const Math3D::Matrix3& R,Config& x);
};

/** @brief The space of rigid body transforms SE(3). 
 * Still need to implement IsFeasible, PathChecker
 */
class SE3CSpace : public MultiCSpace
{
public:
  SE3CSpace(Real bmin=-Inf,Real bmax=Inf);
  SE3CSpace(const Math3D::Vector3& bmin,const Math3D::Vector3& bmax);

  void SetAngleWeight(Real weight);
  void SetDomain(const Math3D::Vector3& bmin,const Math3D::Vector3& bmax);

  static void GetTransform(const Config& x,Math3D::RigidTransform& T);
  static void SetTransform(const Math3D::RigidTransform& T,Config& x);
};


#endif
