#ifndef ROBOTICS_STABILITY_H
#define ROBOTICS_STABILITY_H

#include "Contact.h"
#include <KrisLibrary/geometry/UnboundedPolytope2D.h>
#include <KrisLibrary/optimization/LinearProgram.h>
#include <KrisLibrary/optimization/LPRobust.h>

/// Tests whether the contacts admit force closure
bool TestForceClosure(const std::vector<ContactPoint>& contacts,int numFCEdges);
bool TestForceClosure(const std::vector<CustomContactPoint>& contacts);

/// Tests whether the contacts admit force closure
bool TestForceClosure(const std::vector<ContactPoint2D>& contacts);
bool TestForceClosure(const std::vector<CustomContactPoint2D>& contacts);

/// Tests whether the given COM is stable for the given contacts.
/// If f is non-empty, it returns the contact forces in f.
bool TestCOMEquilibrium(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,const Vector3& com,std::vector<Vector3>& f);
bool TestCOMEquilibrium(const std::vector<CustomContactPoint>& contacts,const Vector3& fext,const Vector3& com,std::vector<Vector3>& f);

/// Tests whether the given contacts admits any stable COM.
bool TestAnyCOMEquilibrium(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges);
bool TestAnyCOMEquilibrium(const std::vector<CustomContactPoint>& contacts,const Vector3& fext);

/// Tests whether the given COM is stable for the given contacts.
/// If f is non-empty, it returns the contact forces in f.
bool TestCOMEquilibrium(const std::vector<ContactPoint2D>& contacts,const Vector2& fext,const Vector2& com,std::vector<Vector2>& f);
bool TestCOMEquilibrium(const std::vector<CustomContactPoint2D>& contacts,const Vector2& fext,const Vector2& com,std::vector<Vector2>& f);

/// Tests whether the given contacts admits any stable COM.
bool TestAnyCOMEquilibrium(const std::vector<ContactPoint2D>& contacts,const Vector2& fext);
bool TestAnyCOMEquilibrium(const std::vector<CustomContactPoint2D>& contacts,const Vector2& fext);

/** @ingroup Robotics
 * @brief Testing COM equilibrium given some number of contacts.
 *
 * The class methods have a potential advantage over the static TestCOM()
 * and TestAnyCOM() functions if you're testing multiple centers of mass with
 * the same number of contacts.  They also let you do more sophisticated
 * things with robustness and force limiting.
 *
 * However, if the contacts remain the same, and you're testing a large number
 * of COMs (10-20), it's better to use the SupportPolygon class to compute the
 * entire support polygon.
 *
 * @sa SupportPolygon
 */
class EquilibriumTester
{
 public:
  EquilibriumTester();
  void Setup(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,const Vector3& com);
  void Setup(const std::vector<CustomContactPoint>& contacts,const Vector3& fext,const Vector3& com);
  void Setup(const CustomContactFormation& contacts,const Vector3& fext,const Vector3& com);
  void SetupAnyCOM(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges);
  void SetupAnyCOM(const std::vector<CustomContactPoint>& contacts,const Vector3& fext);
  void SetupAnyCOM(const CustomContactFormation& contacts,const Vector3& fext);
  bool TestCOM(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,const Vector3& com);
  bool TestCOM(const std::vector<CustomContactPoint>& contacts,const Vector3& fext,const Vector3& com);
  bool TestCOM(const CustomContactFormation& contacts,const Vector3& fext,const Vector3& com);
  bool TestAnyCOM(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges);
  bool TestAnyCOM(const std::vector<CustomContactPoint>& contacts,const Vector3& fext);
  bool TestAnyCOM(const CustomContactFormation& contacts,const Vector3& fext);
  void ChangeContacts(const std::vector<ContactPoint>& contacts);
  void ChangeContact(int i,ContactPoint& contact);
  void ChangeGravity(const Vector3& fext);
  void ChangeCOM(const Vector3& com);
  void SetRobustnessFactor(Real frobust);
  void SetRobustnessFactor(int i,Real frobust);
  void LimitContactForce(int i,Real maximum,const Vector3& dir);
  void LimitContactForceSum(const std::vector<int>& indices,Real maximum,const Vector3& dir);
  bool TestCurrent();
  void GetValidCOM(Vector3& com) const;
  void GetForceVector(Vector& f) const;
  void GetForces(std::vector<Vector3>& f) const;
  void Clear();
  bool IsEmpty();
  int NumContacts() const;
  int NumFCEdges() const;

 private:
  Optimization::LinearProgram_Sparse lp;
  Optimization::RobustLPSolver lps;
  bool testingAnyCOM;
  Vector3 testedCOM;
  Vector3 conditioningShift;
  int numFCEdges;
};


/** @ingroup Robotics
 * @brief Calculation of the support polygon given a number of contacts.
 *
 * To test stability call the Set() method, then call the TestCOM() method.
 * This first computes the entire support polygon, and the second tests
 * for inclusion in the polygon.  This is faster than the static 
 * TestCOMEquilibrium() method if you need to test a large number of COMs.
 *
 * In the future, the incremental, adaptive algorithm of Bretl 2006 may be
 * implemented.
 *
 * Notes:
 * - Only a vertical gravity force is supported by Set().  For non-vertical
 *   forces, see the OrientedSupportPolygon class.
 * - The friction cones are polyhedralized with numFCEdges edges.
 */
class SupportPolygon : public Geometry::UnboundedPolytope2D
{
public:
  bool Set(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges,int maxExpandDepth=6);
  bool Set(const std::vector<CustomContactPoint>& contacts,const Vector3& fext,int maxExpandDepth=6);
  bool Set(const CustomContactFormation& contacts,const Vector3& fext,int maxExpandDepth=6);
  bool Empty() const;
  bool TestCOM(const Vector3& com) const;
  Real COMMargin(const Vector3& com) const;

  Vector3 fext;
  int numFCEdges;
  std::vector<ContactPoint> contacts;
};

/** @ingroup Robotics
 * @brief Calculation of a support polygon with a non-vertical gravity vector
 */
class OrientedSupportPolygon
{
public:
  bool Set(const std::vector<ContactPoint>& contacts,const Vector3& fext,int numFCEdges);
  bool Set(const std::vector<CustomContactPoint>& contacts,const Vector3& fext);
  bool Set(const CustomContactFormation& contacts,const Vector3& fext);
  bool TestCOM(const Vector3& com) const;
  Real COMMargin(const Vector3& com) const;
  //poly is a slice through the support polytope at the given height
  //in the xy plane
  void GetXYSlice(Real height,Geometry::UnboundedPolytope2D& poly) const;
  //poly is returned in the xy plane of the frame
  void GetSlice(const RigidTransform& frame,Geometry::UnboundedPolytope2D& poly) const;

  RigidTransform T;  //transform from original space to gravity-oriented space
  SupportPolygon sp;  //support polygon in the gravity-oriented space
};


#endif
