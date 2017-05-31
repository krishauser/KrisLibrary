#ifndef CSPACE_HELPERS_H
#define CSPACE_HELPERS_H

#include "CSpace.h"
#include "GeodesicSpace.h"

/** @ingroup MotionPlanning
 * A base class for a CSpace that also has geodesic information built in.
 * You will still need to overload Sample() and LocalPlanner/PathChecker().
 */
class GeodesicCSpace : public CSpace, public GeodesicSpace
{
public:
  GeodesicCSpace() {}
  virtual ~GeodesicCSpace() {}
};

/** @ingroup MotionPlanning
 * A GeodesicCSpace that takes from another GeodesicSpace
 */
class GeodesicCSpaceAdaptor : public GeodesicCSpace
{
public:
  GeodesicCSpaceAdaptor(const SmartPointer<GeodesicSpace>& geodesic);
  virtual int NumDimensions() { return geodesic->NumDimensions(); }
  virtual int NumIntrinsicDimensions() { return geodesic->NumIntrinsicDimensions(); }
  virtual Real Distance(const Config& x, const Config& y) { return geodesic->Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { geodesic->Interpolate(x,y,u,out); }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) { geodesic->Interpolate(x,y,0.5,out); }

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) { geodesic->InterpolateDeriv(a,b,u,dx); }
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) { geodesic->InterpolateDerivA(a,b,u,da,dx); }
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) { geodesic->InterpolateDerivB(a,b,u,db,dx); }
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) { geodesic->InterpolateDeriv2(a,b,u,ddx); }
  virtual void Integrate(const Config& a,const Vector& da,Config& b) { geodesic->Integrate(a,da,b); }

  SmartPointer<GeodesicSpace> geodesic;
};

/** @ingroup MotionPlanning
 * A standard Cartesian CSpace.  
 * You will still need to overload Sample() and LocalPlanner/PathChecker().
 */
class CartesianCSpace : public GeodesicCSpace
{
public:
  CartesianCSpace(int d);
  virtual int NumDimensions() { return d; }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual Real Distance(const Config& x, const Config& y) { return CSpace::Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { CSpace::Interpolate(x,y,u,out); }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) { out.add(x,y); out.inplaceMul(0.5); }

  virtual void Sample(Config& q) { FatalError("Not implemented"); }

  int d;
};

/** @ingroup MotionPlanning
 * A standard box-bounded CSpace.  Adds a set of constraints named "[X]_bound" where
 * [X] is a VariableName().
 */
class BoxCSpace : public CartesianCSpace
{
public:
  BoxCSpace(Real xmin,Real xmax,int d=1);
  BoxCSpace(const Vector& bmin,const Vector& bmax);
  void SetDomain(const Vector& bmin,const Vector& bmax);
  void GetDomain(Vector& bmin,Vector& bmax);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual void Properties(PropertyMap&);

  ///The domain.  NOTE: modifing these does not directly affect the constraints! Use SetDomain instead
  Vector bmin,bmax;
};


/** @ingroup MotionPlanning
 * A Cartesian product of several CSpace's.  Inherits geodesic information.
 */
class MultiCSpace : public GeodesicCSpace
{
public:
  MultiCSpace();
  MultiCSpace(const SmartPointer<CSpace>& space1,const SmartPointer<CSpace>& space2);
  MultiCSpace(const std::vector<SmartPointer<CSpace> >& components);
  void Add(const std::string& name,const SmartPointer<CSpace>& space,Real distanceWeight=1);

  ///Rather than testing individual CSpace's, this flattens all the constraints so that 
  ///they're in the constraints list.
  void FlattenConstraints();
  ///Works once CSpaces are flattened
  void AddConstraint(int spaceIndex,const std::string& name,CSet* constraint);
  ///Works once CSpaces are flattened
  void AddConstraint(int spaceIndex,const std::string& name,const SmartPointer<CSet>& constraint);
  ///Works once CSpaces are flattened
  void AddConstraint(int spaceIndex,const std::string& name,CSet::CPredicate test);

  void Split(const Vector& x,std::vector<Vector>& items);
  void SplitRef(const Vector& x,std::vector<Vector>& items);
  void Join(const std::vector<Vector>& items,Vector& x);

  //CSpace overloads
  virtual int NumDimensions();
  virtual int NumIntrinsicDimensions();
  virtual std::string VariableName(int i);
  virtual int NumConstraints();
  virtual std::string ConstraintName(int i);
  virtual SmartPointer<CSet> Constraint(int i);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);
  virtual bool ProjectFeasible(Config& x);
  virtual Optimization::NonlinearProgram* FeasibleNumeric();
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b,int constraint);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);
  virtual Real ObstacleDistance(const Config& a);
  virtual void Properties(PropertyMap&);

  //GeodesicSpace overloads
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx);
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx);
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  std::vector<SmartPointer<CSpace> > components;
  std::vector<std::string> componentNames;
  std::vector<Real> distanceWeights;
};

/** @brief A helper class that assists with selective overriding
 * of another cspace's methods (similar to "monkey-patching").
 *
 * All CSpace methods are copied except those that are overridden
 * by the subclass of PiggybackCSpace. 
 *
 * Warning: unexpected behavior may be observed when using the 
 * baseSpace's local planners, because they often contain pointers
 * to the CSpace that created them, not the piggybacking CSpace.
 */
class PiggybackCSpace : public CSpace
{
public:
  PiggybackCSpace(CSpace* _baseSpace=NULL);
  virtual int NumDimensions();
  virtual std::string VariableName(int i);
  virtual int NumConstraints();
  virtual std::string ConstraintName(int i);
  virtual SmartPointer<CSet> Constraint(int i);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b,int obstacle);
  virtual bool IsFeasible(const Config& x);
  virtual bool IsFeasible(const Config&,int constraint);
  virtual bool ProjectFeasible(Config& x);
  virtual Optimization::NonlinearProgram* FeasibleNumeric();
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);
  virtual void Properties(PropertyMap& map);

  CSpace* baseSpace;
};

/** @brief Converts an CSpace so that it only checks
 * one or a subset of selected constraints.
 */
class SubsetConstraintCSpace : public PiggybackCSpace
{
public:
  SubsetConstraintCSpace(CSpace* baseSpace,const std::vector<int>& constraints);
  SubsetConstraintCSpace(CSpace* baseSpace,int constraints);
  virtual bool IsFeasible(const Config& x) { return CSpace::IsFeasible(x); }
  virtual bool IsFeasible(const Config& x,int obstacle) { return CSpace::IsFeasible(x,obstacle); }
  virtual bool ProjectFeasible(Config& x) { return CSpace::ProjectFeasible(x); }
  virtual Optimization::NonlinearProgram* FeasibleNumeric() { return CSpace::FeasibleNumeric(); }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return CSpace::LocalPlanner(a,b); }
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b,int obstacle);

  std::vector<int> activeConstraints;
};


/** @brief A class that optimizes constraint testing order using empirical
 * data.
 *
 * Allows feasibility and visibility tests to have dependent tests, which establishes
 * constraints on the order of optimized feasibility/visibility testing.  This lets you
 * implement quick-reject tests.
 * 
 * This functionality also (experimentally) allows a test to compute some data
 * (e.g., forward kinematics) which will then shared between several subsequent
 * tests.  However, this use case can lead to subtle bugs particularly with
 * single-obstacle visibility checks.
 */
class AdaptiveCSpace : public PiggybackCSpace
{
public:
  AdaptiveCSpace(CSpace* baseSpace);
  virtual bool IsFeasible(const Config& x);
  virtual bool IsFeasible(const Config& x,int obstacle);
  virtual void CheckConstraints(const Config& x,std::vector<bool>& satisfied);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b,int obstacle);
  bool IsFeasible_NoDeps(const Config& x,int obstacle);
  EdgePlanner* PathChecker_NoDeps(const Config& a,const Config& b,int obstacle);
  void SetupAdaptiveInfo();
  bool AddFeasibleDependency(int constraint,int dependency);
  bool AddVisibleDependency(int constraint,int dependency);
  bool AddFeasibleDependency(const char* name,const char* dependency);
  bool AddVisibleDependency(const char* name,const char* dependency);
  void OptimizeQueryOrder();
  void GetFeasibleDependencies(int obstacle,std::vector<int>& deps,bool recursive=true) const;
  void GetVisibleDependencies(int obstacle,std::vector<int>& deps,bool recursive=true) const;
  void GetStats(PropertyMap& stats) const;
  void LoadStats(const PropertyMap& stats);

  struct PredicateStats
  {
    double cost;
    double probability;
    double count;
  };
  bool adaptive;
  std::map<std::string,int> constraintMap;
  std::vector<PredicateStats> feasibleStats,visibleStats;
  std::vector<std::vector<int> > feasibleTestDeps,visibleTestDeps;
  std::vector<int> feasibleTestOrder,visibleTestOrder;
  bool useBaseVisibleTest;
  PredicateStats baseVisibleStats;
};


/** @brief Create a single-obstacle edge checker that uses discretization.
 */
EdgePlanner* MakeSingleConstraintEpsilonChecker(CSpace* space,const Config& a,const Config& b,int obstacle,Real epsilon);

/** @brief Create a single-obstacle edge checker that uses repeated bisection.
 */
EdgePlanner* MakeSingleConstraintBisectionPlanner(CSpace* space,const Config& a,const Config& b,int obstacle,Real epsilon);


#endif
