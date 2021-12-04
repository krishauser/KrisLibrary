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
  GeodesicCSpaceAdaptor(const std::shared_ptr<GeodesicSpace>& geodesic);
  virtual int NumDimensions() override { return geodesic->NumDimensions(); }
  virtual int NumIntrinsicDimensions() override { return geodesic->NumIntrinsicDimensions(); }
  virtual Real Distance(const Config& x, const Config& y) override { return geodesic->Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override { geodesic->Interpolate(x,y,u,out); }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) override { geodesic->Interpolate(x,y,0.5,out); }

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) override { geodesic->InterpolateDeriv(a,b,u,dx); }
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) override { geodesic->InterpolateDerivA(a,b,u,da,dx); }
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) override { geodesic->InterpolateDerivB(a,b,u,db,dx); }
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) override { geodesic->InterpolateDeriv2(a,b,u,ddx); }
  virtual void Integrate(const Config& a,const Vector& da,Config& b) override { geodesic->Integrate(a,da,b); }

  std::shared_ptr<GeodesicSpace> geodesic;
};

/** @ingroup MotionPlanning
 * A standard Cartesian CSpace.  
 * You will still need to overload Sample() and LocalPlanner/PathChecker().
 */
class CartesianCSpace : public GeodesicCSpace
{
public:
  CartesianCSpace(int d);
  virtual int NumDimensions() override { return d; }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) override;
  virtual Real Distance(const Config& x, const Config& y) override { return CSpace::Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override { CSpace::Interpolate(x,y,u,out); }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) override { out.add(x,y); out.inplaceMul(0.5); }

  virtual void Sample(Config& q) override { FatalError("Not implemented"); }

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
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual void Sample(Config& x) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) override;
  virtual bool ProjectFeasible(Config& x) override;
  virtual void Properties(PropertyMap&) override;

  ///The domain.  NOTE: modifing these does not directly affect the constraints! Use SetDomain instead
  Vector bmin,bmax;
};


/** @ingroup MotionPlanning
 * A Cartesian product of several CSpace's.  Inherits geodesic information.
 *
 * Note: although this automatically sets up how the Cartesian product handles
 * geodesic information (sampling, interpolation, distances), *you will need to
 * customize the constraint checking behavior.* 
 *
 * If components interact (typical): 
 * - Call FlattenConstraints to adapt the subspaces' constraints to the joint
 *   C-space.
 * - Call AddConstraint(name,constraint) (no space index) to add constraints 
 *   on configurations in the joint C-space
 * - Override PathChecker (both) as you see fit (e.g., return a
 *   BisectionEpsilonEdgePlanner)
 * - Optionally, override LocalPlanner. By default this just calls PathChecker,
 *   which is typically the intended behavior.
 *
 * If you were really being clever to optimize collision checks, you might implement IsFeasible
 * by calling IsFeasible_Independent for each of the component constraints, and then checking
 * the joint checker for the collective constraints.  The same goes for PathChecker.
 * 
 * If there is no interaction between any components: just add the subspaces,
 * The IsFeasible / LocalPlanner/ PathChecker methods are implemented by calling
 * the corresponding X_Independent function.
 * 
 * In this behavior, each subspace's constraint will be tested individually, and 
 * configurations / paths will be checked by splitting into subspaces and calling 
 * the checkers of individual sub-spaces.  
 */
class MultiCSpace : public GeodesicCSpace
{
public:
  MultiCSpace();
  MultiCSpace(const std::shared_ptr<CSpace>& space1,const std::shared_ptr<CSpace>& space2);
  MultiCSpace(const std::vector<std::shared_ptr<CSpace> >& components);
  void Add(const std::string& name,const std::shared_ptr<CSpace>& space,Real distanceWeight=1);

  ///If this returns true, no additional constraints have been added on the Cartesian product
  ///space, and each subspace can be treated independently.
  inline bool AreSubspacesIndependent() const { return constraints.empty(); }

  ///Rather than testing individual CSpace's, this flattens all the constraints so that 
  ///they're in the constraints list of this object.
  ///
  ///After this is called, do not call componentSpaces[i]->AddConstraint(name,constraint)
  ///directly.  Instead, call this->AddConstraint(i,name,constraint)
  void FlattenConstraints();
  ///If FlattenConstraints has already been called, this lets you add new single-space 
  ///constraints.  The constraint is added to the designated component space AND this object.  
  void AddConstraint(int spaceIndex,const std::string& name,CSet* constraint);
  ///If FlattenConstraints has already been called, this lets you add new single-space 
  ///constraints.  The constraint is added to the designated component space AND this object.  
  void AddConstraint(int spaceIndex,const std::string& name,const std::shared_ptr<CSet>& constraint);
  ///If FlattenConstraints has already been called, this lets you add new single-space 
  ///constraints.  The constraint is added to the designated component space AND this object.  
  void AddConstraint(int spaceIndex,const std::string& name,CSet::CPredicate test);

  void Split(const Vector& x,std::vector<Vector>& items);
  void SplitRef(const Vector& x,std::vector<Vector>& items);
  void Join(const std::vector<Vector>& items,Vector& x);

  ///Available for your use in LocalPlanner.  Can be slightly faster than IsFeasible, because the
  ///extraction of sub-components is done only once.
  bool IsFeasible_Independent(const Config&);
  ///Available for your use in LocalPlanner
  EdgePlannerPtr LocalPlanner_Independent(const Config& a,const Config& b);
  ///Available for your use in PathChecker
  EdgePlannerPtr PathChecker_Independent(const Config& a,const Config& b);
  ///Available for your use in PathChecker
  EdgePlannerPtr PathChecker_Independent(const Config& a,const Config& b,int constraint);

  //CSpace overloads
  virtual int NumDimensions() override;
  virtual int NumIntrinsicDimensions() override;
  virtual std::string VariableName(int i) override;
  virtual int NumConstraints() override;
  virtual std::string ConstraintName(int i) override;
  virtual std::shared_ptr<CSet> Constraint(int i) override;
  virtual void Sample(Config& x) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) override;
  virtual bool IsFeasible(const Config& q) override;
  virtual bool ProjectFeasible(Config& x) override;
  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle) override;

  virtual Optimization::NonlinearProgram* FeasibleNumeric() override;
  virtual Real Distance(const Config& x, const Config& y) override;
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override;
  virtual void Midpoint(const Config& x,const Config& y,Config& out) override;
  virtual Real ObstacleDistance(const Config& a) override;
  virtual void Properties(PropertyMap&) override;

  //GeodesicSpace overloads
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) override;
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) override;
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) override;
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) override;
  virtual void Integrate(const Config& a,const Vector& da,Config& b) override;

  std::vector<std::shared_ptr<CSpace> > components;
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
  virtual int NumDimensions() override;
  virtual std::string VariableName(int i) override;
  virtual int NumConstraints() override;
  virtual std::string ConstraintName(int i) override;
  virtual std::shared_ptr<CSet> Constraint(int i) override;
  virtual void Sample(Config& x) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) override;
  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle) override;
  virtual bool IsFeasible(const Config& x) override;
  virtual bool IsFeasible(const Config&,int constraint) override;
  virtual bool ProjectFeasible(Config& x) override;
  virtual Optimization::NonlinearProgram* FeasibleNumeric() override;
  virtual Real Distance(const Config& x, const Config& y) override;
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override;
  virtual void Midpoint(const Config& x,const Config& y,Config& out) override;
  virtual void Properties(PropertyMap& map) override;

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
  virtual bool IsFeasible(const Config& x) override { return CSpace::IsFeasible(x); }
  virtual bool IsFeasible(const Config& x,int obstacle) override { return CSpace::IsFeasible(x,obstacle); }
  virtual bool ProjectFeasible(Config& x) override { return CSpace::ProjectFeasible(x); }
  virtual Optimization::NonlinearProgram* FeasibleNumeric() override { return CSpace::FeasibleNumeric(); }
  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b) override { return CSpace::LocalPlanner(a,b); }
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle) override;

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
  virtual bool IsFeasible(const Config& x) override;
  virtual bool IsFeasible(const Config& x,int obstacle) override;
  virtual void CheckConstraints(const Config& x,std::vector<bool>& satisfied) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle) override;
  bool IsFeasible_NoDeps(const Config& x,int obstacle);
  EdgePlannerPtr PathChecker_NoDeps(const Config& a,const Config& b,int obstacle);
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
EdgePlannerPtr MakeSingleConstraintEpsilonChecker(CSpace* space,const Config& a,const Config& b,int obstacle,Real epsilon);

/** @brief Create a single-obstacle edge checker that uses repeated bisection.
 */
EdgePlannerPtr MakeSingleConstraintBisectionPlanner(CSpace* space,const Config& a,const Config& b,int obstacle,Real epsilon);


#endif
