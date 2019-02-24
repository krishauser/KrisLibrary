#ifndef ROBOTICS_CSPACE_H
#define ROBOTICS_CSPACE_H


#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math/metric.h>
#include <KrisLibrary/utils/PropertyMap.h>
#include <memory>
#include "CSet.h"

namespace Optimization {
  class NonlinearProgram;
} //namespace Math
using namespace Math;
typedef Vector Config;
class Interpolator;
class EdgePlanner;
class CSpace;
typedef std::shared_ptr<EdgePlanner> EdgePlannerPtr;


/** @ingroup MotionPlanning
 * @brief Motion planning configuration space base class.  The configuration space
 * implements an interpolation space with obstacles.   To implement obstacles,
 * the user will create some CSets (or predicates) and add them via the AddConstraint
 * method.
 *
 * Note: these predicates must return true if the configuration is feasible.  In other
 * words, return false if it hits an obstacle.  The same goes for CSet constraint
 * representations: the cset->Contains test returns true if the configuration is
 * feasible for that constraint.
 *
 * At the very least, the subclass must implement the Sample() and one of the PathChecker()
 * functions. All of the other methods assume Euclidean space.
 *
 * Subclasses can also overload the LocalPlanner function to make fancier local planners
 * that do more than just interpolate straight line paths.
 */
class CSpace 
{
public:
  virtual ~CSpace() {}
  void AddConstraint(const std::string& name,CSet* constraint);
  void AddConstraint(const std::string& name,const std::shared_ptr<CSet>& constraint);
  void AddConstraint(const std::string& name,CSet::CPredicate test);
  void CopyConstraints(const CSpace* space,const std::string& prefix="");
  virtual int NumDimensions();
  virtual std::string VariableName(int i);
  virtual int NumConstraints() { return (int)constraints.size(); }
  virtual std::string ConstraintName(int i) { return constraintNames[i]; }
  virtual std::shared_ptr<CSet> Constraint(int i) { return constraints[i]; }
  virtual void Sample(Config& x)=0;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);
  virtual bool IsFeasible(const Config&,int constraint);
  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int constraint);

  ///optionally overrideable (default uses euclidean space)
  virtual Real Distance(const Config& x, const Config& y) { return Distance_L2(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);

  ///If possible, project x onto a nearby feasible configuration and return true.
  ///If not, return false.
  virtual bool ProjectFeasible(Config& x);

  ///If possible, give the feasible set as a nonlinear program
  virtual Optimization::NonlinearProgram* FeasibleNumeric();

  ///for local planners using obstacle distance
  virtual Real ObstacleDistance(const Config& a);

  /** @brief Returns properties of the space that might be useful for planners.
   *
   * Typical properties may include
   * - name (string): an identifier for the space
   * - cartesian (0 or 1): whether its a euclidean space
   * - submanifold (0 or 1): whether its a submanifold space
   * - canproject (0 or 1): whether ProjectFeasible is implemented
   * - hasobstacledistance (0 or 1): whether ObstacleDistance is implemented
   * - volume (real): volume of the space
   * - diameter (real): maximum distance between any two points in the space
   * - minimum (real array): minimum element of the space
   * - maximum (real array): maximum element of the space
   * - intrinsicDimension (real): for submanifolds, intrinsic dimension of
   *   space
   * - geodesic (0 or 1): whether interpolation is along geodesics
   * - metric (string): the type of metric ("euclidean", "weighted euclidean",
   *   "mahalanobis", "manhattan", "weighted manhattan", "Linf",
   *   "weighted Linf")
   * - metricWeights (real array): the metric weight vector
   * Empty values indicate that the property is unknown.
   *
   * Default implementation returns the properties of a Euclidean space.
   */
  virtual void Properties(PropertyMap&);

  ///Returns a vector indicating which constraints are satisfied
  virtual void CheckConstraints(const Config&,std::vector<bool>& satisfied);

  ///Gets a list of feasible obstacles for the given configuration
  void GetFeasibleNames(const Config& q,std::vector<std::string>& names);
  ///Gets a list of infeasible obstacles for the given configuration
  void GetInfeasibleNames(const Config& q,std::vector<std::string>& names);
  ///Prints out the list of infeasible obstacles for the given configuration
  void PrintInfeasibleNames(const Config& q,std::ostream& out=std::cout,const char* prefix="",const char* suffix="\n");

  std::vector<std::string> constraintNames;
  std::vector<std::shared_ptr<CSet> > constraints;
};

#endif
