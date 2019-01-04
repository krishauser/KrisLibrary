#ifndef OMPL_INTERFACE_H
#define OMPL_INTERFACE_H

#if HAVE_OMPL

#include <KrisLibrary/planning/CSpace.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/Planner.h>
namespace ob = ompl::base;

class OMPLCSpace;
class CSpaceOMPLSpaceInformation;
class CSpaceOMPLStateSpace;
class CSpaceOMPLStateSampler;
class CSpaceOMPLValidityChecker;
class KrisLibraryOMPLPlanner;

/** @brief An adaptor that maps OMPL spaces to Klamp't spaces.
 * NOTE: must call SetSpaceBounds before using.
 */
class OMPLCSpace : public CSpace
{
public:
  OMPLCSpace(const ob::SpaceInformationPtr& si);
  virtual ~OMPLCSpace();
  void SetSpaceBounds(const Config& qmin, const Config& qmax);
  ///helper: sets uniform bounds [qmin,qmax] on each dimension
  void SetSpaceBounds(const double& qmin, const double& qmax);
  ///helper: convert a Config to an OMPL state (value is a new State created by allocState())
  ob::State * ToOMPL(const Config& q);
  ///helper: converts a Config to a previously allocated OMPL state
  void ToOMPL(const Config& q,ob::State * s);
  ///helper: convert an OMPL state to a Config
  Config FromOMPL(const ob::State *);
  ///helper: convert an OMPL state to a Config
  void FromOMPL(const ob::State *,Config& q);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config& x);
  virtual double Distance(const Config& a,const Config& b);
  virtual void Interpolate(const Config& a,const Config& b,Real u,Config& x);
  virtual void Midpoint(const Config& x,const Config& y,Config& out) { Interpolate(x,y,0.5,out); }
  virtual EdgePlannerPtr PathChecker(const Config& x,const Config& y);  
  virtual void Properties(PropertyMap&) const;

  ob::SpaceInformationPtr si_;
  ob::StateSamplerPtr sampler_;
  ob::State *stemp1, *stemp2, *stemp3;
  vector<Real> vtemp;
  Config qMin, qMax;
  int nD;
  double resolution;
};

/* @brief Creates an OMPL planner from a KrisLibrary planner. 
 *
 * Constructor assumes the factory type is completely specified when setting
 * the OMPL planner name.
 */
class KrisLibraryOMPLPlanner : public ob::Planner
{
 public:
  KrisLibraryOMPLPlanner(const ob::SpaceInformationPtr &si,const MotionPlannerFactory& factory);
  virtual void clear();
  virtual void setup();
  virtual ob::PlannerStatus solve (const ob::PlannerTerminationCondition &ptc);
  virtual void getPlannerData (ob::PlannerData &data) const;

  MotionPlannerFactory factory;
  std::shared_ptr<OMPLCSpace> cspace;
  std::shared_ptr<MotionPlannerInterface> planner;
};

/** @brief Adapts a CSpace into a SpaceInformation that can be used with
 * OMPL.
 *
 * The StateSpace points to an instance of CSpaceOMPLStateSpace.
 */
class CSpaceOMPLSpaceInformation : public ob::SpaceInformation
{
 public:
  CSpaceOMPLSpaceInformation(CSpace* space);
  ob::State * ToOMPL(const Config& q) const;
  Config FromOMPL(const ob::State *) const;

  CSpace* cspace;
};


/** @brief Adapts a CSpace into a StateSpace that can be used with
 * OMPL.
 *
 * NOTE: interally used, most users should use CSpaceOMPLSpaceInformation.
 */
class CSpaceOMPLStateSpace : public ob::RealVectorStateSpace
{
 public:
  CSpaceOMPLStateSpace(CSpace* space,CSpaceOMPLSpaceInformation* si);
  virtual unsigned int getDimension (void) const;
  virtual double getMaximumExtent (void) const;
  virtual void enforceBounds (ob::State *state) const;
  virtual bool satisfiesBounds (const ob::State *state) const;
  virtual double distance (const ob::State *state1, const ob::State *state2) const;
  virtual void interpolate (const ob::State *from, const ob::State *to, const double t, ob::State *state) const;
  virtual ob::StateSamplerPtr allocDefaultStateSampler (void) const;

  CSpace* space;
  CSpaceOMPLSpaceInformation* si;
  std::vector<double> minimum,maximum;
};

/** @brief Adapts a CSpace into a state sampler that can be used with
 * OMPL.
 */
class CSpaceOMPLStateSampler : public ob::StateSampler
{
 public:
  CSpaceOMPLStateSampler(const CSpaceOMPLStateSpace* space);
  virtual void sampleUniform (ob::State *state);
  virtual void sampleUniformNear (ob::State *state, const ob::State *near, const double distance); 
  virtual void sampleGaussian (ob::State *state, const ob::State *mean, const double stdDev);

  CSpace* space;
};

/** @brief Adapts a CSpace into a validity checker that can be used with
 * OMPL.
 *
 * NOTE: interally used, most users should use CSpaceOMPLSpaceInformation.
 */
class CSpaceOMPLValidityChecker : public ob::StateValidityChecker
{
 public:
  CSpaceOMPLValidityChecker(CSpaceOMPLSpaceInformation* space);
  virtual bool isValid(const ob::State* state) const;

  CSpace* space;
};

/** @brief Adapts a CSpace into a motion validator that can be used with
 * OMPL.
 *
 * NOTE: interally used, most users should use CSpaceOMPLSpaceInformation.
 */
class CSpaceOMPLMotionValidator : public ob::MotionValidator
{
 public:
  CSpaceOMPLMotionValidator(CSpaceOMPLSpaceInformation* space);
  virtual bool checkMotion (const ob::State *s1, const ob::State *s2) const;
  virtual bool checkMotion (const ob::State *s1, const ob::State *s2, std::pair< ob::State *, double > &lastValid) const;

  CSpace* space;
};


#endif //HAVE_OMPL

#endif
